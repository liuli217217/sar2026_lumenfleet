"""Search and rescue without a prior map or victim positions.

The Go2 knows nothing about the field. It builds an occupancy grid from the lidar
while it walks, explores until the camera has seen every reachable free area, and
confirms, approaches and sits next to every person YOLO finds.

  SPIN      at the start: one turn on the spot, the camera sees the start area
  EXPLORE   next best view: go to the viewpoint whose camera would see the most
            unknown / unseen area and unseen obstacle surface per metre of path
  LOOK      at the viewpoint: turn the camera towards it
  APPROACH  walk to a confirmed victim until the lidar says < `sit_distance`
  SIT       sit down next to the victim (= victim found), then STAND_UP
  DONE      all `expected_victims` found, nothing left to explore, or time over

Localization: IMU heading + CHAMP leg odometry, corrected by matching every scan
(translation-only ICP) against the occupancy grid built so far.

Lidar blind zone: the sensor sees nothing closer than its minimum range (and looks
*through* obstacles there), so remembered obstacles end free-space and camera rays,
the footprint never erases occupied cells, the own body is cut out of the scan, and
the planner only clears inflation (never obstacles) around the robot.

Motion safety (compared with go2_sar_prior.sar_prior):
  1. blocked / stuck detection uses sliding windows, so a robot that alternates
     between "blocked" and "free" every tick still triggers the recovery;
  2. a goal that makes no progress for `progress_timeout` is abandoned and the
     spot in front of the robot gets a high path cost, so A* picks another route;
  3. the planner keeps more clearance (`robot_radius`) than the blind-zone check
     looks at (`blind_half_width`), and narrow gaps cost a lot (`cost_weight`);
  4. trying to turn without the yaw changing (body against a wall end), or blocked /
     stuck -> walk away from the obstacles around the body, sideways if needed (the
     Go2 can strafe); large heading changes are turned on an arc where there is room
     (CHAMP turns on the spot at only ~0.2 rad/s).
"""
import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import CameraInfo, Imu, PointCloud2
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from go2_sar_core.pointcloud_utils import (cloud_to_xyz, corridor_clearance, rotation_rpy,
                                      voxel_downsample_2d, wrap_angle, yaw_from_quaternion)
from go2_sar_slam.live_map import LiveMap, angle_bins, bin_angles

WAIT, SPIN, EXPLORE, LOOK, APPROACH, SIT, STAND_UP, DONE = (
    'WAIT', 'SPIN', 'EXPLORE', 'LOOK', 'APPROACH', 'SIT', 'STAND_UP', 'DONE')
N_BINS = 720


def stamp_of(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class NoMapExplorer(Node):
    def __init__(self):
        super().__init__('sar_slam')
        p = self._declare_parameters()
        self.p = p
        self.map = LiveMap(p['map_size'], p['map_resolution'])

        # ---------------------------------------------------------------- state
        self.x = self.y = self.yaw = 0.0          # world frame = start pose
        self.imu_yaw0 = self.imu_yaw = None
        self.tilt_history = deque(maxlen=300)
        self.pose_history = deque(maxlen=300)
        self.last_odom = None
        self.ground_z = p['ground_z_init']
        self.latest_obstacles = np.zeros((0, 2))   # base frame, last scan
        self.latest_points = np.zeros((0, 3))      # base frame x, y, height
        self.have_cloud = False
        self.lidar_R = rotation_rpy(*p['lidar_rpy'])
        self.lidar_t = np.asarray(p['lidar_xyz'])
        # camera model: from camera_info if given (real D435i), else from hfov + image size (sim)
        self.fx = (p['image_width'] / 2.0) / math.tan(p['camera_hfov'] / 2.0)
        self.cx0, self.cy0 = p['image_width'] / 2.0, p['image_height'] / 2.0
        self.img_h = p['image_height']
        self.cam_hfov = p['camera_hfov']

        self.state = WAIT
        self.state_since = 0.0
        self.mission_start = None
        self.goal = None                 # (x, y) viewpoint
        self.look_yaw = None
        self.last_goal_search = -1e9
        self.path = None
        self.path_goal = None
        self.last_plan = -1e9
        self.progress = None             # (best remaining distance, time) for the current goal
        self.blacklist = []              # (until, x, y, r): viewpoints not to pick again
        self.penalties = []              # (until, x, y, r): spots with a high path cost
        self.motion = deque()            # (t, blocked, forward_intent, x, y)
        self.turns = deque()             # (t, yaw, turning on the spot)
        self.escape = None               # (until, vx, vy): walk away from what the body touches
        self.last_escape = None          # (t, x, y, attempt)
        self.clearance = None            # distance field [m] of the last planner grids
        self.corridor_free = None        # cells far enough from obstacles for straight cuts
        self.tracks = []                 # victim hypotheses, see _on_detections
        self.target = None               # track being approached
        self.last_path_ok = -1e9
        self.sit_until = None
        self.settle_until = -1e9         # no map updates before (after standing up)
        self.no_path = 0                 # viewpoints in a row without a fine path
        self.found = 0
        self.cmd_v = self.cmd_w = 0.0
        self.last_tick = None
        self.icp_ok = self.icp_tries = 0
        self.no_goal = 0
        self.spun = 0.0
        self.rescue_spins = 0
        self.trail = []
        self.last_map_dump = -1e9
        self.trace = None
        if p['trace_file']:
            self.trace = open(p['trace_file'], 'w', buffering=1)
            self.trace.write('t,elapsed,state,x,y,yaw_deg,v,w,goal_x,goal_y,front,blind,path_len,'
                             'tracks,found,seen_m2,free_m2,icp_ok,icp_tries,ground_z,tick_ms\n')

        # ---------------------------------------------------------------- ROS I/O
        self.create_subscription(PointCloud2, p['cloud_topic'], self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, p['imu_topic'], self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, p['odom_topic'], self._on_odom, 10)
        self.create_subscription(Detection2DArray, p['detections_topic'], self._on_detections, 10)
        if p['camera_info_topic']:
            self.create_subscription(CameraInfo, p['camera_info_topic'], self._on_camera_info, 10)
        self.cmd_pub = self.create_publisher(Twist, p['cmd_vel_topic'], 10)
        self.body_pose_pub = self.create_publisher(Pose, p['body_pose_topic'], 10)
        self.detector_pub = self.create_publisher(Bool, p['detector_enable_topic'], 10)
        self.event_pub = self.create_publisher(String, '/sar/events', 10)
        self.status_pub = self.create_publisher(String, '/sar/status', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/sar/pose', 10)
        self.path_pub = self.create_publisher(Path, '/sar/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/sar/markers', 10)
        self.create_timer(1.0 / p['control_rate_hz'], self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(f'no prior map; looking for {p["expected_victims"]} victims, '
                               f'time limit {p["time_limit"]:.0f} s')

    def _declare_parameters(self):
        defaults = {
            'cloud_topic': '/velodyne_points/points',
            'imu_topic': '/imu/data',
            'odom_topic': '/odom',
            'detections_topic': '/sar/person_detections',
            'cmd_vel_topic': '/cmd_vel',
            'body_pose_topic': '/body_pose',
            'detector_enable_topic': '/sar/detector_enable',
            'expected_victims': 4,
            'time_limit': 300.0,
            'start_after_sim_time': 40.0,           # CHAMP controllers come up at ~30 s
            'control_rate_hz': 10.0,
            'debug': False,
            'debug_map_dir': '',                    # if set: map snapshots (png) every 20 s
            'trace_file': '',                       # if set: one CSV row per control tick
            # Sensors (Go2 URDF).
            'lidar_xyz': [0.2, 0.0, 0.1177],
            'lidar_rpy': [0.0, 0.0, 0.0],
            'body_box': [-0.5, 0.35, 0.25],         # [x min, x max, half width] own body in the scan
            'lidar_blind_range': 0.8,               # [m from the body centre] lidar minimum range + margin
            'footprint_radius': 0.2,                # the robot stands here: free (unless mapped occupied)
            'camera_xyz': [0.326, 0.0, 0.043],
            'camera_hfov': 1.367,
            'camera_info_topic': '',                # real camera: intrinsics from here
            'camera_range': 3.5,                    # coverage: YOLO finds a person up to here
            'image_width': 640,
            'image_height': 480,
            # Lidar processing.
            'ground_z_init': -0.21,
            'ground_z_min': -0.32,
            'ground_z_max': -0.10,
            'obstacle_min_height': 0.07,
            'tilt_tolerance': 3.0,
            'obstacle_max_height': 0.75,
            'max_range': 5.0,
            'free_range': 4.5,
            'near_guard': 1.2,                      # remembered obstacles end lidar rays this close
            # Map.
            'map_size': 20.0,
            'map_resolution': 0.05,
            # Localization: scan to own map.
            'icp_gate': 0.25,
            'icp_min_inliers': 0.5,
            'icp_max_correction': 0.15,
            'icp_gain': 0.7,
            # Planning (fix 3: clearance > blind-zone check, narrow gaps are expensive).
            'robot_radius': 0.25,                   # hard inflation (Go2 half width 0.155; mapped walls are noisy)
            'body_clearance': 0.15,                 # never plan closer to an obstacle than this
            'follow_clearance': 0.20,               # the follower does not cut corners closer than this
            'cost_distance': 0.65,
            'cost_weight': 6.0,
            'unknown_cost': 3.0,
            'replan_period': 1.0,
            # Exploration.
            'goal_period': 2.0,
            'candidate_spacing': 0.3,               # [m] viewpoint candidates grid
            'min_gain': 0.5,                        # [m^2] visible unknown / unseen area
            'goal_dist_offset': 2.0,                # score = gain / (path + offset)
            'goal_cost_scale': 0.3,                 # share of the planner cost in the viewpoint path length
            'spin_time': 16.0,                      # SPIN: at most this long
            'goal_reached': 0.35,
            'goal_switch_ratio': 1.5,               # a new viewpoint must score this much better
            'goal_finish_distance': 1.5,            # [m] closer than this: finish the current viewpoint
            'obstacle_surface_gain': 1.0,           # [m2 per m] obstacle surface the camera has not seen
            'look_clearance': 0.45,                 # viewpoints where the body can turn on the spot
            'blacklist_time': 40.0,
            # Motion.
            'lookahead': 0.5,
            'max_linear_speed': 0.3,
            'min_forward_speed': 0.22,              # CHAMP walks on the spot below ~0.15 m/s
            'max_angular_speed': 0.6,
            'heading_gain': 1.5,
            'arc_clearance': 0.6,                   # turn while walking if this free around the body
            'turn_stall_time': 2.5,                 # turning on the spot without yaw change -> escape
            'turn_stall_angle': 0.15,
            'escape_speed': 0.2,                    # [m/s] escape walk (forwards / sideways)
            'escape_time': 1.5,
            'robot_half_width': 0.18,
            'blind_half_width': 0.15,
            'stop_distance': 0.40,
            # Fix 1 / 2: recovery.
            'blocked_window': 3.0,
            'blocked_fraction': 0.4,
            'stuck_window': 5.0,
            'stuck_distance': 0.10,
            'progress_timeout': 10.0,
            'penalty_cost': 10.0,
            'penalty_time': 60.0,
            # Victims.
            'detection_min_score': 0.4,
            'min_person_extent': 0.35,              # [m] box size x range: smaller -> range is an occluder
            'victim_max_bottom': 0.45,              # [m above the floor] box bottom edge (on the floor)
            'victim_max_top': 1.3,                  # [m] box top edge (a sitting person is ~0.9 m)
            'association_radius': 1.0,
            'confirm_hits': 3,
            'confirm_min_score': 0.5,
            'found_suppress_radius': 2.0,           # a lying body is 1.8 m long
            'victim_cluster_radius': 0.6,
            'sit_distance': 0.7,
            'approach_slow_range': 1.5,             # full speed until this close to the victim
            'approach_timeout': 40.0,
            'approach_fallback_radius': 1.2,        # unreachable victim cell: closest reachable one
            'approach_no_path': 3.0,                # [s] without a path to the victim -> give up
            'retry_failed_after': 45.0,             # [s] a given-up track may be confirmed again
            'sit_duration': 3.0,
            'stand_up_duration': 1.5,
            'sit_height_offset': -0.08,
            'sit_pitch': -0.3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        return {name: self.get_parameter(name).value for name in defaults}

    # ================================================================= sensors
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_imu(self, msg):
        q = msg.orientation
        yaw = yaw_from_quaternion(q)
        if self.imu_yaw0 is None:
            self.imu_yaw0 = yaw
        self.imu_yaw = wrap_angle(yaw - self.imu_yaw0)
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        self.tilt_history.append((stamp_of(msg.header), roll, pitch))

    def _on_odom(self, msg):
        pos = msg.pose.pose.position
        if self.last_odom is not None and self.state != WAIT:
            self.x += pos.x - self.last_odom[0]
            self.y += pos.y - self.last_odom[1]
        self.last_odom = (pos.x, pos.y)

    def _tilt_at(self, stamp):
        if not self.tilt_history:
            return 0.0, 0.0
        times = np.fromiter((h[0] for h in self.tilt_history), float, len(self.tilt_history))
        return self.tilt_history[int(np.abs(times - stamp).argmin())][1:]

    def _to_world(self, pts):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.column_stack([self.x + c * pts[:, 0] - s * pts[:, 1],
                                self.y + s * pts[:, 0] + c * pts[:, 1]])

    def _on_cloud(self, msg):
        if self.imu_yaw is None:
            return
        pts = cloud_to_xyz(msg)
        if len(pts) == 0:
            return
        p = self.p
        pts = pts @ self.lidar_R.T + self.lidar_t
        roll, pitch = self._tilt_at(stamp_of(msg.header))
        pts = pts @ rotation_rpy(roll, pitch, 0.0).T             # gravity-levelled base frame
        r = np.hypot(pts[:, 0], pts[:, 1])
        ground_sel = (r > 0.8) & (r < 4.0) & (np.abs(pts[:, 2] - self.ground_z) < 0.10)
        if ground_sel.sum() > 30:
            self.ground_z = 0.9 * self.ground_z + 0.1 * float(np.percentile(pts[ground_sel, 2], 20))
            self.ground_z = min(max(self.ground_z, p['ground_z_min']), p['ground_z_max'])
        height = pts[:, 2] - self.ground_z
        in_range = r < p['max_range']
        min_h = p['obstacle_min_height'] + r * math.tan(math.radians(p['tilt_tolerance']))
        # own rear legs / body: ~0.5 m behind the lidar, right at its minimum range
        bx0, bx1, bhw = p['body_box']
        body = (pts[:, 0] > bx0) & (pts[:, 0] < bx1) & (np.abs(pts[:, 1]) < bhw)
        obstacle = in_range & ~body & (height > min_h) & (height < p['obstacle_max_height'])
        self.latest_obstacles = voxel_downsample_2d(pts[obstacle, :2], 0.05)
        keep = in_range & ~body & (height > 0.03) & (height < 1.2)
        self.latest_points = np.column_stack([pts[keep, :2], height[keep]])
        ground = pts[in_range & (np.abs(height) < 0.05), :2]
        self.have_cloud = True
        if self.state in (SIT, STAND_UP) or self._now() < self.settle_until:
            return              # lowered, pitched body: the ground looks like obstacles
        if self.state != WAIT:
            self._match_map(self.latest_obstacles)
        self._update_map(self.latest_obstacles, ground)

    def _match_map(self, scan):
        """Translation-only ICP of the scan against the map built so far."""
        p = self.p
        if len(scan) < 30:
            return
        src = voxel_downsample_2d(scan, 0.1)
        if len(src) > 300:
            src = src[np.random.default_rng().choice(len(src), 300, replace=False)]
        src = self._to_world(src)
        lo, hi = src.min(axis=0) - 0.5, src.max(axis=0) + 0.5
        r0, c0 = self.map.to_cell(*lo)
        r1, c1 = self.map.to_cell(*hi)
        r0, c0 = max(r0, 0), max(c0, 0)
        sub = self.map.logodds[r0:r1 + 1, c0:c1 + 1] > 2.0          # confident walls only
        rows, cols = np.nonzero(sub)
        if len(rows) < 100:
            return
        ref = self.map.cell_centres(rows + r0, cols + c0)
        tree = cKDTree(ref)
        self.icp_tries += 1
        total = np.zeros(2)
        moved = src.copy()
        ratio = 0.0
        for i in range(10):
            d, idx = tree.query(moved)
            gate = max(p['icp_gate'] * (0.7 ** i), 0.08)
            inl = d < gate
            ratio = inl.mean()
            if inl.sum() < 30:
                return
            step = (ref[idx[inl]] - moved[inl]).mean(axis=0)
            moved += step
            total += step
            if np.hypot(*step) < 1e-3:
                break
        if ratio >= p['icp_min_inliers'] and np.hypot(*total) <= p['icp_max_correction']:
            self.x += p['icp_gain'] * total[0]
            self.y += p['icp_gain'] * total[1]
            self.icp_ok += 1

    def _bin_ranges(self, obstacles, ground):
        """Per direction: range of the nearest obstacle (inf if none) and of the
        farthest ground return (0 if none), both from the body centre."""
        near = np.full(N_BINS, np.inf)
        if len(obstacles):
            np.minimum.at(near, angle_bins(obstacles, N_BINS), np.hypot(obstacles[:, 0], obstacles[:, 1]))
        far = np.zeros(N_BINS)
        if len(ground):
            np.maximum.at(far, angle_bins(ground, N_BINS), np.hypot(ground[:, 0], ground[:, 1]))
        return near, far

    def _update_map(self, obstacles, ground):
        p = self.p
        near, far = self._bin_ranges(obstacles, ground)
        self.bins_near = near
        # free space: along every direction up to just before the first obstacle
        # (or the farthest ground return), never inside the lidar blind zone
        ang = bin_angles(N_BINS)
        reach = np.where(np.isfinite(near), near - 0.1, far)
        # close to the body the lidar looks *through* obstacles (minimum range), so
        # a remembered obstacle there ends the ray as well
        reach = np.minimum(reach, self._remembered_range(ang, p['near_guard']) - 0.1)
        reach = np.minimum(reach, p['free_range'])
        steps = np.arange(p['lidar_blind_range'], p['free_range'], self.map.resolution)
        mask = steps[None, :] < reach[:, None]
        rr = np.broadcast_to(steps[None, :], mask.shape)[mask]
        aa = np.broadcast_to(ang[:, None], mask.shape)[mask]
        free_base = np.column_stack([rr * np.cos(aa), rr * np.sin(aa)])
        # the footprint is free (the robot stands there); the rest of the blind zone is
        # left alone: the lidar does not see obstacles there (minimum range), so free
        # evidence would erase walls the robot remembers
        k = int(p['footprint_radius'] / self.map.resolution)
        g = np.arange(-k, k + 1) * self.map.resolution
        gx, gy = np.meshgrid(g, g)
        disc = np.hypot(gx, gy) <= p['footprint_radius']
        footprint = np.column_stack([gx[disc], gy[disc]])
        hits = obstacles[np.hypot(obstacles[:, 0], obstacles[:, 1]) < p['max_range']] if len(obstacles) else obstacles
        self.map.update(self._to_world(hits) if len(hits) else hits, self._to_world(free_base),
                        self._to_world(footprint))
        # camera coverage: what the camera sees up to the first obstacle
        half = self.cam_hfov / 2.0
        in_fov = np.abs(ang) < half
        cam_reach = np.where(np.isfinite(near), near + 0.05, np.where(far > 0, far, 0.0))
        cam_reach = np.minimum(cam_reach, p['camera_range'])
        # the camera does not see behind remembered obstacles either (the lidar may
        # have looked through a box right in front of the nose)
        cam_reach[in_fov] = np.minimum(cam_reach[in_fov],
                                       self._remembered_range(ang[in_fov], p['camera_range']) + 0.05)
        csteps = np.arange(0.3, p['camera_range'], self.map.resolution)
        cmask = (csteps[None, :] < cam_reach[:, None]) & in_fov[:, None]
        rr = np.broadcast_to(csteps[None, :], cmask.shape)[cmask]
        aa = np.broadcast_to(ang[:, None], cmask.shape)[cmask]
        if len(rr):
            self.map.mark_seen(self._to_world(np.column_stack([rr * np.cos(aa), rr * np.sin(aa)])))

    def _remembered_range(self, ang, max_range):
        """Per direction (base frame angles): distance from the body centre to the
        first occupied map cell, inf if none within max_range."""
        steps = np.arange(0.3, max_range, self.map.resolution)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        a = ang[:, None] + 0.0 * steps[None, :]
        bx, by = steps[None, :] * np.cos(a), steps[None, :] * np.sin(a)
        wx, wy = self.x + c * bx - s * by, self.y + s * bx + c * by
        cols = np.floor((wx - self.map.origin[0]) / self.map.resolution).astype(np.int64)
        rows = np.floor((wy - self.map.origin[1]) / self.map.resolution).astype(np.int64)
        ok = (rows >= 0) & (rows < self.map.height) & (cols >= 0) & (cols < self.map.width)
        occ = np.zeros(rows.shape, bool)
        occ[ok] = self.map.occupied[rows[ok], cols[ok]]
        first = occ.argmax(axis=1)
        return np.where(occ.any(axis=1), steps[first], np.inf)

    # ============================================================== detections
    def _on_detections(self, msg):
        if self.state in (WAIT, DONE, SIT, STAND_UP):
            return
        p = self.p
        now = self._now()
        px, py, pyaw = self._pose_at(stamp_of(msg.header))
        for det in msg.detections:
            if not det.results or det.results[0].hypothesis.score < p['detection_min_score']:
                continue
            score = det.results[0].hypothesis.score
            fix = self._locate(det, px, py, pyaw)
            if fix is None or not self._line_of_sight(px, py, pyaw, fix):
                continue
            if any(math.hypot(fix[0] - t['x'], fix[1] - t['y']) < p['found_suppress_radius']
                   for t in self.tracks if t['status'] == 'found'):
                continue
            track = min((t for t in self.tracks if t['status'] in ('candidate', 'approach', 'failed')),
                        key=lambda t: math.hypot(fix[0] - t['x'], fix[1] - t['y']), default=None)
            if track is None or math.hypot(fix[0] - track['x'], fix[1] - track['y']) > p['association_radius']:
                track = {'id': len(self.tracks) + 1, 'hits': [], 'status': 'candidate', 'x': fix[0], 'y': fix[1]}
                self.tracks.append(track)
            if track['status'] == 'failed':
                if now - track['failed_at'] < p['retry_failed_after']:
                    continue
                track['status'], track['hits'] = 'candidate', []     # needs a fresh confirmation
            track['hits'].append((fix[0], fix[1], score))
            recent = track['hits'][-10:]
            track['x'] = float(np.median([h[0] for h in recent]))
            track['y'] = float(np.median([h[1] for h in recent]))
            if self.p['debug']:
                self.get_logger().info(f'  track {track["id"]} hit {len(track["hits"])}: score {score:.2f} '
                                       f'at ({fix[0]:.2f}, {fix[1]:.2f}), range {fix[2]:.2f} m, '
                                       f'box {det.bbox.size_x:.0f}x{det.bbox.size_y:.0f}')

    def _on_camera_info(self, msg):
        if msg.k[0] > 0:
            self.fx, self.cx0, self.cy0 = msg.k[0], msg.k[2], msg.k[5]
            self.img_h = msg.height
            self.cam_hfov = 2.0 * math.atan(msg.width / 2.0 / msg.k[0])

    def _pose_at(self, stamp):
        if not self.pose_history:
            return self.x, self.y, self.yaw
        times = np.fromiter((h[0] for h in self.pose_history), float, len(self.pose_history))
        return self.pose_history[int(np.abs(times - stamp).argmin())][1:]

    def _locate(self, det, px, py, pyaw):
        """Camera bearing + lidar range -> person position (world)."""
        p = self.p
        cx0, cy0 = self.cx0, self.cy0
        u, half_w = det.bbox.center.position.x, det.bbox.size_x / 2.0
        bearing = -math.atan((u - cx0) / self.fx)
        lo = -math.atan((u + 0.5 * half_w - cx0) / self.fx)
        hi = -math.atan((u - 0.5 * half_w - cx0) / self.fx)
        cam_x, cam_y, cam_z = p['camera_xyz']
        rng = None
        pts = self.latest_points
        if len(pts):
            pts = pts[pts[:, 2] < 1.0]
            rx, ry = pts[:, 0] - cam_x, pts[:, 1] - cam_y
            ang = np.arctan2(ry, rx)
            sel = (rx > 0.2) & (ang >= lo - 0.03) & (ang <= hi + 0.03)
            if sel.sum() >= 3:
                rng = float(np.percentile(np.hypot(rx[sel], ry[sel]), 20))
        if rng is None:
            v_bottom = det.bbox.center.position.y + det.bbox.size_y / 2.0
            alpha = math.atan((v_bottom - cy0) / self.fx)
            if alpha > 0.03:
                rng = (cam_z - self.ground_z) / math.tan(alpha)
        if rng is None or rng > 6.0 or rng < 0.5:     # too close: the box cannot be a whole person
            return None
        # a far person behind a near door edge gets the edge's range: the box is then
        # far too small for a person at that distance
        if max(det.bbox.size_x, det.bbox.size_y) * rng / self.fx < p['min_person_extent']:
            return None
        # victims lie or sit on the floor: the box must reach down to the floor and not
        # far above a sitting person. Spectators outside the arena show only their heads
        # and shoulders above the 1.2 m walls (box bottom = wall top, range = the wall).
        cam_h = cam_z - self.ground_z
        v_top = det.bbox.center.position.y - det.bbox.size_y / 2.0
        v_bot = det.bbox.center.position.y + det.bbox.size_y / 2.0
        bottom_h = cam_h - (v_bot - cy0) * rng / self.fx
        top_h = cam_h + (cy0 - v_top) * rng / self.fx
        if v_bot < self.img_h - 3 and bottom_h > p['victim_max_bottom']:
            return None
        if v_top > 3 and top_h > p['victim_max_top']:
            return None
        bx, by = cam_x + rng * math.cos(bearing), cam_y + rng * math.sin(bearing)
        c, s = math.cos(pyaw), math.sin(pyaw)
        # a victim already found farther away in the same direction: that is who we see
        cxw, cyw = px + c * cam_x - s * cam_y, py + s * cam_x + c * cam_y
        box_half = abs(math.atan((u + half_w - cx0) / self.fx) - math.atan((u - half_w - cx0) / self.fx)) / 2
        for t in self.tracks:
            if t['status'] == 'found' and math.hypot(t['x'] - cxw, t['y'] - cyw) > rng + 0.5:
                off = wrap_angle(math.atan2(t['y'] - cyw, t['x'] - cxw) - (pyaw + bearing))
                if abs(off) < box_half + 0.1:
                    return None
        return px + c * bx - s * by, py + s * bx + c * by, rng

    def _line_of_sight(self, px, py, pyaw, fix):
        """False if a remembered wall lies between the camera and the person: with the
        nose closer to a wall than the sensors' minimum range, camera and lidar both
        see through it."""
        cx, cy, _ = self.p['camera_xyz']
        c, s = math.cos(pyaw), math.sin(pyaw)
        ax, ay = px + c * cx - s * cy, py + s * cx + c * cy
        length = math.hypot(fix[0] - ax, fix[1] - ay)
        n = int((length - 0.4) / self.map.resolution)     # the body itself is occupied
        if n <= 0:
            return True
        f = np.arange(n) * self.map.resolution / length
        rows, cols = self.map.cells(np.column_stack([ax + f * (fix[0] - ax), ay + f * (fix[1] - ay)]))
        return not (self.map.logodds[rows, cols] > 2.0).any()

    def _confirmed(self, t):
        hits = t['hits']
        near_found = any(math.hypot(t['x'] - f['x'], t['y'] - f['y']) < self.p['found_suppress_radius']
                         for f in self.tracks if f['status'] == 'found')
        return (t['status'] == 'candidate' and not near_found and len(hits) >= self.p['confirm_hits']
                and np.median([h[2] for h in hits]) >= self.p['confirm_min_score'])

    def _victim_distance(self, vx, vy):
        """Robot centre -> nearest point of the victim near (vx, vy): from the current
        scan and from the occupied map cells (the lidar is blind close to the body)."""
        rad = self.p['victim_cluster_radius']
        best = []
        pts = self.latest_points
        if len(pts):
            world = self._to_world(pts[:, :2])
            near = np.hypot(world[:, 0] - vx, world[:, 1] - vy) < rad
            if near.sum() >= 3:
                best.append(float(np.min(np.hypot(pts[near, 0], pts[near, 1]))))
        r = int(math.ceil(rad / self.map.resolution))
        row0, col0 = self.map.to_cell(vx, vy)
        ra, ca = max(row0 - r, 0), max(col0 - r, 0)
        rows, cols = np.nonzero(self.map.occupied[ra:row0 + r + 1, ca:col0 + r + 1])
        if len(rows):
            xy = self.map.cell_centres(rows + ra, cols + ca)
            near = np.hypot(xy[:, 0] - vx, xy[:, 1] - vy) < rad
            if near.any():
                best.append(float(np.min(np.hypot(xy[near, 0] - self.x, xy[near, 1] - self.y)))
                            - self.map.resolution / 2)
        return min(best) if best else None

    # ================================================================ planning
    def _grids(self, now):
        """(blocked, cost) for the planner."""
        p = self.p
        occ = self.map.occupied
        blocked = self.map.inflate(occ, p['robot_radius'])
        dist = self.map.distance_field(occ)
        self.clearance = dist
        span = p['cost_distance'] - p['robot_radius']
        cost = p['cost_weight'] * np.clip((p['cost_distance'] - dist) / span, 0.0, 1.0)
        cost += p['unknown_cost'] * self.map.unknown
        self.penalties = [q for q in self.penalties if q[0] > now]
        # the robot may stand inside the inflation zone: open the zone around it, but
        # never the obstacles themselves (plus what the body needs where possible)
        row, col = self.map.to_cell(self.x, self.y)
        here = float(dist[row, col]) if self.map.inside(row, col) else 1.0
        core = self.map.inflate(occ, max(min(p['body_clearance'], here - 0.05), 0.0))
        near = np.zeros(blocked.shape, np.uint8)
        cv2.circle(near, (int(col), int(row)), int(0.4 / self.map.resolution), 1, -1)
        blocked = np.where(near.astype(bool), core, blocked)
        # straight segments the path follower may cut across
        self.corridor_free = dist > p['follow_clearance']
        cost = cost.astype(np.float32)
        for _, x, y, rad in self.penalties:
            row, col = self.map.to_cell(x, y)
            if self.map.inside(row, col):
                cv2.circle(cost, (int(col), int(row)), int(rad / self.map.resolution), float(p['penalty_cost']), -1)
        return blocked, cost

    def _penalize_ahead(self, now, reason):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        px, py = self.x + 0.45 * c, self.y + 0.45 * s
        self.penalties.append((now + self.p['penalty_time'], px, py, 0.35))
        self.get_logger().info(f'recovery ({reason}): high cost at ({px:.2f}, {py:.2f}), re-planning')
        self.last_plan = -1e9
        self.motion.clear()
        self.progress = None

    def _choose_goal(self, now, through_unknown=False):
        """Next best view: the reachable viewpoint whose camera could see the most
        unknown or not-yet-seen area (ray cast on the map, all around, since LOOK
        turns there) per metre of path. through_unknown: also search through unknown
        cells (fallback). Returns ((x, y), look yaw, gain m2, path m) or None."""
        p = self.p
        blocked, cost = self._grids(now)
        window = self.map.known_bounds()
        # the lidar blind zone around the robot is unknown, not unexplored: no gain
        # there, and the search may cross it
        row, col = self.map.to_cell(self.x, self.y)
        near = np.zeros(blocked.shape, np.uint8)
        cv2.circle(near, (int(col), int(row)), int((p['lidar_blind_range'] + 0.1) / self.map.resolution), 1, -1)
        unknown = self.map.unknown & ~near.astype(bool)
        # viewpoints are searched through known free space only (unknown cells leak
        # through walls the lidar has not seen yet); A* may still cross unknown cells
        dist, r0, c0, step = self.map.geodesic(blocked if through_unknown else blocked | unknown,
                                               cost * p['goal_cost_scale'], (self.x, self.y), window)
        # thin slivers of unseen cells (wall edges, map noise) are not worth a detour
        unseen = cv2.morphologyEx((self.map.free & ~self.map.seen).astype(np.uint8), cv2.MORPH_OPEN,
                                  np.ones((3, 3), np.uint8)).astype(bool)
        interest = unseen | unknown
        # candidates: reachable coarse cells on a sparser grid, plus the current goal
        h, w = dist.shape
        sub = max(int(round(p['candidate_spacing'] / (self.map.resolution * step))), 1)
        reach = np.zeros_like(dist, bool)
        reach[::sub, ::sub] = np.isfinite(dist[::sub, ::sub])
        ck = None
        if self.goal is not None:
            cr, cc = self.map.to_cell(*self.goal)
            ck = ((cr - r0) // step, (cc - c0) // step)
            if 0 <= ck[0] < h and 0 <= ck[1] < w and np.isfinite(dist[ck]):
                reach[ck] = True
        rows, cols = np.nonzero(reach)
        if len(rows) == 0:
            return None
        xy = self.map.cell_centres(rows * step + r0, cols * step + c0)
        gain, look = self._view_gain(xy, interest)
        d = dist[rows, cols]
        raw = gain / (d + p['goal_dist_offset'])
        score = np.where(gain >= p['min_gain'], raw, -np.inf)
        # prefer viewpoints where the body can turn on the spot (LOOK)
        room = self.clearance[rows * step + r0, cols * step + c0] >= p['look_clearance']
        if np.isfinite(score[room]).any():
            score[~room] = -np.inf
        self.blacklist = [q for q in self.blacklist if q[0] > now]
        for _, bx, by, br in self.blacklist:
            score[np.hypot(xy[:, 0] - bx, xy[:, 1] - by) < br] = -np.inf
        k = int(np.argmax(score))
        best = score[k]
        # commitment: every switch costs a slow turn, so keep the current viewpoint
        # while it still has something to see, unless another one is clearly better
        # (judged on the raw score: the turn-room filter flickers)
        if ck is not None:
            cur = np.flatnonzero((rows == ck[0]) & (cols == ck[1]))
            if len(cur):
                c = int(cur[0])
                almost_there = d[c] < p['goal_finish_distance']
                if gain[c] >= 0.5 * p['min_gain'] and (almost_there or not best >= p['goal_switch_ratio'] * raw[c]):
                    return self.goal, float(look[c]), float(gain[c]), float(d[c])
        if not np.isfinite(best):
            return None
        return (float(xy[k, 0]), float(xy[k, 1])), float(look[k]), float(gain[k]), float(d[k])

    def _view_gain(self, xy, interest):
        """For every viewpoint (N, 2): area [m2] of interesting cells visible within
        camera range (rays stop at mapped obstacles), plus obstacle surface the camera
        has not looked at yet (to the lidar a person is just an obstacle), and the yaw
        of the camera window that sees the most of it."""
        p = self.p
        n_rays = 72
        ang = np.arange(n_rays) * (2 * math.pi / n_rays)
        dr = self.map.resolution * 0.7
        rr = np.arange(0.2, p['camera_range'], dr)
        res = self.map.resolution
        # a ray must not slip through a one-cell wall between two samples
        occ = self.map.occupied
        wall = cv2.dilate(occ.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        # rays stop one cell before an obstacle (dilation): look one cell further
        unseen_surface = cv2.dilate((occ & ~self.map.seen).astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        gain = np.zeros(len(xy))
        look = np.zeros(len(xy))
        half = int(round(self.cam_hfov / 2 / (2 * math.pi / n_rays)))
        for i0 in range(0, len(xy), 64):            # chunks bound the memory use
            q = xy[i0:i0 + 64]
            px = q[:, 0, None, None] + rr[None, None, :] * np.cos(ang)[None, :, None]
            py = q[:, 1, None, None] + rr[None, None, :] * np.sin(ang)[None, :, None]
            cols = np.floor((px - self.map.origin[0]) / res).astype(np.int64)
            rows = np.floor((py - self.map.origin[1]) / res).astype(np.int64)
            ok = (rows >= 0) & (rows < self.map.height) & (cols >= 0) & (cols < self.map.width)
            rows, cols = np.where(ok, rows, 0), np.where(ok, cols, 0)
            open_ = ok & ~wall[rows, cols]
            visible = np.cumprod(open_, axis=2).astype(bool)
            area = (visible & interest[rows, cols]) * (rr * dr * (2 * math.pi / n_rays))[None, None, :]
            per_ray = area.sum(axis=2)                                       # (n, rays)
            # where each ray ends: an obstacle surface not seen by the camera yet?
            stop = visible.sum(axis=2)                                       # first blocked sample
            hit = stop < len(rr)
            si = np.minimum(stop, len(rr) - 1)
            hr = np.take_along_axis(rows, si[..., None], axis=2)[..., 0]
            hc = np.take_along_axis(cols, si[..., None], axis=2)[..., 0]
            surface = hit & unseen_surface[hr, hc]
            per_ray += surface * rr[si] * (2 * math.pi / n_rays) * p['obstacle_surface_gain']
            gain[i0:i0 + 64] = per_ray.sum(axis=1)
            # camera window (hfov) with the most to see
            win = sum(np.roll(per_ray, s, axis=1) for s in range(-half, half + 1))
            look[i0:i0 + 64] = ang[win.argmax(axis=1)]
        return gain, look

    def _replan(self, now, gx, gy):
        blocked, cost = self._grids(now)
        if self.state == APPROACH:      # the victim itself is an obstacle, often against a wall
            goal, fallback = (gx, gy), self.p['approach_fallback_radius']
        else:
            goal, fallback = self.map.nearest_free(blocked, gx, gy) or (gx, gy), 0.0
        path, _ = self.map.plan(blocked, cost, (self.x, self.y), goal, fallback_radius=fallback)
        self.path, self.path_goal, self.last_plan = path, goal, now
        self._publish_path()

    def _follow(self, now, gx, gy, v_max=None):
        """Drive along the A* path to (gx, gy). Returns (v, w, remaining distance)."""
        p = self.p
        if self.path is None or now - self.last_plan > p['replan_period']:
            self._replan(now, gx, gy)
        if not self.path:
            return 0.0, 0.0, math.inf
        pts = np.asarray(self.path)
        d = np.hypot(pts[:, 0] - self.x, pts[:, 1] - self.y)
        i = int(d.argmin())
        j = i
        while j < len(pts) - 1 and d[j] < p['lookahead']:
            j += 1
        # pure pursuit cuts corners: aim at the farthest point the body reaches in a
        # straight line with enough clearance (door edges, wall ends)
        while j > i + 1 and not self._straight_clear(*pts[j]):
            j -= 1
        remaining = float(np.sum(np.hypot(np.diff(pts[i:, 0]), np.diff(pts[i:, 1])))) + d[i]
        # fix 2: no progress towards this goal for a while -> give the spot ahead a high cost
        if self.progress is None or remaining < self.progress[0] - 0.25:
            self.progress = (remaining, now)
        elif now - self.progress[1] > p['progress_timeout']:
            self._penalize_ahead(now, 'no progress')
            return 0.0, 0.0, -1.0           # caller abandons the goal
        v, w = self._drive_to(now, *pts[j], v_max or p['max_linear_speed'])
        return v, w, remaining

    def _straight_clear(self, tx, ty):
        length = math.hypot(tx - self.x, ty - self.y)
        if length < 0.2 or self.corridor_free is None:
            return True
        f = np.arange(0.15, length, self.map.resolution) / length
        rows, cols = self.map.cells(np.column_stack([self.x + f * (tx - self.x), self.y + f * (ty - self.y)]))
        return bool(self.corridor_free[rows, cols].all())

    def _drive_to(self, now, tx, ty, v_max):
        p = self.p
        err = wrap_angle(math.atan2(ty - self.y, tx - self.x) - self.yaw)
        w = max(-p['max_angular_speed'], min(p['max_angular_speed'], p['heading_gain'] * err))
        front = corridor_clearance(self.latest_obstacles, np.array([0.0]), p['robot_half_width'],
                                   p['max_range'])[0]
        if abs(err) > 0.6:
            # CHAMP turns on the spot at ~0.2 rad/s but at ~0.4 rad/s while walking:
            # turn on an arc where there is room for it
            row, col = self.map.to_cell(self.x, self.y)
            room = self.clearance is not None and self.map.inside(row, col) and \
                self.clearance[row, col] >= p['arc_clearance']
            if room and front > 2 * p['stop_distance'] and not self._blind_zone_blocked():
                self._record_motion(now, False, False)
                return p['min_forward_speed'], w
            self._record_motion(now, False, False)
            return 0.0, w
        blocked = front < p['stop_distance'] or self._blind_zone_blocked()
        self._record_motion(now, blocked, True)
        if self._recovery_needed(now):
            self._start_escape(now, 'blocked / stuck')
            return 0.0, 0.0
        if blocked:
            self.last_plan = -1e9
            return 0.0, w
        return max(v_max * math.cos(err), p['min_forward_speed']), w

    def _blind_zone_blocked(self):
        """Remembered obstacles right in front (the lidar is blind close to the body)."""
        occ = self.map.logodds
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hw = self.p['blind_half_width']
        for ahead in (0.35, 0.45, 0.55):
            for side in (-hw, 0.0, hw):
                row, col = self.map.to_cell(self.x + c * ahead - s * side, self.y + s * ahead + c * side)
                if self.map.inside(row, col) and occ[row, col] > 1.0:
                    return True
        return False

    def _check_turn(self, now, v, w):
        """Trying to turn while the yaw does not change: the body is touching something
        (the lidar cannot see that close) -> escape."""
        p = self.p
        if self.escape is not None:
            return v, w
        self.turns.append((now, self.yaw, abs(w) > 0.25))
        while self.turns and now - self.turns[0][0] > p['turn_stall_time']:
            self.turns.popleft()
        if now - self.turns[0][0] < p['turn_stall_time'] - 0.2 or \
                np.mean([t[2] for t in self.turns]) < 0.8:
            return v, w
        yaws = np.unwrap([t[1] for t in self.turns])
        if np.ptp(yaws) > p['turn_stall_angle']:
            return v, w
        self._start_escape(now, f'turn stuck at {math.degrees(self.yaw):.0f} deg')
        return v, w

    def _start_escape(self, now, reason):
        """Walk (forwards / backwards and sideways: the Go2 can strafe) away from the
        obstacles around the body - remembered ones too, the lidar is blind there. Along
        a wall that means sideways, in a corner diagonally. If the last escape from the
        same spot did not help, turn the direction by 90, -90, then 180 deg."""
        p = self.p
        if self.escape is not None:
            return
        pts = [self.latest_obstacles[np.hypot(*self.latest_obstacles.T) < 0.7]] if len(self.latest_obstacles) else []
        r = int(0.7 / self.map.resolution)
        row, col = self.map.to_cell(self.x, self.y)
        ra, ca = max(row - r, 0), max(col - r, 0)
        rows, cols = np.nonzero(self.map.occupied[ra:row + r + 1, ca:col + r + 1])
        if len(rows):
            xy = self.map.cell_centres(rows + ra, cols + ca) - (self.x, self.y)
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            pts.append(np.column_stack([c * xy[:, 0] + s * xy[:, 1], -s * xy[:, 0] + c * xy[:, 1]]))
        pts = np.vstack(pts) if pts else np.zeros((0, 2))
        d = np.hypot(pts[:, 0], pts[:, 1])
        keep = (d > 0.01) & (d < 0.7)
        pts, d = pts[keep], d[keep]
        if len(pts):
            push = -(pts / d[:, None] / np.maximum(d, 0.1)[:, None] ** 2).sum(axis=0)
            ang = math.atan2(push[1], push[0]) if np.hypot(*push) > 1e-6 else math.pi
        else:
            ang = math.pi                                    # nothing around: back up
        attempt = 0
        if self.last_escape is not None and now - self.last_escape[0] < 15.0 and \
                math.hypot(self.x - self.last_escape[1], self.y - self.last_escape[2]) < 0.15:
            attempt = self.last_escape[3] + 1
        ang += (0.0, math.pi / 2, -math.pi / 2, math.pi)[attempt % 4]
        speed = p['escape_speed']
        self.escape = (now + p['escape_time'], speed * math.cos(ang), speed * math.sin(ang))
        self.last_escape = (now, self.x, self.y, attempt)
        self.get_logger().info(f'recovery ({reason}): walking {math.degrees(wrap_angle(ang)):.0f} deg '
                               f'off the heading for {p["escape_time"]:.1f} s (attempt {attempt + 1})')

    def _record_motion(self, now, blocked, forward):
        self.motion.append((now, blocked, forward, self.x, self.y))
        while self.motion and now - self.motion[0][0] > max(self.p['blocked_window'], self.p['stuck_window']):
            self.motion.popleft()

    def _recovery_needed(self, now):
        """Fix 1: decide on windows, not on consecutive ticks."""
        p = self.p
        if not self.motion or now - self.motion[0][0] < p['blocked_window'] - 0.2:
            return False
        recent = [m for m in self.motion if now - m[0] <= p['blocked_window']]
        if np.mean([m[1] for m in recent]) >= p['blocked_fraction']:
            self._penalize_ahead(now, 'blocked')
            return True
        if now - self.motion[0][0] >= p['stuck_window'] - 0.2:
            fwd = np.mean([m[2] and not m[1] for m in self.motion])
            x0, y0 = self.motion[0][3], self.motion[0][4]
            moved = max(math.hypot(m[3] - x0, m[4] - y0) for m in self.motion)
            if fwd >= 0.5 and moved < p['stuck_distance']:
                self._penalize_ahead(now, 'stuck')
                return True
        return False

    # ================================================================= control
    def _set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'state {self.state} -> {state}')
        self.state = state
        self.state_since = self._now()
        self.path = None
        self.progress = None
        self.motion.clear()

    def _event(self, text):
        self.get_logger().info(f'EVENT {text}')
        self.event_pub.publish(String(data=text))

    def _elapsed(self, now):
        return now - self.mission_start if self.mission_start is not None else 0.0

    def _tick(self):
        t_wall = time.perf_counter()
        now = self._now()
        self.last_tick = now
        if self.imu_yaw is not None:
            self.yaw = self.imu_yaw
        self.pose_history.append((now, self.x, self.y, self.yaw))
        if self.state == WAIT:
            if self.imu_yaw is not None and self.have_cloud and now >= self.p['start_after_sim_time']:
                self.mission_start = now
                # forget what the lidar saw while CHAMP was still standing up (tilted body,
                # ground classified as obstacles inside the blind zone, never cleared)
                self.map = LiveMap(self.p['map_size'], self.p['map_resolution'])
                self._event('MISSION_START no prior map')
                self.spun = 0.0
                self._set_state(SPIN)
            return
        if self.state != DONE and self._elapsed(now) >= self.p['time_limit']:
            self._event(f'TIME_UP after {self._elapsed(now):.1f} s')
            self._finish(now)
        if self.state in (SPIN, EXPLORE, LOOK):
            target = min((t for t in self.tracks if self._confirmed(t)),
                         key=lambda t: math.hypot(t['x'] - self.x, t['y'] - self.y), default=None)
            if target is not None:
                target['status'] = 'approach'
                self.target = target
                self._event(f'CONFIRMED track_{target["id"]} at ({target["x"]:.2f}, {target["y"]:.2f}) '
                            f'({len(target["hits"])} YOLO hits)')
                self._set_state(APPROACH)
        handler = {SPIN: self._spin, EXPLORE: self._explore, LOOK: self._look, APPROACH: self._approach,
                   SIT: self._sit, STAND_UP: self._stand_up, DONE: lambda now: (0.0, 0.0)}[self.state]
        v, w = handler(now)
        vy = 0.0
        if self.state not in (SIT, STAND_UP, DONE):
            v, w = self._check_turn(now, v, w)
            if self.escape is not None:
                if now < self.escape[0]:
                    v, w, vy = self.escape[1], 0.0, self.escape[2]
                else:
                    self.escape = None
                    self.turns.clear()
                    self.motion.clear()
        else:
            self.escape = None
        if int(now) % 2 == 0:
            self.detector_pub.publish(Bool(data=self.state not in (DONE,)))
        self._publish_cmd(v, w, vy)
        if self.p['debug']:
            self._debug_log(now, v, w)
        if self.trace is not None:
            self._trace_row(now, v, w, (time.perf_counter() - t_wall) * 1000.0)
        if not self.trail or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 0.05:
            self.trail.append((self.x, self.y))
        if self.p['debug_map_dir'] and now - self.last_map_dump > (1e9 if self.state == DONE else 20.0):
            self._dump_map(now)
            self.last_map_dump = now
        if self.p['debug_map_dir'] and self.state == DONE and self.last_map_dump < 1e17:
            self._dump_map(now)
            self.last_map_dump = 1e18

    def _explore(self, now):
        p = self.p
        if self.goal is None or now - self.last_goal_search > p['goal_period']:
            self.last_goal_search = now
            choice = self._choose_goal(now)
            if choice is None and (self.blacklist or self.penalties):
                # maybe the only ways on were given up too early: forget them
                self.get_logger().info('no viewpoint: clearing blacklist and penalties')
                self.blacklist, self.penalties = [], []
                choice = self._choose_goal(now)
            if choice is None:
                choice = self._choose_goal(now, through_unknown=True)
                if choice is not None:
                    self.get_logger().info('no viewpoint through known free space: searching through unknown')
            self.no_goal = self.no_goal + 1 if choice is None else 0
            if choice is not None:
                self.rescue_spins = 0
            if choice is None:
                if self.goal is None and self.no_goal >= 3:
                    if self.rescue_spins < 2:
                        # look around once more: refreshes the map next to the robot
                        self.rescue_spins += 1
                        self.no_goal, self.spun = 0, 0.0
                        self.get_logger().info('no viewpoint: turning once on the spot and trying again')
                        self._set_state(SPIN)
                        return 0.0, 0.0
                    self._event('EXPLORED nothing left to see')
                    self._finish(now)
                    return 0.0, 0.0
                if self.goal is None:
                    return 0.0, 0.3            # turn on the spot to see more
            else:
                goal, look, gain, dist = choice
                if self.goal is None or math.hypot(goal[0] - self.goal[0], goal[1] - self.goal[1]) > 0.3:
                    if p['debug']:
                        self.get_logger().info(f'  goal ({goal[0]:.2f}, {goal[1]:.2f}) gain {gain:.2f} m2 '
                                               f'path {dist:.1f} m')
                    self.goal, self.look_yaw, self.path, self.progress = goal, look, None, None
        gx, gy = self.goal
        if math.hypot(gx - self.x, gy - self.y) < p['goal_reached']:
            self._set_state(LOOK)
            return 0.0, 0.0
        v, w, remaining = self._follow(now, gx, gy)
        if self.path:
            self.no_path = 0
        elif self.no_path < 3:
            self.no_path += 1
        else:
            # boxed in by something only the map believes in (inside the lidar blind
            # zone nothing is ever cleared): forget the map right around the robot
            row, col = self.map.to_cell(self.x, self.y)
            disc = np.zeros(self.map.logodds.shape, np.uint8)
            cv2.circle(disc, (int(col), int(row)), int(p['lidar_blind_range'] / self.map.resolution), 1, -1)
            self.map.logodds[disc.astype(bool)] = 0.0
            self.get_logger().info('recovery (no path to 3 viewpoints): cleared the map around the robot')
            self.no_path = 0
            self.blacklist = []
        if remaining < 0 or not self.path:       # no progress or no path: try another viewpoint
            self.blacklist.append((now + p['blacklist_time'], gx, gy, 0.6))
            self.goal = None
        return v, w

    def _spin(self, now):
        if self.pose_history and len(self.pose_history) > 1:
            self.spun += abs(wrap_angle(self.pose_history[-1][3] - self.pose_history[-2][3]))
        if self.spun > 2 * math.pi - 0.3 or now - self.state_since > self.p['spin_time']:
            self._set_state(EXPLORE)
            return 0.0, 0.0
        return 0.0, 0.5

    def _look(self, now):
        p = self.p
        gx, gy = self.goal
        if self.look_yaw is not None and now - self.state_since < 6.0:
            err = wrap_angle(self.look_yaw - self.yaw)
            if abs(err) > 0.2:
                return 0.0, max(-0.5, min(0.5, 1.5 * err))
        self.blacklist.append((now + p['blacklist_time'], gx, gy, 0.5))
        self.goal = None
        self._set_state(EXPLORE)
        return 0.0, 0.0

    def _approach(self, now):
        p = self.p
        t = self.target
        vx, vy = t['x'], t['y']
        measured = self._victim_distance(vx, vy)
        if measured is not None and measured <= p['sit_distance']:
            t['status'] = 'found'
            self.found += 1
            for o in self.tracks:     # other hypotheses on the same body
                if o is not t and o['status'] == 'candidate' and \
                        math.hypot(o['x'] - t['x'], o['y'] - t['y']) < p['found_suppress_radius']:
                    o['status'] = 'merged'
            self._event(f'SIT track_{t["id"]} measured_distance={measured:.2f} m '
                        f'robot=({self.x:.2f}, {self.y:.2f}) t={self._elapsed(now):.1f} s')
            self._publish_body_pose(p['sit_height_offset'], p['sit_pitch'])
            self.sit_until = now + p['sit_duration']
            self._set_state(SIT)
            return 0.0, 0.0
        if math.hypot(vx - self.x, vy - self.y) < 0.6:
            self.last_path_ok = now
            err = wrap_angle(math.atan2(vy - self.y, vx - self.x) - self.yaw)
            v, w = 0.0, max(-0.4, min(0.4, 1.5 * err))
        else:
            far = math.hypot(vx - self.x, vy - self.y) > p['approach_slow_range']
            v, w, remaining = self._follow(now, vx, vy, v_max=None if far else p['min_forward_speed'])
            if self.path:
                self.last_path_ok = now
        reason = None
        if now - self.state_since > p['approach_timeout']:
            reason = 'could not get within reach'
        elif now - max(self.last_path_ok, self.state_since) > p['approach_no_path']:
            reason = 'no path'
        if reason:
            t['status'], t['failed_at'] = 'failed', now
            self._event(f'GIVE_UP track_{t["id"]}: {reason}')
            self.goal = None
            self._set_state(EXPLORE)
            return 0.0, 0.0
        return v, w

    def _sit(self, now):
        if now < self.sit_until:
            return 0.0, 0.0
        self._publish_body_pose(0.0, 0.0)
        self.sit_until = now + self.p['stand_up_duration']
        self._set_state(STAND_UP)
        return 0.0, 0.0

    def _stand_up(self, now):
        if now < self.sit_until:
            return 0.0, 0.0
        self.settle_until = now + 1.0
        if self.found >= self.p['expected_victims']:
            self._finish(now)
        else:
            self.goal = None
            self._set_state(EXPLORE)
        return 0.0, 0.0

    def _finish(self, now):
        if self.state == DONE:
            return
        self._set_state(DONE)
        self._event(f'DONE found {self.found}/{self.p["expected_victims"]} in {self._elapsed(now):.1f} s; '
                    f'icp {self.icp_ok}/{self.icp_tries} scans corrected')

    # ================================================================= outputs
    def _publish_cmd(self, v, w, vy=0.0):
        self.cmd_v, self.cmd_w = v, w
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(v), float(vy), float(w)
        self.cmd_pub.publish(msg)

    def _publish_body_pose(self, height_offset, pitch):
        msg = Pose()
        msg.position.z = float(height_offset)
        msg.orientation.y = math.sin(pitch / 2.0)
        msg.orientation.w = math.cos(pitch / 2.0)
        self.body_pose_pub.publish(msg)

    def _publish_path(self):
        msg = Path()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path or []:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _publish_status(self):
        now = self._now()
        self.status_pub.publish(String(data=(
            f'state={self.state} found={self.found}/{self.p["expected_victims"]} '
            f't={self._elapsed(now):.0f}/{self.p["time_limit"]:.0f}s pose=({self.x:.2f}, {self.y:.2f}, '
            f'{math.degrees(self.yaw):.0f}deg) seen={int(self.map.seen.sum() * self.map.resolution ** 2)}m2')))
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = self.x, self.y
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.pose_pub.publish(ps)
        markers = MarkerArray()
        colours = {'candidate': (0.9, 0.6, 0.1), 'approach': (0.9, 0.9, 0.1), 'merged': (0.5, 0.5, 0.5),
                   'found': (0.1, 0.9, 0.2), 'failed': (0.9, 0.1, 0.1)}
        for t in self.tracks:
            m = Marker()
            m.header.frame_id = 'world'
            m.ns, m.id, m.type, m.action = 'victims', t['id'], Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = t['x'], t['y'], 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.4
            m.color.r, m.color.g, m.color.b = colours[t['status']]
            m.color.a = 0.8
            markers.markers.append(m)
        self.marker_pub.publish(markers)

    def _debug_log(self, now, v, w):
        front = corridor_clearance(self.latest_obstacles, np.array([0.0]), self.p['robot_half_width'],
                                   self.p['max_range'])[0]
        goal = f' goal=({self.goal[0]:.2f},{self.goal[1]:.2f})' if self.goal else ''
        if self.state == APPROACH and self.target:
            m = self._victim_distance(self.target['x'], self.target['y'])
            goal = f' target=({self.target["x"]:.2f},{self.target["y"]:.2f}) dist={m if m is None else round(m, 2)}'
        self.get_logger().info(
            f'[debug] {self.state} t={self._elapsed(now):.0f} pose=({self.x:.2f},{self.y:.2f},'
            f'{math.degrees(self.yaw):.0f}deg) front={front:.2f} cmd=({v:.2f},{w:.2f}){goal} '
            f'tracks={len(self.tracks)} seen={self.map.seen.sum() * self.map.resolution ** 2:.1f}m2',
            throttle_duration_sec=2.0)

    def _trace_row(self, now, v, w, tick_ms):
        front = corridor_clearance(self.latest_obstacles, np.array([0.0]), self.p['robot_half_width'],
                                   self.p['max_range'])[0]
        goal = self.goal
        if self.state == APPROACH and self.target:
            goal = (self.target['x'], self.target['y'])
        gx, gy = goal if goal else (math.nan, math.nan)
        a = self.map.resolution ** 2
        self.trace.write(
            f'{now:.2f},{self._elapsed(now):.2f},{self.state},{self.x:.3f},{self.y:.3f},'
            f'{math.degrees(self.yaw):.1f},{v:.3f},{w:.3f},{gx:.2f},{gy:.2f},{min(front, 9.9):.2f},'
            f'{int(self._blind_zone_blocked())},{len(self.path or [])},{len(self.tracks)},{self.found},'
            f'{self.map.seen.sum() * a:.2f},{self.map.free.sum() * a:.2f},{self.icp_ok},{self.icp_tries},'
            f'{self.ground_z:.3f},{tick_ms:.0f}\n')

    def _dump_map(self, now):
        """Map snapshot: black occupied, white free, grey unknown, green tint = seen by
        the camera, blue trail, yellow path, red tracks (green when found)."""
        import os
        r0, r1, c0, c1 = self.map.known_bounds(0.5)
        img = np.full((self.map.height, self.map.width, 3), 128, np.uint8)
        img[self.map.free] = (255, 255, 255)
        img[self.map.free & self.map.seen] = (200, 255, 200)
        img[self.map.occupied] = (0, 0, 0)
        for (ax, ay), (bx, by) in zip(self.trail, self.trail[1:]):
            a, b = self.map.to_cell(ax, ay), self.map.to_cell(bx, by)
            cv2.line(img, (a[1], a[0]), (b[1], b[0]), (255, 0, 0), 1)
        for x, y in self.path or []:
            r, c = self.map.to_cell(x, y)
            if self.map.inside(r, c):
                img[r, c] = (0, 220, 255)
        for t in self.tracks:
            r, c = self.map.to_cell(t['x'], t['y'])
            cv2.circle(img, (c, r), 4, (0, 200, 0) if t['status'] == 'found' else (0, 0, 255), -1)
        r, c = self.map.to_cell(self.x, self.y)
        cv2.circle(img, (c, r), 5, (255, 0, 255), 2)
        crop = np.flipud(img[r0:r1, c0:c1])
        crop = cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
        d = os.path.expanduser(self.p['debug_map_dir'])
        os.makedirs(d, exist_ok=True)
        name = os.path.join(d, f'map_t{self._elapsed(now):05.1f}_{self.state}')
        cv2.imwrite(name + '.png', crop)
        np.savez_compressed(name + '.npz', logodds=self.map.logodds, seen=self.map.seen,
                            pose=np.array([self.x, self.y, self.yaw, now]),
                            goal=np.array(self.goal if self.goal else (np.nan, np.nan)),
                            blacklist=np.array(self.blacklist).reshape(-1, 4),
                            penalties=np.array(self.penalties).reshape(-1, 4))

    def stop(self):
        try:
            self._publish_cmd(0.0, 0.0)
            self._publish_body_pose(0.0, 0.0)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = NoMapExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
