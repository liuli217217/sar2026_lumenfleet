"""Map-based search and rescue for the Go2.

The robot knows a prior map (walls only) and rough victim positions. It has
3 minutes (sim time) to reach every victim: a victim counts as found when the
robot is within 1 m of it and sits down.

  WAIT      until the CHAMP controllers are up
  GOTO      follow an A* path towards the next victim's estimated position
  SEARCH    within `search_radius` of the estimate: YOLO on, confirm the victim
  APPROACH  walk to the confirmed victim until the lidar says < `sit_distance`
  SIT       sit down next to it (= found), then STAND_UP and go to the next one
  DONE      all victims handled or time is up

No ground truth is used. Pose = IMU heading + CHAMP leg odometry (/odom),
corrected by matching the lidar scan against the walls of the prior map.
Obstacles that are not in the map are added to a short-term obstacle layer
from the lidar and the path is re-planned around them.

Motion safety (the same as go2_sar_slam): blocked / stuck is judged over time
windows (a robot alternating "blocked" / "free" every tick still triggers the
recovery), the spot ahead then gets a high path cost so the planner picks another
route, a goal without progress for `progress_timeout` does the same; trying to turn
without the yaw changing (body against an obstacle the lidar cannot see that close)
or blocked / stuck -> walk away from it, sideways if needed (the Go2 can strafe).
Detections are checked against spectators standing outside the arena: victims lie
or sit on the floor, and no known wall may lie between the camera and the person.
"""
import itertools
import math
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Imu, PointCloud2
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from go2_sar_core.map_utils import OccupancyMap, load_victims
from go2_sar_core.pointcloud_utils import (cloud_to_xyz, corridor_clearance, icp_translation,
                                      rotation_rpy, voxel_downsample_2d, wrap_angle,
                                      yaw_from_quaternion)

WAIT, GOTO, SEARCH, APPROACH, SIT, STAND_UP, DONE = (
    'WAIT', 'GOTO', 'SEARCH', 'APPROACH', 'SIT', 'STAND_UP', 'DONE')


def stamp_of(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class SarNavigator(Node):
    def __init__(self):
        super().__init__('sar_prior')
        p = self._declare_parameters()
        self.p = p

        # ---------------------------------------------------------- prior knowledge
        self.map = OccupancyMap(p['map_yaml'])
        self.static_inflated = self.map.inflate(self.map.occupied, p['robot_radius'])
        self.wall_dist = self.map.distance_field(self.map.occupied)
        self.wall_points = self.map.wall_points()
        self.victims = [{'name': n, 'x': x, 'y': y, 'status': 'pending'}
                        for n, x, y in load_victims(p['victims_yaml'])]
        self.obstacle_seen = np.full(self.map.occupied.shape, -1e9)   # last time a cell was hit
        self.obstacle_hits = np.zeros(self.map.occupied.shape, np.int32)  # scans that hit the cell

        # ---------------------------------------------------------------- state
        self.x = self.y = self.yaw = 0.0          # world frame = start pose
        self.imu_yaw0 = None
        self.imu_yaw = None
        self.tilt_history = deque(maxlen=300)
        self.pose_history = deque(maxlen=300)
        self.last_odom = None
        self.have_odom = False
        self.ground_z = p['ground_z_init']
        self.latest_obstacles = np.zeros((0, 2))   # base frame, last scan
        self.latest_points = np.zeros((0, 3))      # base frame xyz (+ height) for victim ranging
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
        self.order = []
        self.current = None           # index into self.victims
        self.path = None
        self.path_goal = None
        self.last_plan = -1e9
        self.hits = []                # (x, y, score) YOLO + lidar fixes of the current victim
        self.victim_fix = None        # confirmed victim position
        self.motion = deque()         # (t, blocked, forward_intent, x, y)
        self.progress = None          # (best remaining distance, time) for the current goal
        self.penalties = []           # (until, x, y, r): spots with a high path cost
        self.turns = deque()          # (t, yaw, turning)
        self.escape = None            # (until, vx, vy): walk away from what the body touches
        self.last_escape = None       # (t, x, y, attempt)
        self.settle_until = -1e9      # no obstacle updates before (after standing up)
        self.corridor_free = None     # cells far enough from obstacles for straight cuts
        self.clearance = None         # distance [m] to walls and obstacles, last plan
        self.sit_until = None
        self.cmd_v = self.cmd_w = 0.0
        self.detector_on = None
        self.last_tick = None
        self.icp_ok = self.icp_tries = 0
        self.trace = None
        if p['trace_file']:
            # same columns as go2_sar_slam's trace (scripts/analyze_run.py reads both)
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
        # Which clock the limit is counted on depends on use_sim_time, which the
        # launch file sets per target (true in Gazebo, false on the robot) -- say
        # which one it actually is, so nobody reads "sim time" on the robot.
        clock = 'sim time' if self.get_parameter('use_sim_time').value else 'wall clock'
        self.get_logger().info(
            f'map {self.map.width}x{self.map.height} @ {self.map.resolution} m, '
            f'{len(self.victims)} victims to find, time limit {p["time_limit"]:.0f} s ({clock})')

    def _declare_parameters(self):
        try:        # the real robot has no disaster_models: map_yaml / victims_yaml are given
            mission = os.path.join(get_package_share_directory('disaster_models'), 'missions', 'arena_5x8')
        except LookupError:
            mission = ''
        defaults = {
            'map_yaml': os.path.join(mission, 'map.yaml'),
            'victims_yaml': os.path.join(mission, 'victims_estimated.yaml'),
            'cloud_topic': '/velodyne_points/points',
            'imu_topic': '/imu/data',
            'odom_topic': '/odom',                  # CHAMP leg odometry + EKF (not ground truth)
            'detections_topic': '/sar/person_detections',
            'cmd_vel_topic': '/cmd_vel',
            'body_pose_topic': '/body_pose',
            'detector_enable_topic': '/sar/detector_enable',
            'time_limit': 180.0,                    # [s, sim time] from the mission start
            'start_after_sim_time': 40.0,           # CHAMP controllers come up at ~30 s
            'control_rate_hz': 10.0,
            'debug': False,
            'trace_file': '',                       # if set: one CSV row per control tick
            # Sensors (Go2 URDF).
            'lidar_xyz': [0.2, 0.0, 0.1177],
            'lidar_rpy': [0.0, 0.0, 0.0],
            'camera_xyz': [0.326, 0.0, 0.043],
            'camera_hfov': 1.367,
            'camera_info_topic': '',                # real camera: intrinsics from here
            'image_width': 640,
            'image_height': 480,
            # Lidar processing.
            'ground_z_init': -0.21,
            'ground_z_min': -0.32,
            'ground_z_max': -0.10,
            'obstacle_min_height': 0.07,
            'tilt_tolerance': 3.0,                  # [deg] residual attitude error allowed for
            'obstacle_max_height': 0.75,
            'max_range': 5.0,
            'body_box': [-0.5, 0.35, 0.25],         # [x min, x max, half width] own body in the scan
            # Localization (scan-to-map matching).
            'velocity_scale': 0.6,                  # only if /odom is missing
            'map_matching': True,
            'icp_gate': 0.25,
            'icp_min_inliers': 0.45,
            'icp_max_correction': 0.25,
            # Planning.
            'robot_radius': 0.25,                   # hard inflation (Go2 half width 0.155; 0.30 closes 0.7 m gaps)
            'body_clearance': 0.15,                 # never plan closer to an obstacle than this
            'follow_clearance': 0.20,               # the follower does not cut corners closer than this
            'cost_distance': 0.65,                  # prefer paths away from walls and obstacles
            'cost_weight': 6.0,
            'obstacle_memory': 180.0,               # [s] keep unknown obstacles (also in the blind zone)
            'obstacle_min_hits': 2,                 # scans that must see a cell before it counts
            'obstacle_wall_margin': 0.15,           # lidar points this close to a map wall are the wall
            'obstacle_mark_range': 3.0,
            'replan_period': 1.0,
            # Motion.
            'lookahead': 0.5,
            'max_linear_speed': 0.3,
            'min_forward_speed': 0.22,              # CHAMP walks on the spot below ~0.15 m/s
            'max_angular_speed': 0.6,
            'heading_gain': 1.5,
            'robot_half_width': 0.22,
            'stop_distance': 0.45,                  # emergency stop (latest scan, straight ahead)
            'blind_half_width': 0.15,               # remembered obstacles checked this far to the sides
            'arc_clearance': 0.6,                   # turn while walking if this free around the body
            'blocked_window': 3.0,                  # blocked >= blocked_fraction of this window -> recover
            'blocked_fraction': 0.4,
            'stuck_window': 5.0,                    # walking but moved < stuck_distance in this window
            'stuck_distance': 0.10,
            'progress_timeout': 10.0,               # [s] no progress towards the goal -> other route
            'penalty_cost': 10.0,
            'penalty_time': 60.0,
            'turn_stall_time': 2.5,                 # turning without yaw change -> escape
            'turn_stall_angle': 0.15,
            'escape_speed': 0.2,
            'escape_time': 1.5,
            # Victims.
            'search_radius': 2.0,                   # YOLO on within this distance of the estimate
            'association_radius': 1.2,              # detections this close to the estimate count
            'detection_min_score': 0.35,
            'min_person_extent': 0.35,              # [m] box size x range: smaller -> range is an occluder
            'victim_max_bottom': 0.45,              # [m above the floor] box bottom edge (on the floor)
            'victim_max_top': 1.3,                  # [m] box top edge (a sitting person is ~0.9 m)
            'approach_fallback_radius': 1.2,        # unreachable victim cell: closest reachable one
            'confirm_hits': 3,
            'confirm_min_score': 0.4,
            'victim_max_height': 0.42,              # a lying person is low; walls are not
            'victim_cluster_radius': 0.45,           # around the fix (= near end of the body); victims can lie end to end 0.56 m apart
            'success_distance': 1.0,                # task definition
            'sit_distance': 0.75,                   # approach goal (margin below 1 m)
            'victim_timeout': 35.0,                 # [s] per victim in SEARCH + APPROACH
            'goto_timeout': 50.0,                   # [s] on the way: postpone the victim once, then give up
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
            # The odom frame starts at the spawn pose with the IMU heading, like the world frame.
            self.x += pos.x - self.last_odom[0]
            self.y += pos.y - self.last_odom[1]
        self.last_odom = (pos.x, pos.y)
        self.have_odom = True

    def _tilt_at(self, stamp):
        if not self.tilt_history:
            return 0.0, 0.0
        times = np.fromiter((h[0] for h in self.tilt_history), float, len(self.tilt_history))
        return self.tilt_history[int(np.abs(times - stamp).argmin())][1:]

    def _on_cloud(self, msg):
        pts = cloud_to_xyz(msg)
        if len(pts) == 0:
            return
        pts = pts @ self.lidar_R.T + self.lidar_t
        roll, pitch = self._tilt_at(stamp_of(msg.header))
        pts = pts @ rotation_rpy(roll, pitch, 0.0).T             # gravity-levelled base frame
        r = np.hypot(pts[:, 0], pts[:, 1])

        ground = pts[(r > 0.8) & (r < 4.0) & (np.abs(pts[:, 2] - self.ground_z) < 0.10), 2]
        if len(ground) > 30:
            self.ground_z = 0.9 * self.ground_z + 0.1 * float(np.percentile(ground, 20))
            self.ground_z = min(max(self.ground_z, self.p['ground_z_min']), self.p['ground_z_max'])
        height = pts[:, 2] - self.ground_z
        in_range = r < self.p['max_range']
        min_h = self.p['obstacle_min_height'] + r * math.tan(math.radians(self.p['tilt_tolerance']))
        # own rear legs / body: ~0.5 m behind the lidar, right at its minimum range
        bx0, bx1, bhw = self.p['body_box']
        body = (pts[:, 0] > bx0) & (pts[:, 0] < bx1) & (np.abs(pts[:, 1]) < bhw)
        obstacle = in_range & ~body & (height > min_h) & (height < self.p['obstacle_max_height'])
        self.latest_obstacles = voxel_downsample_2d(pts[obstacle, :2], 0.05)
        keep = in_range & ~body & (height > 0.03) & (height < 1.2)
        self.latest_points = np.column_stack([pts[keep, :2], height[keep]])
        self.have_cloud = True
        if self.state in (WAIT, SIT, STAND_UP) or self._now() < self.settle_until:
            return              # lowered, pitched body while sitting: the ground looks like obstacles
        if self.p['map_matching']:
            self._match_map(self.latest_obstacles)
        self._mark_obstacles(self.latest_obstacles)

    def _to_world(self, pts):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.column_stack([self.x + c * pts[:, 0] - s * pts[:, 1],
                                self.y + s * pts[:, 0] + c * pts[:, 1]])

    def _match_map(self, scan):
        """Correct the position by aligning the scan to the walls of the prior map."""
        if len(scan) < 30:
            return
        src = voxel_downsample_2d(scan, 0.1)
        if len(src) > 300:
            src = src[np.random.default_rng().choice(len(src), 300, replace=False)]
        world = self._to_world(src)
        # only map walls near the scan take part
        lo, hi = world.min(axis=0) - 0.5, world.max(axis=0) + 0.5
        ref = self.wall_points[np.all((self.wall_points >= lo) & (self.wall_points <= hi), axis=1)]
        self.icp_tries += 1
        result = icp_translation(world, ref, max_correspondence=self.p['icp_gate'], min_inliers=20)
        if result is None:
            return
        dx, dy, ratio = result
        if ratio >= self.p['icp_min_inliers'] and math.hypot(dx, dy) <= self.p['icp_max_correction']:
            self.x += dx
            self.y += dy
            self.icp_ok += 1

    def _mark_obstacles(self, scan):
        """Lidar obstacles that are not map walls go into the obstacle layer."""
        near = scan[np.hypot(scan[:, 0], scan[:, 1]) < self.p['obstacle_mark_range']]
        if len(near) == 0:
            return
        world = self._to_world(near)
        now = self._now()
        rows = np.floor((world[:, 1] - self.map.origin[1]) / self.map.resolution).astype(np.int64)
        cols = np.floor((world[:, 0] - self.map.origin[0]) / self.map.resolution).astype(np.int64)
        ok = (rows >= 0) & (rows < self.map.height) & (cols >= 0) & (cols < self.map.width)
        keys = np.unique(rows[ok] * self.map.width + cols[ok])      # each cell once per scan
        rows, cols = keys // self.map.width, keys % self.map.width
        keep = self.wall_dist[rows, cols] > self.p['obstacle_wall_margin']
        rows, cols = rows[keep], cols[keep]
        self.obstacle_seen[rows, cols] = now
        self.obstacle_hits[rows, cols] += 1

    # ============================================================== detections
    def _on_detections(self, msg):
        if self.state not in (SEARCH, APPROACH) or self.current is None:
            return
        victim = self.victims[self.current]
        ref = self.victim_fix or (victim['x'], victim['y'])
        px, py, pyaw = self._pose_at(stamp_of(msg.header))
        for det in msg.detections:
            if not det.results:
                continue
            score = det.results[0].hypothesis.score
            if score < self.p['detection_min_score']:
                continue
            fix = self._locate(det, px, py, pyaw)
            if fix is None:
                continue
            if math.hypot(fix[0] - ref[0], fix[1] - ref[1]) > self.p['association_radius']:
                continue
            self.hits.append((fix[0], fix[1], score))
            if self.p['debug']:
                self.get_logger().info(f'  {victim["name"]} hit {len(self.hits)}: score {score:.2f} '
                                       f'at ({fix[0]:.2f}, {fix[1]:.2f}), range {fix[2]:.2f} m')

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

    def _low_points(self, pts):
        """Points of low objects only (cells whose highest return is below
        victim_max_height): separates a lying person from the wall behind it."""
        if len(pts) == 0:
            return pts
        cells = np.floor(pts[:, :2] / 0.1).astype(np.int64)
        keys = cells[:, 0] * 100003 + cells[:, 1]
        order = np.argsort(keys)
        sorted_keys = keys[order]
        starts = np.r_[0, np.flatnonzero(np.diff(sorted_keys)) + 1]
        cell_max = np.maximum.reduceat(pts[order, 2], starts)
        max_per_point = np.empty(len(pts))
        max_per_point[order] = np.repeat(cell_max, np.diff(np.r_[starts, len(pts)]))
        return pts[max_per_point <= self.p['victim_max_height']]

    def _locate(self, det, px, py, pyaw):
        """Camera bearing + lidar range on low points -> victim position (world)."""
        p = self.p
        cx0, cy0 = self.cx0, self.cy0
        u, half_w = det.bbox.center.position.x, det.bbox.size_x / 2.0
        bearing = -math.atan((u - cx0) / self.fx)
        lo = -math.atan((u + 0.5 * half_w - cx0) / self.fx)
        hi = -math.atan((u - 0.5 * half_w - cx0) / self.fx)
        cam_x, cam_y, cam_z = p['camera_xyz']
        rng = None
        low = self._low_points(self.latest_points)
        if len(low):
            rx, ry = low[:, 0] - cam_x, low[:, 1] - cam_y
            ang = np.arctan2(ry, rx)
            sel = (rx > 0.2) & (ang >= lo - 0.03) & (ang <= hi + 0.03)
            if sel.sum() >= 3:
                rng = float(np.percentile(np.hypot(rx[sel], ry[sel]), 20))
        if rng is None:
            v_bottom = det.bbox.center.position.y + det.bbox.size_y / 2.0
            alpha = math.atan((v_bottom - cy0) / self.fx)
            if alpha > 0.03:
                rng = (cam_z - self.ground_z) / math.tan(alpha)
        if rng is None or rng > 6.0:
            return None
        # a far person behind a near edge gets the edge's range: the box is then far
        # too small for a person at that distance
        if max(det.bbox.size_x, det.bbox.size_y) * rng / self.fx < p['min_person_extent']:
            return None
        # victims lie or sit on the floor: the box must reach down to the floor and not
        # far above a sitting person. Spectators outside the arena show only their heads
        # and shoulders above the 1.2 m walls (box bottom = wall top, range = the wall).
        cam_h = cam_z - self.ground_z
        v_top = det.bbox.center.position.y - det.bbox.size_y / 2.0
        v_bot = det.bbox.center.position.y + det.bbox.size_y / 2.0
        if v_bot < self.img_h - 3 and cam_h - (v_bot - cy0) * rng / self.fx > p['victim_max_bottom']:
            return None
        if v_top > 3 and cam_h + (cy0 - v_top) * rng / self.fx > p['victim_max_top']:
            return None
        bx, by = cam_x + rng * math.cos(bearing), cam_y + rng * math.sin(bearing)
        c, s = math.cos(pyaw), math.sin(pyaw)
        fx, fy = px + c * bx - s * by, py + s * bx + c * by
        # no known wall between the camera and the person (with the nose at a wall the
        # camera and the lidar see through it)
        ax, ay = px + c * cam_x - s * cam_y, py + s * cam_x + c * cam_y
        n = int((rng - 0.4) / self.map.resolution)
        if n > 0:
            f = np.arange(n) * self.map.resolution / rng
            rows = np.floor((ay + f * (fy - ay) - self.map.origin[1]) / self.map.resolution).astype(np.int64)
            cols = np.floor((ax + f * (fx - ax) - self.map.origin[0]) / self.map.resolution).astype(np.int64)
            ok = (rows >= 0) & (rows < self.map.height) & (cols >= 0) & (cols < self.map.width)
            if self.map.occupied[rows[ok], cols[ok]].any():
                return None
        return fx, fy, rng

    def _victim_distance(self, vx, vy):
        """Distance from the robot centre to the nearest point of the victim, from
        the current scan and from the remembered obstacle cells around (vx, vy)
        (needed once the victim is inside the lidar blind zone, < ~0.7 m)."""
        scan = self._victim_distance_scan(vx, vy)
        layer = self._obstacle_layer(self._now())
        r = int(math.ceil(self.p['victim_cluster_radius'] / self.map.resolution))
        row0, col0 = self.map.to_cell(vx, vy)
        rows, cols = np.nonzero(layer[max(row0 - r, 0):row0 + r + 1, max(col0 - r, 0):col0 + r + 1])
        remembered = None
        if len(rows):
            wx, wy = self.map.to_world(rows + max(row0 - r, 0), cols + max(col0 - r, 0))
            near = np.hypot(wx - vx, wy - vy) < self.p['victim_cluster_radius']
            if near.any():
                # cell centres: subtract half a cell so a touching body reads ~0
                remembered = float(np.min(np.hypot(wx[near] - self.x, wy[near] - self.y))) - self.map.resolution / 2
        values = [d for d in (scan, remembered) if d is not None]
        return min(values) if values else None

    def _victim_distance_scan(self, vx, vy):
        low = self._low_points(self.latest_points)
        if len(low) == 0:
            return None
        world = self._to_world(low[:, :2])
        near = np.hypot(world[:, 0] - vx, world[:, 1] - vy) < self.p['victim_cluster_radius']
        if near.sum() < 3:
            return None
        world, base = world[near], low[near, :2]
        rows = np.floor((world[:, 1] - self.map.origin[1]) / self.map.resolution).astype(int)
        cols = np.floor((world[:, 0] - self.map.origin[0]) / self.map.resolution).astype(int)
        ok = (rows >= 0) & (rows < self.map.height) & (cols >= 0) & (cols < self.map.width)
        not_wall = np.zeros(len(world), bool)
        not_wall[ok] = self.wall_dist[rows[ok], cols[ok]] > 0.12
        if not_wall.sum() < 3:
            return None
        return float(np.min(np.hypot(base[not_wall, 0], base[not_wall, 1])))

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

    def _tick(self):
        t_wall = time.perf_counter()
        now = self._now()
        dt = 0.0 if self.last_tick is None else min(max(now - self.last_tick, 0.0), 0.5)
        self.last_tick = now
        if self.imu_yaw is not None:
            self.yaw = self.imu_yaw
        if not self.have_odom and self.state != WAIT:
            self.x += self.cmd_v * self.p['velocity_scale'] * dt * math.cos(self.yaw)
            self.y += self.cmd_v * self.p['velocity_scale'] * dt * math.sin(self.yaw)
        self.pose_history.append((now, self.x, self.y, self.yaw))

        if self.state == WAIT:
            if self.imu_yaw is not None and self.have_cloud and now >= self.p['start_after_sim_time']:
                self._start_mission(now)
            return

        if self.state != DONE and now - self.mission_start >= self.p['time_limit']:
            self._event(f'TIME_UP after {now - self.mission_start:.1f} s')
            self._finish()

        handler = {GOTO: self._goto, SEARCH: self._search, APPROACH: self._approach,
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
        self._set_detector(self.state in (SEARCH, APPROACH))
        self._publish_cmd(v, w, vy)
        if self.p['debug']:
            self._debug_log(now, v, w)
        if self.trace is not None:
            self._trace_row(now, v, w, (time.perf_counter() - t_wall) * 1000.0)

    def _trace_row(self, now, v, w, tick_ms):
        front = corridor_clearance(self.latest_obstacles, np.array([0.0]), self.p['robot_half_width'],
                                   self.p['max_range'])[0]
        goal = (math.nan, math.nan)
        if self.current is not None:
            vi = self.victims[self.current]
            goal = self.victim_fix if self.state == APPROACH and self.victim_fix else (vi['x'], vi['y'])
        found = sum(v_['status'] == 'found' for v_ in self.victims)
        elapsed = now - self.mission_start if self.mission_start is not None else 0.0
        self.trace.write(
            f'{now:.2f},{elapsed:.2f},{self.state},{self.x:.3f},{self.y:.3f},{math.degrees(self.yaw):.1f},'
            f'{v:.3f},{w:.3f},{goal[0]:.2f},{goal[1]:.2f},{min(front, 9.9):.2f},'
            f'{int(self._blind_zone_blocked(now))},{len(self.path or [])},{len(self.hits)},{found},0,0,'
            f'{self.icp_ok},{self.icp_tries},{self.ground_z:.3f},{tick_ms:.0f}\n')

    def _start_mission(self, now):
        self.mission_start = now
        self.order = self._best_order()
        names = ' -> '.join(self.victims[i]['name'] for i in self.order)
        self._event(f'MISSION_START order: {names}')
        self._next_victim()

    def _best_order(self):
        """Shortest visiting order (A* lengths, all permutations; 4 victims = 24)."""
        blocked = self.static_inflated
        cost = self._cost_grid()
        goals = [self.map.nearest_free(blocked, v['x'], v['y']) or (v['x'], v['y']) for v in self.victims]
        points = [(self.x, self.y)] + goals
        n = len(points)
        dist = np.full((n, n), math.inf)
        for i in range(n):
            for j in range(i + 1, n):
                _, length = self.map.plan(blocked, cost, points[i], points[j])
                dist[i, j] = dist[j, i] = length
        best, best_len = list(range(len(goals))), math.inf
        for perm in itertools.permutations(range(len(goals))):
            length = dist[0, perm[0] + 1] + sum(dist[a + 1, b + 1] for a, b in zip(perm, perm[1:]))
            if length < best_len:
                best, best_len = list(perm), length
        self.get_logger().info(f'planned route length {best_len:.1f} m')
        return best

    def _next_victim(self):
        pending = [i for i in self.order if self.victims[i]['status'] == 'pending']
        if not pending:
            self._finish()
            return
        self.current = pending[0]
        self.hits, self.victim_fix, self.path = [], None, None
        self.last_plan = -1e9
        v = self.victims[self.current]
        self.get_logger().info(f'next: {v["name"]} (estimate {v["x"]:.2f}, {v["y"]:.2f})')
        self._set_state(GOTO)

    def _finish(self):
        found = [v['name'] for v in self.victims if v['status'] == 'found']
        self._publish_body_pose(0.0, 0.0)
        self._set_state(DONE)
        elapsed = self._now() - self.mission_start if self.mission_start is not None else 0.0
        self._event(f'DONE found {len(found)}/{len(self.victims)} in {elapsed:.1f} s: {", ".join(found) or "-"}')

    # ------------------------------------------------------------ path following
    def _cost_grid(self):
        """Static cost (walls only): visiting order."""
        return np.clip(self.p['cost_distance'] - self.wall_dist, 0.0, None) * self.p['cost_weight']

    def _obstacle_layer(self, now):
        return ((now - self.obstacle_seen) < self.p['obstacle_memory']) & \
            (self.obstacle_hits >= self.p['obstacle_min_hits'])

    def _grids(self, now):
        """(blocked, cost) from the walls, the obstacle layer and the penalties. Around
        the robot only the inflation is opened (it may stand inside it), never the
        obstacles themselves."""
        p = self.p
        occ = self.map.occupied | self._obstacle_layer(now)
        dist = self.map.distance_field(occ)
        self.clearance = dist
        self.corridor_free = dist > p['follow_clearance']
        blocked = dist <= p['robot_radius']
        span = p['cost_distance'] - p['robot_radius']
        cost = (p['cost_weight'] * np.clip((p['cost_distance'] - dist) / span, 0.0, 1.0)).astype(np.float32)
        row, col = self.map.to_cell(self.x, self.y)
        if self.map.inside(row, col):
            here = float(dist[row, col])
            core = dist <= max(min(p['body_clearance'], here - 0.05), 0.0)
            near = np.zeros(blocked.shape, np.uint8)
            cv2.circle(near, (int(col), int(row)), int(0.4 / self.map.resolution), 1, -1)
            blocked = np.where(near.astype(bool), core | self.map.occupied, blocked)
        self.penalties = [q for q in self.penalties if q[0] > now]
        for _, x, y, rad in self.penalties:
            prow, pcol = self.map.to_cell(x, y)
            if self.map.inside(prow, pcol):
                cv2.circle(cost, (int(pcol), int(prow)), int(rad / self.map.resolution), float(p['penalty_cost']), -1)
        return blocked, cost

    def _replan(self, now, gx, gy):
        blocked, cost = self._grids(now)
        if self.state == APPROACH:      # the victim itself is an obstacle, often against a wall
            goal, fallback = (gx, gy), self.p['approach_fallback_radius']
        else:
            goal, fallback = self.map.nearest_free(blocked, gx, gy) or (gx, gy), 0.0
        path, _ = self.map.plan(blocked, cost, (self.x, self.y), goal, fallback_radius=fallback)
        if path is None:   # e.g. boxed in by remembered obstacles: plan on the static map
            path, _ = self.map.plan(self.static_inflated, self._cost_grid(), (self.x, self.y), goal,
                                    fallback_radius=fallback)
        self.path, self.path_goal, self.last_plan = path, goal, now
        self._publish_path()

    def _penalize_ahead(self, now, reason):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        px, py = self.x + 0.45 * c, self.y + 0.45 * s
        self.penalties.append((now + self.p['penalty_time'], px, py, 0.35))
        self.get_logger().info(f'recovery ({reason}): high cost at ({px:.2f}, {py:.2f}), re-planning')
        self.last_plan = -1e9
        self.motion.clear()
        self.progress = None

    def _follow(self, now, gx, gy, v_max=None):
        """Drive along the path to (gx, gy). Returns (v, w, remaining distance)."""
        p = self.p
        if self.path is None or now - self.last_plan > p['replan_period']:
            self._replan(now, gx, gy)
        if not self.path:
            return 0.0, 0.0, math.hypot(gx - self.x, gy - self.y)
        pts = np.asarray(self.path)
        d = np.hypot(pts[:, 0] - self.x, pts[:, 1] - self.y)
        i = int(d.argmin())
        j = i
        while j < len(pts) - 1 and d[j] < p['lookahead']:
            j += 1
        # pure pursuit cuts corners: aim at the farthest point the body reaches in a
        # straight line with enough clearance (door edges, wall ends, cones)
        while j > i + 1 and not self._straight_clear(*pts[j]):
            j -= 1
        remaining = float(np.sum(np.hypot(np.diff(pts[i:, 0]), np.diff(pts[i:, 1])))) + d[i]
        # no progress towards this goal for a while -> high cost ahead, another route
        if self.progress is None or remaining < self.progress[0] - 0.25:
            self.progress = (remaining, now)
        elif now - self.progress[1] > p['progress_timeout']:
            self._penalize_ahead(now, 'no progress')
        v, w = self._drive_to(now, *pts[j], v_max or p['max_linear_speed'])
        return v, w, remaining

    def _straight_clear(self, tx, ty):
        length = math.hypot(tx - self.x, ty - self.y)
        if length < 0.2 or self.corridor_free is None:
            return True
        f = np.arange(0.15, length, self.map.resolution) / length
        rows = np.floor((self.y + f * (ty - self.y) - self.map.origin[1]) / self.map.resolution).astype(np.int64)
        cols = np.floor((self.x + f * (tx - self.x) - self.map.origin[0]) / self.map.resolution).astype(np.int64)
        ok = (rows >= 0) & (rows < self.map.height) & (cols >= 0) & (cols < self.map.width)
        return bool(self.corridor_free[rows[ok], cols[ok]].all())

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
            self._record_motion(now, False, False)
            if room and front > 2 * p['stop_distance'] and not self._blind_zone_blocked(now):
                return p['min_forward_speed'], w
            return 0.0, w
        blocked = front < p['stop_distance'] or self._blind_zone_blocked(now)
        self._record_motion(now, blocked, True)
        if self._recovery_needed(now):
            self._start_escape(now, 'blocked / stuck')
            return 0.0, 0.0
        if blocked:
            self.last_plan = -1e9
            return 0.0, w
        return max(v_max * math.cos(err), p['min_forward_speed']), w

    def _blind_zone_blocked(self, now):
        """The lidar cannot see closer than ~0.7 m from the body centre: check the
        remembered obstacle layer for anything right in front of the robot."""
        layer = self._obstacle_layer(now)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hw = self.p['blind_half_width']
        for ahead in (0.35, 0.45, 0.55):
            for side in (-hw, 0.0, hw):
                row, col = self.map.to_cell(self.x + c * ahead - s * side, self.y + s * ahead + c * side)
                if self.map.inside(row, col) and layer[row, col]:
                    return True
        return False

    def _record_motion(self, now, blocked, forward):
        self.motion.append((now, blocked, forward, self.x, self.y))
        while self.motion and now - self.motion[0][0] > max(self.p['blocked_window'], self.p['stuck_window']):
            self.motion.popleft()

    def _recovery_needed(self, now):
        """Decide on time windows, not on consecutive ticks."""
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
        if np.ptp(np.unwrap([t[1] for t in self.turns])) > p['turn_stall_angle']:
            return v, w
        self._start_escape(now, f'turn stuck at {math.degrees(self.yaw):.0f} deg')
        return v, w

    def _start_escape(self, now, reason):
        """Walk (forwards / backwards and sideways) away from the walls and remembered
        obstacles around the body; if the last escape from the same spot did not
        help, turn the direction by 90, -90, then 180 deg."""
        p = self.p
        if self.escape is not None:
            return
        occ = self.map.occupied | self._obstacle_layer(now)
        r = int(0.7 / self.map.resolution)
        row, col = self.map.to_cell(self.x, self.y)
        ra, ca = max(row - r, 0), max(col - r, 0)
        rows, cols = np.nonzero(occ[ra:row + r + 1, ca:col + r + 1])
        pts = [self.latest_obstacles] if len(self.latest_obstacles) else []
        if len(rows):
            wx, wy = self.map.to_world(rows + ra, cols + ca)
            dx, dy = wx - self.x, wy - self.y
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            pts.append(np.column_stack([c * dx + s * dy, -s * dx + c * dy]))
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
        self.escape = (now + p['escape_time'], p['escape_speed'] * math.cos(ang), p['escape_speed'] * math.sin(ang))
        self.last_escape = (now, self.x, self.y, attempt)
        self.get_logger().info(f'recovery ({reason}): walking {math.degrees(wrap_angle(ang)):.0f} deg '
                               f'off the heading for {p["escape_time"]:.1f} s (attempt {attempt + 1})')

    # ------------------------------------------------------------------ states
    def _goto(self, now):
        v_info = self.victims[self.current]
        if math.hypot(v_info['x'] - self.x, v_info['y'] - self.y) <= self.p['search_radius']:
            self._set_state(SEARCH)
            return 0.0, 0.0
        if now - self.state_since > self.p['goto_timeout']:
            if v_info.get('postponed'):
                return self._give_up(now, 'could not get there')
            # try the others first, come back at the end
            v_info['postponed'] = True
            self.order.remove(self.current)
            self.order.append(self.current)
            self._event(f'POSTPONE {v_info["name"]}: no way there after {self.p["goto_timeout"]:.0f} s')
            self._next_victim()
            return 0.0, 0.0
        v, w, _ = self._follow(now, v_info['x'], v_info['y'])
        return v, w

    def _confirmed(self):
        if len(self.hits) < self.p['confirm_hits']:
            return False
        return float(np.median([h[2] for h in self.hits])) >= self.p['confirm_min_score']

    def _search(self, now):
        v_info = self.victims[self.current]
        if self._confirmed():
            xs, ys = zip(*[(h[0], h[1]) for h in self.hits])
            self.victim_fix = (float(np.median(xs)), float(np.median(ys)))
            self._event(f'CONFIRMED {v_info["name"]} at ({self.victim_fix[0]:.2f}, {self.victim_fix[1]:.2f}) '
                        f'({len(self.hits)} YOLO hits)')
            self.path = None
            self._set_state(APPROACH)
            return 0.0, 0.0
        if now - self.state_since > self.p['victim_timeout']:
            return self._give_up(now, 'not confirmed by YOLO')
        # Keep walking towards the estimate; once there, turn on the spot to look around.
        v, w, remaining = self._follow(now, v_info['x'], v_info['y'], v_max=self.p['min_forward_speed'])
        if remaining < 0.4 or (self.path and len(self.path) < 3):
            return 0.0, 0.4
        return v, w

    def _approach(self, now):
        v_info = self.victims[self.current]
        if self.hits:
            recent = self.hits[-10:]
            self.victim_fix = (float(np.median([h[0] for h in recent])), float(np.median([h[1] for h in recent])))
        vx, vy = self.victim_fix
        measured = self._victim_distance(vx, vy)
        if measured is not None and measured <= self.p['sit_distance']:
            v_info['status'] = 'found'
            v_info['fix'] = (vx, vy)
            self._event(f'SIT {v_info["name"]} measured_distance={measured:.2f} m '
                        f'robot=({self.x:.2f}, {self.y:.2f}) t={now - self.mission_start:.1f} s')
            self._publish_body_pose(self.p['sit_height_offset'], self.p['sit_pitch'])
            self.sit_until = now + self.p['sit_duration']
            self._publish_markers()
            self._set_state(SIT)
            return 0.0, 0.0
        if now - self.state_since > self.p['victim_timeout']:
            return self._give_up(now, 'could not get within reach')
        if math.hypot(vx - self.x, vy - self.y) < 0.6:
            # Right next to the fix but the lidar does not see it closer: face it.
            err = wrap_angle(math.atan2(vy - self.y, vx - self.x) - self.yaw)
            return 0.0, max(-0.4, min(0.4, 1.5 * err))
        v, w, _ = self._follow(now, vx, vy, v_max=self.p['min_forward_speed'])
        return v, w

    def _give_up(self, now, reason):
        v_info = self.victims[self.current]
        v_info['status'] = 'failed'
        self._event(f'GIVE_UP {v_info["name"]}: {reason}')
        self._next_victim()
        return 0.0, 0.0

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
        self._next_victim()
        return 0.0, 0.0

    # ================================================================= outputs
    def _set_detector(self, on):
        if on != self.detector_on or int(self._now()) % 2 == 0:
            self.detector_pub.publish(Bool(data=on))
            self.detector_on = on

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

    def _publish_markers(self):
        markers = MarkerArray()
        colours = {'pending': (0.9, 0.6, 0.1), 'found': (0.1, 0.9, 0.2), 'failed': (0.9, 0.1, 0.1)}
        for i, v in enumerate(self.victims):
            m = Marker()
            m.header.frame_id = 'world'
            m.ns, m.id, m.type, m.action = 'victims', i, Marker.CYLINDER, Marker.ADD
            x, y = v.get('fix', (v['x'], v['y']))
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.4
            m.scale.z = 0.4
            m.color.r, m.color.g, m.color.b = colours[v['status']]
            m.color.a = 0.8
            markers.markers.append(m)
        self.marker_pub.publish(markers)

    def _publish_status(self):
        elapsed = 0.0 if self.mission_start is None else self._now() - self.mission_start
        target = self.victims[self.current]['name'] if self.current is not None else '-'
        found = sum(v['status'] == 'found' for v in self.victims)
        self.status_pub.publish(String(data=(
            f'state={self.state} target={target} found={found}/{len(self.victims)} '
            f't={elapsed:.0f}/{self.p["time_limit"]:.0f}s pose=({self.x:.2f}, {self.y:.2f}, '
            f'{math.degrees(self.yaw):.0f}deg)')))
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = self.x, self.y
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.pose_pub.publish(ps)
        self._publish_markers()

    def _debug_log(self, now, v, w):
        front = corridor_clearance(self.latest_obstacles, np.array([0.0]), self.p['robot_half_width'],
                                   self.p['max_range'])[0]
        extra = ''
        if self.state == APPROACH and self.victim_fix:
            m = self._victim_distance(*self.victim_fix)
            extra = f' fix=({self.victim_fix[0]:.2f},{self.victim_fix[1]:.2f}) victim_dist={m if m is None else round(m, 2)}'
        self.get_logger().info(
            f'[debug] {self.state} pose=({self.x:.2f},{self.y:.2f},{math.degrees(self.yaw):.0f}deg) '
            f'front={front:.2f} hits={len(self.hits)} cmd=({v:.2f},{w:.2f}){extra}',
            throttle_duration_sec=2.0)

    def stop(self):
        try:
            self._publish_cmd(0.0, 0.0)
            self._publish_body_pose(0.0, 0.0)
            self.detector_pub.publish(Bool(data=False))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SarNavigator()
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
