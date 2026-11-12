"""Reactive exploration for the Go2: find every victim in the disaster field.

Task: a victim counts as found when the Go2 gets within `success_distance`
(1 m) of it and sits down next to it.

Sensors used: IMU (heading, body tilt), 3D lidar (obstacles, range to targets)
and the RGB camera through /sar/person_detections (YOLO). There is no map and
no global planner; the robot reacts to what it currently senses:

  EXPLORE   pick the free heading with the best clearance / novelty score
  SCAN      spin 360 deg in place so the narrow camera sees the surroundings
  APPROACH  walk towards a new person detection until it is within reach
  SIT       sit down next to the victim (= victim found), then STAND_UP
  RECOVER   turn (and back up if needed) when every heading is blocked
  DONE      all victims found (or mission time over): stand still

Position = IMU heading + commanded velocity, corrected by lidar ICP. It is
used to remember visited places, to keep the robot inside the field and to
avoid counting the same person twice.
"""
import math
from collections import defaultdict, deque

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Imu, PointCloud2
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from go2_sar_core.pointcloud_utils import (cloud_to_xyz, corridor_clearance, icp_translation,
                                      rotation_rpy, voxel_downsample_2d, wrap_angle,
                                      yaw_from_quaternion)

WAIT, EXPLORE, SCAN, APPROACH, SIT, STAND_UP, RECOVER, DONE = (
    'WAIT', 'EXPLORE', 'SCAN', 'APPROACH', 'SIT', 'STAND_UP', 'RECOVER', 'DONE')


class ReactiveExplorer(Node):
    def __init__(self):
        super().__init__('sar_scout')
        p = self._declare_parameters()
        self.p = p

        # Pose estimate in the world frame, starting at the spawn pose.
        self.x, self.y, self.yaw = p['initial_x'], p['initial_y'], p['initial_yaw']
        bounds = p['field_bounds']
        self.field = bounds if len(bounds) == 4 and bounds[0] < bounds[1] and bounds[2] < bounds[3] \
            else None
        self.roll = self.pitch = 0.0
        self.tilt_history = deque(maxlen=200)  # (stamp, roll, pitch), IMU at ~100 Hz
        self.imu_yaw0 = None
        self.imu_yaw = None
        self.pose_history = deque(maxlen=200)
        self.odom_pose = None

        # Perception state.
        self.lidar_R = rotation_rpy(*p['lidar_rpy'])
        self.lidar_t = np.asarray(p['lidar_xyz'])
        self.ground_z = p['ground_z_init']
        self.obstacle_memory = deque()  # (stamp, (N, 2) points in odom frame)
        self.target_points = np.zeros((0, 3))  # base frame, for ranging people
        self.latest_scan = np.zeros((0, 2))    # base frame obstacles of the last scan only
        self.have_cloud = False
        self.headings = np.linspace(-math.pi, math.pi, p['num_headings'], endpoint=False)
        # Camera model: from camera_info when the topic is given (the real D435i
        # publishes its own calibration and streams at whatever resolution the
        # driver was started with), else from hfov + image size (simulation).
        # Hard-coding it was a real-robot defect: the organizers' camera launch
        # streams 1280x720 while these defaults describe a 640x480 sim camera,
        # which doubles every bearing and ruins the ground-plane range.
        self.fx = (p['image_width'] / 2.0) / math.tan(p['camera_hfov'] / 2.0)
        self.cx0, self.cy0 = p['image_width'] / 2.0, p['image_height'] / 2.0
        self.have_camera_info = False

        # Mission state.
        self.state = WAIT
        self.state_since = None
        self.visits = defaultdict(float)
        self.victims = []    # dicts: x, y, hits (found = reached and sat down)
        self.rejected = []   # (x, y, until): ignore detections there until sim time `until`
        self.sit_until = None
        self.goal_heading = None
        self.last_replan = -1e9
        self.dist_since_scan = 0.0
        self.last_scan_time = None
        self.scan_turned = 0.0
        self.scan_last_yaw = 0.0
        self.target = None   # current approach target
        self.blocked_time = 0.0
        self.backup_until = None
        self.cmd_v = self.cmd_w = 0.0
        self.last_tick = None
        self.start_time = None

        self.create_subscription(PointCloud2, p['cloud_topic'], self._on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, p['imu_topic'], self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, p['detections_topic'], self._on_detections, 10)
        if p['camera_info_topic']:
            self.create_subscription(CameraInfo, p['camera_info_topic'], self._on_camera_info, 10)
        if p['pose_source'] == 'odom':
            self.create_subscription(Odometry, p['odom_topic'], self._on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, p['cmd_vel_topic'], 10)
        self.body_pose_pub = self.create_publisher(Pose, p['body_pose_topic'], 10)
        self.victims_pub = self.create_publisher(PoseArray, '/sar/victims', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/sar/markers', 10)
        self.status_pub = self.create_publisher(String, '/sar/status', 10)
        self.create_timer(1.0 / p['control_rate_hz'], self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f'reactive explorer ready, searching for {p["target_victims"]} people '
            f'from ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.0f} deg), '
            f'field {self.field if self.field else "unbounded"}, pose source: {p["pose_source"]}')

    def _declare_parameters(self):
        defaults = {
            'cloud_topic': '/velodyne_points/points',
            'imu_topic': '/imu/data',
            'detections_topic': '/sar/person_detections',
            'cmd_vel_topic': '/cmd_vel',
            'body_pose_topic': '/body_pose',   # CHAMP body pose (used to sit down)
            'pose_source': 'dead_reckoning',   # or 'odom' (uses odom_topic)
            'odom_topic': '/odom',             # CHAMP leg odometry + EKF
            'velocity_scale': 0.6,
            'scan_matching': True,             # correct the position with lidar ICP
            'scan_matching_min_inliers': 0.5,
            'scan_matching_max_correction': 0.3,
            # Spawn pose in the world frame (must match the Gazebo spawn).
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            # [x_min, x_max, y_min, y_max] in the world frame: the robot never plans
            # beyond it (virtual walls). [0, 0, 0, 0] = no limit.
            'field_bounds': [0.0, 0.0, 0.0, 0.0],
            'target_victims': 4,
            'max_mission_time': 1500.0,
            'start_after_sim_time': 40.0,
            'startup_delay': 2.0,
            'control_rate_hz': 10.0,
            'debug': False,                    # log internal decisions every 2 s
            # Sensor extrinsics in base_link (from the Go2 URDF).
            'lidar_xyz': [0.2, 0.0, 0.1177],
            'lidar_rpy': [0.0, 0.0, 0.0],
            'camera_xyz': [0.326, 0.0, 0.043],
            'camera_hfov': 1.367,
            # '' = use camera_hfov + image_width/height (simulation).
            # Set to the real camera's info topic on the robot.
            'camera_info_topic': '',
            'image_width': 640,
            'image_height': 480,
            # Obstacle extraction (heights relative to the estimated ground).
            'ground_z_init': -0.21,            # measured: ground ~0.21 m below base_link
            'ground_z_min': -0.32,
            'ground_z_max': -0.10,             # body is lower while sitting
            'obstacle_min_height': 0.07,       # at the robot; grows with range:
            'tilt_tolerance': 3.0,             # + r * tan(3 deg) for attitude errors
            'obstacle_max_height': 0.75,
            'obstacle_memory_time': 3.0,
            'max_range': 5.0,
            # Motion.
            'robot_half_width': 0.28,
            'stop_distance': 0.65,
            'slow_distance': 1.4,
            'min_free_distance': 1.1,
            'max_linear_speed': 0.25,
            'approach_linear_speed': 0.22,
            'min_forward_speed': 0.22,         # CHAMP dead zone: slower = walking on the spot
            'max_angular_speed': 0.5,
            'heading_gain': 1.2,
            'num_headings': 48,
            'replan_period': 1.0,
            # Exploration scoring.
            'visit_cell_size': 0.5,
            'weight_clearance': 1.0,
            'weight_turn': 0.35,
            'weight_visited': 1.2,             # reward for heading to unvisited places
            'weight_keep_heading': 0.25,
            'scan_every_distance': 4.0,
            'scan_every_time': 60.0,
            'scan_angular_speed': 0.45,
            # People.
            'detection_min_score': 0.35,
            'dedup_radius': 1.0,
            'association_radius': 1.5,
            'min_hits': 4,
            'confirm_min_score': 0.45,         # median YOLO score of the hits
            'approach_timeout': 40.0,
            'lost_timeout': 8.0,
            'retry_after': 60.0,               # re-try an unreached detection after [s]
            # Success: within success_distance of the victim, then sit down.
            'success_distance': 1.0,
            'sit_distance': 0.8,               # approach goal (margin below 1 m)
            'sit_duration': 5.0,
            'stand_up_duration': 2.0,
            'sit_height_offset': -0.08,        # body height change [m] (nominal 0.225)
            'sit_pitch': -0.3,                 # nose up [rad] -> hips down
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        return {name: self.get_parameter(name).value for name in defaults}

    # ------------------------------------------------------------------ sensors
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_imu(self, msg):
        q = msg.orientation
        yaw = yaw_from_quaternion(q)
        if self.imu_yaw0 is None:
            self.imu_yaw0 = yaw
        self.imu_yaw = wrap_angle(yaw - self.imu_yaw0 + self.p['initial_yaw'])
        self.roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        self.pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.tilt_history.append((stamp, self.roll, self.pitch))

    def _tilt_at(self, stamp):
        """Roll and pitch when the scan was taken: the body rocks by several
        degrees while trotting, and a stale attitude lifts the far ground into
        the obstacle band."""
        if not self.tilt_history:
            return self.roll, self.pitch
        times = np.fromiter((h[0] for h in self.tilt_history), float, len(self.tilt_history))
        return self.tilt_history[int(np.abs(times - stamp).argmin())][1:]

    def _on_odom(self, msg):
        pos = msg.pose.pose.position
        x0, y0, yaw0 = self.p['initial_x'], self.p['initial_y'], self.p['initial_yaw']
        c, s = math.cos(yaw0), math.sin(yaw0)
        self.odom_pose = (x0 + c * pos.x - s * pos.y, y0 + s * pos.x + c * pos.y)

    def _on_cloud(self, msg):
        pts = cloud_to_xyz(msg)
        if len(pts) == 0:
            return
        # Lidar frame -> base_link -> gravity-levelled base frame (IMU roll/pitch
        # at the scan time).
        pts = pts @ self.lidar_R.T + self.lidar_t
        roll, pitch = self._tilt_at(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        pts = pts @ rotation_rpy(roll, pitch, 0.0).T
        r = np.hypot(pts[:, 0], pts[:, 1])

        # Ground height: a low percentile of the returns near the current estimate.
        # (A median would creep up walls in narrow corridors.) Clamped to the
        # physically possible range of the body height.
        ground = pts[(r > 0.8) & (r < 4.0) & (np.abs(pts[:, 2] - self.ground_z) < 0.10), 2]
        if len(ground) > 30:
            measured = float(np.percentile(ground, 20))
            self.ground_z = 0.9 * self.ground_z + 0.1 * measured
            self.ground_z = min(max(self.ground_z, self.p['ground_z_min']), self.p['ground_z_max'])

        height = pts[:, 2] - self.ground_z
        in_range = r < self.p['max_range']
        # The height threshold grows with range to tolerate a residual attitude
        # error of `tilt_tolerance` degrees (ground at range r appears r*tan(e) high).
        min_height = self.p['obstacle_min_height'] + r * math.tan(math.radians(self.p['tilt_tolerance']))
        obstacle = in_range & (height > min_height) & (height < self.p['obstacle_max_height'])
        self.target_points = pts[in_range & (height > 0.03) & (height < 1.2)]

        obst = voxel_downsample_2d(pts[obstacle, :2], 0.05)
        self.latest_scan = obst
        if self.p['scan_matching']:
            self._correct_pose_with_scan(obst)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        odom_pts = np.column_stack([self.x + c * obst[:, 0] - s * obst[:, 1],
                                    self.y + s * obst[:, 0] + c * obst[:, 1]])
        now = self._now()
        self.obstacle_memory.append((now, odom_pts))
        while self.obstacle_memory and now - self.obstacle_memory[0][0] > self.p['obstacle_memory_time']:
            self.obstacle_memory.popleft()
        self.have_cloud = True

    def _correct_pose_with_scan(self, scan_base):
        """Lidar odometry: align the new scan to the recent obstacle memory.

        The heading comes from the IMU, so only the translation is estimated.
        The dead-reckoned position is the initial guess.
        """
        if len(self.obstacle_memory) < 2 or len(scan_base) < 30:
            return
        reference = voxel_downsample_2d(np.concatenate([m[1] for m in self.obstacle_memory]), 0.1)
        source = voxel_downsample_2d(scan_base, 0.1)
        rng = np.random.default_rng()
        if len(source) > 400:
            source = source[rng.choice(len(source), 400, replace=False)]
        if len(reference) > 1500:
            reference = reference[rng.choice(len(reference), 1500, replace=False)]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        source_odom = np.column_stack([self.x + c * source[:, 0] - s * source[:, 1],
                                       self.y + s * source[:, 0] + c * source[:, 1]])
        result = icp_translation(source_odom, reference)
        if result is None:
            return
        dx, dy, ratio = result
        if ratio >= self.p['scan_matching_min_inliers'] and \
                math.hypot(dx, dy) <= self.p['scan_matching_max_correction']:
            self.x += dx
            self.y += dy

    def _obstacles_in_base(self):
        if not self.obstacle_memory:
            return np.zeros((0, 2))
        pts = np.concatenate([m[1] for m in self.obstacle_memory]) - (self.x, self.y)
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        return np.column_stack([c * pts[:, 0] - s * pts[:, 1], s * pts[:, 0] + c * pts[:, 1]])

    def _pose_at(self, stamp):
        if not self.pose_history:
            return self.x, self.y, self.yaw
        times = np.array([h[0] for h in self.pose_history])
        return self.pose_history[int(np.abs(times - stamp).argmin())][1:]

    # --------------------------------------------------------------- detections
    def _on_detections(self, msg):
        if self.state in (WAIT, SIT, STAND_UP, DONE):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        px, py, pyaw = self._pose_at(stamp)
        candidates = []
        for det in msg.detections:
            if not det.results or det.results[0].hypothesis.score < self.p['detection_min_score']:
                continue
            located = self._locate_person(det, px, py, pyaw)
            if located is not None:
                candidates.append((det.results[0].hypothesis.score, *located))
        now = self._now()
        active_rejected = [(x, y) for x, y, until in self.rejected if now < until]
        for score, wx, wy, rng in sorted(candidates, reverse=True):
            if self._near(wx, wy, [(v['x'], v['y']) for v in self.victims]) or \
                    self._near(wx, wy, active_rejected):
                continue
            if self.state != APPROACH:
                self.target = {'est': [(wx, wy, rng, score)], 'start': now, 'last_hit': now}
                self.blocked_time = 0.0
                self._set_state(APPROACH)
                self.get_logger().info(
                    f'person detected (score {score:.2f}) at ~({wx:.1f}, {wy:.1f}), '
                    f'range {rng:.1f} m -> approaching')
                return
            tx, ty = self._target_xy()
            if math.hypot(wx - tx, wy - ty) < self.p['association_radius']:
                self.target['est'].append((wx, wy, rng, score))
                self.target['last_hit'] = now
                self.get_logger().info(
                    f'  hit {len(self.target["est"])}: score {score:.2f}, '
                    f'at ({wx:.2f}, {wy:.2f}), range {rng:.1f} m')
                return

    def _on_camera_info(self, msg):
        if msg.k[0] > 0:
            self.fx, self.cx0, self.cy0 = msg.k[0], msg.k[2], msg.k[5]
            if not self.have_camera_info:
                self.have_camera_info = True
                self.get_logger().info(
                    f'camera model from camera_info: fx={self.fx:.1f} '
                    f'cx={self.cx0:.1f} cy={self.cy0:.1f} ({msg.width}x{msg.height})')

    def _locate_person(self, det, px, py, pyaw):
        """Bearing from the camera, range from the lidar (or the ground plane)."""
        p = self.p
        cx0, cy0 = self.cx0, self.cy0
        u, half_w = det.bbox.center.position.x, det.bbox.size_x / 2.0
        bearing = -math.atan((u - cx0) / self.fx)
        lo = -math.atan((u + 0.5 * half_w - cx0) / self.fx)
        hi = -math.atan((u - 0.5 * half_w - cx0) / self.fx)
        cam_x, cam_y, cam_z = p['camera_xyz']

        d_lidar = None
        pts = self.target_points
        if len(pts):
            rx, ry = pts[:, 0] - cam_x, pts[:, 1] - cam_y
            ang = np.arctan2(ry, rx)
            sel = (rx > 0.2) & (ang >= lo - 0.02) & (ang <= hi + 0.02)
            if sel.sum() >= 3:
                d_lidar = float(np.percentile(np.hypot(rx[sel], ry[sel]), 25))

        d_ground = None
        v_bottom = det.bbox.center.position.y + det.bbox.size_y / 2.0
        alpha = math.atan((v_bottom - cy0) / self.fx)  # square pixels: fy == fx
        if alpha > 0.03:
            d_ground = (cam_z - self.ground_z) / math.tan(alpha)

        if d_lidar is not None and d_ground is not None and d_lidar > 1.5 * d_ground + 0.3:
            d_lidar = None  # beams went over a low body and hit something behind it
        rng = d_lidar if d_lidar is not None else d_ground
        if rng is None or rng > 8.0:
            return None
        bx = cam_x + rng * math.cos(bearing)
        by = cam_y + rng * math.sin(bearing)
        c, s = math.cos(pyaw), math.sin(pyaw)
        return px + c * bx - s * by, py + s * bx + c * by, rng

    def _near(self, x, y, points, radius=None):
        radius = self.p['dedup_radius'] if radius is None else radius
        return any(math.hypot(x - qx, y - qy) < radius for qx, qy in points)

    def _target_xy(self):
        est = np.array(self.target['est'])
        close = est[est[:, 2] < 3.5]
        use = close if len(close) else est
        return float(np.median(use[:, 0])), float(np.median(use[:, 1]))

    # ------------------------------------------------------------------ control
    def _set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'state {self.state} -> {state}')
        self.state = state
        self.state_since = self._now()

    def _tick(self):
        now = self._now()
        if self.last_tick is None:
            self.last_tick = now
            return
        dt = min(max(now - self.last_tick, 0.0), 0.5)
        self.last_tick = now
        self._update_pose(dt, now)

        if self.state == WAIT:
            # The CHAMP controllers come up ~30 s (sim time) after Gazebo starts.
            if self.imu_yaw is None or not self.have_cloud or now < self.p['start_after_sim_time']:
                return
            if self.start_time is None:
                self.start_time = now
            if now - self.start_time >= self.p['startup_delay']:
                self.last_scan_time = now
                self._start_scan()
            return

        if self.state != DONE and now - self.start_time > self.p['max_mission_time']:
            self.get_logger().warn('mission time is over')
            self._finish()

        obstacles = self._obstacles_in_base()
        clearance = corridor_clearance(obstacles, self.headings, self.p['robot_half_width'],
                                       self.p['max_range'])
        if self.field:
            clearance = np.minimum(clearance, self._field_clearance())
        self.visits[self._cell(self.x, self.y)] += dt

        handler = {EXPLORE: self._explore, SCAN: self._scan, APPROACH: self._approach,
                   SIT: self._sit, STAND_UP: self._stand_up, RECOVER: self._recover,
                   DONE: lambda *_: (0.0, 0.0)}[self.state]
        v, w = handler(now, dt, clearance)
        self._publish_cmd(v, w)
        if self.p['debug']:
            best = int(clearance.argmax())
            target = ''
            if self.target is not None:
                tx, ty = self._target_xy()
                target = (f' target=({tx:.2f},{ty:.2f}) dist={math.hypot(tx - self.x, ty - self.y):.2f}'
                          f' err={math.degrees(wrap_angle(math.atan2(ty - self.y, tx - self.x) - self.yaw)):.0f}deg')
            self.get_logger().info(
                f'[debug] {self.state} pose=({self.x:.2f},{self.y:.2f},{math.degrees(self.yaw):.0f}deg) '
                f'front={self._clearance_at(clearance, 0.0):.2f} free={int((clearance >= self.p["min_free_distance"]).sum())}'
                f'/{len(clearance)} best={math.degrees(self.headings[best]):.0f}deg:{clearance[best]:.2f} '
                f'ground_z={self.ground_z:.3f} cmd=({v:.2f},{w:.2f}){target}',
                throttle_duration_sec=2.0)

    def _update_pose(self, dt, now):
        if self.imu_yaw is not None:
            self.yaw = self.imu_yaw
        if self.p['pose_source'] == 'odom' and self.odom_pose is not None:
            self.x, self.y = self.odom_pose
        else:
            dist = self.cmd_v * self.p['velocity_scale'] * dt
            self.x += dist * math.cos(self.yaw)
            self.y += dist * math.sin(self.yaw)
        self.dist_since_scan += abs(self.cmd_v) * dt
        self.pose_history.append((now, self.x, self.y, self.yaw))

    def _field_clearance(self):
        """Distance along each heading until the robot centre would leave the field."""
        x_min, x_max, y_min, y_max = self.field
        out = np.full(len(self.headings), self.p['max_range'])
        for i, rel in enumerate(self.headings):
            dx, dy = math.cos(self.yaw + rel), math.sin(self.yaw + rel)
            for d, lo, hi, pos in ((dx, x_min, x_max, self.x), (dy, y_min, y_max, self.y)):
                if d > 1e-6:
                    out[i] = min(out[i], (hi - pos) / d)
                elif d < -1e-6:
                    out[i] = min(out[i], (lo - pos) / d)
        return np.maximum(out, 0.0)

    def _clearance_at(self, clearance, rel_heading):
        diff = (self.headings - rel_heading + math.pi) % (2.0 * math.pi) - math.pi
        return clearance[int(np.abs(diff).argmin())]

    def _drive(self, heading_error, clearance, v_max):
        p = self.p
        w = max(-p['max_angular_speed'], min(p['max_angular_speed'], p['heading_gain'] * heading_error))
        if abs(heading_error) > 0.6:
            return 0.0, w
        front = self._clearance_at(clearance, 0.0)
        scale = (front - p['stop_distance']) / (p['slow_distance'] - p['stop_distance'])
        v = v_max * max(0.0, min(1.0, scale)) * math.cos(heading_error)
        # CHAMP only marks time below ~0.15 m/s forward: never command a slow walk.
        if v > 0.02:
            v = max(v, p['min_forward_speed'])
        return v, w

    def _explore(self, now, dt, clearance):
        p = self.p
        if self.dist_since_scan > p['scan_every_distance'] or \
                now - self.last_scan_time > p['scan_every_time']:
            self._start_scan()
            return 0.0, 0.0

        goal_blocked = self.goal_heading is None or \
            self._clearance_at(clearance, wrap_angle(self.goal_heading - self.yaw)) < p['stop_distance']
        if goal_blocked or now - self.last_replan > p['replan_period']:
            if not self._choose_heading(clearance):
                self._set_state(RECOVER)
                return 0.0, 0.0
            self.last_replan = now

        error = wrap_angle(self.goal_heading - self.yaw)
        return self._drive(error, clearance, p['max_linear_speed'])

    def _choose_heading(self, clearance):
        p = self.p
        free = clearance >= p['min_free_distance']
        if not free.any():
            return False
        scores = np.full(len(self.headings), -np.inf)
        for i in np.where(free)[0]:
            rel = self.headings[i]
            absolute = self.yaw + rel
            # Novelty of the farthest reachable point in this direction (and the
            # cells around it): open headings that lead to unvisited places win.
            reach = min(clearance[i] - p['stop_distance'], 3.0)
            ex, ey = self.x + reach * math.cos(absolute), self.y + reach * math.sin(absolute)
            visited = self._visited_level(ex, ey)
            score = (p['weight_clearance'] * min(clearance[i], 4.0) / 4.0
                     + p['weight_visited'] * (1.0 - visited)
                     - p['weight_turn'] * abs(rel) / math.pi)
            if self.goal_heading is not None and \
                    abs(wrap_angle(absolute - self.goal_heading)) < 0.3:
                score += p['weight_keep_heading']
            scores[i] = score + np.random.uniform(0.0, 0.05)
        self.goal_heading = wrap_angle(self.yaw + self.headings[int(scores.argmax())])
        return True

    def _start_scan(self):
        self.scan_turned = 0.0
        self.scan_last_yaw = self.yaw
        self._set_state(SCAN)

    def _scan(self, now, dt, clearance):
        # Count the rotation actually measured by the IMU, not the commanded one.
        self.scan_turned += abs(wrap_angle(self.yaw - self.scan_last_yaw))
        self.scan_last_yaw = self.yaw
        if self.scan_turned >= 2.0 * math.pi:
            self.dist_since_scan = 0.0
            self.last_scan_time = now
            self.goal_heading = None
            self._set_state(EXPLORE)
            return 0.0, 0.0
        return 0.0, self.p['scan_angular_speed']

    def _enough_evidence(self):
        """Enough consistent and confident detections to call it a person."""
        scores = [e[3] for e in self.target['est']]
        return len(scores) >= self.p['min_hits'] and \
            float(np.median(scores)) >= self.p['confirm_min_score']

    def _approach(self, now, dt, clearance):
        p = self.p
        tx, ty = self._target_xy()
        # Distance from the robot centre to the victim. Close up, a low body falls
        # into the lidar blind zone and the estimate drifts to whatever is behind
        # it; when the robot is facing the target, the nearest obstacle straight
        # ahead *is* the victim, so the front clearance is used as well.
        # Only the latest scan is used here: the 3 s obstacle memory can hold stale
        # points, and a prone victim (0.3 m high) is always visible to the lidar.
        error = wrap_angle(math.atan2(ty - self.y, tx - self.x) - self.yaw)
        dist = math.hypot(tx - self.x, ty - self.y)
        if abs(error) < math.radians(20) and dist < p['success_distance'] + 0.6:
            front_now = corridor_clearance(self.latest_scan, np.array([0.0]),
                                           p['robot_half_width'], p['max_range'])[0]
            dist = min(dist, front_now)

        if dist <= p['sit_distance'] and self._enough_evidence():
            return self._reach_victim(tx, ty, dist)
        if now - self.target['start'] > p['approach_timeout'] or \
                now - self.target['last_hit'] > p['lost_timeout']:
            return self._give_up_target(now, 'timeout' if self._enough_evidence() else 'not confirmed')

        v, w = self._drive(error, clearance, p['approach_linear_speed'])
        if dist <= p['sit_distance']:
            v = 0.0  # close enough, keep facing it until enough detections arrive
        elif v < 0.02 and abs(error) < 0.5:
            self.blocked_time += dt
            if self.blocked_time > 4.0:
                # A victim lying against something can still be within reach.
                if dist <= p['success_distance'] and self._enough_evidence():
                    return self._reach_victim(tx, ty, dist)
                return self._give_up_target(now, 'path blocked')
        return v, w

    def _evidence_text(self):
        scores = [e[3] for e in self.target['est']]
        return f'{len(scores)} hits, median score {float(np.median(scores)):.2f}'

    def _reach_victim(self, tx, ty, dist):
        """Within reach of a confirmed victim: record it and sit down."""
        self.victims.append({'x': tx, 'y': ty, 'hits': len(self.target['est'])})
        self.get_logger().info(
            f'VICTIM #{len(self.victims)} found at ({tx:.2f}, {ty:.2f}), robot {dist:.2f} m away '
            f'({self._evidence_text()}) [{len(self.victims)}/{self.p["target_victims"]}] -> sitting down')
        self._mark_visited(tx, ty, radius=1.0)
        self.target = None
        self._publish_markers()
        self._publish_body_pose(self.p['sit_height_offset'], self.p['sit_pitch'])
        self.sit_until = self._now() + self.p['sit_duration']
        self._set_state(SIT)
        return 0.0, 0.0

    def _give_up_target(self, now, reason):
        tx, ty = self._target_xy()
        self.rejected.append((tx, ty, now + self.p['retry_after']))
        self.get_logger().info(
            f'target at ({tx:.2f}, {ty:.2f}) dropped: {reason} ({self._evidence_text()})')
        self.target = None
        self.goal_heading = None
        self._set_state(EXPLORE)
        return 0.0, 0.0

    def _sit(self, now, dt, clearance):
        if now < self.sit_until:
            return 0.0, 0.0
        self._publish_body_pose(0.0, 0.0)
        self.sit_until = now + self.p['stand_up_duration']
        self._set_state(STAND_UP)
        return 0.0, 0.0

    def _stand_up(self, now, dt, clearance):
        if now < self.sit_until:
            return 0.0, 0.0
        if len(self.victims) >= self.p['target_victims']:
            self._finish()
        else:
            self.goal_heading = None
            self._set_state(EXPLORE)
        return 0.0, 0.0

    def _recover(self, now, dt, clearance):
        p = self.p
        if self.backup_until is not None:
            if now < self.backup_until and self._clearance_at(clearance, math.pi) > 0.7:
                return -0.1, 0.0
            self.backup_until = None
            self.state_since = now
        if self._clearance_at(clearance, 0.0) >= p['min_free_distance']:
            self.goal_heading = self.yaw
            self._set_state(EXPLORE)
            return 0.0, 0.0
        if now - self.state_since > 12.0:
            self.backup_until = now + 2.0
            return 0.0, 0.0
        best = self.headings[int(clearance.argmax())]
        return 0.0, math.copysign(0.8 * p['max_angular_speed'], best if best != 0.0 else 1.0)

    def _finish(self):
        self._set_state(DONE)
        lines = [f'  #{i + 1}: ({v["x"]:.2f}, {v["y"]:.2f})' for i, v in enumerate(self.victims)]
        self.get_logger().info(
            f'search finished, {len(self.victims)} victim(s) found:\n' + '\n'.join(lines))

    # ---------------------------------------------------------------- outputs
    def _cell(self, x, y):
        size = self.p['visit_cell_size']
        return int(math.floor(x / size)), int(math.floor(y / size))

    def _visited_level(self, x, y):
        """0 = never visited, 1 = well visited (3x3 cells around (x, y))."""
        cx, cy = self._cell(x, y)
        total = sum(min(self.visits.get((cx + i, cy + j), 0.0) / 10.0, 1.0)
                    for i in (-1, 0, 1) for j in (-1, 0, 1))
        return total / 9.0

    def _mark_visited(self, x, y, radius):
        size = self.p['visit_cell_size']
        n = int(math.ceil(radius / size))
        cx, cy = self._cell(x, y)
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                self.visits[(cx + i, cy + j)] += 10.0

    def _publish_cmd(self, v, w):
        self.cmd_v, self.cmd_w = v, w
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _publish_body_pose(self, height_offset, pitch):
        """CHAMP /body_pose: position.z is added to the nominal walking height."""
        msg = Pose()
        msg.position.z = float(height_offset)
        msg.orientation.y = math.sin(pitch / 2.0)
        msg.orientation.w = math.cos(pitch / 2.0)
        self.body_pose_pub.publish(msg)

    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        poses = PoseArray()
        poses.header.frame_id = 'world'
        poses.header.stamp = stamp
        markers = MarkerArray()
        for i, v in enumerate(self.victims):
            pose = Pose()
            pose.position.x, pose.position.y = v['x'], v['y']
            pose.orientation.w = 1.0
            poses.poses.append(pose)
            for kind, mid in ((Marker.CYLINDER, 2 * i), (Marker.TEXT_VIEW_FACING, 2 * i + 1)):
                m = Marker()
                m.header = poses.header
                m.ns, m.id, m.type, m.action = 'victims', mid, kind, Marker.ADD
                m.pose = Pose()
                m.pose.position.x, m.pose.position.y = v['x'], v['y']
                m.pose.position.z = 0.3 if kind == Marker.CYLINDER else 1.0
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = 0.5
                m.scale.z = 0.6 if kind == Marker.CYLINDER else 0.35
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.2, 0.8
                m.text = f'victim {i + 1}'
                markers.markers.append(m)
        self.victims_pub.publish(poses)
        self.marker_pub.publish(markers)

    def _publish_status(self):
        msg = String()
        # Configured for a real camera but nothing has arrived: the node is
        # silently using the simulation focal length, which puts every bearing
        # and every ground-plane range wrong. Say so rather than scoring zero.
        if self.p['camera_info_topic'] and not self.have_camera_info:
            self.get_logger().warn(
                f"no camera_info on {self.p['camera_info_topic']} yet -- still using the "
                f"fallback camera model (hfov {self.p['camera_hfov']:.3f}, "
                f"{self.p['image_width']}x{self.p['image_height']}); bearings and ranges "
                f"will be wrong if the real stream is not that size",
                throttle_duration_sec=10.0)
        msg.data = (f'state={self.state} victims={len(self.victims)}/{self.p["target_victims"]} '
                    f'pose=({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.0f}deg)')
        self.status_pub.publish(msg)

    def stop(self):
        try:
            self._publish_cmd(0.0, 0.0)
            self._publish_body_pose(0.0, 0.0)
        except Exception:  # context may already be shut down after Ctrl+C
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
