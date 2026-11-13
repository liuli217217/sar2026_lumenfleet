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
        self.cmd_vy = 0.0              # lateral command (strafe), see _publish_cmd
        self.sidestep_until = None
        self.sidestep_dir = 1.0
        self.sidestep_count = 0
        self.backup_until = None
        self.cmd_v = self.cmd_w = 0.0
        self.last_tick = None
        self.start_time = None
        self.path_length = 0.0         # distance walked, drift-free (speed integral)
        self.egress_from = None        # (path_length, time) of the last sit
        self.protect_blocked = False   # nearest obstacle inside safety_distance
        self.last_travel = None        # (stamp, heading) cached travel direction
        self.protect_stops = 0         # how often the hard floor fired (reported)

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
            # How far the robot may stray past field_bounds before the virtual
            # wall actually stops it. The bounds are not a physical obstacle --
            # nothing breaks if the estimate drifts over them -- so they must
            # not be able to zero the forward clearance and strand the robot.
            # Dead reckoning drifted 1.5 m outward in a 180 s run and three
            # confirmed victims were abandoned as "path blocked" against a wall
            # that was not there.
            'field_margin': 0.6,
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
            # Close-range obstacle protection. This is a second layer on top of
            # the corridor clearance above: the corridor decides which headings
            # are *passable* (and in this cluttered arena its margins must stay
            # small or every heading is rejected and the robot freezes), while
            # these act on the single nearest obstacle point and only ever push
            # the robot away. They cannot rule a heading out, so they cannot
            # trap the robot the way a wider corridor would.
            'protect_sector': 1.57,            # obstacles outside +-90 deg of the
                                               # nose are ignored: the robot only
                                               # drives forward, and the lidar sees
                                               # the robot's own rear at ~0.31 m
            # Hard stop: only what is nearly straight ahead can be walked into.
            # A wall 0.3 m off the flank in a 0.8 m gap is not a reason to stop --
            # that is exactly the arena the robot has to cross.
            'stop_cone': 0.52,                 # +-30 deg
            'safety_distance': 0.30,           # centre-to-obstacle hard floor [m]
            # Repulsion is computed as a left/right *balance*, so a corridor with
            # walls on both sides cancels out instead of shoving the robot into
            # one of them. Kept short so it does not fire for the whole width of
            # a narrow gap.
            'repulsion_distance': 0.55,
            'repulsion_gain': 1.0,             # rad/s at full one-sided intrusion
            # Exploration scoring.
            'visit_cell_size': 0.5,
            'visit_saturation': 3.0,           # seconds in a cell that count as
                                               # "fully visited" (was hard-coded 10)
            'trail_radius': 0.45,              # mark cells within this of the path
            'weight_clearance': 1.0,
            'weight_turn': 0.35,
            'weight_visited': 1.2,             # reward for heading to unvisited places
            'weight_keep_heading': 0.25,
            'weight_backtrack': 0.8,           # penalty for turning back along the
                                               # path just travelled
            'backtrack_angle': 2.0,            # [rad] heading diff that counts as back
            'backtrack_memory': 10.0,          # [s] of path that defines "forward"
            # Sweep the body (and with it the fixed camera) while exploring.
            # The camera sees 78 deg; walking straight past a victim just
            # outside that cone is the single commonest way a body is missed.
            # Amplitude is deliberately below the 0.6 rad at which _drive stops
            # to turn on the spot, or the robot would stutter instead of walk.
            # Costs cos(26 deg) ~= 10% of forward speed.
            'sweep_amplitude': 0.45,           # [rad] +-26 deg -> ~130 deg swept
            'sweep_period': 5.0,               # [s] one full left-right cycle
            'scan_every_distance': 4.0,
            'scan_every_time': 60.0,
            'scan_angular_speed': 0.45,
            # People.
            # Cross-check every detection against the lidar. Measured
            # 2026-11-13 with the arena ground truth: a phantom at (-0.58,
            # 0.13) -- 73 deg and 1.8 m away from the nearest real body --
            # collected 72 confirming hits at median score 0.59 and cost the
            # run ~100 s (approach timeout, a second approach, a sit) in a
            # mission that only covered 10 m. The camera alone cannot tell a
            # body from a texture; the lidar can say whether anything of the
            # right size is actually there.
            # Victims are only ever lying (0.36 m) or sitting (0.72 m). Anything
            # the camera calls a person but the lidar measures as taller than
            # this is not one: a standing spectator is 1.77-1.80 m and the arena
            # wall is 1.20 m. Measured 2026-11-13: the spectators stand outside
            # the arena and are taller than the walls, so the camera sees their
            # heads and shoulders over the top while the lidar beam stops at the
            # wall -- the detection then lands on the wall, 1.1-1.5 m away, with
            # YOLO scores of 0.77-0.89 (a real victim scored 0.88). Those
            # phantoms repeatedly pulled the robot back across the arena.
            'victim_max_height': 1.00,
            # A lying or sitting body stands on the floor, so the bottom of its
            # box is below the horizon and the ground-plane range is defined.
            # A person standing behind a wall has their legs hidden: the box
            # bottom is the wall top, above the horizon, and no ground range
            # can be computed. That absence is itself the giveaway.
            'require_ground_contact': True,
            'require_lidar_support': True,
            'victim_range_agreement': 0.8,     # [m] camera vs lidar range
            'detection_min_score': 0.35,
            'dedup_radius': 1.0,
            # Suppression radius around a victim already scored. Wider than
            # dedup_radius because the same body seen from a new angle moves
            # by more than the track-association tolerance.
            'found_radius': 2.0,
            # After scoring a victim, walk away before accepting another one.
            # found_radius cannot do this job on its own: it compares the new
            # estimate with the recorded one in the SAME drifting frame, so a
            # 1.5 m drift moves both and the suppression silently lapses. Path
            # length is the one odometric quantity that does not rot with
            # heading drift -- it is the integral of commanded speed -- so it
            # is used as the primary gate, with a timeout so that a robot that
            # cannot get away is never locked out for good.
            'egress_distance': 2.5,            # [m] of walking since the sit
            'egress_time': 25.0,               # [s] hard release of the gate
            'association_radius': 1.5,
            'min_hits': 4,
            'confirm_min_score': 0.45,         # median YOLO score of the hits
            # The lidar range straight ahead is only accepted as the victim's
            # range when it agrees with the camera estimate to within this.
            # Taking min() unconditionally meant any cone or barrel in front of
            # a victim became "the victim", and the robot sat down short of it:
            # measured 2026-11-13, four sits across two code versions all landed
            # 1.3-1.4 m from the body while believing they were 0.62-0.70 m away.
            'lidar_range_agreement': 0.6,
            # When the way to a confirmed victim is blocked, step sideways and
            # try again instead of throwing the target away. The Go2 (and
            # go2_bridge, max_vy 0.3) can strafe, which Scout never used: it
            # keeps the camera on the body while moving, so evidence keeps
            # accumulating through the manoeuvre. Measured 2026-11-13: one run
            # dropped 8 targets as "path blocked", among them bodies with 35
            # and 16 confirming hits.
            'blocked_time_limit': 4.0,
            'sidestep_speed': 0.20,            # [m/s] lateral
            'sidestep_duration': 2.5,          # [s] per attempt
            'sidestep_attempts': 2,            # per target, then give up
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
            # A victim already sat on is scored and must never be approached
            # again, and the radius for that is NOT dedup_radius. Measured
            # 2026-11-13: after sitting on victim_4 the explorer re-approached
            # it from the other side, the new estimate landing 1.4 m from the
            # recorded one -- outside dedup_radius 1.0. It spent 10 s on a
            # victim worth nothing and stayed in the half of the arena it had
            # already covered.
            if self._near(wx, wy, [(v['x'], v['y']) for v in self.victims],
                          radius=self.p['found_radius']) or \
                    self._near(wx, wy, active_rejected):
                continue
            if not self._clear_of_last_victim(now):
                continue
            if self.state != APPROACH:
                self.target = {'est': [(wx, wy, rng, score)], 'start': now, 'last_hit': now}
                self.blocked_time = 0.0
                self.sidestep_until = None
                self.sidestep_count = 0
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

    def _clear_of_last_victim(self, now):
        """True once the robot has actually left the victim it last scored.

        Measured 2026-11-13: 23 s after sitting on victim_4 the explorer sat
        down again 0.87 m from the same body and the referee scored it FAIL.
        Its own estimate had the two 2.85 m apart -- outside found_radius --
        because dead reckoning had drifted 1.5 m in between. No radius in the
        drifting frame can catch that, so the test is made of quantities that
        do not drift: how far the legs have walked, and the clock.
        """
        if self.egress_from is None:
            return True
        walked = self.path_length - self.egress_from[0]
        if walked >= self.p['egress_distance'] or \
                now - self.egress_from[1] >= self.p['egress_time']:
            self.egress_from = None
            return True
        return False

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
                r_sel = np.hypot(rx[sel], ry[sel])
                d_lidar = float(np.percentile(r_sel, 25))
                # Height of the nearest cluster only: points further back are
                # the wall or whatever stands behind the body, and including
                # them would fail every victim lying in front of a wall.
                near = r_sel <= d_lidar + 0.4
                if near.sum() >= 3:
                    h = pts[sel, 2][near] - self.ground_z
                    if float(np.percentile(h, 90)) > p['victim_max_height']:
                        return None   # too tall to be a lying or sitting person

        d_ground = None
        v_bottom = det.bbox.center.position.y + det.bbox.size_y / 2.0
        alpha = math.atan((v_bottom - cy0) / self.fx)  # square pixels: fy == fx
        if alpha > 0.03:
            d_ground = (cam_z - self.ground_z) / math.tan(alpha)

        if d_lidar is not None and d_ground is not None and d_lidar > 1.5 * d_ground + 0.3:
            d_lidar = None  # beams went over a low body and hit something behind it
        if p['require_lidar_support']:
            # The camera says there is a person at this bearing. If the lidar
            # sees nothing solid there, or sees it at a quite different range,
            # the two sensors are not describing the same object and the
            # detection is not trusted. A real body 0.3-0.9 m tall is well
            # inside the lidar's view at these ranges.
            if d_lidar is None:
                return None
            if d_ground is not None and abs(d_lidar - d_ground) > p['victim_range_agreement']:
                return None
        if p['require_ground_contact'] and d_ground is None:
            return None
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
            # Softened by field_margin: at the boundary the robot still has
            # field_margin of room to manoeuvre (so it can walk along the edge
            # and reach a victim lying near it), and only once it is that far
            # past the line does the wall bring it to a stop.
            clearance = np.minimum(clearance, self._field_clearance() + self.p['field_margin'])
            clearance = np.maximum(clearance, 0.0)
        # (the visited map is aged in _mark_trail, from _update_pose)

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
                f'ground_z={self.ground_z:.3f} path={self.path_length:.2f} '
                f'LRA={self._side_ranges()} '
                f'cmd=({v:.2f},{w:.2f}){target}',
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
        self.path_length += abs(self.cmd_v) * self.p['velocity_scale'] * dt
        self.pose_history.append((now, self.x, self.y, self.yaw))
        self._mark_trail(dt)

    def _mark_trail(self, dt):
        """Age the cells the body actually swept, not only the centre cell.

        Marking one 0.5 m cell per tick left a corridor the robot had just
        walked down looking almost unvisited, so the novelty score sent it
        straight back the way it came.
        """
        size = self.p['visit_cell_size']
        n = int(math.ceil(self.p['trail_radius'] / size))
        cx, cy = self._cell(self.x, self.y)
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                # Centre cell at full rate, the ring around it at half.
                self.visits[(cx + i, cy + j)] += dt if (i == 0 and j == 0) else 0.5 * dt

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
        # Not clamped at zero: a negative value means the estimate is already
        # outside the box, and the caller adds field_margin so that straying
        # further keeps costing clearance instead of stopping dead at the edge.
        return out

    def _clearance_at(self, clearance, rel_heading):
        diff = (self.headings - rel_heading + math.pi) % (2.0 * math.pi) - math.pi
        return clearance[int(np.abs(diff).argmin())]

    def _side_ranges(self, protect_bearing=None, protect_half=0.44):
        """Nearest obstacle on the left and on the right of the nose, and ahead.

        Returns (left, right, ahead). A gap the robot is centred in gives
        left == right, so the repulsion built from their difference is zero and
        the robot walks straight through instead of being pushed into a wall.
        """
        pts = self.latest_scan
        inf = float('inf')
        if len(pts) == 0:
            return inf, inf, inf
        d = np.hypot(pts[:, 0], pts[:, 1])
        bearing = np.arctan2(pts[:, 1], pts[:, 0])
        keep = (d > 1e-3) & (np.abs(bearing) <= self.p['protect_sector'])
        if protect_bearing is not None:
            diff = (bearing - protect_bearing + math.pi) % (2.0 * math.pi) - math.pi
            keep &= np.abs(diff) > protect_half
        if not keep.any():
            return inf, inf, inf
        d, bearing = d[keep], bearing[keep]
        left = float(d[bearing > 0.0].min()) if (bearing > 0.0).any() else inf
        right = float(d[bearing <= 0.0].min()) if (bearing <= 0.0).any() else inf
        cone = np.abs(bearing) <= self.p['stop_cone']
        ahead = float(d[cone].min()) if cone.any() else inf
        return left, right, ahead

    def _drive(self, heading_error, clearance, v_max, protect_bearing=None):
        p = self.p
        w = max(-p['max_angular_speed'], min(p['max_angular_speed'], p['heading_gain'] * heading_error))

        # --- close-range obstacle protection -----------------------------
        # A second layer on top of the corridor clearance. It never removes a
        # heading from consideration, it only bends the command away and, in
        # the last 0.30 m straight ahead, refuses to walk forward.
        left, right, ahead = self._side_ranges(protect_bearing)
        if ahead < p['safety_distance']:
            # Peel off towards the roomier side. Using the side ranges (not the
            # single nearest bearing) keeps the direction stable: sensor noise
            # on one point can flip a bearing sign every tick, which once made
            # the robot spin on the spot for a whole mission.
            if not self.protect_blocked:
                self.protect_stops += 1
                self.get_logger().warn(
                    f'obstacle {ahead:.2f} m straight ahead (L {left:.2f} / R {right:.2f}): '
                    f'stopping and turning away (stop #{self.protect_stops})',
                    throttle_duration_sec=2.0)
            self.protect_blocked = True
            return 0.0, math.copysign(p['max_angular_speed'], 1.0 if left >= right else -1.0)
        self.protect_blocked = False

        # Balanced repulsion: push away from whichever side is closer, by the
        # *difference*. Centred in a gap, left == right and this is zero, so a
        # narrow passage is driven straight through rather than bounced off.
        span = max(p['repulsion_distance'] - p['safety_distance'], 1e-3)
        def intrusion(d):
            return max(0.0, min(1.0, (p['repulsion_distance'] - d) / span))
        bias = intrusion(right) - intrusion(left)   # >0: right is closer -> steer left
        if bias != 0.0:
            w = max(-p['max_angular_speed'],
                    min(p['max_angular_speed'], w + p['repulsion_gain'] * bias))

        if abs(heading_error) > 0.6:
            return 0.0, w
        # Forward clearance along the heading actually being driven. This is the
        # corridor model, which already accounts for the robot's width -- taking
        # a min over neighbouring rays instead judged straight-line travel by
        # the clearance of directions the robot is not going, and walled the
        # robot off from victims it had already confirmed ("path blocked").
        front = self._clearance_at(clearance, 0.0)
        scale = (front - p['stop_distance']) / (p['slow_distance'] - p['stop_distance'])
        v = v_max * max(0.0, min(1.0, scale)) * math.cos(heading_error)
        # Fade out as the hard floor approaches, so the robot arrives at an
        # obstacle slowly instead of hitting the stop at full speed.
        if ahead < p['repulsion_distance']:
            v *= max(0.0, min(1.0, (ahead - p['safety_distance']) / span))
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

        # Weave about the chosen heading so the camera covers far more than the
        # corridor straight ahead. The goal heading itself is untouched, so
        # planning, the keep-heading bonus and the blocked test are unaffected.
        sweep = p['sweep_amplitude'] * math.sin(2.0 * math.pi * now / max(p['sweep_period'], 1e-3))
        base = wrap_angle(self.goal_heading - self.yaw)
        error = wrap_angle(self.goal_heading + sweep - self.yaw)
        # _drive stops and turns on the spot past 0.6 rad. If the robot was
        # walking, the sweep must never be what pushes it over that line --
        # measured 2026-11-13, 43% of EXPLORE ticks were standing still for
        # exactly this reason, and the sweep is supposed to cost 10% of the
        # speed, not 43% of the walking time.
        if abs(base) < 0.6:
            error = max(-0.55, min(0.55, error))
        return self._drive(error, clearance, p['max_linear_speed'])

    def _choose_heading(self, clearance):
        p = self.p
        free = clearance >= p['min_free_distance']
        if not free.any():
            return False
        travel = self._travel_heading()
        scores = np.full(len(self.headings), -np.inf)
        for i in np.where(free)[0]:
            rel = self.headings[i]
            absolute = self.yaw + rel
            # Novelty along the whole ray, not just its far end. Scoring only
            # the endpoint made a heading back down the corridor just travelled
            # look as good as a fresh one, because the far end of that corridor
            # had never been stood in.
            reach = min(clearance[i] - p['stop_distance'], 3.0)
            ca, sa = math.cos(absolute), math.sin(absolute)
            visited = sum(self._visited_level(self.x + f * reach * ca, self.y + f * reach * sa)
                          for f in (0.4, 0.7, 1.0)) / 3.0
            score = (p['weight_clearance'] * min(clearance[i], 4.0) / 4.0
                     + p['weight_visited'] * (1.0 - visited)
                     - p['weight_turn'] * abs(rel) / math.pi)
            if self.goal_heading is not None and \
                    abs(wrap_angle(absolute - self.goal_heading)) < 0.3:
                score += p['weight_keep_heading']
            # Explicit anti-backtrack term: never prefer to reverse down the
            # path just walked unless there is genuinely nothing else.
            if travel is not None and abs(wrap_angle(absolute - travel)) > p['backtrack_angle']:
                score -= p['weight_backtrack']
            scores[i] = score + np.random.uniform(0.0, 0.05)
        self.goal_heading = wrap_angle(self.yaw + self.headings[int(scores.argmax())])
        return True

    def _travel_heading(self):
        """Direction the robot has actually been moving over the last seconds.

        Used to recognise a candidate heading as "back the way we came". It is
        the net displacement, so turning in place or sitting next to a victim
        does not reset it.
        """
        if len(self.pose_history) < 2:
            return None
        now = self.pose_history[-1][0]
        past = None
        for t, x, y, _ in self.pose_history:
            if now - t <= self.p['backtrack_memory']:
                past = (x, y)
                break
        if past is None:
            return None
        dx, dy = self.x - past[0], self.y - past[1]
        if math.hypot(dx, dy) < 0.3:
            # Barely moved. Standing still happens for a reason -- sitting next
            # to a victim takes ~7 s -- and forgetting which way we were going
            # is exactly when the robot used to turn round and walk back over
            # ground it had already cleared. Reuse the last real direction.
            if self.last_travel is not None and \
                    now - self.last_travel[0] <= 3.0 * self.p['backtrack_memory']:
                return self.last_travel[1]
            return None
        heading = math.atan2(dy, dx)
        self.last_travel = (now, heading)
        return heading

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
            # Only when the two sensors agree is the thing straight ahead the
            # victim. A much nearer return is something else standing between
            # the robot and the body -- believing it is what makes the robot
            # sit down too early and score nothing.
            if abs(front_now - dist) <= p['lidar_range_agreement']:
                dist = min(dist, front_now)

        if dist <= p['sit_distance'] and self._enough_evidence():
            return self._reach_victim(tx, ty, dist)
        if now - self.target['start'] > p['approach_timeout'] or \
                now - self.target['last_hit'] > p['lost_timeout']:
            return self._give_up_target(now, 'timeout' if self._enough_evidence() else 'not confirmed')

        # The victim is an obstacle to the lidar: shield its bearing from the
        # repulsion field, or the robot would be pushed away from what it is
        # trying to reach.
        # A sidestep in progress: hold the heading on the victim (so the camera
        # keeps seeing it) and translate sideways out of whatever is in the way.
        if self.sidestep_until is not None:
            if now < self.sidestep_until:
                self.cmd_vy = self.sidestep_dir * p['sidestep_speed']
                return 0.0, max(-p['max_angular_speed'],
                                min(p['max_angular_speed'], p['heading_gain'] * error))
            self.sidestep_until = None
            self.blocked_time = 0.0

        v, w = self._drive(error, clearance, p['approach_linear_speed'], protect_bearing=error)
        if dist <= p['sit_distance']:
            v = 0.0  # close enough, keep facing it until enough detections arrive
        elif v < 0.02 and abs(error) < 0.5:
            self.blocked_time += dt
            if self.blocked_time > p['blocked_time_limit']:
                # A victim lying against something can still be within reach.
                if dist <= p['success_distance'] and self._enough_evidence():
                    return self._reach_victim(tx, ty, dist)
                if self.sidestep_count < p['sidestep_attempts']:
                    left, right, _ = self._side_ranges(protect_bearing=error)
                    self.sidestep_dir = 1.0 if left >= right else -1.0
                    self.sidestep_until = now + p['sidestep_duration']
                    self.sidestep_count += 1
                    self.get_logger().info(
                        f'path to ({tx:.2f}, {ty:.2f}) blocked, stepping '
                        f'{"left" if self.sidestep_dir > 0 else "right"} '
                        f'({self.sidestep_count}/{p["sidestep_attempts"]})')
                    return 0.0, w
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
        self.egress_from = (self.path_length, self._now())
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
            # Carry on outward from here. There is no reason to return to the
            # start after scoring: seed the next plan with the direction we were
            # travelling in, so weight_keep_heading pulls forward rather than
            # letting a fresh plan pick the way back.
            self.goal_heading = self._travel_heading()
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
        sat = max(self.p['visit_saturation'], 1e-3)
        total = sum(min(self.visits.get((cx + i, cy + j), 0.0) / sat, 1.0)
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
        msg.linear.y = float(self.cmd_vy)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)
        self.cmd_vy = 0.0   # one tick only; every tick that strafes sets it again

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
