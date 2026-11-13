#!/usr/bin/env python3
"""Reflex -- search and rescue with no map, no SLAM and no global planner.

The fallback for when SLAM cannot run on the robot: the lidar pipeline is
degraded, the link to the operator is down, scan matching diverges, or there
is no compute budget left for a map. Survey (go2_sar_slam.slam_node)
needs all of that; this node needs a point cloud, an IMU and YOLO boxes.

What it does NOT do, on purpose:
  * no occupancy grid, no scan matching, no pose estimate used for steering
  * no global path, no costmap, no replanning
Every steering decision is a function of the LATEST scan in the body frame.
That is what makes it survive the failures above -- there is no state that can
diverge, because there is almost no state.

  SPIN     one turn on the spot, so the camera sees the start area
  CRUISE   follow a wall at `wall_standoff`, steering into the widest clear
           gap when the wall is lost or the front closes in
  APPROACH a confirmed person: steer to its bearing and close in
  SIT      sit down beside them (= found), stand back up, turn away
  DONE     time is up, or `expected_victims` have been found

Coverage comes from wall following, not from reasoning about what has been
seen: with one consistent hand preference the robot traverses the connected
walls of the arena, and the camera reaches `camera_range` to either side of
that path. It is a worse search than Survey's next-best-view, and it is meant
to be -- it buys back robustness, not efficiency.

Victim handling needs no map either. A detection's bearing comes from the
bounding box, its range from the lidar points in that angular wedge, and both
are in the body frame, which is all `APPROACH` needs.

The one piece of state that would like a world frame is "which victims have I
already sat beside". `/go2/odom` is used for that when it is available -- note
this is the robot's own leg/IMU odometry, not SLAM, so it is still there when
mapping is not. When it is missing or stale the node dead-reckons from the
velocities it commanded, which drifts; the blacklist radius is deliberately
generous to absorb that, and the referee rejects duplicates anyway. Nothing
about steering depends on either.
"""

import json
import math
import sys
import time

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Imu, PointCloud2
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray

from go2_sar_core.pointcloud_utils import (cloud_to_xyz, corridor_clearance,
                                               rotation_rpy, wrap_angle,
                                               yaw_from_quaternion)


class Reflex(Node):
    # ------------------------------------------------------------------ setup
    def __init__(self):
        super().__init__('sar_reflex')
        self.p = self._declare_parameters()
        p = self.p

        # Camera model: from camera_info when the topic is given (the real
        # D435i publishes its own calibration, at whatever resolution the driver
        # was started with), else from hfov + image size (simulation).
        # Reflex rejects victims on apparent SIZE (min_person_extent,
        # victim_max_bottom, victim_max_top), all of which divide by fx -- a
        # wrong fx does not just shift a bearing here, it silently rejects or
        # accepts the wrong bodies.
        self.fx = (p['image_width'] / 2.0) / math.tan(p['camera_hfov'] / 2.0)
        self.cx0, self.cy0 = p['image_width'] / 2.0, p['image_height'] / 2.0
        self.img_h = p['image_height']
        self.have_camera_info = False

        # --- the only persistent state, and none of it steers the robot ---
        self.state = 'WAIT'
        self.state_since = self._now()
        self.start = None
        self.yaw = None                  # IMU heading, for turn control only
        self.yaw0 = None
        self.spun = 0.0
        self.obstacles = np.empty((0, 3))   # latest scan, body frame
        self.ground_z = p['ground_z_init']
        self.target = None               # (bearing, range, stamp) being approached
        self.found = []                  # dead-reckoned (x, y) of victims sat beside
        self.x = self.y = 0.0            # dead reckoning, blacklist only
        self.odom_xy = None
        self.odom_stamp = None
        self.last_cmd = (0.0, 0.0)
        self.last_tick = self._now()
        self.last_tick_target = self.last_tick
        self.last_yaw = None
        self.hits = []                   # recent detections, for confirmation
        self.wall_side = 1.0 if p['follow_wall'] == 'left' else -1.0
        self.sit_until = None
        self.turn_away_until = None
        self.last_heading_world = None   # committed gap heading, WORLD frame
        self.no_wall_since = None        # when the followed wall went out of reach
        self.detector_on = False

        self.create_subscription(PointCloud2, p['cloud_topic'], self._on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, p['imu_topic'], self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, p['odom_topic'], self._on_odom, 10)
        self.create_subscription(Detection2DArray, p['detections_topic'],
                                 self._on_detections, 10)
        if p['camera_info_topic']:
            self.create_subscription(CameraInfo, p['camera_info_topic'], self._on_camera_info, 10)

        self.cmd_pub = self.create_publisher(Twist, p['cmd_vel_topic'], 10)
        self.body_pose_pub = self.create_publisher(Pose, p['body_pose_topic'], 10)
        self.detector_pub = self.create_publisher(Bool, p['detector_enable_topic'], 10)
        self.event_pub = self.create_publisher(String, '/sar/events', 10)
        self.status_pub = self.create_publisher(String, '/sar/status', 10)

        self.create_timer(1.0 / p['control_rate_hz'], self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f'Reflex: no map, no SLAM. following the {p["follow_wall"]} wall at '
            f'{p["wall_standoff"]:.2f} m, {p["expected_victims"]} victims, '
            f'time limit {p["time_limit"]:.0f} s')

    def _declare_parameters(self):
        defaults = {
            'cloud_topic': '/go2/cloud',
            'imu_topic': '/go2/imu',
            'odom_topic': '/go2/odom',
            'detections_topic': '/sar/person_detections',
            'cmd_vel_topic': '/sar/cmd_vel',
            'body_pose_topic': '/sar/body_pose',
            'detector_enable_topic': '/sar/detector_enable',

            'control_rate_hz': 10.0,
            'time_limit': 300.0,
            'start_after_sim_time': 40.0,   # the gait controller needs to be up
            'expected_victims': 4,

            # --- geometry of the robot and its sensors ---
            'robot_half_width': 0.22,
            'lidar_xyz': [0.2, 0.0, 0.1177],
            'lidar_rpy': [0.0, 0.0, 0.0],
            'camera_xyz': [0.326, 0.0, 0.043],
            'camera_hfov': 1.367,
            # '' = use camera_hfov + image_width/height (simulation).
            # Set to the real camera's info topic on the robot.
            'camera_info_topic': '',
            'camera_range': 3.5,
            'image_width': 640,
            'image_height': 480,
            'ground_z_init': -0.21,
            'ground_z_min': -0.32,
            'ground_z_max': -0.10,
            'obstacle_min_height': 0.08,    # above the floor plane, to ignore the floor
            'obstacle_max_height': 1.00,
            'max_range': 5.0,

            # --- motion. NOTE min_forward_speed: CHAMP stalls below ~0.15 m/s,
            # but the REAL Go2 does not walk below 0.30. Raise this on the robot.
            'max_linear_speed': 0.30,
            'min_forward_speed': 0.22,
            'max_angular_speed': 0.60,
            'max_lateral_speed': 0.25,
            'heading_gain': 1.2,
            # Above this heading error, turn on the spot instead of driving.
            # min_forward_speed exists to clear the Go2's gait dead zone, but
            # applying it while a large heading error is being corrected makes
            # the robot circle instead of aiming: measured 2026-11-13, CRUISE
            # held w at the -0.6 rad/s limit with v pinned at 0.22 m/s, a stable
            # 0.37 m circle. Turning in place is not affected by the dead zone
            # (it needs no forward speed), so the floor must not apply here.
            'turn_in_place_angle': 0.6,
            # Stickiness of the chosen gap. _best_gap re-picks from scratch every
            # tick over discrete bins, so the heading jitters and the robot
            # chases it. Penalising movement away from the committed heading
            # holds a course. This -- not a stop-and-turn threshold -- is the
            # real cure for circling: the threshold alone left 33% of the
            # mission turning on the spot and delayed the only score by 51 s.
            'gap_stickiness': 0.6,
            # APPROACH: how far the gap search may bend away from the victim's
            # bearing before the detour is treated as "blocked". Walking a bent
            # heading moves the bearing, which bends the next one further, and
            # the robot orbits the body until the approach times out: measured
            # 2026-11-13, twelve identical commands (v 0.28, w -0.48), a 0.58 m
            # circle. The Go2 can strafe (max_lateral_speed), which sidesteps
            # the obstacle while keeping the camera on the victim.
            'approach_detour_limit': 0.45,  # [rad]
            # With no wall in reach the old code applied a fixed 0.25 rad bias
            # towards the followed side every tick, which is itself a circle
            # (0.22 / (1.2*0.25) = 0.73 m radius) -- the observed "always going
            # left". Ramp it in: go straight first, curl only once the open
            # space has persisted.
            'wall_search_straight': 3.0,    # [s] straight before curling
            'wall_search_bias': 0.25,       # [rad] bias once fully ramped in

            # --- reflex avoidance ---
            'stop_distance': 0.55,          # front corridor clear at least this far
            'slow_distance': 1.00,          # below this, scale the forward speed
            'gap_half_angle': 1.40,         # [rad] search gaps within +-this of ahead
            'gap_bins': 29,                 # odd, so one bin points straight ahead
            'side_clearance': 0.35,         # a gap must leave this beside the body

            # --- wall following ---
            'follow_wall': 'left',          # left | right
            'wall_standoff': 0.90,
            'wall_window': 0.50,            # [rad] cone used to measure the wall
            'wall_search_range': 2.50,      # no wall within this -> go find one
            'wall_gain': 1.10,              # standoff error [m] -> heading offset [rad]
            'wall_max_offset': 0.70,        # [rad] cap on that offset

            # --- stuck handling (no map, so this is purely time + scan based) ---
            'blocked_window': 2.5,
            'blocked_fraction': 0.5,
            'escape_time': 2.0,
            'escape_speed': 0.20,
            'spin_rate': 0.6,

            # --- victims ---
            'detection_min_score': 0.40,
            'confirm_hits': 3,
            'confirm_window': 2.0,
            'confirm_bearing_tol': 0.25,
            'victim_min_range': 0.5,
            'victim_max_range': 6.0,
            'min_person_extent': 0.25,
            'victim_max_bottom': 0.55,      # box bottom must be near the floor
            'victim_max_top': 1.30,         # ... and not tower above a sitting person
            # Lidar-side height gate. The two box tests above are skipped
            # whenever the box touches an image edge (v_bot/v_top guards), which
            # is exactly what happens close up -- so a second, image-independent
            # test measures the nearest cluster itself. Victims only ever lie
            # (0.36 m) or sit (0.72 m); a wall contributes returns all the way
            # up to obstacle_max_height (1.00), so its 90th percentile lands
            # near 0.95 and is rejected.
            'victim_max_height': 0.90,
            'sit_distance': 0.75,
            'sit_duration': 3.0,
            'sit_height_offset': -0.08,
            'sit_pitch': -0.3,
            'approach_timeout': 25.0,
            'victim_fresh_time': 1.5,       # [s] a sit needs a sighting this recent
            'victim_lost_time': 6.0,        # [s] no sighting this long -> give up
            # Close in, a body lying on the floor drops out of the camera's
            # vertical field of view -- the Go2's camera cannot look down at its
            # own feet. The sighting then goes stale through no fault of the
            # robot and the approach was abandoned at 0.68 m, a hand's breadth
            # from scoring. Inside close_range the windows widen: the bearing
            # and range are held by the lidar at that point, not the camera.
            'close_range': 1.0,             # [m] below this, use the windows below
            'victim_fresh_time_close': 4.0,
            'victim_lost_time_close': 12.0,
            'sit_bearing_tol': 0.45,        # [rad] must be facing them to sit
            'approach_face_tol': 0.55,      # [rad] turn to face before closing in
            'track_bearing_tol': 0.60,      # [rad] a detection this close re-fixes the target
            # Distance from a past SIT POSE to a new target's projection. It must
            # cover the sit distance (~0.75 m) plus the projection error, which
            # is why it is larger than the 1.50 m used when both ends were
            # projections.
            'blacklist_radius': 2.20,
            'turn_away_time': 2.0,
        }
        out = {}
        for name, value in defaults.items():
            out[name] = self.declare_parameter(name, value).value
        return out

    # ------------------------------------------------------------------ input
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_imu(self, msg):
        self.last_yaw = self.yaw
        self.yaw = yaw_from_quaternion(msg.orientation)

    def _on_odom(self, msg):
        pos = msg.pose.pose.position
        self.odom_xy = (pos.x, pos.y)
        self.odom_stamp = self._now()

    def _on_cloud(self, msg):
        pts = cloud_to_xyz(msg)
        if not len(pts):
            return
        p = self.p
        rot = rotation_rpy(*p['lidar_rpy'])
        pts = pts @ rot.T + np.asarray(p['lidar_xyz'])
        r = np.hypot(pts[:, 0], pts[:, 1])
        near_ground = pts[(r < 2.0) & (pts[:, 2] > p['ground_z_min']) &
                          (pts[:, 2] < p['ground_z_max']), 2]
        if len(near_ground) > 50:
            self.ground_z = 0.8 * self.ground_z + 0.2 * float(np.percentile(near_ground, 50))
        h = pts[:, 2] - self.ground_z
        keep = (h > p['obstacle_min_height']) & (h < p['obstacle_max_height']) & \
               (r > 0.05) & (r < p['max_range'])
        self.obstacles = pts[keep]

    def _on_detections(self, msg):
        if self.state not in ('CRUISE', 'APPROACH', 'SPIN'):
            return
        p, now = self.p, self._now()
        self.hits = [h for h in self.hits if now - h[2] < p['confirm_window']]
        fixes = []
        for det in msg.detections:
            if not det.results or det.results[0].hypothesis.score < p['detection_min_score']:
                continue
            fix = self._locate(det)
            if fix is None:
                continue
            fixes.append(fix)
            self.hits.append((fix[0], fix[1], now))

        if self.state == 'APPROACH' and self.target is not None and fixes:
            # Re-fix the target from the live detection. Without this the stored
            # bearing is a body-frame value from the moment of confirmation, and
            # the robot turning towards it is exactly what makes it stale -- it
            # then walks at a direction the victim has left, measures the lidar
            # range to whatever stands there instead (a wall), and sits in front
            # of that. Measured 2026-11-12: three sit-downs, all rejected by the
            # referee, the robot still within 0.7 m of its start.
            bearing = self.target[0]
            near = min(fixes, key=lambda f: abs(wrap_angle(f[0] - bearing)))
            if abs(wrap_angle(near[0] - bearing)) < p['track_bearing_tol']:
                self.target = (near[0], near[1], self.target[2], now)
            return
        self._confirm(now)

    # -------------------------------------------------------------- detection
    def _on_camera_info(self, msg):
        if msg.k[0] > 0:
            self.fx, self.cx0, self.cy0 = msg.k[0], msg.k[2], msg.k[5]
            self.img_h = msg.height
            if not self.have_camera_info:
                self.have_camera_info = True
                self.get_logger().info(
                    f'camera model from camera_info: fx={self.fx:.1f} '
                    f'cx={self.cx0:.1f} cy={self.cy0:.1f} ({msg.width}x{msg.height})')

    def _locate(self, det):
        """(bearing, range) of a person, body frame. Camera bearing + lidar range.

        No depth camera and no map are involved. Rejections are the same shape
        as Survey's: they are what keeps spectators standing behind a 1.2 m
        wall from being counted as victims lying on the floor.
        """
        p = self.p
        u = det.bbox.center.position.x
        half_w = det.bbox.size_x / 2.0
        bearing = -math.atan((u - self.cx0) / self.fx)
        lo = -math.atan((u + 0.5 * half_w - self.cx0) / self.fx)
        hi = -math.atan((u - 0.5 * half_w - self.cx0) / self.fx)

        cam_x, cam_y, cam_z = p['camera_xyz']
        rng = None
        pts = self.obstacles
        if len(pts):
            rx, ry = pts[:, 0] - cam_x, pts[:, 1] - cam_y
            ang = np.arctan2(ry, rx)
            sel = (rx > 0.2) & (ang >= lo - 0.03) & (ang <= hi + 0.03)
            if sel.sum() >= 3:
                r_sel = np.hypot(rx[sel], ry[sel])
                rng = float(np.percentile(r_sel, 20))
                # Nearest cluster only: returns further back are the wall behind
                # the body, and counting them would reject every victim lying in
                # front of one.
                near = r_sel <= rng + 0.4
                if near.sum() >= 3:
                    h = pts[sel, 2][near] - self.ground_z
                    if float(np.percentile(h, 90)) > p['victim_max_height']:
                        return None   # too tall to be a lying or sitting person
        if rng is None:
            v_bottom = det.bbox.center.position.y + det.bbox.size_y / 2.0
            alpha = math.atan((v_bottom - self.cy0) / self.fx)
            if alpha > 0.03:
                rng = (cam_z - self.ground_z) / math.tan(alpha)
        if rng is None or not (p['victim_min_range'] <= rng <= p['victim_max_range']):
            return None
        # a far person seen past a near edge inherits the edge's range: the box
        # is then far too small to be a person at that distance
        if max(det.bbox.size_x, det.bbox.size_y) * rng / self.fx < p['min_person_extent']:
            return None
        cam_h = cam_z - self.ground_z
        v_top = det.bbox.center.position.y - det.bbox.size_y / 2.0
        v_bot = det.bbox.center.position.y + det.bbox.size_y / 2.0
        bottom_h = cam_h - (v_bot - self.cy0) * rng / self.fx
        top_h = cam_h + (self.cy0 - v_top) * rng / self.fx
        if v_bot < self.img_h - 3 and bottom_h > p['victim_max_bottom']:
            return None
        if v_top > 3 and top_h > p['victim_max_top']:
            return None
        if self._blacklisted(bearing, rng):
            return None
        return bearing, rng

    def _blacklisted(self, bearing, rng):
        """Already sat beside this one? Dead-reckoned, deliberately tolerant.

        The stored point is where the ROBOT sat, not where it projected the
        body. The projection is the product of three error sources -- the lidar
        range (+-0.4 m measured), the camera bearing and the odometry -- so two
        approaches to the same body from different angles can project more than
        blacklist_radius apart and the suppression lapses. Measured 2026-11-13:
        two sits 0.56 m apart on the same victim, well inside a 1.50 m radius,
        and the second was still accepted. The robot's own pose carries only
        the odometry error, and "I have already sat here" is what we mean.
        """
        if not self.found:
            return False
        c, s = math.cos(self.yaw or 0.0), math.sin(self.yaw or 0.0)
        bx, by = rng * math.cos(bearing), rng * math.sin(bearing)
        wx, wy = self.x + c * bx - s * by, self.y + s * bx + c * by
        return any(math.hypot(wx - fx, wy - fy) < self.p['blacklist_radius']
                   for fx, fy in self.found)

    def _confirm(self, now):
        """`confirm_hits` detections at a consistent bearing inside the window."""
        p = self.p
        if self.state == 'APPROACH' or len(self.hits) < p['confirm_hits']:
            return
        recent = sorted(self.hits, key=lambda h: h[2])[-p['confirm_hits']:]
        bearings = [h[0] for h in recent]
        spread = max(abs(wrap_angle(b - bearings[-1])) for b in bearings)
        if spread > p['confirm_bearing_tol']:
            return
        rng = float(np.median([h[1] for h in recent]))
        self.target = (bearings[-1], rng, now, now)   # bearing, range, started, last seen
        self.last_tick_target = now                  # dt starts here, not at node start
        self.hits.clear()
        self._event(f'CONFIRMED person at bearing {math.degrees(bearings[-1]):.0f} deg, '
                    f'{rng:.2f} m')
        self._set_state('APPROACH')

    # ------------------------------------------------------- reflex avoidance
    def _clearance(self, headings):
        """Free distance down a body-width corridor along each heading."""
        return corridor_clearance(self.obstacles, np.asarray(headings),
                                  self.p['robot_half_width'], self.p['max_range'])

    def _front(self):
        return float(self._clearance([0.0])[0])

    def _best_gap(self, preferred):
        """Heading closest to `preferred` whose corridor is clear enough.

        This is the whole obstacle-avoidance strategy: no map, no memory, just
        the current scan sampled over a fan of candidate headings.
        """
        p = self.p
        angles = np.linspace(-p['gap_half_angle'], p['gap_half_angle'], p['gap_bins'])
        clear = self._clearance(angles)
        ok = clear > max(p['stop_distance'], p['side_clearance'] * 2)
        if not ok.any():
            return None, float(clear.max())
        cand = angles[ok]
        cost = np.abs(wrap_angle(cand - preferred))
        if self.last_heading_world is not None and self.yaw is not None:
            # Held in the WORLD frame on purpose: candidates are body-frame
            # angles, so keeping the previous body-frame value would chase a
            # direction that rotates with the robot -- a self-lock that turns
            # every correction into another turn.
            cost = cost + p['gap_stickiness'] * np.abs(
                wrap_angle((self.yaw + cand) - self.last_heading_world))
        pick = cand[int(np.argmin(cost))]
        if self.yaw is not None:
            self.last_heading_world = wrap_angle(self.yaw + float(pick))
        return float(pick), float(clear[angles == pick][0])

    def _wall_offset(self):
        """Heading offset that holds the followed wall at `wall_standoff`.

        Returns None when there is no wall to follow, which is the signal to
        go looking for one instead of steering against a measurement of noise.
        """
        p = self.p
        side = self.wall_side * math.pi / 2.0
        fan = np.linspace(side - p['wall_window'], side + p['wall_window'], 9)
        d = float(np.min(self._clearance(fan)))
        if d >= p['wall_search_range']:
            return None, d
        err = d - p['wall_standoff']
        off = max(-p['wall_max_offset'], min(p['wall_max_offset'], p['wall_gain'] * err))
        return self.wall_side * off, d

    # ------------------------------------------------------------- behaviours
    def _set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state}')
            self.state = state
            self.state_since = self._now()

    def _event(self, text):
        self.get_logger().info(f'EVENT {text}')
        self.event_pub.publish(String(data=text))

    def _drive(self, v, w, vy=0.0):
        p = self.p
        v = max(-p['max_linear_speed'], min(p['max_linear_speed'], v))
        w = max(-p['max_angular_speed'], min(p['max_angular_speed'], w))
        vy = max(-p['max_lateral_speed'], min(p['max_lateral_speed'], vy))
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = v, vy, w
        self.cmd_pub.publish(msg)
        self.last_cmd = (v, w)

    def _dead_reckon(self, dt):
        """Blacklist bookkeeping only -- never used to steer."""
        if self.odom_xy is not None and self.odom_stamp is not None \
                and self._now() - self.odom_stamp < 1.0:
            self.x, self.y = self.odom_xy
            return
        v = self.last_cmd[0]
        yaw = self.yaw or 0.0
        self.x += v * dt * math.cos(yaw)
        self.y += v * dt * math.sin(yaw)

    def _enable_detector(self, on):
        if on != self.detector_on:
            self.detector_on = on
            self.detector_pub.publish(Bool(data=on))

    # -------------------------------------------------------------- main loop
    def _tick(self):
        now = self._now()
        dt = max(1e-3, now - self.last_tick)
        self.last_tick = now
        self._dead_reckon(dt)

        if self.state == 'WAIT':
            if now < self.p['start_after_sim_time'] or self.yaw is None \
                    or not len(self.obstacles):
                return
            self.start = now
            self.yaw0 = self.yaw
            self._enable_detector(True)
            self._event('MISSION_START reflex: no map, no SLAM')
            self._set_state('SPIN')
            return

        if self.state == 'DONE':
            self._drive(0.0, 0.0)
            return

        if now - self.start > self.p['time_limit']:
            self._drive(0.0, 0.0)
            self._event(f'DONE found {len(self.found)}/{self.p["expected_victims"]} '
                        f'in {now - self.start:.1f} s (time limit)')
            self._set_state('DONE')
            return

        getattr(self, f'_do_{self.state.lower()}')(now)

    def _do_spin(self, now):
        """One turn on the spot so the camera sees the start area."""
        if self.yaw is not None and self.yaw0 is not None:
            self.spun += abs(wrap_angle(self.yaw - self.yaw0))
            self.yaw0 = self.yaw
        self._drive(0.0, self.p['spin_rate'])
        if self.spun >= 2 * math.pi - 0.3 or now - self.state_since > 15.0:
            self._set_state('CRUISE')

    def _do_cruise(self, now):
        p = self.p
        front = self._front()

        if self.turn_away_until and now < self.turn_away_until:
            self._drive(0.0, -self.wall_side * p['spin_rate'])
            return
        self.turn_away_until = None

        offset, wall_d = self._wall_offset()
        if offset is None:
            # No wall in reach: head for open space and let the gap search find
            # one. Biasing slightly towards the followed side makes it curl in
            # rather than cross the arena forever.
            if self.no_wall_since is None:
                self.no_wall_since = now
            ramp = min(1.0, (now - self.no_wall_since) / max(p['wall_search_straight'], 1e-3))
            preferred = self.wall_side * p['wall_search_bias'] * ramp
        else:
            self.no_wall_since = None
            preferred = offset

        heading, clear = self._best_gap(preferred)
        if heading is None or front < p['stop_distance']:
            # Everything ahead is blocked: turn away from the wall being
            # followed. With a consistent hand this walks out of corners.
            self._drive(0.0, -self.wall_side * p['spin_rate'])
            return

        scale = 1.0
        if clear < p['slow_distance']:
            scale = max(0.4, (clear - p['stop_distance']) /
                        max(1e-3, p['slow_distance'] - p['stop_distance']))
        if abs(heading) > p['turn_in_place_angle']:
            # Aim first, then walk. Driving through a large heading error only
            # traces a circle, because the distance covered meanwhile moves the
            # gap solution along with it.
            self._drive(0.0, p['heading_gain'] * heading)
            return
        v = max(p['min_forward_speed'], p['max_linear_speed'] * scale * math.cos(heading))
        self._drive(v, p['heading_gain'] * heading)

    def _do_approach(self, now):
        p = self.p
        if self.target is None:
            self._set_state('CRUISE')
            return
        bearing, rng, since, seen = self.target
        if now - since > p['approach_timeout']:
            self._event(f'GIVE_UP person at {rng:.2f} m: approach timed out')
            self.target = None
            self._set_state('CRUISE')
            return
        close = rng <= p['close_range']
        lost_window = p['victim_lost_time_close'] if close else p['victim_lost_time']
        fresh_window = p['victim_fresh_time_close'] if close else p['victim_fresh_time']
        if now - seen > lost_window:
            self._event(f'GIVE_UP person at {rng:.2f} m: lost sight of them')
            self.target = None
            self._set_state('CRUISE')
            return

        # Sit only on a FRESH sighting at close range. The lidar range along the
        # bearing is not enough on its own -- a wall in that direction reads the
        # same, and that is precisely how the first version sat down three times
        # without ever reaching anybody.
        if rng <= p['sit_distance'] and now - seen <= fresh_window \
                and abs(bearing) < p['sit_bearing_tol']:
            self._drive(0.0, 0.0)
            self.sit_until = None
            self._set_state('SIT')
            return

        # Turn to face them before closing in: a large bearing error means the
        # dead reckoning below is about to be wrong anyway.
        if abs(bearing) > p['approach_face_tol']:
            self._drive(0.0, p['heading_gain'] * bearing)
            self._dead_reckon_target(now, 0.0)
            return

        heading, clear = self._best_gap(bearing)
        if heading is None:
            self._drive(0.0, math.copysign(p['spin_rate'], bearing or 1.0))
            self._dead_reckon_target(now, 0.0)
            return
        if abs(wrap_angle(heading - bearing)) > p['approach_detour_limit']:
            # The direct line is blocked. Hold the nose on the victim (so the
            # camera keeps seeing it) and translate sideways towards whichever
            # flank the gap search prefers, instead of circling the body.
            side = math.copysign(1.0, wrap_angle(heading - bearing))
            self._drive(0.0, p['heading_gain'] * bearing,
                        vy=side * p['max_lateral_speed'])
            self._dead_reckon_target(now, 0.0)
            return
        # Never drive closer than the live scan says is free, victim or not.
        front = self._front()
        v = max(p['min_forward_speed'], p['max_linear_speed'] * math.cos(heading))
        if front < p['stop_distance']:
            v = 0.0
        self._drive(v, p['heading_gain'] * heading)
        self._dead_reckon_target(now, v)

    def _dead_reckon_target(self, now, v):
        """Carry the body-frame fix between sightings.

        The bearing is measured in the body frame, so every degree the robot
        turns moves it. Between detections the fix is rotated by the IMU yaw
        delta and pulled in by the distance actually travelled; a fresh
        detection overwrites it in `_on_detections`.
        """
        if self.target is None:
            return
        bearing, rng, since, seen = self.target
        # Clamped, and reset when a target is created. Without both, the first
        # call after WAIT carried the whole start-up delay as dt (the node idles
        # until start_after_sim_time) and `rng - v*dt` collapsed the range to
        # zero on the first tick of the first approach -- measured 2026-11-12:
        # "SIT track_1 measured_distance=0.00 m t=0.9 s", rejected by the referee.
        dt = min(max(1e-3, now - self.last_tick_target), 0.5)
        self.last_tick_target = now
        if self.yaw is not None and self.last_yaw is not None:
            bearing = wrap_angle(bearing - wrap_angle(self.yaw - self.last_yaw))
        rng = max(0.0, rng - v * dt * math.cos(bearing))
        self.target = (bearing, rng, since, seen)

    def _do_sit(self, now):
        p = self.p
        if self.sit_until is None:
            self._drive(0.0, 0.0)
            pose = Pose()
            pose.position.z = float(p['sit_height_offset'])
            pose.orientation.y = math.sin(p['sit_pitch'] / 2)
            pose.orientation.w = math.cos(p['sit_pitch'] / 2)
            self.body_pose_pub.publish(pose)
            self.sit_until = now + p['sit_duration']
            bearing, rng = (self.target[0], self.target[1]) if self.target else (0.0, 0.0)
            self.found.append((self.x, self.y))
            # `track_N`, not `victim_N`: the referee reads the second word as a
            # claim and checks it against the real victim names. A counter that
            # happens to collide with one of those names would occasionally make
            # a correct sit look like a claim about a different victim. With a
            # name it does not recognise, the referee falls back to the nearest
            # victim not yet credited, which is what we actually mean.
            self._event(f'SIT track_{len(self.found)} measured_distance={rng:.2f} m '
                        f't={now - self.start:.1f} s')
            return
        if now < self.sit_until:
            return
        self.body_pose_pub.publish(Pose())      # stand back up
        self.target = None
        self.turn_away_until = now + p['turn_away_time']
        if len(self.found) >= p['expected_victims']:
            self._event(f'DONE found {len(self.found)}/{p["expected_victims"]} '
                        f'in {now - self.start:.1f} s')
            self._set_state('DONE')
            return
        self._set_state('CRUISE')

    # ------------------------------------------------------------------ status
    def _publish_status(self):
        now = self._now()
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
        self.status_pub.publish(String(data=json.dumps(dict(
            state=self.state, found=len(self.found),
            elapsed=round(now - self.start, 1) if self.start else 0.0,
            front=round(self._front(), 2) if len(self.obstacles) else None,
            points=int(len(self.obstacles))))))


def main(args=None):
    rclpy.init(args=args)
    node = Reflex()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
