#!/usr/bin/env python3
"""Reactive-drive + map-aware-turn hybrid exploration.

Replaces explore_lite, whose frontier-goal replanning loop got stuck
re-sending an already-reached goal near walls/corners (confirmed in its
source: reachedGoal() calls makePlan() immediately on every success with
no cooldown). Instead of frontier goals, this sends short NavigateToPose
steps directly ahead of the robot and only re-plans when it has to:

  DRIVE: no goal in flight and nothing close ahead -> send a short goal
         straight along the current heading.
  AVOID: something close ahead (checked against the LOCAL costmap, which
         is what the controller itself is already reacting to) -> cancel
         the current goal, score candidate headings by how much UNKNOWN
         space (from the cumulative /map, not the local costmap -- only
         the full map distinguishes "never seen" from "seen and open")
         lies that way, and send a short goal along the best one.

All motion still goes through navigate_to_pose -- this node never
publishes cmd_vel directly, so there is only ever one command source.
Fast collision safety (collision_monitor on raw /utlidar/scan, MPPI's own
costmap-aware planning) is unchanged and keeps reacting at full lidar
rate regardless of this node's decisions; this node only decides which
goal to pursue next, not whether the robot is about to hit something.

Coverage caveat: this is a greedy local strategy, not frontier BFS -- it
is not guaranteed to fully cover the map the way explore_lite aimed to.
Watch for dead-ends it doesn't back out of.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf2_ros import TransformException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def yaw_to_quaternion(yaw_rad):
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def yaw_from_quaternion(qz, qw):
    return 2.0 * math.atan2(qz, qw)


def grid_value(grid: OccupancyGrid, wx, wy):
    """Look up the cell value at world coords (wx, wy) in grid's own frame.
    Returns None if outside the grid (treated as unknown by callers)."""
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    col = int((wx - ox) / res)
    row = int((wy - oy) / res)
    if col < 0 or row < 0 or col >= grid.info.width or row >= grid.info.height:
        return None
    return grid.data[row * grid.info.width + col]


class HybridExplorer(Node):

    def __init__(self):
        super().__init__('hybrid_explorer')

        self.forward_step = self.declare_parameter('forward_step', 1.0).value
        self.avoid_step = self.declare_parameter('avoid_step', 1.0).value
        self.check_ahead = self.declare_parameter('obstacle_check_ahead', 0.70).value
        self.check_width = self.declare_parameter('obstacle_check_width', 0.45).value
        # OccupancyGrid-published cost scale is 0-100 (lethal) or -1 (unknown).
        # 50 sits partway into the inflation falloff -- well before the
        # robot_radius-padded lethal zone, matching the spirit of the
        # StopZone/SlowZone margins already used for raw-lidar reactive_avoid.
        self.obstacle_threshold = self.declare_parameter('obstacle_cost_threshold', 50).value
        self.candidate_headings_deg = self.declare_parameter(
            'candidate_headings_deg', [-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0]).value
        self.unknown_check_radius = self.declare_parameter('unknown_check_radius', 2.0).value
        self.control_period = self.declare_parameter('control_period_s', 0.2).value
        # _obstacle_ahead() checks the robot's current physical heading, not
        # the heading of whatever goal is in flight. A freshly-sent AVOID
        # goal picks a new heading but NavigateToPose takes time to actually
        # turn there -- on the very next tick the robot is still facing
        # close to the direction that triggered the check, so it re-fires,
        # cancelling the goal before the robot has had any real chance to
        # turn. Confirmed live (2026-11-09): goals sent and cancelled every
        # ~200ms for 20+ seconds with almost no net progress. This window
        # only withholds hybrid_explorer's own re-evaluation; collision_monitor
        # and MPPI keep reacting at full rate throughout regardless.
        self.min_commit_sec = self.declare_parameter('min_commit_sec', 1.5).value
        self.last_goal_sent_time = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Confirmed via `ros2 topic info -v /local_costmap/costmap` that the
        # actual publisher QoS is RELIABLE + TRANSIENT_LOCAL, depth 1 -- the
        # same profile rviz2's (working) subscription uses. A BEST_EFFORT/
        # VOLATILE subscriber is compatible with this by the standard DDS RxO
        # rules, but in practice received nothing at all on this CycloneDDS
        # setup. Matching the confirmed-working profile exactly instead of
        # relying on theoretical compatibility.
        grid_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.local_costmap = None
        self.global_map = None
        self._local_costmap_logged = False
        self._global_map_logged = False
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._on_local, grid_qos)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, grid_qos)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None
        self.goal_in_flight = False
        self.goal_token = 0

        self.get_logger().info('hybrid_explorer: waiting for navigate_to_pose server ...')
        self.nav_client.wait_for_server()
        self.get_logger().info('hybrid_explorer: server ready, starting control loop.')

        self.timer = self.create_timer(self.control_period, self._tick)

    def _on_local(self, msg):
        if not self._local_costmap_logged:
            self.get_logger().info('First /local_costmap/costmap message received.')
            self._local_costmap_logged = True
        self.local_costmap = msg

    def _on_map(self, msg):
        if not self._global_map_logged:
            self.get_logger().info('First /map message received.')
            self._global_map_logged = True
        self.global_map = msg

    def _pose_in(self, target_frame):
        try:
            t = self.tf_buffer.lookup_transform(target_frame, 'base_link', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'TF lookup {target_frame}<-base_link failed: {e}', throttle_duration_sec=2.0)
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        yaw = yaw_from_quaternion(t.transform.rotation.z, t.transform.rotation.w)
        return x, y, yaw

    def _heading_blocked(self, grid, rx, ry, ryaw, heading_offset_rad):
        """Sample a forward box along (ryaw + heading_offset_rad) in grid's frame."""
        heading = ryaw + heading_offset_rad
        for s in (0.20, 0.35, 0.50, self.check_ahead):
            for t in (-self.check_width, 0.0, self.check_width):
                wx = rx + s * math.cos(heading) - t * math.sin(heading)
                wy = ry + s * math.sin(heading) + t * math.cos(heading)
                val = grid_value(grid, wx, wy)
                if val is not None and val >= self.obstacle_threshold:
                    return True
        return False

    def _obstacle_ahead(self):
        if self.local_costmap is None:
            return False
        pose = self._pose_in(self.local_costmap.header.frame_id or 'odom')
        if pose is None:
            return False
        rx, ry, ryaw = pose
        return self._heading_blocked(self.local_costmap, rx, ry, ryaw, 0.0)

    def _score_heading(self, rx, ry, ryaw, heading_offset_rad):
        """Count unknown cells in /map along this heading; penalize if blocked."""
        heading = ryaw + heading_offset_rad
        score = 0
        for s in (0.3, 0.7, 1.1, 1.5, self.unknown_check_radius):
            wx = rx + s * math.cos(heading)
            wy = ry + s * math.sin(heading)
            val = grid_value(self.global_map, wx, wy)
            if val is None or val < 0:
                score += 1
            elif val >= self.obstacle_threshold:
                score -= 5
                break
        return score

    def _best_avoid_heading(self):
        """Pick the candidate heading with the most unknown space, excluding
        any heading that is itself immediately blocked in the local costmap."""
        map_pose = self._pose_in(self.global_map.header.frame_id or 'map')
        local_pose = self._pose_in(self.local_costmap.header.frame_id or 'odom')
        if map_pose is None or local_pose is None:
            return None

        mx, my, myaw = map_pose
        lx, ly, lyaw = local_pose

        best_offset, best_score = None, None
        for deg in self.candidate_headings_deg:
            offset = math.radians(deg)
            if self._heading_blocked(self.local_costmap, lx, ly, lyaw, offset):
                continue
            score = self._score_heading(mx, my, myaw, offset)
            if best_score is None or score > best_score:
                best_score, best_offset = score, offset

        if best_offset is None:
            return None
        return mx, my, myaw + best_offset

    def _send_goal(self, x, y, yaw, label):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose

        self.goal_token += 1
        token = self.goal_token
        self.goal_in_flight = True
        self.last_goal_sent_time = self.get_clock().now()
        self.get_logger().info(f'[{label}] sending goal ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.0f} deg)')

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(lambda f: self._on_goal_response(f, token, label))

    def _on_goal_response(self, future, token, label):
        if token != self.goal_token:
            return  # superseded by a cancel + new send already
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'[{label}] goal rejected')
            self.goal_in_flight = False
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_goal_result(f, token, label))

    def _on_goal_result(self, future, token, label):
        if token != self.goal_token:
            return
        status = future.result().status
        self.get_logger().info(f'[{label}] finished, status={status}')
        self.goal_in_flight = False
        self.goal_handle = None

    def _cancel_current_goal(self):
        self.goal_token += 1  # invalidate any pending callbacks from the old goal
        self.goal_in_flight = False
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def _tick(self):
        if self.local_costmap is None or self.global_map is None:
            missing = []
            if self.local_costmap is None:
                missing.append('/local_costmap/costmap')
            if self.global_map is None:
                missing.append('/map')
            self.get_logger().warn(
                f'Waiting for data on: {", ".join(missing)} -- no goals can be sent yet.',
                throttle_duration_sec=3.0)
            return

        committing = (
            self.goal_in_flight and self.last_goal_sent_time is not None and
            (self.get_clock().now() - self.last_goal_sent_time).nanoseconds / 1e9 < self.min_commit_sec
        )

        if self._obstacle_ahead() and not committing:
            if self.goal_in_flight:
                self.get_logger().info('Obstacle ahead -- cancelling current goal')
                self._cancel_current_goal()

            result = self._best_avoid_heading()
            if result is None:
                self.get_logger().warn('Obstacle ahead but no clear candidate heading -- waiting', throttle_duration_sec=2.0)
                return
            mx, my, new_yaw = result
            gx = mx + self.avoid_step * math.cos(new_yaw)
            gy = my + self.avoid_step * math.sin(new_yaw)
            self._send_goal(gx, gy, new_yaw, 'AVOID')
            return

        if not self.goal_in_flight:
            pose = self._pose_in('map')
            if pose is None:
                return
            mx, my, myaw = pose
            gx = mx + self.forward_step * math.cos(myaw)
            gy = my + self.forward_step * math.sin(myaw)
            self._send_goal(gx, gy, myaw, 'DRIVE')


def main(args=None):
    rclpy.init(args=args)
    node = HybridExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
