#!/usr/bin/env python3
"""
Waypoint navigation for Unitree Go2.

How to get waypoint coordinates from RViz:
  1. Launch go2_navigation.launch.py and wait for the map to appear
  2. In RViz toolbar: click the "Publish Point" tool (crosshair icon)
  3. Click on the desired location in the map view
  4. The x, y coordinates are printed to the terminal
  5. Add them to the WAYPOINTS list below

Format:  (x,   y,   yaw_deg)
  yaw_deg = final heading at that waypoint
            0   → facing map +X direction
            90  → facing map +Y direction
            180 → facing map -X direction
            -90 → facing map -Y direction
  Set yaw_deg = None to skip heading alignment (robot stops once within
  xy_goal_tolerance — avoids spin-in-place jitter at each stop)

Set LOOP = True to cycle through waypoints continuously until Ctrl-C.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# ── Edit these ───────────────────────────────────────────────────────────────

WAYPOINTS = [
    # (  x,     y,   yaw_deg )
    (  1.7,   -0.7,      -18.0 ),   # waypoint 1
    (  0.0,   1.0,     72.0 ),   # waypoint 2
    (  -2.5,   2.0,    -32.7),   # waypoint 3
    (  -3.8,   0.4,      12.0 ),   # waypoint 4 — back to start
]

LOOP = False   # set True to cycle continuously

# ─────────────────────────────────────────────────────────────────────────────


def yaw_to_quaternion(yaw_rad):
    """Convert a yaw angle (rad) to a geometry_msgs quaternion (z, w only)."""
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _make_goal(self, x, y, yaw_deg):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        if yaw_deg is None:
            pose.pose.orientation.w = 1.0   # identity — any heading accepted
        else:
            qz, qw = yaw_to_quaternion(math.radians(yaw_deg))
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

        goal.pose = pose
        return goal

    def _send_and_wait(self, goal, label):
        """Send a single goal and block until it succeeds, fails, or is cancelled."""
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'[{label}] goal REJECTED by action server')
            return False

        self.get_logger().info(f'[{label}] goal accepted — navigating ...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        # action status codes: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        status = result_future.result().status
        if status == 4:
            self.get_logger().info(f'[{label}] REACHED  ✓')
            return True
        else:
            self.get_logger().warn(
                f'[{label}] did not reach goal (status={status}). '
                'Check for obstacles or increase xy_goal_tolerance.')
            return False

    def run(self):
        self.get_logger().info('Waiting for navigate_to_pose action server ...')
        self._client.wait_for_server()
        self.get_logger().info(
            f'Action server ready.  {len(WAYPOINTS)} waypoints, loop={LOOP}')

        iteration = 0
        while rclpy.ok():
            iteration += 1
            if LOOP:
                self.get_logger().info(f'--- Loop {iteration} ---')

            for idx, wp in enumerate(WAYPOINTS):
                x, y, yaw_deg = wp
                label = f'WP{idx + 1} ({x:.2f}, {y:.2f})'
                goal = self._make_goal(x, y, yaw_deg)
                self._send_and_wait(goal, label)

            if not LOOP:
                break

        self.get_logger().info('Waypoint navigation complete.')


def main():
    rclpy.init()
    node = WaypointNavigator()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — stopping.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()