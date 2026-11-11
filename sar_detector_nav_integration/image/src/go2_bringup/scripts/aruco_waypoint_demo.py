#!/usr/bin/env python3
"""
Demo sequence:
  1. Navigate to waypoint 1.
  2. Navigate to waypoint 2.
  3. Wait for ArUco marker pose in base_link.
  4. Navigate to a goal that stops approach_distance_m before the marker.

This node relies on Nav2 for motion and obstacle avoidance. It expects the
ArUco detector to publish /aruco/marker_pose_base as a PoseStamped in base_link.
"""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


WAYPOINTS = [
    (-0.66118, 0.214656, -11.32949555357836),
    (2.84275, -0.343768, -26.6307918387831),
]


def yaw_to_quaternion(yaw_rad):
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return qz, qw


def quaternion_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ])


class ArucoWaypointDemo(Node):
    def __init__(self):
        super().__init__('aruco_waypoint_demo')

        self.declare_parameter('marker_pose_topic', '/aruco/marker_pose_base')
        self.declare_parameter('approach_distance_m', 0.5)
        self.declare_parameter('marker_timeout_s', 60.0)
        self.declare_parameter('marker_max_age_s', 1.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.marker_pose_topic = self.get_parameter('marker_pose_topic').value
        self.approach_distance_m = float(self.get_parameter('approach_distance_m').value)
        self.marker_timeout_s = float(self.get_parameter('marker_timeout_s').value)
        self.marker_max_age_s = float(self.get_parameter('marker_max_age_s').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.latest_marker_pose = None
        self.latest_marker_received_time = None
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, '/aruco_demo/status', status_qos)
        self.marker_sub = self.create_subscription(
            PoseStamped,
            self.marker_pose_topic,
            self.marker_pose_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def marker_pose_callback(self, msg):
        self.latest_marker_pose = msg
        self.latest_marker_received_time = self.get_clock().now()

    def publish_status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def make_goal(self, x, y, yaw_deg):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        qz, qw = yaw_to_quaternion(math.radians(yaw_deg))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose
        return goal

    def send_goal_and_wait(self, goal, label):
        self.publish_status(f'{label}: sending goal')
        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.publish_status(f'{label}: goal rejected')
            return False

        self.publish_status(f'{label}: accepted')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status(f'{label}: reached')
            return True

        self.publish_status(f'{label}: failed with status {result.status}')
        return False

    def wait_for_fresh_marker(self):
        self.publish_status(
            f'Waiting up to {self.marker_timeout_s:.1f}s for marker on {self.marker_pose_topic}'
        )
        start = self.get_clock().now()
        last_wait_log = start

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_marker_pose is not None and self.marker_is_recent():
                age = self.marker_receive_age()
                self.publish_status(f'Target is detected, marker message age={age:.2f}s')
                return self.latest_marker_pose

            now = self.get_clock().now()
            elapsed = (now - start).nanoseconds * 1e-9
            if elapsed > self.marker_timeout_s:
                self.publish_status('Marker detection timed out')
                return None

            if (now - last_wait_log).nanoseconds * 1e-9 > 2.0:
                if self.latest_marker_pose is None:
                    self.publish_status('Still waiting: no marker pose received yet')
                else:
                    self.publish_status(
                        f'Still waiting: latest marker message age={self.marker_receive_age():.2f}s'
                    )
                last_wait_log = now

            time.sleep(0.05)

        return None

    def marker_receive_age(self):
        if self.latest_marker_received_time is None:
            return float('inf')
        return (self.get_clock().now() - self.latest_marker_received_time).nanoseconds * 1e-9

    def marker_is_recent(self):
        age = self.marker_receive_age()
        return age <= self.marker_max_age_s

    def make_marker_approach_goal(self, marker_pose):
        if marker_pose.header.frame_id != self.base_frame:
            self.publish_status(
                f'Marker pose frame is {marker_pose.header.frame_id}, expected {self.base_frame}'
            )
            return None

        marker_base = np.array([
            marker_pose.pose.position.x,
            marker_pose.pose.position.y,
            marker_pose.pose.position.z,
        ])
        planar_distance = math.hypot(marker_base[0], marker_base[1])
        if planar_distance <= self.approach_distance_m:
            self.publish_status(
                f'Already within {planar_distance:.2f}m of marker, no approach goal needed'
            )
            return None

        direction = marker_base[:2] / planar_distance
        goal_base_xy = marker_base[:2] - direction * self.approach_distance_m

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
        except TransformException as exc:
            self.publish_status(f'Could not transform {self.base_frame} to {self.map_frame}: {exc}')
            return None

        rotation = quaternion_to_matrix(transform.transform.rotation)
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ])

        goal_base = np.array([goal_base_xy[0], goal_base_xy[1], 0.0])
        marker_map = rotation.dot(np.array([marker_base[0], marker_base[1], 0.0])) + translation
        goal_map = rotation.dot(goal_base) + translation

        yaw_to_marker = math.atan2(
            marker_map[1] - goal_map[1],
            marker_map[0] - goal_map[0]
        )

        self.publish_status(
            'Approach goal: '
            f'x={goal_map[0]:.2f}, y={goal_map[1]:.2f}, '
            f'yaw={math.degrees(yaw_to_marker):.1f}deg, '
            f'standoff={self.approach_distance_m:.2f}m'
        )

        return self.make_goal(goal_map[0], goal_map[1], math.degrees(yaw_to_marker))

    def run(self):
        self.publish_status('Waiting for Nav2 navigate_to_pose action server')
        self.nav_client.wait_for_server()
        self.publish_status('Nav2 action server ready')

        for index, waypoint in enumerate(WAYPOINTS, start=1):
            x, y, yaw_deg = waypoint
            goal = self.make_goal(x, y, yaw_deg)
            if not self.send_goal_and_wait(goal, f'Waypoint {index}'):
                self.publish_status('Demo stopped because waypoint navigation failed')
                return

        marker_pose = self.wait_for_fresh_marker()
        if marker_pose is None:
            self.publish_status('Demo stopped because marker was not detected')
            return

        approach_goal = self.make_marker_approach_goal(marker_pose)
        if approach_goal is None:
            self.publish_status('Target is detected. Robot stopped at safe distance.')
            return

        if self.send_goal_and_wait(approach_goal, 'Marker approach'):
            self.publish_status('Target is detected. Robot stopped at safe distance.')
        else:
            self.publish_status('Marker approach goal failed')


def main():
    rclpy.init()
    node = ArucoWaypointDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        node.publish_status('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
