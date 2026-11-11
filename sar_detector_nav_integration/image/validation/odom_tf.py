#!/usr/bin/env python3
"""Broadcast odom -> base_link on /tf from the retimed Go2 odometry.

The Go2 publishes no TF, and RTAB-Map needs this link. Run exactly one TF source for
odom -> base_link. Needs receive_time_adapter running with
odom_input:=/utlidar/robot_odom odom_output:=/lumen/retimed/robot_odom.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        topic = self.declare_parameter('odom_topic', '/lumen/retimed/robot_odom').value
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self.on_odom, 10)

    def on_odom(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
