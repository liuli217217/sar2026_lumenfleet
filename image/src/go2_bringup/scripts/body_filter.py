#!/usr/bin/env python3
"""Drop points inside the robot's own body/leg-swing footprint before mapping.

utlidar_to_laserscan has no self-exclusion at all, so a walking gait (legs
sweeping through the same height band used for obstacle detection) gets
picked up as scattered false obstacles. Subscribes to a base_link-frame
cloud, republishes it with the footprint box removed.
"""
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

# xmin, xmax, ymin, ymax, zmin, zmax relative to base_link. Original box
# validated for the stationary case in lumen_sensor_tests, widened in y for
# walking gait leg swing (2026-11-10). Widened again 2026-11-12: found live
# that this box was never validated against Spin/BackUp recovery motions --
# a pivoting in-place turn moves the legs differently than straight walking,
# and a self-return outside this box during Spin produced a lethal costmap
# cell essentially at the robot's own position, blocking the global planner
# (every goal failed identically regardless of destination) and causing
# collision_monitor to repeatedly re-trigger StopZone through the turn,
# timing out the recovery itself. This widening (x 0.40->0.45, y 0.30->0.40)
# is a reasoned estimate, not a measurement like the walking-gait one above
# -- verify with an isolated stationary Spin test (watch for a self-return
# blob in RViz the same way) before trusting this box size in general use.
EXCLUSION_BOX = (-0.45, 0.45, -0.40, 0.40, -0.50, 0.30)


class BodyFilter(Node):
    def __init__(self):
        super().__init__('body_filter')
        self.input_topic = self.declare_parameter('input_topic', '/utlidar/cloud_base_retimed').value
        self.output_topic = self.declare_parameter('output_topic', '/utlidar/cloud_base_filtered').value
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, 10)
        self.get_logger().info(
            f'body_filter: {self.input_topic} -> {self.output_topic}, box={EXCLUSION_BOX}')

    def on_cloud(self, msg: PointCloud2):
        xmin, xmax, ymin, ymax, zmin, zmax = EXCLUSION_BOX
        x_off = y_off = z_off = None
        for field in msg.fields:
            if field.name == 'x':
                x_off = field.offset
            elif field.name == 'y':
                y_off = field.offset
            elif field.name == 'z':
                z_off = field.offset
        if x_off is None or y_off is None or z_off is None:
            self.pub.publish(msg)
            return

        step = msg.point_step
        if step <= 0:
            self.pub.publish(msg)
            return
        count = len(msg.data) // step
        kept = bytearray()
        kept_count = 0
        try:
            for i in range(count):
                base = i * step
                x = struct.unpack_from('<f', msg.data, base + x_off)[0]
                y = struct.unpack_from('<f', msg.data, base + y_off)[0]
                z = struct.unpack_from('<f', msg.data, base + z_off)[0]
                if xmin < x < xmax and ymin < y < ymax and zmin < z < zmax:
                    continue
                kept.extend(msg.data[base:base + step])
                kept_count += 1
        except struct.error as error:
            self.get_logger().warn(f'Malformed point in cloud, passing through unfiltered: {error}')
            self.pub.publish(msg)
            return

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = kept_count
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = step
        out.row_step = step * kept_count
        out.is_dense = msg.is_dense
        out.data = bytes(kept)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BodyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
