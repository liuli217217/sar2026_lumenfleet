"""Check the real-robot inputs before a mission (no motion commands are sent).

    # A4 SLAM / A3 Prior -- this node is named sar_slam, so real_nomap.yaml
    # (which keys its section to sar_slam) matches without anything extra:
    ros2 run go2_real_bridge check_sensors --ros-args --params-file .../real_nomap.yaml

    # A1 Scout / A2 Reflex -- parameters are keyed by NODE NAME, so the node
    # has to be renamed or the file's section matches nothing and this check
    # silently validates the defaults instead of what the mission will use:
    ros2 run go2_real_bridge check_sensors --ros-args -r __node:=sar_scout \
        --params-file .../real_scout.yaml
    ros2 run go2_real_bridge check_sensors --ros-args -r __node:=sar_reflex \
        --params-file .../real_reflex.yaml

Prints every few seconds: message rates of the bridged odometry / IMU / lidar and
of the camera; the lidar points transformed with the configured extrinsic
(`lidar_xyz`, `lidar_rpy` of sar_explorer / sar_navigator) and levelled with the IMU:
the ground height per direction (it must be about the same, near ground_z_init,
in front / left / right / behind - otherwise the extrinsic is wrong), and how many
points fall inside `body_box` (the robot's own body, cut away by the SAR nodes).
"""
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2

from go2_sar_core.pointcloud_utils import cloud_to_xyz, rotation_rpy


class CheckSensors(Node):
    def __init__(self):
        # Reads the mission node's own parameters out of the given file. The
        # default name matches sar_slam / real_nomap.yaml; for the other
        # missions rename the node (see the module docstring), because ROS 2
        # keys parameter files by node name and a section that does not match
        # is dropped without a word.
        super().__init__('sar_slam', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        get = lambda name, default: self.get_parameter_or(name, rclpy.Parameter(name, value=default)).value
        self.lidar_R = rotation_rpy(*get('lidar_rpy', [0.0, 0.0, 0.0]))
        self.lidar_t = np.asarray(get('lidar_xyz', [0.0, 0.0, 0.0]))
        self.body_box = get('body_box', [-0.5, 0.35, 0.25])
        topics = {'odom': (Odometry, get('odom_topic', '/go2/odom')), 'imu': (Imu, get('imu_topic', '/go2/imu')),
                  'cloud': (PointCloud2, get('cloud_topic', '/go2/cloud')),
                  'image': (Image, '/camera/camera/color/image_raw'),
                  'camera_info': (CameraInfo, get('camera_info_topic', '/camera/camera/color/camera_info'))}
        self.counts = {k: 0 for k in topics}
        self.rp = (0.0, 0.0)
        self.cloud = None
        self.info = None
        for key, (kind, topic) in topics.items():
            self.create_subscription(kind, topic, lambda m, k=key: self._on(k, m), qos_profile_sensor_data)
        self.t0 = time.time()
        self.create_timer(3.0, self._report)
        self.get_logger().info('listening: ' + ', '.join(f'{k}={t}' for k, (_, t) in topics.items()))
        # Say which geometry is actually being checked. These are the values the
        # mission will use -- if they are the defaults below while a params file
        # was passed, the file's section did not match this node's name.
        self.get_logger().info(
            f'node name "{self.get_name()}"; lidar_xyz={list(self.lidar_t)} '
            f'lidar_rpy={get("lidar_rpy", [0.0, 0.0, 0.0])} body_box={self.body_box}')
        if not self.get_parameters_by_prefix(''):
            self.get_logger().warn(
                'no parameter overrides reached this node. If you passed --params-file, its '
                'section is keyed to another node name: add -r __node:=sar_scout (A1), '
                'sar_reflex (A2) or sar_prior (A3). Checking built-in defaults instead.')

    def _on(self, key, msg):
        self.counts[key] += 1
        if key == 'imu':
            q = msg.orientation
            self.rp = (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
                       math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
        elif key == 'cloud':
            self.cloud = msg
        elif key == 'camera_info':
            self.info = msg

    def _report(self):
        dt = time.time() - self.t0
        self.t0 = time.time()
        rates = ', '.join(f'{k} {c / dt:.1f} Hz' for k, c in self.counts.items())
        self.counts = {k: 0 for k in self.counts}
        lines = [f'rates: {rates}']
        if self.info is not None and self.info.k[0] > 0:
            hfov = math.degrees(2 * math.atan(self.info.width / 2 / self.info.k[0]))
            lines.append(f'camera: {self.info.width}x{self.info.height}, fx {self.info.k[0]:.1f}, hfov {hfov:.1f} deg')
        if self.cloud is not None:
            pts = cloud_to_xyz(self.cloud) @ self.lidar_R.T + self.lidar_t
            pts = pts @ rotation_rpy(self.rp[0], self.rp[1], 0.0).T
            x0, x1, hw = self.body_box
            body = (pts[:, 0] > x0) & (pts[:, 0] < x1) & (np.abs(pts[:, 1]) < hw)
            r = np.hypot(pts[:, 0], pts[:, 1])
            ang = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
            ground = []
            for name, a in (('front', 0), ('left', 90), ('behind', 180), ('right', -90)):
                d = np.abs((ang - a + 180) % 360 - 180)
                sel = ~body & (d < 30) & (r > 0.8) & (r < 3.0)
                z = pts[sel, 2]
                ground.append(f'{name} {np.percentile(z, 5):+.2f}' if len(z) > 20 else f'{name} -')
            lines.append(f'lidar: {len(pts)} points, {body.sum()} inside body_box; ground height (5th '
                         f'percentile, 0.8-3 m) ' + ', '.join(ground))
        self.get_logger().info(' | '.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = CheckSensors()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
