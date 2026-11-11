"""Read original clouds and inspect their transform into the robot base frame."""

import math

from sensor_msgs.msg import PointCloud2
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .analysis import cloud_xyz, cloud_summary, filter_xyz, transform_xyz
from .common import SensorDiagnostic, run


class LidarDiagnostic(SensorDiagnostic):
    def __init__(self, name='lidar_diagnostic'):
        super().__init__(name, 'cloud_topic', PointCloud2)
        self.base_frame = self.parameter('base_frame', 'base_link')
        self.min_range = self.parameter('min_range', 0.3)
        self.max_range = self.parameter('max_range', 10.0)
        self.min_height = self.parameter('min_height', -0.1)
        self.max_height = self.parameter('max_height', 1.0)
        # Example body box, must be measured for each robot and sensor mount.
        self.exclusion = self.parameter('exclusion_box', [-0.35, 0.35, -0.2, 0.2, -0.5, 0.3])
        self.exclude_body = self.parameter('enable_self_filter', True)
        limits = [self.min_range, self.max_range, self.min_height, self.max_height]
        if (not self.base_frame or not all(math.isfinite(x) for x in limits)
                or not 0 <= self.min_range < self.max_range
                or self.min_height >= self.max_height):
            raise ValueError('Invalid base frame, range or height bounds')
        if (len(self.exclusion) != 6 or not all(math.isfinite(x) for x in self.exclusion)
                or any(self.exclusion[i] >= self.exclusion[i+1] for i in (0, 2, 4))):
            raise ValueError('exclusion_box must contain xmin,xmax,ymin,ymax,zmin,zmax')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def inspect(self):
        values, issues = self.health()
        values['base_frame'] = self.base_frame
        values['tf_available'] = False
        if self.latest is None:
            return values, issues, None, True
        msg = self.latest
        try:
            xyz = cloud_xyz(msg)
            values['source_cloud'] = cloud_summary(xyz)
            if values['source_cloud']['invalid_count']:
                issues.append('Cloud includes nonfinite XYZ values')
            if not values['source_cloud']['finite_count']:
                issues.append('No finite XYZ observations')
            if not msg.header.frame_id:
                raise ValueError('Missing source frame: transformation unavailable')
            if msg.header.frame_id != self.base_frame:
                if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
                    raise ValueError('Zero timestamp: refusing latest-TF substitution')
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
                t, q = tf.transform.translation, tf.transform.rotation
                xyz = transform_xyz(xyz, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            values['tf_available'] = True
            values['base_cloud'] = cloud_summary(xyz)
            filtered = filter_xyz(xyz, self.min_range, self.max_range,
                                  self.min_height, self.max_height,
                                  self.exclusion if self.exclude_body else [])
            values['filtered_point_count'] = len(filtered)
            if not len(filtered):
                issues.append('No points in configured region; coverage UNKNOWN')
            return values, issues, filtered, False
        except (ValueError, TypeError, BufferError, TransformException) as exc:
            values['analysis_available'] = False
            issues.append(str(exc))
            return values, issues, None, True

    def report(self):
        values, issues, _, error = self.inspect()
        self.publish_report(values, issues, error)


def main(args=None):
    run(LidarDiagnostic, args)
