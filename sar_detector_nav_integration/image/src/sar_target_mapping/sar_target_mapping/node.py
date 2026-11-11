"""DETECTOR_NAV_INTEGRATION: observation-only mapper; never sends motion commands."""
from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from .core import stamp_ns, depth_point, transform_point, associate


class TargetMapper(Node):
    def __init__(self):
        super().__init__('sar_target_mapper')
        def p(name, default):
            return self.declare_parameter(name, default).value
        self.frame = p('target_frame', 'map')
        self.slop = float(p('sync_tolerance_sec', .08))
        self.max_age = float(p('max_observation_age_sec', 2.0))
        self.radius = float(p('merge_radius_m', .6))
        self.labels = set(p('classes', 'mannequin,cone,first_aid').split(','))
        self.path = Path(p('output_file', '/tmp/sar_targets.json')).expanduser()
        if not self.frame or any(not math.isfinite(v) or v <= 0 for v in (self.slop, self.max_age, self.radius)):
            raise ValueError('Frame and positive finite timing/distance parameters required')
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.depth = deque(maxlen=30)  # bounded memory (~1 second at 30 Hz)
        self.info = deque(maxlen=30)
        self.pending = deque(maxlen=30)
        self.records = []
        self.rejected = self.localized = self.arrays = 0
        self.last_detection = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/sar_integration/targets/markers', latched)
        self.targets = self.create_publisher(String, '/sar_integration/targets', latched)
        self.status = self.create_publisher(String, '/sar_integration/status', 1)
        self.create_subscription(Image, p('depth_topic', '/lumen/retimed/depth'), self.depth.append, sensor)
        self.create_subscription(CameraInfo, p('camera_info_topic', '/lumen/retimed/rgb_info'), self.info.append, sensor)
        self.create_subscription(Detection2DArray,
                                 p('detections_topic', '/sar_detector_integrated/detections_confirmed'),
                                 self.on_detections, 10)
        self.create_timer(.1, self.process)
        self.create_timer(2., self.report)
        self.get_logger().info('Observation-only integration: no Nav2 goals or cmd_vel. Records start fresh per run.')

    def on_detections(self, msg):
        self.last_detection = time.monotonic()
        self.arrays += 1
        if msg.detections:
            if len(self.pending) == self.pending.maxlen:
                self.rejected += len(self.pending[0][1].detections)
            self.pending.append((time.monotonic(), msg))

    def process(self):
        remaining = deque(maxlen=30)
        changed = False
        for arrived, msg in self.pending:
            stamp = stamp_ns(msg.header)
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            if stamp <= 0 or age < -self.slop or age > self.max_age or time.monotonic()-arrived > self.max_age:
                self.rejected += len(msg.detections)
                continue
            depths = [d for d in self.depth if d.header.frame_id == msg.header.frame_id
                      and abs(stamp_ns(d.header)-stamp) <= self.slop*1e9]
            # CameraInfo may be static; require same optical frame and a non-future calibration.
            infos = [i for i in self.info if i.header.frame_id == msg.header.frame_id
                     and (stamp_ns(i.header) == 0 or stamp_ns(i.header) <= stamp+self.slop*1e9)]
            if not depths or not infos:
                remaining.append((arrived, msg))
                continue
            depth = min(depths, key=lambda d: abs(stamp_ns(d.header)-stamp))
            info = min(infos, key=lambda i: abs(stamp_ns(i.header)-stamp))
            try:
                tf = self.tf.lookup_transform(self.frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
            except TransformException:
                remaining.append((arrived, msg))
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            for detection in msg.detections:
                if not detection.results:
                    continue
                h = detection.results[0].hypothesis
                if h.class_id not in self.labels or not math.isfinite(h.score):
                    continue
                b = detection.bbox
                try:
                    camera = depth_point(depth, info, (b.center.position.x, b.center.position.y, b.size_x, b.size_y))
                    point = transform_point(camera, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
                except (ValueError, TypeError, BufferError) as exc:
                    self.rejected += 1
                    self.get_logger().warn(str(exc), throttle_duration_sec=5.)
                    continue
                index = associate(self.records, h.class_id, point, self.radius)
                if index is None:
                    self.records.append(dict(id=len(self.records), class_id=h.class_id,
                                             position=list(point), observations=0, first_stamp_ns=stamp))
                    index = len(self.records)-1
                record = self.records[index]
                n = record['observations']
                record['position'] = [(old*n+new)/(n+1) for old, new in zip(record['position'], point)]
                record.update(observations=n+1, last_stamp_ns=stamp, confidence=float(h.score),
                              source_track_id=detection.id)
                self.localized += 1
                changed = True
        self.pending = remaining
        if changed:
            self.publish_records()

    def publish_records(self):
        document = dict(schema_version=1, frame_id=self.frame, targets=self.records,
                        note='Estimated object surface locations; not navigation goals. No loop-closure correction.')
        data = json.dumps(document, ensure_ascii=False, allow_nan=False, indent=2)
        self.targets.publish(String(data=data))
        markers = MarkerArray()
        for record in self.records:
            for label in (False, True):
                m = Marker()
                m.header.frame_id = self.frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'sar_target_labels' if label else 'sar_targets'
                m.id = record['id']
                m.type = Marker.TEXT_VIEW_FACING if label else Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = record['position']
                m.pose.orientation.w = 1.
                m.scale.x = m.scale.y = m.scale.z = .2
                m.color.a = 1.
                m.color.r = 1. if record['class_id'] == 'mannequin' else .2
                m.color.g = .3 if record['class_id'] == 'mannequin' else 1.
                if label:
                    m.pose.position.z += .3
                    m.text = f"{record['class_id']} #{record['id']} ({record['observations']})"
                markers.markers.append(m)
        self.markers.publish(markers)
        try:
            temporary = self.path.with_suffix(self.path.suffix+'.tmp')
            temporary.write_text(data+'\n')
            temporary.replace(self.path)
        except OSError as exc:
            self.get_logger().error(f'Cannot save targets: {exc}', throttle_duration_sec=5.)

    def report(self):
        self.status.publish(String(data=json.dumps(dict(
            detection_arrays=self.arrays, localized=self.localized, rejected=self.rejected,
            pending=len(self.pending), depth_cached=len(self.depth), camera_info_cached=len(self.info),
            detection_arrival_age_sec=None if self.last_detection is None else time.monotonic()-self.last_detection,
            unique_targets=len(self.records)))))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetMapper()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
