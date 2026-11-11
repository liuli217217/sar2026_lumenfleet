"""Explicit arrival-time workaround, preserving raw topics and all other fields."""

import json
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_msgs.msg import Header

from .common import run


def restamp(message, stamp):
    """Republish with a new header.stamp without deep-copying the payload.
    copy.deepcopy() on an Image/PointCloud2 recursively walks its whole
    object graph, including the raw .data bytes -- expensive enough at
    sensor rate to measurably contend with everything else on the CPU. We
    only ever change the stamp, so every other field (including .data,
    which is immutable anyway) is safe to share by reference; only .header
    needs to be a genuinely new object so mutating its .stamp doesn't touch
    the original.
    """
    result = type(message)()
    for field in message.get_fields_and_field_types():
        setattr(result, field, getattr(message, field))
    result.header = Header(frame_id=message.header.frame_id, stamp=stamp)
    return result


class ReceiveTimeAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__('receive_time_adapter', **kwargs)
        self.enabled = self.param('enabled', False)
        self.stale = self.param('stale_timeout_sec', 1.0)
        rate = self.param('report_rate_hz', 1.0)
        if any(not math.isfinite(v) or v <= 0 for v in (self.stale, rate)):
            raise ValueError('Timeout and report rate must be positive and finite')
        reliability = self.param('input_reliability', 'best_effort')
        if reliability not in ('best_effort', 'reliable'):
            raise ValueError('input_reliability must be best_effort or reliable')
        input_qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                               reliability=(ReliabilityPolicy.BEST_EFFORT if
                                            reliability == 'best_effort' else ReliabilityPolicy.RELIABLE))
        # Reliable output matches either kind of downstream subscriber. Depth 1
        # bounds queued samples; it does not prove the source isn't buffering.
        output_qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                                reliability=ReliabilityPolicy.RELIABLE)
        pairs = []
        for kind, typ in [('cloud', PointCloud2), ('imu', Imu), ('odom', Odometry),
                          ('rgb', Image), ('depth', Image), ('rgb_info', CameraInfo)]:
            source = self.param(kind + '_input', '')
            target = self.param(kind + '_output', '/lumen/retimed/' + kind)
            if source:
                pairs.append((kind, typ, self.resolve_topic_name(source),
                              self.resolve_topic_name(target)))
        sources = [p[2] for p in pairs]
        targets = [p[3] for p in pairs]
        if (set(sources) & set(targets) or len(set(sources)) != len(sources)
                or len(set(targets)) != len(targets)):
            raise ValueError('Resolved input/output topics must be distinct; feedback loops are forbidden')
        if self.enabled and not pairs:
            raise ValueError('Enabled adapter needs at least one configured input topic')
        self.streams = {}
        self.diag = self.create_publisher(DiagnosticArray, '/lumen/diagnostics', 10)
        if self.enabled:
            for kind, typ, source, target in pairs:
                self.streams[kind] = {
                    'input': source, 'output': target, 'received': 0, 'published': 0,
                    'last_arrival': None, 'last_original_stamp_ns': None,
                    'original_offset_sec': None, 'repeated_stamps': 0,
                    'backward_stamps': 0, 'zero_stamps': 0, 'receiver_clock_resets': 0,
                    'last_output_stamp_ns': None, 'clock_valid': True,
                    'publisher': self.create_publisher(typ, target, output_qos),
                }
                self.create_subscription(typ, source, lambda msg, k=kind: self.receive(k, msg), input_qos)
            self.get_logger().warn('Receive-time stamping enabled: acquisition freshness UNVERIFIED; no TF is published.')
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0/rate, self.report, clock=self.steady)

    def param(self, name, default):
        return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value

    def receive(self, kind, message):
        # Capture receive time immediately, before copying a potentially large cloud.
        now = self.get_clock().now()
        s = self.streams[kind]
        original = message.header.stamp.sec*1000000000 + message.header.stamp.nanosec
        previous = s['last_original_stamp_ns']
        s['received'] += 1
        s['last_arrival'] = time.monotonic()
        s['zero_stamps'] += int(original <= 0)
        if previous is not None and previous > 0 and original > 0:
            s['repeated_stamps'] += int(original == previous)
            s['backward_stamps'] += int(original < previous)
        s['last_original_stamp_ns'] = original
        s['original_offset_sec'] = (now.nanoseconds-original)/1e9 if original > 0 else None
        previous_output = s['last_output_stamp_ns']
        s['clock_valid'] = now.nanoseconds > 0 and (
            previous_output is None or now.nanoseconds > previous_output)
        if not s['clock_valid']:
            s['receiver_clock_resets'] += 1
            return  # Do not forge monotonic time after a backward/paused clock.
        s['publisher'].publish(restamp(message, now.to_msg()))
        s['last_output_stamp_ns'] = now.nanoseconds
        s['published'] += 1

    def report(self):
        report = DiagnosticArray()
        report.header.stamp = self.get_clock().now().to_msg()
        if not self.enabled:
            status = DiagnosticStatus(name=self.get_fully_qualified_name(),
                                      level=DiagnosticStatus.OK,
                                      message='DISABLED: no sensor subscriptions or outputs')
            report.status = [status]
        for kind, s in self.streams.items():
            age = None if s['last_arrival'] is None else time.monotonic()-s['last_arrival']
            unavailable = age is None or age > self.stale or not s['clock_valid']
            values = {k: v for k, v in s.items() if k not in ('publisher', 'last_arrival')}
            values.update(timestamp_source='receive_time', acquisition_freshness='UNVERIFIED',
                          arrival_age_sec=age, arrivals_fresh=age is not None and age <= self.stale)
            status = DiagnosticStatus(
                name=self.get_fully_qualified_name() + '/' + kind, hardware_id=s['input'],
                level=DiagnosticStatus.ERROR if unavailable else DiagnosticStatus.WARN,
                message=('Input missing/stale or receiver clock invalid; no synthetic keepalive' if unavailable else
                         'RECEIVE_TIME: acquisition freshness UNVERIFIED; inspect original stamp counters'))
            status.values = [KeyValue(key=k, value=json.dumps(v, allow_nan=False)) for k, v in values.items()]
            report.status.append(status)
        self.diag.publish(report)


def main(args=None):
    run(ReceiveTimeAdapter, args)
