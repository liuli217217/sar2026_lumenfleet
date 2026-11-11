"""Shared passive node support and machine-readable diagnostic publication."""

import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from .analysis import ArrivalWindow, timestamp_health


class SensorDiagnostic(Node):
    def __init__(self, name, topic_parameter, message_type):
        super().__init__(name)
        self.topic = self.parameter(topic_parameter, '')
        if not self.topic.strip():
            raise ValueError(f'{topic_parameter} must explicitly name a sensor topic')
        self.stale_timeout = self.parameter('stale_timeout_sec', 1.0)
        self.expected_rate = self.parameter('expected_rate_hz', 0.0)
        self.future_tolerance = self.parameter('future_tolerance_sec', 0.1)
        report_hz = self.parameter('report_rate_hz', 1.0)
        for name, value in [('stale_timeout_sec', self.stale_timeout),
                            ('report_rate_hz', report_hz)]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if (not math.isfinite(self.expected_rate) or self.expected_rate < 0
                or not math.isfinite(self.future_tolerance) or self.future_tolerance < 0):
            raise ValueError('Expected rate and future tolerance must be finite and nonnegative')
        reliability = self.parameter('qos_reliability', 'best_effort')
        if reliability not in ('best_effort', 'reliable'):
            raise ValueError('qos_reliability must be best_effort or reliable')
        qos = QoSProfile(depth=5, durability=DurabilityPolicy.VOLATILE,
                         reliability=(ReliabilityPolicy.BEST_EFFORT
                                      if reliability == 'best_effort'
                                      else ReliabilityPolicy.RELIABLE))
        self.arrivals = ArrivalWindow()
        self.latest = None
        self.received = 0
        self.publisher = self.create_publisher(DiagnosticArray, '/lumen/diagnostics', 10)
        self.subscription = self.create_subscription(message_type, self.topic, self.receive, qos)
        # Continue reporting missing data even if a simulated clock stops.
        self.report_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / report_hz, self.report, clock=self.report_clock)

    def parameter(self, name, default):
        return self.declare_parameter(
            name, default, ParameterDescriptor(read_only=True)).value

    def receive(self, msg):
        self.latest = msg
        self.received += 1
        self.arrivals.add(time.monotonic())

    def health(self):
        values = {'topic': self.topic, 'received_count': self.received,
                  **self.arrivals.summary(time.monotonic())}
        if self.latest is None:
            return values, ['No messages received; check topic, publisher, DDS and QoS']
        stamp = self.latest.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        age, issues = timestamp_health(
            self.get_clock().now().nanoseconds * 1e-9, stamp_sec,
            values['arrival_age_sec'], self.stale_timeout, self.future_tolerance)
        values.update(frame_id=self.latest.header.frame_id,
                      stamp_sec=stamp.sec, stamp_nanosec=stamp.nanosec,
                      measurement_age_sec=age, fresh=not issues)
        if not self.latest.header.frame_id:
            issues.append('Missing frame ID')
        if (self.expected_rate and values['rate_hz'] is not None
                and values['rate_hz'] < self.expected_rate * 0.8):
            issues.append('Arrival rate below 80% of expected rate')
        return values, issues

    def publish_report(self, values, issues, error=False):
        status = DiagnosticStatus()
        status.name = self.get_fully_qualified_name()
        status.hardware_id = self.topic
        status.level = (DiagnosticStatus.ERROR if error else
                        DiagnosticStatus.WARN if issues else DiagnosticStatus.OK)
        status.message = '; '.join(issues) if issues else 'Measurements available'
        status.values = [KeyValue(key=k, value=json.dumps(v, allow_nan=False))
                         for k, v in values.items()]
        report = DiagnosticArray()
        report.header.stamp = self.get_clock().now().to_msg()
        report.status = [status]
        self.publisher.publish(report)
        self.get_logger().info(json.dumps(
            {'status': status.message, **values}, allow_nan=False, sort_keys=True))


def run(factory, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
