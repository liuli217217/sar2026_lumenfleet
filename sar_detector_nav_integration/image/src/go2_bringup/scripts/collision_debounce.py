#!/usr/bin/env python3
"""Release-hysteresis layer between collision_monitor and the cmd_vel bridge.

collision_monitor re-evaluates its StopZone/SlowZone decision from scratch
every scan cycle, with no memory of the previous cycle. When the robot's
distance to a real obstacle hovers right at one of those polygon edges, the
decision flips every cycle. Confirmed via stationary_scan_01 and
baseline_open_turn_01 bag analysis (2026-11-09): the SlowZone/clear boundary
chattered first; after raising SlowZone's min_points, the StopZone/SlowZone
boundary chattered instead at close range -- same mechanism, different edge.
No installed collision_monitor parameter provides release hysteresis
(confirmed against `ros2 param list /collision_monitor` and the Jazzy docs),
so this adds it externally instead of patching collision_monitor itself --
its own safety detection and velocity scaling stay untouched; this node only
decides when to let a change in that output through.

Rule, safety-first: escalating to a MORE restrictive state (DO_NOTHING ->
SLOWDOWN -> STOP) passes through immediately, every time, never delayed.
Easing to a LESS restrictive state only takes effect once that lower
severity has been continuously reported by collision_monitor_state for
`release_dwell_sec`; until then, the last command published at the current,
more restrictive severity is repeated instead of forwarding the fluctuating
one. A stop (or any new restriction) is therefore never delayed -- only its
release is.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState


SEVERITY = {
    CollisionMonitorState.DO_NOTHING: 0,
    CollisionMonitorState.SLOWDOWN: 1,
    CollisionMonitorState.STOP: 2,
}
MOST_RESTRICTIVE = 2


class CollisionDebounce(Node):
    def __init__(self):
        super().__init__('collision_debounce')

        self.release_dwell = self.declare_parameter('release_dwell_sec', 0.5).value
        self.input_timeout = self.declare_parameter('input_timeout_sec', 0.5).value
        in_topic = self.declare_parameter('cmd_vel_in_topic', 'cmd_vel').value
        out_topic = self.declare_parameter('cmd_vel_out_topic', 'cmd_vel_debounced').value
        state_topic = self.declare_parameter('state_topic', 'collision_monitor_state').value

        self.current_severity = 0
        self.held_twist = Twist()
        self.candidate_severity = None
        self.candidate_since = None
        self.last_state_severity = 0
        self.last_cmd_time = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

        self.pub = self.create_publisher(Twist, out_topic, qos)
        self.create_subscription(CollisionMonitorState, state_topic, self.on_state, qos)
        self.create_subscription(Twist, in_topic, self.on_cmd_vel, qos)
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            f'collision_debounce: {in_topic} -> {out_topic}, '
            f'release_dwell_sec={self.release_dwell}')

    def on_state(self, msg):
        # Unknown/unconfigured action types fail safe to most restrictive.
        self.last_state_severity = SEVERITY.get(msg.action_type, MOST_RESTRICTIVE)

    def on_cmd_vel(self, msg):
        now = self.get_clock().now()
        self.last_cmd_time = now
        incoming = self.last_state_severity

        if incoming >= self.current_severity:
            self.current_severity = incoming
            self.candidate_severity = None
            self.candidate_since = None
            self.held_twist = msg
            self.pub.publish(msg)
            return

        if self.candidate_severity != incoming:
            self.candidate_severity = incoming
            self.candidate_since = now

        elapsed = (now - self.candidate_since).nanoseconds / 1e9
        if elapsed >= self.release_dwell:
            self.current_severity = incoming
            self.candidate_severity = None
            self.candidate_since = None
            self.held_twist = msg
            self.pub.publish(msg)
        else:
            self.pub.publish(self.held_twist)

    def watchdog(self):
        # Fail-safe: if collision_monitor's output goes stale, stop rather
        # than keep repeating a possibly-nonzero held command forever.
        if self.last_cmd_time is None:
            return
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age > self.input_timeout:
            if self.current_severity != MOST_RESTRICTIVE or self.candidate_severity is not None:
                self.get_logger().warn(
                    f'collision_debounce: no cmd_vel from collision_monitor for {age:.2f}s '
                    f'(timeout {self.input_timeout}s) -- forcing stop')
            self.held_twist = Twist()
            self.current_severity = MOST_RESTRICTIVE
            self.candidate_severity = None
            self.candidate_since = None
            self.pub.publish(self.held_twist)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDebounce()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
