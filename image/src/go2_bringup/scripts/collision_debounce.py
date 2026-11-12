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

Also applies a minimum-speed floor to every outgoing Twist (2026-11-10,
redesigned 2026-11-12). MPPI can choose a small-but-nonzero "optimal"
trajectory that sits below the robot's real measured minimum executable
speed (forward ~0.3-0.4, reverse ~0.4-0.5, rotation ~0.6) and produces
zero actual movement while looking like a valid command -- confirmed
live via a recorded bag at a real doorway, where this was happening on
a third of all outgoing commands, not just an edge case. The original
fix zeroed anything below that minimum outright; found live that this
just moves the failure to wherever MPPI is most cautious (tight
passages), where it keeps choosing something slower than any fixed
floor. Now: genuine near-zero noise still gets zeroed, but a real,
intentional, too-slow command gets its magnitude clamped UP to the
robot's known minimum instead of discarded -- MPPI still decides
direction and whether to move at all. The obvious place to enforce this
is go2_cmd_vel_bridge, but the Docker image's build of that node
(confirmed by diffing it against the native workspace's copy, which
does have a deadband) is a different, divergent implementation with no
deadband capability at all, and fixing that requires a C++ rebuild
rather than a hot patch. This is a safe, equivalent place to enforce it
instead: it runs before the bridge in the pipeline, held/zero commands
won't ever get mistaken for a "real" severity-change command, and the
stop-while-stale watchdog path already publishes a pure zero Twist
either way.
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
        # Redesigned 2026-11-12, found live via a recorded bag at a real
        # doorway: the original rule ("below deadband -> send zero")
        # discarded MPPI's own legitimate, deliberately-careful slow
        # commands near a tight passage -- confirmed from that bag, 759
        # of 2313 final /cmd_vel samples (33%) had both linear AND
        # angular zeroed despite real, nonzero commands upstream, not
        # just near the doorway. Lowering deadband_linear alone
        # (0.4 -> 0.3) wasn't enough either: right at the tightest part
        # of a passage, MPPI's own obstacle-proximity cost pushes it to
        # choose an even slower speed than whatever fixed floor is set,
        # so a single "below X, zero" threshold is always eventually
        # undercut exactly where progress matters most.
        #
        # Now a genuine floor, not a discard threshold: a command whose
        # magnitude is real (above noise_floor_*, filtering true sensor/
        # rounding jitter) but below deadband_* (this robot's measured
        # minimum reliable speed) is CLAMPED UP to deadband_* instead of
        # zeroed -- MPPI still decides direction and whether to move at
        # all; only a too-slow-to-execute-cleanly magnitude gets raised
        # to what the robot can actually do smoothly. Applies
        # symmetrically to linear (x, y) and angular (z).
        #
        # deadband_linear: this robot's measured forward minimum
        # (velocity_pulse.py: 0.3 min/0.4 smooth).
        # deadband_angular: 0.6 was measured for PURE in-place rotation;
        # lowered to 0.3 as a reasoned-but-unmeasured guess for combined
        # translation+rotation (a different regime -- the legs are
        # already stepping, so pure rotation's own dead zone may not
        # apply the same way). Revisit with a real combined-motion
        # measurement rather than trusting this guess long-term.
        self.deadband_linear = self.declare_parameter('deadband_linear', 0.3).value
        self.deadband_angular = self.declare_parameter('deadband_angular', 0.3).value
        self.noise_floor_linear = self.declare_parameter('noise_floor_linear', 0.05).value
        self.noise_floor_angular = self.declare_parameter('noise_floor_angular', 0.05).value
        # Found live the night before competition: collision_monitor has
        # been directly observed chattering between STOP/SLOWDOWN/normal
        # every 50-300ms at zone boundaries (documented elsewhere in this
        # file's own header). Escalation takes effect immediately, by
        # design, but de-escalation requires release_dwell_sec of
        # UNINTERRUPTED improvement -- any single flicker back to the
        # worse severity resets that timer to zero. At this chatter rate,
        # the uninterrupted window needed to de-escalate could never
        # actually complete, latching the robot at a stale held_twist
        # (often zero) indefinitely even while most readings were fine --
        # confirmed live: /cmd_vel sat at flat zero while /cmd_vel_raw
        # showed real, usable commands. max_latch_sec is a safety-
        # preserving escape hatch: never forces a release while the
        # CURRENT reading is still bad, only stops insisting on a perfect
        # uninterrupted dwell window once we've been stuck for this long
        # despite the current instant genuinely reporting improvement.
        self.max_latch_sec = self.declare_parameter('max_latch_sec', 2.0).value

        self.current_severity = 0
        self.held_twist = Twist()
        self.candidate_severity = None
        self.candidate_since = None
        self.last_state_severity = 0
        self.last_cmd_time = None
        self.severity_entered_at = self.get_clock().now()

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

    @staticmethod
    def _floor_to_minimum(value, noise_floor, minimum):
        """Below noise_floor: true noise, zero it. Between noise_floor and
        minimum: a real but too-slow-to-execute-cleanly command, clamp its
        magnitude up to minimum, keeping its sign/direction. At or above
        minimum: already fine, pass through unchanged."""
        magnitude = abs(value)
        if magnitude < noise_floor:
            return 0.0
        if magnitude < minimum:
            return minimum if value > 0 else -minimum
        return value

    def _apply_deadband(self, msg):
        out = Twist()
        out.linear.x = self._floor_to_minimum(msg.linear.x, self.noise_floor_linear, self.deadband_linear)
        out.linear.y = self._floor_to_minimum(msg.linear.y, self.noise_floor_linear, self.deadband_linear)
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = self._floor_to_minimum(msg.angular.z, self.noise_floor_angular, self.deadband_angular)
        return out

    def on_cmd_vel(self, msg):
        msg = self._apply_deadband(msg)
        now = self.get_clock().now()
        self.last_cmd_time = now
        incoming = self.last_state_severity

        if incoming >= self.current_severity:
            if incoming != self.current_severity:
                self.severity_entered_at = now
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
        latched_for = (now - self.severity_entered_at).nanoseconds / 1e9
        # Release either the normal way (this better severity held
        # uninterrupted for release_dwell_sec) or, if chatter has kept
        # that from ever completing for max_latch_sec, release anyway --
        # safe because we're already inside incoming < current_severity,
        # i.e. the CURRENT instant genuinely reports improvement.
        if elapsed >= self.release_dwell or latched_for >= self.max_latch_sec:
            self.current_severity = incoming
            self.severity_entered_at = now
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
            if self.current_severity != MOST_RESTRICTIVE:
                self.severity_entered_at = self.get_clock().now()
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
