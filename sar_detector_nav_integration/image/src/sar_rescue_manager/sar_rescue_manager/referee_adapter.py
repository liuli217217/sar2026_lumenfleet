#!/usr/bin/env python3
"""DETECTOR_NAV_INTEGRATION: speak the referee's event language.

`mission_monitor` (go2_reactive_map) is the referee the other two SAR
algorithms are scored by. Its contract is two strings on `/sar/events`:

    MISSION_START <anything>      starts its clock
    SIT <name> <anything>         claims a victim was reached

It does not take the claim on trust -- on every SIT it reads the robot's
true pose from Gazebo, measures the distance to each victim's ground-truth
body rectangle, and awards credit only if the robot really is within
`success_distance` and still inside `time_limit`. `<name>` is a hint: if it
does not match a victim within reach, the referee falls back to the nearest
victim not yet credited. So the ids used here do not have to line up with
the referee's victim names, and deliberately do not pretend to.

This workflow had no way to be scored by it. Its completion signal is a
JSON ledger (`completion_mode: 'mock'`), never an event string -- so the
referee would have recorded 0/4 no matter how well the stack performed, and
the three algorithms could not be compared at all.

Why a separate node instead of two publish calls inside rescue_manager:
that node owns a safety-relevant state machine with no-ROS unit tests, and
nothing about being measured should be able to change how it behaves. This
one only listens. It derives everything from `/sar_rescue/status`, which
rescue_manager already publishes every tick, and it cannot send a goal, a
velocity or a service call.

A mock rescue is not a real sit-down. What this reports is "the stack drove
to the victim and declared the rescue complete there", which is what the
referee actually measures anyway (it checks position, not posture). Said
plainly so no one reads a score from here as proof the robot sat.
"""

import json
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String


class RefereeAdapter(Node):
    def __init__(self):
        super().__init__('referee_adapter')

        self.status_topic = self.declare_parameter(
            'status_topic', '/sar_rescue/status').value
        self.events_topic = self.declare_parameter(
            'events_topic', '/sar/events').value
        # Printed after MISSION_START, for the log only.
        self.label = self.declare_parameter(
            'mission_label', 'slam+navigation (detector + Nav2, mock rescue)').value

        # Must match rescue_manager's publisher, which is TRANSIENT_LOCAL depth 1
        # -- a VOLATILE subscription here would silently receive nothing.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.events = self.create_publisher(String, self.events_topic, 10)
        self.create_subscription(String, self.status_topic, self.on_status, latched)

        self.started = False
        self.reported = set()     # record ids already announced

        self.get_logger().info(
            f'referee_adapter: {self.status_topic} -> {self.events_topic}')

    def emit(self, text):
        self.get_logger().info(f'EVENT {text}')
        self.events.publish(String(data=text))

    def on_status(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError as exc:
            self.get_logger().warn(f'unparseable status: {exc}')
            return
        if not isinstance(status, dict):
            return

        state = status.get('state')
        # DISABLED is the idle state before the operator enables the mission,
        # and rescue_manager publishes status from the first tick, so the
        # clock must not start on mere liveness.
        if not self.started and state not in (None, 'DISABLED'):
            self.started = True
            self.emit(f'MISSION_START {self.label}')

        for record in status.get('targets') or []:
            if not isinstance(record, dict) or record.get('state') != 'COMPLETED':
                continue
            rid = record.get('id')
            if rid is None or rid in self.reported:
                continue
            self.reported.add(rid)
            if not self.started:
                # Completed before any non-DISABLED status reached us (a
                # resumed ledger, or a dropped first message). Start the
                # clock rather than drop the credit.
                self.started = True
                self.emit(f'MISSION_START {self.label}')
            pos = record.get('position') or []
            where = (f' at ({pos[0]:.2f}, {pos[1]:.2f})'
                     if len(pos) >= 2 and all(isinstance(v, (int, float)) for v in pos[:2])
                     else '')
            self.emit(f'SIT track_{rid}{where} completion_mode='
                      f'{record.get("completion_mode", "unknown")}')


def main(args=None):
    rclpy.init(args=args)
    node = RefereeAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
