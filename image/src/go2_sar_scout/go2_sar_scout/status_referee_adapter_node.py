#!/usr/bin/env python3
"""Let reactive_explorer be scored by the same referee as the other algorithms.

mission_monitor scores on two strings published to /sar/events --
MISSION_START, then SIT <name> each time a victim is reached. sar_explorer
and sar_navigator publish those directly; reactive_explorer predates that
convention and only reports progress on /sar/status, as

    state=EXPLORE victims=2/4 pose=(1.23, -0.45, 90deg)

so without this it would score 0/4 however well it performed.

This watches that counter and emits an event each time it goes up. It only
subscribes; it cannot send a goal, a velocity or a service call, so being
measured cannot change how the mission behaves. The referee checks the
robot's true distance to each victim before awarding anything, so a wrong
count here cannot manufacture a score.
"""

import re
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

STATUS = re.compile(r'state=(\S+)\s+victims=(\d+)/(\d+)')


class StatusRefereeAdapter(Node):
    def __init__(self):
        super().__init__('status_referee_adapter')
        self.status_topic = self.declare_parameter('status_topic', '/sar/status').value
        self.events_topic = self.declare_parameter('events_topic', '/sar/events').value
        self.label = self.declare_parameter(
            'mission_label', 'reactive_explorer: no map, no planner').value

        self.events = self.create_publisher(String, self.events_topic, 10)
        self.create_subscription(String, self.status_topic, self.on_status, 10)
        self.started = False
        self.count = 0
        self.get_logger().info(f'{self.status_topic} -> {self.events_topic}')

    def emit(self, text):
        self.get_logger().info(f'EVENT {text}')
        self.events.publish(String(data=text))

    def on_status(self, msg):
        m = STATUS.search(msg.data)
        if not m:
            return
        state, found = m.group(1), int(m.group(2))
        # WAIT is the idle state before the gait controller is up; the clock
        # must not start on mere liveness.
        if not self.started and state != 'WAIT':
            self.started = True
            self.emit(f'MISSION_START {self.label}')
        while found > self.count:
            self.count += 1
            if not self.started:
                self.started = True
                self.emit(f'MISSION_START {self.label}')
            self.emit(f'SIT track_{self.count} reported by reactive_explorer')


def main(args=None):
    rclpy.init(args=args)
    node = StatusRefereeAdapter()
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
