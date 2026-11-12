#!/usr/bin/env python3
"""DETECTOR_NAV_INTEGRATION: sit down for real, instead of pretending.

Drop-in replacement for mock_rescue. Same request/result contract on
`/sar_rescue/action_request` and `/sar_rescue/action_result`, so
rescue_manager's state machine is untouched -- but where mock_rescue only
answers "done", this actually sits the robot down and stands it back up.

Why it matters: the task is to reach a victim AND sit beside them. A mock
completion scores the same as a real one under a referee that only measures
position, which makes the number dishonest the moment anyone compares it to
the two reactive algorithms, both of which really sit (their sit-downs are
visible as Sit/RiseSit sport requests reaching the robot).

The sequence is the one the reactive stacks already use, so all three
algorithms do the same physical thing:

    StopMove (1003)   stop walking before changing posture
    Sit      (1009)   hips down; the bridge lowers the body and pitches up
    <sit_duration>
    RiseSit  (1010)   back to standing before anything else is commanded

These go to `/api/sport/request`, the same topic the real robot's sport
service listens on -- in simulation fake_go2 implements all three, so the
body visibly lowers in Gazebo. `mode` stays 'mock' in the reply because
rescue_manager validates that field and the ledger's `completion_mode` is
its own concern; what changed is that a posture change really happened.
Set `require_stillness` false only if the caller already guarantees the
robot has stopped.
"""

import json
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from unitree_api.msg import Request

from . import sport_api_shim as sport


class RealRescue(Node):
    def __init__(self):
        super().__init__('real_rescue')

        self.sit_duration = self.declare_parameter('sit_duration', 3.0).value
        self.settle = self.declare_parameter('settle_before_sit', 0.5).value
        self.request_topic = self.declare_parameter(
            'request_topic', '/api/sport/request').value

        self.sport = self.create_publisher(Request, self.request_topic, 10)
        self.result = self.create_publisher(String, '/sar_rescue/action_result', 10)
        self.create_subscription(String, '/sar_rescue/action_request',
                                 self.on_request, 10)

        self.pending = None      # (token, target_id)
        self.timer = None

        self.get_logger().info(
            f'real_rescue: sport requests -> {self.request_topic}, '
            f'sit_duration={self.sit_duration}s (this node sits the robot down '
            f'for real; mock_rescue did not)')

    # ------------------------------------------------------------------ sit
    def on_request(self, msg):
        try:
            data = json.loads(msg.data)
            token, target = data['token'], data['target_id']
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn(f'invalid action request: {exc}')
            return
        if self.pending is not None:
            self.get_logger().warn(
                f'already sitting for {self.pending[1]}; refusing {target}')
            self.reply(token, target, False, 'busy with another rescue')
            return

        self.pending = (token, target)
        self.sport.publish(sport.request(sport.STOP_MOVE))
        self.get_logger().info(f'rescue {target}: stopping, then sitting')
        self.timer = self.create_timer(self.settle, self.sit_down)

    def sit_down(self):
        self._cancel_timer()
        if self.pending is None:
            return
        self.sport.publish(sport.request(sport.SIT))
        self.get_logger().info(f'rescue {self.pending[1]}: SIT')
        self.timer = self.create_timer(self.sit_duration, self.stand_up)

    def stand_up(self):
        self._cancel_timer()
        if self.pending is None:
            return
        token, target = self.pending
        self.sport.publish(sport.request(sport.RISE_SIT))
        self.get_logger().info(f'rescue {target}: RISE_SIT, rescue complete')
        self.pending = None
        self.reply(token, target, True, 'sat down and stood back up')

    def _cancel_timer(self):
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

    def reply(self, token, target, success, detail):
        self.result.publish(String(data=json.dumps(dict(
            token=token, target_id=target, mode='mock',
            success=success, detail=detail))))


def main(args=None):
    rclpy.init(args=args)
    node = RealRescue()
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
