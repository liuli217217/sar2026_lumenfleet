#!/usr/bin/env python3
"""Bounded-duration velocity pulse, for characterizing the Go2's minimum
executable walking/turning speed in isolation from Nav2/collision_monitor.

OSCILLATION_TEST_AND_SIMULATION_PLAN.md flagged this gap directly: keyboard
teleop's short command expiry confuses startup delay with a real velocity
dead zone, and an indefinite `ros2 topic pub` has no explicit stop handling
-- neither is suitable for this. This publishes one fixed Twist at a steady
rate for exactly duration_sec, then sends a repeated, explicit stop and
exits. It is never left running indefinitely and never depends on an
external node noticing and cutting it off.

Run it against a standalone go2_cmd_vel_bridge (not through the full nav
launch), publishing to a dedicated topic, so nothing else -- no Nav2
controller, no collision_monitor -- can compete with or mask what it sends:

  ros2 run go2_cmd_vel_bridge go2_cmd_vel_bridge --ros-args \
    -p enabled:=true -p cmd_vel_topic:=/cmd_vel_test \
    -p request_topic:=/api/sport/request \
    -p max_vx:=0.20 -p max_vy:=0.10 -p max_wz:=0.30 -p cmd_timeout_s:=0.25

  ros2 run go2_bringup velocity_pulse.py --ros-args \
    -p vx:=0.10 -p duration_sec:=3.0 -p cmd_vel_topic:=cmd_vel_test
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist


class VelocityPulse(Node):
    def __init__(self):
        super().__init__('velocity_pulse')

        self.vx = self.declare_parameter('vx', 0.0).value
        self.vy = self.declare_parameter('vy', 0.0).value
        self.wz = self.declare_parameter('wz', 0.0).value
        self.duration_sec = self.declare_parameter('duration_sec', 3.0).value
        self.rate_hz = self.declare_parameter('rate_hz', 10.0).value
        self.start_delay_sec = self.declare_parameter('start_delay_sec', 2.0).value
        topic = self.declare_parameter('cmd_vel_topic', 'cmd_vel_test').value

        if self.duration_sec <= 0 or self.rate_hz <= 0:
            raise ValueError('duration_sec and rate_hz must be positive')
        if self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0:
            raise ValueError('vx, vy and wz are all zero -- nothing to pulse')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(Twist, topic, qos)

        self.get_logger().warn(
            f'velocity_pulse: will publish vx={self.vx} vy={self.vy} wz={self.wz} '
            f'to "{topic}" at {self.rate_hz}Hz for {self.duration_sec}s, '
            f'starting in {self.start_delay_sec}s -- physical controller ready.')

    def run(self):
        remaining = self.start_delay_sec
        while remaining > 0:
            self.get_logger().info(f'starting in {remaining:.0f}s...')
            step = min(1.0, remaining)
            time.sleep(step)
            remaining -= step

        cmd = Twist()
        cmd.linear.x = float(self.vx)
        cmd.linear.y = float(self.vy)
        cmd.angular.z = float(self.wz)

        self.get_logger().warn('PULSE START')
        period = 1.0 / self.rate_hz
        n_cycles = int(self.duration_sec / period)
        for _ in range(n_cycles):
            self.pub.publish(cmd)
            time.sleep(period)

        self.stop()
        self.get_logger().warn('PULSE STOP (completed normally)')

    def stop(self):
        zero = Twist()
        for _ in range(5):
            self.pub.publish(zero)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPulse()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('PULSE STOP (interrupted)')
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
