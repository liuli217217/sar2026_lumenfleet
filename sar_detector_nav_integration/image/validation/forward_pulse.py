#!/usr/bin/env python3
"""Single bounded forward trial through the existing Twist bridge."""
import argparse
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from keyboard_teleop import validate_mode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--speed', type=float, default=0.05,
                        help='Forward linear.x in m/s (default 0.05)')
    parser.add_argument('--duration', type=float, default=0.5,
                        help='How long to hold the command, in seconds (default 0.5)')
    parser.add_argument('--cmd-vel-topic', default='/lumen/teleop/cmd_vel',
                        help='Twist topic to publish to (default /lumen/teleop/cmd_vel)')
    args = parser.parse_args()
    validate_mode(args.live, os.environ.get('ROS_DOMAIN_ID'))
    if not 0.0 < args.speed <= 0.3:
        raise SystemExit('--speed must be > 0 and <= 0.3 m/s for this bounded trial')
    if not 0.0 < args.duration <= 3.0:
        raise SystemExit('--duration must be > 0 and <= 3.0 s for this bounded trial')
    rclpy.init()
    node = rclpy.create_node('lumen_forward_pulse')
    pub = node.create_publisher(Twist, args.cmd_vel_topic, 1)
    try:
        deadline = time.monotonic() + 5.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if pub.get_subscription_count() != 1:
            raise RuntimeError('Expected exactly one bridge subscriber; no movement sent')
        print(f'Forward trial starts now: {args.speed} m/s for {args.duration} seconds', flush=True)
        command = Twist()
        command.linear.x = args.speed
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline and rclpy.ok():
            pub.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            print('Sending zero velocity for one second', flush=True)
            deadline = time.monotonic() + 1.0
            try:
                while time.monotonic() < deadline and rclpy.ok():
                    pub.publish(Twist())
                    rclpy.spin_once(node, timeout_sec=0.05)
            except KeyboardInterrupt:
                pass
        node.destroy_node()
        rclpy.try_shutdown()
    print('Finished. Verify physical stop; leave bridge running until stationary.', flush=True)


if __name__ == '__main__':
    main()
