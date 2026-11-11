#!/usr/bin/env python3
"""Bounded, odometry-closed-loop rotation through the existing Twist bridge.

Stops on: target yaw change reached, or the hard time limit -- whichever
comes first. Positive --angle turns left (CCW, +angular.z); negative turns
right (CW).
"""
import argparse
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from keyboard_teleop import validate_mode

PUBLISH_PERIOD_SEC = 0.05


def yaw_of(orientation):
    o = orientation
    return math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))


def wrap_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--angular-speed', type=float, default=0.6,
                        help='|angular.z| in rad/s (default 0.6)')
    parser.add_argument('--angle', type=float, default=90.0,
                        help='Target yaw change in degrees, signed (default +90)')
    parser.add_argument('--odom-topic', default='/utlidar/robot_odom')
    parser.add_argument('--cmd-vel-topic', default='/lumen/teleop/cmd_vel',
                        help='Twist topic to publish to (default /lumen/teleop/cmd_vel)')
    parser.add_argument('--max-duration', type=float, default=15.0,
                        help='Hard time limit in seconds (default 15.0)')
    args = parser.parse_args()
    validate_mode(args.live, os.environ.get('ROS_DOMAIN_ID'))
    if not 0.0 < args.angular_speed <= 0.8:
        raise SystemExit('--angular-speed must be > 0 and <= 0.8 rad/s for this bounded trial')
    if not 0.0 < abs(args.angle) <= 180.0:
        raise SystemExit('--angle must have magnitude > 0 and <= 180 degrees')
    if not 0.0 < args.max_duration <= 30.0:
        raise SystemExit('--max-duration must be > 0 and <= 30.0 s for this bounded trial')

    target_rad = math.radians(args.angle)
    wz = math.copysign(args.angular_speed, target_rad)

    rclpy.init()
    node = rclpy.create_node('lumen_rotate_angle')
    pub = node.create_publisher(Twist, args.cmd_vel_topic, 1)
    state = {'origin_yaw': None, 'turned': 0.0, 'odom_count': 0}

    def on_odom(msg):
        yaw = yaw_of(msg.pose.pose.orientation)
        if state['origin_yaw'] is None:
            state['origin_yaw'] = yaw
            return
        state['turned'] = wrap_pi(yaw - state['origin_yaw'])
        state['odom_count'] += 1

    node.create_subscription(Odometry, args.odom_topic, on_odom, 10)

    def send(angular_z):
        msg = Twist()
        msg.angular.z = angular_z
        pub.publish(msg)

    try:
        deadline = time.monotonic() + 5.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if pub.get_subscription_count() != 1:
            raise RuntimeError('Expected exactly one bridge subscriber; no movement sent')
        print(f'Waiting for first {args.odom_topic} reading to set the origin yaw...', flush=True)
        deadline = time.monotonic() + 5.0
        while state['odom_count'] == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if state['odom_count'] == 0:
            raise RuntimeError('No odometry received; cannot measure rotation')
        print(f'Rotating {args.angle} deg at {args.angular_speed} rad/s...', flush=True)
        last_publish = 0.0
        start = time.monotonic()
        while rclpy.ok():
            if abs(state['turned']) >= abs(target_rad):
                print(f'Target reached: turned {math.degrees(state["turned"]):.1f} deg', flush=True)
                break
            if time.monotonic() - start >= args.max_duration:
                print(f'STOPPED: max duration reached at turned '
                      f'{math.degrees(state["turned"]):.1f} deg', flush=True)
                break
            now = time.monotonic()
            if now - last_publish >= PUBLISH_PERIOD_SEC:
                send(wz)
                last_publish = now
            rclpy.spin_once(node, timeout_sec=0.02)
    except (KeyboardInterrupt, RuntimeError) as error:
        if isinstance(error, RuntimeError):
            print(f'ABORTED: {error}', flush=True)
    finally:
        if rclpy.ok():
            print('Sending zero velocity for one second', flush=True)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and rclpy.ok():
                send(0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()
    print('Finished. Verify physical stop.', flush=True)


if __name__ == '__main__':
    main()
