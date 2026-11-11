#!/usr/bin/env python3
"""Bounded, odometry-closed-loop forward move through the existing Twist bridge.

Stops on: target distance reached, a lidar obstacle in the forward corridor,
or the hard time limit -- whichever comes first. Odometry-only (no lidar
topic configured) is allowed via --no-lidar-safety.
"""
import argparse
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from keyboard_teleop import validate_mode
from lumen_sensor_tests.analysis import cloud_xyz

PUBLISH_PERIOD_SEC = 0.05
# Forward safety corridor: ahead of the robot, narrow enough to ignore side
# clutter, spanning floor-clearance to a bit above the body.
SAFETY_RANGE_M = 0.4
SAFETY_HALF_WIDTH_M = 0.2
SAFETY_MIN_HEIGHT_M = -0.1
SAFETY_MAX_HEIGHT_M = 0.5
SAFETY_MIN_POINTS = 3


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--speed', type=float, default=0.4,
                        help='Forward linear.x in m/s (default 0.4)')
    parser.add_argument('--distance', type=float, default=1.0,
                        help='Target forward distance in metres (default 1.0)')
    parser.add_argument('--odom-topic', default='/utlidar/robot_odom')
    parser.add_argument('--cloud-topic', default='/lumen/retimed/cloud_base')
    parser.add_argument('--cmd-vel-topic', default='/lumen/teleop/cmd_vel',
                        help='Twist topic to publish to (default /lumen/teleop/cmd_vel)')
    parser.add_argument('--no-lidar-safety', action='store_true',
                        help='Skip the forward-corridor obstacle check (odometry only)')
    parser.add_argument('--max-duration', type=float, default=15.0,
                        help='Hard time limit in seconds (default 15.0)')
    args = parser.parse_args()
    validate_mode(args.live, os.environ.get('ROS_DOMAIN_ID'))
    if not 0.0 < args.speed <= 0.5:
        raise SystemExit('--speed must be > 0 and <= 0.5 m/s for this bounded trial')
    if not 0.0 < args.distance <= 3.0:
        raise SystemExit('--distance must be > 0 and <= 3.0 m for this bounded trial')
    if not 0.0 < args.max_duration <= 30.0:
        raise SystemExit('--max-duration must be > 0 and <= 30.0 s for this bounded trial')

    rclpy.init()
    node = rclpy.create_node('lumen_move_distance')
    pub = node.create_publisher(Twist, args.cmd_vel_topic, 1)
    state = {'origin': None, 'traveled': 0.0, 'obstacle': False, 'odom_count': 0}

    def on_odom(msg):
        p = msg.pose.pose.position
        if state['origin'] is None:
            state['origin'] = (p.x, p.y)
            return
        ox, oy = state['origin']
        state['traveled'] = ((p.x - ox) ** 2 + (p.y - oy) ** 2) ** 0.5
        state['odom_count'] += 1

    def on_cloud(msg):
        if state['obstacle']:
            return
        try:
            xyz = cloud_xyz(msg)
        except ValueError:
            return
        finite = xyz[np.isfinite(xyz).all(axis=1)]
        ahead = finite[(finite[:, 0] > 0) & (finite[:, 0] < SAFETY_RANGE_M) &
                       (np.abs(finite[:, 1]) < SAFETY_HALF_WIDTH_M) &
                       (finite[:, 2] > SAFETY_MIN_HEIGHT_M) & (finite[:, 2] < SAFETY_MAX_HEIGHT_M)]
        if len(ahead) >= SAFETY_MIN_POINTS:
            state['obstacle'] = True

    node.create_subscription(Odometry, args.odom_topic, on_odom, 10)
    if not args.no_lidar_safety:
        node.create_subscription(PointCloud2, args.cloud_topic, on_cloud, 10)

    def send(vx):
        msg = Twist()
        msg.linear.x = vx
        pub.publish(msg)

    try:
        deadline = time.monotonic() + 5.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if pub.get_subscription_count() != 1:
            raise RuntimeError('Expected exactly one bridge subscriber; no movement sent')
        print(f'Waiting for first {args.odom_topic} reading to set the origin...', flush=True)
        deadline = time.monotonic() + 5.0
        while state['odom_count'] == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if state['odom_count'] == 0:
            raise RuntimeError('No odometry received; cannot measure distance')
        print(f'Moving forward {args.distance} m at {args.speed} m/s '
              f'(lidar safety {"off" if args.no_lidar_safety else "on"})...', flush=True)
        last_publish = 0.0
        start = time.monotonic()
        while rclpy.ok():
            if state['traveled'] >= args.distance:
                print(f'Target reached: traveled {state["traveled"]:.3f} m', flush=True)
                break
            if state['obstacle']:
                print(f'STOPPED: obstacle in forward corridor at traveled {state["traveled"]:.3f} m',
                      flush=True)
                break
            if time.monotonic() - start >= args.max_duration:
                print(f'STOPPED: max duration reached at traveled {state["traveled"]:.3f} m',
                      flush=True)
                break
            now = time.monotonic()
            if now - last_publish >= PUBLISH_PERIOD_SEC:
                send(args.speed)
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
