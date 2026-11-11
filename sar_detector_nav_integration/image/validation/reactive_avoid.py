#!/usr/bin/env python3
"""Bounded reactive trial: move forward until a lidar obstacle in the forward
corridor, stop, then rotate. Two independent safety nets: a move-phase hard
time/distance limit (stops even if nothing is ever detected) and a
rotate-phase hard time limit. Odometry-closed-loop for both distance and yaw.
"""
import argparse
import math
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
SAFETY_HALF_WIDTH_M = 0.2
SAFETY_MIN_HEIGHT_M = -0.1
SAFETY_MAX_HEIGHT_M = 0.5
SAFETY_MIN_POINTS = 3
# /lumen/retimed/cloud_base reports points relative to base_link's origin, not
# the physical front edge -- the lidar mount itself sits ~0.289m forward of
# base_link (go2_description's radar_joint), so a stop-range measured from
# base_link overstates real clearance by roughly that much.


def yaw_of(orientation):
    o = orientation
    return math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))


def wrap_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--speed', type=float, default=0.4,
                        help='Forward linear.x in m/s during the move phase (default 0.4)')
    parser.add_argument('--stop-range', type=float, default=0.6,
                        help='Forward-corridor obstacle range in metres, measured from '
                             'base_link (default 0.6, leaving ~0.3m at the actual front edge)')
    parser.add_argument('--rotate-safety-range', type=float, default=0.3,
                        help='Abort the rotate phase if anything is within this radius of '
                             'base_link in any direction, not just ahead (default 0.5)')
    parser.add_argument('--max-distance', type=float, default=3.0,
                        help='Give up (no rotate) if this far with no obstacle (default 3.0 m)')
    parser.add_argument('--move-timeout', type=float, default=15.0,
                        help='Hard time limit for the move phase (default 15.0 s)')
    parser.add_argument('--angle', type=float, default=90.0,
                        help='Target yaw change in degrees, signed (default +90)')
    parser.add_argument('--angular-speed', type=float, default=0.6,
                        help='|angular.z| in rad/s (default 0.6)')
    parser.add_argument('--rotate-timeout', type=float, default=15.0,
                        help='Hard time limit for the rotate phase (default 15.0 s)')
    parser.add_argument('--odom-topic', default='/utlidar/robot_odom')
    parser.add_argument('--cloud-topic', default='/lumen/retimed/cloud_base')
    parser.add_argument('--cmd-vel-topic', default='/lumen/teleop/cmd_vel',
                        help='Twist topic to publish to (default /lumen/teleop/cmd_vel)')
    args = parser.parse_args()
    validate_mode(args.live, os.environ.get('ROS_DOMAIN_ID'))
    if not 0.0 < args.speed <= 0.5:
        raise SystemExit('--speed must be > 0 and <= 0.5 m/s for this bounded trial')
    if not 0.0 < args.stop_range <= 1.5:
        raise SystemExit('--stop-range must be > 0 and <= 1.5 m')
    if not 0.0 < args.rotate_safety_range <= 1.5:
        raise SystemExit('--rotate-safety-range must be > 0 and <= 1.5 m')
    if not 0.0 < args.max_distance <= 5.0:
        raise SystemExit('--max-distance must be > 0 and <= 5.0 m for this bounded trial')
    if not 0.0 < abs(args.angle) <= 180.0:
        raise SystemExit('--angle must have magnitude > 0 and <= 180 degrees')
    if not 0.0 < args.angular_speed <= 0.8:
        raise SystemExit('--angular-speed must be > 0 and <= 0.8 rad/s for this bounded trial')
    for name, value in [('move-timeout', args.move_timeout), ('rotate-timeout', args.rotate_timeout)]:
        if not 0.0 < value <= 30.0:
            raise SystemExit(f'--{name} must be > 0 and <= 30.0 s for this bounded trial')

    target_rad = math.radians(args.angle)
    wz = math.copysign(args.angular_speed, target_rad)

    rclpy.init()
    node = rclpy.create_node('lumen_reactive_avoid')
    pub = node.create_publisher(Twist, args.cmd_vel_topic, 1)
    state = {'origin_xy': None, 'traveled': 0.0, 'obstacle': False, 'close_now': False,
             'origin_yaw': None, 'turned': 0.0, 'odom_count': 0, 'cloud_count': 0}

    def on_odom(msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        if state['origin_xy'] is None:
            state['origin_xy'] = (p.x, p.y)
            state['origin_yaw'] = yaw_of(o)
            state['odom_count'] += 1
            return
        ox, oy = state['origin_xy']
        state['traveled'] = ((p.x - ox) ** 2 + (p.y - oy) ** 2) ** 0.5
        state['turned'] = wrap_pi(yaw_of(o) - state['origin_yaw'])
        state['odom_count'] += 1

    def on_cloud(msg):
        state['cloud_count'] += 1
        try:
            xyz = cloud_xyz(msg)
        except ValueError:
            return
        finite = xyz[np.isfinite(xyz).all(axis=1)]
        in_height = (finite[:, 2] > SAFETY_MIN_HEIGHT_M) & (finite[:, 2] < SAFETY_MAX_HEIGHT_M)
        # Move phase: narrow forward corridor, latched once tripped -- the
        # move loop and the later decision to rotate both depend on this
        # still being true after the robot has already stopped.
        if not state['obstacle']:
            ahead = finite[in_height & (finite[:, 0] > 0) & (finite[:, 0] < args.stop_range) &
                           (np.abs(finite[:, 1]) < SAFETY_HALF_WIDTH_M)]
            if len(ahead) >= SAFETY_MIN_POINTS:
                state['obstacle'] = True
        # Rotate phase: a turn swings the body sideways, not just forward, so
        # this checks a radius in every direction and is never latched --
        # it must reflect what's close right now, not what was close once.
        radius = np.hypot(finite[:, 0], finite[:, 1])
        nearby = finite[in_height & (radius < args.rotate_safety_range)]
        state['close_now'] = len(nearby) >= SAFETY_MIN_POINTS

    node.create_subscription(Odometry, args.odom_topic, on_odom, 10)
    node.create_subscription(PointCloud2, args.cloud_topic, on_cloud, 10)

    def send(vx, wz_):
        msg = Twist()
        msg.linear.x, msg.angular.z = vx, wz_
        pub.publish(msg)

    def wait_for_ready():
        deadline = time.monotonic() + 5.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if pub.get_subscription_count() != 1:
            raise RuntimeError('Expected exactly one bridge subscriber; no movement sent')
        deadline = time.monotonic() + 5.0
        while state['odom_count'] == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if state['odom_count'] == 0:
            raise RuntimeError('No odometry received; cannot measure distance/yaw')
        deadline = time.monotonic() + 5.0
        while state['cloud_count'] == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if state['cloud_count'] == 0:
            raise RuntimeError(f'No point cloud received on {args.cloud_topic}; is the '
                                'retime adapter actually running? Refusing to drive blind.')

    rotated = False
    try:
        wait_for_ready()
        print(f'Move phase: forward at {args.speed} m/s, stop-range {args.stop_range} m, '
              f'give up at {args.max_distance} m / {args.move_timeout} s', flush=True)
        last_publish, start = 0.0, time.monotonic()
        while rclpy.ok():
            if state['obstacle']:
                print(f'Obstacle detected at traveled {state["traveled"]:.3f} m', flush=True)
                break
            if state['traveled'] >= args.max_distance:
                print(f'No obstacle found within {args.max_distance} m; stopping', flush=True)
                break
            if time.monotonic() - start >= args.move_timeout:
                print(f'Move timeout at traveled {state["traveled"]:.3f} m; stopping', flush=True)
                break
            now = time.monotonic()
            if now - last_publish >= PUBLISH_PERIOD_SEC:
                send(args.speed, 0.0)
                last_publish = now
            rclpy.spin_once(node, timeout_sec=0.02)

        print('Sending zero velocity for one second before rotating', flush=True)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and rclpy.ok():
            send(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)

        if not state['obstacle']:
            print('No obstacle was detected; skipping the rotate phase.', flush=True)
        else:
            rotated = True
            # on_odom only re-establishes the origin when origin_xy is None,
            # so both must be cleared together or origin_yaw stays None while
            # the arithmetic below still tries to use it.
            state['origin_xy'] = None
            state['origin_yaw'] = None
            state['odom_count'] = 0
            deadline = time.monotonic() + 5.0
            while state['odom_count'] == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            print(f'Rotate phase: {args.angle} deg at {args.angular_speed} rad/s', flush=True)
            last_publish, start = 0.0, time.monotonic()
            while rclpy.ok():
                if state['close_now']:
                    print(f'ABORTED rotation: obstacle within {args.rotate_safety_range} m, '
                          f'turned {math.degrees(state["turned"]):.1f} deg so far', flush=True)
                    break
                if abs(state['turned']) >= abs(target_rad):
                    print(f'Target reached: turned {math.degrees(state["turned"]):.1f} deg', flush=True)
                    break
                if time.monotonic() - start >= args.rotate_timeout:
                    print(f'Rotate timeout at turned {math.degrees(state["turned"]):.1f} deg',
                          flush=True)
                    break
                now = time.monotonic()
                if now - last_publish >= PUBLISH_PERIOD_SEC:
                    send(0.0, wz)
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
                send(0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()
    print(f'Finished (rotated: {rotated}). Verify physical stop.', flush=True)


if __name__ == '__main__':
    main()
