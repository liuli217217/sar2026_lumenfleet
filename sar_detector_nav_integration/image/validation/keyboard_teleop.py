#!/usr/bin/env python3
"""Keyboard trial; isolated by default, live domain 0 requires --live."""
import argparse
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist

DEFAULT_TOPIC = '/lumen/teleop/cmd_vel'
ZERO = (0.0, 0.0, 0.0)
KEYS = {
    'w': (0.2, 0.0, 0.0), 's': (-0.2, 0.0, 0.0),
    'a': (0.0, 0.05, 0.0), 'd': (0.0, -0.05, 0.0),
    'q': (0.0, 0.0, 0.15), 'e': (0.0, 0.0, -0.15),
}
SCALE_KEYS = {'+': 1, '=': 1, '-': -1}
SCALE_STEP = 0.1
MIN_SCALE, MAX_SCALE = 0.5, 1.5
# Well under the bridge's 0.25 s cmd_vel timeout, so a latched command
# never lapses into an unintended StopMove while this script is alive.
PUBLISH_PERIOD_SEC = 0.05


def validate_mode(live, domain):
    expected = '0' if live else '187'
    if domain != expected:
        raise ValueError(f'This mode requires ROS_DOMAIN_ID={expected}.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true',
                        help='Explicitly allow physical robot domain 0')
    parser.add_argument('--cmd-vel-topic', default=DEFAULT_TOPIC,
                        help=f'Twist topic to publish to (default {DEFAULT_TOPIC})')
    args = parser.parse_args()
    try:
        validate_mode(args.live, os.environ.get('ROS_DOMAIN_ID'))
    except ValueError as error:
        raise SystemExit(str(error))
    if not sys.stdin.isatty():
        raise SystemExit('Use an interactive terminal: docker exec -it ...')
    rclpy.init()
    node = rclpy.create_node('lumen_keyboard_test')
    publisher = node.create_publisher(Twist, args.cmd_vel_topic, 1)
    active_key = None
    scale = 1.0

    def current_velocity():
        if active_key is None:
            return ZERO
        return tuple(v * scale for v in KEYS[active_key])

    def send(velocity):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = velocity
        publisher.publish(msg)

    print('LIVE ROBOT CONTROL — keys can move the robot' if args.live else
          'ISOLATED KEYBOARD TEST — keep bridge output on /lumen/test/sport_request')
    print('W/S forward/back; A/D left/right; Q/E turn left/right')
    print('One press LATCHES that velocity; it keeps publishing until you stop it.')
    print('+/- : raise/lower speed scale (0.5x-1.5x) of the currently latched command')
    print('Space or any other key: STOP; X or Ctrl+C: stop and exit')
    print('Releasing the key does NOT stop the robot — press Space to stop.')
    print('Waiting up to 10 seconds for bridge subscription...', flush=True)
    deadline = time.monotonic() + 10
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit('No bridge subscriber. Start the intended bridge first.')

    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        # Discard keys typed during discovery; do not execute queued input.
        termios.tcflush(fd, termios.TCIFLUSH)
        print(f'Ready. speed scale: {scale:.2f}x. Tap a direction key; Space stops.', flush=True)
        escape_until = 0.0
        last_publish = 0.0
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.02)
            if ready:
                key = os.read(fd, 1).decode('ascii', errors='ignore').lower()
                if not key or key == 'x' or key == '\x03':
                    break
                if key == '\x1b':
                    escape_until = time.monotonic() + 0.2
                if time.monotonic() < escape_until:
                    key = ' '  # Ignore arrow/function-key escape sequences.
                if key in SCALE_KEYS:
                    scale = min(MAX_SCALE, max(MIN_SCALE,
                                round(scale + SCALE_KEYS[key] * SCALE_STEP, 2)))
                    print(f'speed scale: {scale:.2f}x', flush=True)
                elif key in KEYS:
                    active_key = key
                    print(f'LATCHED {key!r} -> vx/vy/wz: {current_velocity()} '
                          '(Space to stop)', flush=True)
                elif active_key is not None:
                    active_key = None
                    print('STOP (latch cleared)', flush=True)
            now = time.monotonic()
            if now - last_publish >= PUBLISH_PERIOD_SEC:
                send(current_velocity())
                last_publish = now
            rclpy.spin_once(node, timeout_sec=0)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, original)
        # Best-effort exit stop. Bridge timeout is the fallback if delivery fails.
        if rclpy.ok():
            for _ in range(3):
                send(ZERO)
                rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
