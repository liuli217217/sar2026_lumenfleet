#!/usr/bin/env python3
"""Keyboard control of the Go2 sport mode: stand up / recover / stand down and walking.

Publishes unitree_api/msg/Request directly (the same Move/StopMove requests as
go2_cmd_vel_bridge), so DO NOT run the bridge at the same time.
Isolated by default (domain 187, /lumen/test/sport_request); --live is required
for the physical robot (domain 0, /api/sport/request).
"""
import argparse
import os
import select
import sys
import termios
import time
import tty

import rclpy
from unitree_api.msg import Request

# Sport API ids. Move/StopMove match go2_cmd_vel_bridge. The posture ids follow Unitree's
# sport API but are NOT validated on this robot: compare with
# /opt/sar/src/unitree_ros2 (example/src/common/ros2_sport_client.cpp) before --live.
API_IDS = {'stand_up': 1004, 'stand_down': 1005, 'recovery_stand': 1006,
           'stop': 1003, 'move': 1008}
KEYS = {
    'w': (1.0, 0.0, 0.0), 's': (-1.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0), 'd': (0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 1.0), 'e': (0.0, 0.0, -1.0),
}
MAX_VX, MAX_VY, MAX_WZ = 0.4, 0.1, 0.6    # base speeds in m/s, m/s, rad/s
SCALES = (0.5, 0.75, 1.0)
MOVE_PERIOD_SEC = 0.05
POSTURE_SETTLE_SEC = 3.0   # ignore movement keys while a posture change finishes
CONFIRM_WINDOW_SEC = 3.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true',
                        help='Explicitly allow the physical robot (domain 0)')
    parser.add_argument('--request-topic', default=None,
                        help='default /api/sport/request with --live, else /lumen/test/sport_request')
    parser.add_argument('--latch-sec', type=float, default=3.0,
                        help='A movement key stops by itself after this many seconds (default 3.0)')
    args = parser.parse_args()
    expected = '0' if args.live else '187'
    if os.environ.get('ROS_DOMAIN_ID') != expected:
        raise SystemExit(f'This mode requires ROS_DOMAIN_ID={expected}.')
    if not 0.2 <= args.latch_sec <= 10.0:
        raise SystemExit('--latch-sec must be between 0.2 and 10 s')
    if not sys.stdin.isatty():
        raise SystemExit('Use an interactive terminal.')
    topic = args.request_topic or ('/api/sport/request' if args.live else '/lumen/test/sport_request')

    rclpy.init()
    node = rclpy.create_node('lumen_keyboard_control')
    if 'go2_cmd_vel_bridge' in node.get_node_names():
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit('go2_cmd_vel_bridge is running: stop it; two controllers is unsafe.')
    publisher = node.create_publisher(Request, topic, 1)

    def send(name, parameter=''):
        request = Request()
        request.header.identity.id = time.time_ns()
        request.header.identity.api_id = API_IDS[name]
        request.parameter = parameter
        publisher.publish(request)

    def stop(times=3):
        for _ in range(times):
            send('stop')
            rclpy.spin_once(node, timeout_sec=0.02)

    print(('LIVE ROBOT CONTROL' if args.live else 'ISOLATED KEYBOARD TEST') + f' -> {topic}')
    print('Ids in use (check against /opt/sar/src/unitree_ros2 first):')
    print('  ' + ', '.join(f'{name}={api_id}' for name, api_id in API_IDS.items()))
    print('u stand up | r recovery stand | j then y: stand down (robot lies down)')
    print('w/s forward/back  a/d left/right  q/e turn left/right  +/- speed')
    print(f'A movement key lasts {args.latch_sec:.1f} s unless pressed again. Space: stop. x: stop and exit')
    print('Operator keeps the physical controller in hand. Waiting up to 10 s for the robot...', flush=True)
    deadline = time.monotonic() + 10
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(f'Nothing subscribes to {topic}.')

    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)
    scale_index = len(SCALES) - 1
    active = None            # (vx, vy, wz) while a movement key is latched
    latch_until = 0.0
    blocked_until = 0.0
    confirm_until = 0.0
    last_move = 0.0
    try:
        tty.setcbreak(fd)
        termios.tcflush(fd, termios.TCIFLUSH)
        print('Ready.', flush=True)
        escape_until = 0.0
        while rclpy.ok():
            now = time.monotonic()
            ready, _, _ = select.select([sys.stdin], [], [], 0.02)
            if ready:
                key = os.read(fd, 1).decode('ascii', errors='ignore').lower()
                now = time.monotonic()
                if not key or key == 'x' or key == '\x03':
                    break
                if key == '\x1b':
                    escape_until = now + 0.2
                if now < escape_until:
                    key = ' '
                if key == 'y' and now < confirm_until:
                    confirm_until = 0.0  # one trigger per confirm; OS key-repeat must not resend it
                    active = None
                    stop()
                    send('stand_down')
                    blocked_until = now + POSTURE_SETTLE_SEC
                    print('STAND DOWN sent', flush=True)
                elif key == 'j':
                    confirm_until = now + CONFIRM_WINDOW_SEC
                    print('Press y within 3 s to confirm STAND DOWN', flush=True)
                elif key in ('u', 'r'):
                    if now < blocked_until:
                        print('Posture change in progress; ignoring repeated key', flush=True)
                    else:
                        active = None
                        stop()
                        name = 'stand_up' if key == 'u' else 'recovery_stand'
                        send(name)
                        blocked_until = now + POSTURE_SETTLE_SEC
                        print(f'{name.upper()} sent; movement keys blocked for {POSTURE_SETTLE_SEC:.0f} s',
                              flush=True)
                elif key in ('+', '='):
                    scale_index = min(len(SCALES) - 1, scale_index + 1)
                    print(f'speed scale {SCALES[scale_index]:.2f}x', flush=True)
                elif key == '-':
                    scale_index = max(0, scale_index - 1)
                    print(f'speed scale {SCALES[scale_index]:.2f}x', flush=True)
                elif key in KEYS:
                    if now < blocked_until:
                        print('Posture change in progress; movement ignored', flush=True)
                    else:
                        scale = SCALES[scale_index]
                        unit = KEYS[key]
                        active = (unit[0] * MAX_VX * scale, unit[1] * MAX_VY * scale,
                                  unit[2] * MAX_WZ * scale)
                        latch_until = now + args.latch_sec
                        print(f'MOVE {key!r} vx/vy/wz = {active}', flush=True)
                elif active is not None:
                    active = None
                    stop()
                    print('STOP', flush=True)
            if active is not None:
                if now >= latch_until:
                    active = None
                    stop()
                    print('STOP (latch expired)', flush=True)
                elif now - last_move >= MOVE_PERIOD_SEC:
                    send('move', '{"x":%.6f,"y":%.6f,"z":%.6f}' % active)
                    last_move = now
            rclpy.spin_once(node, timeout_sec=0)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, original)
        if rclpy.ok():
            stop(5)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
