"""Send one Unitree Go2 sport command from the command line (talks to the robot
directly, not through go2_bridge).

    ros2 run go2_real_bridge sport_cli stand_up | stand_down | balance | recovery |
                                       stop | sit | rise | damp
    ros2 run go2_real_bridge sport_cli move VX VY WZ [SECONDS]   (default 1 s, then StopMove)
    ros2 run go2_real_bridge sport_cli speed_level LEVEL           (-1 slow, 0 normal, 1 fast)

damp makes all motors limp: the robot drops to the ground. Emergency only.
"""
import sys
import time

import rclpy
from unitree_api.msg import Request

from go2_real_bridge import sport_api

SIMPLE = {'stand_up': sport_api.STAND_UP, 'stand_down': sport_api.STAND_DOWN,
          'balance': sport_api.BALANCE_STAND, 'recovery': sport_api.RECOVERY_STAND,
          'stop': sport_api.STOP_MOVE, 'sit': sport_api.SIT, 'rise': sport_api.RISE_SIT,
          'damp': sport_api.DAMP}


def main(args=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if not argv or (argv[0] not in SIMPLE and argv[0] not in ('move', 'speed_level')):
        print(__doc__)
        return 1
    rclpy.init(args=args)
    node = rclpy.create_node('sport_cli')
    pub = node.create_publisher(Request, '/api/sport/request', 10)
    time.sleep(0.5)                         # let DDS discovery find the robot
    cmd = argv[0]
    if cmd in SIMPLE:
        for _ in range(3):                  # requests are not acknowledged: send a few times
            pub.publish(sport_api.request(SIMPLE[cmd]))
            time.sleep(0.05)
        print(f'sent {sport_api.NAMES[SIMPLE[cmd]]}')
    elif cmd == 'speed_level':
        pub.publish(sport_api.request(sport_api.SPEED_LEVEL, {'data': int(argv[1])}))
        print(f'sent SpeedLevel {argv[1]}')
    else:
        vx, vy, wz = (float(v) for v in argv[1:4])
        seconds = float(argv[4]) if len(argv) > 4 else 1.0
        end = time.time() + seconds
        while time.time() < end:
            pub.publish(sport_api.move(vx, vy, wz))
            time.sleep(0.05)
        pub.publish(sport_api.request(sport_api.STOP_MOVE))
        print(f'moved ({vx}, {vy}, {wz}) for {seconds} s, then StopMove')
    time.sleep(0.2)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
