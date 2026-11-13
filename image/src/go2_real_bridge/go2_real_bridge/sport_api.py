"""Unitree Go2 high-level sport API over ROS 2 (unitree_ros2): unitree_api/msg/Request
messages on /api/sport/request, api ids and parameters as in unitree_ros2's
example/src/common/ros2_sport_client.cpp."""
import json

from unitree_api.msg import Request

DAMP = 1001             # motors limp: the robot lies down - emergency only
BALANCE_STAND = 1002
STOP_MOVE = 1003
STAND_UP = 1004
STAND_DOWN = 1005
RECOVERY_STAND = 1006
MOVE = 1008             # {"x": vx [m/s], "y": vy [m/s], "z": yaw rate [rad/s]}
SIT = 1009
RISE_SIT = 1010
SPEED_LEVEL = 1015      # {"data": -1 | 0 | 1}

NAMES = {DAMP: 'Damp', BALANCE_STAND: 'BalanceStand', STOP_MOVE: 'StopMove', STAND_UP: 'StandUp',
         STAND_DOWN: 'StandDown', RECOVERY_STAND: 'RecoveryStand', MOVE: 'Move', SIT: 'Sit',
         RISE_SIT: 'RiseSit', SPEED_LEVEL: 'SpeedLevel'}


def request(api_id, params=None):
    msg = Request()
    msg.header.identity.api_id = api_id
    if params is not None:
        msg.parameter = json.dumps(params)
    return msg


def move(vx, vy, wz):
    return request(MOVE, {'x': float(vx), 'y': float(vy), 'z': float(wz)})


def parse(msg):
    """(api_id, params dict) of a Request."""
    try:
        params = json.loads(msg.parameter) if msg.parameter else {}
    except ValueError:
        params = {}
    return msg.header.identity.api_id, params
