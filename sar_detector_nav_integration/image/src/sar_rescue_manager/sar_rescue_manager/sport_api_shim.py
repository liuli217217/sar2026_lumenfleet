"""DETECTOR_NAV_INTEGRATION: the three sport ids real_rescue.py needs.

A deliberate, minimal copy of go2_real_bridge/sport_api.py rather than a
dependency on it. That package is not part of this workspace (it lives in
image/src/ of the robot workspace and is not built here), and pulling it in
just to post three integers would drag the whole real-robot bridge, its
fake_go2 simulator and their dependencies into a build that does not need
them.

Ids come from unitree_ros2's example/src/common/ros2_sport_client.cpp and
match go2_real_bridge/sport_api.py exactly. They are protocol constants on
the robot's side, so they do not drift; if that ever changes, the two files
disagreeing is the thing to look for.
"""

import json

from unitree_api.msg import Request

STOP_MOVE = 1003
SIT = 1009
RISE_SIT = 1010

NAMES = {STOP_MOVE: 'StopMove', SIT: 'Sit', RISE_SIT: 'RiseSit'}


def request(api_id, params=None):
    msg = Request()
    msg.header.identity.api_id = api_id
    if params is not None:
        msg.parameter = json.dumps(params)
    return msg
