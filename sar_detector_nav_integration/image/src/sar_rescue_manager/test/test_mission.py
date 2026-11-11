"""DETECTOR_NAV_INTEGRATION: real mission code with fake ROS/action transports.

No ROS installed here: extract the actual node class, substitute message/action
factories, and exercise its callbacks. This is not a DDS/ROS integration test.
"""
import ast
from concurrent.futures import Future
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
import uuid

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from sar_rescue_manager.core import Registry, cell, approach_candidates, apply_costmap_update
from sar_rescue_manager.action_slot import ActionSlot


def grid(width=100, resolution=.1):
    return NS(header=NS(frame_id='map'), data=[0]*(width*width),
              info=NS(width=width, height=width, resolution=resolution,
                      origin=NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))))


def observation(stamp, point=(3., 3., .5), label='mannequin'):
    return dict(class_id=label, position=list(point), last_stamp_ns=stamp, confidence=.8)


class FakeHandle:
    def __init__(self):
        self.accepted = True
        self.result = Future()
        self.cancel_ack = Future()
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_ack


class FakeClient:
    def __init__(self):
        self.requests = []

    def send_goal_async(self, goal):
        pending = Future()
        self.requests.append((goal, pending))
        return pending

    def accept(self):
        handle = FakeHandle()
        self.requests[-1][1].set_result(handle)
        return handle

    def server_is_ready(self):
        return True


class RegistryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name)/'ledger.json'
        self.registry = Registry(self.path, 'map-a')

    def tearDown(self):
        self.temp.cleanup()

    def confirm(self):
        for n in range(3):
            stamp = 100_000_000_000+n*300_000_000
            self.registry.observe([observation(stamp)], stamp)
        return self.registry.records[0]

    def test_confirmation_requires_distinct_observations(self):
        stamp = 100_000_000_000
        for _ in range(10):
            self.registry.observe([observation(stamp)], stamp)
        self.assertEqual(self.registry.records[0]['hits'], 1)
        self.assertEqual(self.registry.records[0]['state'], 'CANDIDATE')
        self.confirm()
        self.assertEqual(self.registry.records[0]['state'], 'CONFIRMED')

    def test_completion_survives_restart_and_track_changes(self):
        record = self.confirm()
        self.registry.start(record)
        self.registry.complete(record, 101_000_000_000)
        restored = Registry(self.path, 'map-a')
        restored.observe([observation(102_000_000_000, (3.1, 3., .5))], 102_000_000_000)
        self.assertEqual(len(restored.records), 1)
        self.assertIsNone(restored.choose((0, 0), 102_000_000_000))
        self.assertEqual(restored.records[0]['completion_mode'], 'mock')

    def test_completion_write_failure_does_not_mark_complete(self):
        record = self.confirm()
        self.registry.start(record)
        def fail():
            raise OSError('disk full')
        self.registry.save = fail
        with self.assertRaises(OSError):
            self.registry.complete(record, 101_000_000_000)
        self.assertEqual(record['state'], 'IN_PROGRESS')

    def test_mission_mismatch_and_in_progress_restart(self):
        record = self.confirm()
        self.registry.start(record)
        restored = Registry(self.path, 'map-a')
        self.assertEqual(restored.records[0]['state'], 'DEFERRED')
        with self.assertRaises(ValueError):
            Registry(self.path, 'another-map')

    def test_retry_cooldown_limit_and_stale_rejection(self):
        record = self.confirm()
        now = record['last_stamp_ns']
        self.registry.start(record)
        self.registry.defer(record, 'unreachable', now, cooldown=1.)
        self.assertIsNone(self.registry.choose((0, 0), now))
        self.assertIs(self.registry.choose((0, 0), now+1_000_000_000), record)
        self.registry.start(record)
        self.registry.defer(record, 'again', now, cooldown=1.)
        self.assertIsNone(self.registry.choose((0, 0), now+1_000_000_000))
        self.registry.observe([observation(now-10_000_000_000, (8, 8, 0)),
                               observation(now, (8, 8, 0), 'cone')], now)
        self.assertEqual(len(self.registry.records), 1)


class GeometryTests(unittest.TestCase):
    def test_candidates_are_standoff_and_face_target(self):
        g = grid()
        points = approach_candidates(g, (3., 3., 0.), (1., 3.))
        self.assertEqual(len(points), 12)
        self.assertAlmostEqual(points[0][0], 2.)
        for x, y, yaw in points:
            self.assertAlmostEqual(math.hypot(x-3, y-3), 1.)
            self.assertAlmostEqual(math.cos(yaw), (3-x))

    def test_unknown_and_obstacles_block_footprint(self):
        g = grid()
        g.data = [-1]*len(g.data)
        self.assertFalse(approach_candidates(g, (3, 3, 0), (1, 3)))
        g.data = [100]*len(g.data)
        self.assertFalse(approach_candidates(g, (3, 3, 0), (1, 3)))

    def test_origin_rotation_and_negative_floor(self):
        g = grid(width=10, resolution=1.)
        self.assertIsNone(cell(g, -.01, .5))
        g.info.origin.orientation.z = math.sqrt(.5)
        g.info.origin.orientation.w = math.sqrt(.5)
        g.data[2*10+1] = 77
        self.assertEqual(cell(g, -2.5, 1.5), 77)

    def test_incremental_costmap_update(self):
        g = grid(width=5, resolution=1.)
        update = NS(header=NS(frame_id='map'), x=1, y=2, width=2, height=1, data=[100, -1])
        apply_costmap_update(g, update)
        self.assertEqual(cell(g, 1.5, 2.5), 100)
        self.assertEqual(cell(g, 2.5, 2.5), -1)
        update.width = 20
        with self.assertRaises(ValueError):
            apply_costmap_update(g, update)


class ActionTests(unittest.TestCase):
    def test_pending_accept_cancel_ack_does_not_release_ownership(self):
        client, results, faults = FakeClient(), [], []
        slot = ActionSlot(client, lambda *args: results.append(args), faults.append)
        slot.send(NS(), 'explore')
        slot.cancel()
        self.assertTrue(slot.busy)
        with self.assertRaises(RuntimeError):
            slot.send(NS(), 'rescue')
        handle = client.accept()
        self.assertEqual(handle.cancel_calls, 1)
        handle.cancel_ack.set_result(NS(goals_canceling=[1]))
        self.assertTrue(slot.busy)
        handle.result.set_result(NS(status=5, result=None))
        self.assertFalse(slot.busy)
        self.assertEqual(results[0][:2], ('explore', 5))
        self.assertFalse(faults)

    def test_rejection_and_unknown_cancel_timeout(self):
        client, faults = FakeClient(), []
        slot = ActionSlot(client, lambda *args: None, faults.append)
        slot.send(NS(), 'explore')
        client.requests[-1][1].set_result(NS(accepted=False))
        self.assertFalse(slot.busy)
        slot.send(NS(), 'explore')
        slot.cancel()
        slot.cancel_started -= 10.
        slot.check(20., 5.)
        self.assertTrue(slot.busy)
        self.assertTrue(faults)


def manager_class():
    tree = ast.parse((PACKAGE/'sar_rescue_manager/node.py').read_text())
    definition = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RescueManager')
    scope = dict(Node=object, time=time, json=json, math=math, uuid=uuid,
                 NavigateToPose=NS(Goal=NS), ComputePathToPose=NS(Goal=NS),
                 String=lambda **kwargs: NS(**kwargs), approach_candidates=approach_candidates,
                 footprint_clear=lambda *args: True)
    exec(compile(ast.Module(body=[definition], type_ignores=[]), '<actual manager>', 'exec'), scope)
    return scope['RescueManager']


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        cls = manager_class()
        self.node = cls.__new__(cls)
        n = self.node
        n.registry = Registry(Path(self.temp.name)/'ledger.json', 'test')
        self.clock = 101_000_000_000
        n.now_ns = lambda: self.clock
        n.get_logger = lambda: NS(info=lambda *a: None, warn=lambda *a, **kw: None)
        n.enabled, n.state, n.reason = True, 'EXPLORING', ''
        n.entered = time.monotonic()
        n.active = None
        n.candidate = None
        n.candidates = []
        n.action_token = None
        n.standoff, n.radius, n.max_attempts, n.cooldown = 1., .35, 2, 30.
        n.nav_timeout, n.verify_timeout, n.stopped_dwell = 60., 12., 1.
        n.ready = lambda: True
        n.pose = lambda: (1., 3.)
        n.grid = grid()
        n.pose_message = lambda c: NS(candidate=c)
        n.nav_client, n.path_client = FakeClient(), FakeClient()
        n.nav = ActionSlot(n.nav_client, n.navigation_result, n.fault)
        n.planner = ActionSlot(n.path_client, n.path_result, n.fault)
        self.requests = []
        n.action_request = NS(publish=lambda message: self.requests.append(json.loads(message.data)))
        for i in range(3):
            stamp = self.clock-600_000_000+i*300_000_000
            n.registry.observe([observation(stamp)], stamp)

    def tearDown(self):
        self.temp.cleanup()

    def approach(self):
        n = self.node
        n._tick()
        self.assertEqual(n.state, 'PAUSING_EXPLORATION')
        n._tick()
        self.assertEqual(n.state, 'SELECTING_APPROACH')
        n._tick()
        handle = n.path_client.accept()
        end = NS(pose=NS(position=NS(x=n.candidate[0], y=n.candidate[1])))
        handle.result.set_result(NS(status=4, result=NS(path=NS(header=NS(frame_id='map'), poses=[end]), error_code=0)))
        self.assertEqual(n.state, 'APPROACHING')
        return n.nav_client.accept()

    def test_full_mock_rescue_and_no_completed_revisit(self):
        n = self.node
        handle = self.approach()
        handle.result.set_result(NS(status=4, result=None))
        self.assertEqual(n.state, 'VERIFYING_TARGET')
        n.pose = lambda: n.candidate[:2]
        n.still_since = time.monotonic()-2.
        n._tick()  # no post-arrival observation yet
        self.assertFalse(self.requests)
        self.clock += 500_000_000
        n.registry.observe([observation(self.clock)], self.clock)
        n._tick()
        self.assertEqual(n.state, 'RESCUING')
        request = self.requests[0]
        n.action_result(NS(data=json.dumps(dict(token='wrong', target_id=0, mode='mock', success=True))))
        self.assertEqual(n.state, 'RESCUING')
        n.action_result(NS(data=json.dumps(dict(**request, success=True))))
        self.assertEqual(n.state, 'EXPLORING')
        self.assertEqual(n.registry.records[0]['state'], 'COMPLETED')
        n._tick()
        self.assertIsNone(n.active)
        self.assertEqual(n.state, 'EXPLORING')

    def test_victim_preempts_exploration_only_after_terminal(self):
        n = self.node
        n.nav.send(NS(), 'exploration')
        n._tick()
        self.assertEqual(n.state, 'PAUSING_EXPLORATION')
        n._tick()
        self.assertFalse(n.path_client.requests)
        handle = n.nav_client.accept()
        handle.cancel_ack.set_result(NS(goals_canceling=[1]))
        n._tick()
        self.assertFalse(n.path_client.requests)
        handle.result.set_result(NS(status=5, result=None))
        n._tick()
        self.assertEqual(n.state, 'SELECTING_APPROACH')

    def test_navigation_failure_defers_without_rescue(self):
        handle = self.approach()
        handle.result.set_result(NS(status=6, result=None))
        self.assertEqual(self.node.registry.records[0]['state'], 'DEFERRED')
        self.assertEqual(self.node.state, 'EXPLORING')
        self.assertFalse(self.requests)

    def test_disarm_during_approach_waits_for_terminal(self):
        n = self.node
        handle = self.approach()
        n.set_enabled(NS(data=False), NS())
        n._tick()
        self.assertEqual(n.state, 'STOPPING')
        handle.cancel_ack.set_result(NS(goals_canceling=[1]))
        n._tick()
        self.assertEqual(n.state, 'STOPPING')
        handle.result.set_result(NS(status=5, result=None))
        n._tick()
        self.assertEqual(n.state, 'DISABLED')
        self.assertEqual(n.registry.records[0]['state'], 'DEFERRED')

    def test_freshness_loss_faults_and_does_not_resume(self):
        n = self.node
        handle = self.approach()
        n.ready = lambda: False
        n._tick()
        self.assertEqual(n.state, 'FAULT')
        self.assertFalse(n.enabled)
        handle.result.set_result(NS(status=4, result=None))
        self.assertEqual(n.state, 'FAULT')
        self.assertFalse(self.requests)

    def test_no_reachable_approach_defers(self):
        n = self.node
        n.grid.data = [-1]*len(n.grid.data)
        n._tick()
        n._tick()
        n._tick()
        self.assertEqual(n.state, 'EXPLORING')
        self.assertEqual(n.registry.records[0]['state'], 'DEFERRED')
        self.assertFalse(n.nav_client.requests)

    def test_arrival_without_new_detection_defers(self):
        n = self.node
        self.approach().result.set_result(NS(status=4, result=None))
        n.pose = lambda: n.candidate[:2]
        n.still_since = time.monotonic()-2.
        n.entered -= 13.
        n._tick()
        self.assertEqual(n.state, 'EXPLORING')
        self.assertEqual(n.registry.records[0]['state'], 'DEFERRED')
        self.assertFalse(self.requests)

    def test_next_victim_selected_after_first_completed(self):
        n = self.node
        self.approach().result.set_result(NS(status=4, result=None))
        n.pose = lambda: n.candidate[:2]
        n.still_since = time.monotonic()-2.
        for i in range(3):
            self.clock += 300_000_000
            n.registry.observe([observation(self.clock), observation(self.clock, (5., 3., .5))], self.clock)
        n._tick()
        request = self.requests[0]
        n.action_result(NS(data=json.dumps(dict(**request, success=True))))
        n._tick()
        self.assertEqual(n.state, 'PAUSING_EXPLORATION')
        self.assertEqual(n.active['id'], 1)
        self.assertEqual(n.registry.records[0]['state'], 'COMPLETED')

    def test_movement_before_mock_reply_prevents_completion(self):
        n = self.node
        self.approach().result.set_result(NS(status=4, result=None))
        n.pose = lambda: n.candidate[:2]
        n.still_since = time.monotonic()-2.
        self.clock += 500_000_000
        n.registry.observe([observation(self.clock)], self.clock)
        n._tick()
        n.still_since = None
        n.action_result(NS(data=json.dumps(dict(**self.requests[0], success=True))))
        self.assertEqual(n.state, 'FAULT')
        self.assertEqual(n.registry.records[0]['state'], 'IN_PROGRESS')

    def test_mock_timeout_is_not_completion(self):
        n = self.node
        self.approach().result.set_result(NS(status=4, result=None))
        n.pose = lambda: n.candidate[:2]
        n.still_since = time.monotonic()-2.
        self.clock += 500_000_000
        n.registry.observe([observation(self.clock)], self.clock)
        n._tick()
        n.entered -= 11.
        n._tick()
        self.assertEqual(n.state, 'FAULT')
        self.assertNotEqual(n.registry.records[0]['state'], 'COMPLETED')


if __name__ == '__main__':
    unittest.main()
