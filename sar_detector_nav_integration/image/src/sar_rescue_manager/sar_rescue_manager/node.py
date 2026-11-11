"""DETECTOR_NAV_INTEGRATION: sole NavigateToPose owner for the managed workflow.

Requires the original hybrid_explorer/explore_lite/RViz goal tools to remain off.
This milestone executes only a simulated rescue, never a posture API command.
"""
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformException
from .action_slot import ActionSlot
from .core import Registry, approach_candidates, footprint_clear, apply_costmap_update


class RescueManager(Node):
    def __init__(self):
        super().__init__('sar_rescue_manager')
        def p(name, default):
            return self.declare_parameter(name, default).value
        self.mission_id = p('mission_id', 'test-map-001')
        self.ledger_file = p('ledger_file', '/tmp/sar_rescue_ledger.json')
        self.standoff = float(p('standoff_m', 1.0))
        self.radius = float(p('robot_clearance_radius_m', .40))
        self.max_attempts = int(p('max_attempts', 2))
        self.cooldown = float(p('retry_cooldown_sec', 30.))
        self.nav_timeout = float(p('navigation_timeout_sec', 60.))
        self.verify_timeout = float(p('verification_timeout_sec', 12.))
        self.stopped_dwell = float(p('stopped_dwell_sec', 1.))
        self.settle_speed = float(p('stopped_linear_mps', .03))
        self.settle_yaw = float(p('stopped_angular_rps', .05))
        if any(not math.isfinite(v) or v <= 0 for v in
               (self.standoff, self.radius, self.cooldown, self.nav_timeout,
                self.verify_timeout, self.stopped_dwell, self.settle_speed, self.settle_yaw)):
            raise ValueError('Mission limits must be positive and finite')
        if self.standoff <= self.radius or self.max_attempts < 1:
            raise ValueError('Standoff must exceed clearance radius and attempts >= 1')
        self.registry = Registry(self.ledger_file, self.mission_id)
        self.enabled = False
        self.state, self.reason = 'DISABLED', 'explicit enable service required'
        self.entered = time.monotonic()
        self.active = None
        self.grid = self.grid_arrival = None
        self.odom_arrival = self.still_since = None
        self.verify_start_ns = 0
        self.candidates = []
        self.candidate = None
        self.action_token = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.control = self.create_publisher(String, '/sar_rescue/exploration_control', 1)
        self.status = self.create_publisher(String, '/sar_rescue/status', latched)
        self.action_request = self.create_publisher(String, '/sar_rescue/action_request', 10)
        self.create_subscription(String, '/sar_integration/targets', self.observations, latched)
        self.create_subscription(PoseStamped, '/sar_rescue/exploration_goal', self.exploration_goal, 1)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap, latched)
        self.create_subscription(OccupancyGridUpdate, '/global_costmap/costmap_updates', self.costmap_update, 10)
        self.create_subscription(Odometry, p('odom_topic', '/utlidar/robot_odom_retimed'),
                                 self.odometry, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String, '/sar_rescue/action_result', self.action_result, 10)
        self.create_service(SetBool, '/sar_rescue/set_enabled', self.set_enabled)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.nav = ActionSlot(self.nav_client, self.navigation_result, self.fault)
        self.planner = ActionSlot(self.path_client, self.path_result, self.fault)
        self.create_timer(.2, self.tick)
        self.get_logger().info('Managed mission DISABLED. Rescue action is MOCK ONLY.')

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def transition(self, state, reason=''):
        self.state, self.reason, self.entered = state, reason, time.monotonic()
        self.get_logger().info(f'{state}: {reason}')

    def fault(self, reason):
        if self.state != 'FAULT':
            self.enabled = False
            self.transition('FAULT', reason)
            # Stop existing actions, but never release ownership on a mere cancel ACK.
            self.nav.cancel()
            self.planner.cancel()

    def costmap(self, msg):
        self.grid, self.grid_arrival = msg, time.monotonic()

    def costmap_update(self, msg):
        if self.grid is None:
            return
        try:
            apply_costmap_update(self.grid, msg)
            self.grid_arrival = time.monotonic()
        except ValueError as exc:
            self.fault(str(exc))

    def odometry(self, msg):
        stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        age = (self.now_ns()-stamp)/1e9
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        values = (v.x, v.y, v.z, w.x, w.y, w.z)
        if stamp <= 0 or not 0 <= age <= .5 or not all(math.isfinite(x) for x in values):
            self.still_since = None
            return
        now = time.monotonic()
        if self.odom_arrival is None or now-self.odom_arrival > .5:
            self.still_since = None
        self.odom_arrival = now
        if math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z) <= self.settle_speed and max(abs(w.x), abs(w.y), abs(w.z)) <= self.settle_yaw:
            if self.still_since is None:
                self.still_since = now
        else:
            self.still_since = None

    def pose(self):
        try:
            tf = self.tf.lookup_transform('map', 'base_link', Time())
            stamp = tf.header.stamp.sec*1000000000+tf.header.stamp.nanosec
            if stamp <= 0 or not 0 <= (self.now_ns()-stamp)/1e9 <= 1.:
                return None
            t = tf.transform.translation
            if not all(math.isfinite(v) for v in (t.x, t.y, t.z)):
                return None
            return t.x, t.y
        except TransformException:
            return None

    def ready(self):
        now = time.monotonic()
        return (self.grid is not None and self.grid.header.frame_id == 'map'
                and self.grid.info.width > 0 and self.grid.info.height > 0
                and math.isfinite(self.grid.info.resolution) and self.grid.info.resolution > 0
                and len(self.grid.data) == self.grid.info.width*self.grid.info.height
                and self.grid_arrival is not None and now-self.grid_arrival <= 3.
                and self.odom_arrival is not None and now-self.odom_arrival <= .5
                and self.pose() is not None)

    def observations(self, msg):
        try:
            data = json.loads(msg.data)
            if data['frame_id'] != 'map':
                raise ValueError('Target observations must be in map')
            self.registry.observe(data['targets'], self.now_ns())
        except OSError as exc:
            self.fault(f'Ledger persistence failed: {exc}')
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn(f'Invalid target observation: {exc}', throttle_duration_sec=5.)

    def set_enabled(self, request, response):
        if request.data:
            if self.state == 'FAULT':
                response.success, response.message = False, 'FAULT requires diagnosis and process restart'
            elif self.state not in ('DISABLED', 'EXPLORING') or self.nav.busy or self.planner.busy:
                response.success, response.message = False, 'Mission is busy or stopping'
            elif not self.ready() or not self.nav_client.server_is_ready() or not self.path_client.server_is_ready():
                response.success, response.message = False, 'Need fresh map TF, costmap, odometry and Nav2 actions'
            else:
                self.enabled = True
                self.transition('EXPLORING', 'operator enabled managed goals')
                response.success, response.message = True, 'Managed exploration enabled; mock rescue only'
        else:
            self.enabled = False
            if self.state != 'FAULT':
                self.transition('STOPPING', 'operator disabled mission')
            self.nav.cancel()
            self.planner.cancel()
            response.success, response.message = True, 'Stopping; inspect status for terminal DISABLED'
        return response

    def exploration_goal(self, pose):
        if not self.enabled or self.state != 'EXPLORING' or self.nav.busy or not self.ready():
            return
        age = (self.now_ns()-(pose.header.stamp.sec*1000000000+pose.header.stamp.nanosec))/1e9
        q, p = pose.pose.orientation, pose.pose.position
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if (pose.header.frame_id != 'map' or not 0 <= age <= 1.
                or not all(math.isfinite(x) for x in values)
                or abs(sum(x*x for x in (q.x, q.y, q.z, q.w))-1.) > .02):
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.nav.send(goal, 'exploration')

    def pose_message(self, candidate):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = candidate[:2]
        pose.pose.orientation.z = math.sin(candidate[2]/2)
        pose.pose.orientation.w = math.cos(candidate[2]/2)
        return pose

    def navigation_result(self, label, status, result):
        if self.state in ('FAULT', 'STOPPING', 'DISABLED'):
            return
        if self.state == 'PAUSING_EXPLORATION':
            return  # tick advances only after slot released
        if label == 'approach' and self.state == 'APPROACHING':
            if status == 4:
                self.verify_start_ns = self.now_ns()
                self.still_since = None
                self.transition('VERIFYING_TARGET', 'await stopped odometry and a new victim observation')
            else:
                self.defer(f'Approach navigation failed: action status {status}')

    def path_result(self, label, status, result):
        if self.state != 'SELECTING_APPROACH' or not self.enabled:
            return
        if time.monotonic()-self.entered > 40. or self.planner.cancel_requested:
            return
        path = getattr(result, 'path', None)
        if (status != 4 or path is None or path.header.frame_id != 'map' or not path.poses
                or getattr(result, 'error_code', 0) != 0):
            return  # next tick tries next candidate
        end = path.poses[-1].pose.position
        if (not math.isfinite(end.x) or not math.isfinite(end.y)
                or math.hypot(end.x-self.candidate[0], end.y-self.candidate[1]) > .25):
            return
        if not self.ready() or not footprint_clear(self.grid, *self.candidate[:2], self.radius):
            return
        self.transition('APPROACHING', f"target #{self.active['id']}")
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_message(self.candidate)
        self.nav.send(goal, 'approach')

    def defer(self, reason):
        try:
            if self.active is not None:
                self.registry.defer(self.active, reason, self.now_ns(), self.cooldown)
            self.active = None
            self.transition('EXPLORING' if self.enabled else 'DISABLED', reason)
        except OSError as exc:
            self.fault(f'Cannot persist deferred target: {exc}')

    def action_result(self, msg):
        if self.state != 'RESCUING':
            return
        try:
            data = json.loads(msg.data)
            if (data['token'] != self.action_token or data['target_id'] != self.active['id']
                    or data['mode'] != 'mock'):
                return
            if data['success'] is not True:
                self.fault('Simulated rescue failed; target not completed')
                return
            if (not self.ready() or self.still_since is None
                    or time.monotonic()-self.still_since < self.stopped_dwell):
                self.fault('Stop/health verification lost before mock completion')
                return
            self.registry.complete(self.active, self.now_ns())
            self.active = None
            self.action_token = None
            self.transition('EXPLORING', 'mock rescue persisted; resume exploration')
        except OSError as exc:
            self.fault(f'Cannot persist completion: {exc}')
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn(f'Invalid mock action result: {exc}')

    def tick(self):
        try:
            self._tick()
        except (OSError, ValueError, RuntimeError) as exc:
            self.fault(f'Mission operation failed: {exc}')
        # Always publish inhibition, including FAULT/STOPPING.
        self.control.publish(String(data=json.dumps(dict(
            enabled=self.enabled and self.state == 'EXPLORING', busy=self.nav.busy))))
        self.status.publish(String(data=json.dumps(dict(
            state=self.state, reason=self.reason, enabled=self.enabled, mode='mock',
            active_target=None if self.active is None else self.active['id'],
            nav_busy=self.nav.busy, planner_busy=self.planner.busy,
            targets=self.registry.records, ledger_file=str(self.ledger_file)), allow_nan=False)))

    def _tick(self):
        self.nav.check(self.nav_timeout if self.nav.label == 'approach' else 20., 5.)
        self.planner.check(5., 5.)
        if self.state == 'FAULT':
            return
        if self.state == 'STOPPING':
            if not self.nav.busy and not self.planner.busy:
                self.defer('mission disabled; active victim deferred')
            return
        if not self.enabled:
            return
        if not self.ready():
            self.fault('Fresh TF, global costmap or odometry lost')
            return
        robot = self.pose()
        if robot is None:
            self.fault('Map pose became unavailable')
            return
        if self.state == 'EXPLORING':
            record = self.registry.choose(robot, self.now_ns(), self.max_attempts)
            if record is not None:
                self.active = record
                self.registry.start(record)
                self.transition('PAUSING_EXPLORATION', f"selected target #{record['id']}")
                self.nav.cancel()
        elif self.state == 'PAUSING_EXPLORATION' and not self.nav.busy:
            self.candidates = approach_candidates(self.grid, self.active['position'], robot,
                                                  self.standoff, self.radius)
            self.transition('SELECTING_APPROACH', f'{len(self.candidates)} clear candidates')
        elif self.state == 'SELECTING_APPROACH':
            if time.monotonic()-self.entered > 40.:
                if self.planner.busy:
                    self.planner.cancel()
                else:
                    self.defer('Approach planning deadline exceeded')
            elif not self.planner.busy:
                if not self.candidates:
                    self.defer('No reachable approach candidate')
                elif self.path_client.server_is_ready():
                    self.candidate = self.candidates.pop(0)
                    goal = ComputePathToPose.Goal()
                    goal.goal = self.pose_message(self.candidate)
                    goal.use_start = False
                    self.planner.send(goal, 'approach_path')
        elif self.state == 'VERIFYING_TARGET':
            xy = robot
            arrived = self.candidate is not None and math.dist(xy, self.candidate[:2]) <= .4
            near_target = math.dist(xy, self.active['position'][:2]) <= self.standoff+.5
            still = self.still_since is not None and time.monotonic()-self.still_since >= self.stopped_dwell
            new_observation = (self.active['last_stamp_ns'] > self.verify_start_ns
                               and 0 <= (self.now_ns()-self.active['last_stamp_ns'])/1e9 <= 2.)
            if arrived and near_target and still and new_observation:
                self.action_token = uuid.uuid4().hex
                self.transition('RESCUING', 'mock only; robot remains standing')
                self.action_request.publish(String(data=json.dumps(dict(
                    token=self.action_token, target_id=self.active['id'], mode='mock'))))
            elif time.monotonic()-self.entered > self.verify_timeout:
                self.defer('Stopped pose / fresh victim verification timed out')
        elif self.state == 'RESCUING':
            if self.still_since is None:
                self.fault('Robot moved during simulated rescue')
            elif time.monotonic()-self.entered > 10.:
                self.fault('Mock action response timeout; completion not recorded')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RescueManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # Asynchronous best effort only. An external physical stop remains necessary.
            node.enabled = False
            node.nav.cancel()
            node.planner.cancel()
            node.control.publish(String(data='{"enabled":false,"busy":true}'))
            node.destroy_node()
        rclpy.try_shutdown()
