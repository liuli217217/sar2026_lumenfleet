"""Mission referee: the only node that uses ground truth ("GPS").

Subscribes /odom/ground_truth (Gazebo), /sar/pose (the robot's own estimate) and
/sar/events. Every time the robot sits down it checks, with the true robot
position and the true victim bodies, whether a victim really is within
`success_distance` (1 m) of the robot centre. At the end it prints the score.
"""
import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class MissionMonitor(Node):
    def __init__(self):
        super().__init__('mission_monitor')
        mission = os.path.join(get_package_share_directory('disaster_models'), 'missions', 'maze_4x4')
        self.declare_parameter('ground_truth_yaml', os.path.join(mission, 'victims_ground_truth.yaml'))
        self.declare_parameter('ground_truth_topic', '/odom/ground_truth')
        self.declare_parameter('success_distance', 1.0)
        self.declare_parameter('time_limit', 180.0)
        with open(self.get_parameter('ground_truth_yaml').value) as f:
            gt = yaml.safe_load(f)
        self.length, self.half_width = float(gt['length']), float(gt['half_width'])
        self.victims = gt['victims']
        self.success_distance = self.get_parameter('success_distance').value
        self.time_limit = self.get_parameter('time_limit').value

        self.truth = None
        self.estimate = None
        self.errors = []
        self.start = None
        self.found = {}          # victim name -> (time, distance)
        self.sits = 0
        self.finished = False
        self.create_subscription(Odometry, self.get_parameter('ground_truth_topic').value, self._on_truth, 10)
        self.create_subscription(PoseStamped, '/sar/pose', self._on_estimate, 10)
        self.create_subscription(String, '/sar/events', self._on_event, 10)
        self.report_pub = self.create_publisher(String, '/sar/monitor', 10)
        self.create_timer(5.0, self._periodic)
        self.get_logger().info(f'monitoring {len(self.victims)} victims (ground truth), '
                               f'success < {self.success_distance} m, limit {self.time_limit:.0f} s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_truth(self, msg):
        p = msg.pose.pose.position
        self.truth = (p.x, p.y)

    def _on_estimate(self, msg):
        self.estimate = (msg.pose.position.x, msg.pose.position.y)
        if self.truth is not None and self.start is not None:
            self.errors.append(math.hypot(self.estimate[0] - self.truth[0], self.estimate[1] - self.truth[1]))

    def body_distance(self, v, px, py):
        """Distance from (px, py) to the victim body (oriented rectangle)."""
        a = math.radians(v['head_deg'])
        ux, uy = math.cos(a), math.sin(a)
        dx, dy = px - v['x'], py - v['y']
        along, across = dx * ux + dy * uy, -dx * uy + dy * ux
        # per-victim size if given (sitting and lying bodies differ), else the global one
        ea = max(abs(along) - float(v.get('length', self.length)) / 2, 0.0)
        ec = max(abs(across) - float(v.get('half_width', self.half_width)), 0.0)
        return math.hypot(ea, ec)

    def _on_event(self, msg):
        text = msg.data
        now = self._now()
        if text.startswith('MISSION_START'):
            self.start = now
            self._log(f'mission started; {text}')
        elif text.startswith('SIT') and self.truth is not None:
            self.sits += 1
            t = now - self.start if self.start else 0.0
            claimed = text.split()[1]
            dist = {v['name']: self.body_distance(v, *self.truth) for v in self.victims}
            # Credit the victim the robot says it found if it really is within reach
            # (victims can lie close together); otherwise the nearest one not yet counted.
            if claimed in dist and dist[claimed] < self.success_distance:
                name = claimed
            else:
                open_victims = [n for n in dist if n not in self.found] or list(dist)
                name = min(open_victims, key=dist.get)
            d = dist[name]
            if d < self.success_distance and t <= self.time_limit and name not in self.found:
                self.found[name] = (t, d)
                verdict = 'SUCCESS'
            elif d < self.success_distance and name in self.found:
                verdict = 'DUPLICATE'
            else:
                verdict = 'FAIL'
            others = ', '.join(f'{n} {v:.2f} m' for n, v in sorted(dist.items(), key=lambda kv: kv[1])[:2])
            self._log(f'{verdict}: {name} (robot claimed {claimed}); robot sat at true '
                      f'({self.truth[0]:.2f}, {self.truth[1]:.2f}), t={t:.1f} s; nearest bodies: {others}')
        elif text.startswith(('DONE', 'TIME_UP', 'GIVE_UP', 'CONFIRMED')):
            self._log(f'robot: {text}')
            if text.startswith('DONE'):
                self._summary()

    def _periodic(self):
        if self.start is None or self.finished:
            return
        if self._now() - self.start > self.time_limit + 5.0:
            self._summary()

    def _summary(self):
        if self.finished:
            return
        self.finished = True
        err = np.array(self.errors) if self.errors else np.array([math.nan])
        lines = [f'SCORE {len(self.found)}/{len(self.victims)} victims found within '
                 f'{self.time_limit:.0f} s ({self.sits} sit-downs)']
        for v in self.victims:
            if v['name'] in self.found:
                t, d = self.found[v['name']]
                lines.append(f'  {v["name"]}: found at t={t:.1f} s, {d:.2f} m away')
            else:
                lines.append(f'  {v["name"]}: not found')
        lines.append(f'  localization error (estimate vs ground truth): mean {np.nanmean(err):.2f} m, '
                     f'max {np.nanmax(err):.2f} m')
        self._log('\n'.join(lines))

    def _log(self, text):
        self.get_logger().info(text)
        self.report_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._summary()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
