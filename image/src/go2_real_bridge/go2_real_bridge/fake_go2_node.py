"""Fake Unitree Go2 on top of the Gazebo / CHAMP simulation (simulation only).

Lets the real-robot stack (go2_bridge + the SAR nodes with the real topic names)
run unchanged in the simulation:

  sim /odom + /imu/data       -> /sportmodestate (unitree_go/SportModeState)
  sim lidar cloud             -> /utlidar/cloud
  /api/sport/request          -> Move: CHAMP /cmd_vel (stops after `cmd_timeout`),
                                 StopMove: zero velocity, Sit / RiseSit: CHAMP /body_pose
                                 (lowered and nose up / normal), other requests: logged
"""
import math

import rclpy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from unitree_api.msg import Request
from unitree_go.msg import LowState, SportModeState

from go2_real_bridge import sport_api


class FakeGo2(Node):
    def __init__(self):
        super().__init__('fake_go2')
        defaults = {
            'sim_odom': '/odom',
            'sim_imu': '/imu/data',
            'sim_cloud': '/velodyne_points/points',
            'sim_cmd_vel': '/cmd_vel',
            'sim_body_pose': '/body_pose',
            # The topic names the competition robot was observed to publish
            # (sar2026_lumenfleet, robot `seek`): the simulation is then driven
            # through exactly the same bridge inputs as the real robot.
            'state_topic': '/sportmodestate',
            'odom_topic': '/utlidar/robot_odom',
            'lowstate_topic': '/lowstate',
            'cloud_topic': '/utlidar/cloud_base',
            'request_topic': '/api/sport/request',
            'cmd_timeout': 0.5,
            'sit_height_offset': -0.08,
            'sit_pitch': -0.3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = p = {name: self.get_parameter(name).value for name in defaults}
        self.odom = None
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None
        self.state_pub = self.create_publisher(SportModeState, p['state_topic'], qos_profile_sensor_data)
        self.robot_odom_pub = self.create_publisher(Odometry, p['odom_topic'], qos_profile_sensor_data)
        self.lowstate_pub = self.create_publisher(LowState, p['lowstate_topic'], qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, p['cloud_topic'], qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, p['sim_cmd_vel'], 10)
        self.pose_pub = self.create_publisher(Pose, p['sim_body_pose'], 10)
        self.create_subscription(Odometry, p['sim_odom'], self._on_odom, 10)
        self.create_subscription(Imu, p['sim_imu'], self._on_imu, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, p['sim_cloud'], self.cloud_pub.publish, qos_profile_sensor_data)
        self.create_subscription(Request, p['request_topic'], self._on_request, 10)
        self.create_timer(0.05, self._drive)
        self.get_logger().info(f'fake Go2: simulation -> {p["state_topic"]}, {p["odom_topic"]}, '
            f'{p["lowstate_topic"]}, {p["cloud_topic"]}; {p["request_topic"]} -> CHAMP' )

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg):
        self.odom = msg

    def _on_imu(self, msg):
        if self.odom is None:
            return
        q = msg.orientation
        state = SportModeState()
        state.imu_state.quaternion = [float(q.w), float(q.x), float(q.y), float(q.z)]
        state.imu_state.rpy = [math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
                               math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
                               math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))]
        state.imu_state.gyroscope = [float(msg.angular_velocity.x), float(msg.angular_velocity.y),
                                     float(msg.angular_velocity.z)]
        state.imu_state.accelerometer = [float(msg.linear_acceleration.x), float(msg.linear_acceleration.y),
                                         float(msg.linear_acceleration.z)]
        pos, tw = self.odom.pose.pose.position, self.odom.twist.twist
        state.position = [float(pos.x), float(pos.y), float(pos.z)]
        state.velocity = [float(tw.linear.x), float(tw.linear.y), 0.0]
        state.yaw_speed = float(tw.angular.z)
        self.state_pub.publish(state)

        # Same information on the two topics the real robot actually uses, so
        # go2_bridge can be pointed at either source without touching the sim.
        robot_odom = Odometry()
        robot_odom.header = self.odom.header
        robot_odom.header.frame_id = 'odom'
        robot_odom.child_frame_id = 'base_link'
        robot_odom.pose.pose = self.odom.pose.pose
        robot_odom.pose.pose.orientation = msg.orientation       # IMU attitude, as on the Go2
        robot_odom.twist.twist = self.odom.twist.twist
        self.robot_odom_pub.publish(robot_odom)

        low = LowState()
        low.imu_state = state.imu_state
        self.lowstate_pub.publish(low)

    def _on_request(self, msg):
        api_id, params = sport_api.parse(msg)
        if api_id == sport_api.MOVE:
            self.cmd = (params.get('x', 0.0), params.get('y', 0.0), params.get('z', 0.0))
            self.cmd_time = self._now()
            return
        self.get_logger().info(f'request {sport_api.NAMES.get(api_id, api_id)} {params or ""}')
        if api_id in (sport_api.STOP_MOVE, sport_api.SIT, sport_api.DAMP, sport_api.STAND_DOWN):
            self.cmd_time = None
            self.cmd_pub.publish(Twist())
        if api_id in (sport_api.SIT, sport_api.RISE_SIT):
            down = api_id == sport_api.SIT
            pitch = self.p['sit_pitch'] if down else 0.0
            pose = Pose()
            pose.position.z = float(self.p['sit_height_offset'] if down else 0.0)
            pose.orientation.y, pose.orientation.w = math.sin(pitch / 2), math.cos(pitch / 2)
            self.pose_pub.publish(pose)

    def _drive(self):
        if self.cmd_time is None:
            return
        twist = Twist()
        if self._now() - self.cmd_time < self.p['cmd_timeout']:
            twist.linear.x, twist.linear.y, twist.angular.z = (float(v) for v in self.cmd)
        else:
            self.cmd_time = None
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FakeGo2()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
