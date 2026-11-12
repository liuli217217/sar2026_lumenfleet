"""Bridge between the real Unitree Go2 and the search-and-rescue nodes.

The SAR nodes (go2_sar_prior sar_prior, go2_sar_slam sar_slam) were written
against the simulation (CHAMP): /odom, /imu/data, a lidar cloud, /cmd_vel and
/body_pose. On the real Go2 this node provides the same interface:

  /sportmodestate (unitree_go/SportModeState) -> odom_out (nav_msgs/Odometry, the
                       robot's own leg + IMU odometry) and imu_out (sensor_msgs/Imu)
  odom_in / imu_in / lowstate_in              -> the same two outputs from the topics the
                       competition robot was actually observed to publish, for firmware
                       that has no /sportmodestate. sar2026_lumenfleet measured
                       /utlidar/robot_odom (nav_msgs/Odometry, ~150 Hz) and /lowstate
                       (unitree_go/LowState, carries imu_state) on robot `seek`, and its
                       own driver uses those two, never /sportmodestate. Set whichever
                       exist; each output has exactly one source (see _pick_sources).
  /utlidar/cloud_base (built-in L1 lidar)     -> cloud_out, re-stamped with this
                       computer's clock (the Go2 and the Jetson clocks are not synchronised;
                       the SAR nodes match clouds to IMU samples by time stamp)
  cmd_vel_in (geometry_msgs/Twist)            -> sport API Move (clamped), StopMove when
                       no command arrived for `cmd_timeout` (watchdog)
  body_pose_in (geometry_msgs/Pose)           -> sport API Sit when the body is lowered
                       (z < sit_threshold, the SAR nodes' "sit down" pose), RiseSit back
  estop_topic (std_msgs/Bool)                 -> true: StopMove and ignore all motion
                       commands until false

enable_motion=false (the default) is a dry run: everything is computed and logged,
but no motion request is sent to the robot.
"""
import time

import rclpy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool
from unitree_api.msg import Request
from unitree_go.msg import LowState, SportModeState

from go2_real_bridge import sport_api


class Go2Bridge(Node):
    def __init__(self):
        super().__init__('go2_bridge')
        defaults = {
            'state_topic': '/sportmodestate',
            'request_topic': '/api/sport/request',
            'cloud_in': '/utlidar/cloud_base',
            # Alternative state sources; empty = derive everything from state_topic.
            'odom_in': '',
            'imu_in': '',
            'lowstate_in': '',
            'cloud_out': '/go2/cloud',
            'odom_out': '/go2/odom',
            'imu_out': '/go2/imu',
            'cmd_vel_in': '/sar/cmd_vel',
            'body_pose_in': '/sar/body_pose',
            'estop_topic': '/sar/estop',
            'enable_motion': False,         # false: dry run, nothing is sent to the robot
            'max_vx': 0.5,                  # [m/s] clamps (the SAR nodes command <= 0.3)
            'max_vy': 0.3,
            'max_wz': 0.8,                  # [rad/s]
            'cmd_timeout': 0.5,             # [s] watchdog: no fresh command -> StopMove
            'move_rate': 20.0,              # [Hz] Move requests while a command is fresh
            'sit_threshold': -0.04,         # body_pose z below this = "sit down"
            'rise_settle': 3.0,             # [s] after RiseSit before Move is accepted again
            'stand_up_on_start': False,     # send StandUp + BalanceStand once at start
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = p = {name: self.get_parameter(name).value for name in defaults}

        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None
        self.moving = False
        self.sitting = False
        self.settle_until = -1e9
        self.estop = False
        self.state_count = self.imu_count = self.cloud_count = 0

        self.req_pub = self.create_publisher(Request, p['request_topic'], 10)
        self.odom_pub = self.create_publisher(Odometry, p['odom_out'], 10)
        self.imu_pub = self.create_publisher(Imu, p['imu_out'], qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, p['cloud_out'], qos_profile_sensor_data)
        self.odom_from_state, self.imu_from_state = self._pick_sources(p)
        if p['state_topic']:
            self.create_subscription(SportModeState, p['state_topic'], self._on_state,
                                     qos_profile_sensor_data)
        if p['odom_in']:
            self.create_subscription(Odometry, p['odom_in'], self._on_odom_in,
                                     qos_profile_sensor_data)
        if p['imu_in']:
            self.create_subscription(Imu, p['imu_in'], self._on_imu_in, qos_profile_sensor_data)
        if p['lowstate_in']:
            self.create_subscription(LowState, p['lowstate_in'], self._on_lowstate,
                                     qos_profile_sensor_data)
        self.create_subscription(PointCloud2, p['cloud_in'], self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Twist, p['cmd_vel_in'], self._on_cmd, 10)
        self.create_subscription(Pose, p['body_pose_in'], self._on_body_pose, 10)
        self.create_subscription(Bool, p['estop_topic'], self._on_estop, 10)
        self.create_timer(1.0 / p['move_rate'], self._tick)
        self.create_timer(5.0, self._report)
        mode = 'MOTION ENABLED' if p['enable_motion'] else 'DRY RUN (enable_motion:=true to move)'
        self.get_logger().info(f'Go2 bridge: {mode}; {p["cloud_in"]} -> {p["cloud_out"]}; '
                               f'{p["cmd_vel_in"]} / {p["body_pose_in"]} -> {p["request_topic"]}')
        if p['stand_up_on_start']:
            self._send(sport_api.request(sport_api.STAND_UP))
            self._send(sport_api.request(sport_api.BALANCE_STAND))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------- robot -> SAR
    def _pick_sources(self, p):
        """Which input feeds odom_out and imu_out. Explicit topics win over state_topic
        so the two never publish the same output at once."""
        odom_from_state = not p['odom_in']
        imu_from_state = not (p['imu_in'] or p['lowstate_in'])
        if p['imu_in'] and p['lowstate_in']:
            self.get_logger().warn('both imu_in and lowstate_in are set; using imu_in')
        self.get_logger().info(
            f'odom_out from {p["odom_in"] or p["state_topic"] or "NOTHING"}, '
            f'imu_out from {p["imu_in"] or p["lowstate_in"] or p["state_topic"] or "NOTHING"}')
        return odom_from_state, imu_from_state

    def _imu_from_quaternion_gyro_accel(self, stamp, quat, gyro, accel):
        w, x, y, z = (float(v) for v in quat)           # Unitree order: w, x, y, z
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu'
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = w, x, y, z
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (
            float(v) for v in gyro)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (
            float(v) for v in accel)
        return imu

    def _on_odom_in(self, msg):
        """Pass through, re-stamped: the robot's clock is offset from this computer's."""
        self.state_count += 1
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = msg.header.frame_id or 'odom'
        msg.child_frame_id = msg.child_frame_id or 'base_link'
        self.odom_pub.publish(msg)

    def _on_imu_in(self, msg):
        self.imu_count += 1
        msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_pub.publish(msg)

    def _on_lowstate(self, msg):
        self.imu_count += 1
        s = msg.imu_state
        self.imu_pub.publish(self._imu_from_quaternion_gyro_accel(
            self.get_clock().now().to_msg(), s.quaternion, s.gyroscope, s.accelerometer))

    def _on_state(self, msg):
        self.state_count += 1
        stamp = self.get_clock().now().to_msg()
        w, x, y, z = (float(v) for v in msg.imu_state.quaternion)   # Unitree order: w, x, y, z
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2])
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x = w, x
        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z = y, z
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.angular.z = float(msg.yaw_speed)
        if self.odom_from_state:
            self.odom_pub.publish(odom)
        if self.imu_from_state:
            self.imu_count += 1
            self.imu_pub.publish(self._imu_from_quaternion_gyro_accel(
                stamp, msg.imu_state.quaternion, msg.imu_state.gyroscope,
                msg.imu_state.accelerometer))

    def _on_cloud(self, msg):
        self.cloud_count += 1
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cloud_pub.publish(msg)

    # ------------------------------------------------------------- SAR -> robot
    def _on_cmd(self, msg):
        p = self.p
        clamp = lambda v, lim: max(-lim, min(lim, float(v)))
        self.cmd = (clamp(msg.linear.x, p['max_vx']), clamp(msg.linear.y, p['max_vy']),
                    clamp(msg.angular.z, p['max_wz']))
        self.cmd_time = self._now()

    def _on_body_pose(self, msg):
        lowered = msg.position.z < self.p['sit_threshold']
        if lowered and not self.sitting:
            self.sitting = True
            self._stop()
            self._send(sport_api.request(sport_api.SIT))
        elif not lowered and self.sitting:
            self.sitting = False
            self.settle_until = self._now() + self.p['rise_settle']
            self._send(sport_api.request(sport_api.RISE_SIT))

    def _on_estop(self, msg):
        if msg.data and not self.estop:
            self.get_logger().warn('EMERGENCY STOP: StopMove, motion commands ignored until estop=false')
            self._stop(force=True)
        elif not msg.data and self.estop:
            self.get_logger().info('emergency stop released')
        self.estop = bool(msg.data)

    def _tick(self):
        now = self._now()
        fresh = self.cmd_time is not None and now - self.cmd_time < self.p['cmd_timeout']
        if self.estop or self.sitting or now < self.settle_until or not fresh:
            self._stop()
            return
        vx, vy, wz = self.cmd
        if abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3:
            self._stop()
            return
        self.moving = True
        self._send(sport_api.move(vx, vy, wz), quiet=True)

    def _stop(self, force=False):
        if self.moving or force:
            self.moving = False
            self._send(sport_api.request(sport_api.STOP_MOVE))

    def _send(self, req, quiet=False):
        api_id, params = sport_api.parse(req)
        text = f'{sport_api.NAMES.get(api_id, api_id)} {params or ""}'.strip()
        if self.p['enable_motion']:
            self.req_pub.publish(req)
            if quiet:
                self.get_logger().info(f'-> {text}', throttle_duration_sec=2.0)
            else:
                self.get_logger().info(f'-> {text}')
        else:
            self.get_logger().info(f'(dry run) would send {text}', throttle_duration_sec=1.0 if quiet else 0.0)

    def _report(self):
        self.get_logger().info(
            f'last 5 s: {self.state_count} odom/state, {self.imu_count} imu, '
            f'{self.cloud_count} lidar clouds'
            + ('' if self.state_count and self.imu_count and self.cloud_count else
               '  <-- no data: check the topic names and the network interface / CYCLONEDDS_URI'))
        self.state_count = self.imu_count = self.cloud_count = 0

    def shutdown(self):
        """StopMove before the node goes away (a few times: requests are not acknowledged)."""
        for _ in range(3):
            try:
                self._stop(force=True)
            except Exception as e:      # the context is already gone
                print(f'go2_bridge: could not send StopMove on exit: {e}')
                return
            time.sleep(0.05)


def main(args=None):
    # Ctrl+C / SIGINT is handled here, not by rclpy: the ROS context must still be valid
    # to send StopMove to the robot on the way out
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Go2Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
