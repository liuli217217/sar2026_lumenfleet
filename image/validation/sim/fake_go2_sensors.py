#!/usr/bin/env python3
"""Synthetic Go2 sensor feed: 3D lidar cloud + RGB-D + odometry + TF.

Stands in for retime_utlidar.py / receive_time_adapter / the RealSense so the
RTAB-Map launch files can be exercised with no robot attached. Publishes on
exactly the topics those nodes would:
    /utlidar/cloud_deskewed_retimed   sensor_msgs/PointCloud2  (base_link)
    /utlidar/cloud_base_retimed       sensor_msgs/PointCloud2  (base_link)
    /utlidar/robot_odom_retimed       nav_msgs/Odometry        (odom->base_link)
    /lumen/retimed/rgb                sensor_msgs/Image  rgb8
    /lumen/retimed/depth              sensor_msgs/Image  32FC1
    /lumen/retimed/rgb_info           sensor_msgs/CameraInfo
    /joint_states                     sensor_msgs/JointState   (standing / trot)

World: a 10 x 7 x 2.6 m room with two pillars.

Two drive modes (ros param 'drive'):
    cmd_vel  (default) -- unicycle integration of /cmd_vel, i.e. whatever Nav2
                          and the collision monitor actually command. Send a
                          "2D Nav Goal" in RViz and the robot drives there.
    circle             -- ignores /cmd_vel and loops on a fixed circle, for
                          exercising mapping on its own.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

ROOM_X, ROOM_Y, ROOM_Z = 10.0, 7.0, 2.6
PILLARS = [(3.0, 2.0, 0.4), (7.0, 5.0, 0.4)]

# base_link is the body centre, ~0.32 m off the floor when the Go2 stands.
# Getting this wrong puts the floor returns inside the
# pointcloud_to_laserscan height band and fills the costmaps with phantom
# obstacles, so Nav2 cannot plan anywhere.
BODY_Z = 0.32
LIDAR_DZ = 0.10         # utlidar above base_link
CAM_XYZ = (0.33, 0.0, 0.04)   # camera_tf.launch.py 'seek' defaults
LEG_JOINTS = [f'{leg}_{j}_joint'
              for leg in ('FL', 'FR', 'RL', 'RR')
              for j in ('hip', 'thigh', 'calf')]
STAND = (0.0, 0.80, -1.55)      # hip, thigh, calf at rest
CAM_W, CAM_H, CAM_HFOV = 320, 240, math.radians(60.0)

TRAJ_CENTER = (5.0, 3.5)
TRAJ_RADIUS = 2.0
TRAJ_OMEGA = 0.18       # rad/s around the loop ('circle' mode)

START_POSE = (2.0, 3.5, 0.0)    # ('cmd_vel' mode) clear of both pillars
CMD_TIMEOUT = 0.5               # s without /cmd_vel -> stop, like the bridge

# Surface ids, used to colour the RGB image.
WALL, FLOOR, CEIL, PILLAR = 0, 1, 2, 3


def raycast(px, py, dirx, diry):
    """Horizontal distance to the nearest vertical surface, per ray."""
    best = np.full(dirx.shape, np.inf)
    hit = np.full(dirx.shape, WALL, dtype=np.int8)

    for bound, d, p in ((0.0, dirx, px), (ROOM_X, dirx, px),
                        (0.0, diry, py), (ROOM_Y, diry, py)):
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (bound - p) / d
        t = np.where((t > 0) & np.isfinite(t), t, np.inf)
        # Check the hit stays inside the room on the other axis.
        hx, hy = px + t * dirx, py + t * diry
        inside = ((hx >= -1e-6) & (hx <= ROOM_X + 1e-6)
                  & (hy >= -1e-6) & (hy <= ROOM_Y + 1e-6))
        t = np.where(inside, t, np.inf)
        best = np.minimum(best, t)

    for cx, cy, r in PILLARS:
        fx, fy = px - cx, py - cy
        b = 2.0 * (fx * dirx + fy * diry)
        c = fx * fx + fy * fy - r * r
        disc = b * b - 4.0 * c
        with np.errstate(invalid='ignore'):
            root = np.sqrt(np.where(disc > 0, disc, 0.0))
        t = np.where(disc > 0, (-b - root) / 2.0, np.inf)
        t = np.where(t > 0, t, np.inf)
        hit = np.where(t < best, PILLAR, hit)
        best = np.minimum(best, t)

    return best, hit


def trace(px, py, pz, dirx, diry, dirz):
    """Range along each (unit) ray, plus which surface it landed on."""
    hlen = np.hypot(dirx, diry)
    safe = np.where(hlen > 1e-9, hlen, 1e-9)
    d2d, hit = raycast(px, py, dirx / safe, diry / safe)

    rng = d2d / safe                      # ray length to the vertical surface
    z_at = pz + rng * dirz

    # Rays that leave through the floor or ceiling instead.
    with np.errstate(divide='ignore', invalid='ignore'):
        t_floor = np.where(dirz < -1e-9, -pz / dirz, np.inf)
        t_ceil = np.where(dirz > 1e-9, (ROOM_Z - pz) / dirz, np.inf)
    rng = np.where(z_at < 0.0, t_floor, rng)
    hit = np.where(z_at < 0.0, FLOOR, hit)
    rng = np.where(z_at > ROOM_Z, t_ceil, rng)
    hit = np.where(z_at > ROOM_Z, CEIL, hit)

    return rng, hit


class FakeGo2(Node):

    def __init__(self):
        super().__init__('fake_go2_sensors')
        self.cloud_pub = self.create_publisher(
            point_cloud2.PointCloud2, '/utlidar/cloud_deskewed_retimed', 10)
        # body_filter -> pointcloud_to_laserscan -> /utlidar/scan feeds the
        # Nav2 costmaps off this second copy, exactly as on the robot.
        self.cloud_base_pub = self.create_publisher(
            point_cloud2.PointCloud2, '/utlidar/cloud_base_retimed', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/utlidar/robot_odom_retimed', 10)
        self.rgb_pub = self.create_publisher(Image, '/lumen/retimed/rgb', 10)
        self.depth_pub = self.create_publisher(Image, '/lumen/retimed/depth', 10)
        self.info_pub = self.create_publisher(
            CameraInfo, '/lumen/retimed/rgb_info', 10)
        # robot_state_publisher turns these into the leg TFs, so the Go2 shows
        # up in RViz standing on the floor instead of as a bare base_link.
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.drive = self.declare_parameter('drive', 'cmd_vel').value
        self.x, self.y, self.yaw = START_POSE
        self.cmd = Twist()
        self.cmd_time = 0.0
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self._publish_camera_tf()

        # Lidar beam table (body frame): 16 rings x 720 azimuths.
        self.az = np.linspace(-math.pi, math.pi, 720, endpoint=False)
        self.el = np.radians(np.linspace(-16.0, 16.0, 16))
        az, el = np.meshgrid(self.az, self.el)
        self.beam = np.stack([
            np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)
        ], axis=-1).reshape(-1, 3)

        # Pinhole rays (optical frame -> body frame).
        self.fx = self.fy = CAM_W / (2.0 * math.tan(CAM_HFOV / 2.0))
        self.cx, self.cy = CAM_W / 2.0, CAM_H / 2.0
        u, v = np.meshgrid(np.arange(CAM_W), np.arange(CAM_H))
        ox = (u - self.cx) / self.fx
        oy = (v - self.cy) / self.fy
        oz = np.ones_like(ox)
        n = np.sqrt(ox ** 2 + oy ** 2 + oz ** 2)
        self.opt = np.stack([ox / n, oy / n, oz / n], axis=-1).reshape(-1, 3)
        # optical (x right, y down, z fwd) -> body (x fwd, y left, z up)
        self.cam_body = np.stack(
            [self.opt[:, 2], -self.opt[:, 0], -self.opt[:, 1]], axis=-1)

        self.t0 = self.get_clock().now()
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'fake_go2_sensors publishing at 10 Hz, drive={self.drive}')

    def _publish_camera_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'camera_link'
        tf.transform.translation.x, tf.transform.translation.y, \
            tf.transform.translation.z = CAM_XYZ
        # roll -90, yaw -90: body axes -> optical axes
        tf.transform.rotation.x = 0.5
        tf.transform.rotation.y = -0.5
        tf.transform.rotation.z = 0.5
        tf.transform.rotation.w = -0.5
        self.static_tf.sendTransform(tf)

    def on_cmd(self, msg):
        self.cmd = msg
        self.cmd_time = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def step_pose(self, t, dt):
        """Advance the robot one tick and return its pose."""
        if self.drive == 'circle':
            a = TRAJ_OMEGA * t
            return (TRAJ_CENTER[0] + TRAJ_RADIUS * math.cos(a),
                    TRAJ_CENTER[1] + TRAJ_RADIUS * math.sin(a),
                    a + math.pi / 2.0)

        vx, vy, wz = 0.0, 0.0, 0.0
        if t - self.cmd_time < CMD_TIMEOUT:
            vx, vy = self.cmd.linear.x, self.cmd.linear.y
            wz = self.cmd.angular.z
        # Unicycle + strafe, integrated in the body frame.
        self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        self.yaw = (self.yaw + wz * dt + math.pi) % (2.0 * math.pi) - math.pi
        # Keep it inside the room whatever Nav2 asks for.
        self.x = min(max(self.x, 0.5), ROOM_X - 0.5)
        self.y = min(max(self.y, 0.5), ROOM_Y - 0.5)
        return self.x, self.y, self.yaw

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        x, y, yaw = self.step_pose(t, 0.1)
        stamp = now.to_msg()

        self._send_joints(stamp, t)
        self._send_odom(stamp, x, y, yaw)
        self._send_cloud(stamp, x, y, yaw)
        self._send_camera(stamp, x, y, yaw)

    def _send_joints(self, stamp, t):
        """Stand still, or swing the legs in a diagonal trot while moving."""
        speed = abs(self.cmd.linear.x) + abs(self.cmd.linear.y) \
            + 0.3 * abs(self.cmd.angular.z)
        if self.drive == 'circle':
            speed = TRAJ_OMEGA * TRAJ_RADIUS
        elif t - self.cmd_time >= CMD_TIMEOUT:
            speed = 0.0
        amp = min(speed, 0.4) * 0.5          # rad, scales with commanded speed

        js = JointState()
        js.header = Header(stamp=stamp, frame_id='')
        js.name = LEG_JOINTS
        pos = []
        for leg in ('FL', 'FR', 'RL', 'RR'):
            # FL/RR swing together, FR/RL on the opposite half-cycle.
            phase = 2.0 * math.pi * 1.8 * t + (math.pi if leg in ('FR', 'RL') else 0.0)
            pos += [
                STAND[0],
                STAND[1] + amp * math.sin(phase),
                STAND[2] - amp * 0.6 * math.sin(phase),
            ]
        js.position = pos
        self.joint_pub.publish(js)

    def _rot(self, yaw, vecs):
        c, s = math.cos(yaw), math.sin(yaw)
        return np.stack([
            c * vecs[:, 0] - s * vecs[:, 1],
            s * vecs[:, 0] + c * vecs[:, 1],
            vecs[:, 2],
        ], axis=-1)

    def _send_odom(self, stamp, x, y, yaw):
        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id='odom')
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = BODY_Z
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = BODY_Z
        tf.transform.rotation.z = odom.pose.pose.orientation.z
        tf.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf.sendTransform(tf)

    def _send_cloud(self, stamp, x, y, yaw):
        world = self._rot(yaw, self.beam)
        rng, _ = trace(x, y, BODY_Z + LIDAR_DZ,
                       world[:, 0], world[:, 1], world[:, 2])
        keep = np.isfinite(rng) & (rng < 20.0)
        rng = rng[keep] + np.random.normal(0.0, 0.01, keep.sum())
        pts = self.beam[keep] * rng[:, None]
        pts[:, 2] += LIDAR_DZ         # cloud is expressed in base_link
        cloud = point_cloud2.create_cloud_xyz32(
            Header(stamp=stamp, frame_id='base_link'), pts.astype(np.float32))
        self.cloud_pub.publish(cloud)
        self.cloud_base_pub.publish(cloud)

    def _send_camera(self, stamp, x, y, yaw):
        world = self._rot(yaw, self.cam_body)
        cx = x + CAM_XYZ[0] * math.cos(yaw)
        cy = y + CAM_XYZ[0] * math.sin(yaw)
        cam_z = BODY_Z + CAM_XYZ[2]
        rng, hit = trace(cx, cy, cam_z,
                         world[:, 0], world[:, 1], world[:, 2])
        rng = np.where(np.isfinite(rng), rng, 0.0)

        # Perpendicular depth, as a depth camera reports it.
        depth = (rng * self.opt[:, 2]).reshape(CAM_H, CAM_W).astype(np.float32)
        depth[(depth <= 0.1) | (depth > 10.0)] = 0.0

        # Colour by surface, with a world-anchored checker so visual features
        # land in the same place frame to frame.
        px = (cx + rng * world[:, 0])
        py = (cy + rng * world[:, 1])
        pz = (cam_z + rng * world[:, 2])
        checker = ((np.floor(px * 2.5).astype(int)
                    + np.floor(py * 2.5).astype(int)
                    + np.floor(pz * 2.5).astype(int)) % 2)
        base = np.array([[170, 170, 180], [110, 115, 125],
                         [200, 200, 195], [180, 90, 70]], dtype=np.float32)
        rgb = base[hit] * np.where(checker, 1.0, 0.65)[:, None]
        rgb = np.clip(rgb, 0, 255).astype(np.uint8).reshape(CAM_H, CAM_W, 3)

        self.rgb_pub.publish(self._image(stamp, 'rgb8', rgb, CAM_W * 3))
        self.depth_pub.publish(self._image(stamp, '32FC1', depth, CAM_W * 4))

        info = CameraInfo()
        info.header = Header(stamp=stamp, frame_id='camera_link')
        info.width, info.height = CAM_W, CAM_H
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0,
                  0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    def _image(self, stamp, encoding, data, step):
        msg = Image()
        msg.header = Header(stamp=stamp, frame_id='camera_link')
        msg.height, msg.width = data.shape[0], data.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = step
        msg.data = data.tobytes()
        return msg


def main():
    rclpy.init()
    node = FakeGo2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
