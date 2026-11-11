import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import os
import yaml

class Go2CameraNode(Node):
    def __init__(self):
        super().__init__('go2_camera_node')
        
        # Parameters
        self.declare_parameter('interface', 'eno1')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera_face')
        self.declare_parameter(
            'calibration_file',
            os.path.join(
                get_package_share_directory('go2_camera_bridge'),
                'config',
                'go2_camera_calibration.yaml'
            )
        )
        
        interface = self.get_parameter('interface').value
        fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        calibration_file = self.get_parameter('calibration_file').value

        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self.bridge = CvBridge()
        self.camera_info = self.load_camera_info(calibration_file)

        # GStreamer Pipeline (Directly from your working test)
        gst_str = (
            f"udpsrc address=230.1.1.1 port=1720 multicast-iface={interface} ! "
            "application/x-rtp, media=video, encoding-name=H264 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
        )

        self.get_logger().info(f"Starting Go2 Bridge on {interface}...")
        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error("GStreamer pipeline failed to open!")
            return

        self.timer = self.create_timer(1.0/fps, self.timer_callback)

    def load_camera_info(self, calibration_file):
        info_msg = CameraInfo()
        if not calibration_file:
            self.get_logger().warn("No calibration_file set; publishing empty CameraInfo")
            return info_msg

        try:
            with open(calibration_file, 'r') as file:
                calib = yaml.safe_load(file)
        except OSError as exc:
            self.get_logger().warn(
                f"Could not load camera calibration '{calibration_file}': {exc}. "
                "Publishing empty CameraInfo."
            )
            return info_msg

        info_msg.width = int(calib.get('image_width', 0))
        info_msg.height = int(calib.get('image_height', 0))
        info_msg.distortion_model = calib.get('distortion_model', 'plumb_bob')
        info_msg.d = calib.get('distortion_coefficients', {}).get('data', [])
        info_msg.k = calib.get('camera_matrix', {}).get('data', [0.0] * 9)
        info_msg.r = calib.get('rectification_matrix', {}).get(
            'data',
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )
        info_msg.p = calib.get('projection_matrix', {}).get('data', [0.0] * 12)
        self.get_logger().info(f"Loaded camera calibration: {calibration_file}")
        return info_msg

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            now = self.get_clock().now().to_msg()
            
            # Publish Image
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = now
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)

            info_msg = self.camera_info
            info_msg.header.stamp = now
            info_msg.header.frame_id = self.frame_id
            if info_msg.width == 0 or info_msg.height == 0:
                info_msg.height, info_msg.width = frame.shape[:2]
            self.info_pub.publish(info_msg)

def main():
    rclpy.init()
    node = Go2CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
