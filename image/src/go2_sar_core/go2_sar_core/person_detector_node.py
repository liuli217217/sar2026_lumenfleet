"""YOLOv8n person detector for the Go2 RGB camera.

Subscribes:  /rgb_image (sensor_msgs/Image)
Publishes:   /sar/person_detections (vision_msgs/Detection2DArray)
             /sar/person_detections/image (sensor_msgs/Image, annotated, optional)
"""
import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from go2_sar_core.yolo_onnx import YoloOnnxDetector

_CHANNELS = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}


def image_msg_to_rgb(msg):
    """Convert a sensor_msgs/Image to an HxWx3 RGB uint8 array (no cv_bridge needed)."""
    channels = _CHANNELS.get(msg.encoding)
    if channels is None:
        raise ValueError(f'unsupported image encoding: {msg.encoding}')
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    image = data[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding.startswith('bgr'):
        image = image[..., 2::-1]
    return np.ascontiguousarray(image[..., :3])


class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__('person_detector')
        default_model = os.path.join(
            get_package_share_directory('go2_sar_core'), 'models', 'yolov8n.onnx')
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('image_topic', '/rgb_image')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('detection_rate_hz', 4.0)
        self.declare_parameter('num_threads', 4)
        self.declare_parameter('publish_debug_image', True)
        # If set, annotated frames that contain a person are saved here (max 1 per second).
        self.declare_parameter('save_detections_dir', '')
        # Run YOLO only while enabled (std_msgs/Bool on enable_topic), e.g. near a victim.
        self.declare_parameter('start_enabled', True)
        self.declare_parameter('enable_topic', '/sar/detector_enable')

        model_path = self.get_parameter('model_path').value
        self.detector = YoloOnnxDetector(
            model_path,
            conf_threshold=self.get_parameter('confidence_threshold').value,
            iou_threshold=self.get_parameter('iou_threshold').value,
            class_ids=(0,),  # COCO class 0 = person
            num_threads=self.get_parameter('num_threads').value)
        self.publish_debug = self.get_parameter('publish_debug_image').value
        self.save_dir = os.path.expanduser(self.get_parameter('save_detections_dir').value)
        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)
        self.last_save = 0.0

        self.latest_image = None
        self.enabled = self.get_parameter('start_enabled').value
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self._on_image, qos_profile_sensor_data)
        self.create_subscription(Bool, self.get_parameter('enable_topic').value, self._on_enable, 10)
        self.det_pub = self.create_publisher(Detection2DArray, '/sar/person_detections', 10)
        self.debug_pub = self.create_publisher(Image, '/sar/person_detections/image', 2)
        period = 1.0 / self.get_parameter('detection_rate_hz').value
        # Wall-clock timer: inference cost does not scale with the simulation clock.
        self.create_timer(period, self._process, clock=rclpy.clock.Clock())
        self.infer_ms = None
        self.get_logger().info(
            f'YOLO model {model_path} (input {self.detector.input_size}px) loaded')

    def _on_image(self, msg):
        self.latest_image = msg

    def _on_enable(self, msg):
        if msg.data != self.enabled:
            self.get_logger().info('YOLO ' + ('enabled' if msg.data else 'disabled'))
        self.enabled = msg.data

    def _process(self):
        msg, self.latest_image = self.latest_image, None
        if msg is None or not self.enabled:
            return
        try:
            image = image_msg_to_rgb(msg)
        except ValueError as err:
            self.get_logger().error(str(err), throttle_duration_sec=5.0)
            return

        start = time.perf_counter()
        detections = self.detector.detect(image)
        elapsed = (time.perf_counter() - start) * 1000.0
        self.infer_ms = elapsed if self.infer_ms is None else 0.9 * self.infer_ms + 0.1 * elapsed
        self.get_logger().info(
            f'inference {self.infer_ms:.0f} ms, persons: {len(detections)}',
            throttle_duration_sec=10.0)

        out = Detection2DArray()
        out.header = msg.header
        for det in detections:
            d = Detection2D()
            d.header = msg.header
            d.bbox.center.position.x = (det.x1 + det.x2) / 2.0
            d.bbox.center.position.y = (det.y1 + det.y2) / 2.0
            d.bbox.size_x = det.x2 - det.x1
            d.bbox.size_y = det.y2 - det.y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'person'
            hyp.hypothesis.score = det.score
            d.results.append(hyp)
            out.detections.append(d)
        self.det_pub.publish(out)

        want_debug = self.publish_debug and self.debug_pub.get_subscription_count() > 0
        want_save = self.save_dir and detections and time.time() - self.last_save > 1.0
        if want_debug or want_save:
            canvas = self._annotate(image, detections)
            if want_debug:
                self._publish_debug(msg, canvas)
            if want_save:
                self.last_save = time.time()
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                best = max(d.score for d in detections)
                path = os.path.join(self.save_dir, f'person_t{stamp:09.2f}_s{best:.2f}.jpg')
                cv2.imwrite(path, cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))

    @staticmethod
    def _annotate(image, detections):
        canvas = image.copy()
        for det in detections:
            p1, p2 = (int(det.x1), int(det.y1)), (int(det.x2), int(det.y2))
            cv2.rectangle(canvas, p1, p2, (255, 0, 0), 2)
            cv2.putText(canvas, f'person {det.score:.2f}', (p1[0], max(p1[1] - 5, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        return canvas

    def _publish_debug(self, msg, canvas):
        out = Image()
        out.header = msg.header
        out.height, out.width = canvas.shape[:2]
        out.encoding = 'rgb8'
        out.step = out.width * 3
        out.data = canvas.tobytes()
        self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
