"""Minimal YOLOv8 (Ultralytics export) inference with ONNX Runtime.

The exported model takes a 1x3xSxS RGB float tensor in [0, 1] and returns
1x(4+num_classes)xN, where each column is (cx, cy, w, h, class scores...)
in letterboxed input pixels.
"""
from dataclasses import dataclass

import cv2
import numpy as np
import onnxruntime as ort


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    class_id: int


def _nms(boxes, scores, iou_threshold):
    """Greedy non-maximum suppression. boxes: (N, 4) xyxy."""
    order = scores.argsort()[::-1]
    areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
        yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
        xx2 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
        yy2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        order = order[1:][iou <= iou_threshold]
    return keep


class YoloOnnxDetector:
    def __init__(self, model_path, conf_threshold=0.25, iou_threshold=0.45,
                 class_ids=(0,), num_threads=4):
        options = ort.SessionOptions()
        options.intra_op_num_threads = num_threads
        options.log_severity_level = 3  # hide GPU discovery warnings on Jetson
        self.session = ort.InferenceSession(
            model_path, options, providers=['CPUExecutionProvider'])
        model_input = self.session.get_inputs()[0]
        self.input_name = model_input.name
        self.input_size = int(model_input.shape[2])
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.class_ids = np.asarray(class_ids)

    def _letterbox(self, image):
        h, w = image.shape[:2]
        scale = min(self.input_size / h, self.input_size / w)
        new_w, new_h = int(round(w * scale)), int(round(h * scale))
        pad_x = (self.input_size - new_w) // 2
        pad_y = (self.input_size - new_h) // 2
        canvas = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv2.resize(
            image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        blob = canvas.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        return blob, scale, pad_x, pad_y

    def detect(self, image_rgb):
        """Run detection on an HxWx3 RGB uint8 image; returns a list of Detection."""
        blob, scale, pad_x, pad_y = self._letterbox(image_rgb)
        output = self.session.run(None, {self.input_name: blob})[0][0].T  # (N, 4+C)
        class_scores = output[:, 4:]
        class_ids = class_scores.argmax(axis=1)
        scores = class_scores[np.arange(len(class_ids)), class_ids]
        mask = (scores >= self.conf_threshold) & np.isin(class_ids, self.class_ids)
        if not mask.any():
            return []
        cxcywh, scores, class_ids = output[mask, :4], scores[mask], class_ids[mask]

        boxes = np.empty_like(cxcywh)
        boxes[:, 0] = cxcywh[:, 0] - cxcywh[:, 2] / 2
        boxes[:, 1] = cxcywh[:, 1] - cxcywh[:, 3] / 2
        boxes[:, 2] = cxcywh[:, 0] + cxcywh[:, 2] / 2
        boxes[:, 3] = cxcywh[:, 1] + cxcywh[:, 3] / 2
        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_x) / scale
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_y) / scale
        h, w = image_rgb.shape[:2]
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, w - 1)
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, h - 1)

        detections = []
        for cls in np.unique(class_ids):
            idx = np.where(class_ids == cls)[0]
            for k in _nms(boxes[idx], scores[idx], self.iou_threshold):
                b = boxes[idx[k]]
                detections.append(Detection(float(b[0]), float(b[1]), float(b[2]), float(b[3]),
                                            float(scores[idx[k]]), int(cls)))
        return detections
