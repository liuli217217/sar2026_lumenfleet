#!/usr/bin/env python3
"""SAR 目标检测 ROS 2 节点 (Team Lumen Fleet)。

订阅相机图像 -> YOLO 推理 -> 发布检测结果。

所有行为通过 ROS 参数控制，换相机 / 换权重 / 换类别都不用改代码。
完整接口说明见 README_victim_detector.md。
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)


def sensor_qos(best_effort: bool, depth: int) -> QoSProfile:
    """相机话题的 QoS 配置。

    realsense-ros 用 BEST_EFFORT 发布图像。订阅端若用默认的 RELIABLE，
    QoS 不兼容，会一帧都收不到、而且不报任何错 —— 真机上最常见的静默失败。
    """
    return QoSProfile(
        reliability=(ReliabilityPolicy.BEST_EFFORT if best_effort
                     else ReliabilityPolicy.RELIABLE),
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


def _conf_map(spec):
    """"mannequin:0.10,cone:0.2" -> {"mannequin": 0.1, "cone": 0.2}。

    为什么要分类别设阈值：实测 cone 在 0.20 时召回 0.92 / 精确率 0.86，
    而 mannequin 想要召回就得压到 0.10 —— 一个全局阈值服务三个类是浪费。
    """
    out = {}
    for tok in str(spec).replace(" ", "").split(","):
        if not tok:
            continue
        name, _, val = tok.partition(":")
        if name and val:
            out[name] = float(val)
    return out


def _iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


class _Track:
    """一个被跟了若干帧的目标。"""
    __slots__ = ("box", "cls", "score", "hits", "misses", "tid")

    def __init__(self, box, cls, score, tid):
        self.box, self.cls, self.score = box, cls, score
        self.hits, self.misses, self.tid = 1, 0, tid


class _Tracker:
    """最简单的 IoU 关联跟踪。

    动机：单帧 mannequin 召回只有 ~0.57，但机器人是在走的，经过一个伤员会在
    几十帧里从不同角度看到它，失败并不完全相关。连续命中才确认，可以同时
    压掉「闪一下就没」的误报、并把有效召回抬上去 —— 这是唯一一条两头都改善
    的路子，而且不用重训模型。
    """

    def __init__(self, iou_thr, max_misses, min_hits, bridge=False):
        self.iou_thr = iou_thr
        self.max_misses = max_misses
        self.min_hits = min_hits
        # bridge=False: 只有本帧检到才上报 -> 压误报，召回不变
        # bridge=True : 已确认的目标在丢失期间继续上报 -> 桥接空档，召回提高，
        #               但那几帧的框是上一次检到的位置，会滞后
        self.bridge = bridge
        self.tracks = []
        self._next_id = 0

    def update(self, dets):
        """dets: [(cls, score, x1, y1, x2, y2)] -> 已确认的检测列表"""
        unmatched = list(range(len(dets)))
        for t in self.tracks:
            best, bi = self.iou_thr, -1
            for i in unmatched:
                c, s, x1, y1, x2, y2 = dets[i]
                if c != t.cls:
                    continue
                v = _iou(t.box, (x1, y1, x2, y2))
                if v >= best:
                    best, bi = v, i
            if bi >= 0:
                c, s, x1, y1, x2, y2 = dets[bi]
                t.box, t.score = (x1, y1, x2, y2), s
                t.hits += 1
                t.misses = 0
                unmatched.remove(bi)
            else:
                t.misses += 1

        for i in unmatched:
            c, s, x1, y1, x2, y2 = dets[i]
            self.tracks.append(_Track((x1, y1, x2, y2), c, s, self._next_id))
            self._next_id += 1

        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]
        return [(t.cls, t.score, t.box[0], t.box[1], t.box[2], t.box[3], t.tid)
                for t in self.tracks
                if t.hits >= self.min_hits
                and (self.bridge or t.misses == 0)]


def _int_set(spec):
    """"0,2" -> {0, 2}；空串 -> 空集合（表示不过滤）。"""
    return {int(t) for t in str(spec).replace(" ", "").split(",") if t}


def _str_list(spec):
    """"a,b" -> ["a", "b"]；空串 -> []。"""
    return [t for t in (x.strip() for x in str(spec).split(",")) if t]


class SarDetector(Node):

    COLORS = {
        "mannequin": (0, 0, 255),
        "person": (0, 0, 255),
        "cone": (0, 165, 255),
        "first_aid": (0, 200, 0),
    }

    def __init__(self):
        # DETECTOR_NAV_INTEGRATION: renamed copy; upstream detector is untouched.
        super().__init__("sar_detector_integrated")

        p = self.declare_parameter
        # --- 输入 ---
        p("image_topic", "/camera/camera/color/image_raw")
        p("best_effort_qos", True)      # 真机 realsense 必须为 True
        p("queue_depth", 1)             # 1 = 永远处理最新帧，不积压
        # --- 模型 ---
        p("weights", "yolov8s.pt")
        p("device", "auto")             # auto | cpu | cuda:0
        p("imgsz", 640)
        p("conf", 0.25)
        p("iou", 0.45)
        p("half", False)                # GPU 上用 FP16，Jetson 建议开
        # 这两个用逗号分隔的字符串，不用数组：
        #   ROS 2 无法从空列表 [] 推断参数类型，节点会起不来；
        #   而且从 launch 传进来的值本来就是字符串。
        p("keep_classes", "")           # "" = 全部；COCO 只要人就填 "0"
        p("class_names", "")            # 覆盖模型自带类名，"" = 用模型的
        # --- 输出 ---
        # 分类别阈值；留空 = 所有类都用全局 conf
        p("class_conf", "")
        # --- 时序聚合 ---
        p("track_enable", False)     # 默认关，保持原有行为不变
        p("track_iou", 0.3)          # 帧间关联的 IoU 阈值
        p("track_min_hits", 3)       # 连续命中几帧才算确认
        p("track_max_misses", 5)     # 丢几帧后丢弃这个 track
        # True = 已确认目标在短暂丢失期间继续上报（桥接空档，提召回，框会滞后）
        # False = 只在本帧检到时上报（仅压误报，不改召回）
        p("track_bridge", False)
        p("confirmed_topic", "~/detections_confirmed")
        # --- 输出 ---
        p("publish_annotated", True)
        p("detections_topic", "~/detections")
        p("annotated_topic", "~/image_annotated")
        p("status_topic", "~/status")
        p("max_rate_hz", 0.0)           # 0 = 不限速；Orin 50W 建议 5~10
        p("log_every_n", 30)

        def g(key):
            return self.get_parameter(key).value

        self.imgsz = int(g("imgsz"))
        self.conf = float(g("conf"))
        self.iou = float(g("iou"))
        self.half = bool(g("half"))
        self.keep = _int_set(g("keep_classes"))
        self.name_override = _str_list(g("class_names"))
        self.class_conf = _conf_map(g("class_conf"))
        # DETECTOR_NAV_INTEGRATION: let lower per-class thresholds reach filtering.
        if not 0 <= self.conf <= 1 or any(not 0 <= v <= 1 for v in self.class_conf.values()):
            raise ValueError("Confidence thresholds must be in [0, 1]")
        self.inference_conf = min([self.conf, *self.class_conf.values()])
        self.track_enable = bool(g("track_enable"))
        self.publish_annotated = bool(g("publish_annotated"))
        self.log_every_n = max(1, int(g("log_every_n")))
        rate = float(g("max_rate_hz"))
        self.min_period = (1.0 / rate) if rate > 0 else 0.0

        self.bridge = CvBridge()
        self.n_frames = 0
        self.n_skipped = 0
        self.n_confirmed = 0
        self.last_infer_t = 0.0
        self.infer_ms = []

        self.model, self.names, self.device = self._load_model(
            str(g("weights")), str(g("device")))

        depth = max(1, int(g("queue_depth")))
        qos = sensor_qos(bool(g("best_effort_qos")), depth)
        topic = str(g("image_topic"))
        self.sub = self.create_subscription(Image, topic, self.on_image, qos)

        self.pub_det = self.create_publisher(
            Detection2DArray, str(g("detections_topic")), 10)
        self.pub_img = (self.create_publisher(Image, str(g("annotated_topic")), 1)
                        if self.publish_annotated else None)
        self.tracker = _Tracker(float(g("track_iou")),
                                int(g("track_max_misses")),
                                int(g("track_min_hits")),
                                bool(g("track_bridge"))) if self.track_enable else None
        self.pub_conf = (self.create_publisher(
            Detection2DArray, str(g("confirmed_topic")), 10)
            if self.track_enable else None)
        self.pub_status = self.create_publisher(String, str(g("status_topic")), 1)
        self.create_timer(5.0, self._publish_status)

        self.get_logger().info(
            "订阅 %s (best_effort=%s, depth=%d)"
            % (topic, g("best_effort_qos"), depth))
        self.get_logger().info(
            "模型 %s on %s, imgsz=%d, conf=%.2f, keep_classes=%s"
            % (g("weights"), self.device, self.imgsz, self.conf,
               sorted(self.keep) or "ALL"))
        self.get_logger().info("类别: %s" % (self.names,))
        if self.class_conf:
            self.get_logger().info("分类别阈值: %s" % self.class_conf)
        if self.track_enable:
            self.get_logger().info(
                "时序聚合开启: 连续 %d 帧命中才确认, IoU>=%.2f, 容忍丢 %d 帧 -> %s"
                % (int(g("track_min_hits")), float(g("track_iou")),
                   int(g("track_max_misses")), g("confirmed_topic")))
            self.get_logger().info(
                "  桥接模式 %s" % ("开（丢帧期间继续上报，提召回）"
                                 if g("track_bridge")
                                 else "关（仅压误报，不改召回）"))

    # ---------- 模型 ----------

    def _load_model(self, weights, device):
        from ultralytics import YOLO      # 延迟导入，启动失败时报错更清楚
        import torch

        if device == "auto":
            device = "cuda:0" if torch.cuda.is_available() else "cpu"
        if device == "cpu" and self.half:
            self.get_logger().warn("CPU 上不支持 half，已自动关闭")
            self.half = False

        model = YOLO(weights)
        model.to(device)
        names = self.name_override or [model.names[i] for i in sorted(model.names)]

        # 预热一次。第一帧推理会慢很多（CUDA 上下文 + cudnn benchmark），
        # 不预热的话上线头几帧会被丢掉。
        dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        t0 = time.perf_counter()
        model.predict(dummy, imgsz=self.imgsz, verbose=False, **self._extra())
        self.get_logger().info(
            "预热完成 %.0f ms" % (1000 * (time.perf_counter() - t0)))
        return model, names, device

    def _extra(self):
        """只有开了 FP16 才传 half。

        ultralytics 8.4 起 half 已废弃（提示改用 quantize），每帧都会刷一条
        警告。默认关闭时干脆不传这个参数，日志就干净了；真要开 FP16
        （Jetson 上建议开）再传，那时的警告是有意义的提示。
        """
        return {"half": True} if self.half else {}

    def _name_of(self, cid):
        return self.names[cid] if 0 <= cid < len(self.names) else str(cid)

    # ---------- 主回调 ----------

    def on_image(self, msg):
        now = time.perf_counter()
        if self.min_period and (now - self.last_infer_t) < self.min_period:
            self.n_skipped += 1
            return
        self.last_infer_t = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn("图像转换失败: %s" % exc,
                                   throttle_duration_sec=5.0)
            return

        t0 = time.perf_counter()
        try:
            res = self.model.predict(frame, imgsz=self.imgsz, conf=self.inference_conf,
                                     iou=self.iou, verbose=False,
                                     **self._extra())[0]
        except Exception as exc:
            self.get_logger().error("推理失败: %s" % exc,
                                    throttle_duration_sec=5.0)
            return
        self.infer_ms.append((time.perf_counter() - t0) * 1000.0)
        if len(self.infer_ms) > 100:
            self.infer_ms.pop(0)

        dets = []
        for b in res.boxes:
            cid = int(b.cls)
            if self.keep and cid not in self.keep:
                continue
            conf = float(b.conf)
            # 分类别阈值：这一类单独设了就用它的，否则沿用全局 conf
            thr = self.class_conf.get(self._name_of(cid), self.conf)
            if thr is not None and conf < thr:
                continue
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
            dets.append((cid, conf, x1, y1, x2, y2))

        # 空结果也要发 —— 下游据此知道节点活着、只是这一帧没有目标
        self.pub_det.publish(self._to_msg(dets, msg.header))

        if self.tracker is not None:
            confirmed = self.tracker.update(dets)
            self.n_confirmed = len(confirmed)
            arr = self._to_msg([c[:6] for c in confirmed], msg.header)
            # 把 track id 填进 Detection2D.id，下游可以据此跨帧对应同一个目标
            for d, c in zip(arr.detections, confirmed):
                d.id = str(c[6])
            self.pub_conf.publish(arr)

        if self.pub_img is not None:
            out = self.bridge.cv2_to_imgmsg(self._draw(frame, dets),
                                            encoding="bgr8")
            out.header = msg.header
            self.pub_img.publish(out)

        self.n_frames += 1
        if self.n_frames % self.log_every_n == 0:
            avg = sum(self.infer_ms) / len(self.infer_ms)
            self.get_logger().info(
                "%d 帧 | 本帧 %d 个目标 | 推理 %.0f ms (上限 %.1f fps) | 跳过 %d"
                % (self.n_frames, len(dets), avg, 1000.0 / avg, self.n_skipped))

    # ---------- 输出 ----------

    def _to_msg(self, dets, header):
        arr = Detection2DArray()
        arr.header = header
        for cid, conf, x1, y1, x2, y2 in dets:
            d = Detection2D()
            d.header = header
            bb = BoundingBox2D()
            bb.center.position.x = (x1 + x2) / 2.0
            bb.center.position.y = (y1 + y2) / 2.0
            bb.size_x = x2 - x1
            bb.size_y = y2 - y1
            d.bbox = bb
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = self._name_of(cid)   # 字符串类名，不是数字
            h.hypothesis.score = conf
            d.results.append(h)
            arr.detections.append(d)
        return arr

    def _draw(self, frame, dets):
        vis = frame.copy()
        for cid, conf, x1, y1, x2, y2 in dets:
            name = self._name_of(cid)
            col = self.COLORS.get(name, (255, 0, 255))
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), col, 2)
            cv2.putText(vis, "%s %.2f" % (name, conf),
                        (int(x1), max(14, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        return vis

    def _publish_status(self):
        avg = (sum(self.infer_ms) / len(self.infer_ms)) if self.infer_ms else 0.0
        self.pub_status.publish(String(
            data="frames=%d skipped=%d infer_ms=%.1f device=%s confirmed=%d"
                 % (self.n_frames, self.n_skipped, avg, self.device,
                    self.n_confirmed)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SarDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
