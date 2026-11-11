#!/usr/bin/env python3
"""给 detector_node.py 加两样东西：
  1. per-class 置信度阈值（cone 和 mannequin 想要的阈值差很多）
  2. 时序聚合（连续多帧命中才确认，压误报、提有效召回）

对现有接口零破坏：~/detections 行为完全不变，确认后的结果发到新话题
~/detections_confirmed。下游想用哪个用哪个。
"""
import pathlib
import sys

p = pathlib.Path(sys.argv[1])
src = p.read_text(encoding="utf-8")


def must_replace(old, new, what):
    global src
    if old not in src:
        sys.exit("找不到锚点，节点可能已改过: %s" % what)
    if src.count(old) != 1:
        sys.exit("锚点不唯一: %s" % what)
    src = src.replace(old, new, 1)


# ---------- 1. 工具函数 ----------
must_replace(
    'def _int_set(spec):',
    '''def _conf_map(spec):
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

    def __init__(self, iou_thr, max_misses, min_hits):
        self.iou_thr = iou_thr
        self.max_misses = max_misses
        self.min_hits = min_hits
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
                if t.hits >= self.min_hits and t.misses == 0]


def _int_set(spec):''',
    "工具函数")

# ---------- 2. 参数 ----------
must_replace(
    '        p("publish_annotated", True)',
    '''        # 分类别阈值；留空 = 所有类都用全局 conf
        p("class_conf", "")
        # --- 时序聚合 ---
        p("track_enable", False)     # 默认关，保持原有行为不变
        p("track_iou", 0.3)          # 帧间关联的 IoU 阈值
        p("track_min_hits", 3)       # 连续命中几帧才算确认
        p("track_max_misses", 5)     # 丢几帧后丢弃这个 track
        p("confirmed_topic", "~/detections_confirmed")
        # --- 输出 ---
        p("publish_annotated", True)''',
    "参数声明")

must_replace(
    '        self.publish_annotated = bool(g("publish_annotated"))',
    '''        self.class_conf = _conf_map(g("class_conf"))
        self.track_enable = bool(g("track_enable"))
        self.publish_annotated = bool(g("publish_annotated"))''',
    "参数读取")

# ---------- 3. 建 tracker + publisher ----------
must_replace(
    '        self.pub_status = self.create_publisher(String, str(g("status_topic")), 1)',
    '''        self.tracker = _Tracker(float(g("track_iou")),
                                int(g("track_max_misses")),
                                int(g("track_min_hits"))) if self.track_enable else None
        self.pub_conf = (self.create_publisher(
            Detection2DArray, str(g("confirmed_topic")), 10)
            if self.track_enable else None)
        self.pub_status = self.create_publisher(String, str(g("status_topic")), 1)''',
    "tracker 构造")

must_replace(
    '''        self.get_logger().info("类别: %s" % (self.names,))''',
    '''        self.get_logger().info("类别: %s" % (self.names,))
        if self.class_conf:
            self.get_logger().info("分类别阈值: %s" % self.class_conf)
        if self.track_enable:
            self.get_logger().info(
                "时序聚合开启: 连续 %d 帧命中才确认, IoU>=%.2f, 容忍丢 %d 帧 -> %s"
                % (int(g("track_min_hits")), float(g("track_iou")),
                   int(g("track_max_misses")), g("confirmed_topic")))''',
    "启动日志")

# ---------- 4. 回调里应用 ----------
must_replace(
    '''            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
            dets.append((cid, float(b.conf), x1, y1, x2, y2))''',
    '''            conf = float(b.conf)
            # 分类别阈值：这一类单独设了就用它的，否则沿用全局 conf
            thr = self.class_conf.get(self._name_of(cid))
            if thr is not None and conf < thr:
                continue
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
            dets.append((cid, conf, x1, y1, x2, y2))''',
    "分类别过滤")

must_replace(
    '''        # 空结果也要发 —— 下游据此知道节点活着、只是这一帧没有目标
        self.pub_det.publish(self._to_msg(dets, msg.header))''',
    '''        # 空结果也要发 —— 下游据此知道节点活着、只是这一帧没有目标
        self.pub_det.publish(self._to_msg(dets, msg.header))

        if self.tracker is not None:
            confirmed = self.tracker.update(dets)
            self.n_confirmed = len(confirmed)
            arr = self._to_msg([c[:6] for c in confirmed], msg.header)
            # 把 track id 填进 Detection2D.id，下游可以据此跨帧对应同一个目标
            for d, c in zip(arr.detections, confirmed):
                d.id = str(c[6])
            self.pub_conf.publish(arr)''',
    "tracker 调用")

must_replace(
    '        self.n_skipped = 0',
    '        self.n_skipped = 0\n        self.n_confirmed = 0',
    "计数器初始化")

must_replace(
    '''            data="frames=%d skipped=%d infer_ms=%.1f device=%s"
                 % (self.n_frames, self.n_skipped, avg, self.device)))''',
    '''            data="frames=%d skipped=%d infer_ms=%.1f device=%s confirmed=%d"
                 % (self.n_frames, self.n_skipped, avg, self.device,
                    self.n_confirmed)))''',
    "status 输出")

p.write_text(src, encoding="utf-8")
print("已打补丁:", p)
