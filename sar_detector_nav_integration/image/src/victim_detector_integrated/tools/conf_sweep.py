#!/usr/bin/env python3
"""用 5 个折模型，各自在自己的 val 上跑，算出「置信度阈值 -> 召回/精确率」曲线。

动机：搜救任务里漏掉一个伤员的代价远大于多报一个误检（人可以复核）。
所以该调的不是 mAP，而是在可接受的误报率下把召回顶到最高。
节点的 conf 是个 ROS 参数，改一下就行，不用重训。

用法: conf_sweep.py <kfold工作目录> <折数>
"""
import collections
import pathlib
import statistics
import sys

from ultralytics import YOLO

work = pathlib.Path(sys.argv[1])
K = int(sys.argv[2]) if len(sys.argv) > 2 else 5
NAMES = ["cone", "first_aid", "mannequin"]
THRESH = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50]


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


# per_fold[t][cls] = (tp, fp, n_gt)
per_fold = collections.defaultdict(lambda: collections.defaultdict(list))

for k in range(K):
    wts = work / ("run%d" % k) / "weights" / "best.pt"
    vdir = work / ("fold%d" % k) / "val"
    if not wts.is_file() or not vdir.is_dir():
        print("跳过 fold %d（缺文件）" % k)
        continue
    model = YOLO(str(wts))

    # 一次低阈值推理，之后在内存里按不同阈值筛，省时间
    cache = []
    for ip in sorted((vdir / "images").iterdir()):
        if ip.suffix.lower() not in (".jpg", ".jpeg", ".png"):
            continue
        lp = vdir / "labels" / (ip.stem + ".txt")
        if not lp.is_file():
            continue
        res = model.predict(str(ip), imgsz=640, conf=0.02, iou=0.6,
                            verbose=False)[0]
        H, W = res.orig_shape
        preds = [(int(b.cls), float(b.conf),
                  tuple(float(v) for v in b.xyxy[0].tolist()))
                 for b in res.boxes]
        gts = []
        for row in lp.read_text(errors="replace").splitlines():
            f = row.split()
            if len(f) < 5:
                continue
            c = int(float(f[0]))
            cx, cy, w, h = (float(v) for v in f[1:5])
            gts.append((c, ((cx - w / 2) * W, (cy - h / 2) * H,
                            (cx + w / 2) * W, (cy + h / 2) * H)))
        cache.append((preds, gts))

    for t in THRESH:
        tp = collections.Counter()
        fp = collections.Counter()
        ngt = collections.Counter()
        for preds, gts in cache:
            kept = [p for p in preds if p[1] >= t]
            used = set()
            for pc, pconf, pbox in sorted(kept, key=lambda p: -p[1]):
                best, bi = 0.0, -1
                for i, (gc, gbox) in enumerate(gts):
                    if i in used or gc != pc:
                        continue
                    v = iou(pbox, gbox)
                    if v > best:
                        best, bi = v, i
                if best >= 0.5:
                    tp[pc] += 1
                    used.add(bi)
                else:
                    fp[pc] += 1
            for gc, _ in gts:
                ngt[gc] += 1
        for c in range(3):
            if ngt[c]:
                r = tp[c] / ngt[c]
                p = tp[c] / max(1, tp[c] + fp[c])
                per_fold[t][c].append((r, p, fp[c], ngt[c]))

print("\n置信度阈值扫描（5 折平均，IoU>=0.5）\n")
for c in range(3):
    print("=== %s ===" % NAMES[c])
    print("  %-6s %-22s %-22s %s" % ("conf", "召回 R", "精确率 P", "平均每折误报数"))
    for t in THRESH:
        rows = per_fold[t][c]
        if not rows:
            continue
        rs = [x[0] for x in rows]
        ps = [x[1] for x in rows]
        fps = [x[2] for x in rows]
        sd = statistics.stdev(rs) if len(rs) > 1 else 0.0
        sdp = statistics.stdev(ps) if len(ps) > 1 else 0.0
        print("  %-6.2f %6.3f ± %-13.3f %6.3f ± %-13.3f %.1f"
              % (t, statistics.mean(rs), sd, statistics.mean(ps), sdp,
                 statistics.mean(fps)))
    print()
