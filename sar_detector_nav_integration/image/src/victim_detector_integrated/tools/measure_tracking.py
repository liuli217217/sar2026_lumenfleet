#!/usr/bin/env python3
"""量化时序聚合到底有没有用。

我之前说「时序聚合能提高有效召回」是基于机制推理，没有测过。这里测。

做法：数据集只有静态图，没有视频。所以从每张 val 图合成一段「机器人走近」
的序列 —— 视野从远到近推进、带横向漂移和轻微的亮度/旋转抖动，真值框跟着变换。
然后比较四个量：

  单帧召回        —— 我一直在报的那个数
  至少命中一次    —— 时序方法的理论上限
  确认召回        —— tracker 实际能救回多少
  误报            —— 原始 vs 确认后

局限（必须说清楚）：合成平移缩放不等于真实视角变化，没有视差、没有换角度。
真实情况里不同帧的失败相关性更低，所以这个测法倾向于**低估**时序聚合的收益。
但它能回答「在最保守的假设下还有没有用」。

用法: measure_tracking.py <权重> <数据集根> <节点源码> [conf] [帧数]
"""
import ast
import collections
import pathlib
import sys

import cv2
import numpy as np
from ultralytics import YOLO

weights, root, nodesrc = sys.argv[1:4]
CONF = float(sys.argv[4]) if len(sys.argv) > 4 else 0.10
NFRAMES = int(sys.argv[5]) if len(sys.argv) > 5 else 24
root = pathlib.Path(root)
MANNEQUIN = 2

# 从节点源码里把 tracker 抠出来，保证测的就是实际部署的那份代码
tree = ast.parse(pathlib.Path(nodesrc).read_text(encoding="utf-8"))
ns = {}
exec(compile(ast.Module(                                        # noqa: S102
    body=[n for n in tree.body
          if getattr(n, "name", None) in {"_iou", "_Track", "_Tracker"}],
    type_ignores=[]), "<node>", "exec"), ns)
Tracker, iou = ns["_Tracker"], ns["_iou"]

model = YOLO(weights)


def load(split):
    out = []
    idir, ldir = root / split / "images", root / split / "labels"
    if not idir.is_dir():
        return out
    for ip in sorted(idir.iterdir()):
        if ip.suffix.lower() not in (".jpg", ".jpeg", ".png"):
            continue
        lp = ldir / (ip.stem + ".txt")
        if not lp.is_file():
            continue
        img = cv2.imread(str(ip))
        if img is None:
            continue
        H, W = img.shape[:2]
        if max(H, W) > 1600:
            s = 1600.0 / max(H, W)
            img = cv2.resize(img, (int(W * s), int(H * s)))
            H, W = img.shape[:2]
        gts = []
        for row in lp.read_text(errors="replace").splitlines():
            f = row.split()
            if len(f) < 5:
                continue
            c = int(float(f[0]))
            cx, cy, w, h = (float(v) for v in f[1:5])
            gts.append((c, (cx - w / 2) * W, (cy - h / 2) * H,
                        (cx + w / 2) * W, (cy + h / 2) * H))
        if any(g[0] == MANNEQUIN for g in gts):
            out.append((ip.stem, img, gts, W, H))
    return out


def pass_frames(img, gts, W, H, n):
    """合成一段由远及近的推进序列。"""
    rng = np.random.default_rng(abs(hash(img.shape)) % 2**31)
    for i in range(n):
        t = i / max(1, n - 1)
        z = 1.0 - 0.45 * t                     # 视野从整图收到 55%
        cw, ch = W * z, H * z
        # 横向漂移，模拟机器人不是正对着走
        cx = W / 2 + (0.10 * W) * np.sin(2.6 * t) * (1 - t * 0.4)
        cy = H / 2 + (0.05 * H) * np.cos(2.0 * t)
        x1 = float(np.clip(cx - cw / 2, 0, W - cw))
        y1 = float(np.clip(cy - ch / 2, 0, H - ch))
        crop = img[int(y1):int(y1 + ch), int(x1):int(x1 + cw)]
        if crop.size == 0:
            continue
        frame = cv2.resize(crop, (640, 480))
        sx, sy = 640.0 / cw, 480.0 / ch
        # 曝光抖动
        frame = np.clip(frame.astype(np.int16)
                        + int(rng.normal(0, 7)), 0, 255).astype(np.uint8)
        fg = []
        for gi, (c, gx1, gy1, gx2, gy2) in enumerate(gts):
            nx1, ny1 = (gx1 - x1) * sx, (gy1 - y1) * sy
            nx2, ny2 = (gx2 - x1) * sx, (gy2 - y1) * sy
            # 中心仍在画面内才算这一帧应该看得见
            mx, my = (nx1 + nx2) / 2, (ny1 + ny2) / 2
            if 0 <= mx < 640 and 0 <= my < 480:
                # gi 是这个伤员在原图里的下标，整段序列不变 —— 用它做身份
                fg.append((gi, c, max(0, nx1), max(0, ny1),
                           min(640, nx2), min(480, ny2)))
        yield frame, fg


items = load("val") or load("valid")
print("合成序列: %d 个场景 x %d 帧, conf=%.2f\n" % (len(items), NFRAMES, CONF))

stat = collections.Counter()
per_scene = []

for stem, img, gts, W, H in items:
    tr_off = Tracker(0.3, 5, 3, bridge=False)
    tr_on = Tracker(0.3, 5, 3, bridge=True)
    seen_any = set()
    conf_off_hit = set()
    conf_on_hit = set()
    frames_gt = 0
    frames_hit = 0
    fp_raw = fp_off = fp_on = 0

    for frame, fg in pass_frames(img, gts, W, H, NFRAMES):
        res = model.predict(frame, imgsz=640, conf=CONF, verbose=False)[0]
        dets = []
        for b in res.boxes:
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
            dets.append((int(b.cls), float(b.conf), x1, y1, x2, y2))

        gt_m = [g for g in fg if g[1] == MANNEQUIN]
        det_m = [d for d in dets if d[0] == MANNEQUIN]
        frames_gt += len(gt_m)

        matched_raw = set()
        for g in gt_m:
            for d in det_m:
                if iou((g[2], g[3], g[4], g[5]), (d[2], d[3], d[4], d[5])) >= 0.5:
                    matched_raw.add(g[0])
                    seen_any.add(g[0])
                    break
        frames_hit += len(matched_raw)
        fp_raw += max(0, len(det_m) - len(matched_raw))

        for tr, hit, fpname in ((tr_off, conf_off_hit, "off"),
                                (tr_on, conf_on_hit, "on")):
            out = tr.update(dets)
            out_m = [o for o in out if o[0] == MANNEQUIN]
            used = 0
            for g in gt_m:
                for o in out_m:
                    if iou((g[2], g[3], g[4], g[5]),
                           (o[2], o[3], o[4], o[5])) >= 0.5:
                        hit.add(g[0])
                        used += 1
                        break
            if fpname == "off":
                fp_off += max(0, len(out_m) - used)
            else:
                fp_on += max(0, len(out_m) - used)

    n_victims = sum(1 for g in gts if g[0] == MANNEQUIN)
    stat["victims"] += n_victims
    stat["seen_any"] += len(seen_any)
    stat["conf_off"] += len(conf_off_hit)
    stat["conf_on"] += len(conf_on_hit)
    stat["frames_gt"] += frames_gt
    stat["frames_hit"] += frames_hit
    stat["fp_raw"] += fp_raw
    stat["fp_off"] += fp_off
    stat["fp_on"] += fp_on
    per_scene.append((stem, n_victims, len(seen_any), len(conf_on_hit)))

for k in ("seen_any", "conf_off", "conf_on"):
    if stat[k] > stat["victims"]:
        sys.exit("自检失败: %s=%d 超过伤员总数 %d —— 身份标识又错了"
                 % (k, stat[k], stat["victims"]))

print("=" * 62)
print("每个伤员在一次经过中的检出情况（%d 个伤员）" % stat["victims"])
print("=" * 62)
pf = stat["frames_hit"] / max(1, stat["frames_gt"])
print("  单帧召回（逐帧统计）        %.3f   <- 一直在报的数字" % pf)
print("  至少命中一次（理论上限）    %.3f   (%d/%d)"
      % (stat["seen_any"] / max(1, stat["victims"]),
         stat["seen_any"], stat["victims"]))
print("  tracker 确认, bridge=off    %.3f   (%d/%d)"
      % (stat["conf_off"] / max(1, stat["victims"]),
         stat["conf_off"], stat["victims"]))
print("  tracker 确认, bridge=on     %.3f   (%d/%d)"
      % (stat["conf_on"] / max(1, stat["victims"]),
         stat["conf_on"], stat["victims"]))
print()
print("误报总数（整段序列累计）")
print("  原始逐帧        %d" % stat["fp_raw"])
print("  确认后 off      %d  (%.0f%% 被压掉)"
      % (stat["fp_off"],
         100 * (1 - stat["fp_off"] / max(1, stat["fp_raw"]))))
print("  确认后 on       %d  (%.0f%% 被压掉)"
      % (stat["fp_on"],
         100 * (1 - stat["fp_on"] / max(1, stat["fp_raw"]))))
