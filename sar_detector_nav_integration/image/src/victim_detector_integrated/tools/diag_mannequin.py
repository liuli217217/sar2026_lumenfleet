#!/usr/bin/env python3
"""查清楚 mannequin 为什么召回只有 0.31。

对 val 里每一个真值框，找 IoU 最高的预测框，报告：
  * 有没有被检出（IoU>=0.5）
  * 没检出的话，模型在那个位置给出的最高置信度是多少
  * 漏检的框有多大、是不是贴边（部分出画）

用法: diag_mannequin.py <权重> <data.yaml 所在的数据集根> <输出目录>
"""
import collections
import pathlib
import sys

from ultralytics import YOLO

weights, root, outdir = sys.argv[1:4]
root = pathlib.Path(root)
outdir = pathlib.Path(outdir)
outdir.mkdir(parents=True, exist_ok=True)
NAMES = ["cone", "first_aid", "mannequin"]

model = YOLO(weights)


def iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


img_dir = root / "val" / "images"
lbl_dir = root / "val" / "labels"
rows = []
for ip in sorted(img_dir.iterdir()):
    if ip.suffix.lower() not in (".jpg", ".jpeg", ".png"):
        continue
    lp = lbl_dir / (ip.stem + ".txt")
    if not lp.is_file():
        continue

    # 用很低的 conf 跑，这样能看到「模型其实看到了但分数低」的情况
    res = model.predict(str(ip), imgsz=640, conf=0.01, iou=0.6, verbose=False)[0]
    W, H = res.orig_shape[1], res.orig_shape[0]

    preds = []
    for b in res.boxes:
        x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
        preds.append((int(b.cls), float(b.conf), (x1, y1, x2, y2)))

    for line in lp.read_text(errors="replace").splitlines():
        f = line.split()
        if len(f) < 5:
            continue
        c = int(float(f[0]))
        cx, cy, w, h = (float(v) for v in f[1:5])
        gt = ((cx - w / 2) * W, (cy - h / 2) * H, (cx + w / 2) * W, (cy + h / 2) * H)

        best_same = (0.0, 0.0)          # (iou, conf) 同类里最好的
        best_any = (0.0, 0.0, -1)       # (iou, conf, cls) 任意类里最好的
        for pc, pconf, pbox in preds:
            v = iou(gt, pbox)
            if pc == c and v > best_same[0]:
                best_same = (v, pconf)
            if v > best_any[0]:
                best_any = (v, pconf, pc)

        # 贴边程度：框有多少比例贴在画面边缘（说明目标部分出画）
        edge = (gt[0] <= 2 or gt[1] <= 2 or gt[2] >= W - 2 or gt[3] >= H - 2)
        rows.append(dict(img=ip.name, cls=c, area=w * h, edge=edge,
                         aspect=(w * W) / max(1e-6, h * H),
                         iou=best_same[0], conf=best_same[1],
                         any_iou=best_any[0], any_conf=best_any[1],
                         any_cls=best_any[2]))

print("val 真值框共 %d 个\n" % len(rows))

for c in sorted({r["cls"] for r in rows}):
    sub = [r for r in rows if r["cls"] == c]
    hit = [r for r in sub if r["iou"] >= 0.5]
    miss = [r for r in sub if r["iou"] < 0.5]
    print("=== %s: %d 个真值, 检出 %d, 漏检 %d (召回 %.2f) ==="
          % (NAMES[c], len(sub), len(hit), len(miss), len(hit) / len(sub)))

    if hit:
        cs = sorted(r["conf"] for r in hit)
        print("  检出的置信度: 中位 %.2f, 最低 %.2f" % (cs[len(cs) // 2], cs[0]))

    if miss:
        # 漏检的为什么漏？分三种情况
        nothing = [r for r in miss if r["any_iou"] < 0.3]
        lowconf = [r for r in miss if r["any_iou"] >= 0.5 and r["any_cls"] == c]
        wrongcls = [r for r in miss if r["any_iou"] >= 0.5 and r["any_cls"] != c]
        print("  漏检原因拆分:")
        print("    完全没有框     %d  (模型在那个位置什么都没看到)" % len(nothing))
        print("    框对了但分数低 %d  (调低 conf 能救)" % len(lowconf))
        print("    框对了但分错类 %d  (类别混淆)" % len(wrongcls))
        if wrongcls:
            w = collections.Counter(NAMES[r["any_cls"]] for r in wrongcls)
            print("      被分成了: %s" % dict(w))
        if lowconf:
            print("      这些的分数: %s"
                  % [round(r["any_conf"], 3) for r in lowconf][:10])

        ma = sorted(r["area"] for r in miss)
        ha = sorted(r["area"] for r in hit) if hit else [0]
        print("  漏检框面积中位 %.3f%%  vs 检出框面积中位 %.3f%%"
              % (100 * ma[len(ma) // 2], 100 * ha[len(ha) // 2]))
        print("  漏检里贴边(部分出画)的: %d/%d；检出里贴边的: %d/%d"
              % (sum(r["edge"] for r in miss), len(miss),
                 sum(r["edge"] for r in hit), len(hit)))
        asp = sorted(r["aspect"] for r in miss)
        print("  漏检框宽高比中位 %.2f (>1 是横着的，躺倒的人应该 >1)"
              % asp[len(asp) // 2])
        print("  漏检最多的图: %s"
              % collections.Counter(r["img"] for r in miss).most_common(4))
    print()
