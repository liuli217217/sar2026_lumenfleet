#!/usr/bin/env python3
"""把一张图的真值框和预测框画出来并排对比。

用法: render_one.py <权重> <图片> <标注txt> <输出png>
"""
import pathlib
import sys

import cv2
from ultralytics import YOLO

weights, img_path, lbl_path, out_path = sys.argv[1:5]
NAMES = ["cone", "first_aid", "mannequin"]
COL = [(0, 165, 255), (0, 200, 0), (0, 0, 255)]

img = cv2.imread(img_path)
H, W = img.shape[:2]
print("图片尺寸: %dx%d" % (W, H))

gt_img = img.copy()
n_gt = 0
areas = []
for line in pathlib.Path(lbl_path).read_text(errors="replace").splitlines():
    f = line.split()
    if len(f) < 5:
        continue
    c = int(float(f[0]))
    cx, cy, w, h = (float(v) for v in f[1:5])
    x1, y1 = int((cx - w / 2) * W), int((cy - h / 2) * H)
    x2, y2 = int((cx + w / 2) * W), int((cy + h / 2) * H)
    cv2.rectangle(gt_img, (x1, y1), (x2, y2), COL[c], 3)
    cv2.putText(gt_img, NAMES[c], (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, COL[c], 2)
    n_gt += 1
    areas.append((w * W, h * H, 100 * w * h))

print("真值框 %d 个" % n_gt)
for i, (bw, bh, pct) in enumerate(sorted(areas, key=lambda t: t[2])[:8]):
    print("  最小的第%d个: %.0fx%.0f px, 占全图 %.2f%%" % (i + 1, bw, bh, pct))

model = YOLO(weights)
res = model.predict(img_path, imgsz=640, conf=0.05, iou=0.6, verbose=False)[0]
pr_img = img.copy()
print("预测框 %d 个 (conf>=0.05)" % len(res.boxes))
for b in res.boxes:
    c = int(b.cls)
    conf = float(b.conf)
    x1, y1, x2, y2 = (int(v) for v in b.xyxy[0].tolist())
    cv2.rectangle(pr_img, (x1, y1), (x2, y2), COL[c], 3)
    cv2.putText(pr_img, "%s %.2f" % (NAMES[c], conf), (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, COL[c], 2)

# 并排，缩到合适宽度
scale = 1100.0 / W
small = lambda im: cv2.resize(im, (int(W * scale), int(H * scale)))
a, b_ = small(gt_img), small(pr_img)
cv2.putText(a, "GROUND TRUTH (%d)" % n_gt, (12, 34),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)
cv2.putText(b_, "PREDICTED (%d)" % len(res.boxes), (12, 34),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)
cv2.imwrite(out_path, cv2.hconcat([a, b_]))
print("写入:", out_path)
