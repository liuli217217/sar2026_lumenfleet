#!/usr/bin/env python3
"""从训练图里裁出「确实没有目标」的区域，当作背景负样本。

动机：微调后的模型在纯黑图上报 3 个 mannequin（最高 0.407），原版 COCO 不会。
101 张图里没有一张背景图，模型没学过「什么都没有」长什么样。
精确率只有 0.51，为了压误报就得把 conf 调高，召回跟着掉 —— 补负样本
是同时改善两边的正路子。

YOLO 的约定：图片配一个空的 .txt 就是背景图。

裁剪规则：和任何真值框的 IoU 必须为 0，而且不能包含任何真值框的中心点。
用法: mine_negatives.py <数据集根> <输出目录> [每图裁几块] [随机种子]
"""
import pathlib
import random
import sys

import cv2
import numpy as np

src = pathlib.Path(sys.argv[1])
out = pathlib.Path(sys.argv[2])
per_img = int(sys.argv[3]) if len(sys.argv) > 3 else 2
seed = int(sys.argv[4]) if len(sys.argv) > 4 else 0
# 只从指定的 split 里挖。必须限制成 train —— 从 val 图里裁背景
# 等于把验证集的背景泄漏进训练集。
SPLITS = sys.argv[5].split(",") if len(sys.argv) > 5 else ["train"]
print("从这些 split 里挖负样本:", SPLITS)

out_img = out / "images"
out_lbl = out / "labels"
for d in (out_img, out_lbl):
    d.mkdir(parents=True, exist_ok=True)

rng = random.Random(seed)


def overlaps(box, gts):
    """box 和任何真值框有交集，或者含有真值框中心 -> True"""
    x1, y1, x2, y2 = box
    for gx1, gy1, gx2, gy2 in gts:
        if not (x2 <= gx1 or x1 >= gx2 or y2 <= gy1 or y1 >= gy2):
            return True
        cx, cy = (gx1 + gx2) / 2, (gy1 + gy2) / 2
        if x1 <= cx <= x2 and y1 <= cy <= y2:
            return True
    return False


n_written = 0
n_imgs = 0
for split in SPLITS:
    idir, ldir = src / split / "images", src / split / "labels"
    if not idir.is_dir():
        continue
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
        n_imgs += 1

        gts = []
        for row in lp.read_text(errors="replace").splitlines():
            f = row.split()
            if len(f) < 5:
                continue
            cx, cy, w, h = (float(v) for v in f[1:5])
            gts.append(((cx - w / 2) * W, (cy - h / 2) * H,
                        (cx + w / 2) * W, (cy + h / 2) * H))

        got = 0
        for _ in range(60):                 # 最多试 60 次
            if got >= per_img:
                break
            cw = rng.uniform(0.25, 0.5) * W
            ch = rng.uniform(0.25, 0.5) * H
            x1 = rng.uniform(0, W - cw)
            y1 = rng.uniform(0, H - ch)
            box = (x1, y1, x1 + cw, y1 + ch)
            if overlaps(box, gts):
                continue
            crop = img[int(y1):int(y1 + ch), int(x1):int(x1 + cw)]
            if crop.size == 0:
                continue
            # 太平的区域（纯色墙）保留一部分就够，不要刷屏
            if float(np.std(crop)) < 6 and rng.random() < 0.7:
                continue
            name = "%s_neg%d.jpg" % (ip.stem[:40], got)
            cv2.imwrite(str(out_img / name), crop)
            (out_lbl / (name[:-4] + ".txt")).write_text("")   # 空标注 = 背景
            got += 1
            n_written += 1

print("扫过 %d 张图，裁出 %d 张背景负样本" % (n_imgs, n_written))

# 再补几张退化图，专门治「纯黑报 mannequin」
rngn = np.random.default_rng(seed)
degenerate = {
    "deg_black": np.zeros((480, 640, 3), np.uint8),
    "deg_white": np.full((480, 640, 3), 255, np.uint8),
    "deg_dark": np.full((480, 640, 3), 12, np.uint8),
    "deg_dark_noise": np.clip(
        rngn.normal(10, 4, (480, 640, 3)), 0, 255).astype(np.uint8),
    "deg_noise": rngn.integers(0, 256, (480, 640, 3), dtype=np.uint8),
}
for name, img in degenerate.items():
    cv2.imwrite(str(out_img / (name + ".jpg")), img)
    (out_lbl / (name + ".txt")).write_text("")
    n_written += 1
print("加上 %d 张退化图，背景样本共 %d 张" % (len(degenerate), n_written))
