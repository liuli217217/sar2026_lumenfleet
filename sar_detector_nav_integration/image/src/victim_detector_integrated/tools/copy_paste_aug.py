#!/usr/bin/env python3
"""把已有的 mannequin 实例抠出来，贴到别的训练图上，合成更多训练样本。

为什么这招对得上病根：
  1. 整个数据集只有 70 个 mannequin 实例 —— 实例数是硬瓶颈。
  2. 类别和尺寸强相关（first_aid 框占全图 34%，cone 只占 2%），
     模型可能学成「按大小猜类别」。随机缩放再贴，能把这个相关性打散。
  3. first_aid 的 27 张图里一个别的目标都没有 —— 上下文泄漏。
     往这些图里贴假人，强迫模型看物体本身而不是整张图的「氛围」。

只从 train 里抠、只贴到 train 图上。贴的位置和已有真值框不重叠。

用法: copy_paste_aug.py <切分好的数据集根> <输出目录> [每图贴几个] [倍数] [种子]
"""
import pathlib
import random
import sys

import cv2
import numpy as np

src = pathlib.Path(sys.argv[1])
out = pathlib.Path(sys.argv[2])
max_paste = int(sys.argv[3]) if len(sys.argv) > 3 else 3
n_rounds = int(sys.argv[4]) if len(sys.argv) > 4 else 2
seed = int(sys.argv[5]) if len(sys.argv) > 5 else 0

MANNEQUIN = 2
rng = random.Random(seed)
out_img, out_lbl = out / "images", out / "labels"
for d in (out_img, out_lbl):
    d.mkdir(parents=True, exist_ok=True)


def read_label(lp, W, H):
    rows = []
    for line in lp.read_text(errors="replace").splitlines():
        f = line.split()
        if len(f) < 5:
            continue
        c = int(float(f[0]))
        cx, cy, w, h = (float(v) for v in f[1:5])
        rows.append((c, (cx - w / 2) * W, (cy - h / 2) * H,
                     (cx + w / 2) * W, (cy + h / 2) * H))
    return rows


# ---- 1. 把所有 mannequin 抠出来 ----
tdir = src / "train"
bank = []
for ip in sorted((tdir / "images").iterdir()):
    if ip.suffix.lower() not in (".jpg", ".jpeg", ".png"):
        continue
    lp = tdir / "labels" / (ip.stem + ".txt")
    if not lp.is_file():
        continue
    img = cv2.imread(str(ip))
    if img is None:
        continue
    H, W = img.shape[:2]
    for c, x1, y1, x2, y2 in read_label(lp, W, H):
        if c != MANNEQUIN:
            continue
        x1, y1 = max(0, int(x1)), max(0, int(y1))
        x2, y2 = min(W, int(x2)), min(H, int(y2))
        if x2 - x1 < 20 or y2 - y1 < 20:
            continue
        bank.append(img[y1:y2, x1:x2].copy())
print("抠出 %d 个 mannequin 实例" % len(bank))
if not bank:
    sys.exit("没有可用的 mannequin 实例")


def fits(box, boxes):
    x1, y1, x2, y2 = box
    for _, bx1, by1, bx2, by2 in boxes:
        if not (x2 <= bx1 or x1 >= bx2 or y2 <= by1 or y1 >= by2):
            return False
    return True


# ---- 2. 贴 ----
n_out = 0
n_pasted = 0
for rnd in range(n_rounds):
    for ip in sorted((tdir / "images").iterdir()):
        if ip.suffix.lower() not in (".jpg", ".jpeg", ".png"):
            continue
        lp = tdir / "labels" / (ip.stem + ".txt")
        if not lp.is_file():
            continue
        img = cv2.imread(str(ip))
        if img is None:
            continue
        H, W = img.shape[:2]
        # 大图缩一下，否则贴完存盘太慢太占地方
        if max(H, W) > 1600:
            s = 1600.0 / max(H, W)
            img = cv2.resize(img, (int(W * s), int(H * s)))
            H, W = img.shape[:2]
        boxes = read_label(lp, W, H)
        canvas = img.copy()
        added = 0

        for _ in range(max_paste * 6):
            if added >= max_paste:
                break
            patch = bank[rng.randrange(len(bank))]
            ph, pw = patch.shape[:2]
            # 目标高度取整图的 8%~45%，大范围随机 —— 这是打散「尺寸=类别」的关键
            target_h = rng.uniform(0.08, 0.45) * H
            sc = target_h / ph
            nw, nh = int(pw * sc), int(ph * sc)
            if nw < 12 or nh < 12 or nw >= W or nh >= H:
                continue
            p = cv2.resize(patch, (nw, nh))
            if rng.random() < 0.5:
                p = cv2.flip(p, 1)
            # 躺倒的伤员很重要，旋转一部分
            if rng.random() < 0.35:
                p = cv2.rotate(p, rng.choice(
                    [cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE]))
                nh, nw = p.shape[:2]
                if nw >= W or nh >= H:
                    continue
            x1 = rng.randrange(0, W - nw)
            y1 = rng.randrange(0, H - nh)
            box = (x1, y1, x1 + nw, y1 + nh)
            if not fits(box, boxes):
                continue

            # 亮度对齐：让贴上去的块和周围环境亮度接近，不然模型会学「亮度突变 = 假人」
            around = canvas[y1:y1 + nh, x1:x1 + nw]
            if around.size:
                d = float(np.mean(around)) - float(np.mean(p))
                p = np.clip(p.astype(np.int16) + int(d * 0.6), 0, 255).astype(np.uint8)

            # 边缘羽化，减少硬边伪影
            mask = np.ones((nh, nw), np.float32)
            b = max(2, min(nh, nw) // 12)
            mask[:b, :] *= np.linspace(0, 1, b)[:, None]
            mask[-b:, :] *= np.linspace(1, 0, b)[:, None]
            mask[:, :b] *= np.linspace(0, 1, b)[None, :]
            mask[:, -b:] *= np.linspace(1, 0, b)[None, :]
            mask = mask[:, :, None]
            canvas[y1:y1 + nh, x1:x1 + nw] = (
                p * mask + canvas[y1:y1 + nh, x1:x1 + nw] * (1 - mask)
            ).astype(np.uint8)

            boxes.append((MANNEQUIN, x1, y1, x1 + nw, y1 + nh))
            added += 1
            n_pasted += 1

        if added == 0:
            continue
        name = "%s_cp%d" % (ip.stem[:38], rnd)
        cv2.imwrite(str(out_img / (name + ".jpg")), canvas)
        lines = []
        for c, x1, y1, x2, y2 in boxes:
            lines.append("%d %.6f %.6f %.6f %.6f" % (
                c, (x1 + x2) / 2 / W, (y1 + y2) / 2 / H,
                (x2 - x1) / W, (y2 - y1) / H))
        (out_lbl / (name + ".txt")).write_text("\n".join(lines) + "\n")
        n_out += 1

print("生成 %d 张合成图，新增 %d 个 mannequin 实例" % (n_out, n_pasted))
