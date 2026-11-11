#!/usr/bin/env python3
"""模型在「什么都没有」的画面上会不会凭空报目标。

动机：在空白黑图上 conf=0.25 居然检出 3 个目标。场地里大片空墙、地板
会不会也这样？这直接决定误报率，必须查清楚。

用法: hallucination_test.py <权重> [对照权重]
"""
import sys

import numpy as np
from ultralytics import YOLO

NAMES = ["cone", "first_aid", "mannequin"]
rng = np.random.default_rng(0)

CASES = {
    "纯黑": np.zeros((480, 640, 3), np.uint8),
    "纯白": np.full((480, 640, 3), 255, np.uint8),
    "中灰": np.full((480, 640, 3), 128, np.uint8),
    "随机噪声": rng.integers(0, 256, (480, 640, 3), dtype=np.uint8),
    "墙面色(青灰)": np.full((480, 640, 3), (150, 140, 120), np.uint8),
    "地板色(浅灰)": np.full((480, 640, 3), (190, 190, 185), np.uint8),
    "轻微噪声灰": np.clip(
        128 + rng.normal(0, 8, (480, 640, 3)), 0, 255).astype(np.uint8),
}


def report(tag, weights):
    m = YOLO(weights)
    print("\n########## %s ##########" % tag)
    for conf in (0.10, 0.25, 0.50):
        print("  --- conf=%.2f ---" % conf)
        for name, img in CASES.items():
            r = m.predict(img, imgsz=640, conf=conf, verbose=False)[0]
            if len(r.boxes) == 0:
                print("    %-14s 干净" % name)
            else:
                det = [(NAMES[int(b.cls)] if int(b.cls) < len(NAMES)
                        else str(int(b.cls)), round(float(b.conf), 3))
                       for b in r.boxes]
                det.sort(key=lambda t: -t[1])
                print("    %-14s !! %d 个: %s" % (name, len(det), det[:6]))


report("微调后的三类模型", sys.argv[1])
if len(sys.argv) > 2:
    report("对照：原版 COCO yolov8s", sys.argv[2])
