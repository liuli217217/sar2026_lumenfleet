#!/usr/bin/env python3
"""微调 yolov8s 到 SAR 三类 (cone / first_aid / mannequin)。

用法: train_sar.py <data.yaml> <权重> <输出目录> <run名> [epochs]

数据集只有 81 张训练图，所以配置的重点全在抗过拟合：
  * scale=0.7（默认 0.5）—— 数据集里类别和目标大小强相关
    （first_aid 框中位数占全图 34%，cone 只占 2%），
    不打散尺度的话模型会学成「大的就是急救包」。
  * mosaic=1.0 —— 把 4 张图拼一张，顺带破掉「整张图只有一类」的上下文泄漏
    （first_aid 的 27 张图里一个别的目标都没有）。
  * degrees=10 —— 四足机器人走起来相机会侧倾。
  * cos_lr + patience —— 小数据集很快就过拟合，让它自己早停。
"""
import sys

from ultralytics import YOLO

data, weights, project, name = sys.argv[1:5]
epochs = int(sys.argv[5]) if len(sys.argv) > 5 else 200
imgsz = int(sys.argv[6]) if len(sys.argv) > 6 else 640
# 分辨率翻倍显存大约翻四倍，batch 跟着降
batch = int(sys.argv[7]) if len(sys.argv) > 7 else (16 if imgsz <= 640 else 8)

model = YOLO(weights)
model.train(
    data=data,
    epochs=epochs,
    imgsz=imgsz,        # 默认 640，和部署节点的默认值一致
    batch=batch,
    device=0,
    workers=4,
    seed=0,
    deterministic=True,
    patience=40,
    cos_lr=True,
    # 增强
    scale=0.7,
    degrees=10.0,
    fliplr=0.5,
    flipud=0.0,         # 场地照片都是正的，上下翻转没意义
    mosaic=1.0,
    close_mosaic=15,    # 最后 15 轮关掉 mosaic，让它见真实构图
    hsv_v=0.4,          # 室内光照差异大
    project=project,
    name=name,
    exist_ok=True,
    plots=True,
    val=True,
)
print("训练完成")
