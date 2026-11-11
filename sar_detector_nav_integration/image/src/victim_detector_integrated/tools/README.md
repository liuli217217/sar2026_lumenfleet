# 训练与评估脚本

数据集不在本仓库里（是组织方的数据，CC BY 4.0，见顶层 README 的 Attribution）。
先自行从 Roboflow 导出 YOLOv8 格式，解压到 `~/datasets/hazel_v6`。

```bash
# 1. 看清楚数据集长什么样（类别分布、框大小、共现模式）
python3 analyze_ds.py ~/datasets/hazel_v6 cone,first_aid,mannequin

# 2. 重新切分。原始 valid 只有 10 张图、cone 仅 1 个实例，没有验证信号。
python3 resplit.py ~/datasets/hazel_v6 ~/datasets/hazel_split 0.20 0

# 3. 训练
python3 train_sar.py ~/datasets/hazel_split/data.yaml yolov8s.pt ./runs v1 200

# 4. 诚实的数字：5 折交叉验证（末位 1 = 每折自动挖背景负样本）
python3 kfold.py ~/datasets/hazel_v6 ./runs/kfold yolov8s.pt 5 200 1

# 5. 置信度阈值 -> 召回/误报
python3 conf_sweep.py ./runs/kfold 5

# 6. 空画面幻觉测试（微调过的模型会在纯黑图上报 mannequin）
python3 hallucination_test.py ./runs/kfold/run0/weights/best.pt yolov8s.pt
```

`diag_mannequin.py` 逐个真值框给出漏检原因（完全没框 / 分数低 / 分错类），
`render_one.py` 把单张图的真值和预测并排画出来。
