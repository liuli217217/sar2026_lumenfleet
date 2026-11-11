#!/usr/bin/env python3
"""5 折交叉验证 + 最终模型。

为什么要做交叉验证：整个数据集只有 101 张图，单次 80/20 切分的验证集只有
20 张、几十个实例，一个实例就能让某类召回跳 10 个百分点。单个数字没有意义，
必须给出折间波动。

用法: kfold.py <源数据集> <工作目录> <基础权重> <折数> <epochs>
"""
import collections
import json
import pathlib
import random
import shutil
import statistics
import sys

from ultralytics import YOLO

src, work, base_w = sys.argv[1], pathlib.Path(sys.argv[2]), sys.argv[3]
K = int(sys.argv[4]) if len(sys.argv) > 4 else 5
EPOCHS = int(sys.argv[5]) if len(sys.argv) > 5 else 200
# 1 = 每折都从「该折的 train 图」里挖背景负样本。
# 必须逐折挖，不能全局挖一次 —— 否则某折的 val 图的背景会出现在它自己的训练集里。
MINE_NEG = len(sys.argv) > 6 and sys.argv[6] == "1"
NAMES = ["cone", "first_aid", "mannequin"]
CAP = 10          # 和 resplit.py 一致：>10 个实例的图只进 train
src = pathlib.Path(src)

# ---- 收集 ----
pairs = []
for split in ("train", "valid", "val", "test"):
    idir, ldir = src / split / "images", src / split / "labels"
    if not idir.is_dir():
        continue
    for ip in sorted(idir.iterdir()):
        if ip.suffix.lower() not in (".jpg", ".jpeg", ".png"):
            continue
        lp = ldir / (ip.stem + ".txt")
        if lp.is_file():
            cnt = collections.Counter()
            for row in lp.read_text(errors="replace").splitlines():
                f = row.split()
                if len(f) >= 5:
                    try:
                        cnt[int(float(f[0]))] += 1
                    except ValueError:
                        pass
            pairs.append((ip, lp, cnt))

always_train = [p for p in pairs if sum(p[2].values()) > CAP]
poolable = [p for p in pairs if sum(p[2].values()) <= CAP]
print("共 %d 对；强制进 train 的密集离群图 %d 张；参与轮换的 %d 张"
      % (len(pairs), len(always_train), len(poolable)))

# ---- 按共现模式分层，再轮流分到 K 个折 ----
rng = random.Random(0)
groups = collections.defaultdict(list)
for p in poolable:
    groups[frozenset(p[2])].append(p)
folds = [[] for _ in range(K)]
for sig in sorted(groups, key=lambda s: (-len(groups[s]), sorted(s))):
    g = groups[sig][:]
    rng.shuffle(g)
    for i, item in enumerate(g):
        folds[i % K].append(item)
print("每折图数: %s" % [len(f) for f in folds])


def mine_negatives(train_items, root, rng_seed=0):
    """从这一折的 train 图里裁出零目标区域，配空标注作为背景样本。"""
    import cv2
    import numpy as np
    r = random.Random(rng_seed)
    idir, ldir = root / "train" / "images", root / "train" / "labels"
    n = 0
    for ip, lp, _ in train_items:
        img = cv2.imread(str(ip))
        if img is None:
            continue
        H, W = img.shape[:2]
        gts = []
        for row in lp.read_text(errors="replace").splitlines():
            f = row.split()
            if len(f) >= 5:
                cx, cy, w, h = (float(v) for v in f[1:5])
                gts.append(((cx - w / 2) * W, (cy - h / 2) * H,
                            (cx + w / 2) * W, (cy + h / 2) * H))
        got = 0
        for _ in range(60):
            if got >= 2:
                break
            cw, ch = r.uniform(.25, .5) * W, r.uniform(.25, .5) * H
            x1, y1 = r.uniform(0, W - cw), r.uniform(0, H - ch)
            x2, y2 = x1 + cw, y1 + ch
            if any(not (x2 <= g[0] or x1 >= g[2] or y2 <= g[1] or y1 >= g[3])
                   for g in gts):
                continue
            crop = img[int(y1):int(y2), int(x1):int(x2)]
            if crop.size == 0 or (float(np.std(crop)) < 6 and r.random() < .7):
                continue
            nm = "%s_neg%d" % (ip.stem[:40], got)
            cv2.imwrite(str(idir / (nm + ".jpg")), crop)
            (ldir / (nm + ".txt")).write_text("")
            got += 1
            n += 1
    rngn = __import__("numpy").random.default_rng(rng_seed)
    deg = {"deg_black": np.zeros((480, 640, 3), np.uint8),
           "deg_white": np.full((480, 640, 3), 255, np.uint8),
           "deg_dark": np.full((480, 640, 3), 12, np.uint8),
           "deg_noise": rngn.integers(0, 256, (480, 640, 3), dtype=np.uint8)}
    for nm, im in deg.items():
        cv2.imwrite(str(idir / (nm + ".jpg")), im)
        (ldir / (nm + ".txt")).write_text("")
        n += 1
    return n


def materialise(train_items, val_items, root):
    for name, items in (("train", train_items), ("val", val_items)):
        for sub in ("images", "labels"):
            d = root / name / sub
            if d.exists():
                shutil.rmtree(d)
            d.mkdir(parents=True)
        for ip, lp, _ in items:
            shutil.copy2(ip, root / name / "images" / ip.name)
            shutil.copy2(lp, root / name / "labels" / lp.name)
    if MINE_NEG:
        n = mine_negatives(train_items, root)
        print("  本折额外加入 %d 张背景负样本" % n)
    (root / "data.yaml").write_text(
        "path: %s\ntrain: train/images\nval: val/images\nnc: 3\nnames: %r\n"
        % (root, NAMES), encoding="utf-8")
    return root / "data.yaml"


results = []
for k in range(K):
    val_items = folds[k]
    train_items = always_train + [x for i, f in enumerate(folds)
                                  if i != k for x in f]
    root = work / ("fold%d" % k)
    yaml = materialise(train_items, val_items, root)

    vc = collections.Counter()
    for _, _, c in val_items:
        vc.update(c)
    print("\n########## fold %d/%d  train %d 图 / val %d 图 (%s) ##########"
          % (k + 1, K, len(train_items), len(val_items),
             ", ".join("%s=%d" % (NAMES[i], vc[i]) for i in range(3))))

    m = YOLO(base_w)
    m.train(data=str(yaml), epochs=EPOCHS, imgsz=640, batch=16, device=0,
            workers=4, seed=0, deterministic=True, patience=40, cos_lr=True,
            scale=0.7, degrees=10.0, fliplr=0.5, flipud=0.0, mosaic=1.0,
            close_mosaic=15, hsv_v=0.4, project=str(work), name="run%d" % k,
            exist_ok=True, plots=False, val=True, verbose=False)
    mt = m.val(data=str(yaml), imgsz=640, device=0, verbose=False).box
    row = {"fold": k, "n_val_img": len(val_items),
           "inst": {NAMES[i]: vc[i] for i in range(3)},
           "mAP50": float(mt.map50), "mAP50_95": float(mt.map)}
    for i, name in enumerate(NAMES):
        try:
            row[name] = {"P": float(mt.p[i]), "R": float(mt.r[i]),
                         "mAP50": float(mt.ap50[i])}
        except Exception:
            row[name] = None
    results.append(row)
    print("fold %d: mAP50=%.3f  mAP50-95=%.3f" % (k, row["mAP50"], row["mAP50_95"]))

(work / "kfold_results.json").write_text(json.dumps(results, indent=2))

print("\n\n================ %d 折交叉验证汇总 ================" % K)


def summarise(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return "n/a"
    if len(vals) < 2:
        return "%.3f" % vals[0]
    return "%.3f ± %.3f  (最低 %.3f, 最高 %.3f)" % (
        statistics.mean(vals), statistics.stdev(vals), min(vals), max(vals))


print("整体 mAP50     : %s" % summarise([r["mAP50"] for r in results]))
print("整体 mAP50-95  : %s" % summarise([r["mAP50_95"] for r in results]))
print()
for name in NAMES:
    tot = sum(r["inst"][name] for r in results)
    print("%-11s (全部 %3d 个实例)" % (name, tot))
    for key in ("R", "mAP50"):
        print("   %-7s %s" % (key, summarise(
            [r[name][key] if r[name] else None for r in results])))
print("\n每折 val 实例数: %s"
      % [sum(r["inst"].values()) for r in results])
