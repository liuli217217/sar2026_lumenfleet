#!/usr/bin/env python3
"""把 Hazel v6 重新切分成一个像样的 train/val。

为什么要重切：原始 valid 只有 10 张图、14 个实例，其中 cone 只有 1 个、
first_aid 只有 2 个。这种规模下单类 AP 只能是 0% 或 100%，等于没有验证信号。
而且原 valid 的框面积中位数（cone 53%）和 train（cone 2%）完全不是一个分布。

用法: resplit.py <源数据集根> <输出根> [val比例] [随机种子]
"""
import collections
import pathlib
import random
import shutil
import sys

src = pathlib.Path(sys.argv[1]).resolve()
dst = pathlib.Path(sys.argv[2]).resolve()
val_frac = float(sys.argv[3]) if len(sys.argv) > 3 else 0.20
seed = int(sys.argv[4]) if len(sys.argv) > 4 else 0

NAMES = ["cone", "first_aid", "mannequin"]

# 收集所有 (图片, 标注) 对，不管它原来在哪个 split
pairs = []
for split in ("train", "valid", "val", "test"):
    img_dir, lbl_dir = src / split / "images", src / split / "labels"
    if not img_dir.is_dir():
        continue
    for ip in sorted(img_dir.iterdir()):
        if ip.suffix.lower() not in (".jpg", ".jpeg", ".png"):
            continue
        lp = lbl_dir / (ip.stem + ".txt")
        if lp.is_file():
            pairs.append((ip, lp))

print("收集到 %d 对图片/标注" % len(pairs))


def classes_of(lp):
    """这张图里出现了哪些类，以及每类几个。"""
    cnt = collections.Counter()
    for row in lp.read_text(errors="replace").splitlines():
        f = row.split()
        if len(f) >= 5:
            try:
                cnt[int(float(f[0]))] += 1
            except ValueError:
                pass
    return cnt


info = [(ip, lp, classes_of(lp)) for ip, lp in pairs]
total = collections.Counter()
for _, _, c in info:
    total.update(c)

n_val = max(1, round(len(info) * val_frac))
print("目标 val 图数 %d" % n_val)

# 按「这张图里有哪几类」分组，每组内按比例抽 val。
# 这样每种共现模式（只有 cone / cone+mannequin / ...）都按同样比例出现在两边。
#
# 不要按「实例数加权的稀有度」排序再贪心取 —— 那样会优先挑中目标最多的图，
# val 会把密集图全吃掉（实测过：val 20 张拿到 127 个实例，train 81 张只剩 107）。
rng = random.Random(seed)

# 密集离群图强制进 train，不许进 val。
# 起因：mannequin10.png 是一张网上的「人体模特腿」商品图，2x10 整齐排列 20 个实例，
# 而数据集里其他图最多 6 个。它一张就占了 val 里 mannequin 实例的 65%，
# 整个类的指标都由这一张说了算（实测召回被它从 ~0.91 拉到 0.35）。
# 它的外观对「残肢」是有用的训练信号，所以留在 train；但它不是场地照片，
# 不能用它来衡量部署表现。
# 阈值定在 10：真实场地照片里单帧最多 7 个目标（Image-2x 那批多锥桶的图），
# 那批是合法的验证数据，不能排除——第一次把阈值定成 6 就把它们全扫走了，
# val 从 56 个实例缩到 24 个，指标反而更不可信。只有那张 20 个实例的商品图
# 越过 10 这条线。
counts_per_img = sorted(sum(c.values()) for _, _, c in info)
med = counts_per_img[len(counts_per_img) // 2]
cap = max(10, 3 * med)
outliers = [x for x in info if sum(x[2].values()) > cap]
normal = [x for x in info if sum(x[2].values()) <= cap]
if outliers:
    print("密集离群图（每图实例数 > %d，强制进 train）: %d 张" % (cap, len(outliers)))
    for ip, _, c in outliers:
        print("   %-46s %d 个实例" % (ip.name[:46], sum(c.values())))

groups = collections.defaultdict(list)
for item in normal:
    groups[frozenset(item[2])].append(item)

val, train = [], []
# 组大的先分，保证大组的比例先被满足；小组再按四舍五入补
for sig in sorted(groups, key=lambda s: (-len(groups[s]), sorted(s))):
    g = groups[sig][:]
    rng.shuffle(g)
    k = int(round(len(g) * val_frac))
    # 整体还差很多 val 时，至少从每个 ≥2 张的组里拿 1 张
    if k == 0 and len(g) >= 2 and len(val) < n_val:
        k = 1
    val.extend(g[:k])
    train.extend(g[k:])

train.extend(outliers)

print("共现模式 %d 种: %s" % (
    len(groups),
    ", ".join("{%s}x%d" % ("+".join(NAMES[c] for c in sorted(s)) or "空", len(groups[s]))
              for s in sorted(groups, key=lambda s: -len(groups[s])))))

# 写出去
for name, items in (("train", train), ("val", val)):
    for sub in ("images", "labels"):
        d = dst / name / sub
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True)
    for ip, lp, _ in items:
        shutil.copy2(ip, dst / name / "images" / ip.name)
        shutil.copy2(lp, dst / name / "labels" / lp.name)

(dst / "data.yaml").write_text(
    "# 由 resplit.py 生成（原始 valid 只有 10 张，无法验证）\n"
    "path: %s\ntrain: train/images\nval: val/images\n\nnc: %d\nnames: %r\n"
    % (dst, len(NAMES), NAMES), encoding="utf-8")

print("\n%-8s %6s %8s %10s %12s %12s" % ("split", "图数", "实例", "cone", "first_aid", "mannequin"))
counts = {}
for name, items in (("train", train), ("val", val)):
    c = collections.Counter()
    for _, _, cc in items:
        c.update(cc)
    counts[name] = c
    print("%-8s %6d %8d %10d %12d %12d"
          % (name, len(items), sum(c.values()), c[0], c[1], c[2]))

print("\n%-8s %6s %8s %10s %12s %12s"
      % ("val占比", "", "", "cone", "first_aid", "mannequin"))
print("%-8s %6s %8s %9.0f%% %11.0f%% %11.0f%%"
      % ("", "", "",
         100 * counts["val"][0] / max(1, total[0]),
         100 * counts["val"][1] / max(1, total[1]),
         100 * counts["val"][2] / max(1, total[2])))

# 自检：val 不该吃掉过多实例，每类也不该为 0
bad = []
v_inst = sum(counts["val"].values())
if v_inst > 0.40 * sum(total.values()):
    bad.append("val 占了 %.0f%% 的实例，切分失衡"
               % (100 * v_inst / sum(total.values())))
for c in sorted(total):
    if counts["val"][c] == 0:
        bad.append("val 里没有 %s，该类无法验证" % NAMES[c])
    if counts["train"][c] == 0:
        bad.append("train 里没有 %s，该类学不到" % NAMES[c])
if bad:
    print("\n!! 切分有问题:")
    for b in bad:
        print("   -", b)
else:
    print("\n自检通过：两边各类都有，val 实例占比合理")

print("\n写入:", dst / "data.yaml")
