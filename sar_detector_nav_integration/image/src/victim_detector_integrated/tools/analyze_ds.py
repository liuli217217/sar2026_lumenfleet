#!/usr/bin/env python3
"""分析一个 YOLO 格式数据集。数据集根目录作为参数传入。

用 python3 -I 跑（隔离模式），因为数据集是下载来的，不信任它所在目录。
"""
import collections
import pathlib
import sys

root = pathlib.Path(sys.argv[1]).resolve()
names = sys.argv[2].split(",") if len(sys.argv) > 2 else None

try:
    from PIL import Image
except Exception:
    Image = None


def scan(split):
    img_dir = root / split / "images"
    lbl_dir = root / split / "labels"
    if not img_dir.is_dir():
        return None

    imgs = sorted(p for p in img_dir.iterdir() if p.suffix.lower()
                  in (".jpg", ".jpeg", ".png"))
    lbls = sorted(p for p in lbl_dir.iterdir() if p.suffix.lower() == ".txt") \
        if lbl_dir.is_dir() else []

    inst = collections.Counter()      # 每类实例数
    imgs_with = collections.Counter()  # 每类出现在多少张图里
    empty, bad = [], []
    areas = collections.defaultdict(list)
    per_img = []

    stems = {p.stem for p in imgs}
    lbl_stems = {p.stem for p in lbls}

    for lp in lbls:
        rows = [r for r in lp.read_text(errors="replace").splitlines() if r.strip()]
        if not rows:
            empty.append(lp.name)
            per_img.append(0)
            continue
        seen = set()
        n = 0
        for r in rows:
            f = r.split()
            if len(f) < 5:
                bad.append("%s: %r" % (lp.name, r[:40]))
                continue
            try:
                c = int(float(f[0]))
                w, h = float(f[3]), float(f[4])
            except ValueError:
                bad.append("%s: %r" % (lp.name, r[:40]))
                continue
            inst[c] += 1
            seen.add(c)
            areas[c].append(w * h)
            n += 1
        per_img.append(n)
        for c in seen:
            imgs_with[c] += 1

    return dict(imgs=imgs, lbls=lbls, inst=inst, imgs_with=imgs_with,
                empty=empty, bad=bad, areas=areas, per_img=per_img,
                no_label=sorted(stems - lbl_stems),
                no_image=sorted(lbl_stems - stems))


def nm(c):
    if names and 0 <= c < len(names):
        return "%d %s" % (c, names[c])
    return str(c)


print("数据集根目录:", root)
splits = {}
for s in ("train", "valid", "val", "test"):
    r = scan(s)
    if r:
        splits[s] = r

if not splits:
    sys.exit("找不到 train/valid 子目录")

total_inst = collections.Counter()
for s, r in splits.items():
    print("\n=== %s ===" % s)
    print("  图片 %d 张, 标注 %d 个" % (len(r["imgs"]), len(r["lbls"])))
    if r["no_label"]:
        print("  !! %d 张图没有对应标注: %s" % (len(r["no_label"]), r["no_label"][:5]))
    if r["no_image"]:
        print("  !! %d 个标注没有对应图片: %s" % (len(r["no_image"]), r["no_image"][:5]))
    if r["empty"]:
        print("  空标注(背景图) %d 个: %s" % (len(r["empty"]), r["empty"][:3]))
    if r["bad"]:
        print("  !! 格式错误行 %d 条: %s" % (len(r["bad"]), r["bad"][:3]))

    tot = sum(r["inst"].values())
    print("  实例总数 %d, 平均每图 %.1f 个"
          % (tot, tot / max(1, len(r["lbls"]))))
    print("  %-16s %8s %8s %10s" % ("类别", "实例数", "占比", "出现图数"))
    for c in sorted(set(r["inst"]) | set(r["imgs_with"])):
        print("  %-16s %8d %7.1f%% %10d"
              % (nm(c), r["inst"][c], 100 * r["inst"][c] / max(1, tot),
                 r["imgs_with"][c]))
        total_inst[c] += r["inst"][c]

    # 目标大小：决定 imgsz 够不够
    if r["areas"]:
        print("  框面积（占整图比例，中位数）:")
        for c in sorted(r["areas"]):
            a = sorted(r["areas"][c])
            med = a[len(a) // 2]
            print("    %-14s 中位 %.3f%%  最小 %.4f%%  最大 %.2f%%"
                  % (nm(c), 100 * med, 100 * a[0], 100 * a[-1]))

print("\n=== 全体合计 ===")
g = sum(total_inst.values())
for c in sorted(total_inst):
    print("  %-16s %6d  (%.1f%%)" % (nm(c), total_inst[c],
                                     100 * total_inst[c] / max(1, g)))
print("  实例总计 %d" % g)

# 图片尺寸：训练 imgsz 的参考
if Image is not None:
    sizes = collections.Counter()
    for s, r in splits.items():
        for p in r["imgs"][:400]:
            try:
                with Image.open(p) as im:
                    sizes[im.size] += 1
            except Exception:
                pass
    print("\n=== 图片尺寸 ===")
    for sz, n in sizes.most_common(8):
        print("  %sx%s : %d 张" % (sz[0], sz[1], n))
else:
    print("\n(PIL 不可用，跳过尺寸统计)")
