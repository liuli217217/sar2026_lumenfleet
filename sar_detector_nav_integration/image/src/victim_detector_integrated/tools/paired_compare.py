#!/usr/bin/env python3
"""配对比较两次 5 折交叉验证。

两次跑的折是完全相同的（同样的切分、同样的种子），所以应该逐折配对比较，
而不是拿「均值 ± 标准差」互相看 —— 后者把折与折之间的固有难度差异
当成了噪声，会把真实差异淹掉。

用法: paired_compare.py <A的json> <B的json> [A名] [B名]
"""
import json
import pathlib
import statistics
import sys

a_path, b_path = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
a_name = sys.argv[3] if len(sys.argv) > 3 else "A"
b_name = sys.argv[4] if len(sys.argv) > 4 else "B"

A = json.loads(a_path.read_text())
B = json.loads(b_path.read_text())
NAMES = ["cone", "first_aid", "mannequin"]


def paired(get):
    rows = []
    for ra, rb in zip(A, B):
        va, vb = get(ra), get(rb)
        if va is None or vb is None:
            continue
        rows.append((ra["fold"], va, vb, vb - va))
    return rows


def verdict(diffs):
    """5 个折太少，不做正式显著性检验。只看方向是否一致 + 差值相对波动。"""
    if not diffs:
        return "无数据"
    n_better = sum(d > 0 for d in diffs)
    mean = statistics.mean(diffs)
    sd = statistics.stdev(diffs) if len(diffs) > 1 else 0.0
    se = sd / (len(diffs) ** 0.5) if sd else 0.0
    if n_better == len(diffs):
        tag = "5/5 折都变好"
    elif n_better == 0:
        tag = "5/5 折都变差"
    else:
        tag = "%d/5 折变好" % n_better
    if se and abs(mean) < se:
        tag += "，但差值小于标准误 —— 不能断言有差别"
    elif se and abs(mean) > 2 * se:
        tag += "，差值 > 2 倍标准误 —— 差别比较可信"
    return "%s (平均 %+.3f)" % (tag, mean)


print("配对比较： %s  ->  %s\n" % (a_name, b_name))

for label, getter in (
        ("整体 mAP50", lambda r: r.get("mAP50")),
        ("整体 mAP50-95", lambda r: r.get("mAP50_95"))):
    rows = paired(getter)
    print("=== %s ===" % label)
    print("  %-6s %8s %8s %8s" % ("fold", a_name[:8], b_name[:8], "差值"))
    for f, va, vb, d in rows:
        print("  %-6d %8.3f %8.3f %+8.3f" % (f, va, vb, d))
    print("  -> %s\n" % verdict([r[3] for r in rows]))

for cls in NAMES:
    for metric in ("R", "mAP50"):
        rows = paired(lambda r: (r.get(cls) or {}).get(metric))
        if not rows:
            continue
        print("=== %s / %s ===" % (cls, metric))
        print("  %-6s %8s %8s %8s" % ("fold", a_name[:8], b_name[:8], "差值"))
        for f, va, vb, d in rows:
            print("  %-6d %8.3f %8.3f %+8.3f" % (f, va, vb, d))
        print("  -> %s\n" % verdict([r[3] for r in rows]))
