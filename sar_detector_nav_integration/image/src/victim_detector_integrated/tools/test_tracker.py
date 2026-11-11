#!/usr/bin/env python3
"""测 _Tracker 的逻辑。不依赖 ROS，直接从节点源码里把类抠出来执行。

要验证的性质：
  1. 少于 min_hits 帧的目标不确认（压掉闪一下的误报）
  2. 连续命中够了就确认
  3. 目标移动时能跟住，不会断成两个 track
  4. 丢几帧能容忍，超过 max_misses 才丢弃
  5. 不同类别不会互相关联
  6. 间歇性检测（真实情况：模型时有时无）最终能确认
"""
import ast
import pathlib
import sys

srcfile = pathlib.Path(sys.argv[1])
tree = ast.parse(srcfile.read_text(encoding="utf-8"))
wanted = {"_iou", "_Track", "_Tracker"}
ns = {}
mod = ast.Module(body=[n for n in tree.body
                       if getattr(n, "name", None) in wanted],
                 type_ignores=[])
exec(compile(mod, "<extracted>", "exec"), ns)        # noqa: S102
Tracker = ns["_Tracker"]

fails = []


def check(name, cond, detail=""):
    print("  %-52s %s" % (name, "OK" if cond else "失败  " + detail))
    if not cond:
        fails.append(name)


BOX = (100.0, 100.0, 200.0, 300.0)


def det(cls=2, score=0.8, box=BOX):
    return (cls, score, box[0], box[1], box[2], box[3])


print("=== 1. 少于 min_hits 不确认 ===")
t = Tracker(0.3, 5, 3)
check("第 1 帧不确认", len(t.update([det()])) == 0)
check("第 2 帧不确认", len(t.update([det()])) == 0)
out = t.update([det()])
check("第 3 帧确认", len(out) == 1, str(out))

print("\n=== 2. 一闪而过的误报不会被确认 ===")
t = Tracker(0.3, 5, 3)
t.update([det(score=0.9)])
for _ in range(6):
    r = t.update([])
check("单帧误报始终不确认", all(len(t.update([])) == 0 for _ in range(3)))

print("\n=== 3. 目标移动时跟得住 ===")
t = Tracker(0.3, 5, 3)
ids = set()
for i in range(8):
    # 每帧平移 12 像素，框 100x200，IoU 仍远高于 0.3
    b = (100.0 + 12 * i, 100.0, 200.0 + 12 * i, 300.0)
    out = t.update([det(box=b)])
    for o in out:
        ids.add(o[6])
check("移动过程中只产生 1 个 track id", len(ids) == 1, "ids=%s" % ids)
check("内部 track 数没有膨胀", len(t.tracks) == 1, "有 %d 个" % len(t.tracks))

print("\n=== 4. 容忍丢帧 ===")
t = Tracker(0.3, max_misses=5, min_hits=3)
for _ in range(3):
    t.update([det()])
check("确认后丢 1 帧，track 还在", (t.update([]) is not None
                                   and len(t.tracks) == 1))
for _ in range(3):
    t.update([])
check("丢 4 帧仍保留（未超 max_misses=5）", len(t.tracks) == 1,
      "剩 %d" % len(t.tracks))
for _ in range(4):
    t.update([])
check("丢够 max_misses 后被清掉", len(t.tracks) == 0,
      "还剩 %d" % len(t.tracks))

print("\n=== 5. 确认后若当帧没检到，不应继续上报 ===")
t = Tracker(0.3, 5, 3)
for _ in range(3):
    t.update([det()])
out_miss = t.update([])
check("丢帧当帧不上报（misses>0 被过滤）", len(out_miss) == 0, str(out_miss))

print("\n=== 6. 不同类别不互相关联 ===")
t = Tracker(0.3, 5, 2)
t.update([det(cls=2)])
t.update([det(cls=0)])          # 同一位置但不同类
check("同位置不同类产生 2 个 track", len(t.tracks) == 2,
      "有 %d 个" % len(t.tracks))

print("\n=== 7. 间歇性检测最终能确认（真实场景）===")
t = Tracker(0.3, max_misses=5, min_hits=3)
pattern = [1, 0, 1, 0, 1, 1]     # 模型时有时无
confirmed_at = None
for i, hit in enumerate(pattern):
    out = t.update([det()] if hit else [])
    if out and confirmed_at is None:
        confirmed_at = i
check("间歇命中也能确认", confirmed_at is not None,
      "pattern=%s" % pattern)
check("确认发生在第 5 帧（第 3 次命中）", confirmed_at == 4,
      "实际第 %s 帧" % confirmed_at)

print("\n=== 8. 多目标同时跟踪 ===")
t = Tracker(0.3, 5, 2)
a = (10.0, 10.0, 60.0, 110.0)
b = (400.0, 200.0, 460.0, 320.0)
for _ in range(3):
    out = t.update([det(box=a), det(box=b)])
check("两个目标都确认", len(out) == 2, "只确认了 %d 个" % len(out))
check("两个 id 不同", len({o[6] for o in out}) == 2)

print()
if fails:
    print("!! %d 项失败: %s" % (len(fails), fails))
    sys.exit(1)
print("全部通过")
