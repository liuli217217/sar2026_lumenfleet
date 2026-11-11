# victim_detector — arm64 / Jetson deployment notes

For whoever owns the team ROS 2 Docker image. Target: Jetson AGX Orin,
JetPack 7.2.1, CUDA 13.2, TensorRT 10.16, Python 3.12, **offline on competition
day**.

This is a fragment for your Dockerfile, not a standalone image — you own the
base image, I own the package.

---

## 0. Run this on the robot first

`scripts/jetson_check.sh` is read-only, needs no sudo, installs nothing.

```bash
scp jetson_check.sh <user>@<robot>:~/
ssh <user>@<robot> 'bash ~/jetson_check.sh 2>&1 | tee jetson_report.txt'
```

It prints a `判定` (verdict) line in section 5. Everything below is keyed off it.
It auto-sources ROS, because a non-interactive `ssh` never reads `.bashrc`.

---

## 1. The one thing that is actually hard

`pip install ultralytics` pulls torch/torchvision wheels that **do not run on
Orin**, and the failure is late and confusing: `import torch` succeeds,
`torch.cuda.is_available()` returns `True`, and then the first real inference
dies with

```
CUDA error: no kernel image is available for execution on the device
```

Cause: Orin is compute capability **sm_87**. The generic wheels are built for
other architectures. Verified concretely — the `torch 2.15.0+cu132` on our dev
box advertises `sm_75 sm_80 sm_86 sm_90 sm_100 sm_120` and **no sm_87**, so
copying the dev environment to the robot would not work.

**Fix: install torch from the CUDA-matched index, after ultralytics, so it wins.**

```dockerfile
# order matters: ultralytics first, then overwrite its torch
RUN pip install --no-cache-dir ultralytics==8.4.199 \
 && pip install --no-cache-dir --force-reinstall \
      torch torchvision \
      --index-url https://download.pytorch.org/whl/cu132
```

The `cu132` suffix must match the CUDA toolkit on the device. `jetson_check.sh`
section 2 prints the right one on the line `-> 应使用的 pip index`. CUDA 13.2 →
`cu132`, 13.0 → `cu130`.

Then assert it at build time, so a bad wheel fails the build instead of the
field test:

```dockerfile
RUN python3 -c "import torch; \
    a=torch.cuda.get_arch_list(); print('archs:', a); \
    assert any('87' in x for x in a), 'wheel has no sm_87 - wrong index URL'"
```

Note the build-time check can only inspect the arch list; the real kernel-launch
test needs the GPU, so run `jetson_check.sh` **inside the container** on the
robot once it is built.

A cosmetic `Found GPU ... compute capability` warning may still appear. It is
harmless; NVIDIA says upstream will drop it.

---

## 2. NumPy and OpenCV — the other trap

ROS 2 Jazzy's `cv_bridge` is compiled against NumPy 1.x. `ultralytics` pulls
NumPy 2.x, and then `import cv_bridge` crashes. Also do **not** install
`opencv-python` from pip — it shadows the system `cv2` and re-demands NumPy ≥ 2.

```dockerfile
RUN pip install --no-cache-dir "numpy<2" \
 && pip uninstall -y opencv-python opencv-python-headless || true
RUN python3 -c "import numpy, cv2, cv_bridge; \
    print('numpy', numpy.__version__, '| cv2', cv2.__version__); \
    assert numpy.__version__.startswith('1'), 'numpy 2 present - cv_bridge will break'"
```

Both of these are already in the package README; repeated here because they are
build-order constraints, not runtime ones.

---

## 3. Offline: bake the weights in

Ultralytics downloads weights **and a font** on first use. On competition day
there is no network, and the node will fail to start. Copy the weights in and
pass an absolute path.

```dockerfile
ENV YOLO_CONFIG_DIR=/opt/ultralytics_cfg
RUN mkdir -p /opt/models /opt/ultralytics_cfg
COPY models/sar_3class.pt /opt/models/sar_3class.pt
```

Launch with the absolute path:

```bash
ros2 launch victim_detector sar_detector.launch.py \
    weights:=/opt/models/sar_3class.pt keep_classes:="" half:=true
```

**Verify offline for real**, not by assumption:

```bash
docker run --rm --runtime nvidia --network none <image> \
  ros2 launch victim_detector sar_detector.launch.py \
      weights:=/opt/models/sar_3class.pt
```

`--network none` is the whole point. If it starts and the `~/status` topic
reports frames, it will work in the arena.

---

## 4. Performance expectations

Ultralytics' published AGX Orin 64GB numbers (FP32, imgsz 640, inference only —
their power mode is not stated, so treat these as a ceiling, not a promise):

| backend | ms/image |
|---|---|
| PyTorch | 11.6 |
| TorchScript | 4.6 |
| ONNX | 9.9 |
| TensorRT FP32 | 4.2 |
| TensorRT FP16 | 2.6 |

Those are for the smallest model. Ours is `yolov8s`-class and the Orin is
**capped at 50 W**, so expect materially worse. Our measured dev-box figure is
50–53 ms/frame on an RTX 3070 Ti at `imgsz=640`.

Levers in order of preference:

1. `half:=true` — FP16, free accuracy-neutral win on Orin
2. `max_rate_hz:=5` — cap inference rate; the node drops frames cleanly and
   reports `skipped` in `~/status`
3. `publish_annotated:=false` — stops encoding and publishing a debug JPEG
4. TensorRT export — biggest win, most work

**TensorRT engines must be built on the robot.** Ultralytics is explicit that
engines are device- and version-specific and must be validated on the
deployment device. Do not ship a prebuilt `.engine`. Also: keep TensorRT 10.x —
do not upgrade to 11.x on JetPack 7.

Read actual numbers off `~/status` before tuning anything.

---

## 5. Decision table

Keyed to `jetson_check.sh` output.

| what the report says | what to do |
|---|---|
| §5 `判定: OK` | torch is fine. Proceed. |
| §5 `架构警告: 不含 sm_87` | wrong wheel. Reinstall from the `cu132` index (§1). |
| §5 `GPU kernel 测试 失败` | same fix as above; this is the definitive symptom. |
| §5 `torch 未安装` | install per §1. |
| §5 `cuda.is_available() False` | container missing `--runtime nvidia`, or driver/CUDA mismatch. Check §2 and §7. |
| §6 `numpy 2.x` | pin `numpy<2` (§2) or `cv_bridge` will crash on import. |
| §2 index ≠ `cu132` | use the index the script prints, not the one in this document. |
| §7 `nvidia runtime 没有` | container gets no GPU. Needs host-side fix by whoever has sudo. |
| §7 `daemon 可访问 no` | user not in `docker` group. Needs admin. |
| §8 no camera topics | camera node not running, or `ROS_DOMAIN_ID` mismatch between containers. |
| §9 `解析不了 pypi.org` | already offline — all pip installs must happen at image build time. |

---

## 6. Still open

- **Model.** The node is done; the weights are not. Blocked on the organisers'
  Roboflow annotation export. Current default (`yolov8s` + COCO `person`) is a
  stand-in and is **not** competition-ready — 17/38 on real arena photos.
- **Scope.** 1 class (mannequin) or 3 (mannequin / cone / first_aid)? The
  organisers' dataset has 3 and the Phase 2 task list mentions hazard reporting.
  Needs a team decision; it changes the training run, not the node.
- **Nothing in this document has been run on the actual robot yet.** It is
  derived from Ultralytics' JetPack 7.2 docs, an NVIDIA staff forum transcript,
  and measurements on our x86 dev box. Section 0 is how we find out.
