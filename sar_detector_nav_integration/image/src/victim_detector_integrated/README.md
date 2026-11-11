# sar-victim-detector

A ROS 2 node that detects search-and-rescue targets — **mannequins (victims),
traffic cones, and first-aid kits** — from a camera stream and publishes
`vision_msgs/Detection2DArray`.

Built for Team Lumen Fleet (University of Adelaide), IEEE SMCS SAR competition
Phase 2. Runs on a Unitree Go2 with a Jetson AGX Orin and a RealSense D435i.

> **Status: the interface is production-ready. The model is not.**
> Cones are detected reliably. Mannequins — the actual victims — are not.
> See [Performance](#performance) for measured numbers, and read them before
> relying on this.

---

## What it does

Subscribes to a camera topic, runs YOLOv8, publishes detections. Every behaviour
is a ROS parameter, so swapping camera, weights, classes or thresholds needs no
code change.

```bash
# with the fine-tuned 3-class model
ros2 launch victim_detector sar_detector.launch.py \
    weights:=models/sar_3class.pt keep_classes:=""

# simulation
ros2 launch victim_detector sar_detector.launch.py image_topic:=/d455/image
```

| topic | type | notes |
|---|---|---|
| *(subscribes)* `image_topic` | `sensor_msgs/Image` | default `/camera/camera/color/image_raw` |
| `~/detections` | `vision_msgs/Detection2DArray` | **published every frame, even when empty** |
| `~/image_annotated` | `sensor_msgs/Image` | debug only |
| `~/status` | `std_msgs/String` | frames, skipped, inference ms, device |

`class_id` is the **string class name** (`mannequin`, `cone`, `first_aid`), not a
number. The header is copied verbatim from the input image, so consumers can look
up TF at exactly the capture time.

Full interface contract: [`README_victim_detector.md`](README_victim_detector.md).
Jetson/arm64 deployment: [`DEPLOY_arm64.md`](DEPLOY_arm64.md).

---

## Performance

Fine-tuned `yolov8s` on the organisers' 101-image dataset. Measured by **5-fold
cross-validation** — every image takes a turn as validation data — because a
single 20-image split on a dataset this small is not a measurement.

| class | instances | recall | mAP50 | spread across folds |
|---|---|---|---|---|
| `cone` | 106 | 0.870 ± 0.141 | **0.918 ± 0.102** | 0.74 – 0.995 |
| `first_aid` | 38 | 0.738 ± 0.108 | 0.810 ± 0.128 | 0.63 – 0.93 |
| `mannequin` | 70 | 0.569 ± 0.232 | **0.614 ± 0.237** | **0.26 – 0.91** |

Overall mAP50 **0.781 ± 0.110**.

**Read the mannequin row.** Recall 0.569 means it misses roughly 43% of victims
per frame on average, and in the worst fold it missed 80%. The ±0.237 spread is
the signature of too little data: the whole dataset contains 70 mannequin
instances.

A single 80/20 split reported mannequin mAP50 = 0.959 for the same model. That
was a lucky fold. Cross-validation is the honest number.

### Confidence threshold

Lowering `conf` buys less than you would hope — recall saturates around 0.70
while false positives triple:

| conf | mannequin recall | precision | false positives per fold |
|---|---|---|---|
| 0.05 | 0.698 | 0.229 | 33 |
| 0.10 | 0.652 | 0.292 | 23 |
| 0.25 *(default)* | 0.608 | 0.513 | 8 |
| 0.50 | 0.482 | 0.728 | 2.6 |

Cones are usable at `conf=0.20`: recall 0.923, precision 0.859.

### What did not work

- **`imgsz=960`** — worse across the board than 640, at 2.25× the compute.
- **Lowering `conf`** — see above; the recall ceiling is the model, not the threshold.

### What did work

- **Background negatives.** The fine-tuned model reported `mannequin 0.41` on a
  pure black frame; stock COCO never does. Fine-tuning on 101 images cost it its
  background calibration. Adding 84 mined background crops (regions of training
  images with no annotated object, plus degenerate frames) fixed it completely —
  black frames are now clean at every threshold.

---

## Known limitations

1. **Not a reliable victim detector yet.** Per-frame mannequin recall ~0.57. The
   binding constraint is data, not hyperparameters.
2. **Single-frame metrics.** These numbers are per frame. A robot moving past a
   victim sees it in many frames from changing viewpoints, so effective recall
   over a pass is higher than 0.57 — but that has not been measured here.
3. **Fixed-arena model.** Trained on 101 images of one indoor arena. Do not
   expect it to transfer.
4. **TensorRT engines must be built on the robot** — they are device- and
   version-specific.

---

## Repository layout

```
victim_detector/detector_node.py   the node
launch/sar_detector.launch.py      every parameter as a launch argument
models/sar_3class.pt               fine-tuned weights (22 MB)
scripts/jetson_check.sh            read-only on-robot diagnostic, no sudo
worlds/sar_test.sdf                Gazebo test world with 5 ground-truth victims
DEPLOY_arm64.md                    Jetson / Docker / offline deployment
README_victim_detector.md          full interface contract
```

The training and evaluation scripts are in [`tools/`](tools/).

---

## Attribution

The model was fine-tuned on **Hazel_Project1 v6** from the Roboflow Universe
workspace `sarcompetition`, licensed **CC BY 4.0**:
<https://universe.roboflow.com/sarcompetition/hazel_project1-rfxjp>

The dataset itself is not redistributed here. Base weights are `yolov8s` from
Ultralytics (AGPL-3.0) — note that Ultralytics' licence propagates to derived
weights and to code that imports it.
