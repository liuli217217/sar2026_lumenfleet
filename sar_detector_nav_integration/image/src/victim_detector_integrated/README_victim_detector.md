# SAR Detector — ROS 2 node (Team Lumen Fleet)

Owner: Fangyikun Jian. Status: **interface complete, model pending**.

A ROS 2 node that subscribes to a camera image topic, runs YOLO, and publishes
detections. Everything is a ROS parameter — swapping camera, weights, classes or
thresholds needs no code change.

---

## 1. Quick start

```bash
# real robot, RealSense D435i - uses the bundled 3-class model by default
ros2 launch victim_detector sar_detector.launch.py

# simulation (Gazebo D455) - QoS default is fine, only the topic changes
ros2 launch victim_detector sar_detector.launch.py \
    image_topic:=/d455/image

# recommended for SAR: per-class thresholds + temporal aggregation
ros2 launch victim_detector sar_detector.launch.py \
    class_conf:="mannequin:0.10,cone:0.20,first_aid:0.20" \
    track_enable:=true

# offline deployment, weights baked in elsewhere
ros2 launch victim_detector sar_detector.launch.py \
    weights:=/opt/models/sar_3class.pt
```

The fine-tuned 3-class model ships with the package and is the **default** —
the first command above needs no arguments.

> **To pass an empty string on the command line, write `keep_classes:="''"`.**
> Plain `keep_classes:=""` does not work: the shell strips the quotes, launch
> receives `keep_classes:=` and rejects it as a malformed argument. An earlier
> version of this file documented that broken form.

---

## 2. Interface contract

### Subscribes

| topic | type | notes |
|---|---|---|
| `image_topic` (param) | `sensor_msgs/Image` | default `/camera/camera/color/image_raw`, bgr8 |

### Publishes

| topic | type | notes |
|---|---|---|
| `~/detections` | `vision_msgs/Detection2DArray` | **always published, even when empty** |
| `~/image_annotated` | `sensor_msgs/Image` | debug only, disable in production |
| `~/detections_confirmed` | `vision_msgs/Detection2DArray` | only with `track_enable`; temporally confirmed targets, `Detection2D.id` carries a stable track id |
| `~/status` | `std_msgs/String` | every 5 s: frames, skipped, inference ms, device, confirmed |

With the default node name the full names are `/sar_detector/detections` etc.

### Detection2DArray layout

- `header` is **copied verbatim from the input image** — same stamp, same
  `frame_id`. Downstream can therefore look up TF at exactly the capture time.
- `detections[i].bbox.center.position` — box centre in **pixels**
- `detections[i].bbox.size_x / size_y` — box size in **pixels**
- `detections[i].results[0].hypothesis.class_id` — **string class name**
  (`person`, `mannequin`, `cone`, `first_aid`), not a number
- `detections[i].results[0].hypothesis.score` — confidence in [0, 1]
- `results[0].pose` is left at identity. 2D detector only; 3D back-projection is
  the consumer's job.

---

## 3. Parameters

| name | default | meaning |
|---|---|---|
| `image_topic` | `/camera/camera/color/image_raw` | input topic |
| `best_effort_qos` | `true` | leave at `true`; see the QoS section |
| `queue_depth` | `1` | 1 = always process the newest frame |
| `weights` | `yolov8s.pt` | path to .pt — use an absolute in-container path when offline |
| `device` | `auto` | `auto` / `cpu` / `cuda:0` |
| `imgsz` | `640` | inference resolution |
| `conf` | `0.25` | confidence threshold |
| `iou` | `0.45` | NMS IoU |
| `half` | `false` | FP16 — recommended on Jetson |
| `keep_classes` | `0` | comma-separated class ids to keep; empty = all |
| `class_names` | `` | comma-separated override of model class names |
| `publish_annotated` | `true` | turn off in production to save bandwidth |
| `max_rate_hz` | `0.0` | throttle inference; 5–10 suggested on a 50 W Orin |
| `log_every_n` | `30` | log one line every N frames |
| `class_conf` | `` | per-class thresholds, e.g. `mannequin:0.10,cone:0.20`; empty = all use `conf` |
| `track_enable` | `false` | temporal aggregation: confirm a target only after repeated hits |
| `track_iou` | `0.3` | IoU for associating a detection with an existing track |
| `track_min_hits` | `3` | hits required before a track is reported |
| `track_max_misses` | `5` | frames a track survives without a detection |
| `track_bridge` | `false` | keep reporting a confirmed track through a gap. **Measured: recovers no extra victim and adds 17% more false positives. Leave it off.** |

### Why per-class thresholds

One global `conf` is wrong for this model. Measured over 5 folds: `cone` is
best at 0.20 (recall 0.92, precision 0.86), while `mannequin` needs 0.10 to
reach any useful recall. A single threshold can only compromise between them.

### Why temporal aggregation

Per-frame mannequin recall is ~0.57, but the robot is moving: it sees a victim
across many frames from changing viewpoints, and those failures are not fully
correlated.

Measured over 12 victims, each as a synthetic 24-frame approach sequence:

| | recall | false positives |
|---|---|---|
| raw, per frame | 0.801 | 150 |
| at least once in the pass *(ceiling)* | 0.917 | — |
| `track_bridge:=false` | **0.917** | **96 (-36%)** |
| `track_bridge:=true` | 0.917 | 176 (+17%) |

**Leave `track_bridge` off.** Confirmation alone already reaches the ceiling
of what is recoverable, and cuts false positives by a third. Bridging gaps
recovered no additional victim and only added false positives. An earlier
version of this file recommended `track_bridge:=true` on the strength of
an argument rather than a measurement.

Caveat: the sequences are synthetic pans over held-out stills, not real
robot footage. Per-frame recall in that regime is 0.801 versus 0.569 on the
real cross-validation, so the regime is easier than the arena and the
absolute numbers do not transfer. The *ordering* of the two modes is the
part worth keeping.

`~/detections` is unchanged either way — raw, per frame. Nothing downstream
breaks if you ignore the new topic.

---

## 4. Two things that will bite you

**QoS.** `realsense-ros` publishes images with **BEST_EFFORT** reliability. ROS 2
compatibility is one-directional: a subscriber may not demand *more* reliability
than the publisher offers. Measured on this stack:

| publisher | subscriber | result |
|---|---|---|
| BEST_EFFORT (realsense) | RELIABLE | **zero frames, no error at all** |
| BEST_EFFORT (realsense) | BEST_EFFORT | works |
| RELIABLE (Gazebo bridge) | BEST_EFFORT | works |

So **leave `best_effort_qos` at `true`** - it is safe against both publishers, in
simulation and on the robot. Set it to `false` only if you have a RELIABLE
publisher and genuinely need delivery guarantees. If the node logs that it
subscribed but the frame counter never moves, check this first.

**Offline.** On competition day the robot has no internet. Ultralytics will try
to download weights (and fonts) on first use and will fail. Bake the `.pt` into
the image and pass an absolute path. Verify with the network disconnected before
you rely on it.

---

## 5. Model status — read this

The node is finished; the **model is not**. Measured on 38 real arena photos
from the organisers' Roboflow set (`SARCompetition/Hazel_Project1`):

| model | images with a detection | mean top conf |
|---|---|---|
| yolov8s COCO `person` | 17/38 | 0.64 |
| yolov8m COCO `person` | 18/38 | 0.64 |
| our Phase 1 `v5` weights | 22/38 | 0.45 |

v5's higher count is **false positives** — it boxes first-aid kits and wall
signs while missing the hi-vis mannequins entirely. COCO `person` lands roughly
on target but with sloppy boxes and misses detached prosthetic limbs.

Neither is deployable. `yolov8s` + COCO `person` is the current default because
it fails in the more predictable direction. A 3-class fine-tune
(`mannequin` / `cone` / `first_aid`) on the organisers' dataset is in progress;
when it lands, only `weights` and `keep_classes` change.

**Open question for the team:** the brief said "victim detector", but the
organisers' dataset has three classes and the Phase 2 tasks list hazard and
special-hazard reporting. Should this node detect cones and first-aid kits too?

---

## 6. Performance

Measured on an RTX 3070 Ti in simulation: **50–53 ms/frame** at `imgsz=640`
with `yolov8s` (~19 fps ceiling), 0 frames skipped against a ~2 Hz camera.

The Jetson AGX Orin is **fixed at 50 W**, so expect materially slower. Levers,
in order of preference: set `half:=true`, cap `max_rate_hz`, set
`publish_annotated:=false`, then export the model to TensorRT (engines are
hardware- and version-specific and **must be built on the robot**).

`~/status` reports live inference time — check it on the robot before tuning.

---

## 7. Dependencies

`rclpy`, `sensor_msgs`, `vision_msgs`, `cv_bridge`, `ultralytics`, `torch`,
`opencv`, `numpy`.

**NumPy must be < 2.** ROS 2 Jazzy's `cv_bridge` is compiled against NumPy 1.x;
installing `ultralytics` pulls NumPy 2.x and breaks `cv_bridge` on import. Also
let `cv_bridge` use the system `python3-opencv` — the pip `opencv-python` wheel
shadows it and demands NumPy ≥ 2, which re-creates the same conflict.

```
numpy<2
# do NOT pip install opencv-python; use the system python3-opencv
```

On the Jetson, install torch **from the CUDA-matched PyTorch index**, and do it
*after* `pip install ultralytics` -- ultralytics pulls generic x86-flavoured
torch/torchvision wheels that do not work on JetPack 7, so they must be
overwritten:

```bash
nvcc --version                      # read the CUDA release, e.g. 13.2
pip install ultralytics
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu132
```

The index suffix must match the CUDA toolkit on the device: CUDA 13.2 -> `cu132`,
13.0 -> `cu130`. A bare `pip install torch` with no index gives a wheel built for
the wrong compute capability and dies at the first kernel launch with
`no kernel image is available for execution on the device`. Expect
`torch 2.12.x+cu132` and Python 3.12 (`cp312`); verify with
`torch.cuda.is_available()` before anything else.

A harmless `Found GPU ... compute capability` warning may appear -- Orin is
sm_87 and the wheel advertises newer architectures. It is cosmetic.

Run `scripts/jetson_check.sh` on the robot first. It is read-only and needs no
sudo, and it prints exactly which of the above apply.
