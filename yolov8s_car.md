# yolov8s_car — fine-tuned YOLOv8s for the three SAR robot platforms

`yolov8s_car.pt` detects and tells apart the team's three robots. It is a YOLOv8s
detector fine-tuned from the COCO checkpoint on 27 phone photos taken on 2026-11-13.

Generated 2026-11-14 · Ultralytics 8.4.211 · torch 2.14.1+cpu

## Classes

| id | name | appearance |
|----|------|------------|
| 0 | `white_mecanum` | white aluminium plate chassis, mecanum wheels, red standoffs |
| 1 | `metal_frame` | grey metal-beam frame, blue tape marker |
| 2 | `clear_round` | transparent acrylic disc, red/green LEGO parts |

## Dataset

The source folder held **27 JPEGs and no annotations at all**, so every box in this
dataset was produced for this project.

Labelling went through three attempts:

1. **GrabCut** foreground segmentation — rejected. The black mecanum wheels sit on a
   dark floor, so colour-based segmentation assigned them to the background and most
   boxes clipped the wheels off.
2. **MobileSAM** with point prompts — rejected, worse. SAM treated individual robot
   parts as separate objects and returned only the top plate or the acrylic disc.
3. **Manual annotation** (used). Every image was overlaid with a 5% coordinate grid
   and read by eye, then a second pass re-overlaid the boxes for refinement; the first
   pass systematically under-covered the wheels.

All 27 boxes are human-verified. One object per image. Mean box area is 47% of the
frame — the robots are large, close-range targets in every shot.

| split | images | white_mecanum | metal_frame | clear_round |
|-------|--------|---------------|-------------|-------------|
| train (originals) | 18 | 6 | 5 | 7 |
| train (after augmentation) | 144 | 48 | 40 | 56 |
| val (originals, never augmented) | 9 | 3 | 3 | 3 |

Split is stratified: 3 held-out originals per class, chosen across viewpoints
(indices [3, 5, 8, 12, 14, 17, 20, 23, 26]).

## Preprocessing

1. EXIF orientation applied (phone photos carry rotation flags).
2. Long edge normalised to 1280 px, aspect ratio kept (source was 3024x4032 / 4032x3024).
3. Training copies written at long edge 640 px to match `imgsz=640`.

## Augmentation

Offline, 7 generated copies per training original (albumentations 2.0.8), bounding
boxes transformed alongside the image:

- `RandomRotate90` (p=0.75), `HorizontalFlip` (p=0.5), `VerticalFlip` (p=0.3)
- `Affine`: scale 0.5–1.1, translate ±15%, rotate ±30°
- `RandomBrightnessContrast` (±0.3), `HueSaturationValue`
- `GaussNoise` / `ISONoise` (p=0.25), motion / Gaussian blur / defocus (p=0.3)
- `ImageCompression` quality 55–95 (p=0.3), `RandomShadow` (p=0.2)

Two augmentation defects were found by inspecting rendered samples and fixed:

- albumentations 2.x `GaussNoise()` defaults buried the images in noise →
  constrained to `std_range=(0.02, 0.07)`.
- `BORDER_REFLECT_101` mirrored a **second, unlabelled robot** into the frame on
  down-scaled copies, which teaches the model to suppress real detections →
  replaced with a constant fill set to each image's own median border colour
  (approximately the floor), so padded regions carry no phantom objects.

Ultralytics' built-in online augmentation (mosaic, HSV, scale, flip) ran on top,
with mosaic disabled for the last 15 epochs.

## Training configuration

| setting | value |
|---------|-------|
| base weights | `yolov8s.pt` (COCO-pretrained) |
| epochs | 120 (budget 120, `patience=40`) |
| image size | 640 |
| batch | 8 |
| optimizer | AdamW, lr0 1.43e-3 (`optimizer=auto`) |
| device | CPU — Intel Core i7-8700T, 12 threads (no GPU on this machine) |
| close_mosaic | 15 |
| seed | 0 |
| wall-clock | 143 min (~1.2 min/epoch) |
| parameters | 11.1 M, 28.4 GFLOPs |
| weights size | 22.5 MB |

## Results

Best epoch: **95** of 120, selected on mAP50-95.

### Overall, on the 9 held-out originals

| metric | value |
|--------|-------|
| mAP50 | **0.995** |
| mAP50-95 | **0.653** |
| precision | 0.980 |
| recall | 1.000 |

### Per class

| class | images | P | R | mAP50 | mAP50-95 |
|-------|--------|---|---|-------|----------|
| `white_mecanum` | 3 | 0.984 | 1.000 | 0.995 | 0.487 |
| `metal_frame` | 3 | 0.972 | 1.000 | 0.995 | 0.774 |
| `clear_round` | 3 | 0.984 | 1.000 | 0.995 | 0.697 |
| **all** | 9 | 0.980 | 1.000 | 0.995 | 0.653 |

### Inference speed (CPU, 640px, batch 1)

preprocess 1.9 ms · inference 147.8 ms · postprocess 0.4 ms
→ roughly 6.7 FPS on this CPU. A GPU or a smaller
`imgsz` will be needed for real-time use on the robot.

### Loss at the final epoch

| | box | cls | dfl |
|---|-----|-----|-----|
| train | 0.450 | 0.332 | 1.063 |
| val | 1.580 | 0.604 | 3.004 |

## Per-image check on all 27 originals

Confidence threshold 0.25; a row counts as correct when the top-confidence box has the
right class **and** IoU ≥ 0.5 against the hand-drawn box. Rows marked `val` were never
seen in training; the rest were, so their numbers are not evidence of generalisation.

| # | split | ground truth | predicted | conf | IoU | result |
|---|-------|--------------|-----------|------|-----|--------|
| 1 | train | `white_mecanum` | `white_mecanum` | 0.84 | 0.94 | OK |
| 2 | train | `white_mecanum` | `white_mecanum` | 0.91 | 0.94 | OK |
| 3 | val | `white_mecanum` | `white_mecanum` | 0.82 | 0.81 | OK |
| 4 | train | `white_mecanum` | `white_mecanum` | 0.89 | 0.96 | OK |
| 5 | val | `white_mecanum` | `white_mecanum` | 0.87 | 0.65 | OK |
| 6 | train | `white_mecanum` | `white_mecanum` | 0.86 | 0.89 | OK |
| 7 | train | `white_mecanum` | `white_mecanum` | 0.86 | 0.97 | OK |
| 8 | val | `white_mecanum` | `white_mecanum` | 0.84 | 0.77 | OK |
| 9 | train | `white_mecanum` | `white_mecanum` | 0.77 | 0.96 | OK |
| 10 | train | `metal_frame` | `metal_frame` | 0.86 | 0.88 | OK |
| 11 | train | `metal_frame` | `metal_frame` | 0.88 | 0.98 | OK |
| 12 | val | `metal_frame` | `metal_frame` | 0.85 | 0.82 | OK |
| 13 | train | `metal_frame` | `metal_frame` | 0.85 | 0.83 | OK |
| 14 | val | `metal_frame` | `metal_frame` | 0.87 | 0.74 | OK |
| 15 | train | `metal_frame` | `metal_frame` | 0.89 | 0.94 | OK |
| 16 | train | `metal_frame` | `metal_frame` | 0.87 | 0.87 | OK |
| 17 | val | `metal_frame` | `metal_frame` | 0.85 | 0.90 | OK |
| 18 | train | `clear_round` | `clear_round` | 0.87 | 0.86 | OK |
| 19 | train | `clear_round` | `clear_round` | 0.83 | 0.93 | OK |
| 20 | val | `clear_round` | `clear_round` | 0.84 | 0.65 | OK |
| 21 | train | `clear_round` | `clear_round` | 0.85 | 0.91 | OK |
| 22 | train | `clear_round` | `clear_round` | 0.87 | 0.92 | OK |
| 23 | val | `clear_round` | `clear_round` | 0.85 | 0.80 | OK |
| 24 | train | `clear_round` | `clear_round` | 0.85 | 0.85 | OK |
| 25 | train | `clear_round` | `clear_round` | 0.88 | 0.94 | OK |
| 26 | val | `clear_round` | `clear_round` | 0.78 | 0.78 | OK |
| 27 | train | `clear_round` | `clear_round` | 0.86 | 0.90 | OK |

**27/27 correct overall, 9/9 on the held-out split.**

Rendered side by side in `runs/predictions_all27.jpg` (white = hand-drawn box,
colour = prediction; `*` marks held-out images).

## Limitations

Read the numbers above with these in mind.

- **18 independent training images, 9 validation images.** At this size a single
  image flips mAP by several points, so the validation figures are a sanity check
  that the three robots are separable, not a reliable estimate of field accuracy.
- **One robot per image, always centred and large** (mean 47% of frame). The model has
  never seen two robots in one frame, a partially occluded robot, or a robot far away.
- **Two floors, one session.** All 27 photos are from 2026-11-13 under one lighting
  setup: dark tiles for `white_mecanum` / `metal_frame`, grey carpet for `clear_round`.
  Class and background are correlated, so the model may be keying on the floor.
- **Boxes are hand-read to roughly ±2–3%**, which caps achievable mAP50-95.

To make this deployable, the highest-value additions are: photos at varied distances
(especially small, far targets), mixed lighting, multiple robots in one frame, and
each robot on both floor types to break the background correlation.

## Reproduce

```bash
python scripts/01_preprocess.py     # EXIF fix + resize to 1280
python scripts/05_write_labels.py   # hand-read boxes -> YOLO labels + review sheet
python scripts/07_build_dataset.py  # split + offline augmentation -> dataset/
python scripts/08_train.py          # fine-tune yolov8s (CPU, ~2.4 h)
python scripts/10_report.py yolov8s_car.pt   # metrics + this document
```

`scripts/02_autolabel.py`, `03_sam_label.py`, `04_gridsheet.py` and `06_refine_sheet.py`
are the labelling path described above — the two rejected automatic attempts and the
grid sheets used for manual reading. They are kept so the process is auditable.

Inference:

```python
from ultralytics import YOLO
model = YOLO('yolov8s_car.pt')
r = model.predict('photo.jpg', imgsz=640, conf=0.25)[0]
for b in r.boxes:
    print(r.names[int(b.cls)], float(b.conf), b.xyxy[0].tolist())
```
