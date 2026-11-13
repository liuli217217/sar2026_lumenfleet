# yolo_car — YOLOv8s fine-tuned to detect the team's three robots

This is a **standalone orphan branch**. It shares no history with the ROS branches of
this repository and contains only the vision training project.

- **`yolov8s_car.pt`** — the fine-tuned weights (22.5 MB, 3 classes).
- **`yolov8s_car.md`** — full metrics, dataset description, augmentation details and
  limitations. **Read this first.**

```python
from ultralytics import YOLO
model = YOLO('yolov8s_car.pt')
r = model.predict('photo.jpg', imgsz=640, conf=0.25)[0]
for b in r.boxes:
    print(r.names[int(b.cls)], float(b.conf), b.xyxy[0].tolist())
```

## Layout

| path | contents |
|------|----------|
| `data/raw/` | the 27 original 12 MP phone photos (2026-11-13) |
| `data/prep/` | same images, EXIF-corrected, long edge 1280 |
| `labels/labels_manual.json` | hand-read boxes, the source of truth |
| `labels/yolo/` | the same 27 boxes in YOLO txt format |
| `scripts/` | preprocessing, labelling, augmentation, training, reporting |
| `dataset/data.yaml` | dataset config (images themselves are regenerated) |
| `runs/` | training curves, confusion matrix, metrics, predictions on all 27 |
| `review/` | visual proof of label quality and the two rejected auto-labelling attempts |

`dataset/images/` and `dataset/labels/` are gitignored — run
`python scripts/07_build_dataset.py` to regenerate them from `data/prep/` + `labels/`.
