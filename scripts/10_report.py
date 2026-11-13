"""Step 7: evaluate the fine-tuned weights and emit yolov8s_car.md (English)."""
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import os, json, csv, sys, datetime
import numpy as np, cv2, torch
from ultralytics import YOLO
torch.set_num_threads(12)


RUN=f'{ROOT}/runs'
WEIGHTS=sys.argv[1] if len(sys.argv)>1 else f'{ROOT}/yolov8s_car.pt'
CLASSES=['white_mecanum','metal_frame','clear_round']
VAL_IDX={3,5,8,12,14,17,20,23,26}
OUT_MD=f'{ROOT}/yolov8s_car.md'

rows=[{k.strip():v for k,v in r.items()} for r in csv.DictReader(open(f'{RUN}/results.csv'))]
def f(r,k):
    try: return float(r[k])
    except: return float('nan')
best=max(rows,key=lambda r: f(r,'metrics/mAP50-95(B)'))
best_ep=int(f(best,'epoch')); n_ep=len(rows)
train_min=f(rows[-1],'time')/60

m=YOLO(WEIGHTS)
res=m.val(data=f'{ROOT}/dataset/data.yaml', imgsz=640, device='cpu', plots=True,
          project=f'{ROOT}/runs', name='val_best', exist_ok=True, verbose=False)
sp=res.speed

def iou(a,b):
    ix1,iy1=max(a[0],b[0]),max(a[1],b[1]); ix2,iy2=min(a[2],b[2]),min(a[3],b[3])
    iw,ih=max(0,ix2-ix1),max(0,iy2-iy1); inter=iw*ih
    ua=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/ua if ua>0 else 0.0

recs=sorted(json.load(open(f'{ROOT}/labels/labels_manual.json')), key=lambda x:x['idx'])
COLORS={0:(80,200,255),1:(120,255,120),2:(255,140,220)}
cell=(360,480); cols=7; sheet=np.full((((len(recs)+cols-1)//cols)*cell[1],cols*cell[0],3),25,np.uint8)
per_img=[]; n_ok=n_val_ok=0
for i,r in enumerate(recs):
    p=f"{ROOT}/data/prep/{r['out']}"; img=cv2.imread(p); H,W=img.shape[:2]
    out=m.predict(p, imgsz=640, conf=0.25, device='cpu', verbose=False)[0]
    gt=list(r['xyxy']); hit=False; cname='-'; cf=0.0; ii=0.0
    if len(out.boxes):
        k=int(out.boxes.conf.argmax()); cid=int(out.boxes.cls[k]); cf=float(out.boxes.conf[k])
        bb=[float(v) for v in out.boxes.xyxyn[k]]; ii=iou(bb,gt); cname=CLASSES[cid]
        hit=(cid==r['cls_id']) and ii>=0.5
    isval=r['idx'] in VAL_IDX
    per_img.append(dict(idx=r['idx'], cls=r['cls'], split='val' if isval else 'train',
                        pred=cname, conf=cf, iou=ii, ok=hit, n=len(out.boxes)))
    n_ok+=hit; n_val_ok+=hit and isval
    cv2.rectangle(img,(int(gt[0]*W),int(gt[1]*H)),(int(gt[2]*W),int(gt[3]*H)),(255,255,255),3)
    for b in out.boxes:
        cid=int(b.cls); x1,y1,x2,y2=[float(v) for v in b.xyxy[0]]
        cv2.rectangle(img,(int(x1),int(y1)),(int(x2),int(y2)),COLORS[cid],7)
        cv2.putText(img,f'{CLASSES[cid]} {float(b.conf):.2f}',(int(x1),max(28,int(y1)-10)),
                    cv2.FONT_HERSHEY_SIMPLEX,1.0,COLORS[cid],3)
    sc=min(cell[0]/W,cell[1]/H); img=cv2.resize(img,(round(W*sc),round(H*sc)))
    ox,oy=(i%cols)*cell[0],(i//cols)*cell[1]
    sheet[oy:oy+img.shape[0],ox:ox+img.shape[1]]=img
    cv2.putText(sheet,f"{r['idx']}{'*' if isval else ''} {'OK' if hit else 'MISS'}",(ox+6,oy+22),
                cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,255) if hit else (0,0,255),2)
cv2.imwrite(f'{RUN}/predictions_all27.jpg',sheet,[cv2.IMWRITE_JPEG_QUALITY,85])

sz=os.path.getsize(WEIGHTS)/1e6
md=f"""# yolov8s_car — fine-tuned YOLOv8s for the three SAR robot platforms

`yolov8s_car.pt` detects and tells apart the team's three robots. It is a YOLOv8s
detector fine-tuned from the COCO checkpoint on 27 phone photos taken on 2026-11-13.

Generated {datetime.date.today().isoformat()} · Ultralytics 8.4.211 · torch 2.14.1+cpu

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
(indices {sorted(VAL_IDX)}).

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
| epochs | {n_ep} (budget 120, `patience=40`) |
| image size | 640 |
| batch | 8 |
| optimizer | AdamW, lr0 1.43e-3 (`optimizer=auto`) |
| device | CPU — Intel Core i7-8700T, 12 threads (no GPU on this machine) |
| close_mosaic | 15 |
| seed | 0 |
| wall-clock | {train_min:.0f} min (~{train_min/max(n_ep,1):.1f} min/epoch) |
| parameters | 11.1 M, 28.4 GFLOPs |
| weights size | {sz:.1f} MB |

## Results

Best epoch: **{best_ep}** of {n_ep}, selected on mAP50-95.

### Overall, on the 9 held-out originals

| metric | value |
|--------|-------|
| mAP50 | **{res.box.map50:.3f}** |
| mAP50-95 | **{res.box.map:.3f}** |
| precision | {res.box.mp:.3f} |
| recall | {res.box.mr:.3f} |

### Per class

| class | images | P | R | mAP50 | mAP50-95 |
|-------|--------|---|---|-------|----------|
"""
for i,c in enumerate(CLASSES):
    md+=f"| `{c}` | 3 | {res.box.p[i]:.3f} | {res.box.r[i]:.3f} | {res.box.ap50[i]:.3f} | {res.box.ap[i]:.3f} |\n"
md+=f"| **all** | 9 | {res.box.mp:.3f} | {res.box.mr:.3f} | {res.box.map50:.3f} | {res.box.map:.3f} |\n"

md+=f"""
### Inference speed (CPU, 640px, batch 1)

preprocess {sp['preprocess']:.1f} ms · inference {sp['inference']:.1f} ms · postprocess {sp['postprocess']:.1f} ms
→ roughly {1000/max(sp['inference']+sp['preprocess']+sp['postprocess'],1e-6):.1f} FPS on this CPU. A GPU or a smaller
`imgsz` will be needed for real-time use on the robot.

### Loss at the final epoch

| | box | cls | dfl |
|---|-----|-----|-----|
| train | {f(rows[-1],'train/box_loss'):.3f} | {f(rows[-1],'train/cls_loss'):.3f} | {f(rows[-1],'train/dfl_loss'):.3f} |
| val | {f(rows[-1],'val/box_loss'):.3f} | {f(rows[-1],'val/cls_loss'):.3f} | {f(rows[-1],'val/dfl_loss'):.3f} |

## Per-image check on all 27 originals

Confidence threshold 0.25; a row counts as correct when the top-confidence box has the
right class **and** IoU ≥ 0.5 against the hand-drawn box. Rows marked `val` were never
seen in training; the rest were, so their numbers are not evidence of generalisation.

| # | split | ground truth | predicted | conf | IoU | result |
|---|-------|--------------|-----------|------|-----|--------|
"""
for r in per_img:
    md+=f"| {r['idx']} | {r['split']} | `{r['cls']}` | `{r['pred']}` | {r['conf']:.2f} | {r['iou']:.2f} | {'OK' if r['ok'] else 'MISS'} |\n"
md+=f"""
**{n_ok}/27 correct overall, {n_val_ok}/9 on the held-out split.**

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
python scripts/08_train.py          # fine-tune yolov8s (CPU, ~{train_min/60:.1f} h)
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
"""
open(OUT_MD,'w').write(md)
print('wrote', OUT_MD)
print(f'mAP50={res.box.map50:.4f} mAP50-95={res.box.map:.4f} P={res.box.mp:.4f} R={res.box.mr:.4f}')
print(f'correct {n_ok}/27 overall, {n_val_ok}/9 held-out')
