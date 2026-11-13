"""Step 4: stratified train/val split + offline augmentation (albumentations).

val = 3 held-out originals per class (never augmented).
train = 18 originals + N augmented copies each.
"""
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import os, json, shutil, random
import cv2, numpy as np
import albumentations as A

random.seed(0); np.random.seed(0)
PREP=f'{ROOT}/data/prep'; DS=f'{ROOT}/dataset'
CLASSES=['white_mecanum','metal_frame','clear_round']
VAL_IDX={3,5,8, 12,14,17, 20,23,26}     # 3 per class, varied viewpoints
N_AUG=7
LONG=640

for sp in ('train','val'):
    for k in ('images','labels'):
        os.makedirs(f'{DS}/{k}/{sp}', exist_ok=True)
        for f in os.listdir(f'{DS}/{k}/{sp}'): os.remove(f'{DS}/{k}/{sp}/{f}')

recs=json.load(open(f'{ROOT}/labels/labels_manual.json'))

def resize_long(img, box, long=LONG):
    H,W=img.shape[:2]; s=long/max(H,W)
    return cv2.resize(img,(round(W*s),round(H*s)),interpolation=cv2.INTER_AREA), box

def write(split, name, img, cls_id, yolo):
    cv2.imwrite(f'{DS}/images/{split}/{name}.jpg', img, [cv2.IMWRITE_JPEG_QUALITY,92])
    cx,cy,w,h=yolo
    open(f'{DS}/labels/{split}/{name}.txt','w').write(f'{cls_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n')

def floor_color(img):
    """Median colour of a thin border ring ~= the floor, used as constant fill
    so geometric augmentation never mirrors a second, unlabelled robot into frame."""
    H,W=img.shape[:2]; m=max(2,int(0.04*min(H,W)))
    ring=np.concatenate([img[:m].reshape(-1,3), img[-m:].reshape(-1,3),
                         img[:,:m].reshape(-1,3), img[:,-m:].reshape(-1,3)])
    return tuple(int(v) for v in np.median(ring,axis=0))

def make_aug(fill):
    return A.Compose([
        A.RandomRotate90(p=0.75),
        A.HorizontalFlip(p=0.5),
        A.VerticalFlip(p=0.3),
        A.Affine(scale=(0.5,1.1), translate_percent=(-0.15,0.15), rotate=(-30,30),
                 border_mode=cv2.BORDER_CONSTANT, fill=fill, p=0.95),
        A.RandomBrightnessContrast(brightness_limit=0.3, contrast_limit=0.3, p=0.7),
        A.HueSaturationValue(hue_shift_limit=8, sat_shift_limit=25, val_shift_limit=20, p=0.5),
        A.OneOf([A.GaussNoise(std_range=(0.02,0.07)),
                 A.ISONoise(color_shift=(0.01,0.03), intensity=(0.1,0.3))], p=0.25),
        A.OneOf([A.MotionBlur(blur_limit=5), A.GaussianBlur(blur_limit=5), A.Defocus(radius=(1,3))], p=0.3),
        A.ImageCompression(quality_range=(55,95), p=0.3),
        A.RandomShadow(p=0.2),
    ], bbox_params=A.BboxParams(format='yolo', label_fields=['cls'], min_visibility=0.45, min_area=400))

counts={'train':0,'val':0}; per_cls={c:{'train':0,'val':0} for c in CLASSES}
for r in recs:
    img=cv2.imread(f"{PREP}/{r['out']}")
    cid=r['cls_id']; yolo=tuple(r['yolo'])
    split='val' if r['idx'] in VAL_IDX else 'train'
    small,_=resize_long(img, yolo)
    base=f"{r['cls']}_{r['idx']:02d}"
    write(split, base, small, cid, yolo)
    counts[split]+=1; per_cls[r['cls']][split]+=1
    if split=='val': continue
    aug = make_aug(floor_color(small))
    made=0; tries=0
    while made<N_AUG and tries<N_AUG*6:
        tries+=1
        try:
            out=aug(image=small, bboxes=[list(yolo)], cls=[cid])
        except Exception as e:
            continue
        if not out['bboxes']: continue
        bb=list(out['bboxes'][0])[:4]
        if bb[2]<0.03 or bb[3]<0.03: continue
        write('train', f'{base}_a{made}', out['image'], cid, bb)
        made+=1; counts['train']+=1; per_cls[r['cls']]['train']+=1

yaml=f"""# SAR 3-robot detection dataset (auto-generated)
# 'path' is intentionally omitted: Ultralytics then resolves train/val relative to
# this file's own directory, so the dataset works from any checkout and any cwd.
train: images/train
val: images/val
nc: 3
names:
  0: white_mecanum
  1: metal_frame
  2: clear_round
"""
open(f'{DS}/data.yaml','w').write(yaml)
os.makedirs(f'{ROOT}/labels/yolo', exist_ok=True)
for r in recs:
    cx,cy,w,h=r['yolo']
    open(f"{ROOT}/labels/yolo/{r['out'].replace('.jpg','.txt')}",'w').write(
        f"{r['cls_id']} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")
print('train imgs:',counts['train'],' val imgs:',counts['val'])
for c in CLASSES: print(f'  {c:<15} train={per_cls[c]["train"]:<4} val={per_cls[c]["val"]}')
print('\ndata.yaml ->', f'{DS}/data.yaml')
