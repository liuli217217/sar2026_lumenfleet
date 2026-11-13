"""Step 2: auto-generate bounding boxes with GrabCut foreground segmentation.
The robots sit near the image centre on a fairly uniform floor, so GrabCut with a
centre-rectangle prior separates them well. Output = YOLO-format boxes + a review sheet.
"""
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import os, glob, json
import cv2, numpy as np

PREP = os.path.join(ROOT,'data','prep')
OUT  = os.path.join(ROOT,'review')
os.makedirs(OUT, exist_ok=True)
WORK = 640          # grabcut working resolution
MARGIN = 0.04       # init rect inset
PAD = 0.012         # final box padding

def auto_box(path):
    img = cv2.imread(path)
    H, W = img.shape[:2]
    s = WORK / max(H, W)
    small = cv2.resize(img, (round(W*s), round(H*s)), interpolation=cv2.INTER_AREA)
    h, w = small.shape[:2]
    # mild denoise helps on carpet/tile texture
    sm = cv2.bilateralFilter(small, 7, 50, 50)
    mask = np.zeros((h, w), np.uint8)
    rect = (int(w*MARGIN), int(h*MARGIN), int(w*(1-2*MARGIN)), int(h*(1-2*MARGIN)))
    bgd, fgd = np.zeros((1, 65), np.float64), np.zeros((1, 65), np.float64)
    cv2.grabCut(sm, mask, rect, bgd, fgd, 6, cv2.GC_INIT_WITH_RECT)
    fg = np.where((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)
    # clean up: close holes, drop specks, keep largest component
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, k, iterations=2)
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, k, iterations=1)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(fg, 8)
    if n <= 1:
        return None, fg, 0.0
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, bw, bh, area = stats[i]
    frac = area / (w*h)
    # back to full-res normalised xywh
    cx = (x + bw/2) / w; cy = (y + bh/2) / h
    nw = min(1.0, bw/w + 2*PAD); nh = min(1.0, bh/h + 2*PAD)
    cx = min(max(cx, nw/2), 1-nw/2); cy = min(max(cy, nh/2), 1-nh/2)
    return (cx, cy, nw, nh), (lab == i).astype(np.uint8)*255, frac

meta = json.load(open(os.path.join(PREP, 'meta.json')))
res = []
for m in meta:
    p = os.path.join(PREP, m['out'])
    box, fg, frac = auto_box(p)
    res.append({**m, 'box': box, 'fg_frac': round(frac, 3)})
    print(f"{m['out']:<22} box={None if box is None else tuple(round(v,3) for v in box)} fg={frac:.3f}")

json.dump(res, open(os.path.join(OUT, 'boxes.json'), 'w'), indent=1)

# review sheet with boxes drawn
COLORS = {'white_mecanum': (80,200,255), 'metal_frame': (120,255,120), 'clear_round': (255,140,220)}
cell = (360, 480); cols = 7
rows = (len(res)+cols-1)//cols
sheet = np.full((rows*cell[1], cols*cell[0], 3), 25, np.uint8)
for i, r in enumerate(res):
    img = cv2.imread(os.path.join(PREP, r['out'])); H, W = img.shape[:2]
    if r['box']:
        cx, cy, bw, bh = r['box']
        x1, y1 = int((cx-bw/2)*W), int((cy-bh/2)*H)
        x2, y2 = int((cx+bw/2)*W), int((cy+bh/2)*H)
        cv2.rectangle(img, (x1,y1), (x2,y2), COLORS[r['cls']], 8)
    sc = min(cell[0]/W, cell[1]/H)
    img = cv2.resize(img, (round(W*sc), round(H*sc)))
    ox, oy = (i%cols)*cell[0], (i//cols)*cell[1]
    sheet[oy:oy+img.shape[0], ox:ox+img.shape[1]] = img
    cv2.putText(sheet, f"{r['idx']} {r['cls'][:11]}", (ox+6, oy+22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,255), 2)
cv2.imwrite(os.path.join(OUT, 'review_sheet.jpg'), sheet, [cv2.IMWRITE_JPEG_QUALITY, 85])
print('\nreview sheet ->', os.path.join(OUT, 'review_sheet.jpg'))
