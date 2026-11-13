"""Step 2b: re-generate boxes with MobileSAM, prompted by the GrabCut centroid.
SAM uses learned features, so it keeps the dark mecanum wheels that colour-based
GrabCut dropped against the dark floor.
"""
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import os, json, sys
import numpy as np, cv2
from ultralytics import SAM

PREP = os.path.join(ROOT,'data','prep')
OUT  = os.path.join(ROOT,'review')
boxes = json.load(open(os.path.join(OUT, 'boxes.json')))
model = SAM('mobile_sam.pt')

res = []
for r in boxes:
    p = os.path.join(PREP, r['out'])
    img = cv2.imread(p); H, W = img.shape[:2]
    cx, cy, bw, bh = r['box']
    # prompt: GrabCut centroid + 4 points inside the coarse box (robust to a bad edge)
    pts = [[cx*W, cy*H],
           [(cx-bw*0.22)*W, cy*H], [(cx+bw*0.22)*W, cy*H],
           [cx*W, (cy-bh*0.22)*H], [cx*W, (cy+bh*0.22)*H]]
    out = model(p, points=[pts], labels=[[1]*len(pts)], verbose=False)
    m = out[0].masks
    if m is None:
        res.append({**r, 'sam_box': None}); print(r['out'], 'NO MASK'); continue
    mask = m.data.cpu().numpy()
    mask = (mask.sum(0) > 0).astype(np.uint8)
    if mask.shape != (H, W):
        mask = cv2.resize(mask, (W, H), interpolation=cv2.INTER_NEAREST)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    mask = cv2.morphologyEx(mask*255, cv2.MORPH_CLOSE, k, 2)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        res.append({**r, 'sam_box': None}); print(r['out'], 'EMPTY'); continue
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, bwp, bhp, area = stats[i]
    nb = ((x+bwp/2)/W, (y+bhp/2)/H, bwp/W, bhp/H)
    res.append({**r, 'sam_box': tuple(round(float(v), 4) for v in nb), 'sam_frac': round(area/(W*H), 3)})
    print(f"{r['out']:<22} sam={tuple(round(v,3) for v in nb)} frac={area/(W*H):.3f}")

json.dump(res, open(os.path.join(OUT, 'boxes_sam.json'), 'w'), indent=1)

COLORS = {'white_mecanum': (80,200,255), 'metal_frame': (120,255,120), 'clear_round': (255,140,220)}
cell=(360,480); cols=7; rows=(len(res)+cols-1)//cols
sheet=np.full((rows*cell[1], cols*cell[0],3),25,np.uint8)
for i,r in enumerate(res):
    img=cv2.imread(os.path.join(PREP,r['out'])); H,W=img.shape[:2]
    b = r.get('sam_box') or r['box']
    cxx,cyy,bww,bhh=b
    cv2.rectangle(img,(int((cxx-bww/2)*W),int((cyy-bhh/2)*H)),(int((cxx+bww/2)*W),int((cyy+bhh/2)*H)),COLORS[r['cls']],8)
    sc=min(cell[0]/W,cell[1]/H); img=cv2.resize(img,(round(W*sc),round(H*sc)))
    ox,oy=(i%cols)*cell[0],(i//cols)*cell[1]
    sheet[oy:oy+img.shape[0],ox:ox+img.shape[1]]=img
    cv2.putText(sheet,f"{r['idx']} {r['cls'][:11]}",(ox+6,oy+22),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,255,255),2)
cv2.imwrite(os.path.join(OUT,'review_sam.jpg'),sheet,[cv2.IMWRITE_JPEG_QUALITY,85])
print('\nsheet ->', os.path.join(OUT,'review_sam.jpg'))
