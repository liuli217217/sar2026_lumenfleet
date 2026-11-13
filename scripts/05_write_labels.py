"""Step 3: write manually-read boxes (percent x1,y1,x2,y2) as YOLO labels + review sheet."""
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import os, json
import cv2, numpy as np

PREP=os.path.join(ROOT,'data','prep')
OUT=os.path.join(ROOT,'review')
CLASSES=['white_mecanum','metal_frame','clear_round']

# idx -> (x1,y1,x2,y2) in percent of image, read visually off the grid sheets
BOXES={
 1:(15,23,83,86),  2:(16,30,96,77),  3:(21,16,97,85),  4:(26,30,81,78),  5:(20, 9,89,63),
 6:(23,24,99,75),  7:( 8,18,81,77),  8:(21,13,91,79),  9:(21,30,97,98),
10:(10,16,80,87), 11:(13, 9,89,78), 12:(12,17,97,81), 13:( 4,21,93,74), 14:(10, 9,94,67),
15:(12,11,97,66), 16:( 4, 6,90,74), 17:( 9,10,98,65),
18:(15,20,92,93), 19:(20,21,90,80), 20:(27,17,96,71), 21:(21,15,94,77), 22:(11,15,90,79),
23:(26,21,93,85), 24:(21,17,95,91), 25:( 9, 0,93,63), 26:(17,10,97,87), 27:(22, 5,96,73),
}

os.makedirs(OUT, exist_ok=True)
meta=json.load(open(os.path.join(PREP,'meta.json')))
recs=[]
for m in meta:
    x1,y1,x2,y2=[v/100 for v in BOXES[m['idx']]]
    cx,cy,w,h=(x1+x2)/2,(y1+y2)/2,x2-x1,y2-y1
    recs.append({**m,'cls_id':CLASSES.index(m['cls']),'yolo':(cx,cy,w,h),'xyxy':(x1,y1,x2,y2)})
os.makedirs(os.path.join(ROOT,'labels'), exist_ok=True)
json.dump(recs,open(os.path.join(ROOT,'labels','labels_manual.json'),'w'),indent=1)
print('boxes:',len(recs),' mean area frac:',round(np.mean([r['yolo'][2]*r['yolo'][3] for r in recs]),3))

COLORS={'white_mecanum':(80,200,255),'metal_frame':(120,255,120),'clear_round':(255,140,220)}
cell=(360,480); cols=7; rows=(len(recs)+cols-1)//cols
sheet=np.full((rows*cell[1],cols*cell[0],3),25,np.uint8)
for i,r in enumerate(recs):
    img=cv2.imread(os.path.join(PREP,r['out'])); H,W=img.shape[:2]
    x1,y1,x2,y2=r['xyxy']
    cv2.rectangle(img,(int(x1*W),int(y1*H)),(int(x2*W),int(y2*H)),COLORS[r['cls']],8)
    sc=min(cell[0]/W,cell[1]/H); img=cv2.resize(img,(round(W*sc),round(H*sc)))
    ox,oy=(i%cols)*cell[0],(i//cols)*cell[1]
    sheet[oy:oy+img.shape[0],ox:ox+img.shape[1]]=img
    cv2.putText(sheet,f"{r['idx']} {r['cls'][:11]}",(ox+6,oy+22),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,255,255),2)
cv2.imwrite(os.path.join(OUT,'labels_final.jpg'),sheet,[cv2.IMWRITE_JPEG_QUALITY,85])
print('->',os.path.join(OUT,'labels_final.jpg'))
