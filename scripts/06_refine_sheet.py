"""Overlay current boxes on a 5% grid, 3 per sheet, for visual refinement."""
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import sys, os, json
import cv2, numpy as np
PREP=os.path.join(ROOT,'data','prep'); OUT=os.path.join(ROOT,'review')
recs={r['idx']:r for r in json.load(open(os.path.join(OUT,'labels_manual.json')))}
lo,hi=int(sys.argv[1]),int(sys.argv[2])
sel=[recs[i] for i in range(lo,hi+1) if i in recs]
CW,CH=660,880; cols=3
sheet=np.full((CH,cols*CW,3),15,np.uint8)
for i,r in enumerate(sel):
    img=cv2.imread(os.path.join(PREP,r['out'])); H,W=img.shape[:2]
    sc=min((CW-50)/W,(CH-50)/H); img=cv2.resize(img,(round(W*sc),round(H*sc))); h,w=img.shape[:2]
    pad=np.full((CH,CW,3),15,np.uint8); ox0,oy0=(CW-w)//2,(CH-h)//2+12
    pad[oy0:oy0+h,ox0:ox0+w]=img
    x1,y1,x2,y2=r['xyxy']
    cv2.rectangle(pad,(ox0+int(x1*w),oy0+int(y1*h)),(ox0+int(x2*w),oy0+int(y2*h)),(0,0,255),3)
    for p in range(0,101,5):
        x=ox0+int(w*p/100); y=oy0+int(h*p/100)
        c=(0,255,255) if p%50==0 else ((140,140,140) if p%10==0 else (70,70,70))
        cv2.line(pad,(x,oy0),(x,oy0+h),c,1); cv2.line(pad,(ox0,y),(ox0+w,y),c,1)
        if p%10==0:
            cv2.putText(pad,str(p),(x-9,oy0-4),cv2.FONT_HERSHEY_SIMPLEX,0.38,(0,255,255),1)
            cv2.putText(pad,str(p),(max(1,ox0-30),y+4),cv2.FONT_HERSHEY_SIMPLEX,0.38,(0,255,255),1)
    cv2.putText(pad,f"#{r['idx']} cur={tuple(int(v*100) for v in r['xyxy'])}",(ox0,CH-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
    sheet[:,i*CW:(i+1)*CW]=pad
p=os.path.join(OUT,f'refine_{lo}_{hi}.jpg'); cv2.imwrite(p,sheet,[cv2.IMWRITE_JPEG_QUALITY,92]); print(p)
