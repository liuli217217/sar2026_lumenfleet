"""Render a batch of prep images with a 5%/10% coordinate grid for manual box reading."""
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import sys, os, json
import cv2, numpy as np

PREP=os.path.join(ROOT,'data','prep')
OUT=os.path.join(ROOT,'review')
meta=json.load(open(os.path.join(PREP,'meta.json')))
lo,hi=int(sys.argv[1]),int(sys.argv[2])          # 1-based inclusive
sel=[m for m in meta if lo<=m['idx']<=hi]
CW,CH=620,820; cols=3
rows=(len(sel)+cols-1)//cols
sheet=np.full((rows*CH,cols*CW,3),15,np.uint8)
for i,m in enumerate(sel):
    img=cv2.imread(os.path.join(PREP,m['out'])); H,W=img.shape[:2]
    sc=min((CW-40)/W,(CH-40)/H)
    img=cv2.resize(img,(round(W*sc),round(H*sc))); h,w=img.shape[:2]
    pad=np.full((CH,CW,3),15,np.uint8)
    ox0,oy0=(CW-w)//2,(CH-h)//2+10
    pad[oy0:oy0+h, ox0:ox0+w]=img
    # grid every 10% of the IMAGE area (not the cell)
    for p in range(0,101,10):
        x=ox0+int(w*p/100); y=oy0+int(h*p/100)
        c=(0,255,255) if p%50==0 else (90,90,90)
        t=2 if p%50==0 else 1
        cv2.line(pad,(x,oy0),(x,oy0+h),c,t)
        cv2.line(pad,(ox0,y),(ox0+w,y),c,t)
        if p%20==0:
            cv2.putText(pad,str(p),(x-10,oy0-6),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,255,255),1)
            cv2.putText(pad,str(p),(max(2,ox0-32),y+5),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,255,255),1)
    cv2.putText(pad,f"#{m['idx']} {m['cls']}",(ox0,CH-12),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)
    r,c2=i//cols,i%cols
    sheet[r*CH:(r+1)*CH, c2*CW:(c2+1)*CW]=pad
p=os.path.join(OUT,f'grid_{lo}_{hi}.jpg')
cv2.imwrite(p,sheet,[cv2.IMWRITE_JPEG_QUALITY,90]); print(p, sheet.shape)
