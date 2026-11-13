"""Step 5: fine-tune YOLOv8s on the 3-robot dataset (CPU)."""
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
from ultralytics import YOLO
import torch, time
torch.set_num_threads(12)

m = YOLO('yolov8s.pt')
t0 = time.time()
m.train(
    data=os.path.join(ROOT,'dataset','data.yaml'),
    epochs=120, imgsz=640, batch=8, device='cpu', workers=8, cache=True,
    project=os.path.join(ROOT,'runs'), name='train', exist_ok=True,
    patience=40, close_mosaic=15, seed=0, plots=True, val=True, verbose=True,
)
print('TRAIN_DONE_SEC', round(time.time()-t0, 1))
