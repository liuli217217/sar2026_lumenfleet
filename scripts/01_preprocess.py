"""Step 1: EXIF-correct orientation, resize long edge to 1280, save normalized JPEGs."""
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import os, glob, json
from PIL import Image, ImageOps

SRC = os.path.join(ROOT,'data','raw')
DST = os.path.join(ROOT,'data','prep')
LONG_EDGE = 1280

# grouping by capture order (3 visually distinct robots, shot in consecutive bursts)
GROUPS = {
    'white_mecanum': range(0, 9),    # imgs 1-9
    'metal_frame':   range(9, 17),   # imgs 10-17
    'clear_round':   range(17, 27),  # imgs 18-27
}

os.makedirs(DST, exist_ok=True)
files = sorted(glob.glob(os.path.join(SRC, '*.jpg')))
assert len(files) == 27, len(files)

meta = []
for i, f in enumerate(files):
    cls = next(k for k, r in GROUPS.items() if i in r)
    im = Image.open(f)
    orig = im.size
    im = ImageOps.exif_transpose(im)          # apply EXIF rotation
    im = im.convert('RGB')
    w, h = im.size
    s = LONG_EDGE / max(w, h)
    im = im.resize((round(w*s), round(h*s)), Image.LANCZOS)
    name = f'{cls}_{i+1:02d}.jpg'
    im.save(os.path.join(DST, name), quality=92)
    meta.append({'idx': i+1, 'src': os.path.basename(f), 'out': name,
                 'cls': cls, 'orig_size': orig, 'new_size': im.size})
    print(f'{i+1:2d} {os.path.basename(f)} {orig} -> {im.size}  [{cls}]')

json.dump(meta, open(os.path.join(ROOT,'data','prep','meta.json'), 'w'), indent=1)
from collections import Counter
print('\nper-class:', Counter(m['cls'] for m in meta))
