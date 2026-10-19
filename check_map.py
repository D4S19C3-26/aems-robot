import yaml
import numpy as np
from PIL import Image

with open('/home/dsc/aems_ws/maps/house_map.yaml') as f:
    config = yaml.safe_load(f)

img = Image.open(config['image'])
img = np.array(img)

origin_x = config['origin'][0]
origin_y = config['origin'][1]
res = config['resolution']

# Check what is at x=-2.0, y=-0.5
px = int((-2.0 - origin_x) / res)
py = img.shape[0] - 1 - int((-0.5 - origin_y) / res)

print(f"Robot spawn px: {px}, py: {py}")
print(f"Pixel value (0=black/occ, 255=white/free, 205=gray/unk): {img[py, px]}")
