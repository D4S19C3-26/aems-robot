import yaml
import numpy as np
from PIL import Image

with open('/home/dsc/aems_ws/maps/house_map.yaml') as f:
    config = yaml.safe_load(f)

img = Image.open(config['image'])
img = np.array(img)

free_pixels = np.where(img == 254)
if len(free_pixels[0]) > 0:
    print(f"Free space bounding box: X min={free_pixels[1].min()}, max={free_pixels[1].max()} Y min={free_pixels[0].min()}, max={free_pixels[0].max()}")
else:
    print("No free space (254)? Let's check 255")
    free_pixels = np.where(img == 255)
    print(f"Free space bounding box: X min={free_pixels[1].min()}, max={free_pixels[1].max()} Y min={free_pixels[0].min()}, max={free_pixels[0].max()}")
