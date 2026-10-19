#!/usr/bin/env python3
"""
Generate arena_map.pgm analytically from arena.sdf geometry.
ROS2 map convention:
  - Origin = bottom-left corner in world coordinates
  - World (x,y) -> pixel (col, row):
      col = (x - origin_x) / res
      row = map_h - 1 - (y - origin_y) / res
"""
import math
import numpy as np

ORIGIN_X   = -10.0
ORIGIN_Y   = -10.0
RESOLUTION = 0.05
MAP_W      = 401
MAP_H      = 399
FREE, WALL = 254, 0

def world_to_pixel(wx, wy):
    col = (wx - ORIGIN_X) / RESOLUTION
    row = MAP_H - 1 - (wy - ORIGIN_Y) / RESOLUTION
    return col, row

def draw_box_wall(grid, cx, cy, length, width, yaw):
    half_l = length / 2.0
    half_w = max(width, RESOLUTION * 2) / 2.0
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    n_u = max(4, int(length / RESOLUTION) + 1)
    n_v = max(3, int(half_w * 2 / RESOLUTION) + 1)
    for i in range(n_u + 1):
        u = -half_l + length * i / n_u
        for j in range(n_v + 1):
            v = -half_w + (half_w * 2) * j / n_v
            wx = cx + u * cos_y - v * sin_y
            wy = cy + u * sin_y + v * cos_y
            col, row = world_to_pixel(wx, wy)
            ci, ri = int(round(col)), int(round(row))
            if 0 <= ci < MAP_W and 0 <= ri < MAP_H:
                grid[ri, ci] = WALL

grid = np.full((MAP_H, MAP_W), FREE, dtype=np.uint8)

# Outer walls
draw_box_wall(grid, cx=0,   cy=10,  length=20.4, width=0.5, yaw=0)
draw_box_wall(grid, cx=0,   cy=-10, length=20.4, width=0.5, yaw=0)
draw_box_wall(grid, cx=10,  cy=0,   length=20.4, width=0.5, yaw=math.pi/2)
draw_box_wall(grid, cx=-10, cy=0,   length=20.4, width=0.5, yaw=math.pi/2)

# Interior obstacle walls (iw_1 through iw_4 from arena.sdf)
draw_box_wall(grid, cx=3,  cy=4,  length=4.0, width=0.3, yaw=0.785)
draw_box_wall(grid, cx=-4, cy=5,  length=3.5, width=0.3, yaw=1.57)
draw_box_wall(grid, cx=-5, cy=-4, length=5.0, width=0.3, yaw=-0.5)
draw_box_wall(grid, cx=5,  cy=-5, length=4.5, width=0.3, yaw=0)

OUTPUT = '/home/dsc/aems_ws/maps/arena_map.pgm'
with open(OUTPUT, 'wb') as f:
    f.write(f'P5\n{MAP_W} {MAP_H}\n255\n'.encode('ascii'))
    f.write(grid.tobytes())

print(f'Written {OUTPUT}  ({MAP_W}x{MAP_H} px)')

checks = [
    ('North wall (0, 10)',    0,   10),
    ('South wall (0, -10)',   0,  -10),
    ('West wall (-10, 0)',  -10,    0),
    ('East wall (10, 0)',    10,    0),
    ('Robot spawn (0, 7)',    0,    7),
    ('Car standoff (-5.5, 6)', -5.5, 6),
    ('Person standoff (5.5, 6)', 5.5, 6),
]
print('\n=== Pixel sanity check ===')
for label, wx, wy in checks:
    col, row = world_to_pixel(wx, wy)
    ci, ri = int(round(col)), int(round(row))
    val = grid[ri, ci] if 0<=ci<MAP_W and 0<=ri<MAP_H else -1
    status = 'WALL' if val==0 else ('FREE' if val==254 else 'UNKNOWN')
    print(f'  {label:35s}  pixel=({ci:3d},{ri:3d})  [{status}]')
