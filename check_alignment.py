#!/usr/bin/env python3
import yaml
import cv2

map_yaml = "/home/dsc/aems_ws/maps/arena_map.yaml"
map_pgm = "/home/dsc/aems_ws/maps/arena_map.pgm"

try:
    with open(map_yaml, 'r') as f:
        data = yaml.safe_load(f)
        
    resolution = data['resolution']
    origin_x, origin_y = data['origin'][0], data['origin'][1]
    
    # Robot spawn pose is (0, 0)
    spawn_x, spawn_y = 0.0, 0.0
    
    # Convert physical coordinates to pixels
    pixel_x = int((spawn_x - origin_x) / resolution)
    
    # Y is inverted in PGM images
    img = cv2.imread(map_pgm, cv2.IMREAD_GRAYSCALE)
    height = img.shape[0]
    pixel_y = height - int((spawn_y - origin_y) / resolution)
    
    val = img[pixel_y, pixel_x]
    
    status = "FREE SPACE" if val > 250 else ("WALL/OBSTACLE" if val < 10 else "UNKNOWN")
    
    print(f"--- ALIGNMENT CHECK ---")
    print(f"Spawn Pose: ({spawn_x}, {spawn_y})")
    print(f"Map Origin: ({origin_x}, {origin_y}), Res: {resolution}")
    print(f"Pixel Coord: ({pixel_x}, {pixel_y})")
    print(f"Pixel Value: {val} -> {status}")
    print(f"-----------------------")
except Exception as e:
    print(f"Error checking alignment: {e}")
