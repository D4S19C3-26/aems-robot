import xml.etree.ElementTree as ET
import sys

def parse_sdf(filepath):
    tree = ET.parse(filepath)
    root = tree.getroot()
    
    poses = []
    
    # In this SDF, models are under <sdf><model><model>...
    for model in root.findall('.//model'):
        name = model.get('name')
        pose_elem = model.find('pose')
        if pose_elem is not None:
            pose_text = pose_elem.text.strip()
            parts = pose_text.split()
            if len(parts) >= 2:
                x, y = float(parts[0]), float(parts[1])
                poses.append((name, x, y))
                
    if not poses:
        print("No models with poses found.")
        return
        
    xs = [p[1] for p in poses]
    ys = [p[2] for p in poses]
    
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    
    print(f"Bounding Box: X: [{min_x:.2f}, {max_x:.2f}], Y: [{min_y:.2f}, {max_y:.2f}]")
    
    # Generate a set of waypoints covering this area
    # Let's say a 3x3 grid or similar
    waypoints = []
    num_x = 4
    num_y = 3
    
    # shrink the bounding box slightly to avoid hitting walls exactly on the edge
    pad = 1.0
    safe_min_x = min_x + pad
    safe_max_x = max_x - pad
    safe_min_y = min_y + pad
    safe_max_y = max_y - pad
    
    # If the space is too small, fallback
    if safe_max_x < safe_min_x: safe_max_x = safe_min_x
    if safe_max_y < safe_min_y: safe_max_y = safe_min_y
    
    import numpy as np
    x_steps = np.linspace(safe_min_x, safe_max_x, num_x)
    y_steps = np.linspace(safe_min_y, safe_max_y, num_y)
    
    # Create snake pattern to minimize travel distance
    for i, x in enumerate(x_steps):
        ys_for_this_x = y_steps if i % 2 == 0 else reversed(y_steps)
        for y in ys_for_this_x:
            waypoints.append((x, y))
            
    print("\nProposed Waypoints (X, Y):")
    for idx, (wx, wy) in enumerate(waypoints):
        print(f"  {idx+1}: ({wx:.2f}, {wy:.2f})")

parse_sdf('/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_house/model.sdf')
