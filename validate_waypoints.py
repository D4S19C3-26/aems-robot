import xml.etree.ElementTree as ET
import math

def parse_pose(elem):
    if elem is None or elem.text is None:
        return (0.0, 0.0, 0.0)
    parts = list(map(float, elem.text.strip().split()))
    if len(parts) >= 6:
        return (parts[0], parts[1], parts[5]) # x, y, yaw
    return (0.0, 0.0, 0.0)

def combine_poses(p1, p2):
    x1, y1, yaw1 = p1
    x2, y2, yaw2 = p2
    # Rotate p2 by yaw1
    nx = x1 + x2 * math.cos(yaw1) - y2 * math.sin(yaw1)
    ny = y1 + x2 * math.sin(yaw1) + y2 * math.cos(yaw1)
    nyaw = yaw1 + yaw2
    return (nx, ny, nyaw)

tree = ET.parse('/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_house/model.sdf')
root = tree.getroot()

geometries = []

# Support models inside models (like the cafe table)
for model in root.findall('.//model'):
    model_pose = parse_pose(model.find('pose'))
    
    for link in model.findall('.//link'):
        link_pose = parse_pose(link.find('pose'))
        abs_link_pose = combine_poses(model_pose, link_pose)
        
        for coll in link.findall('.//collision'):
            coll_pose = parse_pose(coll.find('pose'))
            abs_pose = combine_poses(abs_link_pose, coll_pose)
            
            geom = coll.find('geometry')
            if geom is not None:
                if geom.find('box') is not None:
                    size = geom.find('box/size').text.strip().split()
                    geometries.append(('box', abs_pose, (float(size[0]), float(size[1]))))
                elif geom.find('cylinder') is not None:
                    rad = float(geom.find('cylinder/radius').text)
                    geometries.append(('cylinder', abs_pose, rad))
                elif geom.find('mesh') is not None:
                    # Treat meshes as 0.5m radius cylinders roughly
                    geometries.append(('cylinder', abs_pose, 0.5))

# Don't forget links directly under root model
for link in root.findall('./model/link'):
    link_pose = parse_pose(link.find('pose'))
    for coll in link.findall('.//collision'):
        coll_pose = parse_pose(coll.find('pose'))
        abs_pose = combine_poses(link_pose, coll_pose)
        
        geom = coll.find('geometry')
        if geom is not None:
            if geom.find('box') is not None:
                size = geom.find('box/size').text.strip().split()
                geometries.append(('box', abs_pose, (float(size[0]), float(size[1]))))
            elif geom.find('cylinder') is not None:
                rad = float(geom.find('cylinder/radius').text)
                geometries.append(('cylinder', abs_pose, rad))
            elif geom.find('mesh') is not None:
                geometries.append(('cylinder', abs_pose, 0.5))

def dist_to_geom(px, py, geom):
    gtype, pose, data = geom
    gx, gy, gyaw = pose
    
    dx = px - gx
    dy = py - gy
    
    # local frame
    lx = dx * math.cos(-gyaw) - dy * math.sin(-gyaw)
    ly = dx * math.sin(-gyaw) + dy * math.cos(-gyaw)
    
    if gtype == 'box':
        sx, sy = data
        dist_x = max(0.0, abs(lx) - sx/2.0)
        dist_y = max(0.0, abs(ly) - sy/2.0)
        return math.sqrt(dist_x**2 + dist_y**2)
    elif gtype == 'cylinder':
        rad = data
        d = math.sqrt(lx**2 + ly**2)
        return max(0.0, d - rad)
    return float('inf')

def is_safe(px, py, margin=0.35):
    for geom in geometries:
        if dist_to_geom(px, py, geom) < margin:
            return False
    return True

original_wps = [
  (-6.18, -2.19), (-6.18, 1.00), (-6.18, 4.19),
  (-2.34, 4.19), (-2.34, 1.00), (-2.34, -2.19),
  (1.51, -2.19), (1.51, 1.00), (1.51, 4.19),
  (5.36, 4.19), (5.36, 1.00), (5.36, -2.19)
]

final_wps = []
print("Validating original waypoints...")
for i, (wx, wy) in enumerate(original_wps):
    if is_safe(wx, wy):
        print(f"WP {i+1} is SAFE: ({wx:.2f}, {wy:.2f})")
        final_wps.append((wx, wy))
    else:
        # Spiral search
        found = False
        for r in [0.2, 0.4, 0.6, 0.8, 1.0, 1.2]:
            for angle in range(0, 360, 45):
                rad = math.radians(angle)
                nx = wx + r * math.cos(rad)
                ny = wy + r * math.sin(rad)
                if is_safe(nx, ny):
                    print(f"WP {i+1} in COLLISION! Nudged by {r}m to ({nx:.2f}, {ny:.2f})")
                    final_wps.append((nx, ny))
                    found = True
                    break
            if found: break
        if not found:
            print(f"WP {i+1} in COLLISION! Could not find safe spot within 1.2m. Dropped.")

# Gap analysis
print("\nChecking for missed rooms...")
free_points = []
for x in range(-7, 7):
    for y in range(-3, 6):
        if is_safe(x, y, 0.5):
            # Distance to nearest WP
            min_d = min(math.sqrt((x-wx)**2 + (y-wy)**2) for wx, wy in final_wps)
            if min_d > 3.0:
                free_points.append((x, y))

if free_points:
    # Very naive clustering: just pick the first one, then filter, repeat.
    missed_wps = []
    for fx, fy in free_points:
        if not missed_wps or min(math.sqrt((fx-mx)**2 + (fy-my)**2) for mx, my in missed_wps) > 2.0:
            missed_wps.append((fx, fy))
            final_wps.append((fx, fy))
            print(f"Added targeted WP for missed area: ({fx:.2f}, {fy:.2f})")
else:
    print("No missed rooms detected.")

print("\nFINAL WAYPOINT LIST:")
for idx, (wx, wy) in enumerate(final_wps):
    print(f"  {idx+1}: ({wx:.2f}, {wy:.2f})")

