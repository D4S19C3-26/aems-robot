#!/usr/bin/env python3
"""
Move any object in Gazebo Sim at runtime via Gazebo service.
Usage:
    python3 move_object.py <object_name> <x> <y> [z] [yaw_deg]

Examples:
    python3 move_object.py person 5.0 5.0
    python3 move_object.py car -4.0 6.0
    python3 move_object.py sofa 2.0 -4.0
    python3 move_object.py hydrant 6.0 2.0
"""
import sys
import subprocess
import math

NAME_MAP = {
    'car': 'suv',
    'suv': 'suv',
    'person': 'standing_person',
    'standing_person': 'standing_person',
    'hydrant': 'fire_hydrant',
    'fire_hydrant': 'fire_hydrant',
    'couch': 'sofa',
    'sofa': 'sofa',
    'table': 'dining_table',
    'dining_table': 'dining_table',
    'chair': 'dining_chair',
    'dining_chair': 'dining_chair',
}

def main():
    if len(sys.argv) < 4:
        print("Usage: python3 move_object.py <object> <x> <y> [z] [yaw_deg]")
        print("Available objects:", list(set(NAME_MAP.keys())))
        sys.exit(1)

    obj_input = sys.argv[1].lower().strip()
    model_name = NAME_MAP.get(obj_input, obj_input)

    x = float(sys.argv[2])
    y = float(sys.argv[3])
    z = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    yaw_deg = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
    yaw_rad = math.radians(yaw_deg)

    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)

    req_str = f'name: "{model_name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}'

    cmd = [
        'gz', 'service',
        '-s', '/world/arena_world/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000',
        '--req', req_str
    ]

    print(f"Moving '{model_name}' to ({x:.2f}, {y:.2f}, {z:.2f}) yaw={yaw_deg}°...")
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode == 0 and 'data: true' in res.stdout:
        print(f"✅ Success: '{model_name}' moved to ({x:.2f}, {y:.2f})!")
    else:
        print(f"Output: {res.stdout.strip()} {res.stderr.strip()}")

if __name__ == '__main__':
    main()
