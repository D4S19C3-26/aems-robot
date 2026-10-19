import os

SDF_PATH = os.path.expanduser('~/aems_ws/worlds/realistic_house.sdf')

# Wall definitions: (name, cx, cy, length, is_horizontal)
walls = [
    ("living_west", -6.0, 3.5, 4.0, False),
    ("living_north", -3.5, 5.5, 5.15, True), # added 0.15 for corner overlap
    ("living_east", -1.0, 3.5, 4.0, False),
    ("living_south1", -4.75, 1.5, 2.5, True),
    ("living_south2", -1.8, 1.5, 1.6, True),
    
    ("kitchen_north", 1.0, 5.0, 4.15, True),
    ("kitchen_east", 3.0, 3.25, 3.65, False),
    ("kitchen_south1", 0.0, 1.5, 2.0, True),
    ("kitchen_south2", 2.45, 1.5, 1.1, True),
    
    ("bathroom_north", 4.25, 3.5, 2.65, True),
    ("bathroom_east", 5.5, 2.5, 2.15, False),
    ("bathroom_south1", 3.25, 1.5, 0.5, True),
    ("bathroom_south2", 4.95, 1.5, 1.1, True),
    
    ("bedroom_west", -6.0, -1.75, 3.5, False),
    ("bedroom_north1", -5.0, 0.0, 2.0, True),
    ("bedroom_north2", -2.55, 0.0, 1.1, True),
    ("bedroom_east", -2.0, -1.75, 3.5, False),
    ("bedroom_south", -4.0, -3.5, 4.15, True),
    
    ("hallway_south", 1.75, 0.0, 7.65, True),
    ("hallway_west", -6.0, 0.75, 1.5, False),
    ("hallway_east", 5.5, 0.75, 1.5, False),
]

# Furniture definitions: (uri, x, y, z, yaw)
furniture = [
    # Living Room (Center-ish, open)
    ("https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sofa", -3.5, 4.0, 0, 0),
    ("https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table", -3.5, 3.0, 0, 0),
    # Kitchen (Center-ish)
    ("https://fuel.gazebosim.org/1.0/OpenRobotics/models/Dining table", 1.0, 3.25, 0, 0),
    # Bedroom (Flush against south wall)
    ("https://fuel.gazebosim.org/1.0/OpenRobotics/models/Bed", -4.0, -2.5, 0, 1.5708),
    # Bathroom
    ("https://fuel.gazebosim.org/1.0/OpenRobotics/models/Cabinet", 4.25, 3.0, 0, 3.1415),
]

sdf_content = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="realistic_house">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics" />
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands" />
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster" />
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="house_walls">
      <static>true</static>
"""

THICKNESS = 0.15
HEIGHT = 2.4

for name, cx, cy, length, is_horizontal in walls:
    sx = length if is_horizontal else THICKNESS
    sy = THICKNESS if is_horizontal else length
    
    sdf_content += f"""
      <link name="{name}">
        <pose>{cx} {cy} {HEIGHT/2} 0 0 0</pose>
        <collision name="{name}_col">
          <geometry><box><size>{sx} {sy} {HEIGHT}</size></box></geometry>
        </collision>
        <visual name="{name}_vis">
          <geometry><box><size>{sx} {sy} {HEIGHT}</size></box></geometry>
          <material>
            <ambient>0.9 0.9 0.9 1</ambient>
            <diffuse>0.9 0.9 0.9 1</diffuse>
          </material>
        </visual>
      </link>
"""

sdf_content += "    </model>\n\n"

for i, (uri, x, y, z, yaw) in enumerate(furniture):
    sdf_content += f"""
    <include>
      <name>furniture_{i}</name>
      <uri>{uri}</uri>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
    </include>
"""

sdf_content += """
  </world>
</sdf>
"""

os.makedirs(os.path.dirname(SDF_PATH), exist_ok=True)
with open(SDF_PATH, 'w') as f:
    f.write(sdf_content)

print(f"Generated {SDF_PATH}")
