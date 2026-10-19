import xml.etree.ElementTree as ET

def get_geometry_types(filepath):
    tree = ET.parse(filepath)
    root = tree.getroot()
    types = set()
    for geom in root.findall('.//collision/geometry/*'):
        types.add(geom.tag)
    print("Geometry types found:", types)

get_geometry_types('/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_house/model.sdf')
