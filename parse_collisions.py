import xml.etree.ElementTree as ET

tree = ET.parse('/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_house/model.sdf')
root = tree.getroot()

for link in root.findall('.//link'):
    link_name = link.get('name')
    for coll in link.findall('.//collision'):
        geom = coll.find('geometry')
        if geom is not None:
            tags = [child.tag for child in geom]
            print(f"Link: {link_name}, Collision: {coll.get('name')}, Geom: {tags}")
