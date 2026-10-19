# Phase 1: visual object detection

The standard TurtleBot3 House world is the Phase 1 simulation. The Waffle model launched by `turtlebot3_house.launch.py` includes a Gazebo RGB camera, and its standard TurtleBot spawn launch bridges the raw feed to ROS 2 at `/camera/image_raw`.

```bash
cd ~/aems_ws
source /opt/ros/jazzy/setup.bash
source aems_env.sh
colcon build --symlink-install
source install/aems_interfaces/share/aems_interfaces/local_setup.bash
source install/adaptive_episodic_memory/share/adaptive_episodic_memory/package.bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

In another terminal, source the same environment, start RViz2, add an **Image**
display, select `/camera/image_raw`, and choose the Sensor Data QoS profile.
Then start the perception/memory pipeline:

```bash
ros2 launch adaptive_episodic_memory aems.launch.py
ros2 topic echo /aems/detections
```

`/aems/detections` now uses the custom `aems_interfaces/msg/ObjectDetection`
message. Every event has the original camera timestamp, class, confidence,
`[x1,y1,x2,y2]` bounding box, and robot pose. The detector prioritizes
`/amcl_pose`; `/odom` is an automatic fallback while localization is not yet
running.

Use visually realistic bottle, mug, and chair models in the TurtleBot3 House world for the recognition demonstration. Then drive past them and record the echoed detection messages. A COCO-pretrained YOLOv8n model is not expected to reliably identify simple geometric primitives.
