#!/bin/bash
# Spawns 4 realistic objects into the running Gazebo simulator

source /opt/ros/jazzy/setup.bash
source ~/aems_ws/aems_env.sh

echo "Spawning Coke Can..."
ros2 run ros_gz_sim create -world default -string '
<sdf version="1.8">
  <include>
    <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke Can</uri>
    <name>coke_can_1</name>
    <pose>-3.5 -0.5 0.82 0 0 0</pose>
  </include>
</sdf>' > /dev/null 2>&1

echo "Spawning Beer Bottle..."
ros2 run ros_gz_sim create -world default -string '
<sdf version="1.8">
  <include>
    <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Beer</uri>
    <name>beer_bottle_1</name>
    <pose>1.5 3.0 0.82 0 0 0</pose>
  </include>
</sdf>' > /dev/null 2>&1

echo "Spawning Bowl..."
ros2 run ros_gz_sim create -world default -string '
<sdf version="1.8">
  <include>
    <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Bowl</uri>
    <name>bowl_1</name>
    <pose>-1.0 2.5 0.82 0 0 0</pose>
  </include>
</sdf>' > /dev/null 2>&1

echo "Spawning Backpack..."
ros2 run ros_gz_sim create -world default -string '
<sdf version="1.8">
  <include>
    <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Backpack</uri>
    <name>backpack_1</name>
    <pose>3.0 -1.0 0.0 0 0 0</pose>
  </include>
</sdf>' > /dev/null 2>&1

echo "Done! Look in Gazebo — the objects should now be visible!"
