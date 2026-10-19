#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /home/dsc/aems_ws/aems_env.sh
source /home/dsc/aems_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle

cleanup() {
  jobs -p | xargs -r kill
}
trap cleanup EXIT INT TERM

# Starts the arena world, robot_state_publisher, TurtleBot spawn, and ROS-GZ bridge.
# The bridge publishes /scan and /odom and subscribes to /cmd_vel_nav.
ros2 launch /home/dsc/aems_ws/arena.launch.py &
ARENA_PID=$!

sleep 8

# Online SLAM mapping only. Keep Nav2 off during manual mapping so it does not
# publish competing velocity commands while you drive with WASD.
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true &
SLAM_PID=$!

wait $ARENA_PID $SLAM_PID
