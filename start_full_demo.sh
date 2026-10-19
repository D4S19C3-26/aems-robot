#!/usr/bin/env bash
# ==============================================================================
# AEMS Full Simulation & Demo Launch Script
# Starts Gazebo Harmonic, Nav2, AEMS Brain, and Web Dashboard in separate tabs/windows
# ==============================================================================

source /home/dsc/aems_ws/aems_env.sh

echo "=========================================================="
echo "  Starting AEMS Autonomous Mobile Robot Simulation Stack  "
echo "=========================================================="

# 1. Kill any stale ROS/Gazebo processes
killall -9 gz-sim ruby python3 rviz2 2>/dev/null || true
sleep 1

# 2. Check if tmux or gnome-terminal is available
if command -v gnome-terminal &> /dev/null; then
  echo "[1/4] Launching Gazebo Arena Simulation..."
  gnome-terminal --title="1. Gazebo Arena" -- bash -c "source /home/dsc/aems_ws/aems_env.sh && ros2 launch /home/dsc/aems_ws/arena.launch.py; exec bash"
  sleep 4

  echo "[2/4] Launching Nav2 Navigation Stack..."
  gnome-terminal --title="2. Nav2 Minimal" -- bash -c "source /home/dsc/aems_ws/aems_env.sh && ros2 launch /home/dsc/aems_ws/nav2_minimal.launch.py; exec bash"
  sleep 3

  echo "[3/4] Launching AEMS Intelligence Stack..."
  gnome-terminal --title="3. AEMS Brain" -- bash -c "source /home/dsc/aems_ws/aems_env.sh && ros2 launch adaptive_episodic_memory aems.launch.py; exec bash"
  sleep 2

  echo "[4/4] Launching RViz2..."
  gnome-terminal --title="4. RViz2" -- bash -c "source /home/dsc/aems_ws/aems_env.sh && rviz2 -d /home/dsc/aems_ws/install/turtlebot3_navigation2/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz; exec bash"
  
  echo ""
  echo "All windows launched! Now opening Web Dashboard at http://localhost:8000..."
  xdg-open /home/dsc/aems_ws/dashboard/index.html &
else
  echo "gnome-terminal not found. Please run the commands in separate terminal tabs manually."
fi
