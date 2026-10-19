# AEMS ROS 2 runtime environment for Jazzy.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_IP=127.0.0.1
export TURTLEBOT3_MODEL=waffle
# Load environment variables from .env if available
if [ -f /home/dsc/aems_ws/.env ]; then
  export $(grep -v '^#' /home/dsc/aems_ws/.env | xargs)
fi
unset ROS_LOCALHOST_ONLY
unset FASTRTPS_DEFAULT_PROFILES_FILE

# This workspace uses isolated installs. Source the interface first because
# adaptive_episodic_memory imports its generated ObjectDetection message.
if [ -f /home/dsc/aems_ws/install/aems_interfaces/share/aems_interfaces/local_setup.bash ]; then
  source /home/dsc/aems_ws/install/aems_interfaces/share/aems_interfaces/local_setup.bash
fi
if [ -f /home/dsc/aems_ws/install/adaptive_episodic_memory/share/adaptive_episodic_memory/package.bash ]; then
  source /home/dsc/aems_ws/install/adaptive_episodic_memory/share/adaptive_episodic_memory/package.bash
fi
