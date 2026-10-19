# AEMS Installation Guide

## 1. System Requirements & Specifications

- **Operating System:** Ubuntu 24.04 LTS (Noble Numbat), 64-bit x86_64
- **ROS 2 Distribution:** ROS 2 Jazzy Jalisco
- **Simulation Engine:** Gazebo Harmonic (gz-sim 8)
- **Robot Platform:** TurtleBot3 Waffle simulation model
- **Python Version:** Python 3.12+
- **Hardware Minimum:**
  - CPU: Quad-core 2.0 GHz+ (Tested on AMD Ryzen 5 5500U, 6 cores / 12 threads)
  - RAM: 8 GB minimum, 16 GB recommended (Tested on 13.5 GB)
  - Storage: 20 GB free disk space
  - Network: Broadband internet access for initial model weights and Groq API queries

---

## 2. Prerequisites & System Dependencies

### 2.1 ROS 2 Jazzy Installation
If ROS 2 Jazzy is not already installed:
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep
```

### 2.2 Nav2 & TurtleBot3 Packages
```bash
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-turtlebot3* \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-rosbridge-suite
```

### 2.3 Python Dependencies
Install required scientific computing and deep learning packages:
```bash
pip install --break-system-packages \
  ultralytics \
  sentence-transformers \
  faiss-cpu \
  groq \
  pyyaml \
  pytest
```

---

## 3. Workspace Setup & Environment Configuration

### 3.1 Clone & Directory Structure
```bash
cd ~
# If setting up fresh workspace:
mkdir -p aems_ws/src
cd aems_ws
```

### 3.2 Environment File (`.env`)
Create `/home/dsc/aems_ws/.env` (DO NOT commit API keys to version control):
```bash
cat << 'ENVEOF' > /home/dsc/aems_ws/.env
export GROQ_API_KEY="your-groq-api-key-here"
export TURTLEBOT3_MODEL="waffle"
export GAZEBO_MODEL_PATH="/home/dsc/aems_ws/models:$GAZEBO_MODEL_PATH"
ENVEOF
chmod 600 /home/dsc/aems_ws/.env
```

### 3.3 Environment Sourcing Script (`aems_env.sh`)
```bash
cat << 'SHEOF' > /home/dsc/aems_ws/aems_env.sh
#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
if [ -f /home/dsc/aems_ws/install/setup.bash ]; then
  source /home/dsc/aems_ws/install/setup.bash
fi
if [ -f /home/dsc/aems_ws/.env ]; then
  set -a
  source /home/dsc/aems_ws/.env
  set +a
fi
export TURTLEBOT3_MODEL=waffle
SHEOF
chmod +x /home/dsc/aems_ws/aems_env.sh
```

---

## 4. Building the Workspace

```bash
cd /home/dsc/aems_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 5. Verification & Testing

Verify that all packages build and the integration suite passes:
```bash
source /home/dsc/aems_ws/aems_env.sh
python3 -m pytest src/adaptive_episodic_memory/test/test_aems_integration.py -v
```
All 24 test cases must pass.
