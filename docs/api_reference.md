# AEMS API Reference

## 1. ROS 2 Topics

### 1.1 Published Topics

| Topic Name | Message Type | Description |
|---|---|---|
| `/aems/detections` | `aems_interfaces/msg/ObjectDetection` | 3D localized detections from YOLO + LiDAR fusion |
| `/aems/command_response` | `std_msgs/msg/String` | JSON payload containing command decisions, explanations, and outcomes |
| `/aems/task_status` | `std_msgs/msg/String` | High-level status messages and progress percentages for dashboard UI |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Safe velocity commands sent to robot base controller |
| `/cmd_vel_raw` | `geometry_msgs/msg/Twist` | Raw velocity commands published during visual sweep & rotation |

### 1.2 Subscribed Topics

| Topic Name | Message Type | Description |
|---|---|---|
| `/aems/nl_command` | `std_msgs/msg/String` | Natural-language commands from user or web dashboard |
| `/emergency_stop` | `std_msgs/msg/Bool` | Emergency stop signal (`True` halts robot, `False` releases) |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | AMCL localization pose in map frame |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR range scan data |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Raw RGB image frames from camera |

---

## 2. ROS 2 Services & Actions

### 2.1 Action Servers & Clients

#### `navigate_to_pose`
- **Action Type:** `nav2_msgs/action/NavigateToPose`
- **Client Node:** `dashboard_command_node`
- **Description:** Dispatches goal coordinates $(x, y, \theta)$ to Nav2 motion planner.

### 2.2 Service Servers & Clients

#### `/query_memory`
- **Service Type:** `aems_interfaces/srv/QueryMemory`
- **Server:** `memory_manager_node`
- **Request:**
  ```text
  string object_class
  ---
  ```
- **Response:**
  ```text
  bool found
  float64 pos_x
  float64 pos_y
  float64 confidence
  int32 corroboration_count
  string scene_description
  ```

#### `/replay_route`
- **Service Type:** `aems_interfaces/srv/ReplayRoute`
- **Server:** `route_replayer_node`
- **Request:**
  ```text
  string target_class
  ---
  ```
- **Response:**
  ```text
  bool success
  int32 landmarks_visited
  string message
  ```

---

## 3. Custom ROS 2 Messages (`aems_interfaces`)

### `ObjectDetection.msg`
```text
std_msgs/Header header
string class_name
float32 confidence
geometry_msgs/PoseStamped robot_pose
float32 distance_m
float32 angle_rad
```

---

## 4. WebSocket JSON Protocol (Web Bridge)

The web bridge broadcasts JSON packets over WebSocket (`ws://localhost:9090`):

### 4.1 Periodic Snapshot (`db_snapshot`)
Broadcast every 3 seconds:
```json
{
  "type": "db_snapshot",
  "memories": [
    {
      "episode_id": 1,
      "object_class": "couch",
      "confidence": 0.92,
      "corroboration_count": 4,
      "robot_pos_x": 0.0,
      "robot_pos_y": -7.0,
      "task_info": "autonomous_patrol",
      "scene_description": "couch located near southern wall",
      "status": "active"
    }
  ],
  "landmarks": [...],
  "edges": [...],
  "task_history": [
    {
      "task_id": 1,
      "command": "go to couch",
      "strategy": "direct",
      "decision": "navigate_trust",
      "outcome": "SUCCESS_VISUAL",
      "start_time": 1729158000.0,
      "duration": 8.4,
      "goal_pos": "(0.00, -5.80)"
    }
  ]
}
```
