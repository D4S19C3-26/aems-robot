# AEMS User Manual

## 1. Introduction

The **Adaptive Episodic Memory System (AEMS)** equips autonomous mobile service robots (TurtleBot3 Waffle) with long-term, semantic, and spatial memory capabilities. Rather than relying solely on static metric maps, AEMS links perceptual observations (YOLOv8s object detections) to topometric landmark graphs and natural-language spatial scene descriptions.

---

## 2. Launching the System

### 2.1 Complete System Launch
To start Gazebo Harmonic simulation, Nav2 stack, YOLO perception, memory manager, dashboard node, and web bridge:
```bash
source /home/dsc/aems_ws/aems_env.sh
ros2 launch adaptive_episodic_memory aems.launch.py
```

### 2.2 Accessing the Web Dashboard
Open any modern web browser and navigate to:
```
http://localhost:8000
```
Or open the local file:
```bash
xdg-open /home/dsc/aems_ws/dashboard/index.html
```
The dashboard establishes a WebSocket connection over port `9090` to receive live system state, camera feed, landmark topology, and task history.

---

## 3. Command Modalities & Interface

Commands can be issued either via the Web Dashboard input box or by publishing directly to the ROS topic `/aems/nl_command`:
```bash
ros2 topic pub --once /aems/nl_command std_msgs/msg/String "{data: 'Where is the couch?'}"
```

### 3.1 Mode 1: Natural-Language Memory Queries (No Navigation)
Free-form inquiries querying the robot's past episodic observations without triggering robot motion.
- **Examples:**
  - `"Where was the couch seen?"`
  - `"What objects are stored in memory?"`
  - `"What did you observe near the dining table?"`
  - `"Tell me everything you remember"`
- **Response:** The system uses sentence-transformers embedding and FAISS top-k retrieval, followed by Groq LLM synthesis to produce a grounded response.

### 3.2 Mode 2: Named Zone Navigation
Commands directing the robot to named architectural arena regions.
- **Supported Zones & Aliases:**
  - `vehicle_bay` (Garage, parking area, bay) -> standoff `(-5.5, 6.0)`
  - `reception_area` (Lobby, entrance, front desk) -> standoff `(0.0, 5.0)`
  - `east_corridor` (Hallway, passage) -> standoff `(5.5, 2.0)`
  - `seating_area` (Lounge, living room, rest area) -> standoff `(-1.5, -4.5)`
  - `dining_area` (Cafeteria, kitchen, breakroom) -> standoff `(-5.5, -2.0)`
  - `study_alcove` (Office, workstation, quiet room) -> standoff `(3.0, -4.5)`
- **Examples:**
  - `"Drive to reception area"`
  - `"Navigate to the cafeteria"`
  - `"Head to the garage"`

### 3.3 Mode 3: Object Localization & Targeted Navigation
Commands requesting the robot to find, verify, and approach a specific physical object.
- **Examples:**
  - `"Go to the couch"`
  - `"Navigate to the SUV"`
  - `"Find the fire hydrant"`
- **Execution Flow:**
  1. Retrieve highest-scoring active memory for the object.
  2. Compute cost of trust vs. reverification (`TrustVerifyReasoner`).
  3. Select strategy: Direct navigation vs. Landmark route replay.
  4. Compute unobstructed standoff outside wall occlusion zones.
  5. If occluded, execute Autonomous Multi-Room Perimeter Scout.
  6. Visual verification confirm upon high-confidence sighting.

---

## 4. Safety & Emergency Controls

### 4.1 Software Emergency Stop (E-Stop)
To immediately halt all robot motion:
```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
```
- Active Nav2 goals are immediately cancelled.
- Zero twist is published to both `/cmd_vel_raw` and `/cmd_vel`.
- Active tasks are logged in `task_history` as `ABORTED_ESTOP`.

To release E-Stop:
```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}"
```

### 4.2 Arena Goal Bounding
All navigation goals are automatically clamped to the designated safe arena envelope:
`x in [-9.5, 9.5] meters`, `y in [-9.5, 9.5] meters`.

---

## 5. Web Dashboard Panels

- **Panel 1: Camera Feed & Object Detections:** Live camera stream with overlaid bounding boxes and confidence scores.
- **Panel 2: Topometric Landmark Map:** Interactive topological graph showing landmarks and inter-landmark edges.
- **Panel 3: Strategy & Reasoning Explanations:** Live reasoning stream explaining cost evaluations, strategy selection, and LLM explanations.
- **Panel 4: Memory Vault & Task History:** Tabbed view displaying active episodic memories with filter controls and historical task execution logs.
