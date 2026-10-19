# AEMS Demo Guide

## 1. Demo Overview

This guide walks through the complete end-to-end evaluation scenario for AEMS:
1. **Initial Patrol & Observation:** Robot discovers objects in the arena and populates episodic memory.
2. **Natural-Language Query:** User asks where an object is located without initiating navigation.
3. **Trust-Based Navigation:** User commands navigation to a known object; robot executes direct navigation with unobstructed wall-clearing standoff.
4. **Autonomous Multi-Room Perimeter Scout:** If an object is occluded behind interior partition walls upon arrival, robot executes perimeter search.
5. **Conflict Supersede & Update:** Object is moved to a new location; robot detects conflict, supersedes old record, and updates memory.
6. **Named Zone Navigation:** User directs the robot to architectural zones (`reception_area`, `vehicle_bay`).
7. **Emergency Stop (E-Stop):** User triggers E-Stop mid-motion; robot halts instantly and aborts task safely.

---

## 2. Step-by-Step Execution

### Step 1: Start System Simulation
Terminal 1:
```bash
source /home/dsc/aems_ws/aems_env.sh
ros2 launch adaptive_episodic_memory aems.launch.py
```
Wait until Gazebo Harmonic and RViz2 load completely.

### Step 2: Open Web Dashboard
In browser, open:
```
http://localhost:8000
```
Verify WebSocket connection indicator displays green (Connected).

### Step 3: NL Memory Query
Issue an open-ended memory query:
```bash
ros2 topic pub --once /aems/nl_command std_msgs/msg/String "{data: 'What do you remember?'}"
```
Verify the dashboard displays the grounded LLM response listing all active object sightings.

### Step 4: Targeted Navigation
Command the robot to navigate to the couch:
```bash
ros2 topic pub --once /aems/nl_command std_msgs/msg/String "{data: 'Go to couch'}"
```
- Observe `TrustVerifyReasoner` cost computation in Dashboard Panel 3.
- Observe Nav2 dispatching the robot to the unobstructed standoff $(0.0, -5.8)$.
- Observe visual stop confirmation upon high-confidence camera detection.

### Step 5: Test Emergency Stop
Command navigation to another object, then immediately trigger E-Stop:
```bash
ros2 topic pub --once /aems/nl_command std_msgs/msg/String "{data: 'Go to car'}"
sleep 2
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
```
- Verify robot stops instantaneously.
- Verify `task_history` records `ABORTED_ESTOP`.

Release E-Stop:
```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}"
```

### Step 6: Named Zone Navigation
Direct the robot to the reception lobby:
```bash
ros2 topic pub --once /aems/nl_command std_msgs/msg/String "{data: 'Navigate to lobby'}"
```
- Verify `_resolve_zone` resolves `lobby` -> `reception_area`.
- Robot navigates to $(0.0, 5.0)$ standoff.
