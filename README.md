# AEMS — Adaptive Episodic Memory System for Robots

> A ROS 2 Jazzy framework that gives a mobile robot long-term object memory with semantic retrieval, decision-theoretic navigation, and a live web dashboard.

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Python 3.12](https://img.shields.io/badge/Python-3.12-blue?logo=python)](https://www.python.org/)
[![Gazebo Sim](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
[![YOLOv8](https://img.shields.io/badge/YOLO-v8-purple)](https://github.com/ultralytics/ultralytics)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-green.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/Tests-24%20passed-brightgreen)]()

---

## What is AEMS?

AEMS is a complete **robotic episodic memory system** built on **ROS 2 Jazzy** and **Gazebo Harmonic** that enables a TurtleBot3 Waffle to:

- **Remember** where it has seen objects (chair, couch, car, person, fire hydrant, dining table) using a persistent SQLite database with 384-dimensional semantic vector embeddings
- **Reason** about whether memory is stale using a **decision-theoretic cost model** (trust vs. re-verify cost formulation)
- **Navigate** intelligently — going directly to a remembered location, adapting en-route when an object has moved, or autonomously scouting multiple rooms when the object is occluded
- **Search** its entire memory in **< 15 ms** across 10,000 records using FAISS dense vector retrieval
- **Answer** natural language queries about what it has seen using a Groq-hosted LLM
- **Broadcast** everything live to a **web dashboard** (HTTP :8080 + WebSocket :9090)

---

## System Architecture

```
RGB Camera + 2D LiDAR
        │
        ▼
┌─────────────────────────────┐
│  YOLOv8 + Camera-LiDAR     │  ← yolo_perception_node.py
│  Fusion + Spatial Disambig. │    Async 30 FPS HUD overlay
└────────────┬────────────────┘
             │ /aems/detections
             ▼
┌─────────────────────────────┐
│  Landmark Manager Node      │  ← landmark_manager_node.py
│  SQLite Memory Store        │    Topometric graph + FAISS index
│  FAISS IndexFlatIP          │    all-MiniLM-L6-v2 embeddings
└────────────┬────────────────┘
             │ ROS 2 Service
             ▼
┌─────────────────────────────┐
│  Dashboard / Command Node   │  ← dashboard_node.py
│  - NL command parsing       │    Nav2 Action Client
│  - Trust-vs-Verify cost     │    Decision-theoretic strategy
│  - Multi-Room Perimeter     │    selection
│    Scout                    │
│  - En-route re-targeting    │
└────────────┬────────────────┘
             │ WebSocket
             ▼
┌─────────────────────────────┐
│  Web Dashboard              │  ← aems_web_bridge.py + dashboard/
│  Live HUD | Memory Map      │    http://localhost:8080
│  Task Status | Benchmarks   │
└─────────────────────────────┘
```

---

## Key Features

| Feature | Details |
|---|---|
| **Perception** | YOLOv8n/s + Camera-LiDAR fusion, multi-cluster occlusion detection, spatial disambiguation |
| **Memory Store** | SQLite with robot pose, task context, scene description, binary embedding per sighting |
| **Semantic Retrieval** | FAISS `IndexFlatIP` on `all-MiniLM-L6-v2` 384-d vectors, **11.67 ms** mean across 10k records |
| **Decision Reasoning** | Decision-theoretic cost model — C_trust vs C_reverify vs C_explore |
| **Adaptive Navigation** | Direct Nav → En-Route Re-targeting → 360° Sweep → Multi-Room Perimeter Scout |
| **Negative Corroboration** | Confidence decay on failed searches; stale memory detection |
| **NL Interface** | Groq LLM answers queries like *"where did you last see the couch?"* |
| **Web Dashboard** | Live camera HUD, memory table, robot map, task progress — no RViz needed |
| **Safety** | Input sanitizer (SQL injection, shell metachar), E-Stop gate, goal bounding box |
| **Benchmarks** | Empirical 50-trial comparison vs. Frontier Exploration and Naive Static Memory |

---

## Benchmark Results

### Benchmark 1: FAISS Semantic Retrieval (10,000 Records)

Run: `python3 benchmarks/benchmark_retrieval_10k.py`

| Metric | Value |
|---|---|
| Mean Latency | **11.67 ms** |
| Median Latency | 11.61 ms |
| P95 Latency | 13.04 ms |
| P99 Latency | 13.70 ms |
| Throughput | **85.7 queries/sec** |
| NFR-P2 Target (< 1000 ms) | ✅ EXCEEDED |

Breakdown: Query Embedding 10.53 ms (90%) · FAISS Search 0.95 ms (8%) · Scoring 0.18 ms (2%)

---

### Benchmark 2: Navigation Comparison (N=50 Trials)

Run: `python3 benchmarks/run_baseline_comparison.py`

| Scenario | Method | Success | Latency | Distance |
|---|---|---|---|---|
| **Sc.1** Static Target | Frontier Exploration | 100% | 196.4 ± 99.2s | 37.3m |
| | Naive Static Memory | 100% | 82.2 ± 2.4s | 16.1m |
| | **AEMS (Proposed)** | **100%** | **65.6 ± 1.9s** | **14.8m** |
| **Sc.2** Relocated Target | Frontier Exploration | 100% | 197.7 ± 108.3s | 37.7m |
| | Naive Static Memory | 100% | 286.1 ± 94.0s | 51.7m |
| | **AEMS (Proposed)** | **100%** | **43.0 ± 1.2s** | **9.9m** |
| **Sc.3** Occluded Target | Frontier Exploration | 100% | 192.4 ± 106.9s | 37.6m |
| | Naive Static Memory | **0%** | 60.1 ± 1.8s | 10.3m |
| | **AEMS (Proposed)** | **100%** | **29.6 ± 0.9s** | **6.5m** |

---

## Decision-Theoretic Cost Model

At each navigation request, AEMS evaluates three strategies:

```
p_valid   = conf × exp(-λ × Δt) × min(1.0, 0.5 + 0.25 × (corroboration - 1))

C_trust   = 8.0 + (1 - p_valid) × 40.0
C_reverify = 12.0 + 8.0 × 0.6
C_explore  = 40.0
```

- `C_trust ≤ C_reverify` → **Direct Trust** (navigate straight to memory)
- `C_reverify < C_trust` → **Reverify** (go there + 360° scan on arrival)
- Both exceed threshold → **Full Scout** (multi-room perimeter search)

---

## Repository Structure

```
aems_ws/
├── src/
│   ├── adaptive_episodic_memory/        # Main ROS 2 package
│   │   ├── adaptive_episodic_memory/
│   │   │   ├── perception/
│   │   │   │   └── yolo_perception_node.py   # YOLOv8 + LiDAR fusion, 30 FPS HUD
│   │   │   ├── memory/
│   │   │   │   ├── landmark_manager_node.py  # Episodic memory + FAISS index
│   │   │   │   ├── memory_manager_node.py    # Per-class sighting deduplication
│   │   │   │   ├── retrieval.py              # Semantic retrieval engine
│   │   │   │   ├── memory_store.py           # SQLite abstraction layer
│   │   │   │   └── route_replayer_node.py    # Landmark route replay
│   │   │   ├── reasoning/
│   │   │   │   ├── dashboard_node.py         # NL commands, Nav2, trust-verify
│   │   │   │   ├── trust_verify.py           # Decision-theoretic cost formulation
│   │   │   │   └── llm_reasoning.py          # Groq LLM integration
│   │   │   └── utils/
│   │   │       ├── aems_web_bridge.py        # WebSocket + HTTP server (:8080/:9090)
│   │   │       ├── auto_explorer.py          # Autonomous frontier exploration
│   │   │       └── teleop_wasd.py            # Manual WASD keyboard teleop
│   │   ├── config/
│   │   │   └── zones.yaml                    # Named zone definitions
│   │   ├── launch/
│   │   │   └── aems.launch.py                # Full system launch file
│   │   └── test/
│   │       └── test_aems_integration.py      # 24 integration tests (all pass)
│   ├── aems_interfaces/                      # Custom ROS 2 msg: ObjectDetection
│   └── frontier_exploration_ros2/            # Baseline frontier explorer
├── benchmarks/
│   ├── benchmark_retrieval_10k.py            # FAISS 10k benchmark
│   ├── run_baseline_comparison.py            # 3-scenario comparison vs baselines
│   ├── baseline_comparison_results.json      # Results (N=50 trials)
│   └── retrieval_benchmark_results.json      # FAISS latency results
├── dashboard/
│   └── index.html                            # Web dashboard UI
├── worlds/
│   └── arena.sdf                             # Gazebo arena world
├── maps/                                     # Pre-built SLAM maps (.pgm + .yaml)
├── docs/                                     # Architecture, SRS, API docs
├── arena.launch.py                           # Launches Gazebo + TurtleBot3
├── nav2_minimal.launch.py                    # Launches Nav2 navigation stack
├── nav2_params.yaml                          # Nav2 DWB controller params
├── move_object.py                            # Move Gazebo objects at runtime
├── reset_memory_db.py                        # Reset database to ground truth
├── yolov8n.pt                                # YOLOv8n weights (6.5 MB)
└── yolov8s.pt                                # YOLOv8s weights (21 MB)
```

---

## Installation & Setup

### Prerequisites

- **OS**: Ubuntu 24.04
- **ROS 2**: Jazzy Jalisco
- **Gazebo**: Harmonic
- **Python**: 3.12

### 1. Clone the Repository

```bash
git clone https://github.com/<your-username>/aems_ws.git
cd aems_ws
```

### 2. Install Python Dependencies

```bash
pip3 install ultralytics sentence-transformers faiss-cpu groq opencv-python numpy scipy
```

### 3. Set Up Environment Variables

```bash
cp .env.example .env
# Edit .env and add your Groq API key:
# GROQ_API_KEY=gsk_xxxxxxxxxxxxxxxxxxxx
```

Get a free API key at: https://console.groq.com

### 4. Build the Workspace

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Running the System

Open **4 separate terminals**:

**Terminal 1 — Gazebo Simulation**
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch arena.launch.py
```

**Terminal 2 — Nav2 Navigation Stack**
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch nav2_minimal.launch.py
```

**Terminal 3 — AEMS Core System**
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch adaptive_episodic_memory aems.launch.py
```

**Web Dashboard** → Open **http://localhost:8080** in your browser

---

## Web Dashboard

| Panel | Description |
|---|---|
| **Live Camera HUD** | 30 FPS annotated camera feed with YOLO bounding boxes and disambiguated class labels |
| **Robot Telemetry** | Real-time pose (X, Y, Yaw), task phase, progress bar |
| **Memory Table** | All active object sightings — class, position, confidence, corroboration count |
| **Command Interface** | Natural language input: *"go to couch"*, *"find the chair"*, *"navigate to vehicle bay"* |
| **Task Log** | History of every command, strategy chosen, and outcome |

---

## Arena Layout

The simulation arena is a **20m × 20m** indoor environment:

| Object | Position | COCO Class |
|---|---|---|
| Sofa / Couch | (0, -7) | `couch` |
| Dining Chair | (-3, -6) | `chair` |
| Dining Table | (-8, -2) | `dining table` |
| Fire Hydrant | (8, 0) | `fire hydrant` |
| Standing Person | (7, 7) | `person` |
| SUV / Car | (-7, 7) | `car` |

Interior partition walls create occlusion zones to test the perimeter scouting pipeline.

---

## Utility Scripts

```bash
# Move an object in Gazebo at runtime (for demos)
python3 move_object.py chair -2.0 -5.0        # move chair to new position
python3 move_object.py couch 1.5 -5.5         # move couch
python3 move_object.py person 5.0 5.0         # move person

# Reset the memory database to ground truth
python3 reset_memory_db.py

# Run benchmarks
python3 benchmarks/benchmark_retrieval_10k.py
python3 benchmarks/run_baseline_comparison.py

# Run all integration tests
source /opt/ros/jazzy/setup.bash && source install/setup.bash
pytest src/adaptive_episodic_memory/test/test_aems_integration.py -v
# Expected: 24 passed
```

---

## Integration Tests (24/24 Passing)

| Test Group | Tests | Coverage |
|---|---|---|
| `TestStep1Schema` | 3 | DB schema: robot_pos, task_info, scene_description, embedding |
| `TestStep2Retrieval` | 4 | FAISS retrieval correctness, latency, 10k benchmark verified |
| `TestStep3NLQuery` | 2 | LLM reasoner import, query answering |
| `TestStep4TaskHistory` | 2 | Task history insert/retrieve, outcome recording |
| `TestStep5NamedZones` | 3 | zones.yaml exists, required fields, zone resolution |
| `TestStep6Safety` | 6 | Input sanitize (valid, semicolon, too-long, SQL injection), goal bounding |
| `TestStep7LoggingBackup` | 4 | system_events logging, backup/restore, retrieval enforcement |

---

## Named Zones

| Zone | Aliases |
|---|---|
| `vehicle_bay` | "garage", "car bay", "parking area", "northwest zone" |
| `reception_area` | "lobby", "waiting area", "northeast zone" |
| `east_corridor` | "hydrant station", "emergency corridor", "east hallway" |
| `seating_area` | "lounge", "couch area", "south lounge" |
| `dining_area` | "cafeteria", "conference table", "mess hall" |
| `study_alcove` | "reading nook", "chair area", "office nook" |

---

## ROS 2 Topics & Services

| Topic / Service | Type | Direction | Description |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Sub | Raw RGB camera feed |
| `/scan` | `sensor_msgs/LaserScan` | Sub | 2D LiDAR scan |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Sub | Localized robot pose |
| `/aems/detections` | `aems_interfaces/ObjectDetection` | Pub | Fused object detections |
| `/aems/annotated_image` | `sensor_msgs/Image` | Pub | 30 FPS HUD overlay image |
| `/aems/task_status` | `std_msgs/String` | Pub | JSON task stage + progress |
| `/aems/response` | `std_msgs/String` | Pub | JSON LLM/action response |
| `/aems/command` | `std_msgs/String` | Sub | Natural language commands |
| `/navigate_to_pose` | Nav2 Action | Client | Goal dispatch to Nav2 |

---

## Documentation

All detailed docs are in the [`docs/`](docs/) directory:

- [`architecture.md`](docs/architecture.md) — 4-layer system architecture with diagrams
- [`SRS_v1.1_changes.md`](docs/SRS_v1.1_changes.md) — Software Requirements Specification
- [`decision_theoretic_cost_formulation.md`](docs/decision_theoretic_cost_formulation.md) — Full cost model derivation
- [`api_reference.md`](docs/api_reference.md) — ROS 2 topics, services, message types
- [`demo_guide.md`](docs/demo_guide.md) — Step-by-step demo walkthrough
- [`user_manual.md`](docs/user_manual.md) — End-user manual
- [`troubleshooting.md`](docs/troubleshooting.md) — Common issues and fixes

---

## Environment Variables

```env
GROQ_API_KEY=gsk_xxxxxxxxxxxxxxxxxxxx   # Required for LLM NL queries
```

Copy `.env.example` to `.env` and fill in your key. The `.env` file is git-ignored.

---

## Dependencies

| Dependency | Version | Purpose |
|---|---|---|
| ROS 2 Jazzy | latest | Core robotics middleware |
| Gazebo Harmonic | latest | Physics simulation |
| YOLOv8 (ultralytics) | 8.4.129 | Real-time object detection |
| sentence-transformers | 6.1.0 | `all-MiniLM-L6-v2` embeddings |
| faiss-cpu | 1.15.1 | Vector similarity search index |
| OpenCV | 4.x | Camera image processing |
| SQLite3 | built-in | Persistent episodic memory store |
| Groq SDK | latest | Cloud LLM inference (free tier) |
| Nav2 | Jazzy | Autonomous navigation stack |

---

## License

Apache License 2.0 — see [LICENSE](src/adaptive_episodic_memory/LICENSE)

---

## Author

**DSC** · Branch: `feature/srs-v1.1-implementation`
