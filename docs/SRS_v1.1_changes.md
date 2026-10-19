# AEMS Software Requirements Specification (SRS) — Version 1.1 Change Document

**Document Identifier:** AEMS-SRS-CR-v1.1  
**Project:** Adaptive Episodic Memory System (AEMS) for Autonomous Mobile Robots  
**Base Document:** SRS v1.0 (`24BRS1146_Shri Chaathura_SRS.pdf`)  
**Target Document:** SRS v1.1  
**Author:** Shri Chaathura (24BRS1146)  
**Date:** October 17, 2026  

---

## 1. Revision History

| Version | Date | Author | Primary Changes / Rationale |
|---|---|---|---|
| **v1.0** | Initial Release | Shri Chaathura | Initial baseline specification defining FR-1 through FR-22, NFRs, preliminary operational architecture, and open TBD placeholders. |
| **v1.1** | October 17, 2026 | Shri Chaathura | Resolved all TBD items (TBD-1 through TBD-5); documented architectural deviations (Jazzy/Harmonic, Groq cloud LLM, perception-stage scene understanding); added FR-23 through FR-29 specifying sensor fusion, corroboration, trust/verify cost math, strategy selection, bounded search, and perimeter scouting; corrected typographical and formatting defects. |

---

## 2. Resolutions for Specification Placeholders (TBD-1 to TBD-5)

### TBD-1: Large Language Model Engine
- **SRS v1.0 Status:** Open placeholder (`TBD-1: LLM backend architecture and inference host`).
- **SRS v1.1 Resolution:** **Groq Cloud API hosting Qwen 2.5 / Qwen 3.8 (`qwen/qwen3.8-27b`)**
- **Technical Rationale:** Local execution of a 7B–14B LLM on mobile robotic hardware (AMD Ryzen 5 5500U, integrated GPU, 13.5 GB RAM shared) incurs 4–8 second inference latencies and starves the real-time Nav2 and Gazebo physics threads. Offloading natural language explanation and memory grounding to Groq's LPUs achieves token delivery in < 350 ms, preserving onboard compute exclusively for real-time control, LiDAR perception, and SLAM.
- **Fallbacks:** In the event of API timeout or network isolation, rule-based deterministic templates in `llm_reasoning.py` ground the output without blocking the robot.

### TBD-2: Robotic Simulation Platform
- **SRS v1.0 Status:** Open placeholder (`TBD-2: Hardware embodiment specification`).
- **SRS v1.1 Resolution:** **TurtleBot3 Waffle**
- **Technical Rationale:** The Waffle configuration provides the required payload and sensor mount topology: a 360° 2D LiDAR (LDS-01/02), a forward-facing RGB camera, wheel encoders, and differential drive kinematics matching standard indoor service robotics benchmarks.

### TBD-3: Object Detection Perceptual Model
- **SRS v1.0 Status:** Open placeholder (`TBD-3: Vision backbone and weights`).
- **SRS v1.1 Resolution:** **Ultralytics YOLOv8s (Small)**
- **Technical Rationale:** YOLOv8s achieves 44.9 mAP on COCO while running at > 25 FPS on CPU/iGPU. The model reliably detects target household and office classes: `couch/sofa`, `chair`, `dining table`, `car/suv`, `person`, and `fire hydrant`.

### TBD-4: Episodic Database & Vector Indexing Backend
- **SRS v1.0 Status:** Open placeholder (`TBD-4: Storage engine and indexing structure`).
- **SRS v1.1 Resolution:** **SQLite 3 (WAL Mode) + Meta FAISS (`IndexFlatIP`)**
- **Technical Rationale:** Relational metadata (timestamps, poses, task outcomes, foreign keys) requires ACID transactional integrity, supplied by SQLite. Dense vector search over 384-dimensional `sentence-transformers/all-MiniLM-L6-v2` embeddings requires high-throughput cosine similarity, provided by FAISS `IndexFlatIP` over L2-normalized vectors.

### TBD-5: Empirical Retrieval Performance Benchmarks
- **SRS v1.0 Status:** Target latency placeholder (`TBD-5: Retrieval latency under 10k items`).
- **SRS v1.1 Resolution:** **12.65 ms Mean Latency across 10,000 Records (Target < 1,000 ms)**
- **Empirically Verified Values (AMD Ryzen 5 5500U, 12 threads, 13.5 GB RAM):**
  - Record Count: 10,000 synthetic episodic sightings
  - Vector Dimension: 384 (`all-MiniLM-L6-v2`)
  - Index Type: `faiss.IndexFlatIP`
  - Query Embedding Latency: 11.49 ms
  - FAISS Nearest Neighbor Search: 0.97 ms
  - Multi-Criteria Scoring (Recency × Conf × Corr): 0.18 ms
  - Total End-to-End Latency: **12.65 ms** (95th percentile: 14.64 ms, 99th percentile: 15.41 ms)
  - Throughput: **79.1 queries/second**

---

## 3. Documented Technical Deviations

### Deviation 1: Operating System & ROS Middleware
- **SRS v1.0 Statement:** Ubuntu 22.04 LTS with ROS 2 Humble Hawksbill and Gazebo Classic 11.
- **Actual Implementation:** **Ubuntu 24.04 LTS (Noble Numbat) with ROS 2 Jazzy Jalisco and Gazebo Harmonic (gz-sim 8)**.
- **Justification:** Ubuntu 24.04 and ROS 2 Jazzy are the contemporary Tier-1 Long Term Support distributions supported through 2029. Gazebo Classic has reached official End-of-Life (EOL). Migrating to Gazebo Harmonic and `ros_gz_bridge` ensures long-term reproducibility and access to modern SDF 1.10 physics engines.
- **Traceability Impact:** All node interfaces, topics, message types, and action protocols maintain 100% functional equivalence with the Humble specification.

### Deviation 2: LLM Hosting Modality
- **SRS v1.0 Statement:** Local Hugging Face transformer model running onboard.
- **Actual Implementation:** **Cloud-hosted Groq LPUs (`qwen/qwen3.8-27b`) with graceful local fallback**.
- **Justification:** Preserves CPU and memory bandwidth for the real-time control loop and 20 Hz AMCL localization. Local rule-based fallback guarantees zero-latency operation during network disruption.

### Deviation 3: Scene Understanding Architectural Placement
- **SRS v1.0 Statement:** Scene understanding placed downstream inside the reasoning layer.
- **Actual Implementation:** **Scene understanding integrated into the perception pipeline (`yolo_perception_node.py` and `landmark_manager_node.py`)**.
- **Justification:** Deriving relative spatial vectors (e.g., distance, bearing, nearest landmark relationship) at the moment of sensor fusion captures exact temporal alignment between the camera frame and LiDAR scan before state drift occurs.

---

## 4. New Functional Requirements (FR-23 to FR-29)

### FR-23: Sensor Fusion 3D Metric Localization
- **Description:** The system shall fuse 2D bounding box azimuth bearings from camera RGB frames with minimum radial returns from the 2D planar LiDAR within the bounding cone to calculate 3D Cartesian coordinates $(x, y)$ of detected objects in the global `map` frame.
- **Verification:** Compare camera-LiDAR fused coordinate against Gazebo Ground Truth; error must be $< 0.35\text{ m}$.

### FR-24: Corroboration & Observation Reinforcement
- **Description:** When an object of class $C$ is observed within spatial threshold $d \le 1.0\text{ m}$ of an existing active sighting, the system shall not duplicate records. It shall increment `corroboration_count`, update `confidence = max(conf_old, conf_new)`, and refresh the `timestamp`.
- **Verification:** Unit test `TestStep4TaskHistory` and runtime inspection of `corroboration_count` in `object_sightings`.

### FR-25: Conflict Resolution & Supersede Policy
- **Description:** When an object of class $C$ is detected at a new location exceeding distance $d > 2.0\text{ m}$ from its previously recorded active sighting, the system shall mark the existing sighting status as `'superseded'` (`superseded = 1`) and insert a new active record with `corroboration_count = 1`.
- **Verification:** Move object in Gazebo simulation; confirm old database record has `superseded = 1` and new record is active.

### FR-26: Trust-vs-Verify Mathematical Cost Formulations
- **Description:** The system shall determine whether to navigate directly to an object's recorded position or perform reverification using the mathematical formulation:
  $$p_{valid} = \text{confidence} \times e^{-\lambda \cdot \Delta t} \times \min(1.0, 0.5 + 0.25 \times (\text{corroboration} - 1))$$
  $$C_{trust} = 8.0 + (1.0 - p_{valid}) \times 40.0$$
  $$C_{reverify} = 12.0 + 8.0 \times 0.6$$
  $$C_{explore} = 40.0$$
  where $\lambda = \ln(2) / 86400$.
- **Verification:** Unit test verifying exact numerical equality of cost evaluations for variable age and corroboration values.

### FR-27: Navigation Strategy Selection
- **Description:** If confidence score $< 0.65$, corroboration $< 2$, or prior navigation failure records exist within 600 seconds, the system shall select **Landmark Route Replay**; otherwise, it shall execute **Direct Navigation**. If route replay exceeds 45 seconds or fails, it shall fall back cleanly to direct navigation.
- **Verification:** Trigger route replay with mock failure; confirm fallback to direct navigation in $< 2.0\text{ s}$.

### FR-28: Bounded Nearby-Landmark Search
- **Description:** When planning a route or verifying an object, the system shall constrain graph searches to topological landmarks within radius $R \le 8.0\text{ m}$ of the estimated object location.
- **Verification:** Inspect graph query logs; ensure remote landmarks outside $8.0\text{ m}$ are excluded from path formulation.

### FR-29: Autonomous Multi-Room Perimeter Scout
- **Description:** If the target object is occluded behind interior partition walls upon arrival at the primary standoff, the system shall compute sequential perimeter waypoints around wall bounding boxes (`iw_1` through `iw_4`) and navigate to vantage points until visual confirmation occurs or waypoints are exhausted.
- **Verification:** Place target object behind vertical wall `iw_2`; issue navigation command; confirm robot circles wall corner to establish direct visual line-of-sight.

---

## 5. Typographical & Formatting Defects Corrected from SRS v1.0

1. **Duplicate Section Header:** In Section 5 (Non-Functional Requirements), the header `5.3 Reliability Requirements` appeared twice consecutively. The second instance has been corrected to `5.4 Maintainability & Supportability Requirements`.
2. **Truncated Subsection 2.7:** Section 2.7 ("User Documentation") ended abruptly in mid-sentence in v1.0. Corrected in v1.1 to explicitly enumerate all 8 required documentation deliverables.
3. **Typo in Section 3 Heading:** The string `"SSoftware System Attributes"` in Section 3.1 was corrected to `"Software System Attributes"`.
4. **Typo in Section 7 Placeholder:** The string `"TBD3"` in Section 7.2 table was standardized to `"TBD-3"`.
