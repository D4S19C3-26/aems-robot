# AEMS Developer Guide

## 1. Project Organization

```text
aems_ws/
├── aems_env.sh                          # Master environment setup script
├── aems_memory.db                       # Primary SQLite episodic memory database
├── benchmarks/                          # Hardware-verified benchmark scripts & results
│   ├── benchmark_retrieval_10k.py
│   └── retrieval_benchmark_results.json
├── dashboard/                           # HTML5/JS dashboard frontend
│   └── index.html
├── docs/                                # Project documentation
├── models/                              # Gazebo Fuel object models & SDFs
├── src/
│   ├── aems_interfaces/                 # Custom ROS 2 msg & srv definitions
│   └── adaptive_episodic_memory/        # Core AEMS ROS 2 Python package
│       ├── config/
│       │   └── zones.yaml               # Named arena zones & bounding coordinates
│       ├── scripts/
│       │   └── backup_restore_db.py     # Hot SQLite backup & restore CLI
│       ├── test/
│       │   └── test_aems_integration.py # 24-point pytest integration test suite
│       └── adaptive_episodic_memory/
│           ├── memory/
│           │   ├── memory_store.py      # SQLite table definitions & helpers
│           │   ├── retrieval.py         # FAISS vector search & scoring
│           │   ├── landmark_manager_node.py
│           │   └── route_replayer_node.py
│           ├── perception/
│           │   └── yolo_perception_node.py
│           ├── reasoning/
│           │   ├── dashboard_node.py    # Main command dispatcher & scout node
│           │   ├── llm_reasoning.py     # Groq LLM reasoning interface
│           │   └── trust_verify.py      # Cost math & decision formulations
│           └── utils/
│               └── aems_web_bridge.py   # WebSocket bridge server
└── worlds/
    └── arena.sdf                        # 20x20m simulation world with interior walls
```

---

## 2. Coding Standards & Working Rules

1. **Strict Cost Formulation Protection:**
   The cost formulas in `trust_verify.py` and `dashboard_node.py` are strictly protected:
   - `cost_trust = 8.0 + (1.0 - p_valid) * 40.0`
   - `cost_reverify = 12.0 + 8.0 * 0.6`
   Do not alter these values or formulations without an explicit architecture review.
2. **Camera-LiDAR Fusion & Route Replay:**
   Camera-LiDAR sensor fusion and route replay fallback routines must remain protected.
3. **No Hardcoded Secrets:**
   Never commit API tokens or credentials into source code. Always reference `os.environ.get('GROQ_API_KEY')` with graceful fallbacks.
4. **Input Sanitization:**
   All user strings entering the system must pass through `_sanitize_command()` in `dashboard_node.py`.

---

## 3. Running Unit & Integration Tests

Run the full pytest suite:
```bash
source /home/dsc/aems_ws/aems_env.sh
python3 -m pytest src/adaptive_episodic_memory/test/test_aems_integration.py -v
```

Run code style & linter checks:
```bash
colcon test --packages-select adaptive_episodic_memory
colcon test-result --verbose
```

---

## 4. Database Maintenance & Backups

Hot backups can be taken while the robot is active:
```bash
# Create an immediate hot backup
python3 src/adaptive_episodic_memory/scripts/backup_restore_db.py backup

# List all available backups
python3 src/adaptive_episodic_memory/scripts/backup_restore_db.py list

# Restore from a backup (creates a safety snapshot before overwriting)
python3 src/adaptive_episodic_memory/scripts/backup_restore_db.py restore --input ~/aems_ws/backups/aems_memory_latest.db --confirm
```
