# AEMS Troubleshooting Guide

## 1. Common Issues & Solutions

### 1.1 Groq API Returns 404
- **Symptom:** Logs show `groq.NotFoundError: Error code: 404` or `model not found`.
- **Cause:** Incorrect or deprecated model name specified (e.g. `qwen/qwen3-32b`).
- **Solution:** Verify `adaptive_episodic_memory/reasoning/llm_reasoning.py` specifies `qwen/qwen3.8-27b`. Verify that `GROQ_API_KEY` is exported in `~/.env`.

### 1.2 HF Hub Rate Limit Warning
- **Symptom:** Pytest or node startup shows:
  ```text
  Warning: You are sending unauthenticated requests to the HF Hub.
  ```
- **Explanation:** This is a non-fatal informational notice from `sentence-transformers` when downloading or verifying cached weights without an explicit `HF_TOKEN`. Cached weights load locally in ~0.5s.
- **Solution (Optional):** Export `HF_TOKEN="your_token"` in `~/.env`.

### 1.3 Nav2 Action Server Rejection
- **Symptom:** `Nav2 rejected goal for <class>` or robot does not move.
- **Cause:** The computed standoff coordinate falls inside an obstacle, inside an interior wall, or outside the arena boundary.
- **Solution:** Verify the `ARENA_BOUNDS` clamping in `dashboard_node.py` is active. Check that `compute_standoff` successfully validated ray clearance against `INTERIOR_WALLS`.

### 1.4 Web Dashboard Disconnected
- **Symptom:** Dashboard UI shows `Status: Disconnected (Port 9090)`.
- **Cause:** `aems_web_bridge.py` is not running, or another process is occupying port 9090.
- **Solution:**
  ```bash
  # Check if port 9090 is in use
  lsof -i :9090
  # Ensure the bridge is running
  ros2 run adaptive_episodic_memory aems_web_bridge
  ```

### 1.5 Database Lock Errors
- **Symptom:** `sqlite3.OperationalError: database is locked`.
- **Cause:** Multiple nodes attempting concurrent exclusive transactions.
- **Solution:** AEMS uses SQLite WAL mode (`PRAGMA journal_mode=WAL`) and `check_same_thread=False` with read-only cursor pooling. Backups should only be executed using `backup_restore_db.py` (which uses SQLite's hot backup API).

---

## 2. Diagnostics Commands

Check active ROS 2 nodes:
```bash
ros2 node list
```

Check detection stream:
```bash
ros2 topic echo /aems/detections --once
```

Inspect recent task history:
```bash
sqlite3 ~/aems_ws/aems_memory.db "SELECT task_id, command, outcome, duration FROM task_history ORDER BY task_id DESC LIMIT 5;"
```

Inspect recent system events:
```bash
sqlite3 ~/aems_ws/aems_memory.db "SELECT id, event, detail, datetime(timestamp, 'unixepoch') FROM system_events ORDER BY id DESC LIMIT 5;"
```
