# AEMS Documentation Index

Welcome to the documentation suite for the **Adaptive Episodic Memory System (AEMS)**.

This documentation conforms to SRS Section 2.6 and covers the complete system architecture, deployment, operation, and maintenance.

---

## Documentation Directory

| Document | Description |
|---|---|
| [Installation Guide](installation_guide.md) | System requirements, ROS 2 Jazzy, Gazebo Harmonic, and Python dependency setup. |
| [User Manual](user_manual.md) | System launch, web dashboard overview, query modes, zone navigation, and safety operations. |
| [Architecture Document](architecture.md) | High-level system architecture, perception, memory store, cost formulas, and navigation pipelines. |
| [API Reference](api_reference.md) | Specifications for ROS 2 topics, services, actions, custom messages, and WebSocket protocols. |
| [Developer Guide](developer_guide.md) | Codebase organization, coding standards, working rules, unit tests, and database maintenance. |
| [Demo Guide](demo_guide.md) | Step-by-step walkthrough of the end-to-end evaluation demo sequence. |
| [Troubleshooting Guide](troubleshooting.md) | Solutions for common errors (API issues, Nav2 rejections, port conflicts, database locks). |
| [SRS v1.1 Changes](SRS_v1.1_changes.md) | Formal SRS revision history, TBD resolutions, documented environment deviations, and FR-23+ additions. |

---

## Quick Start
```bash
# 1. Source environment
source /home/dsc/aems_ws/aems_env.sh

# 2. Run unit & integration test suite
python3 -m pytest /home/dsc/aems_ws/src/adaptive_episodic_memory/test/test_aems_integration.py -v

# 3. Launch full system simulation
ros2 launch adaptive_episodic_memory aems.launch.py
```
