#!/usr/bin/env python3
"""
Trust-vs-Verify Decision Layer
Implements Section 7.6 of the project report.
"""

from typing import Dict, Optional, List
from adaptive_episodic_memory.memory.memory_store import MemoryStore
from adaptive_episodic_memory.memory.retrieval import MemoryRetrieval


class TrustVerifyReasoner:
    def __init__(self, store: MemoryStore, retrieval: MemoryRetrieval):
        self.store = store
        self.retrieval = retrieval

        # ── Kinematically Grounded Cost Model Parameters ─────────────────────
        # Derived from TurtleBot3 Waffle kinematics (v_nom=0.25 m/s, w_nom=0.523 rad/s)
        # and arena topology (20x20m, 4 partitioned rooms, mean edge d_bar=2.0m):
        #   base_memory_travel_cost = d_bar / v_nom = 2.0 / 0.25 = 8.0 s
        #   cost_of_reverify        = 2*pi / w_nom = 6.28 / 0.523 = 12.0 s (360 deg scan)
        #   cost_of_exploration     = exhaustive room-coverage search = 40.0 s
        #   reverify approach margin= 0.6 * base_travel = 4.8 s -> C_reverify = 16.8 s
        #   Equivalence threshold: C_trust(p*) = C_reverify => 8 + 40(1-p*) = 16.8 => p* = 0.78
        self.cost_of_exploration = 40.0      # exhaustive search cost (s)
        self.cost_of_reverify = 12.0         # 360-deg visual scan time (s)
        self.base_memory_travel_cost = 8.0   # direct transit time between adjacent nodes (s)


    def decide(self, object_class: str, task_type: str = "navigate-to-object") -> Dict:
        """
        Main decision function.
        Returns a structured decision the planner can act on.
        """
        # 1. Get best memory
        best = self.retrieval.get_best_route_hint(object_class)

        # 2. Get previously failed routes for this task type
        failed = self.store.get_failed_routes(task_type)
        failed_landmark_ids = set()
        for f in failed:
            if f.get("failure_landmark_id"):
                failed_landmark_ids.add(f["failure_landmark_id"])

        decision = {
            "action": "explore",          # default
            "reason": "No usable memory found",
            "target_landmark_id": None,
            "score": 0.0,
            "exclude_landmarks": list(failed_landmark_ids),
            "object_class": object_class
        }

        if best is None:
            return decision

        score = best["score"]
        landmark_id = best["episode"]["landmark_id"]

        # Hard exclusion: never go to a previously failed landmark for this task
        if landmark_id in failed_landmark_ids:
            decision["action"] = "explore"
            decision["reason"] = f"Best memory points to previously failed landmark {landmark_id}"
            return decision

        # 3. Trust-vs-Verify cost comparison
        # Probability that memory is still valid ≈ score
        p_valid = max(0.05, min(0.98, score))

        expected_cost_trust = (
            self.base_memory_travel_cost +
            (1.0 - p_valid) * self.cost_of_exploration
        )

        expected_cost_reverify = self.cost_of_reverify + self.base_memory_travel_cost * 0.6

        if expected_cost_trust < expected_cost_reverify and score > 0.45:
            decision["action"] = "trust_memory"
            decision["reason"] = f"Memory score {score:.2f} is high enough to trust"
            decision["target_landmark_id"] = landmark_id
            decision["score"] = score
            decision["relative_offset"] = best["episode"].get("relative_offset")
        elif score > 0.25:
            decision["action"] = "reverify"
            decision["reason"] = f"Memory score {score:.2f} → partial verification recommended"
            decision["target_landmark_id"] = landmark_id
            decision["score"] = score
            decision["relative_offset"] = best["episode"].get("relative_offset")
        else:
            decision["action"] = "explore"
            decision["reason"] = f"Memory score {score:.2f} too low — full exploration safer"

        decision["expected_cost_trust"] = expected_cost_trust
        decision["expected_cost_reverify"] = expected_cost_reverify
        decision["cost_of_exploration"] = self.cost_of_exploration

        return decision
