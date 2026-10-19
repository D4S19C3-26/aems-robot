#!/usr/bin/env python3
"""
Route Replayer Node — Phase 3 core.

Provides /replay_route service. Given a target object class:
  1. Finds the landmark in the DB.
  2. Finds an A* path from nearest known landmark to the target.
  3. Navigates leg-by-leg through Nav2.
  4. At each intermediate landmark, runs perceptual re-confirmation:
       - Waits 5s for YOLO detection.
       - If not seen: rotates ±90° left then right.
       - If still not seen: decrement landmark confidence, count miss.
       - If 3 consecutive misses: abort route (D2 escalation rule).
  5. If graph is too sparse for a path: falls back to Phase 2 direct
     coordinate navigation and logs a GRAPH_FALLBACK event (D3).

Key fixes vs. previous version:
  - _nav_to() no longer spin-blocks the ROS executor; uses
    executor.spin_until_future_complete() so action-client futures resolve.
  - edges list is fetched once and reused safely (no cursor exhaustion).
  - A* replaces Dijkstra (heuristic prunes search on sparse graphs quickly).
  - auto-connect nearby landmarks when the graph has no edges for the
    robot's start node (prevents always falling through to direct fallback).
  - Detailed per-hop logging for easier debugging.
"""

import math
import heapq
import sqlite3
import os
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from aems_interfaces.msg import ObjectDetection
from aems_interfaces.srv import ReplayRoute

DB_PATH = os.path.expanduser('~/aems_ws/aems_memory.db')

ALIASES = {
    'sofa': 'couch', 'couch': 'couch', 'bench': 'couch',
    'suv': 'car', 'car': 'car', 'truck': 'car',
    'table': 'dining table', 'dining table': 'dining table',
    'chair': 'chair',
    'person': 'person', 'human': 'person',
    'fire hydrant': 'fire hydrant', 'hydrant': 'fire hydrant',
}

CONFIRM_WAIT_SECS      = 4.0    # seconds to listen for re-confirmation at each landmark
ROTATE_SPEED           = 0.4    # rad/s for re-confirmation rotation
ROTATE_DURATION        = 1.8    # seconds per ~60° sweep
CONSECUTIVE_MISS_ABORT = 3     # D2: abort after this many consecutive missed confirmations
NAV_ACCEPT_TIMEOUT     = 20.0  # seconds to wait for Nav2 to accept a goal
NAV_RESULT_TIMEOUT     = 120.0 # seconds to wait for Nav2 to reach a goal
AUTO_CONNECT_RADIUS    = 8.0   # metres — synthesise edges for disconnected starts


# ──────────────────────────────────────────────────────────────────────────────
# A* path-finder (replaces Dijkstra)
# ──────────────────────────────────────────────────────────────────────────────

def _astar(landmarks: dict, adj: dict, start_id: int, goal_id: int):
    """Return list of landmark IDs start→goal, or None if unreachable."""
    gx = landmarks[goal_id]['pos_x']
    gy = landmarks[goal_id]['pos_y']

    def h(lid):
        lm = landmarks[lid]
        return math.hypot(gx - lm['pos_x'], gy - lm['pos_y'])

    g_score = {lid: float('inf') for lid in landmarks}
    g_score[start_id] = 0.0
    prev = {}
    pq = [(h(start_id), 0.0, start_id)]  # (f, g, id)

    while pq:
        f, g, u = heapq.heappop(pq)
        if u == goal_id:
            break
        if g > g_score[u]:
            continue
        for w, v in adj.get(u, []):
            ng = g + w
            if ng < g_score[v]:
                g_score[v] = ng
                prev[v] = u
                heapq.heappush(pq, (ng + h(v), ng, v))

    if g_score[goal_id] == float('inf'):
        return None

    path, cur = [], goal_id
    while cur != start_id:
        path.append(cur)
        cur = prev[cur]
    path.append(start_id)
    path.reverse()
    return path


def _build_adjacency(landmarks: dict, edges: list) -> dict:
    """Build a bidirectional adjacency list, de-duplicating DB rows."""
    adj = {lid: [] for lid in landmarks}
    seen = set()
    for e in edges:
        fid, tid, dist = e['from_id'], e['to_id'], e['distance']
        key = (min(fid, tid), max(fid, tid))
        if key not in seen:
            seen.add(key)
            if fid in adj and tid in adj:
                adj[fid].append((dist, tid))
                adj[tid].append((dist, fid))
    return adj


def _auto_connect(landmarks: dict, adj: dict, node_id: int, radius: float):
    """Synthesise in-memory edges if node_id is isolated in the graph."""
    if adj.get(node_id):
        return
    lm = landmarks[node_id]
    for lid, other in landmarks.items():
        if lid == node_id:
            continue
        d = math.hypot(lm['pos_x'] - other['pos_x'], lm['pos_y'] - other['pos_y'])
        if d <= radius:
            adj[node_id].append((d, lid))
            adj[lid].append((d, node_id))


class RouteReplayerNode(Node):
    def __init__(self):
        super().__init__('route_replayer')

        self.cb_group = ReentrantCallbackGroup()

        self.conn = sqlite3.connect(DB_PATH, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        self.cursor = self.conn.cursor()
        self._db_lock = threading.Lock()

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        # Executor reference — set by main() after construction so _nav_to
        # can call spin_until_future_complete() without blocking the event loop.
        self._executor: MultiThreadedExecutor | None = None

        # Track robot pose for distance cost calculations
        self.robot_x = 0.0
        self.robot_y = 7.0
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._pose_cb, 10,
            callback_group=self.cb_group)

        self._route_lock = threading.Lock()

        # Re-confirmation detection tracking
        self._confirm_lock = threading.Lock()
        self._confirm_class: str | None = None
        self._confirm_detected = False
        self._det_sub = self.create_subscription(
            ObjectDetection, '/aems/detections', self._detection_cb, 10,
            callback_group=self.cb_group)

        self._srv = self.create_service(
            ReplayRoute, '/replay_route', self._replay_cb,
            callback_group=self.cb_group)

        self.get_logger().info('Route Replayer ready. Call /replay_route to navigate.')

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    # ─────────────────────── GRAPH HELPERS ────────────────────────────────────

    def _get_all_landmarks(self) -> dict:
        with self._db_lock:
            self.cursor.execute('SELECT * FROM landmarks')
            return {r['id']: dict(r) for r in self.cursor.fetchall()}

    def _get_valid_edges(self, task_type: str) -> list:
        """Return all edges as a plain list[dict], minus recently-failed pairs."""
        with self._db_lock:
            self.cursor.execute('SELECT * FROM landmark_edges')
            edges = [dict(r) for r in self.cursor.fetchall()]   # materialise immediately

            now_ts = self.get_clock().now().nanoseconds * 1e-9
            self.cursor.execute('''
                SELECT start_landmark_id, end_landmark_id, COUNT(*) as fail_count
                FROM task_attempts
                WHERE task_type = ? AND outcome = 'FAILURE' AND timestamp > ?
                GROUP BY start_landmark_id, end_landmark_id
                HAVING fail_count >= 2
            ''', (task_type, now_ts - 60.0))
            failed_edges = {(r['start_landmark_id'], r['end_landmark_id'])
                            for r in self.cursor.fetchall()}

        valid = []
        for e in edges:
            if ((e['from_id'], e['to_id']) in failed_edges
                    or (e['to_id'], e['from_id']) in failed_edges):
                self.get_logger().warn(
                    f"Skipping recently failed edge [{e['from_id']}]<->[{e['to_id']}]")
                continue
            valid.append(e)
        return valid

    def _nearest_landmark(self, landmarks: dict, rx: float, ry: float):
        """Find landmark closest to the robot's current position."""
        best_id, best_dist = None, float('inf')
        for lid, lm in landmarks.items():
            d = math.hypot(rx - lm['pos_x'], ry - lm['pos_y'])
            if d < best_dist:
                best_dist = d
                best_id = lid
        return best_id

    def _log_event(self, event: str, detail: str = ''):
        with self._db_lock:
            self.cursor.execute(
                'INSERT INTO system_events (event, detail, timestamp) VALUES (?,?,?)',
                (event, detail, self.get_clock().now().nanoseconds * 1e-9))
            self.conn.commit()

    def _log_task_attempt(self, task_type: str, target_class: str,
                          start_id: int, end_id: int, outcome: str, reason: str):
        with self._db_lock:
            self.cursor.execute('''
                INSERT INTO task_attempts
                (task_type, target_class, start_landmark_id, end_landmark_id,
                 outcome, reason, timestamp)
                VALUES (?, ?, ?, ?, ?, ?, ?)
            ''', (task_type, target_class, start_id, end_id, outcome, reason,
                  self.get_clock().now().nanoseconds * 1e-9))
            self.conn.commit()
        self.get_logger().info(
            f'[ATTEMPT] {task_type} → [{end_id}] {target_class}: {outcome} ({reason})')

    # ──────────────────────── NAVIGATION ──────────────────────────────────────

    def _nav_to(self, x: float, y: float, yaw: float = None) -> bool:
        """
        Send a NavigateToPose goal and block until it completes.

        CRITICAL FIX: The old code used time.sleep() busy-wait, which
        prevented the ROS executor from processing incoming messages, so
        action-client futures never resolved → permanent timeout.
        We now use executor.spin_until_future_complete() so the event loop
        keeps running while we wait.
        """
        if not self._nav_client.wait_for_server(timeout_sec=NAV_ACCEPT_TIMEOUT):
            self.get_logger().error('Nav2 action server not available.')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        if yaw is not None:
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'[NAV] Sending goal → ({x:.2f}, {y:.2f})')

        # Step 1 — wait for goal acceptance
        send_future = self._nav_client.send_goal_async(goal)
        if self._executor:
            self._executor.spin_until_future_complete(
                send_future, timeout_sec=NAV_ACCEPT_TIMEOUT)
        else:
            deadline = time.time() + NAV_ACCEPT_TIMEOUT
            while not send_future.done() and time.time() < deadline:
                time.sleep(0.05)

        if not send_future.done():
            self.get_logger().error('Timed out waiting for Nav2 to accept goal.')
            return False

        handle = send_future.result()
        if not handle or not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal.')
            return False

        self.get_logger().info('[NAV] Goal accepted — waiting for result …')

        # Step 2 — wait for navigation result
        result_future = handle.get_result_async()
        if self._executor:
            self._executor.spin_until_future_complete(
                result_future, timeout_sec=NAV_RESULT_TIMEOUT)
        else:
            deadline = time.time() + NAV_RESULT_TIMEOUT
            while not result_future.done() and time.time() < deadline:
                time.sleep(0.1)

        if not result_future.done():
            self.get_logger().error('Timed out waiting for Nav2 navigation result.')
            return False

        result = result_future.result()
        status = result.status
        ok = (status == GoalStatus.STATUS_SUCCEEDED or status == 4)
        self.get_logger().info(f'[NAV] Result status={status} → {"OK" if ok else "FAIL"}')
        return ok


    # ──────────────────────── RE-CONFIRMATION ─────────────────────────────────

    def _detection_cb(self, msg: ObjectDetection):
        raw_cls = msg.class_name.lower().strip()
        canonical = ALIASES.get(raw_cls, raw_cls)
        with self._confirm_lock:
            if self._confirm_class:
                target_can = ALIASES.get(self._confirm_class.lower().strip(), self._confirm_class.lower().strip())
                if canonical == target_can and msg.confidence >= 0.25:
                    self._confirm_detected = True

    def _rotate(self, direction: float, duration: float):
        """Rotate in-place. direction: +1.0 = left, -1.0 = right."""
        twist = Twist()
        twist.angular.z = direction * ROTATE_SPEED
        end = time.time() + duration
        while time.time() < end:
            self._cmd_pub.publish(twist)
            time.sleep(0.1)
        self._cmd_pub.publish(Twist())  # stop

    def _confirm_landmark(self, cls: str) -> bool:
        """
        Try to visually confirm landmark `cls` after arriving.
        Tries: wait 4s → rotate left 90° → wait 4s → rotate right 90° → wait 4s.
        Returns True if confirmed, False if missed.
        """
        with self._confirm_lock:
            self._confirm_class = cls
            self._confirm_detected = False

        def wait_for_detection(secs: float) -> bool:
            end = time.time() + secs
            while time.time() < end:
                with self._confirm_lock:
                    if self._confirm_detected:
                        return True
                time.sleep(0.05)
            return False

        # First wait at arrival heading
        if wait_for_detection(CONFIRM_WAIT_SECS):
            with self._confirm_lock:
                self._confirm_class = None
            return True

        # Rotate left ~60°, wait
        self._rotate(+1.0, ROTATE_DURATION)
        with self._confirm_lock:
            self._confirm_detected = False
        if wait_for_detection(CONFIRM_WAIT_SECS):
            with self._confirm_lock:
                self._confirm_class = None
            return True

        # Rotate right ~120° (back past centre + 60° right)
        self._rotate(-1.0, ROTATE_DURATION * 2)
        with self._confirm_lock:
            self._confirm_detected = False
        if wait_for_detection(CONFIRM_WAIT_SECS):
            with self._confirm_lock:
                self._confirm_class = None
            return True

        # Rotate back to centre
        self._rotate(+1.0, ROTATE_DURATION)
        with self._confirm_lock:
            self._confirm_class = None
        return False

    def _decrement_confidence(self, landmark_id: int):
        with self._db_lock:
            self.cursor.execute('''
                UPDATE landmarks SET confidence_score = MAX(confidence_score - 0.2, 0.0)
                WHERE id = ?
            ''', (landmark_id,))
            self.conn.commit()

    def _compute_standoff(self, rx: float, ry: float, tx: float, ty: float, dist: float = 1.2):
        d = math.hypot(tx - rx, ty - ry)
        if d > 0.3:
            ux, uy = (tx - rx) / d, (ty - ry) / d
            sx = tx - dist * ux
            sy = ty - dist * uy
            yaw = math.atan2(ty - sy, tx - sx)
            return sx, sy, yaw
        return tx, ty, 0.0

    # ─────────────────────── MAIN SERVICE ─────────────────────────────────────

    def _replay_cb(self, request, response):
        if not self._route_lock.acquire(blocking=False):
            response.success = False
            response.message = 'Another route replay is already in progress.'
            self.get_logger().warn(response.message)
            return response

        try:
            target_cls = request.target_class.lower().strip()
            target_cls = ALIASES.get(target_cls, target_cls)
            self.get_logger().info(f'[REPLAY] Route replay requested: target = "{target_cls}"')

            # ── 1. Load graph ──────────────────────────────────────────────────
            landmarks = self._get_all_landmarks()
            edges     = self._get_valid_edges('navigate')
            self.get_logger().info(
                f'[GRAPH] {len(landmarks)} landmarks, {len(edges)} valid edges.')

            adj = _build_adjacency(landmarks, edges)

            # ── 2. Find candidate sightings ────────────────────────────────────
            with self._db_lock:
                self.cursor.execute('''
                    SELECT * FROM object_sightings
                    WHERE object_class = ? AND superseded = 0
                ''', (target_cls,))
                candidates = [dict(r) for r in self.cursor.fetchall()]

            if not candidates:
                response.success = False
                response.message = f'EXPLORE: No memory found for "{target_cls}"'
                self.get_logger().warn(response.message)
                return response

            self.get_logger().info(
                f'[REPLAY] {len(candidates)} candidate sighting(s) found.')

            # ── 3. Phase-5 Trust vs Verify scoring ────────────────────────────
            now_ts = self.get_clock().now().nanoseconds * 1e-9
            best_candidate, best_score = None, -1.0

            for c in candidates:
                age          = max(0.0, now_ts - c['timestamp'])
                recency      = max(0.1, 1.0 - (age / 300.0))
                confidence   = c['confidence']
                corroboration = math.log2(c['corroboration_count'] + 1)
                score = recency * confidence * corroboration
                if score > best_score:
                    best_score, best_candidate = score, c

            W_DIST          = 1.0
            W_FAIL_PENALTY  = 20.0
            W_VERIFY_ACTION = 5.0

            distance    = math.hypot(best_candidate['pos_x'] - self.robot_x,
                                     best_candidate['pos_y'] - self.robot_y)
            cost_trust  = distance * W_DIST + max(0.0, 1.0 - best_score) * W_FAIL_PENALTY
            cost_verify = distance * W_DIST + W_VERIFY_ACTION
            decision    = 'TRUST' if cost_trust < cost_verify else 'PARTIAL_VERIFY'

            self.get_logger().info(
                f'[PHASE5] target={target_cls} score={best_score:.2f} '
                f'C_trust={cost_trust:.1f} C_verify={cost_verify:.1f} → {decision}')

            goal_x      = best_candidate['pos_x']
            goal_y      = best_candidate['pos_y']
            raw_goal_id = best_candidate['landmark_id']

            if raw_goal_id not in landmarks:
                self.get_logger().warn(
                    f'Sighting landmark_id={raw_goal_id} not in landmark table; '
                    f'using nearest.')
                goal_id = self._nearest_landmark(landmarks, goal_x, goal_y)
            else:
                goal_id = raw_goal_id

            start_id = self._nearest_landmark(landmarks, self.robot_x, self.robot_y)

            self.get_logger().info(
                f'[GRAPH] Robot=({self.robot_x:.2f},{self.robot_y:.2f}) '
                f'start={start_id}({landmarks[start_id]["class_name"] if start_id else "?"}), '
                f'goal={goal_id}({landmarks[goal_id]["class_name"] if goal_id else "?"})')

            # ── 4. A* path planning ────────────────────────────────────────────
            path_ids = None
            if start_id and goal_id and start_id != goal_id:
                _auto_connect(landmarks, adj, start_id, AUTO_CONNECT_RADIUS)
                path_ids = _astar(landmarks, adj, start_id, goal_id)
                if path_ids:
                    named = [f"[{lid}]{landmarks[lid]['class_name']}" for lid in path_ids]
                    self.get_logger().info(
                        f'[A*] Path ({len(path_ids)} hops): {" → ".join(named)}')
                else:
                    self.get_logger().warn(
                        f'[A*] No path found {start_id}→{goal_id}. '
                        f'adj[start]={adj.get(start_id,[])}')
            elif start_id == goal_id:
                self.get_logger().info('[GRAPH] start == goal; navigating directly.')

            if not path_ids or len(path_ids) < 2:
                # ── D3 FALLBACK ────────────────────────────────────────────────
                self.get_logger().info(
                    f'[FALLBACK] Sparse graph. Direct standoff nav to '
                    f'({goal_x:.2f},{goal_y:.2f}).')
                self._log_event('GRAPH_FALLBACK',
                                f'target={target_cls} goal_id={goal_id} '
                                f'landmarks={len(landmarks)} edges={len(edges)}')
                self._log_task_attempt('navigate', target_cls,
                                       start_id or -1, goal_id or -1,
                                       'ATTEMPT', 'fallback_nav')
                sx, sy, yaw = self._compute_standoff(
                    self.robot_x, self.robot_y, goal_x, goal_y)
                success = self._nav_to(sx, sy, yaw)
                outcome = 'SUCCESS' if success else 'FAILURE'
                reason  = 'nav2_success' if success else 'nav2_abort'
                self._log_task_attempt('navigate', target_cls,
                                       start_id or -1, goal_id or -1,
                                       outcome, reason)
                response.success              = success
                response.message              = ('Direct navigation succeeded (graph fallback).'
                                                 if success else 'Direct navigation failed.')
                response.landmarks_visited    = 0
                response.confirmations_failed = 0
                return response

            # ── 5. Leg-by-leg graph navigation ─────────────────────────────────
            visited          = 0
            failed           = 0
            consecutive_fails = 0

            for i, lid in enumerate(path_ids):
                lm       = landmarks[lid]
                cls      = lm['class_name']
                is_final = (i == len(path_ids) - 1)
                prev_lid = path_ids[i - 1] if i > 0 else (start_id or -1)

                self.get_logger().info(
                    f'[LEG {i+1}/{len(path_ids)}] → [{lid}] "{cls}" '
                    f'at ({lm["pos_x"]:.2f},{lm["pos_y"]:.2f}) '
                    f'{"[FINAL]" if is_final else ""}')

                sx, sy, yaw = self._compute_standoff(
                    self.robot_x, self.robot_y, lm['pos_x'], lm['pos_y'])
                nav_ok = self._nav_to(sx, sy, yaw)

                if not nav_ok:
                    self.get_logger().warn(
                        f'[LEG {i+1}] Nav2 failed for [{lid}] "{cls}". '
                        f'Jumping to final goal …')
                    sx2, sy2, yaw2 = self._compute_standoff(
                        self.robot_x, self.robot_y, goal_x, goal_y)
                    nav_ok = self._nav_to(sx2, sy2, yaw2)
                    if not nav_ok:
                        self._log_task_attempt(
                            'navigate', cls, prev_lid, lid, 'FAILURE', 'nav2_abort')
                        response.success              = False
                        response.message              = (
                            f'Nav2 aborted at leg {i+1}/{len(path_ids)} '
                            f'to [{lid}] "{cls}".')
                        response.landmarks_visited    = visited
                        response.confirmations_failed = failed
                        return response

                self._log_task_attempt(
                    'navigate', cls, prev_lid, lid, 'SUCCESS', 'nav2_success')

                if is_final:
                    break

                # Re-confirmation at intermediate landmark
                if decision == 'TRUST':
                    self.get_logger().info(
                        f'[CONFIRM] Skipping at [{lid}] (TRUST policy).')
                    confirmed = True
                else:
                    self.get_logger().info(
                        f'[CONFIRM] Visual check at [{lid}] "{cls}" …')
                    confirmed = self._confirm_landmark(cls)

                visited += 1
                if confirmed:
                    self.get_logger().info(f'[CONFIRM] ✓ [{lid}] "{cls}"')
                    consecutive_fails = 0
                    self._log_event('CONFIRM_OK', f'landmark={lid} class={cls}')
                else:
                    failed            += 1
                    consecutive_fails += 1
                    self._decrement_confidence(lid)
                    self._log_event('CONFIRM_FAIL',
                                    f'landmark={lid} class={cls} '
                                    f'consecutive={consecutive_fails}')
                    self.get_logger().warn(
                        f'[CONFIRM] ✗ [{lid}] "{cls}" '
                        f'(consecutive={consecutive_fails})')

                    if consecutive_fails >= CONSECUTIVE_MISS_ABORT:
                        self.get_logger().warn(
                            f'[D2] {consecutive_fails} consecutive misses — '
                            f'jumping to goal directly.')
                        sx, sy, yaw = self._compute_standoff(
                            self.robot_x, self.robot_y, goal_x, goal_y)
                        success = self._nav_to(sx, sy, yaw)
                        response.success              = success
                        response.message              = (
                            'Arrived at goal after confirmation misses.'
                            if success else 'Navigation failed after confirmation misses.')
                        response.landmarks_visited    = visited
                        response.confirmations_failed = failed
                        return response

            self._log_event('ROUTE_COMPLETE',
                            f'target={target_cls} visited={visited} fails={failed}')
            response.success              = True
            response.message              = (
                f'Arrived at "{target_cls}". '
                f'{visited} intermediate landmark(s) visited, '
                f'{failed} re-confirmation(s) missed.')
            response.landmarks_visited    = visited
            response.confirmations_failed = failed
            return response
        finally:
            self._route_lock.release()



def main(args=None):
    rclpy.init(args=args)
    node     = RouteReplayerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # Give the node its executor reference so _nav_to() can spin futures
    node._executor = executor
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
