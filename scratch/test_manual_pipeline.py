#!/usr/bin/env python3
import time, os, json, sqlite3

db_path = os.path.expanduser('~/aems_ws/aems_memory.db')

print('=== 1. TESTING RETRIEVAL & FAISS ===')
from adaptive_episodic_memory.memory.retrieval import MemoryRetrieval
retriever = MemoryRetrieval(db_path=db_path)
res = retriever.retrieve('Where is the couch located in the living area?', top_k=3)
if res:
    top = res[0]
    print(f"Top match: {top.get('object_class')} with score {top.get('retrieval_score')}")
    print(f"Coordinates: ({top.get('pos_x')}, {top.get('pos_y')})")
    print(f"Scene description: {top.get('scene_description')}")
else:
    print("No retrieval match found.")

print('\n=== 2. TESTING GROQ LLM WITH GROUNDED MEMORIES ===')
from adaptive_episodic_memory.reasoning.llm_reasoning import LLMReasoner
llm = LLMReasoner()
t0 = time.time()
ans = llm.answer_memory_query('Where is the couch located and what is near it?', res)
print(f"Groq LLM answer ({time.time()-t0:.2f}s):\n{ans}")

print('\n=== 3. TESTING TRUST-VS-VERIFY COST MATH ===')
from adaptive_episodic_memory.memory.memory_store import MemoryStore
store = MemoryStore(db_path=db_path)

# Let's verify the protected cost formulation:
# cost_trust = 8.0 + (1 - p_valid) * 40.0
# cost_reverify = 12.0 + 8.0 * 0.6 = 16.8
for p in [0.95, 0.70, 0.50, 0.20]:
    c_trust = 8.0 + (1.0 - p) * 40.0
    c_reverify = 12.0 + 8.0 * 0.6
    action = 'trust_memory' if c_trust < c_reverify and p > 0.45 else ('reverify' if p > 0.25 else 'explore')
    print(f"p_valid={p:.2f} -> C_trust={c_trust:.1f}s, C_reverify={c_reverify:.1f}s -> Decision: {action}")

print('\n=== 4. TESTING STANDOFF & WALL CLEARING ===')
from adaptive_episodic_memory.reasoning.dashboard_node import compute_standoff
# Robot at (0, 7), Object at (0, -7)
sx, sy, yaw = compute_standoff(0.0, 7.0, 0.0, -7.0)
print(f"Computed Standoff: x={sx:.2f}, y={sy:.2f}, yaw={yaw:.2f} rad ({yaw*180/3.14159:.1f} deg)")

print('\n=== 5. TESTING E-STOP & INPUT SANITIZER ===')
from adaptive_episodic_memory.reasoning.dashboard_node import DashboardNode
import rclpy
rclpy.init()
dn = DashboardNode()
v1, t1 = dn._sanitize_command('Go to the couch')
v2, t2 = dn._sanitize_command('find couch; rm -rf /')
v3, t3 = dn._sanitize_command('locate car -- DROP TABLE object_sightings')
print(f"Sanitizer clean cmd: valid={v1}, text='{t1}'")
print(f"Sanitizer shell injection: valid={v2}, msg='{t2}'")
print(f"Sanitizer SQL injection: valid={v3}, msg='{t3}'")

# Test named zones
print('\n=== 6. TESTING NAMED ZONES RESOLUTION ===')
for q in ['go to reception area', 'drive to the vehicle bay', 'navigate to dining area', 'take me to the library']:
    zk, zi = dn._resolve_zone(q)
    print(f"Resolve '{q}' -> Zone: {zk}")
dn.destroy_node()
rclpy.shutdown()
