#!/usr/bin/env python3
"""
AEMS Synthetic 10,000-Record Retrieval Benchmark
Validates NFR-P2: End-to-end memory retrieval latency < 1.0 s across 10,000 records.
End-to-end includes:
  1. NL query embedding generation (all-MiniLM-L6-v2, 384-dim)
  2. FAISS IndexFlatIP nearest-neighbor search across 10,000 vectors
  3. Multi-criteria episode scoring (recency x confidence x corroboration)
  4. Ranking and top candidate selection
"""

import os
import sys
import time
import json
import random
import math
import platform
import subprocess
import numpy as np
import faiss
from sentence_transformers import SentenceTransformer

# Ensure adaptive_episodic_memory is in python path
sys.path.insert(0, '/home/dsc/aems_ws/src/adaptive_episodic_memory')
from adaptive_episodic_memory.memory.retrieval import MemoryRetrieval


def get_hardware_info():
    """Retrieve CPU and RAM hardware specifications."""
    cpu_name = platform.processor() or "Unknown"
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                if "model name" in line:
                    cpu_name = line.split(":", 1)[1].strip()
                    break
    except Exception:
        pass

    ram_gb = "Unknown"
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if "MemTotal" in line:
                    kb = int(line.split()[1])
                    ram_gb = f"{kb / (1024 * 1024):.1f} GB"
                    break
    except Exception:
        pass

    return {
        "cpu": cpu_name,
        "cores": os.cpu_count(),
        "ram": ram_gb,
        "os": platform.platform()
    }


def generate_synthetic_dataset(n=10000, dim=384):
    """Generates 10,000 synthetic episodic records with pre-generated normalized vectors."""
    print(f"Generating {n} synthetic episodic records...")
    random.seed(42)
    np.random.seed(42)

    classes = [
        "couch", "dining table", "chair", "car", "fire hydrant",
        "person", "bottle", "cup", "bench", "backpack"
    ]
    directions = ["north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest"]

    records = []
    now = time.time()

    for i in range(n):
        cls_a = random.choice(classes)
        cls_b = random.choice([c for c in classes if c != cls_a])
        dist = round(random.uniform(1.0, 15.0), 1)
        direction = random.choice(directions)
        desc = f"{cls_a} is {dist} m {direction} of {cls_b}"

        records.append({
            "id": i + 1,
            "object_class": cls_a,
            "pos_x": round(random.uniform(-9.0, 9.0), 2),
            "pos_y": round(random.uniform(-9.0, 9.0), 2),
            "confidence": round(random.uniform(0.50, 0.98), 2),
            "corroboration_count": random.randint(1, 8),
            "timestamp": now - random.uniform(60, 48 * 3600),  # up to 48 hours ago
            "superseded": 0,
            "scene_description": desc
        })

    # Generate random normalized 384-dim vectors
    raw_vecs = np.random.randn(n, dim).astype(np.float32)
    faiss.normalize_L2(raw_vecs)
    return records, raw_vecs


def run_retrieval_benchmark():
    hw = get_hardware_info()
    print("=" * 65)
    print("AEMS RETRIEVAL BENCHMARK: 10,000 RECORDS (NFR-P2)")
    print(f"Hardware: {hw['cpu']} ({hw['cores']} cores) | RAM: {hw['ram']}")
    print(f"Platform: {hw['os']}")
    print("=" * 65)

    n_records = 10000
    dim = 384
    records, vectors = generate_synthetic_dataset(n=n_records, dim=dim)

    # Build FAISS index
    print("Indexing 10,000 records in FAISS IndexFlatIP...")
    t_idx_start = time.perf_counter()
    index = faiss.IndexFlatIP(dim)
    index.add(vectors)
    idx_time = time.perf_counter() - t_idx_start
    print(f"Index built in {idx_time * 1000:.2f} ms ({index.ntotal} vectors).")

    # Load SentenceTransformer model
    print("Loading SentenceTransformer ('all-MiniLM-L6-v2')...")
    model = SentenceTransformer('all-MiniLM-L6-v2')

    # Instance of scoring engine
    retriever = MemoryRetrieval(db_path='/home/dsc/aems_ws/aems_memory.db')

    test_queries = [
        "Where was the couch located near the dining table?",
        "Find the car parked in the north area",
        "Where was the person standing near the fire hydrant?",
        "Look for the chair beside the couch in the lounge",
        "Where is the dining table in the cafeteria?",
        "Locate the backpack left near the entrance",
        "Where did we last see the blue bottle?",
        "Find the seating area with the sofa"
    ]

    # Warmup
    print("Running 3 warm-up queries...")
    for q in test_queries[:3]:
        q_emb = model.encode([q], normalize_embeddings=True).astype(np.float32)
        D, I = index.search(q_emb, 20)
        for sim, idx in zip(D[0], I[0]):
            _ = retriever.score_episode(records[idx], semantic_similarity=sim)

    # Benchmark loop: 50 end-to-end iterations
    n_runs = 50
    latencies_ms = []
    embed_latencies_ms = []
    search_latencies_ms = []
    score_latencies_ms = []

    print(f"Executing {n_runs} benchmark iterations...")
    for i in range(n_runs):
        q = test_queries[i % len(test_queries)]

        t0 = time.perf_counter()

        # Step 1: Query embedding
        t_emb_start = time.perf_counter()
        q_emb = model.encode([q], normalize_embeddings=True).astype(np.float32)
        t_emb = (time.perf_counter() - t_emb_start) * 1000.0

        # Step 2: FAISS search across 10,000 records
        t_srch_start = time.perf_counter()
        D, I = index.search(q_emb, 20)
        t_srch = (time.perf_counter() - t_srch_start) * 1000.0

        # Step 3: Multi-criteria episode scoring and ranking
        t_score_start = time.perf_counter()
        candidates = []
        for sim, idx in zip(D[0], I[0]):
            rec = records[idx]
            s = retriever.score_episode(rec, semantic_similarity=sim)
            candidates.append((s, rec))
        candidates.sort(key=lambda x: x[0], reverse=True)
        best = candidates[0]
        t_score = (time.perf_counter() - t_score_start) * 1000.0

        total_ms = (time.perf_counter() - t0) * 1000.0

        latencies_ms.append(total_ms)
        embed_latencies_ms.append(t_emb)
        search_latencies_ms.append(t_srch)
        score_latencies_ms.append(t_score)

    latencies_ms = np.array(latencies_ms)
    mean_ms = float(np.mean(latencies_ms))
    median_ms = float(np.median(latencies_ms))
    p95_ms = float(np.percentile(latencies_ms, 95))
    p99_ms = float(np.percentile(latencies_ms, 99))
    min_ms = float(np.min(latencies_ms))
    max_ms = float(np.max(latencies_ms))
    qps = float(1000.0 / mean_ms)

    mean_embed = float(np.mean(embed_latencies_ms))
    mean_search = float(np.mean(search_latencies_ms))
    mean_score = float(np.mean(score_latencies_ms))

    target_met = (p95_ms < 1000.0)

    print("\n" + "=" * 65)
    print("BENCHMARK RESULTS (10,000-Record Synthetic Retrieval):")
    print("-" * 65)
    print(f"  Target Threshold       : < 1000.0 ms (< 1.0 s)")
    print(f"  Mean Total Latency     : {mean_ms:.2f} ms")
    print(f"  Median Latency         : {median_ms:.2f} ms")
    print(f"  95th Percentile (P95)  : {p95_ms:.2f} ms")
    print(f"  99th Percentile (P99)  : {p99_ms:.2f} ms")
    print(f"  Min / Max Latency      : {min_ms:.2f} ms / {max_ms:.2f} ms")
    print(f"  Throughput (QPS)       : {qps:.1f} queries/sec")
    print("-" * 65)
    print(f"  Breakdown:")
    print(f"    - Query Embedding    : {mean_embed:.2f} ms ({mean_embed/mean_ms*100:.1f}%)")
    print(f"    - FAISS 10k Search   : {mean_search:.2f} ms ({mean_search/mean_ms*100:.1f}%)")
    print(f"    - Episode Scoring    : {mean_score:.2f} ms ({mean_score/mean_ms*100:.1f}%)")
    print("-" * 65)
    print(f"  NFR-P2 Target Status   : {'PASSED (EXCEEDED TARGET)' if target_met else 'FAILED'}")
    print("=" * 65)

    results = {
        "hardware": hw,
        "record_count": n_records,
        "embedding_model": "all-MiniLM-L6-v2",
        "index_type": "faiss.IndexFlatIP",
        "runs": n_runs,
        "mean_latency_ms": round(mean_ms, 2),
        "median_latency_ms": round(median_ms, 2),
        "p95_latency_ms": round(p95_ms, 2),
        "p99_latency_ms": round(p99_ms, 2),
        "min_latency_ms": round(min_ms, 2),
        "max_latency_ms": round(max_ms, 2),
        "qps": round(qps, 1),
        "breakdown": {
            "query_embedding_ms": round(mean_embed, 2),
            "faiss_search_ms": round(mean_search, 2),
            "scoring_ms": round(mean_score, 2)
        },
        "target_ms": 1000.0,
        "target_met": target_met
    }

    out_file = "/home/dsc/aems_ws/benchmarks/retrieval_benchmark_results.json"
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Benchmark results saved to {out_file}")

    assert target_met, f"P95 latency {p95_ms}ms exceeded 1000ms target"
    return results


if __name__ == "__main__":
    run_retrieval_benchmark()
