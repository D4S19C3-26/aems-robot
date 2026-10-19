#!/usr/bin/env python3
"""
Adaptive Episodic Memory System — Memory Retrieval & FAISS Indexing
Implements FR-11, FR-12, and NFR-P2:
- Embeds scene descriptions using sentence-transformers (all-MiniLM-L6-v2, 384-dim).
- Indexes active memories with FAISS (IndexFlatIP with L2-normalized embeddings for cosine similarity).
- FAISS produces initial candidates; score_episode ranks candidates using
  multi-criteria scoring: recency decay × detection confidence × corroboration count.
- Provides unified retrieval interface for both object queries and semantic scene queries.
"""

import os
import json
import math
import sqlite3
import numpy as np
from datetime import datetime, timezone
from typing import List, Dict, Optional, Tuple, Union

try:
    import faiss
    from sentence_transformers import SentenceTransformer
    FAISS_AVAILABLE = True
except ImportError:
    FAISS_AVAILABLE = False


class MemoryRetrieval:
    _embedder_instance = None  # Singleton to avoid reloading weights repeatedly

    @classmethod
    def get_embedder(cls, model_name: str = 'all-MiniLM-L6-v2'):
        if cls._embedder_instance is None and FAISS_AVAILABLE:
            cls._embedder_instance = SentenceTransformer(model_name)
        return cls._embedder_instance

    def __init__(self, store=None, db_path: str = None, conn=None, model_name: str = 'all-MiniLM-L6-v2'):
        if conn is not None:
            self.conn = conn
            self.store = store
            self.db_path = db_path or os.path.expanduser('~/aems_ws/aems_memory.db')
        elif store is not None:
            self.conn = getattr(store, 'conn', None)
            self.store = store
            self.db_path = getattr(store, 'db_path', None)
        else:
            self.store = None
            if db_path is None:
                db_path = os.path.expanduser('~/aems_ws/aems_memory.db')
            self.db_path = db_path
            self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
            self.conn.row_factory = sqlite3.Row

        # Scoring weights
        self.w_recency = 0.4
        self.w_confidence = 0.35
        self.w_corroboration = 0.25

        # Recency half-life in hours
        self.recency_half_life_hours = 24.0

        # Vector dimension for all-MiniLM-L6-v2
        self.embedding_dim = 384
        self.model_name = model_name
        self.model = self.get_embedder(model_name)

        # FAISS index (IndexFlatIP for cosine similarity on normalized vectors)
        if FAISS_AVAILABLE:
            self.index = faiss.IndexFlatIP(self.embedding_dim)
        else:
            self.index = None

        # Mapping: FAISS integer index -> sighting record dict
        self.records_map: List[Dict] = []

        # Sync FAISS index with database
        self.sync_index()

    def _recency_score(self, timestamp_val: Union[str, float, int]) -> float:
        """Exponential decay based on age."""
        try:
            if isinstance(timestamp_val, (int, float)):
                ts = datetime.fromtimestamp(timestamp_val, tz=timezone.utc)
            elif isinstance(timestamp_val, str) and timestamp_val:
                try:
                    ts = datetime.fromisoformat(timestamp_val)
                    if ts.tzinfo is None:
                        ts = ts.replace(tzinfo=timezone.utc)
                except ValueError:
                    ts = datetime.fromtimestamp(float(timestamp_val), tz=timezone.utc)
            else:
                return 0.5

            now = datetime.now(timezone.utc)
            age_hours = max(0.0, (now - ts).total_seconds() / 3600.0)
            return math.exp(-age_hours * math.log(2) / self.recency_half_life_hours)
        except Exception:
            return 0.1

    def score_episode(self, episode: Dict, semantic_similarity: float = 1.0) -> float:
        """
        Multi-criteria scoring: recency x confidence x corroboration,
        scaled by semantic similarity.
        """
        stamp = episode.get("timestamp", 0.0)
        recency = self._recency_score(stamp)
        confidence = float(episode.get("confidence", 0.5))
        corroboration = float(episode.get("corroboration_count", 1))

        # Normalized corroboration with diminishing returns
        corr_norm = min(1.0, math.log1p(corroboration) / math.log1p(10))

        base_score = (
            self.w_recency * recency +
            self.w_confidence * confidence +
            self.w_corroboration * corr_norm
        )

        sim_factor = 0.5 + 0.5 * max(0.0, min(1.0, float(semantic_similarity)))
        return round(base_score * sim_factor, 4)

    def encode_text(self, text: str) -> np.ndarray:
        """Generate normalized 384-dimensional embedding for a text string."""
        if not FAISS_AVAILABLE or self.model is None:
            return np.zeros((1, self.embedding_dim), dtype=np.float32)
        emb = self.model.encode([text], normalize_embeddings=True)
        return emb.astype(np.float32)

    def sync_index(self):
        """
        Reads all active object sightings from SQLite, computes missing embeddings,
        saves embeddings to DB, and populates the in-memory FAISS index.
        """
        if not FAISS_AVAILABLE or self.conn is None or self.index is None:
            return

        cursor = self.conn.cursor()
        try:
            cursor.execute("""
                SELECT id, object_class, pos_x, pos_y, pos_z, confidence, timestamp,
                       landmark_id, corroboration_count, superseded, robot_pos_x,
                       robot_pos_y, robot_yaw, task_info, scene_description, embedding
                FROM object_sightings
                WHERE superseded = 0
                ORDER BY timestamp DESC
            """)
        except sqlite3.OperationalError:
            cursor.execute("""
                SELECT * FROM object_sightings WHERE (superseded = 0 OR superseded IS NULL)
            """)

        rows = cursor.fetchall()
        self.index.reset()
        self.records_map = []
        vectors = []

        for row in rows:
            ep = dict(row)
            emb_blob = ep.get('embedding')
            desc = ep.get('scene_description') or f"{ep.get('object_class')} at ({ep.get('pos_x')}, {ep.get('pos_y')})"

            if emb_blob is not None and len(emb_blob) == self.embedding_dim * 4:
                vec = np.frombuffer(emb_blob, dtype=np.float32).reshape(1, self.embedding_dim)
            else:
                vec = self.encode_text(desc)
                try:
                    cursor.execute(
                        "UPDATE object_sightings SET embedding = ? WHERE id = ?",
                        (vec.tobytes(), ep['id'])
                    )
                except Exception:
                    pass

            vectors.append(vec[0])
            self.records_map.append(ep)

        if vectors:
            mat = np.array(vectors, dtype=np.float32)
            faiss.normalize_L2(mat)
            self.index.add(mat)
            try:
                self.conn.commit()
            except Exception:
                pass

    def retrieve(self, query_text: str, top_k: int = 3) -> List[Dict]:
        """
        Semantic retrieval:
        1. Embeds query_text.
        2. FAISS produces candidate nearest neighbors.
        3. score_episode ranks candidates.
        4. Returns top_k ranked records.
        """
        if not self.records_map or not FAISS_AVAILABLE or self.index is None:
            return self._fallback_sql_retrieve(query_text, top_k)

        query_clean = query_text.strip().lower()
        q_vec = self.encode_text(query_clean)

        k_search = min(len(self.records_map), max(top_k * 3, 5))
        similarities, indices = self.index.search(q_vec, k_search)

        scored_candidates = []
        seen_ids = set()

        for sim, idx in zip(similarities[0], indices[0]):
            if idx < 0 or idx >= len(self.records_map):
                continue
            rec = dict(self.records_map[idx])
            rec_id = rec.get('id', idx)
            if rec_id in seen_ids:
                continue
            seen_ids.add(rec_id)

            obj_cls = rec.get('object_class', '').lower()
            rec['class_match'] = 1 if (obj_cls == query_clean or obj_cls in query_clean.split()) else 0
            if rec['class_match']:
                sim = max(sim, 0.98)

            rec['similarity'] = float(sim)
            rec['score'] = self.score_episode(rec, semantic_similarity=sim)
            scored_candidates.append(rec)

        for rec in self.records_map:
            rec_id = rec.get('id')
            if rec_id not in seen_ids:
                obj_cls = rec.get('object_class', '').lower()
                if obj_cls and (obj_cls in query_clean or query_clean in obj_cls):
                    seen_ids.add(rec_id)
                    rec_copy = dict(rec)
                    rec_copy['class_match'] = 1 if (obj_cls == query_clean or obj_cls in query_clean.split()) else 0
                    rec_copy['similarity'] = 0.95
                    rec_copy['score'] = self.score_episode(rec_copy, semantic_similarity=0.95)
                    scored_candidates.append(rec_copy)

        scored_candidates.sort(key=lambda x: (x.get('class_match', 0), x['score']), reverse=True)
        return scored_candidates[:top_k]

    def _fallback_sql_retrieve(self, query_text: str, top_k: int = 3) -> List[Dict]:
        """Fallback keyword query when FAISS or embeddings are unavailable."""
        if not self.conn:
            return []
        cursor = self.conn.cursor()
        cursor.execute("""
            SELECT * FROM object_sightings
            WHERE (superseded = 0 OR superseded IS NULL)
            ORDER BY timestamp DESC
        """)
        rows = cursor.fetchall()
        scored = []
        q = query_text.lower()
        for r in rows:
            ep = dict(r)
            cls = ep.get('object_class', '').lower()
            sim = 1.0 if (cls in q or q in cls) else 0.5
            ep['score'] = self.score_episode(ep, semantic_similarity=sim)
            ep['similarity'] = sim
            scored.append(ep)
        scored.sort(key=lambda x: x['score'], reverse=True)
        return scored[:top_k]

    def get_best_sighting(self, query_text: str) -> Optional[Dict]:
        """Returns single best matching episode for query_text."""
        candidates = self.retrieve(query_text, top_k=1)
        return candidates[0] if candidates else None

    def retrieve_for_object(self, object_class: str, top_k: int = 3) -> List[Dict]:
        return self.retrieve(object_class, top_k=top_k)

    def get_best_route_hint(self, object_class: str) -> Optional[Dict]:
        candidates = self.retrieve(object_class, top_k=1)
        if not candidates:
            return None
        best = candidates[0]
        landmark = None
        if self.store and hasattr(self.store, 'get_landmark'):
            landmark = self.store.get_landmark(best.get('landmark_id'))
        return {
            "episode": best,
            "landmark": landmark,
            "score": best.get("score", 0.5)
        }
