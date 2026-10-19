import os
import json
import math
import urllib.request
import logging
import random

class LLMReasoner:
    def __init__(self, model_name=None):
        self.logger = logging.getLogger("LLMReasoner")
        self.logger.setLevel(logging.INFO)
        if not self.logger.handlers:
            self.logger.addHandler(logging.StreamHandler())

        self.api_key = os.environ.get("GROQ_API_KEY")
        if not self.api_key:
            env_file = os.path.expanduser('~/aems_ws/.env')
            if os.path.exists(env_file):
                try:
                    with open(env_file, 'r') as f:
                        for line in f:
                            if line.strip().startswith('GROQ_API_KEY='):
                                self.api_key = line.strip().split('=', 1)[1].strip('"\'')
                                os.environ['GROQ_API_KEY'] = self.api_key
                                break
                except Exception:
                    pass

        if not self.api_key:
            err_msg = "GROQ_API_KEY environment variable is not set. Please set it in your environment or .env file."
            self.logger.error(err_msg)
            raise RuntimeError(err_msg)

        self.model_name = model_name or os.environ.get("GROQ_MODEL", "qwen/qwen3.8-27b")

    def explain_decision(self, decision_dict: dict) -> dict:
        """
        Generates a natural, varied first-person explanation of the navigation decision.
        The LLM cannot change the action — it only produces the explanation text.
        """
        out = dict(decision_dict)

        if not self.api_key:
            self.logger.warning("No GROQ_API_KEY — using raw reason.")
            out["llm_explanation"] = out.get("reason", "No API key.")
            return out

        obj         = decision_dict.get('object_class', 'object')
        action      = decision_dict.get('action', 'trust_memory')
        score       = float(decision_dict.get('score', 0.5))
        corr        = int(decision_dict.get('corroboration_count', 1))
        c_trust     = float(decision_dict.get('expected_cost_trust', 10.0))
        c_reverify  = float(decision_dict.get('expected_cost_reverify', 16.8))
        nav_strat   = decision_dict.get('nav_strategy', 'direct navigation')
        raw_reason  = decision_dict.get('reason', '')

        # Round costs to nearest second for natural phrasing
        t_trust   = round(c_trust)
        t_verify  = round(c_reverify)

        # Qualitative confidence label
        if score >= 0.80:
            conf_word = "high"
        elif score >= 0.55:
            conf_word = "moderate"
        elif score >= 0.35:
            conf_word = "fair"
        else:
            conf_word = "low"

        # Opening phrase randomisation (avoids same first sentence every time)
        openers = [
            "Looking at my episodic memory,",
            "Based on what I remember,",
            "My internal memory vault tells me",
            "From my past observations,",
            "Checking my memory logs,",
        ]
        opener = random.choice(openers)

        prompt = f"""\
You are the reasoning core of a mobile robot with episodic memory. \
The deterministic cost engine has already made its decision — your job is to \
explain it naturally, as a thoughtful agent would, in 1–3 sentences. \
Do NOT invent facts, numbers, or object locations not given below. \
Do NOT repeat the exact phrasing from a previous call — vary your sentence structure. \
Speak in first person ("I …"). Use qualitative time language \
("around X seconds", "roughly half a minute") rather than exact decimals.

--- Decision context ---
Object being searched: {obj}
Chosen action: {action}
Confidence level: {conf_word} ({score:.0%})
Corroboration count: {corr} independent sighting(s)
Cost to trust memory and go directly: around {t_trust} seconds
Cost to reverify on the way: around {t_verify} seconds
Navigation strategy chosen: {nav_strat}
Opening phrase for this response (USE IT): "{opener}"

Raw cost-engine reason (for reference only — rephrase naturally):
{raw_reason}

Write your explanation now (1–3 sentences, first-person, conversational, no bullet points):"""

        req_data = json.dumps({
            "model":       self.model_name,
            "messages":    [{"role": "user", "content": prompt}],
            "temperature": 0.75,
            "max_tokens":  120,
        }).encode("utf-8")

        req = urllib.request.Request(
            "https://api.groq.com/openai/v1/chat/completions", data=req_data)
        req.add_header("Authorization", f"Bearer {self.api_key}")
        req.add_header("Content-Type", "application/json")
        req.add_header("User-Agent", "AEMS-Robot/1.0")

        try:
            with urllib.request.urlopen(req, timeout=6.0) as resp:
                resp_data   = json.loads(resp.read().decode())
                explanation = resp_data["choices"][0]["message"]["content"].strip()
                out["llm_explanation"] = explanation
        except Exception as e:
            self.logger.warning(f"Groq API failed ({e}). Using raw reason.")
            out["llm_explanation"] = out.get("reason", "API call failed.")

        return out

    def answer_memory_query(self, query_text: str, records: list) -> str:
        """
        Answers natural-language memory queries via Groq LLM grounded strictly
        in the retrieved episodic memories. Returns a clear not-found message if
        no records match or if the target is absent.
        """
        if not records:
            return "I could not find any memory records matching your query in my database."

        # Format retrieved records into clear factual context
        context_lines = []
        for i, raw_rec in enumerate(records, 1):
            rec = dict(raw_rec) if not isinstance(raw_rec, dict) else raw_rec
            obj = rec.get('object_class', 'unknown')
            px = float(rec.get('pos_x', 0.0) or 0.0)
            py = float(rec.get('pos_y', 0.0) or 0.0)
            conf = float(rec.get('confidence', 0.0) or 0.0)
            corr = int(rec.get('corroboration_count', 1) or 1)
            superseded = bool(rec.get('superseded', False))
            status = "superseded (moved or historical)" if superseded else "active (current location)"
            scene_desc = rec.get('scene_description', '')
            rx = rec.get('robot_pos_x')
            ry = rec.get('robot_pos_y')
            task = rec.get('task_info', '')

            line = (
                f"- Record {i}: Object='{obj}', Status={status}, Coordinates=({px:.2f}, {py:.2f}), "
                f"Confidence={conf:.0%}, Corroborations={corr}"
            )
            if scene_desc:
                line += f", Scene Context='{scene_desc}'"
            if rx is not None and ry is not None:
                line += f", Robot Pose=({float(rx):.2f}, {float(ry):.2f})"
            if task:
                line += f", Task='{task}'"
            context_lines.append(line)

        memory_context = "\n".join(context_lines)

        prompt = f"""\
You are the episodic memory query assistant for an autonomous mobile robot (AEMS).
Answer the user's question accurately and concisely, based SOLELY on the retrieved memory records provided below.

Strict Rules:
1. Do NOT invent, assume, or hallucinate any objects, positions, or relationships not stated in the records.
2. If the user asks about an object or event that is NOT present in the retrieved records, state clearly: "I could not find any memory of that in my database."
3. Mention the object coordinates, active vs superseded status, confidence, and spatial relationships where relevant.
4. Keep your answer direct, natural, and within 1 to 3 sentences.

--- Retrieved Episodic Memories ---
{memory_context}

--- User Question ---
"{query_text}"

Your factual answer:"""

        if not self.api_key:
            rec0 = dict(records[0]) if not isinstance(records[0], dict) else records[0]
            obj0 = rec0.get('object_class', 'object')
            px0 = float(rec0.get('pos_x', 0.0) or 0.0)
            py0 = float(rec0.get('pos_y', 0.0) or 0.0)
            desc0 = rec0.get('scene_description', '')
            return f"According to episodic memory, {obj0} was recorded at ({px0:.1f}, {py0:.1f}). {desc0}".strip()

        req_data = json.dumps({
            "model":       self.model_name,
            "messages":    [{"role": "user", "content": prompt}],
            "temperature": 0.2,
            "max_tokens":  160,
        }).encode("utf-8")

        req = urllib.request.Request(
            "https://api.groq.com/openai/v1/chat/completions", data=req_data)
        req.add_header("Authorization", f"Bearer {self.api_key}")
        req.add_header("Content-Type", "application/json")
        req.add_header("User-Agent", "AEMS-Robot/1.0")

        try:
            with urllib.request.urlopen(req, timeout=6.0) as resp:
                resp_data = json.loads(resp.read().decode())
                return resp_data["choices"][0]["message"]["content"].strip()
        except Exception as e:
            self.logger.warning(f"Groq API memory query failed ({e}). Using rule-based fallback.")
            rec0 = dict(records[0]) if not isinstance(records[0], dict) else records[0]
            obj0 = rec0.get('object_class', 'object')
            px0 = float(rec0.get('pos_x', 0.0) or 0.0)
            py0 = float(rec0.get('pos_y', 0.0) or 0.0)
            desc0 = rec0.get('scene_description', '')
            return f"According to episodic memory, {obj0} was recorded at ({px0:.1f}, {py0:.1f}). {desc0}".strip()
