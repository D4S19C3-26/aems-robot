# AEMS Experimental Evaluation & Comparative Baseline Analysis (Weakness A Resolution)

## 1. Experimental Design & Hypotheses

To validate the real-world utility of the Adaptive Episodic Memory System (AEMS), we benchmark against two fundamental mobile robot navigation paradigms across $N = 50$ independent trials per scenario:

1. **Baseline 1: Memoryless Frontier Exploration (`Frontier_Exploration`)**  
   Standard exploration without semantic memory. The robot visits topological room vantage points in greedy order until target visual intersection occurs.
2. **Baseline 2: Naive Static Spatial Memory (`Naive_Static_Memory`)**  
   Conventional spatial caching without recency decay ($\lambda = 0$), without corroboration reinforcement, without conflict detection, and without wall-clearing standoff planning.
3. **Proposed Method: AEMS (`AEMS_Proposed`)**  
   Full system with topometric FAISS vector indexing, exponential recency decay, Bayesian corroboration reinforcement, decision-theoretic trust-vs-verify cost selection, and autonomous multi-room perimeter scouting.

---

## 2. Experimental Scenarios

- **Scenario 1: Static Target (Fresh Memory)**  
  The target object (`couch`) remains at its previously sighted coordinate $(0.0, -7.0)$.
- **Scenario 2: Relocated Target (Spatial Conflict & Stale Memory)**  
  The target object was moved to a different room $(-7.0, 7.0)$ while the robot was elsewhere. The historical memory is in spatial conflict with reality.
- **Scenario 3: Occluded Target (Interior Wall Partition)**  
  The target object is occluded behind vertical interior wall `iw_2` at $(-4.5, 6.0)$, testing wall-clearing standoff computation and autonomous perimeter scouting.

---

## 3. Quantitative Results & Comparative Findings

### Summary Performance Table ($N=50$ Trials per Scenario)

| Scenario | Navigation Method | Success Rate (%) | Search Latency (s) | Path Length (m) | Latency Reduction vs. Frontier |
|---|---|:---:|:---:|:---:|:---:|
| **Scenario 1: Static Target** | Memoryless Frontier Exploration | 100.0% | $196.4 \pm 99.2$ | 37.3 | Baseline (0.0%) |
| | Naive Static Spatial Memory | 100.0% | $82.2 \pm 2.4$ | 16.1 | -58.2% |
| | **AEMS (Proposed Method)** | **100.0%** | **$65.6 \pm 1.9$** | **14.8** | **-66.6%** |
| **Scenario 2: Relocated Target** | Memoryless Frontier Exploration | 100.0% | $197.7 \pm 108.3$ | 37.7 | Baseline (0.0%) |
| | Naive Static Spatial Memory | 100.0% | $286.1 \pm 94.0$ | 51.7 | +44.7% (Overhead) |
| | **AEMS (Proposed Method)** | **100.0%** | **$43.0 \pm 1.2$** | **9.9** | **-78.2%** |
| **Scenario 3: Occluded Target** | Memoryless Frontier Exploration | 100.0% | $192.4 \pm 106.9$ | 37.6 | Baseline (0.0%) |
| | Naive Static Spatial Memory | 0.0% | $60.1 \pm 1.8$ | 10.3 | Failed (Occluded) |
| | **AEMS (Proposed Method)** | **100.0%** | **$29.6 \pm 0.9$** | **6.5** | **-84.6%** |

---

## 4. Analytical Insights & Reviewer Takeaways

1. **Static Target Efficiency:**  
   In Scenario 1, AEMS achieves a **20.2% latency reduction** over Naive Static Memory ($65.6\text{ s}$ vs. $82.2\text{ s}$) and **66.6% reduction** over Frontier Exploration. This gain stems from the *high-confidence en-route visual stop* ($conf \ge 0.75$), which preempts redundant final $360^\circ$ sweeps once visual confirmation is established.
2. **Dynamic Conflict Resilience:**  
   In Scenario 2, Naive Static Memory suffers a severe penalty ($286.1\text{ s}$, **+44.7% worse than blind frontier search**) because it drives to the stale coordinate, times out, and only then falls back to full exploration. AEMS detects the conflict rapidly, supersedes the old observation, and achieves **78.2% lower latency** ($43.0\text{ s}$) and **74.7% shorter path length** ($9.9\text{ m}$ vs. $37.7\text{ m}$).
3. **Occlusion Robustness:**  
   In Scenario 3, Naive Static Memory **fails completely (0% success rate)** because its line-of-sight ray intersects wall `iw_2`. AEMS achieves **100% success rate** with **84.6% latency reduction** ($29.6\text{ s}$) via its *Autonomous Multi-Room Perimeter Scout*.
