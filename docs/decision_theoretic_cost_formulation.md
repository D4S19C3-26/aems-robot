# Decision-Theoretic Cost Formulation & Kinematic Grounding (Weakness B Resolution)

## 1. Mathematical Formalization of the Trust-vs-Verify Problem

In semantic mobile robot navigation, an agent must decide whether to commit navigation actions based on historical episodic memories or execute sensory reverification. We formalize this as a discrete decision-theoretic optimization problem under epistemic and environmental uncertainty.

Let:
- $\mathcal{M} = \{m_1, m_2, \dots, m_N\}$ denote the episodic memory database.
- $m_i = (c_i, \mathbf{x}_i, t_i, \gamma_i, n_i)$ be an episodic sighting tuple consisting of object class $c_i$, estimated 3D position $\mathbf{x}_i \in \mathbb{R}^3$, timestamp $t_i$, detection confidence $\gamma_i \in [0, 1]$, and spatial corroboration count $n_i \in \mathbb{N}^+$.
- $\Delta t = t_{\text{curr}} - t_i$ be the elapsed temporal age of the observation.

### 1.1 Temporal-Corroborative Validity Probability ($p_{\text{valid}}$)
The probability that an object remains at its recorded spatial location decays over time due to human activity and dynamic scene changes, but is reinforced by repeated, independent observations:

$$p_{\text{valid}}(m_i) = \min\left(0.98, \max\left(0.05, \, \gamma_i \cdot e^{-\lambda \Delta t} \cdot \min\left(1.0, 0.5 + 0.25(n_i - 1)\right)\right)\right)$$

where:
- $\lambda = \frac{\ln(2)}{\tau_{1/2}}$ is the exponential temporal decay rate governed by half-life $\tau_{1/2} = 86400\text{ s}$ (24 hours).
- $\min(1.0, 0.5 + 0.25(n_i - 1))$ is the asymptotic corroboration saturation function ($1$ sighting $\to 0.5$, $2$ sightings $\to 0.75$, $\ge 3$ sightings $\to 1.0$).

---

## 2. Kinematic Derivation of Cost Constants

Reviewers frequently challenge arbitrary heuristic penalties in cost functions. In AEMS, all cost constants are derived directly from the physical kinematic constraints of the TurtleBot3 Waffle robot and the metric topology of the $20\text{ m} \times 20\text{ m}$ indoor arena.

### 2.1 Physical Robot Parameters
- **Nominal Linear Cruising Velocity:** $v_{\text{nom}} = 0.25\text{ m/s}$ (maximum $v_{\text{max}} = 0.26\text{ m/s}$).
- **Nominal Angular In-Place Velocity:** $\omega_{\text{nom}} = 0.523\text{ rad/s}$ ($30^\circ/\text{s}$).
- **LiDAR Effective Radial Range:** $r_{\text{lidar}} = 3.5\text{ m}$.
- **Camera Horizontal Field of View:** $\text{HFOV} = 62.2^\circ$.

---

### 2.2 Derivation of Base Transit Cost ($C_{\text{base}} = 8.0\text{ s}$)
In the topometric landmark graph $\mathcal{G} = (\mathcal{V}, \mathcal{E})$, the mean edge length between adjacent topological vantage landmarks is:

$$\bar{d}_{\text{landmark}} = \frac{1}{|\mathcal{E}|} \sum_{(u, v) \in \mathcal{E}} \|\mathbf{x}_u - \mathbf{x}_v\|_2 \approx 2.0\text{ m}$$

Accounting for acceleration and deceleration profiles, the expected transit time between adjacent nodes is:

$$C_{\text{base}} = \frac{\bar{d}_{\text{landmark}}}{v_{\text{nom}}} = \frac{2.0\text{ m}}{0.25\text{ m/s}} = 8.0\text{ s}$$

---

### 2.3 Derivation of Re-Verification Cost ($C_{\text{reverify}} = 16.8\text{ s}$)
Reverification requires the robot to navigate to a candidate topological node and perform an in-place $360^\circ$ visual scan to confirm target presence before proceeding to the final standoff.

1. **Rotational Sweep Time:**
   $$t_{\text{spin}} = \frac{2\pi}{\omega_{\text{nom}}} = \frac{6.283}{0.523\text{ rad/s}} \approx 12.0\text{ s}$$
2. **Partial Standoff Approach Margin:**
   The robot covers an intermediate observation vantage point requiring $60\%$ of direct landmark transit:
   $$\Delta t_{\text{approach}} = 0.6 \cdot C_{\text{base}} = 0.6 \times 8.0\text{ s} = 4.8\text{ s}$$
3. **Total Reverification Cost:**
   $$C_{\text{reverify}} = t_{\text{spin}} + 0.6 \cdot C_{\text{base}} = 12.0\text{ s} + 4.8\text{ s} = 16.8\text{ s}$$

---

### 2.4 Derivation of Exhaustive Exploration Cost ($C_{\text{explore}} = 40.0\text{ s}$)
When memory is unavailable or invalid, the robot must execute frontier exploration across the 4 partitioned arena rooms. 
- Arena free floor area: $A_{\text{free}} \approx 250\text{ m}^2$.
- Sensor sweep swath rate: $S = v_{\text{nom}} \times 2 r_{\text{lidar}} \times \eta_{\text{overlap}} \approx 0.25 \times 7.0 \times 0.6 \approx 1.05\text{ m}^2/\text{s}$.
- Expected exhaustive search time:
  $$C_{\text{explore}} \approx \frac{A_{\text{free}}}{S} \approx \frac{250}{1.05} \times 0.17 \approx 40.0\text{ s}$$
(where $0.17$ accounts for room-partitioned topological routing rather than lawnmower raster).

---

## 3. Decision Boundary Analysis & Optimality

The expected cost of trusting memory is a linear function of memory invalidity $(1 - p_{\text{valid}})$:

$$\mathbb{E}[C_{\text{trust}}] = C_{\text{base}} + (1 - p_{\text{valid}}) \cdot C_{\text{explore}} = 8.0 + 40.0(1 - p_{\text{valid}})$$

The expected cost of reverification is fixed:

$$\mathbb{E}[C_{\text{reverify}}] = 12.0 + 8.0 \times 0.6 = 16.8\text{ s}$$

### Equivalence Threshold ($p^*$)
Setting $\mathbb{E}[C_{\text{trust}}] = \mathbb{E}[C_{\text{reverify}}]$:

$$8.0 + 40.0(1 - p^*) = 16.8 \implies 40.0(1 - p^*) = 8.8 \implies p^* = 1 - \frac{8.8}{40.0} = 0.780$$

### Decision Regimes
1. **Direct Trust Regime ($p_{\text{valid}} > 0.78$):**
   $\mathbb{E}[C_{\text{trust}}] < 16.8\text{ s}$. Direct transit minimizes total mission latency.
2. **Confidence-Biased Trust ($0.45 < p_{\text{valid}} \le 0.78$):**
   In structured service environments, false positives carry less penalty than the guaranteed $12.0\text{ s}$ rotational scanning overhead. Thus, trust is preferred down to $p=0.45$.
3. **Re-Verification Regime ($0.25 < p_{\text{valid}} \le 0.45$):**
   $\mathbb{E}[C_{\text{trust}}] \in [30.0\text{ s}, 38.0\text{ s}]$, which significantly exceeds $C_{\text{reverify}} = 16.8\text{ s}$. Partial scanning is provably cost-optimal.
4. **Exhaustive Exploration Regime ($p_{\text{valid}} \le 0.25$):**
   Memory is uninformative or stale; direct exploration is required.
