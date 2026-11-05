# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning (Cycle 15)

> **Assumption:** I don't have your Cycle 14 data or project-specific tolerances. Below is a complete, self-contained procedure. Placeholders like ⟨…⟩ mark where your program values go.

---

## 1. Reference Frame and Definitions

- **Origin O** is on the pitch-axis bearing centerline, at the midplane between the two bearings.
- **y-axis** is the pitch rotation axis.
- **x-axis** points forward along the optical axis at θ = 0 (level).
- **z-axis** points up at θ = 0.
- **Cradle assembly** (cradle, camera, lens, accessories, trim masses) has total mass *M*, center of gravity **r_c** = (x_c, y_c, z_c), and inertia tensor **I_O** about O:

$$
\mathbf I_O=\begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz}\\ -I_{xy} & I_{yy} & -I_{yz}\\ -I_{xz} & -I_{yz} & I_{zz}\end{bmatrix},\qquad I_{xy}=\textstyle\int xy\,dm,\ \text{etc.}
$$

---

## 2. Balance Objectives

| Condition | Requirement | Physical consequence if violated |
|---|---|---|
| **Static (pitch)** | x_c = 0, z_c = 0 | Gravity torque about pitch varies with θ; motor holds a constant load and heats |
| **Static (lateral)** | y_c ≈ 0, relative to the roll-axis intersection | Gravity load on the roll axis; moves the outer-gimbal CG |
| **Dynamic** | I_xy = 0, I_yz = 0 | Pitch motion produces reaction torques on the roll and yaw axes (cross-axis coupling) |
| **Inertia magnitude** | I_yy within ⟨target band⟩ | Sets control bandwidth, motor sizing, and disturbance rejection |

### 2.1 Gravity torque about pitch

With gravity in −z of the base frame and cradle pitch θ:

$$
\tau_g(\theta)=Mg\,(x_c\cos\theta+z_c\sin\theta)
$$

### 2.2 Cross-axis coupling from rotation about y

For ω = (0, ω, 0) and α = ω̇, Euler's equations give:

$$
\tau_x = -I_{xy}\,\alpha + I_{yz}\,\omega^2,\qquad
\tau_z = -I_{yz}\,\alpha - I_{xy}\,\omega^2,\qquad
\tau_y = I_{yy}\,\alpha
$$

Signs follow the tensor convention above. Zeroing **I_xy** and **I_yz** eliminates both the acceleration-coupled and the centrifugal cross-torques. **I_xz** does not couple pure pitch motion, but it matters when roll and yaw are active. Include it in the tuning if the full 3-axis model calls for it.

---

## 3. Procedure

### Step 0: Configuration freeze
1. Record the Cycle 15 payload configuration: camera body, lens and focal setting, filters, battery, cables, and accessories.
2. Weigh every component and record its CAD position.
3. Route cables in their flight or operational positions and tie them down. Cable stiffness and drape are the most common source of irreproducible balance.

### Step 1: Analytical prediction from CAD

Use the parallel-axis theorem in tensor form for each component *i* with mass m_i, centroid **p_i**, and centroidal tensor **I_Ci**:

$$
\mathbf I_O=\sum_i\Big[\mathbf R_i\mathbf I_{C,i}\mathbf R_i^T+m_i\big((\mathbf p_i\!\cdot\!\mathbf p_i)\mathbf E-\mathbf p_i\mathbf p_i^T\big)\Big],\qquad
\mathbf r_c=\frac{1}{M}\sum_i m_i\mathbf p_i
$$

Compute the predicted x_c, z_c, I_xy, and I_yz to set the initial trim.

### Step 2: Coarse static balance (motor de-energized)
1. Unlock the pitch axis, or disconnect it if the motor has cogging.
2. Slide the camera along the fore-aft rail (x) until the cradle holds level (θ = 0) with no drift. This sets x_c ≈ 0.
3. Rotate the cradle to θ = 90° (nose up).
4. Adjust the vertical riser (z) until it holds there too. This sets z_c ≈ 0.
5. Verify neutral stability at 45° and −45°. The cradle should stay put at any angle.

### Step 3: Fine static balance (motor-current method)
1. Energize the motor in position hold.
2. Sweep θ over ⟨−90°…+90°⟩ in about 15° steps at quasi-static speed, in both directions so friction can be removed.
3. At each angle, record the holding torque τ = K_t·i_q.
4. Average the up and down sweeps to cancel Coulomb friction, then fit:

$$
\tau(\theta)=A\cos\theta+B\sin\theta+C \;\;\Rightarrow\;\; x_c=\frac{A}{Mg},\quad z_c=\frac{B}{Mg}
$$

C is residual bias: cable spring, cogging, or sensor offset. Investigate it if it exceeds ⟨10 %⟩ of the gravity residual.

### Step 4: Dynamic measurement
1. Command a sinusoidal pitch motion θ(t) = Θ sin(Ωt). Run at least three frequencies spanning ⟨0.5–5 Hz⟩ while keeping Θ within the mechanical limits.
2. Log the pitch torque and the roll- and yaw-axis reaction torques. Use motor currents with the outer axes in stiff position hold, or a 6-axis load cell at the cradle mount.
3. Identify the inertias by least-squares regression on (α, ω²):
   - **I_yy:** fit τ_y = I_yy α + b ω + τ_g(θ) + τ_f sgn(ω).
   - **I_xy and I_yz:** fit τ_x and τ_z against the α and ω² columns using the relations in §2.2.
4. **Optional cross-check of I_yy:** do a compound-pendulum test. Add a known offset mass m₀ at distance d below the axis and measure the swing period T:

$$
I_{yy}+m_0d^2=\frac{(Mz_{c}+m_0 d)\,g\,T^2}{4\pi^2}\quad(\text{with } z_c\approx 0\text{ after Step 3})
$$

### Step 5: Trim-mass solution (two-plane method)

A point trim mass m_k at **p_k** = (x_k, y_k, z_k) changes:

$$
\Delta(Mx_c)=m_kx_k,\quad \Delta(Mz_c)=m_kz_k,\quad \Delta I_{xy}=m_kx_ky_k,\quad \Delta I_{yz}=m_ky_kz_k
$$

1. Place trim pads on **two planes** at y = +b and y = −b, at the cradle side plates as far apart as possible.
2. Solve for the four unknowns (m₁x₁, m₁z₁, m₂x₂, m₂z₂), each pad having fixed y = ±b:

$$
\begin{bmatrix}1&0&1&0\\0&1&0&1\\ b&0&-b&0\\ 0&b&0&-b\end{bmatrix}
\begin{bmatrix}m_1x_1\\ m_1z_1\\ m_2x_2\\ m_2z_2\end{bmatrix}
=-\begin{bmatrix}Mx_c\\ Mz_c\\ I_{xy}\\ I_{yz}\end{bmatrix}
$$

   This gives a closed-form solution:

$$
m_1x_1=-\tfrac12\!\left(Mx_c+\tfrac{I_{xy}}{b}\right),\;
m_2x_2=-\tfrac12\!\left(Mx_c-\tfrac{I_{xy}}{b}\right)
$$

$$
m_1z_1=-\tfrac12\!\left(Mz_c+\tfrac{I_{yz}}{b}\right),\;
m_2z_2=-\tfrac12\!\left(Mz_c-\tfrac{I_{yz}}{b}\right)
$$

3. Pick the trim-pad radius from the available hole pattern, which sets m_k and the angle in the x–z plane.
4. Prefer **larger radius with smaller mass**. This minimizes the added I_yy, which grows as Σ m_k(x_k² + z_k²).
5. If the I_yy target is binding, add a mass-penalized least-squares term or redistribute toward the axis.

### Step 6: I_yy tuning (if outside the target band)
- **To reduce I_yy:** move dense items (battery, lens counterweights) toward the axis. Replace aluminum brackets with lighter ones at large radius.
- **To increase I_yy:** do this only for controller or resonance reasons, such as separating a structural mode. Add symmetric pairs of mass at ±x or ±z, at equal y on both planes, so the static and product balance is preserved.
- **Check the inertia ratio:** I_yy / (rotor inertia × N²) should be ⟨within the controller design range⟩. Re-tune the loop gains if I_yy changes by more than ⟨±10 %⟩.

### Step 7: Verify and iterate
1. Repeat Steps 3–4 with the trim installed.
2. Iterate, usually 1–2 passes, until the acceptance criteria are met.
3. Torque-seal all fasteners and trim pads.

---

## 4. Acceptance Criteria (fill with program values)

| Parameter | Criterion | Typical starting value |
|---|---|---|
| Residual gravity torque, max over θ | ≤ ⟨ ⟩ | ≤ 5 % of motor continuous torque |
| Static offset √(x_c² + z_c²) | ≤ ⟨ ⟩ | ≤ 0.1–0.3 mm |
| Lateral offset y_c | ≤ ⟨ ⟩ | ≤ 0.5 mm |
| \|I_xy\|/I_yy and \|I_yz\|/I_yy | ≤ ⟨ ⟩ | ≤ 1–2 % |
| I_yy | within ⟨ ⟩ of target | ±5–10 % |
| Hysteresis C term | ≤ ⟨ ⟩ | ≤ 10 % of gravity residual |
| Repeatability, remove and reinstall payload | Δx_c, Δz_c ≤ ⟨ ⟩ | ≤ 0.1 mm |

---

## 5. Configuration Robustness

If the payload changes (lens zoom or focus shifts mass, battery swaps, accessories):
1. Characterize **Δr_c** and **ΔI** for each configuration.
2. Balance to the **centroid of the configuration envelope**, or provide indexed rail detents per configuration.
3. For zoom lenses, verify the criteria at both focal-length extremes.

---

## 6. Cycle 15 Record Template

| Item | Pre-trim | Trim applied | Post-trim | Pass/Fail |
|---|---|---|---|---|
| M (kg) | | | | |
| x_c, z_c (mm) | | | | |
| y_c (mm) | | | | |
| I_yy (kg·m²) | | | | |
| I_xy, I_yz (kg·m²) | | | | |
| Max τ_g residual (N·m) | | | | |
| Trim pads (mass, location) | — | | — | |
| Controller gains re-tuned? | | | | |
| Deltas vs Cycle 14 | | | | |

Notes: cable routing photo reference, fastener torques, environmental temperature, and test-rig ID.

---

If you share the Cycle 14 results, the payload mass properties, or the motor and controller specs, I can turn the placeholders into concrete numbers and compute the trim-mass solution directly.