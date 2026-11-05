# Pitch-Axis Camera Cradle: Balancing and Inertia Tensor Tuning Procedure (Cycle 12)

> **Scope note:** I don't have your Cycles 1–11 data, drawings, or acceptance limits. This is a general, self-contained procedure. Values marked **[TBD]** should come from your design baseline or previous-cycle results.

---

## 1. Frames and Definitions

- **Pitch frame {P}:** origin at the intersection of the pitch axis with the bearing mid-plane.
  - **x̂** lies along the pitch axis (rotation axis).
  - **ẑ** lies along the camera optical axis at nominal level (θ = 0).
  - **ŷ** completes the right-handed set.
- **Rotating payload:** the cradle, camera body, lens, cables within the moving envelope, motor rotor, encoder disk, and trim masses. Model it as N rigid bodies with:
  - mass mᵢ,
  - CG position **rᵢ** in {P},
  - local inertia **I_cᵢ** about its own CG, rotated into {P} by **Rᵢ**.

**Composite properties:**

$$
M=\sum m_i,\qquad \mathbf{r}_{cg}=\frac{1}{M}\sum m_i\mathbf{r}_i
$$

$$
\mathbf{I}_P=\sum\Big[\mathbf{R}_i\mathbf{I}_{c,i}\mathbf{R}_i^{T}+m_i\big(\|\mathbf{r}_i\|^2\mathbf{1}-\mathbf{r}_i\mathbf{r}_i^{T}\big)\Big]
=\begin{bmatrix}I_{xx}&-I_{xy}&-I_{xz}\\-I_{xy}&I_{yy}&-I_{yz}\\-I_{xz}&-I_{yz}&I_{zz}\end{bmatrix}
$$

---

## 2. Balance Objectives

| Condition | Requirement | Physical meaning |
|---|---|---|
| **Static balance** | r_cg,y = r_cg,z = 0 | No gravity torque about pitch at any angle |
| **Axial CG** | r_cg,x ≈ 0 (bearing mid-plane) | Equalizes bearing loads; reduces roll-frame moment |
| **Dynamic balance** | I_xy = I_xz = 0 | Pitch axis is a principal axis; pitch acceleration produces no reaction torque on roll/yaw |
| **Inertia target** | I_xx = I_target ± tol | Sets plant gain for the controller and the disturbance-rejection inertia |

**Gravity torque from residual imbalance** (gravity in the y–z plane):

$$
\tau_g(\theta)=Mg\,(r_z\sin\theta - r_y\cos\theta)\;\Rightarrow\;|\tau_g|_{max}=Mg\sqrt{r_y^2+r_z^2}
$$

**Static acceptance:** |τ_g|max ≤ k·τ_motor,cont, typically k = 2–5%. For example, with M = 1.2 kg, a 0.1 mm offset gives about 1.2 mN·m.

**Cross-coupling torque** on the outer axes during a pitch maneuver at θ̈ and θ̇:

$$
\boldsymbol{\tau}_{coupling}= \ddot\theta\begin{bmatrix}0\\-I_{xy}\\-I_{xz}\end{bmatrix}+\dot\theta^2\begin{bmatrix}0\\I_{xz}\\-I_{xy}\end{bmatrix}
$$

**Dynamic acceptance:** |I_xy|, |I_xz| ≤ ε·I_xx, with ε typically 0.5–2% **[TBD]**, chosen so the coupling torque stays below the roll/yaw loop disturbance budget.

---

## 3. Trim-Mass Tuning Formulation

Place K trim masses m_k at candidate locations **p_k** (tapped holes or slots on the cradle). Let the untrimmed state be (M₀, **S₀** = M₀**r**₀, **I**₀).

**First moments.** These are linear in the masses:

$$
S_y = S_{0,y}+\sum m_k p_{k,y}=0,\qquad S_z = S_{0,z}+\sum m_k p_{k,z}=0
$$

**Products of inertia.** These are also linear in the masses, treating trims as point masses or adding their own small inertia:

$$
I_{xy}=I_{0,xy}+\sum m_k p_{k,x}p_{k,y}=0,\qquad I_{xz}=I_{0,xz}+\sum m_k p_{k,x}p_{k,z}=0
$$

**Pitch inertia:**

$$
I_{xx}=I_{0,xx}+\sum m_k\,(p_{k,y}^2+p_{k,z}^2)
$$

**Optimization problem.** With discrete hole positions, this is a bounded linear program or QP in **m**:

$$
\min_{\mathbf m}\; w_1\!\sum m_k + w_2\,(I_{xx}-I_{target})^2
\quad\text{s.t.}\quad \mathbf{A}\mathbf{m}=-\mathbf{b}_0,\;\; 0\le m_k\le m_{max},\;\; \sum m_k\le \Delta M_{budget}
$$

where **A** stacks the rows [p_y], [p_z], [p_x p_y], [p_x p_z].

**Practical notes:**

- Correcting the products of inertia requires at least **two axially separated correction planes** (±x). Correcting in one plane cannot fix I_xy or I_xz without disturbing static balance.
- After solving, round to available washer or slug masses and re-check the residuals.
- If I_xx must *decrease*, trim masses cannot achieve it. Remove material or relocate components instead, and feed this back to design.

**Order of adjustment in practice:**

1. Slide the camera on its plate along **z** (fore/aft) to null r_z.
2. Adjust the cradle height or shim along **y** to null r_y.
3. Apply two-plane trims for I_xy and I_xz.
4. Re-verify static balance, since the trims shift the CG slightly.

---

## 4. Experimental Identification

### 4.1 Static Balance (motor de-energized or decoupled)

**Release test:** release the payload at θ = 0°, ±45°, and ±90°. Drift indicates residual imbalance. Friction sets the detection floor.

**Quantitative method (motor current):** hold the payload at angles θⱼ under slow closed-loop position control and record the current iⱼ. Then fit:

$$
K_t i_j = Mg(r_z\sin\theta_j - r_y\cos\theta_j) + \tau_{f}\,\text{sgn}(\dot\theta)
$$

- Least squares over a sweep of about 12 angles, run in both directions, gives r_y, r_z, and the Coulomb friction.
- Moving in both directions cancels the friction term.

### 4.2 I_xx Identification

**Option A — Compound pendulum:**

1. Add a known offset mass m_a at a known radius d to create a deliberate CG offset.
2. Measure the small-angle swing period T.
3. Compute:

$$
I_{xx}+m_a d^2 = \frac{(M\,\ell + m_a d)\,g\,T^2}{4\pi^2}
$$

where ℓ is the residual offset, approximately 0 after balancing. Subtract the known contributions.

**Option B — Closed-loop chirp or step identification** (preferred, since it is done in situ):

$$
K_t i = J\ddot\theta + b\dot\theta + \tau_g(\theta) + \tau_f
$$

- J = I_xx + J_rotor + J_encoder.
- Excite with a 1–50 Hz chirp at small amplitude.
- Fit J and b from the frequency-response magnitude slope (−40 dB/dec region) or by time-domain least squares.

### 4.3 Product-of-Inertia Verification

1. Lock roll and yaw through a 6-axis load cell, or use their motor currents as torque sensors.
2. Drive pitch sinusoidally: θ = A sin ωt.
3. The reaction torques on y and z at frequency ω have amplitude Aω²·I_xy and Aω²·I_xz.
4. The 2ω components indicate the centrifugal terms.
5. Fit I_xy and I_xz, then compare them with the dynamic acceptance criterion.

---

## 5. Cycle 12 Step-by-Step Procedure

1. **Configuration freeze:** record the camera, lens, focal setting, filters, cable routing, and firmware. Any change invalidates balance.
2. **CAD baseline:** compute M, **r**_cg, and **I**_P from the model using Section 1. Log the predicted residuals.
3. **Coarse static balance:** perform the z-slide and y-shim adjustments until the release test shows no drift at ±90°.
4. **Fine static identification:** run the current-sweep fit (4.1) and compute the residual |τ_g|max.
5. **Product-of-inertia identification:** run the sinusoidal reaction test (4.3).
6. **Solve the trim QP** (Section 3) using the measured residuals, *not* the CAD values.
7. **Install the trims:** use thread-locker and torque-stripe them. Record their mass, location, and fastener torque.
8. **Re-identify:** repeat steps 4–5 and iterate steps 6–7 until the criteria are met (typically 1–2 iterations).
9. **I_xx identification:** use the chirp method (4.2). Compare with I_target.
10. **Controller update:** rescale the pitch loop gains to the measured J. The crossover frequency scales as √(K/J) for a fixed-structure PID, so preserve the phase margin.
11. **Thermal and attitude check:** repeat step 4 at the temperature extremes and with the gimbal rolled ±30°, to catch cable spring torque and CG shift.
12. **Close-out:** record the cycle data (Section 6) and compare trends against Cycles 10–11.

---

## 6. Cycle 12 Record Template

| Item | CAD | Pre-trim | Post-trim | Limit | Pass |
|---|---|---|---|---|---|
| M [kg] | | | | — | |
| r_y [mm] | | | | ≤ [TBD] | |
| r_z [mm] | | | | ≤ [TBD] | |
| r_x [mm] | | | | ≤ [TBD] | |
| \|τ_g\|max [mN·m] | | | | ≤ k·τ_cont | |
| I_xx [kg·m²] | | | | I_target ± [TBD]% | |
| I_xy / I_xx [%] | | | | ≤ ε | |
| I_xz / I_xx [%] | | | | ≤ ε | |
| Friction τ_f [mN·m] | | | | ≤ [TBD] | |
| Trim masses (m, p) | — | — | list | ΔM budget | |

---

## 7. Common Pitfalls

- **Cable stiffness masquerading as imbalance.** A cable that acts as a spring gives a torque linear in θ, while gravity imbalance gives a sinusoidal torque. Fit both terms to separate them.
- **Zoom or focus shifting the CG.** Balance at the mid or most-used setting and characterize the full range.
- **Single-plane correction.** Fixing static balance in one plane can worsen I_xy and I_xz.
- **Including the rotor in balance but not in J.** The rotor counts in J for control, but it is usually already self-balanced.

If you share the Cycle 11 residuals, the trim hole pattern, and the target I_xx, I can set up the specific trim QP with numbers.