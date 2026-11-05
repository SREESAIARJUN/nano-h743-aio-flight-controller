# Pitch-Axis Camera Cradle: Balancing Procedure and Inertia Tensor Tuning (Cycle 19)

**Note:** I don't have access to data from Cycles 1–18. This is a complete, self-contained procedure. Values in ⟨brackets⟩ are placeholders for your Cycle 19 payload and hardware. Feed in your previous-cycle residuals and I can tighten the tolerances.

---

## 1. Reference Frame and Definitions

**Cradle body frame {C}**, origin on the pitch bearing axis at the motor mid-plane:
- **y** lies along the pitch axis.
- **x** points forward along the optical axis at 0° pitch.
- **z** completes the right-handed frame (up at 0° pitch).

**Payload quantities**
- **Total mass** m: cradle, camera, lens, and accessories.
- **Center of mass** r_cg = (x_g, y_g, z_g).
- **Inertia tensor about the {C} origin**:

$$
\mathbf{I}_C=\begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz}\\ -I_{xy} & I_{yy} & -I_{yz}\\ -I_{xz} & -I_{yz} & I_{zz}\end{bmatrix}
$$

**Governing dynamics for pitch-only motion** (ω = ω_y ŷ, α = α_y ŷ):

$$
\boldsymbol{\tau}=\mathbf{I}_C\boldsymbol{\alpha}+\boldsymbol{\omega}\times(\mathbf{I}_C\boldsymbol{\omega}) + m\,\mathbf{r}_{cg}\times\mathbf{g}_{C}
$$

Expanded by component:

| Component | Expression | Effect |
|---|---|---|
| τ_y (pitch motor) | I_yy α_y + m g (x_g cos θ + z_g sin θ) | Wanted term plus gravity torque |
| τ_x (reacted into roll) | −I_xy α_y + I_yz ω_y² | Cross-coupling disturbance |
| τ_z (reacted into yaw) | −I_yz α_y − I_xy ω_y² | Cross-coupling disturbance |

**Objectives**
1. **Static balance:** x_g, z_g → 0. This removes gravity torque at all pitch angles.
2. **Dynamic balance:** I_xy, I_yz → 0. This makes y a principal axis, so pitch motion injects no torque into roll or yaw.
3. **Inertia tuning:** set I_yy to the target value I_yy* for the control design.
4. **Secondary:** keep y_g within the roll-axis budget, since it affects roll and yaw static balance.

---

## 2. Tolerance Budget

Express each tolerance in terms of the actuator and controller so it is physically meaningful.

| Parameter | Criterion | Typical target |
|---|---|---|
| Static imbalance m·√(x_g² + z_g²) | ≤ k_s · τ_hold / g, with k_s ≈ 0.03–0.05 | ⟨e.g. ≤ 2 g·cm⟩ |
| Cross product \|I_xy\|, \|I_yz\| | Coupled torque at max α ≤ 2% of roll/yaw motor continuous torque | ≤ 0.5–1% of I_yy |
| I_yy deviation | \|I_yy − I_yy*\| / I_yy* ≤ 5% | Keeps loop crossover within about ±2.5% |
| Load-to-rotor ratio | J_L / J_rotor within design band | ⟨e.g. 3–10 for direct drive⟩ |
| y_g | Per roll-axis budget | ⟨≤ 1 mm⟩ |

**Choosing I_yy\*:** for a fixed controller, plant gain scales as 1/J. Pick I_yy* at or slightly above the heaviest expected lens configuration. Lighter configurations are then trimmed up to I_yy*, so one tuned gain set covers all payloads. Also verify that the first structural mode satisfies f_struct > 4–5× the closed-loop bandwidth at I_yy*.

---

## 3. Measurement Methods

### 3.1 Center of mass (x_g, z_g)

**Motor-current method (preferred, in situ):**
1. With the control loop active, hold the cradle quasi-statically at θ = 0°, ±30°, ±60°, ±90°.
2. Record τ_y = K_t · i_q at each angle. Average forward and reverse slow sweeps to cancel Coulomb friction.
3. Fit by least squares: τ_y(θ) = m g (x_g cos θ + z_g sin θ) + τ_0.
4. The fitted coefficients give A = m g x_g and B = m g z_g directly.

**Unpowered tilt-check (fallback):**
- Release the motor at 0° and observe drift. Nose-down drift means x_g > 0.
- Rotate to 90° and repeat to isolate z_g.

### 3.2 Pitch inertia I_yy

**System identification:**
1. Inject a chirp or PRBS torque from 0.5 Hz up to about 0.3 × f_struct.
2. Fit the model: τ = I_yy α + b ω + τ_c sgn(ω) + A cos θ + B sin θ.
3. Subtract rotor inertia: I_yy = J_fit − J_rotor.

**Bench alternative:** a bifilar or trifilar pendulum about the y-axis, with I = m g d² T² / (16 π² L). Bifilar spacing d is half-separation-squared corrected per rig.

### 3.3 Products of inertia I_xy, I_yz

**In situ:**
1. Lock or stiffly hold roll and yaw.
2. Drive a pitch sinusoid θ = Θ sin(Ωt).
3. Measure reaction torques on the roll and yaw motors (current) or with a 6-axis load cell under the mount.
4. In-phase content with α gives I_xy and I_yz. Content at 2Ω, from the ω² terms, provides a cross-check.

**Bench (two-plane balancer):** spin the cradle about y on a soft-bearing balancing stand. Measure bearing reactions F_L and F_R at planes y_L and y_R. The moment-unbalance vector gives I_xy and I_yz.

**CAD prediction:** compute the tensor from CAD with real component masses. Use it for the pre-trim estimate only, never for sign-off.

---

## 4. Correction Mass Synthesis

Let the trim masses be m_k at positions (x_k, y_k, z_k), placed on available hard points or rails. Adding them changes the quantities linearly:

$$
\begin{aligned}
\Delta(m x_g) &= \textstyle\sum m_k x_k, & \Delta(m z_g) &= \textstyle\sum m_k z_k,\\
\Delta I_{xy} &= \textstyle\sum m_k x_k y_k, & \Delta I_{yz} &= \textstyle\sum m_k y_k z_k,\\
\Delta I_{yy} &= \textstyle\sum m_k (x_k^2+z_k^2)
\end{aligned}
$$

These signs use the tensor convention above, where off-diagonals are −I_ij.

**Solve the constrained problem.** Choose m_k ≥ 0 from the discrete trim-weight set to satisfy:

$$
\begin{aligned}
m x_g + \Delta(m x_g) &= 0\\
m z_g + \Delta(m z_g) &= 0\\
I_{xy} + \Delta I_{xy} &= 0\\
I_{yz} + \Delta I_{yz} &= 0\\
I_{yy} + \Delta I_{yy} &= I_{yy}^*
\end{aligned}
$$

while minimizing Σ m_k (added mass) subject to the y_g limit. With 5 equations, use at least 5 candidate locations; 8 corner hard points is typical. Solve as a small NNLS or MILP, then round to available weights and re-evaluate.

**Useful decoupled moves:**
- **Pure static shift:** mass on the y = 0 plane at (x, 0, z). This changes only m x_g, m z_g, and I_yy.
- **Pure product correction:** a mass pair at (a, +b, 0) and (−a, −b, 0). This gives ΔI_xy = 2 m a b with no CoM shift; use the z-analog for I_yz. Each pair adds 2 m a² to I_yy, so account for it.
- **Pure I_yy increase:** symmetric quad at (±r, ±b, 0) or a ring of radius r centred on the axis. This adds ΔI_yy = Σ m r² with no static or product change.

**Recommended order:** coarse static, then products (pairs), then I_yy (symmetric), then fine static. Products and I_yy additions are designed to be CoM-neutral, so the final static trim converges in one or two passes.

---

## 5. Cycle 19 Procedure

**Pre-conditions**
- Final payload is installed: lens at production focus/zoom state, cables dressed and strain-relieved.
- Fasteners are torqued, temperature is stable, and bearings are run in.
- Log the configuration ID, all masses, and the previous-cycle residuals.

**Steps**
1. **Mechanical pre-balance.** Slide the camera on its plate to get coarse x_g, then adjust cradle height (riser) to get coarse z_g, using the unpowered tilt check. The target is no drift at 0° and 90°.
2. **Baseline identification.** Run §3.1–3.3 and record m x_g, m z_g, I_yy, I_xy, I_yz, b, and τ_c. Repeat 3× and confirm repeatability ≤ 20% of tolerance.
3. **Compute the trim set** per §4.
4. **Install trims.** Use thread-locked weights, record positions, and verify total mass on a scale.
5. **Re-identify.** If any parameter is outside tolerance, recompute from the new residuals using the measured, not modelled, sensitivity. Iterate, typically 2–3 times.
6. **Range check.** Repeat the static fit over the full pitch range. Nonlinear residuals indicate cable or bearing preload effects; fix these mechanically, not with trim mass.
7. **Zoom/focus sweep.** For zoom lenses, measure x_g at the zoom extremes. If the shift exceeds tolerance, balance at mid-zoom (or the most-used focal length) and document the residual, or enable feedforward gravity compensation using the fitted A and B per zoom state.
8. **Controller update.** Set the plant inertia to the measured I_yy (rotor included). Re-verify phase margin (≥ 45°) and gain margin (≥ 8 dB), and update feedforward terms J·α_ref + τ_c sgn ω.
9. **Acceptance test.**
   - 30°/s step and 1 Hz ±10° sine.
   - Pitch tracking error and roll/yaw disturbance must stay within spec.
   - Motor RMS current at static hold must be below ⟨x⟩% of continuous rating.

---

## 6. Cycle 19 Record Sheet

| Quantity | Target | Baseline | Iter 1 | Iter 2 | Final | Pass |
|---|---|---|---|---|---|---|
| m (kg) | — | | | | | |
| m·x_g (kg·mm) | 0 ± ⟨⟩ | | | | | |
| m·z_g (kg·mm) | 0 ± ⟨⟩ | | | | | |
| y_g (mm) | ≤ ⟨⟩ | | | | | |
| I_yy (kg·m²) | ⟨I*⟩ ± 5% | | | | | |
| I_xy (kg·m²) | ≤ ⟨⟩ | | | | | |
| I_yz (kg·m²) | ≤ ⟨⟩ | | | | | |
| Viscous friction b, Coulomb friction τ_c | log | | | | | |
| Phase margin / gain margin | ≥ 45° / 8 dB | | | | | |
| Trim masses (ID @ x, y, z) | — | | | | | |

---

## 7. Common Failure Modes

- **Cable drag** shows up as a sin θ term or hysteresis between sweep directions. It looks like z_g error but changes with direction. Fix cable routing, not the balance.
- **Over-trimming I_yy** pushes the structural mode down; re-check f_struct after trimming.
- **Products tuned on the bench, then the payload changed:** the I_xy/I_yz sign can flip with lens length, so re-verify in situ.
- **Thermal drift** of the lens center of mass (focus breathing) affects long takes; check at operating temperature.

If you share the Cycle 19 payload masses, hard-point coordinates, motor K_t, and the Cycle 18 residuals, I can compute the actual trim-mass solution and fill in the tolerance numbers.