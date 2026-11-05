# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning

I don't have the specific requirements or prior-cycle data for your "Cycle 10" iteration, so this is a general, rigorous procedure. Wherever it says **[spec]**, substitute your own values. If you share the payload mass, motor, prior-cycle residuals, or the CAD inertia export, I can make this quantitative.

---

## 1. Objectives

For a pitch cradle rotating about axis **x̂ₚ**, ranked by priority:

1. **Static balance.** The CG of the rotating assembly lies on the pitch axis, so gravity torque is zero at every pitch angle θ.
2. **Dynamic balance.** The pitch axis is a principal axis, so the products of inertia I_xy and I_xz are zero. This eliminates reaction torques into the roll and yaw axes during pitch acceleration.
3. **Inertia tuning.** I_xx is set to a target that trades motor torque margin against disturbance rejection and keeps the structural modes above the control bandwidth.

---

## 2. Coordinate Frame and Definitions

- **Origin O:** the intersection of the pitch axis with the cradle's mid-plane between bearings.
- **x:** along the pitch axis.
- **z:** along the camera optical axis at θ = 0.
- **y:** completes the right-handed frame.

The rotating assembly includes the cradle, camera, lens, any lens accessories, cable-loop share, motor rotor, encoder disk, and trim masses.

**CG offset:**

$$\mathbf{r}_{cg} = \frac{1}{M}\sum m_i \mathbf{r}_i = (x_{cg},\, y_{cg},\, z_{cg})$$

**Inertia tensor about O:**

$$\mathbf{I}_O = \begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz} \\ -I_{xy} & I_{yy} & -I_{yz} \\ -I_{xz} & -I_{yz} & I_{zz} \end{bmatrix}$$

**Gravity disturbance torque about pitch:**

$$\tau_g(\theta) = M g \left( y_{cg}\cos\theta \cdot(\dots) + z_{cg}\sin\theta \right)$$

Projected onto the axis, this reduces to:

$$\tau_g = M g\, d_\perp \sin(\theta + \phi)$$

where d⊥ = √(y_cg² + z_cg²) is the radial CG offset. Only d⊥ matters for pitch. x_cg affects the roll and yaw axes, not pitch.

**Cross-coupling torque during pitch motion** (Euler's equations, pure rotation about x):

$$\tau_y = -I_{xy}\ddot\theta + I_{xz}\dot\theta^2, \qquad \tau_z = -I_{xz}\ddot\theta - I_{xy}\dot\theta^2$$

(Sign conventions vary with tensor definition.) These torques are what the roll and yaw motors "feel" from a dynamically unbalanced cradle.

---

## 3. Acceptance Criteria Template

| Parameter | Criterion | Rationale |
|---|---|---|
| d⊥ (radial CG offset) | ≤ τ_allow / (M g), e.g. **≤ 0.1–0.3 mm** for small gimbals **[spec]** | Gravity torque ≤ ~5–10 % of continuous motor torque |
| x_cg | Within ±[spec] mm of the roll/yaw design point | Keeps the outer axes balanced |
| \|I_xy\|, \|I_xz\| | ≤ 1–2 % of I_xx **[spec]** | Cross-axis torque under peak θ̈ stays below outer-axis noise floor |
| I_xx | Target ± 5 % **[spec]** | Control-loop tuning holds without re-gain |
| First structural mode | ≥ 3–5 × closed-loop bandwidth | Avoids exciting cradle flex |
| Reconfiguration repeatability | d⊥ shift ≤ 50 % of tolerance across lens/accessory set | Field robustness |

---

## 4. Design-Phase Inertia Tuning (CAD/Analytical)

### Step 4.1: Build the component inventory
For each part *i*, record mass mᵢ, CG position **rᵢ**, and centroidal inertia **I_c,i**. Use measured masses for purchased parts (camera, lens, motor), not catalog values. Model cables as distributed mass along their routing.

### Step 4.2: Assemble the tensor using the parallel-axis theorem

$$\mathbf{I}_O = \sum_i \left[ \mathbf{R}_i \mathbf{I}_{c,i}\mathbf{R}_i^T + m_i\left( (\mathbf{r}_i\cdot\mathbf{r}_i)\mathbf{E} - \mathbf{r}_i\mathbf{r}_i^T \right) \right]$$

### Step 4.3: Solve for trim masses
Choose candidate trim locations **p_k** on the cradle, e.g. threaded holes or slide rails, placed far from the axis to get leverage. Then solve for trim masses m_k ≥ 0:

**Static constraints (2 equations):**
$$\sum_i m_i y_i + \sum_k m_k p_{k,y} = 0, \qquad \sum_i m_i z_i + \sum_k m_k p_{k,z} = 0$$

**Dynamic constraints (2 equations):**
$$I_{xy}^{0} + \sum_k m_k p_{k,x}p_{k,y} = 0, \qquad I_{xz}^{0} + \sum_k m_k p_{k,x}p_{k,z} = 0$$

**Optional inertia target:**
$$I_{xx}^{0} + \sum_k m_k (p_{k,y}^2 + p_{k,z}^2) = I_{xx}^{target}$$

**Objective:** minimize Σ m_k, which keeps total mass and outer-axis load low.

This is a small linear program or NNLS problem. You need at least **two axially separated trim planes** (different p_x) to null I_xy and I_xz independently of the static balance. This is the classic two-plane balancing result.

### Step 4.4: Adjustability
Prefer **sliding adjustment** over discrete masses for the dominant terms:

- A **fore/aft (z) slide** of the camera plate handles the lens-dependent z_cg.
- A **vertical (y) slide** handles y_cg.
- Fixed trim screws in two planes handle the residual products of inertia.

Slide range should cover the worst-case payload envelope plus 20 % **[spec]**.

---

## 5. Physical Balancing Procedure

### Preparation
1. Assemble the final flight configuration: camera, lens, filters, cables routed and tied exactly as flown, and batteries/cards inserted.
2. Power off the motors, or set them to zero-torque mode. Confirm bearing friction is low; record breakaway torque τ_f.
3. Lock the roll and yaw axes level.

### Step A: Coarse static balance (z_cg, optical axis)
1. Rotate the cradle to θ = 90° (lens pointing up or down, depending on convention).
2. Release it. If it rotates, adjust the **z-slide** until the cradle holds position.
3. Repeat in the opposite direction.

### Step B: Coarse static balance (y_cg)
1. Set θ = 0 (lens level).
2. Release it. Adjust the **y-slide** until the cradle stays level.

### Step C: Iterate and verify
1. Check the cradle at θ = 0°, ±45°, ±90°. It should remain stationary at every angle.
2. Iterate A and B, since the two adjustments interact slightly if the slides are not orthogonal to the true CG directions.

### Step D: Quantify the residual
Use either method:

- **Motor current method:** Enable a slow constant-velocity sweep through the full pitch range and log motor current i(θ). Then
  $$\tau(\theta) = K_t\, i(\theta) = \tau_f\,\text{sgn}(\dot\theta) + M g\, d_\perp \sin(\theta+\phi) + \text{cogging}$$
  Fit the first-harmonic sinusoid. Sweeping in both directions cancels friction. The amplitude gives M g d⊥ directly, and φ tells you which slide to move.
- **Torque gauge method:** Measure the holding torque at θ = 0° and 90° with a calibrated torque screwdriver or force gauge on a known lever arm.

**Correction vector:**
$$\Delta y = -\frac{A\cos\phi}{Mg}, \quad \Delta z = -\frac{A\sin\phi}{Mg}$$
(Adjust sign to match your angle convention.)

### Step E: Dynamic balance check
1. Command a pitch oscillation, e.g. a sine at a frequency near the expected disturbance band at high amplitude, or a step with high θ̈.
2. Log **roll and yaw motor currents** or IMU rates while those axes hold position.
3. Cross-axis torques correlated with θ̈ indicate I_xy or I_xz. Torques correlated with θ̇² indicate the same terms at high rate.
4. Estimate the products by least squares:
   $$\begin{bmatrix}\tau_y\\ \tau_z\end{bmatrix} = \begin{bmatrix} -\ddot\theta & \dot\theta^2 \\ -\dot\theta^2 & -\ddot\theta\end{bmatrix}\begin{bmatrix} I_{xy}\\ I_{xz}\end{bmatrix}$$
5. Correct by adding equal and opposite trim masses in **two axially separated planes**. Choose a pair (+Δm at +x, −Δm effectively at −x) so the pair adds a product of inertia but **no net static moment**. This keeps the static balance from Step D intact.

### Step F: I_xx identification
Use either method:

- **Torque-acceleration fit:** Apply a known torque profile (chirp or steps) with the static balance verified, then fit
  $$\tau = I_{xx}\ddot\theta + b\dot\theta + \tau_f\,\text{sgn}(\dot\theta)$$
- **Compound/torsion pendulum:** Add a known offset mass m_p at radius r and measure the oscillation period T. Then
  $$I_{xx} = \frac{m_p g r T^2}{4\pi^2} - m_p r^2$$
  Remove the mass afterward.

Compare the result to the CAD prediction. A discrepancy above 10 % suggests unmodeled cables or wrong lens data; update the model.

### Step G: Inertia adjustment (if out of target)
- **To increase I_xx:** add symmetric masses at large radius in ±y or ±z pairs. Symmetry keeps the CG and products unchanged.
- **To decrease I_xx:** lightweight the cradle far from the axis, e.g. with pocketing or a lighter material.

After any change, re-verify Steps D and E.

### Step H: Configuration matrix
Repeat Steps D through F for each supported lens and accessory configuration. Record the slide settings in a balance table, or engrave scale marks on the slides.

---

## 6. Control-Loop Implications

- **Loop gains:** With motor torque constant K_t and current-loop limits, the achievable pitch bandwidth scales as ω_bw ∝ √(K_p / I_xx). Re-tune, or apply gain scheduling, if I_xx changes by more than about 10 % between configurations.
- **Residual imbalance:** Residual d⊥ produces a θ-dependent bias. Integral action handles it, but it consumes torque headroom and increases heat. Record the steady-state current at the attitude extremes as a health metric.
- **Feedforward:** Optional gravity feedforward τ_ff = M g d̂⊥ sin(θ + φ̂) can use the Step D fit, but only if the configuration is fixed.

---

## 7. Documentation / Cycle Record

For each configuration, record:

- M, **r_cg** (measured), d⊥ residual, and fitted A and φ
- I_xx (CAD vs. measured), I_xy, I_xz (identified)
- Trim mass inventory and locations; slide positions
- Motor current at hold for θ = 0°, ±90°
- Cross-axis current RMS during the dynamic test
- Pass/fail against the Section 3 criteria; delta vs. the previous cycle

---

## 8. Common Failure Modes

- **Cable stiffness masquerading as imbalance.** Test with cables disconnected to separate the spring torque, which is approximately linear in θ, from gravity torque, which varies with sin θ.
- **Lens focus/zoom shifting the CG.** Balance at the mid-zoom setting, or provide a per-zoom table.
- **Bearing preload friction masking small residuals.** Always use bidirectional sweeps.
- **Single-plane trimming.** It fixes static balance but leaves products of inertia, which show up as roll/yaw jitter during fast pitch slews.

---

If you can share the Cycle 10 specifics (payload mass range, motor K_t and continuous torque, target bandwidth, and the Cycle 9 residuals), I can turn the template thresholds into actual numbers and size the trim masses and slide ranges.