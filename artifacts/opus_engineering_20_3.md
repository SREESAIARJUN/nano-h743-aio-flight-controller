# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning (Cycle 20)

**Assumptions:** I don't have your Cycle 20 design data, so this procedure is written for a typical 2- or 3-axis gimbal with a brushless direct-drive pitch motor. Replace the bracketed placeholders with your numbers.

---

## 1. Coordinate Frame and Definitions

- **Pitch frame {P}:** origin on the pitch bearing axis at the motor mid-plane.
  - x_P: optical axis (forward, at 0° tilt)
  - y_P: pitch rotation axis
  - z_P: completes the right-handed set
- **Payload set:** cradle + camera body + lens + filters + cables (service loop). Treat this as a single rigid body B with mass *m*, CG vector **r**_G = [x_G, y_G, z_G]^T in {P}, and inertia tensor about the pitch origin:

$$
\mathbf{I}_P = \begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz} \\ -I_{xy} & I_{yy} & -I_{yz} \\ -I_{xz} & -I_{yz} & I_{zz} \end{bmatrix}
$$

- **Parallel-axis relation**, from the CG tensor **I**_G:

$$
\mathbf{I}_P = \mathbf{I}_G + m\left[(\mathbf{r}_G^T\mathbf{r}_G)\mathbf{E}_3 - \mathbf{r}_G\mathbf{r}_G^T\right]
$$

---

## 2. Balance Objectives

| Requirement | Physical meaning | Target (placeholder) |
|---|---|---|
| Static balance, pitch | x_G, z_G → 0 (CG on pitch axis) | ‖(x_G, z_G)‖ ≤ [0.2 mm] |
| Axial CG | y_G centered between bearings / on roll axis plane | \|y_G\| ≤ [0.5 mm] |
| Dynamic balance (cross-coupling) | I_xy, I_yz → 0, so pitch is a principal axis | \|I_xy\|, \|I_yz\| ≤ [1 %] of I_yy |
| Pitch inertia | I_yy within the controller design band | I_yy = [I_nom] ± [10 %] |
| Inertia ratio for the outer axes | Limits roll/yaw reaction torques | per system budget |

**Why each term matters:**

- **Gravity torque** about pitch: τ_g = m·g·(z_G sinθ − x_G cosθ)-type terms. Residual CG offset produces a tilt-dependent bias that the motor must hold continuously. This consumes current and heats the motor.
- **Products I_xy, I_yz** couple pitch acceleration into roll/yaw torques: τ_coupled = −I_xy·θ̈ (and similar). This shows up as cross-axis jitter during fast tilts.
- **I_yy** sets the plant gain 1/(I_yy·s²). It drives the loop bandwidth, phase margin, and the frequency of the first structural mode relative to the crossover.

---

## 3. Analytical Pre-Balance (CAD Stage)

1. **Build the mass model.** Use measured masses for the camera body, lens (at each zoom/focus extreme if it changes), battery, media, and accessories. Override CAD densities with measured part masses.
2. **Compute the composite CG and inertia.** Sum per-component tensors via the parallel-axis theorem:
   $$\mathbf{r}_G = \frac{1}{m}\sum m_i \mathbf{r}_i, \qquad \mathbf{I}_P = \sum \left(\mathbf{R}_i\mathbf{I}_{G,i}\mathbf{R}_i^T + m_i[\ldots]\right)$$
3. **Size the adjustments.** Determine the required travel of each balance mechanism:
   - Fore/aft slide (x): Δx = −m·x_G / m_slide-portion. If the whole payload slides, Δx = −x_G.
   - Vertical riser (z): Δz = −z_G.
   - Lateral slide (y): Δy = −y_G.
4. **Check the tuning range.** Verify that adjustment travel covers the full payload envelope plus a [±20 %] margin.
5. **Solve for trim masses that null the products of inertia** (Section 5) and check the added mass against the budget.

---

## 4. Physical Static Balancing Procedure

**Prerequisites:**
- Motors unpowered or set to zero-torque mode.
- Bearings run in.
- Cable service loop installed in its final routing. Cables are a significant unbalance source.

### Step 1: Lock the outer axes
Lock roll and yaw so the pitch axis is horizontal. Confirm with a 0.1°/m level.

### Step 2: Balance vertical CG (z)
1. Rotate the cradle to the lens-up position (θ = +90°) and release.
2. Adjust the vertical riser until the cradle holds position without drift. In this pose, gravity torque is driven by z_G.

### Step 3: Balance fore/aft CG (x)
1. Set the lens horizontal (θ = 0°) and release.
2. Adjust the fore/aft slide until there is no rotation.

### Step 4: Iterate
Repeat Steps 2 and 3 because the adjustments interact. Stop when the cradle holds at 0°, ±45°, and ±90° with no drift for [10 s].

### Step 5: Quantitative verification
Power the pitch motor in position hold and log the motor current (q-axis) at slow quasi-static sweeps over θ ∈ [−90°, +90°]. Fit:

$$
\tau(\theta) = k_t i_q(\theta) = A\cos\theta + B\sin\theta + \tau_f\,\mathrm{sgn}(\dot\theta) + C
$$

Then:
- x_G ≈ −A/(m·g)
- z_G ≈ B/(m·g)

**Pass criterion:** \|A\|, \|B\| ≤ [0.5 %] of continuous motor torque. Sweep in both directions to separate out friction τ_f.

### Step 6: Lateral (y) balance
1. Unlock roll, set pitch to 0°, and balance roll with the lateral slide.
2. Re-check pitch, since the lateral slide can shift x/z if its rail is not parallel.

---

## 5. Inertia Tensor Tuning (Dynamic Balance)

Static balance does not guarantee that the pitch axis is a principal axis.

### 5.1 Measurement
Use either method:

**a. Torsional pendulum / trifilar rig**
Measure the period T about at least 6 independent axes through the CG. For each axis direction **n**:

$$
I_n = \mathbf{n}^T \mathbf{I}_G \mathbf{n}
$$

This gives a linear system in the 6 unknowns (I_xx, I_yy, I_zz, I_xy, I_xz, I_yz). Solve by least squares, using more than 6 axes for redundancy.

**b. In-situ system identification**
1. Inject chirp or PRBS torque on the pitch axis.
2. Fit I_yy from the low-frequency plant 1/(J·s²).
3. Measure the reaction torque or IMU response on roll/yaw. The cross-coupling gain gives I_xy and I_yz:
   $$\tau_{roll}/\ddot\theta \approx -I_{xy}, \qquad \tau_{yaw}/\ddot\theta \approx -I_{yz}$$
   (with the axes mapped to your gimbal ordering).

### 5.2 Trim-mass correction
Add point trim masses m_k at locations (x_k, y_k, z_k). Each contributes to the totals as follows:

| Quantity | Contribution per trim mass |
|---|---|
| CG first moment | Σ m_k·**r**_k = 0 (to preserve static balance) |
| ΔI_xy | m_k·x_k·y_k |
| ΔI_yz | m_k·y_k·z_k |
| ΔI_yy | m_k·(x_k² + z_k²) |

**Use paired masses.** Place two equal masses symmetric through the CG: (+x, +y) and (−x, −y). Their first moments cancel, so static balance is untouched, while I_xy changes by 2m·x·y. The same applies to the y–z plane for I_yz.

**Solve the constrained minimization:**

$$
\min_{m_k,\mathbf{r}_k} \sum m_k \quad \text{s.t.}\quad
\begin{cases}
\sum m_k \mathbf{r}_k = -m\,\mathbf{r}_G^{res}\\
I_{xy}+\sum m_k x_k y_k = 0\\
I_{yz}+\sum m_k y_k z_k = 0\\
I_{yy}+\sum m_k(x_k^2+z_k^2) \in [I_{min}, I_{max}]
\end{cases}
$$

Restrict **r**_k to the discrete mounting points on the cradle, such as threaded trim holes. This makes it a small mixed-integer problem; solving by enumeration is fine.

### 5.3 Principal axis check
1. Eigen-decompose **I**_G to get the principal axes **e**_1..**e**_3.
2. Compute the angle between ŷ_P and its nearest principal axis.
3. **Pass criterion:** angle ≤ [1°].

---

## 6. I_yy Tuning Against Controller Design

1. **Check the loop gain.** With measured I_yy, compute the crossover ω_c = sqrt(K_p·k_t/I_yy)-type estimate for your PID/LQR. Confirm phase margin ≥ [45°].
2. **Check structural mode separation.** The first cradle/arm structural mode f_s must satisfy f_s ≥ [3–5]·f_c. If adding trim mass lowers f_s:
   - stiffen the cradle side plates, or
   - relocate mass closer to the bearing axis, since ΔI scales with r².
3. **Handle a high I_yy.** If I_yy exceeds budget, prefer moving the payload toward the axis over adding counter-mass.
4. **Handle payload variation.** Store per-payload I_yy in a gain-schedule table, or rely on auto-tune at startup.

---

## 7. Cycle 20 Acceptance Record

| Item | Measured | Limit | Pass |
|---|---|---|---|
| m (payload + cradle) | | | |
| x_G, z_G residual | | ≤ [0.2 mm] | |
| y_G residual | | ≤ [0.5 mm] | |
| A, B gravity coefficients | | ≤ [0.5 %] τ_cont | |
| I_yy | | [I_nom ± 10 %] | |
| \|I_xy\|/I_yy, \|I_yz\|/I_yy | | ≤ [1 %] | |
| Principal-axis misalignment | | ≤ [1°] | |
| Trim mass added | | ≤ [budget g] | |
| f_s / f_c | | ≥ [3] | |
| Cross-axis jitter during [X °/s²] step | | ≤ [Y arcsec RMS] | |

**Regression check vs Cycle 19:** document deltas in CG, tensor, and trim configuration. Flag any change over [10 %] for design review.

---

## 8. Common Error Sources

- **Cable service loop.** Its stiffness and position vary with tilt and mimic CG error. Balance with the final cables installed, and record the hysteresis.
- **Lens zoom/focus mass shift.** Balance at the most-used setting, then record the worst-case residual at the extremes.
- **Bearing friction.** Masks small unbalances during the drift test. Always confirm with the motor-current fit from Step 5.
- **Thermal growth.** Re-verify after a [30 min] powered soak if operating outside [ΔT].

If you share the Cycle 20 payload mass, geometry, motor constant, and the residuals from Cycle 19, I can compute the specific slide offsets and a trim-mass layout.