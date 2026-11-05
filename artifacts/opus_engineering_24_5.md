# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning (Cycle 24)

> **Assumptions:** I don't have your Cycle 23 data, payload spec, or gimbal geometry. This procedure assumes a typical 3-axis gimbal with a pitch (tilt) cradle carried inside a roll frame, driven by a direct-drive BLDC motor. Bracketed values `[ ]` are placeholders to replace with your program's numbers.

---

## 1. Objectives and Acceptance Criteria

| Parameter | Symbol | Target (Cycle 24) | Rationale |
|---|---|---|---|
| CG offset ⟂ pitch axis | ‖r_cg,⊥‖ | ≤ `[0.05]` mm | Gravity torque under 2–5 % of motor holding torque at any angle |
| Residual static torque | τ_g,max = m·g·‖r_cg,⊥‖ | ≤ `[2]` mN·m | Lets the controller run in the low-current, linear region |
| CG offset along pitch axis | r_cg,x | ≤ `[0.5]` mm | Couples into roll/yaw when the outer frames rotate |
| Product of inertia I_xy | | ≤ `[1 %]` of I_xx | Limits cross-coupling torque with pitch rate |
| Product of inertia I_xz | | ≤ `[1 %]` of I_xx | Same as above |
| Principal-axis misalignment | θ_p | ≤ `[0.5]°` | Avoids wobble under fast slews |
| Pitch inertia I_xx | | within ±`[3 %]` of control-model value | Preserves designed loop bandwidth and phase margin |
| First structural mode | f₁ | ≥ `[4×]` the pitch loop crossover | Avoids exciting cradle compliance |

**Frame convention:**
- Origin on the pitch axis at the motor's mid-bearing plane.
- **x** lies along the pitch axis.
- **z** lies along the optical axis at 0° tilt.
- **y** completes the right-handed set.

---

## 2. Theory Basis

**Static balance.** Gravity torque about the pitch axis at tilt angle θ is:

$$\tau_g(\theta) = m g \left( r_{cg,y}\cos\theta \;-\; r_{cg,z}\sin\theta \right)$$

Static balance requires r_cg,y = r_cg,z = 0. This needs two independent corrections: a fore/aft (z) slide and a vertical (y) slide.

**Dynamic balance.** The inertia tensor about the pitch-axis origin is:

$$\mathbf{I} = \begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz} \\ -I_{xy} & I_{yy} & -I_{yz} \\ -I_{xz} & -I_{yz} & I_{zz}\end{bmatrix}$$

For rotation ω = ω_x·x̂, the angular momentum is H = I·ω. Off-axis torque appears when I_xy or I_xz is nonzero:

$$\boldsymbol{\tau}_{\perp} = \omega_x^2 \begin{bmatrix}0\\ I_{xz}\\ -I_{xy}\end{bmatrix} + \dot\omega_x \begin{bmatrix}0\\ -I_{xy}\\ -I_{xz}\end{bmatrix}$$

The roll and yaw motors must react this torque, and the outer-frame bearings carry it as rotating loads. The goal is to make the pitch axis a principal axis.

**Mass-addition model.** A point trim mass m_i at position (x_i, y_i, z_i) changes the system as follows:

- Δ(m·r_cg) = m_i·(x_i, y_i, z_i)
- ΔI_xx = m_i·(y_i² + z_i²)
- ΔI_xy = m_i·x_i·y_i
- ΔI_xz = m_i·x_i·z_i

Two key consequences:
- **Products of inertia need axial lever arm (x_i).** You correct I_xy and I_xz with masses placed in pairs on opposite sides along x.
- **Static balance can be corrected with masses near x ≈ 0**, which leaves the products of inertia almost untouched.

---

## 3. Pre-Balance Preparation

1. **Configuration freeze.** Install the flight-representative payload:
   - camera body, lens at its operational focus/zoom setting, filter, cables, and connectors
   - cable service loops dressed and tied exactly as in operation
2. **Record the configuration** in the Cycle 24 log: lens ID, focus/zoom position, and any changes since Cycle 23 (BOM delta).
3. **Update the CAD mass model** with measured component masses, weighed on a ±0.01 g scale. Compute the predicted r_cg and **I**. This is the *a priori* estimate.
4. **Check bearing friction.** Unpowered breakaway torque must be under `[0.5]` mN·m. Friction masks residual imbalance, so if it is out of spec, fix it before balancing.
5. **Fixture.** Mount the cradle on its own pitch bearings in a fixture that lets you rotate the pitch axis to horizontal or vertical. The roll and yaw axes stay locked.

---

## 4. Static Balancing Procedure

### 4.1 Coarse balance (manual)
1. Disable the motor, set the pitch axis horizontal, and release the cradle at θ = 0°.
2. **Optical axis level (θ = 0°):** if the cradle rotates, r_cg,y ≠ 0 (the CG is above or below the axis). Adjust the vertical slide until the cradle holds position.
3. Rotate to θ = 90° (lens pointing up or down) and release. Drift here indicates r_cg,z ≠ 0. Adjust the fore/aft slide.
4. Iterate steps 2–3 until the cradle holds at θ = 0°, 45°, 90°, and 135° within the friction deadband.

### 4.2 Fine balance (motor-current method)
1. Enable the pitch loop in position mode with feedforward and integrator logging.
2. Step θ from −90° to +90° in 15° increments. Dwell `[3]` s at each step and record the mean phase current i_q(θ).
3. Convert current to torque: τ(θ) = K_t·i_q(θ). Subtract the friction component, either by averaging forward and reverse sweeps or with a Coulomb model.
4. Fit the model by least squares:

$$\tau(\theta) = A\cos\theta + B\sin\theta + C$$

   - A = m·g·r_cg,y
   - B = −m·g·r_cg,z
   - C = cable spring or bias torque, which you should note separately
5. Compute the required correction. For trim mass m_t on the y-slide:

$$\Delta y = -\frac{m\, r_{cg,y}}{m_t} \quad\text{(or move the slide by } -r_{cg,y}\text{ if the whole payload translates)}$$

   Apply the same approach for z.
6. Apply the correction, repeat the sweep, and confirm both A and B are within tolerance.
7. **Cable torque check.** If |C| or a non-sinusoidal residual exceeds `[1]` mN·m, re-route the service loop. Do not compensate cable torque with mass.

---

## 5. Inertia Tensor Identification

Use at least two independent methods and cross-check them.

### 5.1 Pitch inertia I_xx (dynamic identification)
- **Method A, torsional pendulum.** Hang the cradle with the pitch axis vertical on a calibrated torsion wire or bifilar suspension. Then:
  - I = k_θ·T²/(4π²)
  - Subtract the fixture inertia, measured separately.
- **Method B, in-situ chirp.** Apply a current chirp from `[1–100]` Hz in open loop or with a low-bandwidth position hold. Fit the transfer function θ/τ = 1/(I_xx·s² + b·s). Above the friction region, I_xx is read from the −40 dB/dec asymptote.
- **Agreement criterion:** the two methods must agree within `[3 %]`. Otherwise, investigate fixture compliance or K_t error.

### 5.2 Products of inertia I_xy and I_xz
- **Method A, spin balancing.** Spin the cradle about the pitch axis at constant ω on a two-plane balancing rig with force sensors at both bearings. The rotating moment amplitude is:

$$M = \omega^2\sqrt{I_{xy}^2+I_{xz}^2}$$

  The phase of M gives the ratio between I_xy and I_xz. The two-plane force readings also give residual static imbalance, which serves as a cross-check of Section 4.
- **Method B, in-gimbal reaction measurement.** Slew pitch with a trapezoidal profile at `[300]`°/s and `[2000]`°/s². Log i_q on the roll and yaw motors with their loops in a stiff hold. Fit:
  - acceleration-phase reaction torques ≈ −I_xy·ω̇ and −I_xz·ω̇
  - cruise-phase reaction torques ≈ ω²·I_xz and −ω²·I_xy
- **Method C, CAD plus measured deltas.** Use this as a fallback or sanity check only.

### 5.3 Full tensor (optional, for control-model update)
Use six or more torsional-pendulum orientations, or a multi-axis inertia measurement machine. Solve the over-determined linear system for the six unique tensor elements, then eigen-decompose to get the principal moments and axes.

---

## 6. Tensor Tuning (Dynamic Balance Correction)

### 6.1 Correction-mass solution
Choose trim mass stations: at least two axial planes x = ±L at radius R, with tapped holes or a T-slot that allows angular positioning.

Solve for the masses m_k at stations (x_k, y_k, z_k) that satisfy all of these together:

$$\sum_k m_k x_k y_k = -I_{xy}, \quad \sum_k m_k x_k z_k = -I_{xz}, \quad \sum_k m_k y_k = 0, \quad \sum_k m_k z_k = 0$$

The first two equations remove the products of inertia. The last two preserve the static balance from Section 4.

**Standard solution: an antisymmetric couple pair.**
- Place mass m at (+L, r·cos φ, r·sin φ).
- Place an equal mass at (−L, −r·cos φ, −r·sin φ).
- This pair adds zero static moment, so static balance is preserved.
- Its effect on the products of inertia is ΔI_xy = 2·m·L·r·cos φ and ΔI_xz = 2·m·L·r·sin φ.

Solve for the magnitude and angle:

$$\phi = \operatorname{atan2}(-I_{xz},\, -I_{xy}), \qquad m = \frac{\sqrt{I_{xy}^2+I_{xz}^2}}{2Lr}$$

### 6.2 I_xx tuning (control matching)
- Each trim pair adds ΔI_xx = 2·m·r².
- If I_xx must be raised to match the control model (for example, after a lens swap made it lighter), add symmetric ring masses at x ≈ 0. These change only I_xx.
- If I_xx must be lowered, the options are:
  - re-tune the controller instead, which is preferred, or
  - move dense components (such as the battery or PCB) radially inward. This is a design change and should be flagged as a Cycle 25 action.

### 6.3 Iterate
1. Install the trim masses.
2. Repeat static verification (Section 4.2, quick 5-point sweep).
3. Repeat the product-of-inertia measurement (Section 5.2).
4. Converge until all criteria in Section 1 pass, with a maximum of `[3]` iterations. If not converged after that, perform root-cause analysis: look for loose components, mass model errors, or fixture problems.

---

## 7. Verification

1. **Static torque sweep:** −90° to +90°, residual within spec at every point.
2. **Dynamic slew test:** at the max-rate profile, roll and yaw reaction currents stay below `[X]` A.
3. **Closed-loop check:**
   - Measure the pitch loop's open-loop transfer function.
   - Confirm crossover is within ±`[5 %]` of design and phase margin is ≥ `[45]`°.
   - Confirm no new resonance appears below f₁.
4. **Environmental spot-check:** repeat the static sweep at `[−10 °C / +45 °C]` if lens or material CTE could shift the CG.
5. **Torque-stripe** all trim fasteners and photograph the final configuration.

---

## 8. Cycle 24 Record Template

| Item | Cycle 23 | Cycle 24 Pre | Cycle 24 Post | Pass/Fail |
|---|---|---|---|---|
| Payload mass m (g) | | | | – |
| r_cg,y (mm) | | | | |
| r_cg,z (mm) | | | | |
| r_cg,x (mm) | | | | |
| Cable bias C (mN·m) | | | | |
| I_xx (kg·mm²), method A / B | | | | |
| I_xy (kg·mm²) | | | | |
| I_xz (kg·mm²) | | | | |
| Principal-axis misalignment θ_p (°) | | | | |
| Trim masses (mass, location) | | – | | – |
| Loop crossover / phase margin | | | | |
| Configuration changes since last cycle | | | | – |

**Trend note:** compare Cycle 24 to Cycle 23. If the required trim mass has grown for two consecutive cycles, raise a design action (for example, re-centering the payload mount) rather than adding more ballast. Added ballast raises inertia, reduces bandwidth, and increases power draw.

---

If you share the actual geometry, payload mass, motor K_t, or Cycle 23 results, I can compute the specific trim-mass values and correction angles.