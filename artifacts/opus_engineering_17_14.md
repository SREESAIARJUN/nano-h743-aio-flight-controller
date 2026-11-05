# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning Procedure (Cycle 17)

> **Note on scope:** I don't have your program's Cycle 17 baseline data, drawings, or spec limits. The procedure below is complete and self-consistent, but every numeric limit marked **[TBD]** should be replaced with your program values. Where I give example figures, they are typical for small-to-mid electro-optical gimbals and should be treated as placeholders.

---

## 1. Objectives

1. **Static balance:** Place the cradle's center of mass (CM) on the pitch rotation axis, so that gravity and linear acceleration produce no torque about pitch.
2. **Dynamic balance:** Align a principal axis of inertia with the pitch axis, so that pitch rotation produces no reaction torques in roll or yaw.
3. **Inertia tuning:** Set the pitch moment of inertia I_xx to meet the following targets:
   - torque and acceleration budget,
   - control bandwidth,
   - structural resonance separation.
4. **Documentation:** Record the as-balanced state and its delta from Cycle 16.

---

## 2. Frame and Definitions

- **Origin O:** The intersection of the pitch axis with the cradle's mid-plane between the bearings.
- **x-axis:** The pitch rotation axis.
- **z-axis:** The optical boresight at θ = 0.
- **y-axis:** Completes a right-handed frame.

Inertia tensor about O:

$$
\mathbf{I}_O=\begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz}\\ -I_{xy} & I_{yy} & -I_{yz}\\ -I_{xz} & -I_{yz} & I_{zz}\end{bmatrix},\qquad I_{xy}=\textstyle\sum m_i x_i y_i \text{ (etc.)}
$$

Other quantities:

- **m:** Total rotating mass, including the camera, lens, cables (rotating portion), fasteners, and the motor rotor.
- **CM offset:** **r_c** = (x_c, y_c, z_c). Only y_c and z_c generate pitch torque.

**Why each quantity matters:**

| Quantity | Physical effect if nonzero / mistuned |
|---|---|
| y_c, z_c | Gravity torque τ_g(θ) about pitch. Base vibration couples into pitch jitter. |
| x_c | Not a pitch issue. It loads the bearings asymmetrically and affects the roll/yaw stage balance. |
| I_xy, I_xz | Pitch rate and acceleration produce reaction torques on the roll/yaw axes (cross-coupling, jitter). |
| I_xx | Sets required motor torque, achievable bandwidth, and the structural mode f = (1/2π)√(k/I_xx). |

---

## 3. Pre-Balance Preparation

1. **Freeze the configuration.** Install the flight camera body, the lens at its operational focus/zoom setting, filters, lens caps removed, and the flight cable harness routed and tied as in service.
   - Zoom lenses shift CM. Balance at the **most frequently used zoom position**, and also record CM at both extremes.
2. **Mechanical checks:**
   - Torque all fasteners to spec.
   - Verify bearing preload.
   - Measure friction torque τ_f with the motor de-energized: rotate slowly and read with a torque gauge or by motor current at very low rate. Record breakaway and running friction.
3. **CAD prediction.** Export m, **r_c**, and **I_O** from CAD for the Cycle 17 configuration. Compare against Cycle 16 as-measured values and note expected deltas from any hardware changes.
4. **Encoder and current sensing.** Confirm calibration of the motor torque constant K_t (N·m/A) at operating temperature.

---

## 4. Static Balance

### 4.1 Theory

With pitch angle θ, the gravity torque about the pitch axis is:

$$
\tau_g(\theta)= m g\,(z_c\sin\theta + y_c\cos\theta)
$$

The sign convention depends on the frame; fix it once and keep it. Measuring at two orthogonal attitudes separates the two offsets.

### 4.2 Coarse Balance (Unpowered)

1. Unlock the pitch axis with the motor disconnected.
2. Release the cradle at θ = 0°, ±45°, and ±90°. If it rotates toward a consistent "heavy side," adjust the slide or trim weights.
3. Iterate until the cradle stays put at all attitudes within the friction dead-band.
   - Coarse balance cannot resolve offsets smaller than e_min ≈ τ_f / (m g). This is why the powered method follows.

### 4.3 Fine Balance (Powered, Current-Based)

1. Close the position loop. Hold at θ = 0°, then θ = 90°, with the gimbal base level.
2. At each attitude:
   - Record the mean motor current over ≥ 5 s.
   - Approach the attitude once from each direction and average the two readings. This cancels Coulomb friction.
3. Compute torques with τ = K_t · I_avg, then solve for the offsets:

$$
y_c = \frac{\tau(0^\circ)}{m g},\qquad z_c = \frac{\tau(90^\circ)}{m g}
$$

4. For better accuracy, sweep θ in 15° steps and least-squares fit τ(θ) = A cos θ + B sin θ + C. Then y_c = A/(mg) and z_c = B/(mg). C captures cable spring torque and residual bias; record it separately.

### 4.4 Trim Mass Calculation

For a trim mass m_t placed at (y_t, z_t), the following condition nulls the offset:

$$
m\,y_c + m_t\,y_t = 0,\qquad m\,z_c + m_t\,z_t = 0
$$

Prefer moving existing components first (camera slide, battery, counterweight rail). Adding mass also raises I_xx; see Section 6.

### 4.5 Acceptance Criteria

| Parameter | Criterion [TBD] |
|---|---|
| Residual static torque | ≤ 5% of continuous motor torque. Typical: ≤ 0.002–0.01 N·m for small payloads. |
| Equivalent CM offset | ≤ e.g. 0.1 mm |
| Cable bias torque C | Recorded. Re-route if > 50% of the residual allowance. |

---

## 5. Dynamic Balance (Products of Inertia)

### 5.1 Theory

For rotation about x with rate ω and acceleration α, the reaction torques on the support are:

$$
M_y = -I_{xy}\,\alpha + I_{xz}\,\omega^2,\qquad M_z = -I_{xz}\,\alpha - I_{xy}\,\omega^2
$$

Signs depend on the convention; magnitude is what matters here. Nonzero I_xy and I_xz therefore inject torque into roll and yaw at every pitch slew.

### 5.2 Measurement

Choose one method:

- **(a) Instrumented test (preferred):**
  1. Mount the cradle on a 6-axis load cell, or lock the roll and yaw motors and read their holding currents.
  2. Drive pitch with a sinusoid θ = Θ sin(Ωt). Use several frequencies, e.g. 1, 2, and 5 Hz [TBD].
  3. Regress the measured M_y and M_z against α and ω² to extract I_xy and I_xz.
- **(b) Spin test:**
  1. Run continuous pitch rotation at constant ω, if the cable wrap allows.
  2. Measure the once-per-rev support force at each bearing. Two-plane balancing yields the corrections.
- **(c) CAD + verification:** Use CAD values for I_xy and I_xz, and verify with method (a) at a single frequency.

### 5.3 Correction with Paired Masses

Use a **mass pair** so the CM is not disturbed. Place mass m_p at (x_1, y_1, z_1) and an equal mass at (−x_1, −y_1, −z_1). The first moments cancel, so the CM is unchanged, and the pair adds:

$$
\Delta I_{xy} = 2 m_p x_1 y_1,\qquad \Delta I_{xz} = 2 m_p x_1 z_1
$$

To null the measured values, choose:

$$
2m_p x_1 y_1 = -I_{xy},\qquad 2m_p x_1 z_1 = -I_{xz}
$$

Maximize x_1, i.e. place the pair near the bearing planes, to minimize the added mass. The pair also adds ΔI_xx = 2m_p(y_1² + z_1²), so account for it in Section 6.

### 5.4 Acceptance Criteria

| Parameter | Criterion [TBD] |
|---|---|
| \|I_xy\|, \|I_xz\| | ≤ 1–2% of I_xx |
| Principal axis misalignment | ≤ e.g. 0.5° |
| Cross-axis reaction at max slew | ≤ X% of roll/yaw torque budget |

The principal axis misalignment can be estimated as:

$$
\phi \approx \tfrac12\arctan\!\frac{2\sqrt{I_{xy}^2+I_{xz}^2}}{I_{xx}-I_{\perp}}
$$

---

## 6. Pitch Inertia (I_xx) Tuning

### 6.1 Design Constraints

1. **Torque and acceleration:**

$$
\tau_{req} = I_{xx}\,\alpha_{max} + \tau_f + \tau_{res} + \tau_{cable}
$$

   This must be ≤ ~70% of the motor's continuous torque, leaving margin for disturbances.

2. **Disturbance rejection.** Base angular disturbances couple through friction and residual imbalance. Larger I_xx improves passive inertial stabilization, since angular response ∝ τ_dist / I_xx. Treat this as the lower bound on I_xx.

3. **Control bandwidth.** For a rigid-body plant, the crossover frequency is ω_c ≈ √(K_t K_p / I_xx) (PD approximation). Ensure the tuned I_xx still permits the required ω_c [TBD Hz].

4. **Structural mode separation:**

$$
f_1 = \frac{1}{2\pi}\sqrt{\frac{k_{tors}}{I_{xx}}} \;\ge\; (5\text{–}10)\times f_{c}
$$

   Here k_tors is the torsional stiffness from the motor rotor to the camera mount.

5. **Rotor/load inertia ratio** (direct drive). Keep the ratio within the range your controller was tuned for, e.g. load/rotor ≤ 10:1 [TBD].

Together these define an allowable window: **I_xx,min ≤ I_xx ≤ I_xx,max**.

### 6.2 Measuring I_xx

Choose one method:

- **(a) Compound pendulum:**
  1. Start with a statically balanced cradle.
  2. Attach a known mass m_a at radial distance d from the axis.
  3. Measure the small-amplitude oscillation period T over ≥ 20 cycles, with the motor disconnected and low-friction bearings.
  4. Compute:

$$
I_{xx} = \frac{m_a g d\,T^2}{4\pi^2} - m_a d^2
$$

  5. Repeat with two values of d and average.
- **(b) Dynamic identification:**
  1. Drive a chirp or sine-sweep torque.
  2. Fit τ = I_xx α + b ω + τ_f sgn(ω) + τ_g(θ) by least squares.
  3. This also yields viscous damping b, which is useful for controller tuning.
- **(c) Torsional pendulum:** Use a calibrated torsion wire (I = k T² / 4π²) if the cradle can be removed from the gimbal.

### 6.3 Adjusting I_xx

- **To raise I_xx without disturbing balance:** Add symmetric masses diametrically opposed in the y–z plane at the same x, i.e. at (x, ±r, 0) or (x, 0, ±r). These cancel first moments and products of inertia.
- **To lower I_xx:**
  - Relocate mass toward the axis.
  - Use lighter counterweights by moving the payload closer to the axis rather than counterbalancing at long arms.
  - Use lighter materials for brackets.
- **After any change:** Re-verify Section 4 (static) and Section 5 (dynamic). They are coupled through mass placement.

### 6.4 Iteration Loop

1. Coarse static balance
2. Fine static balance (powered)
3. Measure I_xy and I_xz, then apply paired-mass correction
4. Measure I_xx
5. If I_xx is outside its window, apply symmetric mass changes and return to step 2.
6. Repeat until all criteria pass in a single consecutive verification run.

---

## 7. Controller Re-Tuning Hand-off

Provide the following to the controls team:

- Measured I_xx, b, τ_f, and residual τ_g amplitude and phase.
- Updated plant gain. If I_xx changed by more than ~10% from Cycle 16, rescale the loop gains: K_p, K_d ∝ I_xx to preserve bandwidth.
- Notch filter re-check against the measured f_1 (from a tap test or swept-sine on the cradle).

---

## 8. Environmental Verification (Recommended)

- **Temperature:** Repeat the fine static balance at hot and cold operating extremes. Thermal expansion and lens element shifts move the CM.
- **Vibration:** Repeat after vibration exposure to confirm trim masses didn't shift. Use thread-locker or staking on all trim hardware.
- **Zoom/focus extremes:** Record residual torque at each. If the variation exceeds the allowance, consider active CM compensation or a feed-forward gravity term in the controller.

---

## 9. Cycle 17 Record Sheet (Template)

| Item | Cycle 16 | Cycle 17 CAD | Cycle 17 As-Built | Limit | Pass/Fail |
|---|---|---|---|---|---|
| Rotating mass m (kg) | | | | | |
| y_c (mm) | | | | ≤ [TBD] | |
| z_c (mm) | | | | 