# Pitch-Axis Camera Cradle: Balancing Procedure and Inertia Tensor Tuning

**Note on "Cycle 29":** I don't have your program's documentation, so I can't know what payload, tolerances, or prior-cycle findings Cycle 29 refers to. Below is a complete, general procedure. Replace the bracketed placeholders with your cycle-specific values.

---

## 1. Objectives and Definitions

**Coordinate frame (cradle body frame, origin at pitch bearing axis):**
- **x**: pitch rotation axis (through both bearings)
- **y**: horizontal, perpendicular to x, in the lens-forward plane at zero pitch
- **z**: completes the right-handed set (up at zero pitch)
- **y'**: optical axis direction (often aligned with y at θ = 0)

**Goals, in priority order:**
1. **Static balance:** The center of mass (CoM) lies on the pitch axis, so y_cm = z_cm = 0. This makes gravity torque zero at every pitch angle.
2. **Axial CoM placement:** x_cm is centered between the bearings. This equalizes bearing loads and minimizes the moment transferred to the roll and yaw stages.
3. **Dynamic balance:** The products of inertia coupling pitch to the other axes are zero, I_xy = I_xz = 0. This means pitch rotation produces no reaction torques on roll or yaw.
4. **Inertia shaping:** I_xx falls within the motor/controller design window. Optionally, I_yy ≈ I_zz, so the cradle's contribution to outer-axis inertia does not vary with pitch angle.

---

## 2. Governing Relations

**Inertia tensor about the pitch-axis origin O:**

$$
\mathbf{I}_O=\begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz}\\ -I_{xy} & I_{yy} & -I_{yz}\\ -I_{xz} & -I_{yz} & I_{zz}\end{bmatrix},\quad I_{xy}=\int xy\,dm,\ \text{etc.}
$$

**Gravity torque about the pitch axis:**

$$
\tau_g(\theta)= m g\,\big(y_{cm}\cos\theta - z_{cm}\sin\theta\big)\quad(\text{sign per frame convention})
$$

Static balance requires y_cm = z_cm = 0. In that case τ_g ≡ 0 for all θ.

**Cross-axis reaction torque during pitch motion** (with the outer axes held fixed):

$$
\boldsymbol{\tau}_{react} = \begin{bmatrix}I_{xx}\ddot\theta\\ -I_{xy}\ddot\theta + I_{xz}\dot\theta^2\\ -I_{xz}\ddot\theta - I_{xy}\dot\theta^2\end{bmatrix}
$$

Nonzero I_xy or I_xz therefore injects disturbance into roll and yaw. I_yz does not couple pitch motion to the other axes in this frame. It does, however, affect the angle dependence of the outer-axis inertia.

**Outer-axis inertia as a function of pitch angle:**

$$
I_{\text{outer}}(\theta) = \tfrac{I_{yy}+I_{zz}}{2} + \tfrac{I_{yy}-I_{zz}}{2}\cos 2\theta - I_{yz}\sin 2\theta
$$

Setting I_yy = I_zz and I_yz = 0 makes this constant. That benefits roll and yaw loop gain scheduling.

**Parallel-axis transfer** (for adding a trim mass m_i at r_i = (x_i, y_i, z_i)):
- ΔI_xx = m_i(y_i² + z_i²)
- ΔI_xy = m_i x_i y_i
- ΔI_xz = m_i x_i z_i
- ΔI_yz = m_i y_i z_i
- Δ(m·r_cm) = m_i·r_i

---

## 3. Procedure

### Phase A: Preparation
1. Configure the flight payload exactly: camera body, lens [model], focus/zoom position [specify; zoom shifts CoM], filters, cables with service loops dressed and tie-wrapped, and any lens motors.
2. Record the payload mass m and the cradle mass. Note the variation across zoom range and any lens-cap or hood options.
3. Disable or disengage the pitch motor so the axis rotates freely. Verify bearing friction τ_f with a spring gauge or by measuring motor current at slow constant speed. Target τ_f below [X] mN·m. Friction sets the resolution floor of the static test.
4. Lock the roll and yaw axes. Level the outer frame to within 0.1°.

### Phase B: Coarse Static Balance (z, the vertical offset)
1. Set the cradle to θ = 0 (lens horizontal) and release it.
2. If it rotates nose-down or nose-up, adjust the camera **fore/aft (y)** on the dovetail until it stays put.
3. Rotate the cradle to θ = 90° (lens pointing up) and release it.
4. If it falls forward or back, adjust **vertical (z)** with risers or the cradle height slide.
5. Iterate steps 1–4. The two adjustments are orthogonal in principle but interact through slide geometry. Usually 2–3 passes suffice.

### Phase C: Fine Static Balance (quantitative)
1. With the motor enabled under a slow, constant-velocity, low-bandwidth controller, sweep θ from −90° to +90° and back. Log motor current i(θ).
2. Convert current to torque: τ = K_t·i. Average the forward and reverse sweeps to cancel Coulomb friction.
3. Fit the averaged torque to τ(θ) = A cos θ + B sin θ + C.
   - y_cm = A/(mg)
   - z_cm = −B/(mg)
   - C is a residual bias from cable spring or a sensor offset.
4. Correct with slide adjustment or trim masses: Δy = −y_cm·m/m_trim.
5. **Acceptance:** peak |τ_g| ≤ [e.g., 2–5 % of continuous motor torque] or |r_cm| ≤ [e.g., 0.2 mm].
6. **Cable check:** If C or a cos/sin residual varies with sweep direction beyond what friction explains, the cable harness is acting as a spring. Re-route the harness rather than trimming mass against it.

### Phase D: Axial CoM (x)
1. Measure the bearing reaction loads, either with a load cell under each bearing block or with a knife-edge balance of the removed cradle.
2. Shift the payload laterally, or add trim to the light side, until x_cm is within ± [e.g., 1 mm] of the bearing midplane.
3. Re-verify Phase C, since lateral trim must sit at y = z = 0 to avoid disturbing static balance.

### Phase E: Inertia Tensor Determination
Use any of three methods, in order of fidelity.

**E1. CAD baseline.** Export I_O from the assembly with measured, not nominal, component masses. Correct the camera and lens density using measured mass and CoM.

**E2. Pitch-axis I_xx by system identification.**
- Inject a chirp or PRBS torque, and fit G(s) = 1/(J s² + b s). Then I_xx = J − J_rotor − J_cradle_hardware, as appropriate.
- Alternatively, use a torsional pendulum: suspend the payload on a wire of known stiffness k and measure the period T, giving I = kT²/(4π²).

**E3. Full tensor via a trifilar or bifilar pendulum.** Measure the moment of inertia about at least 6 independent axes through the CoM, using direction cosines n_k. Solve the least-squares problem I_k = n_kᵀ I n_k for the 6 unknowns. Use 9–12 orientations for redundancy.

**In-situ products of inertia.** Lock roll and yaw through torque sensors, or use the roll and yaw motors in current-hold mode. Command a pitch acceleration profile and read the cross-axis reactions:
- Under pure acceleration: I_xy ≈ −τ_y/θ̈ and I_xz ≈ −τ_z/θ̈.
- Constant-rate segments separate out the θ̇² terms as a check.

### Phase F: Tensor Tuning (Dynamic Balance)
Products of inertia can only be nulled without breaking static balance by using **trim pairs**. Place two equal masses m_t at symmetric points r and −r relative to the CoM along the chosen direction. Their net first moment is zero, but they change the products of inertia.

**To null I_xy:** place the pair at (+a, +b, 0) and (−a, −b, 0), with sign chosen to oppose the measured I_xy.

$$
2 m_t a b = -I_{xy}^{meas}
$$

**To null I_xz:** place the pair at (±a, 0, ±c), using the same logic: 2 m_t a c = −I_xz.

**Design guidance:**
- Maximize the lever arms a, b, c (use the cradle side-plate extremities) to minimize added mass.
- Each pair adds 2m_t(b²) or 2m_t(c²) to I_xx. Check that this stays within the motor torque and bandwidth budget.

**Optional isotropy tuning (I_yy ≈ I_zz, I_yz → 0).** Add symmetric masses on the y or z axis on both sides of the origin. Pairs at (0, ±d, 0) raise I_zz and I_xx. Pairs at (0, 0, ±d) raise I_yy and I_xx. Null I_yz with a diagonal pair at (0, ±e, ±e).

**Solve as a constrained optimization:**

$$
\min_{m_t,\ \mathbf r_t} \sum m_t \quad \text{s.t.}\ \ |I_{xy}|,|I_{xz}|\le\epsilon_P,\ \ \mathbf r_{cm}=0,\ \ I_{xx}\in[I_{min},I_{max}],\ \ \text{mount-point constraints}
$$

Restrict r_t to the discrete set of available tapped mounting points. This makes it a small integer or least-squares problem that a script can solve.

### Phase G: Verification
1. Repeat Phase C. Static balance must still pass.
2. Repeat the cross-axis reaction test. Pass if |I_xy|, |I_xz| ≤ [ε_P, e.g., 1–2 % of I_xx].
3. Re-identify I_xx and retune the pitch loop. Inertia changes shift the crossover frequency in proportion to 1/J. Rescale the gains: K_new = K_old·(J_new/J_old).
4. Run a closed-loop disturbance test. Apply a step pitch command at max slew and confirm the roll and yaw error peaks are below [spec].
5. Sweep the zoom/focus range and repeat a quick static check. Document the worst-case drift and decide whether to balance at mid-zoom or at the most-used setting.

---

## 4. Acceptance Criteria Template

| Parameter | Target | Measured | Pass |
|---|---|---|---|
| Static gravity torque, peak | ≤ [ ] mN·m | | |
| y_cm, z_cm | ≤ [ ] mm | | |
| x_cm offset from bearing midplane | ≤ [ ] mm | | |
| I_xx | [ ] ± [ ] kg·m² | | |
| \|I_xy\|/I_xx, \|I_xz\|/I_xx | ≤ [ ] % | | |
| (I_yy − I_zz)/I_yy (if required) | ≤ [ ] % | | |
| Bearing friction | ≤ [ ] mN·m | | |
| Total trim mass added | ≤ [ ] g | | |
| Cross-axis error under max pitch slew | ≤ [ ] ° | | |

---

## 5. Common Pitfalls
- **Balancing with the wrong configuration.** A different lens hood, cable, or zoom position invalidates the result.
- **Cable stiffness masquerading as imbalance.** Always check direction dependence of the residual torque.
- **Single-mass trim for dynamic balance.** It fixes a product of inertia but breaks static balance. Always use pairs.
- **Forgetting to retune the pitch loop** after inertia changes. This shows up as reduced phase margin or sluggishness.
- **Friction-limited static balance.** If τ_f exceeds the imbalance resolution, use the current-sweep method (Phase C), not release tests.

---

If you share the Cycle 29 specifics (payload mass and lens, the motor's K_t and continuous torque, current tensor estimates, available trim mounting points, and the prior cycle's residuals), I can compute the actual trim-mass placements and the expected post-tuning tensor.