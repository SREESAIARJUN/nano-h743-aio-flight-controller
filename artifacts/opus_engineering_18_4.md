# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning Procedure

**Scope note:** I don't have access to your program's Cycle 18 baseline (payload mass, motor spec, prior cycle results, or acceptance limits). This is a complete procedure template. Bracketed items like **[TBD]** should be filled from your configuration data. Numerical limits shown are typical starting values, not program requirements.

---

## 1. Coordinate Frame and Definitions

Define the **cradle frame {C}**, fixed to the pitch-rotating assembly:

- **Origin O:** intersection of the pitch bearing axis with the bearing mid-plane.
- **y-axis:** along the pitch rotation axis.
- **x-axis:** along the camera optical axis at the zero-pitch position.
- **z-axis:** completes the right-handed set.

**Rotating mass properties** (everything that rotates about pitch: camera, lens, cradle, motor rotor, encoder disk, cabling share):

- Mass: m
- Center of mass: **r**_cm = [x_c, y_c, z_c]ᵀ
- Inertia tensor about O:

$$
\mathbf{I}_O=\begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz}\\ -I_{xy} & I_{yy} & -I_{yz}\\ -I_{xz} & -I_{yz} & I_{zz}\end{bmatrix}
$$

**Balancing objectives:**

| Objective | Condition | Physical consequence if violated |
|---|---|---|
| Static balance | x_c = z_c = 0 | Gravity torque τ_g = m·g·d·cos(θ+φ), plus sensitivity to base acceleration |
| Dynamic balance | I_xy = I_yz = 0 | Pitch acceleration couples into roll and yaw reaction torques; principal axis tilts off the pitch axis |
| Axial CoM placement | y_c ≈ 0 (or a specified value) | Bearing moment loading and roll-axis imbalance at the outer gimbal |
| Inertia target | I_yy within [TBD] band | Sets plant gain 1/(J·s²) and achievable control bandwidth |

---

## 2. Phase A: Analytical Baseline (CAD)

1. Assign verified material densities. Weigh all purchased parts (camera body, lens at the specified focus/zoom state, filters, cables) and override the CAD masses.
2. Export m, **r**_cm, and **I**_O in frame {C}.
3. Compute the predicted residuals:
   - Static imbalance: U_s = m·√(x_c² + z_c²)  [g·mm]
   - Products of inertia: I_xy and I_yz
   - Principal-axis misalignment: the angle between the pitch axis and the nearest principal axis, from the eigendecomposition of **I**_O.
4. Check structural modes. The first cradle and lens-mount mode should be at least **3–5×** above the planned pitch-loop crossover frequency **[TBD Hz]**.
5. Record the configuration state for each payload variant (lens/zoom/focus changes shift the CoM). Balance either to a nominal state or to the centroid of the CoM envelope, and state which one.

---

## 3. Phase B: Mass Property Measurement

### B1. Mass
Weigh the full rotating assembly on a calibrated scale. Record the resolution and uncertainty.

### B2. Static imbalance: gravity-torque method (in situ)
Use this with the motor installed and the axis horizontal:

1. Command slow quasi-static positions θ_k over ±[TBD]° (at least 12 points), dwelling long enough to settle at each.
2. Record the holding torque τ_k = K_t·i_k. Average it to remove ripple, and take readings in both approach directions so friction cancels.
3. Fit the model

$$
\tau(\theta)=A\cos\theta + B\sin\theta + \tau_{f}\,\mathrm{sgn}(\dot\theta) + c
$$

   where A = m·g·x_c and B = m·g·z_c (adjust signs to your conventions).
4. Solve: x_c = A/(m·g) and z_c = B/(m·g). Report the fit residual and the cogging contribution (identify cogging separately with the payload removed if possible).

**Alternative:** use a knife-edge or air-bearing balance fixture for pre-motor integration.

### B3. Pitch moment of inertia, I_yy
Choose one method:

- **Torsional pendulum:** J = k_t·T²/(4π²). Calibrate k_t with a reference body of known inertia, then subtract the fixture tare.
- **Trifilar pendulum** (CoM placed on the pendulum axis): I = m·g·R²·T²/(4π²·L).
- **In situ system identification:** apply a chirp or PRBS torque, fit J·θ̈ + b·θ̇ + τ_g = τ, and cross-check against the pendulum value.

Acceptance: agreement within **[TBD, typically ±3%]** of CAD. Investigate any larger discrepancy before trimming.

### B4. Products of inertia (I_xy, I_yz)
- **Preferred:** a mass-properties instrument (spin balance or multi-orientation inertia table). Measure the full tensor in at least six orientations and solve by least squares.
- **In situ alternative:** drive a pitch acceleration profile and measure the reaction torques at the roll and yaw axes (motor current with those axes locked, or a 6-axis load cell at the base). For pure angular acceleration α about y:

$$
M_x = -I_{xy}\,\alpha + I_{yz}\,\omega^2,\qquad M_z = -I_{yz}\,\alpha - I_{xy}\,\omega^2
$$

  Signs depend on your convention. Separate the α-proportional and ω²-proportional terms by regression.

---

## 4. Phase C: Trim Solution

### C1. Trim mass effects
Adding a point mass Δm_j at position **p**_j = [p_x, p_y, p_z]ᵀ in frame {C} changes the properties as follows.

First moment:

$$
\Delta(m\,\mathbf{r}_{cm}) = \Delta m_j\,\mathbf{p}_j
$$

Inertia (parallel-axis tensor form, plus the trim mass's own inertia I_j):

$$
\Delta\mathbf{I}_O = \Delta m_j\left(\|\mathbf{p}_j\|^2\mathbf{E}_3 - \mathbf{p}_j\mathbf{p}_j^\top\right) + \mathbf{I}_j
$$

Specifically:

- ΔI_xy = Δm·p_x·p_y
- ΔI_yz = Δm·p_y·p_z
- ΔI_yy = Δm·(p_x² + p_z²)

Key insight: **static trim requires p_x or p_z ≠ 0, while dynamic (product) trim requires axial separation p_y ≠ 0.** Correcting both therefore needs at least **two trim planes** at different y-stations, typically one on each side cheek of the cradle.

### C2. Optimization
Choose the decision variables based on your hardware:

- Discrete trim masses at predefined threaded stations, or
- Continuous slide positions (sliding camera plate along x/z, plus sliding counterweights).

Solve the weighted least-squares or constrained problem:

$$
\min_{\Delta m_j,\;\delta x,\;\delta z}\; w_s\left(x_c'^2 + z_c'^2\right) + w_p\left(I_{xy}'^2 + I_{yz}'^2\right) + w_J\left(I_{yy}' - I_{yy}^{\mathrm{target}}\right)^2 + w_m\sum\Delta m_j
$$

subject to:

- Total trim mass ≤ **[TBD g]** (motor torque and inertia budget)
- Clearances and full pitch travel with no interference
- I_yy' within the inertia band
- y_c' within ±[TBD] mm

A sliding camera plate handles x/z static correction with zero added mass, so **slide first and add mass only for product-of-inertia correction.**

### C3. Inertia tuning considerations
- **Lower I_yy** gives more acceleration margin per amp, but more sensitivity to disturbance torques (cable drag, friction).
- **Higher I_yy** gives better passive disturbance rejection and jitter filtering, but needs more torque and lowers mechanical/electrical mode ratios.
- **Inertia ratio:** keep I_yy, reflected, versus the rotor inertia within the controller design assumption **[TBD]**. For direct drive, check the load/rotor ratio against the bandwidth target.
- **Retune or verify control gains** if I_yy changes by more than **[TBD, e.g., 10%]**. Loop gain scales with 1/J.

---

## 5. Phase D: Execution Procedure

1. **Configure** the payload to the declared balance state (lens, focus, zoom, filter, cable routing secured with final dressing).
2. **Measure baseline:** run B2, B3, and B4, and record as "Cycle 18 pre-trim."
3. **Compute trim** per C2. Peer-review the solution before installation.
4. **Coarse static:** adjust the camera plate in x and z until the gravity torque is below 2× the target.
5. **Install dynamic trim masses** at the computed stations. Use threadlocker and torque to spec; record serial or position IDs.
6. **Fine static:** re-slide the plate to null the residual introduced by the trim masses.
7. **Re-measure:** repeat B2, B3, and B4 to get "post-trim" values.
8. **Iterate** steps 4–7 until the criteria in Section 6 are met. Limit to **[3]** iterations before escalating.
9. **Lock and mark:** apply witness marks to the slide plate and trim screws, and photograph the configuration.
10. **Closed-loop verification:** step responses, frequency response (check that the crossover and phase margin match the design), holding current across the pitch range, and roll/yaw reaction during pitch slews.

---

## 6. Acceptance Criteria (Template)

| Parameter | Typical starting value | Cycle 18 requirement |
|---|---|---|
| Static imbalance U_s | ≤ 0.5–2 g·mm per kg of payload | [TBD] |
| Peak gravity torque over travel | ≤ 5% of continuous motor torque | [TBD] |
| \|I_xy\|, \|I_yz\| | ≤ 1–2% of I_yy | [TBD] |
| Principal-axis tilt | ≤ 0.5–1° | [TBD] |
| I_yy vs. target | ±5% | [TBD] |
| Axial CoM y_c | ±1–2 mm | [TBD] |
| Holding-current variation over travel | ≤ [TBD]% of baseline | [TBD] |
| Roll/yaw reaction during pitch slew | ≤ [TBD] N·m at max α | [TBD] |

---

## 7. Records and Error Budget

**Record for each step:** instrument IDs and calibration dates, ambient temperature, configuration state, raw data files, fit residuals, and trim hardware list.

**Uncertainty sources to budget:**
- Scale resolution
- K_t tolerance (±5–10% unless calibrated)
- Cogging and friction residuals
- Pendulum timing and fixture tare
- Cable stiffness and drag (often the dominant in-situ error; measure with cables disconnected as a cross-check)
- Thermal drift

**Cycle-over-cycle tracking:** compare against Cycle 17 values. Flag any shift greater than the measurement uncertainty, since it indicates a hardware or configuration change.

---

If you share the Cycle 18 specifics (payload mass and variants, motor K_t and continuous torque, bandwidth target, available trim stations, and prior-cycle residuals), I can turn the template limits into numbers and work out an actual trim-mass solution.