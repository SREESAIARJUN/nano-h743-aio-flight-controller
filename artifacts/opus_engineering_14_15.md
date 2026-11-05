# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning

**Scope note:** I don't have your program's Cycle 14 documentation, so I can't reproduce its specific requirements, part numbers, or prior-cycle findings. Below is a complete, general procedure. Fields marked **[C14]** are where you should insert your Cycle 14 values. If you share the cradle geometry, payload mass properties, motor specs, or the Cycle 13 results, I can turn this into numeric targets.

---

## 1. Definitions and Coordinate Frame

**Frame {P}: the pitch cradle body frame**
- Origin O_P is on the pitch bearing axis, at the midpoint between the bearings.
- **y_P** runs along the pitch rotation axis.
- **x_P** runs along the camera optical axis at zero pitch.
- **z_P** completes the right-handed set (nominally "up" at zero pitch).

**Rotating assembly (the balanced body)** includes:
- Camera body, lens, and lens accessories (follow-focus gear, hood, filters)
- Cradle and mounting plate
- Pitch motor rotor and encoder magnet or disk
- Rotating fraction of cables and slip-ring
- Trim masses and sliding-rail carriages

**Mass properties of the assembly about O_P:**

- Total mass: m
- Center of mass: r_c = [x_c, y_c, z_c]ᵀ
- Inertia tensor:

```
        ⎡  Ixx  −Ixy  −Ixz ⎤
  I_O = ⎢ −Ixy   Iyy  −Iyz ⎥
        ⎣ −Ixz  −Iyz   Izz ⎦
```

Here Ixy = ∫xy dm, and so on, with the sign convention stated explicitly.

---

## 2. Balance Objectives

### 2.1 Static balance (gravity torque about pitch)

The gravity torque about the pitch axis at pitch angle θ is:

  τ_g(θ) = m·g·(x_c·sin θ + z_c·cos θ)  *(sign depends on the frame convention)*

The requirement is that **x_c → 0 and z_c → 0**, so the CoM lies on the pitch axis. y_c does not create pitch torque, but it loads the roll and yaw axes. Hand y_c off to the roll-axis balance step.

**Acceptance criterion [C14]:**

  |τ_g|_max = m·g·√(x_c² + z_c²) ≤ k_s · τ_motor,cont

A typical target is k_s ≤ 2–5 %, or a residual offset of ≤ 0.1–0.3 mm for small gimbals.

### 2.2 Dynamic balance (products of inertia)

When the cradle spins at rate ω about y_P, the bearing reaction moments are proportional to **Ixy and Iyz**:

  M_bearing ≈ ω²·[Iyz, 0, −Ixy]ᵀ + ω̇·[−Ixy, Iyy, −Iyz]ᵀ

Non-zero Ixy and Iyz cause two problems:
- They couple pitch acceleration into roll and yaw torque disturbances.
- They load the bearings unevenly.

Ixz does not couple when rotation is about y, but it matters for roll and yaw dynamics. Minimize it when it is cheap to do so.

**Target [C14]:** |Ixy|, |Iyz| ≤ ε_p · Iyy, with a typical ε_p ≈ 1–3 %.

### 2.3 Principal-axis alignment

The two conditions above are equivalent to making y_P a principal axis of I_O that passes through the CoM. The misalignment angle between y_P and the nearest principal axis is:

  φ ≈ √(Ixy² + Iyz²) / |Iyy − I_⊥|

where I_⊥ is the nearer of Ixx and Izz. Keep φ ≤ **[C14]**, typically ≤ 1–2°. This becomes ill-conditioned when Iyy ≈ I_⊥. In that case, specify products of inertia directly.

### 2.4 Inertia magnitude tuning (control-driven)

Iyy sets the plant gain and the resonance placement:

- **Plant:** G(s) = 1 / (Iyy·s²), cascaded with the structural mode
- **Mode frequency:** ω_n = √(k_struct / Iyy_eff). Require ω_n ≥ (5–10) × the pitch loop bandwidth **[C14]**.
- **Motor sizing:** α_max = τ_peak / Iyy must be ≥ the required slew acceleration **[C14]**.
- **Inertia ratio to rotor** (for direct-drive stability margins): J_load / J_rotor ≤ **[C14]**

There is a trade-off:
- Lower Iyy gives better agility and a higher mode frequency.
- Somewhat higher Iyy gives better rejection of high-frequency disturbances, because it acts as a passive inertial filter.

Pick the Iyy window first. Balancing must then stay inside that window.

---

## 3. Mass-Property Model (Pre-Balance Prediction)

1. **Build the CAD or tabulated model.** For each component i, record m_i, its CoM r_i, and its tensor I_ci about its own CoM.

2. **Transfer each tensor to O_P** with the parallel-axis theorem:

   I_O,i = R_i·I_ci·R_iᵀ + m_i·( (r_iᵀr_i)·E₃ − r_i·r_iᵀ )

   Sum over i to get I_O. The CoM is r_c = Σm_i·r_i / m.

3. **Run payload variants as a configuration matrix [C14].** Include:
   - Lens A/B/C
   - Zoom extended vs. retracted (the CoM shifts along x)
   - Focus ring position
   - With and without matte box or ND

4. **Define the balance envelope.** This is the range of r_c across all variants. The adjustment mechanisms must cover this envelope plus a margin of at least 20 %.

---

## 4. Adjustment Degrees of Freedom

| DoF | Mechanism | Corrects |
|---|---|---|
| x (fore/aft) | Camera plate slide on cradle | x_c |
| z (vertical) | Cradle arm height slots or vertical riser | z_c |
| y (lateral) | Lateral plate offset (affects roll) | y_c |
| Products of inertia | Paired trim masses at (±x, ±y) and (±y, ±z) | Ixy, Iyz |
| Iyy magnitude | Symmetric mass pairs at radius r | Iyy (no CoM shift) |

### Trim mass equations

A point trim mass Δm at position s = [s_x, s_y, s_z] changes the properties as follows:

- Δ(m·r_c) = Δm·s
- ΔIxy = Δm·s_x·s_y
- ΔIyz = Δm·s_y·s_z
- ΔIyy = Δm·(s_x² + s_z²)

**Decoupled correction strategies:**

- **Product-only correction:** use a diagonal pair, Δm at (a, b, 0) and Δm at (−a, −b, 0).
  - ΔIxy = 2·Δm·a·b
  - The CoM is unchanged.
  - Iyy rises by 2·Δm·a².

- **Iyy-only tuning:** use Δm at (±r, 0, 0) or (0, 0, ±r).
  - ΔIyy = 2·Δm·r²
  - No change to the CoM or the products.

### Solving for corrections

Collect the corrections in a vector u, mapped from the measured errors e = [x_c, z_c, Ixy, Iyz, Iyy − Iyy*]ᵀ through a linear sensitivity matrix S. Solve:

  min ‖W·(e + S·u)‖²  subject to discrete or bounded u

W weights each error by its tolerance. With discrete trim weights, use least squares followed by rounding and a re-check, or small integer programming.

---

## 5. Measurement Procedures

### 5.1 Static balance about pitch (CoM in x–z)

**Method A: release test (coarse)**
1. Disable the pitch motor. Free the axis, decoupling the cable drag if possible.
2. Set the cradle at θ = 0°, then at 90°.
3. If it drifts at 0°, z_c ≠ 0 *(with z "up" and level camera, horizontal offset shows up as x_c at θ = 0)*.
   - Practical rule: **level camera tests fore/aft (x_c); camera pointed straight up or down tests vertical (z_c).**
4. Adjust until the cradle stays put at 0°, ±45°, and ±90°.

**Method B: motor-current measurement (fine, quantitative)**
1. Close the position loop at slow, constant velocity, or hold static at N angles.
2. Record the holding current i(θ). Then τ(θ) = K_t·i(θ).
3. Fit the model:

   τ(θ) = A·sin θ + B·cos θ + τ_f·sgn(ω) + τ_cable(θ)

4. Recover the offsets: x_c = A/(m·g) and z_c = B/(m·g), with signs per your convention.
5. Sweep in both directions and average to remove Coulomb friction.
6. Characterize the cable torque on its own, with the payload removed or with a dummy mass, then subtract it.

**Residual check:** |τ_g|_max ≤ k_s·τ_cont **[C14]**.

### 5.2 Iyy measurement

**Option 1: identification with the pitch motor**
- Inject a chirp or PRBS torque in closed loop with low-gain stabilization, or open loop if safe.
- Fit G(jω) = 1 / (J·(jω)² + b·jω) in the rigid-body band, below the first mode.
- Then Iyy = J − J_rotor (if the rotor is included).

**Option 2: torsional pendulum**
- Suspend the assembly on a torsion wire with known stiffness k_t, aligned on the pitch axis.
- Iyy = k_t·T² / (4π²)

**Option 3: compound pendulum (when a known CoM offset is acceptable)**
- Add a known mass offset d and measure the period T.
- I_pivot = m·g·d·T² / (4π²)
- Then remove the m·d² term.

### 5.3 Products of inertia (Ixy, Iyz)

**Spin or oscillation test:**
1. Drive a sinusoidal pitch oscillation θ = Θ·sin(ωt) at the operating amplitude and frequency **[C14]**.
2. Measure the reaction torques on the roll and yaw motors (current) while holding those axes, or measure base reactions with a 6-axis F/T sensor.
3. With the CoM already on the axis, the reactions are:
   - M_x = −Ixy·θ̈ + Iyz·θ̇²
   - M_z = −Iyz·θ̈ − Ixy·θ̇²
4. Regress against θ̈ and θ̇² to estimate Ixy and Iyz.

**Alternative:** use a trifilar or multi-orientation pendulum to measure the full tensor in about 6 orientations, then solve for the 6 unique tensor elements by least squares. Use this when a mass-properties instrument is available.

---

## 6. Step-by-Step Balancing Procedure (Cycle 14 Run Sheet)

**Pre-conditions**
- Configuration frozen: lens, accessories, cables, and firmware recorded in the log **[C14 config ID]**.
- Batteries and media installed in the flight state.
- Temperature recorded (bearing friction and cable stiffness vary with temperature).
- Roll and yaw axes locked level, or held under servo.

**Step 1: Mount and coarse lateral balance**
- Center the payload laterally (y_c).
- Verify with the roll axis free. This prevents roll-induced torques from contaminating pitch measurements.

**Step 2: Coarse fore/aft (x_c)**
- Camera level, pitch motor off.
- Slide the plate until there is no drift.

**Step 3: Coarse vertical (z_c)**
- Pitch to ±90°.
- Adjust height until there is no drift.

**Step 4: Iterate Steps 2–3**
- Iterate until the cradle holds at any angle. Usually 2–3 iterations, since the adjustments interact slightly when the adjuster axes aren't aligned with {P}.

**Step 5: Fine static balance by current fit**
- Use Method 5.1B with N ≥ 12 angles over the full pitch range **[C14 range]**, bidirectional.
- Compute x_c and z_c. Correct with fine adjusters or trim masses.
- Repeat until within tolerance.

**Step 6: Identify Iyy**
- Use Method 5.2 Option 1.
- Compare against the window [Iyy_min, Iyy_max] **[C14]**.
- If Iyy is low: add symmetric Iyy-only trim pairs.
- If Iyy is high: relocate the heaviest components toward the axis, or reduce the cradle arm span.

**Step 7: Identify Ixy and Iyz**
- Use Method 5.3.
- Apply diagonal paired trim masses (Section 4) to null them.
- Re-verify static balance, since imperfect pairing shifts the CoM.

**Step 8: Variant sweep**
- Repeat Steps 5 and 7 (quick check) for each payload variant in the configuration matrix.
- Record the adjuster settings per variant in the balance lookup table.

**Step 9: Structural resonance check**
- Run a sweep-sine in closed loop and capture the first mode ω_n.
- Confirm ω_n ≥ N × loop bandwidth **[C14]**.
- If not, stiffen the cradle or reduce Iyy before re-tuning gains.

**Step 10: Controller retune**
- Update the inertia estimate (plant gain) in the controller.
- Rescale the gains: K ∝ Iyy for a fixed bandwidth.
- Optionally enable gravity feed-forward using the measured residual A and B.

**Step 11: Final verification and record**
Log the following:
- m, r_c, the full I_O
- Residual τ_g
- Ixy and Iyz
- ω_n
- Loop margins (GM/PM)
- Trim mass inventory and positions
- Adjuster settings

---

## 7. Acceptance Criteria Template [C14]

| Parameter | Target | Measured | Pass/Fail |
|---|---|---|---|
| CoM offset √(x_c² + z_c²) | ≤ ___ mm | | |
| Residual gravity torque | ≤ ___ % of τ_cont | | |
| Iyy | ___ to ___ kg·m² | | |
| \|Ixy\| / Iyy | ≤ ___ % | | |
| \|Iyz\| / Iyy | ≤ ___ % | | |
| Principal axis misalignment | ≤ ___ ° | | |
| First structural mode | ≥ ___ Hz | | |
| Pitch loop PM / GM | ≥ ___ ° / ___ dB | | |
| Holding current at ±90° (motor off-axis worst case) | ≤ ___ A | | |

---

## 8. Error Sources and Mitigations

- **Cable and slip-ring torque.** This position-dependent torque looks like imbalance. Characterize it separately and route cables through or along the axis.
- **Bearing friction and cogging.** These bias the release tests. Use bidirectional current fits and include a cogging harmonic term in the regression.
- **Lens zoom or focus shift.** Balance at the median configuration, or use active CoM compensation (motorized slide) if the envelope exceeds tolerance.
- **Roll not level during pitch tests.** This projects the y_c offset into the pitch torque. Lock roll level and verify it with an inclinometer.
- **Thermal drift.** Cable stiffness and lubricant viscosity change with temperature. Balance at the operating temperature, or record the temperature with the results.
- **Trim mass placement error.** Use dowel-located trim positions. Avoid loose clamped weights.

---

If you provide the Cycle 14 specifics, I can turn this into a numerical plan: specific trim mass sizes and locations, tolerance budgets, and expected Iyy windows. Useful inputs would be:
- Payload masses and CoM locations
- Cradle geometry and adjuster ranges
- Motor K_t and τ_cont
- Target loop band