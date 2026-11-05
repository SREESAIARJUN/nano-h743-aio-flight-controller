# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning (Cycle 4)

I don't have your Cycle 1–3 records, so this is a complete procedure with placeholder values. Substitute your actual masses, motor data and acceptance limits where marked **[▢]**.

---

## 1. Frames, Notation and Assumptions

| Symbol | Definition |
|---|---|
| O | Pivot point on the pitch axis (intersection with the roll axis if co-located) |
| **ŷ** | Pitch rotation axis |
| **x̂** | Optical axis, forward, at θ = 0 |
| **ẑ** | Completes the right-handed frame (up at θ = 0) |
| θ | Pitch angle |
| M | Total rotating mass: cradle + camera + lens + accessories + motor rotor share |
| **r**_c = (x_c, y_c, z_c) | Centre of mass (CoM) relative to O |
| **I**_O | Inertia tensor about O, in cradle-fixed axes |

Assumptions:
- The cradle is treated as a rigid body.
- Products of inertia use the tensor convention I_xy = −∫xy dm.

---

## 2. Governing Equations

### 2.1 Static (gravity) imbalance

The gravity torque about the pitch axis as a function of pitch angle is:

$$
\tau_g(\theta) = -Mg\,(x_c\cos\theta + z_c\sin\theta) = -Mg\,e\,\cos(\theta-\phi)
$$

where:
- e = √(x_c² + z_c²) is the eccentricity of the CoM from the axis,
- φ = atan2(z_c, x_c) is its angular position.

The axial offset y_c does not produce pitch torque. It does load the roll and yaw axes, so it should be centred for those axes.

### 2.2 Inertia about the pivot (parallel-axis theorem)

$$
\mathbf I_O = \mathbf I_C + M\left[(\mathbf r_c\!\cdot\!\mathbf r_c)\mathbf E - \mathbf r_c\mathbf r_c^{T}\right]
$$

Here **I**_C is the inertia tensor about the CoM and **E** is the 3×3 identity.

### 2.3 Dynamic coupling during pitch motion

For pure pitch motion, **ω** = ω**ŷ**. Euler's equation, **τ** = **I**_O ω̇ + **ω** × (**I**_O **ω**), gives:

$$
\boldsymbol\tau =
\begin{pmatrix}
I_{xy}\dot\omega + I_{zy}\omega^2\\[2pt]
I_{yy}\dot\omega\\[2pt]
I_{zy}\dot\omega - I_{xy}\omega^2
\end{pmatrix}
$$

So the products I_xy and I_yz inject reaction torques into the roll and yaw axes whenever the pitch axis accelerates or rotates. Static balance alone does not remove this. Dynamic balance requires **ŷ to be a principal axis**, meaning I_xy = I_yz = 0.

### 2.4 Pitch-axis plant

$$
(I_{yy} + J_m)\,\ddot\theta + b\,\dot\theta + \tau_{fric} = \tau_m + \tau_g(\theta) + \tau_{dist}
$$

In this equation:
- J_m is the motor rotor inertia, and b is the viscous damping coefficient.
- τ_fric is friction torque, τ_m is the commanded motor torque, and τ_dist is external disturbance torque (wind, cable drag, base motion).

---

## 3. Cycle 4 Design Targets

| Parameter | Target | Rationale |
|---|---|---|
| Residual eccentricity e | ≤ **[▢ e.g. 0.2 mm]** | Keep τ_g,max = Mge ≤ 5% of continuous motor torque |
| Residual τ_g,max | ≤ 0.05 τ_cont | Leaves headroom for disturbance rejection; minimises heat |
| \|I_xy\|, \|I_yz\| | ≤ 1–2% of I_yy | Limits cross-axis reaction torque (§2.3) |
| I_yy | Within **[▢ I_min, I_max]** | See §4 |
| First structural mode f_s | ≥ 4–5 × pitch loop bandwidth | Avoids exciting cradle resonance |
| Payload configuration range | All lens/zoom/focus states | CoM shifts must stay within adjuster travel |

---

## 4. Tuning the Pitch Inertia I_yy

The target value of I_yy is a trade-off between two effects.

**Larger I_yy** improves passive inertial stabilisation: high-frequency disturbances are attenuated as 1/(I s²).

**Smaller I_yy** gives higher achievable acceleration and bandwidth:

$$
\alpha_{max} = \frac{\tau_{peak} - Mge}{I_{yy}+J_m}
$$

Two sizing checks follow from this:

1. **Upper bound (agility).** Require α_max ≥ α_req, where α_req comes from the slew profile. This gives:

   I_yy ≤ (τ_peak − τ_g,max)/α_req − J_m

2. **Lower bound (jitter).** For a disturbance torque spectrum τ_d at frequencies above the loop bandwidth, the residual angular jitter is θ_rms ≈ τ_d/(I_yy ω²). Require θ_rms ≤ the pointing spec.

How to tune I_yy:
- **Increase I_yy:** move ballast radially outward symmetrically, in ±x or ±z pairs, so the CoM is unchanged.
- **Decrease I_yy:** lightweight the cradle, and pull the camera body closer to the axis while keeping the CoM on it.

Trim must always be added in **balanced pairs**, so that tuning I_yy does not disturb r_c or the products of inertia.

---

## 5. Balancing Procedure

### Step 0: Pre-check
1. Lock the roll and yaw axes. Level the base to within ≤ 0.1°.
2. Fit the full Cycle 4 payload: camera, lens, filters, cables and their routing exactly as in flight or use.
3. Verify the bearings with the motor unpowered. Measure breakaway friction τ_f with a torque gauge or by the current threshold. Note that τ_f limits how fine the manual balance can be resolved.

### Step 1: Model prediction (CAD)
1. Extract M, **r**_c and **I**_C from the CAD mass model, using measured component masses rather than catalogue values.
2. Compute **I**_O (§2.2) and the predicted adjuster settings.
3. Compare against the Cycle 3 as-built measurements. Apply any model correction factors found in Cycle 3.

### Step 2: Coarse static balance (manual)
1. **Vertical (z) balance.** Rotate the cradle to θ = 90° (lens pointing up) and release it. Adjust the vertical slide until the cradle holds position.
2. **Fore–aft (x) balance.** Return to θ = 0° and release. Adjust the fore–aft slide until the cradle holds position.
3. Iterate both until the cradle stays put at 0°, ±45° and ±90°.

The order matters. Doing z at 90° first decouples the two adjustments, because τ_g(90°) ∝ z_c only.

### Step 3: Fine static balance (motor current method)
1. Power the motor in position-hold mode with integral action. Use slow moves only.
2. At each of N angles (θ = −90° to +90°, steps of 15° or less), record the steady holding torque τ_h,i = k_t·i_q. Average both directions of approach so that friction cancels.
3. Least-squares fit:

   τ_h(θ) = A cosθ + B sinθ + C

4. Recover the CoM offsets:
   - x_c = A/(Mg)
   - z_c = B/(Mg)
   - C is the cable-spring or bias torque. Record it, because a large C means the cable routing needs rework.
5. **Trim correction.** Either move the adjusters by Δx = −x_c and Δz = −z_c, or add a trim mass m_t at position (x_t, z_t) such that:

   m_t·x_t = −M·x_c,  m_t·z_t = −M·z_c

   (Recompute M afterwards to include m_t.)
6. Repeat until e ≤ target.

### Step 4: Dynamic balance (products of inertia)
1. Unlock roll and yaw, or mount the gimbal on a 6-axis load cell.
2. Command a pitch sinusoid θ = θ₀ sin(Ωt). Use several Ω values that span the operational band.
3. Measure the roll-axis reaction torque τ_x and yaw-axis reaction torque τ_z, from motor currents or the load cell.
4. Regress the measured torques against the excitation using §2.3:
   - τ_x = I_xy ω̇ + I_yz ω²
   - τ_z = I_yz ω̇ − I_xy ω²
   
   Fitting both components gives I_xy and I_yz.
5. **Two-plane correction.** Add a mass pair ±m_p at (x_p, ±y_p) with equal and opposite x. This leaves the CoM unchanged and shifts only:

   ΔI_xy = −2 m_p x_p y_p

   Use the analogous (z, y) pair for I_yz.
6. Re-verify static balance (Step 3), since real trim placement is never perfectly symmetric.

### Step 5: Inertia identification
Measure I_yy using either method:

- **Torsional pendulum** (motor removed): known stiffness k, measured period T, so:

  I_yy = kT²/(4π²)

- **System identification** (in situ): inject a chirp torque, fit a frequency response function (FRF) to 1/((I_yy+J_m)s² + bs). This method also captures the first structural mode f_s.

Compare the result to the §4 bounds. If I_yy is outside them, adjust with symmetric ballast and return to Step 3.

### Step 6: Configuration sweep
Repeat Step 3 (abbreviated: 0° and 90° only) for each lens, zoom and focus extreme. Record the CoM excursion. Confirm that all states remain within e_target, or within the motor torque budget if no auto-balance is available.

---

## 6. Acceptance and Cycle 4 Record

| Item | Measured | Limit | Pass/Fail |
|---|---|---|---|
| M | ▢ | — | |
| x_c, z_c, e | ▢ | ≤ ▢ mm | |
| τ_g,max / τ_cont | ▢ | ≤ 5% | |
| Bias torque C | ▢ | ≤ ▢ | |
| I_yy | ▢ | ▢–▢ kg·m² | |
| \|I_xy\|/I_yy, \|I_yz\|/I_yy | ▢ | ≤ 2% | |
| f_s / loop bandwidth | ▢ | ≥ 4 | |
| CoM excursion over configs | ▢ | within adjuster travel | |

**Change log vs. Cycle 3:** record the trim masses and locations, the adjuster positions, the model-vs-measured deltas (for feeding back into the CAD mass properties), and any open actions.

---

If you share the Cycle 4 payload masses, the cradle geometry, and the motor's k_t and torque ratings, I can work through the numbers: trim mass sizing, I_yy bounds and the expected residual torques.