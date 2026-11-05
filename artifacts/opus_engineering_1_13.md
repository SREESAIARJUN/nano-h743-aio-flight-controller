# Pitch-Axis Camera Cradle: Mechanical Balancing & Inertia Tensor Tuning Procedure, Cycle 1

## 0. Scope and Assumptions

| Item | Assumption (edit to match your hardware) |
|---|---|
| Plant | Camera + lens + cradle, rotating on a single pitch shaft driven by a direct-drive BLDC motor (FOC, known torque constant *Kₜ*) |
| Frame | Body frame **B** fixed to cradle, origin on the pitch axis. **y** = pitch axis, **x** = optical axis (forward), **z** = up at θ = 0 |
| Cycle 1 goal | Establish baseline: CoM on axis, measured *I_yy*, products of inertia quantified and reduced, data recorded for controller tuning. Later cycles refine with lens/accessory variants. |
| Not in scope | Roll/yaw axis balance. That follows *after* pitch is closed, because pitch CoM moves the roll-axis load. |

---

## 1. Governing Relations

### 1.1 Static unbalance (gravity torque)
With CoM offset **r** = (x_c, y_c, z_c) in **B** and cradle pitch angle θ:

$$
\tau_g(\theta) = m g\,(x_c\cos\theta + z_c\sin\theta)
$$

(The sign depends on your rotation convention. Keep it consistent with the motor's positive direction.)

- Only x_c and z_c produce pitch torque. y_c (along the axis) affects roll/yaw balance, not pitch.
- Residual unbalance metric: $U = m\sqrt{x_c^2+z_c^2}$ in g·mm.

### 1.2 Inertia tensor about the pitch-axis origin

$$
\mathbf{I} =
\begin{bmatrix}
I_{xx} & -I_{xy} & -I_{xz}\\
-I_{xy} & I_{yy} & -I_{yz}\\
-I_{xz} & -I_{yz} & I_{zz}
\end{bmatrix},
\qquad I_{xy}=\int xy\,dm,\ \text{etc.}
$$

### 1.3 Dynamic coupling for pure pitch motion
For ω = (0, ω, 0) and α = ω̇, Euler's equations give:

$$
\tau_y = I_{yy}\,\alpha \quad(\text{plant seen by pitch loop})
$$
$$
\tau_x = -I_{xy}\,\alpha - I_{yz}\,\omega^2 \quad(\text{reaction into roll})
$$
$$
\tau_z = -I_{yz}\,\alpha + I_{xy}\,\omega^2 \quad(\text{reaction into yaw})
$$

**Conclusion:** I_xy and I_yz are the products that matter for pitch. They inject disturbance torque into the other axes. I_xz does not couple under pure pitch rotation.

### 1.4 Parallel-axis shift
I_yy(axis) = I_yy(CoM) + m(x_c² + z_c²). After static balance, the second term vanishes and I_yy(axis) ≈ I_yy(CoM).

---

## 2. Pre-Balance Preparation

1. **Configure the payload exactly as it will fly.** Include camera, lens, lens hood, filters, battery, media card, cables with dressed slack, and any focus or zoom motors. Cable stiffness is a hidden spring: route it to cross the axis with a service loop.
2. **Set the lens state.** Set zoom and focus to the mid-operating position. Record the position, because zoom changes x_c.
3. **Check bearings.** Verify friction with the motor unpowered: breakaway torque should be under 5% of the expected unbalance tolerance. High friction masks unbalance during the free-swing test.
4. **Get the CAD baseline.** Export m, CoM, and the inertia tensor about the axis origin. Record these as the *predicted* column in the data sheet (§7).
5. **Record the environment.** Note temperature, then level the base to within 0.1°.

---

## 3. Static Balancing

Balance two orthogonal components: fore/aft (x) and vertical (z).

### Step 3.1 – Coarse balance (motor OFF, free-swing)
1. **Balance fore/aft.** Set θ = 0 (optical axis horizontal). Slide the camera along the x-rail until the cradle holds level without drifting. This nulls x_c.
2. **Balance vertically.** Rotate to θ = 90° (lens pointing up or down). Adjust the vertical riser until the cradle holds at 90°. This nulls z_c.
3. **Verify.** Release at ±45°. A balanced cradle stays put. If it returns to a preferred angle, z_c is wrong (pendulous or inverted-pendulous). If it falls one way at every angle, x_c is wrong.

### Step 3.2 – Fine balance (motor ON, torque measurement)
Hold the cradle in position control at several angles. Record steady-state q-axis current I_q, then compute τ = K_t·I_q.

| θ | Measured torque gives |
|---|---|
| 0° | τ₀ = m g x_c → x_c = τ₀/(m g) |
| 90° | τ₉₀ = m g z_c → z_c = τ₉₀/(m g) |

**Better method:** sweep θ from −90° to +90° in 15° steps at very slow speed, in both directions. Least-squares fit:

$$
\tau(\theta) = A\cos\theta + B\sin\theta + \tau_f\,\mathrm{sgn}(\dot\theta) + \tau_0
$$

This gives m g x_c = A and m g z_c = B. It also separates out Coulomb friction τ_f and cable/offset bias τ₀.

### Step 3.3 – Correction
- **Preferred:** move the payload on its rails.
- **If rail travel is exhausted:** add a trim mass m_t at position (x_t, z_t) so that

$$
m_t x_t = -m\,x_c, \qquad m_t z_t = -m\,z_c
$$

Choose the largest practical lever arm to minimize added mass and added I_yy (ΔI_yy = m_t(x_t² + z_t²)).

### Step 3.4 – Acceptance (Cycle 1 placeholders, set per motor)

| Metric | Criterion |
|---|---|
| Gravity torque amplitude √(A²+B²) | ≤ 5% of motor continuous torque, and ≤ 10× bearing friction |
| Residual unbalance U | Record value. Target ≤ 50 g·mm for a ~1 kg payload (adjust) |
| Holding current at worst angle | ≤ 10% of rated I_q |

---

## 4. Measuring the Inertia Tensor

### 4.1 I_yy: compound pendulum method (primary)
1. With the cradle statically balanced and the motor unpowered or disconnected, attach a known test mass m_p at known distance d *below* the axis.
2. Displace it by 3° or less and release. Measure the period T over 10 or more cycles (encoder log or high-speed video).
3. Compute:

$$
I_{yy} = \frac{m_p g d\,T^2}{4\pi^2} - m_p d^2
$$

4. Repeat with two different d values and check consistency. Subtract the rotor inertia if the rotor stays coupled.

### 4.2 I_yy: system identification (cross-check, motor ON)
1. Inject a torque chirp (e.g., 1–100 Hz) or a PRBS signal open-loop at small amplitude around a held angle.
2. Fit the frequency response τ → θ to the model 1/(J s² + b s). Then J = I_yy + J_rotor.
3. Note any structural resonances (cradle arms, lens mount). Record the first mode frequency f₁. This is critical for controller bandwidth.

**Acceptance:** 4.1 and 4.2 agree within 5%, and CAD agrees within 15%. Investigate any larger CAD error, which is often caused by cables, the battery, or lens glass density.

### 4.3 Products of inertia I_xy, I_yz
1. **Direct measurement (preferred).** Mount a 3-axis load cell or 6-axis F/T sensor at the roll joint, or use roll/yaw motor currents with those axes locked.
2. Drive a pitch sinusoid with θ = Θ sin(Ωt).
3. Per §1.3, the roll-axis torque is τ_x = −I_xy·α − I_yz·ω². The α term appears at Ω and the ω² term at 2Ω. Demodulate both:
   - The component at Ω in phase with α gives I_xy.
   - The component at 2Ω gives I_yz.
4. Do the same with τ_z for cross-checking.
5. **Fallback if no sensor is available:** use the CAD values, corrected for measured component masses and positions.

---

## 5. Inertia Tensor Tuning

### 5.1 Zero the coupling products I_xy, I_yz without disturbing static balance
Use **paired masses** placed point-symmetric about the CoM. Their combined CoM is unchanged, but they shift the product of inertia.

To change I_xy, place two masses m at (a, b, ·) and (−a, −b, ·):

$$
\Delta I_{xy} = 2 m a b, \qquad \Delta(\text{CoM}) = 0
$$

Choose m·a·b = −I_xy/2. Apply the same method in the y–z plane for I_yz. Prefer adjusting the camera's lateral (y) position first, if the mount allows it.

**Acceptance:** coupling torque at the maximum pitch rate and acceleration (from the mission profile) should be below 10% of the roll/yaw static unbalance tolerance. Equivalently:

$$
|I_{xy}|\,\alpha_{\max} + |I_{yz}|\,\omega_{\max}^2 \le 0.1\,\tau_{\text{tol,roll}}
$$

### 5.2 Tune I_yy for controller performance
I_yy sets the pitch plant. Determine the target range from:

| Constraint | Expression | Pushes I_yy |
|---|---|---|
| Slew acceleration | I_yy + J_r ≤ τ_peak/α_req | Down |
| Disturbance rejection (passive inertia vs. base angular jitter coupled via friction/cables) | Higher J filters high-frequency torque disturbance: θ_err ≈ τ_d/(J ω²) | Up |
| Motor/load inertia ratio | J_load/J_rotor ≈ 1–10 for direct drive. Higher values are fine but reduce encoder-noise margin | – |
| Structural mode | Keep f₁ ≥ 5× the target loop bandwidth. Added mass far from the axis lowers f₁ | Down/stiffen |

**Tuning moves:**
- **To raise I_yy:** add paired masses at a large radius, symmetric about the axis in x–z (so CoM is unchanged). The increase is ΔI_yy = Σ mᵢ rᵢ².
- **To lower I_yy:** move mass toward the axis, lighten the cradle arms, or relocate the battery or accessories.
- **Principal axis alignment:** after §5.1, **y** is a principal axis. Confirm this by checking that the eigenvector of the measured **I** nearest to ŷ is within 2° of it.

---

## 6. Verification Run (closes Cycle 1)

1. Repeat the §3.2 sweep and confirm static acceptance still holds after tuning masses were added.
2. Repeat the §4.2 system ID to get the final J and f₁.
3. Run the mission-profile pitch maneuver (max slew plus step). Log roll and yaw motor currents and confirm the coupling criterion from §5.1.
4. Check thermal drift: repeat the 0°/90° holding-torque test after a 20-minute warm run. Lens and grease changes can shift x_c.
5. Freeze the configuration: mark rail positions, photograph trim masses, and torque-stripe the fasteners.

---

## 7. Cycle 1 Data Sheet (template)

| Parameter | CAD predicted | Measured (pre) | Measured (post) | Target / Limit | Pass |
|---|---|---|---|---|---|
| Payload mass m [kg] | | | | – | |
| x_c [mm] | | | | ≤ __ | |
| z_c [mm] | | | | ≤ __ | |
| Residual U [g·mm] | | | | ≤ __ | |
| Friction τ_f [mN·m] | – | | | ≤ __ | |
| I_yy [kg·m²] | | | | __–__ | |
| I_xy [kg·m²] | | | | ≤ __ | |
| I_yz [kg·m²] | | | | ≤ __ | |
| Principal-axis misalignment [°] | | | | ≤ 2 | |
| First structural mode f₁ [Hz] | | | | ≥ 5×BW | |
| Trim masses added [g @ position] | – | – | | – | |
| Lens zoom/focus state | | | | – | |

---

## 8. Hand-off to Cycle 2
- Feed J (final) and f₁ into pitch controller tuning (gain scheduling, notch filter placement).
- Characterize x_c versus zoom position. Decide whether active CoM compensation (feed-forward torque) or a mechanical compensator is needed.
- Proceed to roll-axis balance using the frozen pitch configuration, then yaw.
- Re-run the procedure for each lens/accessory variant and build a configuration lookup table.