# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning

**Note:** I don't know your program's specific "Cycle 26" (its payload, hardware revision, or prior iteration data). Below is a general, rigorous procedure, ending with a Cycle 26 record template. You can drop in your measured values, and I can help refine the numbers.

---

## 1. Reference Frame and Definitions

Define a body frame fixed to the cradle, with its origin on the pitch bearing axis:

| Axis | Direction |
|---|---|
| **x** | Pitch rotation axis (through both bearings) |
| **y** | Optical axis (forward, at θ = 0) |
| **z** | Up (at θ = 0, level) |

The other quantities are:

- **m**: total rotating mass (cradle, camera, lens, cables, and the pitch motor rotor if it rotates with the cradle).
- **r_c = (x_c, y_c, z_c)**: center of mass (CoM) relative to the origin.
- **Inertia tensor about the origin:**

$$
\mathbf{I} = \begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz} \\ -I_{xy} & I_{yy} & -I_{yz} \\ -I_{xz} & -I_{yz} & I_{zz} \end{bmatrix},\quad I_{xy}=\sum m_i x_i y_i \text{ (etc.)}
$$

**Balance objectives, in order of priority:**

1. **Static balance:** y_c = z_c = 0. The CoM lies on the pitch axis, so gravity torque is zero at every angle.
2. **Dynamic balance:** I_xy = I_xz = 0. The pitch axis is a principal axis, so pitch motion produces no reaction torques on roll or yaw.
3. **Tensor tuning:** I_yy ≈ I_zz, and I_xx falls in a target band. This keeps the outer-axis inertia independent of pitch angle and matches the control bandwidth.

x_c (CoM position along the pitch axis) does not affect pitch torque. It does load the roll and yaw axes, so it should be centered as part of outer-axis balancing.

---

## 2. Governing Equations

### 2.1 Gravity torque vs. pitch angle

Rotate the cradle by θ about x. The CoM moves to y′ = y_c cos θ − z_c sin θ. The gravity torque about the pitch axis is:

$$
\tau_g(\theta) = -m g\,(y_c\cos\theta - z_c\sin\theta)
$$

This gives two direct measurements:

- At θ = 0° (level): τ_g = −m g y_c, which isolates the **fore/aft offset**.
- At θ = ±90° (pointing up or down): τ_g = ±m g z_c, which isolates the **vertical offset**.

### 2.2 Dynamic reaction torques

Pitch motion is ω = (ω, 0, 0) with acceleration α. Euler's equation, τ = Iα + ω × (Iω), gives:

$$
\tau_x = I_{xx}\,\alpha,\qquad
\tau_y = -I_{xy}\,\alpha + I_{xz}\,\omega^2,\qquad
\tau_z = -I_{xz}\,\alpha - I_{xy}\,\omega^2
$$

Nonzero products of inertia therefore inject disturbances into roll and yaw during pitch slews. This shows up as cross-axis jitter or horizon wobble.

### 2.3 Outer-axis inertia variation

Assume products of inertia are zero. The inertia that the cradle presents to the outer gimbal axis varies with pitch angle:

$$
I_{\text{outer}}(\theta) = I_{yy}\sin^2\theta + I_{zz}\cos^2\theta
$$

If I_yy ≠ I_zz, the roll and yaw plant gain changes with pitch angle. Making I_yy = I_zz removes this variation, which lets you use fixed controller gains.

---

## 3. Balancing Procedure

### Phase 0: Preparation

1. Install the final flight configuration: camera body, lens (at the focal length or zoom position you'll fly), filters, lens hood, battery, and SD card.
2. Route and **strain-relieve all cables** in their final positions. Cable stiffness and drag are the most common source of false balance readings.
3. Lock the outer axes level. Power the pitch motor off, or use calibrated current measurement.
4. Confirm bearing friction is low. Spin the cradle by hand; it should coast smoothly. Record the breakaway torque τ_f, because this is your detection floor.

### Phase 1: Coarse static balance (fore/aft, y_c)

1. Set θ = 0 (level) and release the cradle.
2. If the nose drops, y_c > 0. Slide the camera or cradle rearward. If the tail drops, slide it forward.
3. Iterate until the cradle stays level when released.

### Phase 2: Coarse static balance (vertical, z_c)

1. Set θ = +90° (pointing straight up) and release the cradle.
2. If the cradle falls back toward level, the CoM is off the axis vertically. Raise or lower the camera (or the cradle rails) to correct it.
3. Re-check level (Phase 1), because these two adjustments interact through mounting geometry.
4. **Pass condition:** the cradle holds any angle from −90° to +90° when released.

### Phase 3: Fine static balance (quantitative)

Measure the holding torque at several angles. Use motor current (τ = K_t · i), a torque gauge, or a small spring scale at a known lever arm. Take readings at θ = −90°, −45°, 0°, 45°, and 90°.

Fit the data by least squares:

$$
\tau(\theta) = a\cos\theta + b\sin\theta + \tau_0
\;\Rightarrow\; y_c = -\frac{a}{mg},\quad z_c = \frac{b}{mg}
$$

The constant term τ₀ captures cable spring bias. If τ₀ is significant, fix the cable routing rather than compensating with mass.

**Correction mass sizing:** to null an offset with a trim mass Δm at position (y_Δ, z_Δ):

$$
m\,y_c + \Delta m\,y_\Delta = 0,\qquad m\,z_c + \Delta m\,z_\Delta = 0
$$

Prefer moving the payload over adding mass. Added mass raises I_xx and reduces control authority.

### Phase 4: Dynamic balance (products of inertia)

**Measurement options:**

- **Spin test (preferred):** rotate the cradle at a constant ω, or run sinusoidal pitch sweeps. Measure reaction torques on the roll and yaw motors, either from currents or an IMU on the outer frame. Then fit them to the τ_y and τ_z equations above to extract I_xy and I_xz.
- **CAD estimate:** export the tensor from a CAD model with accurate component masses. Validate it against the spin test.
- **Pendulum tests:** for I_xx, use a bifilar or trifilar pendulum, or the swing period with a known offset mass.

**Correction using CoM-neutral mass pairs:** place two equal masses m_t at (+a, +b, 0) and (−a, −b, 0). This pair:

- leaves the CoM unchanged, so static balance is preserved,
- changes I_xy by 2·m_t·a·b,
- and leaves I_xz untouched.

Use the analogous pair in the x–z plane for I_xz. Place the pairs as far apart along x as possible (near the side plates), which minimizes the trim mass needed.

### Phase 5: Inertia tensor tuning

| Parameter | Target | Lever |
|---|---|---|
| I_xx | Within motor and bandwidth budget (see below) | Compact payload, light cradle, mass close to the axis |
| I_yy / I_zz | ≈ 1.0 (e.g. within ±10%) | Shift mass between the optical and vertical directions; adjust the tall vs. long distribution of the cradle |
| \|I_xy\|, \|I_xz\| | < 1–2% of I_xx | Phase 4 trim pairs |
| Load/rotor inertia ratio | Typically ≤ 10:1 for direct drive | Motor selection, I_xx reduction |

**I_xx sizing check:**

- Peak acceleration requirement: τ_peak ≥ I_xx · α_max + τ_f + τ_residual.
- Bandwidth: the open-loop crossover frequency scales with K_t·K_p / I_xx. Also keep the first structural mode of the cradle at **3–5× the control bandwidth** or higher.

---

## 4. Acceptance Criteria (Typical; Adjust to Your Spec)

| Check | Criterion |
|---|---|
| Static residual torque | < 3–5% of motor continuous torque at all θ |
| Free-release drift | Holds within ±2° over 10 s at θ = −90°, 0°, and 90° (motor off) |
| Cable bias τ₀ | < 1% of continuous torque |
| Products of inertia | < 2% of I_xx |
| Outer-axis inertia variation | \|I_yy − I_zz\| / I_zz < 10% |
| Holding current at rest | Within X% of the idle baseline (program-defined) |
| Cross-axis disturbance during a pitch slew | Roll/yaw error below the jitter spec at max α |

---

## 5. Cycle 26 Record Template

```
CYCLE 26 – PITCH CRADLE BALANCE LOG
Config: camera/lens/zoom ______  accessories ______  cable rev ______
Mass m: ______ kg     Friction floor τ_f: ______ N·m

Static fit:  a = ____  b = ____  τ0 = ____   → y_c = ____ mm  z_c = ____ mm
Corrections: payload shift Δy = ____ Δz = ____ ; trim mass ____ g @ (____)
Post-trim residual (max over θ): ____ N·m  [PASS/FAIL]

Tensor (about pivot, kg·m²):
 Ixx ____  Iyy ____  Izz ____  Ixy ____  Ixz ____  Iyz ____
Trim pairs: m_t ____ g @ (±a, ±b, ±c) = ____
Post-trim Ixy/Ixx ____ %  Ixz/Ixx ____ %   Iyy/Izz ____   [PASS/FAIL]

Δ vs Cycle 25: ______________________   Next action: ______________
```

---

If you share Cycle 26's parameters (payload mass, current CoM offsets, CAD tensor, motor K_t and torque limits, and target bandwidth), I can compute the specific trim masses, positions, and expected residuals.