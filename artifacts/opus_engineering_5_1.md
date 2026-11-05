# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning (Cycle 5)

> **Scope note:** I don't have your project's Cycle 1–4 records, so this procedure is a self-contained, generic formulation. Wherever a number is project-specific (masses, tolerances, motor data), I've marked it **[insert]** or given a typical starting value you should replace with your actual specifications.

---

## 1. Reference Frame and Definitions

Body frame **C** is fixed to the cradle, with its origin **O** on the pitch bearing axis at the axial midpoint between the bearings.

- **y_C**: pitch rotation axis
- **x_C**: optical axis (boresight), at the nominal level position
- **z_C**: completes a right-handed frame (up at level)

Cradle payload (camera, lens, cradle structure, trim masses):

- Total mass: m = Σ mᵢ
- Center of mass: **r**_G = (x_G, y_G, z_G)
- Inertia tensor about O:

$$
\mathbf{I}_O=\begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz}\\ -I_{xy} & I_{yy} & -I_{yz}\\ -I_{xz} & -I_{yz} & I_{zz}\end{bmatrix}
$$

## 2. Balancing Objectives

| Objective | Condition | Physical effect if violated |
|---|---|---|
| **Static balance** | x_G = z_G = 0 | Gravity torque τ_g = m·g·(x_G cos θ + z_G sin θ), which loads the motor and varies with pitch angle θ |
| **Axial centering** | y_G ≈ 0 | Unequal bearing loads and a roll/yaw moment arm |
| **Dynamic balance** | I_xy = I_yz ≈ 0 | Base roll/yaw rates couple into pitch torque |
| **Inertia tuning** | I_yy = J_target | Sets control bandwidth, disturbance rejection, and resonance placement |

**Why the products of inertia matter.** From Euler's equation, the pitch-axis torque is

$$
\tau_y = I_{yy}\dot\omega_y - I_{xy}\dot\omega_x - I_{yz}\dot\omega_z + (I_{xx}-I_{zz})\,\omega_x\omega_z + I_{xz}(\omega_x^2-\omega_z^2) + \omega_y(I_{xy}\omega_z - I_{yz}\omega_x)
$$

- The terms in **I_xy** and **I_yz** couple roll and yaw accelerations from the vehicle or handheld base directly into pitch disturbance torque.
- The term in **(I_xx − I_zz)** produces a gyroscopic/centrifugal coupling during combined motion. Driving it toward zero (I_xx ≈ I_zz) is a secondary goal.

## 3. Inertia Tuning Target

**Step 3.1 – Torque budget**

$$
\tau_{\text{motor,cont}} \ge \underbrace{I_{yy}\,\alpha_{\max}}_{\text{slew}} + \underbrace{m g e_{\max}}_{\text{residual unbalance}} + \underbrace{\tau_{\text{fric}}}_{\text{bearing/cable}} + \underbrace{\tau_{\text{coupling}}}_{\text{Sec. 2}}
$$

Keep a safety factor of at least 1.5 on continuous torque.

**Step 3.2 – Structural resonance placement.** The first cradle mode (torsional or bending, with stiffness k_θ) must sit well above the control bandwidth f_c:

$$
f_n = \frac{1}{2\pi}\sqrt{\frac{k_\theta}{I_{\text{eff}}}} \;\ge\; (5\text{–}10)\,f_c
$$

**Step 3.3 – Inertia window**

$$
I_{\min} \le I_{yy} \le I_{\max}
$$

- **I_max** is set by the torque budget and resonance requirement.
- **I_min** is set by disturbance rejection. Higher inertia filters high-frequency jitter passively.
- For direct-drive BLDC gimbals, a load-to-rotor inertia ratio up to roughly 10:1 is usually workable. With a gear or belt reduction N, the reflected inertia is I_yy/N².

**Record for this cycle:** J_target = **[insert] kg·m²**, with tolerance ±**[insert]**% (typically ±5%).

## 4. Measurement Procedure

**4.1 Pre-check**
- Install the camera and lens in flight/production configuration: battery, media card, lens hood, filters, cables, and cable routing all as used.
- Set the lens to its worst-case configuration. For a zoom lens, record CoM at both extremes and balance at the mid-zoom position, or at the most-used focal length.
- Lock the roll and yaw axes. Power the pitch motor off, or set it to zero-torque mode.

**4.2 Static CoM measurement (x_G, z_G)**

1. **Drift test.** Release the cradle at θ = 0°, then at ±90°. Note any drift direction.
   - Drift at θ = 0° indicates an x_G offset.
   - Drift at ±90° indicates a z_G offset.
2. **Quantitative method A (motor current).** Hold the cradle at angles θ = −90° to +90° in 15° steps using the position loop. Record holding torque τ(θ) = K_t·i_q, then fit

$$
\tau(\theta) = m g\,(x_G\cos\theta + z_G\sin\theta) + \tau_0\,\mathrm{sgn}(\dot\theta)
$$

   Approach each angle from both directions and average, so the friction term cancels.
3. **Quantitative method B (alternative).** Hang known calibration weights at a known arm and find the null-balance point.

**4.3 Axial CoM (y_G)**

Use load cells or bearing-reaction measurement, or a knife-edge balance perpendicular to the pitch axis on the removed cradle.

**4.4 Moment of inertia I_yy**

- **Option 1: compound pendulum.** Add a known offset mass m_p at radius r_p. Measure the small-oscillation period T. Then

$$
I_{yy} + m_p r_p^2 = \frac{m_{\text{tot}}\, g\, d\, T^2}{4\pi^2}
$$

  Here d is the CoM offset of the combined system. Subtract the m_p r_p² contribution.
- **Option 2: system identification.** Apply a torque chirp (0.5–50 Hz) through the motor and fit the low-frequency rigid-body plant 1/(I s²). The same sweep also reveals the first resonance f_n.

**4.5 Products of inertia (I_xy, I_yz)**

- Mount the cradle on a balancing fixture, or use a spin test about the pitch axis. Measure the rotating bearing reaction moments with accelerometers on the bearing housings:

$$
M_{\text{rot}} = \omega^2 \sqrt{I_{xy}^2 + I_{yz}^2}
$$

- Alternatively, compute them from validated CAD. Weigh each component, compare to its CAD mass, and correct the densities.
- **Operational check:** command zero pitch motion while exciting the roll axis sinusoidally. Any induced pitch torque at the excitation frequency indicates I_xy coupling.

## 5. Adjustment Procedure

Perform the steps in this order. Each later step disturbs the earlier ones least.

1. **Coarse axial (y_G):** shift the camera laterally on its plate.
2. **Fore-aft (x_G):** slide the camera along the optical axis on the dovetail or rail.
3. **Vertical (z_G):** adjust the cradle arm height or vertical riser.
4. **Products of inertia:** add trim mass pairs (Δm at positions ±**r**) so that the CoM stays unchanged while the products shift:

$$
\Delta I_{xy} = -2\,\Delta m\, x_t\, y_t, \qquad \Delta I_{yz} = -2\,\Delta m\, y_t\, z_t
$$

   Place the pairs at diagonally opposite corners of the cradle.
5. **I_yy tuning:** add a symmetric trim mass pair at radius ±r about the axis in the x–z plane. This leaves the CoM unchanged:

$$
\Delta I_{yy} = 2\,\Delta m\, r^2 \;\Rightarrow\; \Delta m = \frac{J_{\text{target}} - I_{yy,\text{meas}}}{2r^2}
$$

   If I_yy must be **reduced**, remove material symmetrically or move components toward the axis. Re-verify the resonance afterward, because removing material may reduce stiffness.
6. **Iterate** steps 2–5 until all criteria are met. Usually 2–3 passes are needed.

## 6. Acceptance Criteria (Cycle 5)

| Parameter | Typical limit | Cycle 5 spec |
|---|---|---|
| Radial CoM offset √(x_G²+z_G²) | ≤ 0.1–0.3 mm, or residual torque ≤ 5% of continuous motor torque | [insert] |
| Axial CoM offset y_G | ≤ 1–2 mm | [insert] |
| \|I_xy\|, \|I_yz\| | ≤ 1–2% of I_yy | [insert] |
| I_yy | J_target ±5% | [insert] |
| Holding current variation over ±90° | ≤ 10% of rated | [insert] |
| First structural mode f_n | ≥ 5× control bandwidth | [insert] |
| Free-release drift (motor off) | Stays put at 0°, ±45°, ±90° | — |

## 7. Verification and Documentation

1. Repeat the 4.2 torque sweep and the 4.4 chirp after final trim.
2. Run a closed-loop test with a base disturbance (roll/yaw shaker or a vehicle run). Record the pitch pointing error in RMS and peak.
3. Log the results in a cycle table:

| Item | Cycle 4 | Cycle 5 pre-trim | Cycle 5 final | Spec | Pass/Fail |
|---|---|---|---|---|---|
| x_G, z_G (mm) | | | | | |
| I_yy (kg·m²) | | | | | |
| I_xy, I_yz | | | | | |
| f_n (Hz) | | | | | |
| Trim mass added (g, location) | | | | | |

4. Record the configuration ID (camera, lens, accessories). Any change in payload requires re-balancing.

---

If you share the Cycle 5 payload data (masses, lens type, motor K_t, target bandwidth, and Cycle 4 results), I can compute specific trim-mass values, J_target, and tolerance limits.