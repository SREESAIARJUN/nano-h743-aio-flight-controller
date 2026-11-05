# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning

*Note: I don't have the specific documentation for "Cycle 13." Below is a complete general procedure with placeholder tolerances. If you share the cradle geometry, payload mass, motor specs, or the Cycle 13 requirements, I can tailor the numbers.*

---

## 1. Reference Frame and Definitions

- **Origin O**: the intersection of the pitch axis with the cradle's mid-plane (the bearing centerline).
- **Axes**:
  - **y**: pitch axis, along the bearing bore.
  - **x**: optical axis, forward and parallel to the lens, at θ = 0.
  - **z**: completes a right-handed frame, pointing up at θ = 0.
- **Rotating assembly**: cradle, camera body, lens, cables in the moving loop, trim masses, and the motor rotor.
- **Mass properties**:
  - Total mass: M.
  - CG position relative to O: **r_c** = [x_c, y_c, z_c]ᵀ.
- **Inertia tensor about O**:

$$
\mathbf{I}_O=\begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz}\\ -I_{xy} & I_{yy} & -I_{yz}\\ -I_{xz} & -I_{yz} & I_{zz}\end{bmatrix},\qquad I_{ij}=\int x_i x_j\,dm \;(i\neq j)
$$

- **Parallel-axis transfer** from the CG to O:

$$
\mathbf{I}_O=\mathbf{I}_C+M\left(\|\mathbf r_c\|^2\mathbf E-\mathbf r_c\mathbf r_c^{\mathsf T}\right)
$$

## 2. Governing Torques on the Pitch Axis

**Gravity (static unbalance).** With gravity along −Z in the world frame:

$$
\tau_g(\theta)=-Mg\,(x_c\cos\theta - z_c\sin\theta)\quad\text{(sign per convention)}
$$

This is a sinusoid in θ. Its amplitude is $Mg\sqrt{x_c^2+z_c^2}$, and y_c does not contribute.

**Dynamic (Euler) terms.** For pure pitch motion, ω = θ̇ ŷ:

$$
\boldsymbol\tau = \mathbf I_O\dot{\boldsymbol\omega}+\boldsymbol\omega\times\mathbf I_O\boldsymbol\omega
$$

Expanding this gives the torque on each axis:

| Axis | Torque | Effect |
|---|---|---|
| Pitch (y) | $\tau_y = I_{yy}\ddot\theta$ | Wanted inertia load |
| Roll (x) | $\tau_x = -I_{xy}\ddot\theta - I_{yz}\dot\theta^2$ | Reaction into the roll stage |
| Yaw (z) | $\tau_z = -I_{yz}\ddot\theta + I_{xy}\dot\theta^2$ | Reaction into the yaw stage |

**Objectives:**
1. **Static balance**: x_c, z_c → 0, so there is no gravity torque.
2. **Dynamic balance**: I_xy, I_yz → 0, so pitch motion does not couple into roll or yaw.
3. **Inertia tuning**: set I_yy to meet the control-bandwidth and disturbance-rejection targets. Keep I_xz small so the principal axes stay aligned, which matters when the outer gimbals rotate.

## 3. Procedure

### Phase A — Configuration Freeze
1. Fix the payload configuration: camera, lens, filters, battery, and any accessories.
2. For zoom or focus lenses, record the CG shift across the travel range. Balance at the mid-travel or most-used position.
3. Route and tie down cables in their final service loop. Cable stiffness biases the static test, so measure it separately (Phase B, step 5).
4. Run the bearings in, and measure breakaway friction τ_f by slow rotation in both directions.

### Phase B — Static Balance (CG onto the Pitch Axis)

**Coarse balance**
1. Unpower the motor, or decouple it if it has cogging.
2. Release the cradle at θ = 0°, 90°, and 45°. Adjust the fore-aft (x) slide until there is no drift at 0°, then the vertical (z) slide until there is no drift at 90°.

**Fine balance (motor-current method)**
1. Hold position under closed-loop control at N ≥ 12 angles over ±90° or more.
2. Record the steady-state current i_k at each angle, averaging both approach directions to cancel friction.
3. Fit the data by least squares:

$$
K_t\, i(\theta)=A\cos\theta+B\sin\theta+C
$$

4. Recover the CG offsets: $x_c=-A/(Mg)$ and $z_c=B/(Mg)$.
5. Interpret the constant term: C represents the cable spring preload and cogging bias. Track it, but do not trim it out with masses.

**Correction**
1. Move the sled by Δx = −x_c and Δz = −z_c.
2. If the sled travel is exhausted, add trim mass m_t at **r_t**, sized so that

$$
M\mathbf r_c + m_t\mathbf r_t = \mathbf 0
$$

3. Iterate until the acceptance criterion is met (Section 4).

### Phase C — Dynamic Balance (Products of Inertia)

**Measurement**
1. Lock or stiffly servo the roll and yaw axes. Instrument their reaction torques, using motor currents or a 6-axis load cell at the base.
2. Drive the pitch axis with a sinusoid θ = Θ sin(Ωt) at 2–3 frequencies, staying below the first structural mode.
3. Separate the measured τ_x and τ_z into components:
   - The part in phase with $\ddot\theta$ (at frequency Ω) gives the I_xy and I_yz coefficients.
   - The $\dot\theta^2$ part (at frequency 2Ω) gives a cross-check.
4. Solve for I_xy and I_yz by least squares over all test runs.

**Correction (two-plane balancing)**
- Products of inertia cannot be corrected with a single mass without disturbing the CG. Instead, place **equal masses in pairs, symmetric about O**, at ±**r_p**:

$$
\Delta I_{xy}=2m_p x_p y_p,\qquad \Delta I_{yz}=2m_p y_p z_p,\qquad \Delta\mathbf r_c = 0
$$

- Use the widest available y-spacing, at the cradle side plates, to minimize the trim mass needed.
- Re-verify static balance after correction, since pairs are never perfectly symmetric.

### Phase D — Inertia Tuning (I_yy and Principal-Axis Alignment)

**1. Identify I_yy.** Use either method:
- **Chirp test**: fit the transfer function $\theta/\tau = 1/(I_{yy}s^2 + b s)$, with I_yy including the rotor.
- **Trifilar pendulum** (off-gimbal): $I = \dfrac{MgR^2T^2}{4\pi^2L}$, then transfer to O with the parallel-axis theorem.

**2. Set the target.** Choose I_yy to:
- **Meet the bandwidth requirement.** The loop crossover is ω_c ≈ √(K/I_yy). Larger I_yy improves rejection of high-frequency base disturbances, but it costs motor torque.
- **Respect the torque budget.**

$$
I_{yy}\ddot\theta_{max} + \tau_f + \tau_{residual} \le 0.7\,\tau_{cont}
$$

- **Keep the load/rotor inertia ratio** within the controller's design range, typically 1–30 for direct drive.
- **Avoid structural modes.** The first cradle mode should be at least 3–5 × ω_c.

**3. Align the principal axes.** The x–z principal axis tilt φ satisfies

$$
\tan 2\varphi = \frac{2I_{xz}}{I_{zz}-I_{xx}}
$$

Keep φ within tolerance so that roll and yaw dynamics are not cross-coupled when the pitch angle is nonzero. Correct I_xz with diagonal mass pairs at (+x, +z) / (−x, −z), or the opposite diagonal, depending on sign.

**4. Increase inertia if needed.** Use symmetric pairs placed far along ±x and ±z. These raise I_yy without moving the CG or creating products of inertia.

### Phase E — Verification
1. Repeat the static current sweep, the dynamic reaction test, and the I_yy identification.
2. Perform a full-envelope check: sweep the zoom, rotate the roll and yaw stages, and confirm the residuals stay within tolerance throughout.
3. Log the final mass properties: M, **r_c**, **I_O**, the trim-mass bill, and the sled positions.

## 4. Acceptance Criteria (Placeholders — Set per Cycle 13 Spec)

| Parameter | Typical target |
|---|---|
| Residual CG offset $\sqrt{x_c^2+z_c^2}$ | ≤ 0.1–0.2 mm, or residual gravity torque ≤ 2–5 % of τ_cont |
| Hand-release drift | None at any angle beyond friction band |
| Products of inertia \|I_xy\|, \|I_yz\| | ≤ 1–2 % of I_yy |
| Principal-axis tilt φ | ≤ 1° |
| I_yy | Within ±5 % of design value |
| Cable bias C | ≤ 10 % of τ_f |
| Repeatability across zoom range | Residual torque within the above limits |

## 5. Practical Notes
- **Balance in the operating orientation and temperature.** Thermal growth of the sled rails and lens can shift the CG by tenths of a millimetre.
- **Avoid overbalancing with trim mass.** Every gram added raises I_yy and the outer-axis inertias. Prefer moving existing mass (sleds, battery position) first.
- **Prefer an adjustable sled for frequent payload changes.** If payloads change often, a motorized sled with an auto-balance routine running Phase B is worth the complexity.
- **Re-tune the controller** with the measured I_yy after balancing, and document the gains alongside the mass properties.