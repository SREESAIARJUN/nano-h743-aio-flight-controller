# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning

*I don't have your project's Cycle 7 documents, so this procedure is general. Placeholders in **[brackets]** mark values to replace with your design data, such as motor torque, payload mass, and the lens list.*

---

## 1. Coordinate Frame and Objectives

**Frame (cradle body frame, origin on the pitch bearing axis):**
- **x**: pitch axis, through the bearing centers
- **y**: optical axis, forward
- **z**: up when the camera is level (θ = 0)

**Payload model:** total mass *m*, CG at **r_c** = (x_c, y_c, z_c), and inertia tensor about the origin:

$$
\mathbf{I}_O=\begin{bmatrix} I_{xx} & -P_{xy} & -P_{xz}\\ -P_{xy} & I_{yy} & -P_{yz}\\ -P_{xz} & -P_{yz} & I_{zz}\end{bmatrix},\quad P_{ij}=\sum m\,i\,j
$$

**Balance objectives, in priority order:**

| # | Objective | Condition | Why it matters |
|---|---|---|---|
| 1 | Static balance about pitch | y_c = 0, z_c = 0 | Removes the gravity torque the motor would otherwise hold continuously |
| 2 | Dynamic balance (principal alignment) | P_xy = 0, P_xz = 0 | Stops roll/yaw motion from coupling torque into pitch |
| 3 | Lateral CG placement | x_c = x_target (usually the roll-axis plane) | Supports roll/yaw balance of the outer stages |
| 4 | Inertia magnitude | I_xx within its target band | Sets control bandwidth and the structural/motor inertia ratio |

---

## 2. Governing Equations

**Gravity torque on pitch** (gimbal base level, pitch angle θ):

$$
\tau_g(\theta) = m g\,(y_c\cos\theta - z_c\sin\theta)
$$

A y-offset shows up as torque when the camera is level. A z-offset ("top/bottom heaviness") shows up when the camera is tilted, and it also changes the apparent pendulum stiffness.

**Dynamic coupling into pitch** (Euler's equation, x-component, with body rates ω):

$$
\tau_x = I_{xx}\dot\omega_x - P_{xy}(\dot\omega_y - \omega_x\omega_z) - P_{xz}(\dot\omega_z + \omega_x\omega_y) + (I_{zz}-I_{yy})\omega_y\omega_z + P_{yz}(\omega_z^2-\omega_y^2)
$$

The P_xy and P_xz terms let roll and yaw accelerations from the vehicle or outer axes inject disturbance torque into pitch. Nulling them is the "tensor tuning" step.

**Trim mass effect:** adding a point mass m_k at (x_k, y_k, z_k) changes the properties as follows:

$$
\Delta(m y_c)=m_k y_k,\quad \Delta(m z_c)=m_k z_k,\quad \Delta P_{xy}=m_k x_k y_k,\quad \Delta P_{xz}=m_k x_k z_k,\quad \Delta I_{xx}=m_k(y_k^2+z_k^2)
$$

---

## 3. Adjustment Provisions in the Cradle Design

| Adjustment | Mechanism | Controls |
|---|---|---|
| Fore/aft slide | Dovetail or Arca rail with scale, ±[20] mm | y_c (coarse) |
| Vertical riser | Shim stack or slotted riser, ±[10] mm | z_c (coarse) |
| Lateral slide | Slotted side-plate interface | x_c |
| Trim mass ports | Threaded inserts at ≥4 locations, at large ±x and at different y, z | P_xy, P_xz, fine y_c/z_c |
| Trim mass set | Brass/tungsten screws, [0.5–10] g, in 0.5 g steps | Fine tuning |

**Design rule:** place the trim ports as far apart in ±x as possible. Product-of-inertia correction scales with x_k, so wide spacing gives the most correction for the least added mass and inertia.

---

## 4. Procedure

### Phase A: Analytical Prediction (CAD)
1. Build the full mass model: cradle, camera body, each lens in the **[lens list]**, cables with their strain-relief geometry, and filters or follow-focus motors.
2. Export **r_c** and **I_O** for each payload configuration.
3. Solve for nominal slide positions so that y_c = z_c = 0.
4. Solve the trim-mass problem (Section 5) for P_xy and P_xz.
5. Check that I_xx falls within the target band (Section 6).
6. Record a prediction table for each configuration. These values serve as the Cycle 7 baseline.

### Phase B: Coarse Static Balance (Physical, Motors Off)
1. Level the gimbal base to within 0.1°. Lock roll and yaw.
2. **Fore/aft (y_c):** at θ = 0, release the payload and adjust the slide until it shows no tendency to rotate.
3. **Vertical (z_c):** set θ = 45° and release.
   - If it falls toward 90°, the payload is top-heavy: lower the camera.
   - If it returns to 0°, it is bottom-heavy: raise the camera.
   - Iterate until it stays put at 45°, then verify at −45° and ±90°.
4. Repeat step 2, because the y and z adjustments interact once the camera is tilted.
5. Mark the slide positions for this configuration.

### Phase C: Quantitative Residual Measurement (Motor Current Method)
1. Enable the pitch motor in position hold. Use a low-bandwidth current measurement and average for at least 2 s per point.
2. Measure the holding torque τ = K_t · I_q at θ = {−90, −60, …, +90}°, approaching each angle from both directions to cancel friction.
3. Least-squares fit τ(θ) = A cosθ + B sinθ + C, which gives:
   - y_c = A / (mg)
   - z_c = −B / (mg)
   - C is cable or wiring bias. Note it, and fix the routing if it is significant.
4. Convert the residual offsets to slide or trim corrections and apply them. Iterate until the acceptance criteria are met.

### Phase D: Dynamic Balance (Products of Inertia)
1. Unlock roll and yaw and hold pitch in stiff position control.
2. Excite the roll axis with a sinusoidal or chirp acceleration α_y, holding yaw fixed. Record pitch motor torque.
3. The in-phase pitch torque component is −P_xy · α_y. Solve for P_xy.
4. Repeat with yaw excitation to find P_xz.
5. **Alternative:** if outer-axis excitation is not available, measure on a bifilar or trifilar pendulum, or with an inertia measurement machine, about at least 6 axis orientations, and reconstruct the full tensor.
6. Compute trim masses (Section 5), install them, and re-verify static balance, since trims shift the CG.

### Phase E: Verification
1. Repeat Phase C. The static residual must still pass.
2. Repeat Phase D. The product residuals must pass.
3. Measure I_xx from the closed-loop plant identification: low-frequency slope of the pitch torque-to-acceleration frequency response.
4. Run a disturbance test: apply a roll/yaw step or vibration profile **[spec]**, measure pitch pointing error, and compare against the Cycle 6 result.

---

## 5. Trim Mass Solution

Use two trim ports, k = 1, 2, placed at ±x. To null the residual products P_xy⁰ and P_xz⁰ without shifting the CG, use a pair of masses on opposite sides. This adds a product without moving y_c or z_c:

$$
m_1 = m_2 = m_t,\; (x_1,y_1,z_1)=(+a, b, c),\; (x_2,y_2,z_2)=(-a,-b,-c)
$$
$$
\Delta P_{xy}=2m_t a b,\quad \Delta P_{xz}=2m_t a c,\quad \Delta(my_c)=\Delta(mz_c)=0
$$

Choose b and c from the available port positions so that b : c = P_xy⁰ : P_xz⁰, with the opposite sign. Then:

$$
m_t = \frac{\sqrt{(P^0_{xy})^2+(P^0_{xz})^2}}{2a\sqrt{b^2+c^2}}
$$

**General case** (N ports with discrete masses): solve the constrained least-squares problem

$$
\min_{m_k\ge0}\;\Big\|\mathbf{A}\mathbf{m}+\mathbf{b}_0\Big\|_W^2+\lambda\sum m_k
$$

Each column of **A** is [y_k, z_k, x_k y_k, x_k z_k]ᵀ for port k. The vector **b**₀ holds the residual [m y_c, m z_c, P_xy, P_xz]. The λ term penalizes added mass and inertia. Round the result to the available mass increments and re-check.

---

## 6. Inertia Magnitude Tuning

- **Lower bound:** I_xx should be large enough that the reflected motor rotor inertia, ideally ≤ [10–20]% of the load, does not dominate. This keeps the plant well-conditioned.
- **Upper bound:** I_xx must allow the required angular acceleration: τ_motor,cont ≥ I_xx · α_req + τ_residual + τ_friction, with a [2×] margin.
- **Structural:** the first cradle mode should be at least [5–10]× the pitch control bandwidth. Adding inertia far from the axis lowers this mode, so check it with FEA after trims are added.
- **Across configurations:** keep I_xx variation within [±30]% if a single controller gain set must serve all lenses. Otherwise, schedule gains per lens.

---

## 7. Acceptance Criteria (Proposed, to Be Confirmed)

| Parameter | Criterion |
|---|---|
| Static residual torque, max over θ | ≤ [3]% of continuous motor torque, or CG offset ≤ [0.2] mm |
| z_c (pendulosity) | ≤ [0.3] mm. A slightly bottom-heavy bias may be specified for power-off safety. |
| P_xy, P_xz | ≤ [1]% of I_xx·(1 m) equivalent, or coupled pointing error ≤ [x] mrad under the disturbance spec |
| Cable bias C | ≤ [1]% of continuous torque |
| Added trim mass | ≤ [3]% of payload mass |
| First structural mode | ≥ [8]× control bandwidth |

---

## 8. Cycle 7 Documentation Deliverables

1. Per-configuration table: CAD-predicted values vs. measured r_c, P_xy, P_xz, I_xx.
2. Slide and shim settings plus the trim mass map for each lens. This becomes the operator balancing card.
3. Residual torque fit plots from Phase C, before and after correction.
4. Disturbance rejection comparison against Cycle 6.
5. Lessons learned and design changes for Cycle 8. Examples: relocating trim ports, increasing slide travel, cable routing fixes.

---

If you share the Cycle 7 specifics, I can replace the placeholders with computed values and solve the trim distribution for your geometry. The most useful inputs are:
- payload mass properties
- motor K_t and torque rating
- trim port locations
- control bandwidth target