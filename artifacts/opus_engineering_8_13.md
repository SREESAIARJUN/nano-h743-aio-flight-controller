# Pitch-Axis Camera Cradle: Mechanical Balancing & Inertia Tensor Tuning Procedure (Cycle 8)

I don't have your program's Cycle 7 records, payload specs, or tolerance budget. This procedure is therefore a complete general framework with placeholder limits, marked **[TBD]**. Replace them with your project values.

---

## 1. Coordinate Frame & Definitions

Define the **cradle frame {C}** with its origin on the pitch bearing axis, at the midpoint between the bearings:

| Axis | Direction | Notes |
|---|---|---|
| **y_C** | Pitch rotation axis | Bearing centerline |
| **x_C** | Optical axis (forward) | At pitch = 0° |
| **z_C** | Completes the right-handed set | Up at pitch = 0° |

The rotating assembly includes the cradle, camera body, lens, plate, fasteners, cable loop share, and trim masses. It has:

- Mass **m**
- Center of mass **r_cm = (x_cm, y_cm, z_cm)** in {C}
- Inertia tensor about the {C} origin:

$$
\mathbf{I}_C=\begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz}\\ -I_{xy} & I_{yy} & -I_{yz}\\ -I_{xz} & -I_{yz} & I_{zz}\end{bmatrix},\quad I_{xy}=\int xy\,dm,\ \text{etc.}
$$

---

## 2. Why Each Term Matters (Balancing Objectives)

### 2.1 Static balance: x_cm, z_cm → 0
The gravity torque about the pitch axis is

$$\tau_g(\theta)= m g\,(x_{cm}\cos\theta - z_{cm}\sin\theta)$$

(the sign depends on the pitch sign convention). Any residual must be held continuously by the motor, which costs heat, current, and stiffness margin.

### 2.2 Axial CoM: y_cm
- y_cm does not create pitch torque.
- It does load the **roll axis**, so it should be minimized to support roll-axis balance.

### 2.3 Dynamic balance: I_xy, I_yz → 0
For pure pitch motion **ω** = (0, ω, 0) and **α** = (0, α, 0), Euler's equation gives the reaction moment on the bearings and outer axes:

$$\mathbf{M}=\mathbf{I}\boldsymbol\alpha+\boldsymbol\omega\times(\mathbf{I}\boldsymbol\omega)$$

$$M_x = -I_{xy}\alpha + I_{yz}\omega^2,\qquad M_z = -I_{yz}\alpha - I_{xy}\omega^2$$

Non-zero products of inertia cause two problems:
- **Pitch accelerations inject disturbance torques into the roll and yaw loops.** This is the dominant cross-coupling source in fast-slew gimbals.
- **The principal axis tilts away from the bearing axis.**

### 2.4 I_xz
- I_xz does not couple pitch motion.
- It does affect roll/yaw dynamics when the cradle is pitched. Track it, but it is secondary.

### 2.5 I_yy (tuning target, not a zero-target)
I_yy sets:
- Torque demand: τ = I_yy·α_max
- Plant gain for the control loop
- The inertia ratio to the motor rotor

---

## 3. Acceptance Criteria (Cycle 8 Targets)

| Parameter | Target | Rationale |
|---|---|---|
| \|x_cm\|, \|z_cm\| | ≤ **[TBD, e.g., 0.10 mm]** | Residual τ_g ≤ 5% of motor continuous torque |
| \|y_cm\| | ≤ **[TBD, e.g., 0.5 mm]** | Roll-axis balance budget |
| \|I_xy\|, \|I_yz\| | ≤ **[TBD, e.g., 2% of I_yy]** | Cross-axis disturbance below roll/yaw loop rejection floor |
| I_yy | **[TBD]** ± 10% | Matches the controller gain schedule |
| Principal-axis tilt from y_C | ≤ **[TBD, e.g., 0.5°]** | Equivalent check on products of inertia |
| First structural mode of cradle | ≥ 4–5 × pitch loop bandwidth | Avoids exciting resonance during tuning |
| Holding-current variation over ±90° | ≤ **[TBD] mA** | Practical static-balance verification |

---

## 4. Procedure

### Phase A: Predictive Model (pre-hardware)

1. **Update the CAD mass model** with weighed actuals for all Cycle 8 components. Do not rely on material-density estimates for:
   - Camera body
   - Lens
   - Cable bundle
2. **Extract r_cm and I_C** about the {C} origin. If the CAD tool reports values about the CoM, transform them with the parallel-axis theorem:
$$\mathbf{I}_C=\mathbf{I}_{cm}+m\left[(\mathbf{r}^T\mathbf{r})\mathbf{E}-\mathbf{r}\mathbf{r}^T\right]$$
3. **Check payload variants.** Run the model across each configuration, such as:
   - Zoom extremes
   - Focus extremes
   - Filters
   - Lens options

   Then record the CoM envelope. If the envelope exceeds the static tolerance, Cycle 8 needs an **adjustable slide** or a **motorized balance**, not fixed trim.
4. **Pre-compute trim locations.** Pick two axial trim planes at y = ±b, as far apart as practical. Each trim mass m_t at (x_t, y_t, z_t) changes the CoM and products by:
$$\Delta(m\,x_{cm}) = m_t x_t,\quad \Delta I_{xy} = m_t x_t y_t,\quad \Delta I_{yz}= m_t y_t z_t$$
   Two planes give enough degrees of freedom to null the static terms (x, z) and the products (I_xy, I_yz) at the same time. This is classic two-plane balancing, and Section 5 gives the equations.

### Phase B: Coarse Static Balance (hardware, motor unpowered)

1. Mount the payload in its **reference configuration** (define it, e.g., wide end, focus at infinity).
2. **Fore-aft (x):** set pitch to 0° and release. Slide the camera plate until the cradle holds without rotating.
3. **Vertical (z):** set pitch to 90° (lens up) and release. Adjust the vertical offset until the cradle holds.
4. Repeat steps 2–3. The adjustments interact, so converge in 2–3 iterations.
5. **Check at ±45°.** Neutral equilibrium at all angles confirms both x_cm and z_cm are near zero.
6. **Lock the plate** and torque the fasteners to spec. Re-check, because fastening often shifts the CoM.

### Phase C: Fine Static Balance (motor powered, current-based)

1. Enable the pitch loop in position-hold mode with low bandwidth.
2. Step through θ = −90° to +90° in 15° increments. At each step, dwell and log the averaged q-axis current i_q.
3. Fit the data by least squares:
$$K_t\, i_q(\theta) = a\cos\theta + b\sin\theta + c$$
   - a = m g x_cm
   - b = −m g z_cm
   - c = friction or cable spring bias. Run both directions and average to separate hysteresis.
4. Compute the residual offsets and apply the fine trim. Repeat until the acceptance criteria are met.
5. **Cable check:** a sinusoidal residual that persists after trimming often comes from **cable-loop stiffness**, not mass. Re-route the cables before chasing it with trim masses.

### Phase D: Dynamic Balance (products of inertia)

Choose a method based on available equipment:

**Option 1: In-gimbal sinusoidal excitation (preferred, no extra fixtures)**
1. Lock the roll and yaw axes, or hold them under stiff position control.
2. Drive pitch with a sinusoid θ = A sin(2πft) at several frequencies, keeping amplitudes inside mechanical limits.
3. Log the roll and yaw holding torques (motor currents × K_t).
4. Fit the logged torques to the relations in Section 2.3:
   - The component in phase with α gives I_xy and I_yz.
   - The ω² component gives a cross-check.
5. Correct the imbalance using the two-plane equations in Section 5.

**Option 2: Bench measurement**
- Use a trifilar or compound pendulum, or a commercial MOI/POI instrument.
- Measure about the pitch axis plus at least 3 skewed axes.
- Each measurement I_n = **n**ᵀ**I n** is one linear equation in the six tensor components. Solve with ≥ 6 orientations by least squares.

**Option 3: CAD-only**
- Acceptable only if a Cycle 7 validation showed model error < 10% on the products of inertia.

### Phase E: I_yy Tuning

1. **Measure I_yy.** Apply a small chirp or step torque and fit τ = I_yy α + B ω + τ_c. Alternatively, use a bench pendulum.
2. **Compare against the requirements:**
   - Torque margin: I_yy·α_max + τ_residual ≤ 0.7 × τ_continuous (adjust to your derating).
   - Inertia ratio: J_load / J_rotor within the drive's stable range. For direct drive this is typically benign; for geared drives, scale by N².
   - Controller: if I_yy deviates more than 10% from the value used in the Cycle 7 gain design, rescale the loop gains in proportion (K_p, K_d ∝ I) or re-identify.
3. **Reduce I_yy** if required by:
   - Moving mass radially toward the axis
   - Lightening the cradle arms
   - Using trim masses as close to the axis as the balance allows

   Placing trim mass at radius r adds m_t r² to I_yy, so prefer **many small masses at short radius over large masses at long radius** only when the authority is sufficient.
4. **Check the structural mode.** Tap-test or sweep the cradle and confirm the first mode is well above the loop bandwidth. A heavier cradle tuned for inertia can drop this mode.

### Phase F: Verification & Regression

1. Repeat Phase C at each payload variant. Confirm the CoM envelope is inside the tolerance or within slide-adjustment range.
2. Run a full-rate pitch slew profile and record:
   - Roll and yaw disturbance torques (cross-coupling)
   - Pitch tracking error
   - Motor temperature rise
3. Compare against the Cycle 7 baseline and the Section 3 targets.
4. **Thermal soak** (if the operating range demands it): re-check the static balance after a temperature cycle. Differential expansion and lens creep can shift the CoM.

---

## 5. Two-Plane Trim Solution

Place trim masses at planes y = +b (plane 1) and y = −b (plane 2), and let **(u_i, w_i) = m_i·(x_i, z_i)** be each trim's moment vector. To null the measured residuals (S_x = m·x_cm, S_z = m·z_cm, I_xy, I_yz):

$$
\begin{aligned}
u_1 + u_2 &= -S_x & \quad b(u_1 - u_2) &= -I_{xy}\\
w_1 + w_2 &= -S_z & \quad b(w_1 - w_2) &= -I_{yz}
\end{aligned}
$$

$$
u_{1,2} = \tfrac{1}{2}\left(-S_x \mp \tfrac{I_{xy}}{b}\right),\qquad
w_{1,2} = \tfrac{1}{2}\left(-S_z \mp \tfrac{I_{yz}}{b}\right)
$$

Then realize each plane's (u, w) as a mass at the radius and angle you choose: m_i = √(u_i² + w_i²)/r_i at angle atan2(w_i, u_i). Iterate once after installing, because the trim hardware itself adds small product-of-inertia errors.

---

## 6. Cycle 8 Record Template

| Item | Cycle 7 | Cycle 8 Predicted | Cycle 8 Measured | Limit | Pass |
|---|---|---|---|---|---|
| m (kg) | | | | | |
| x_cm / z_cm (mm) | | | | | |
| y_cm (mm) | | | | | |
| I_yy (kg·m²) | | | | | |
| I_xy / I_yz (kg·m²) | | | | | |
| Principal axis tilt (°) | | | | | |
| Holding-current p-p (mA) | | | | | |
| Peak roll/yaw coupling torque (N·m) | | | | | |
| First cradle mode (Hz) | | | | | |
| Trim masses (plane, mass, r, angle) | | | | | |

---

To tailor this to Cycle 8 specifically, useful inputs would be:
- Payload mass and variants
- Motor torque constant and continuous rating
- Pitch loop bandwidth
- Whether balance is manual-slide, fixed-trim, or motorized
- Any Cycle 7 findings (e.g., cross-coupling issues) that drove this iteration

With those, I can fill in the TBD tolerances and run the trim and torque-budget numbers.