# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning Procedure (Cycle 21)

> **Scope note:** I don't have your Cycle 21 design data (CAD, payload mass, motor spec, prior cycle results). This procedure uses a general formulation for a 3-axis gimbal pitch cradle. Replace the bracketed placeholders `[ ]` with your program's values. The acceptance thresholds shown are typical starting points, not requirements.

---

## 1. Definitions and Frames

**Cradle body frame {C}.** The origin O sits on the pitch bearing axis, at the intersection with the roll-axis plane.
- **y_C**: pitch rotation axis
- **x_C**: optical axis (boresight) at θ_pitch = 0
- **z_C**: completes the right-hand set (up at level)

**Rotating assembly** includes the camera, lens, cradle structure, pitch motor rotor, encoder disk or magnet, cables within the service loop, and trim masses.

**Inertia tensor about O in {C}:**

$$
\mathbf{I}_O=\begin{bmatrix} I_{xx} & -P_{xy} & -P_{xz}\\ -P_{xy} & I_{yy} & -P_{yz}\\ -P_{xz} & -P_{yz} & I_{zz}\end{bmatrix},\qquad P_{ij}=\int ij\,dm
$$

**Centre of mass:**

$$
\mathbf{r}_c = (x_c, y_c, z_c)
$$

---

## 2. Balance Objectives (Physics)

### 2.1 Static balance (gravity torque)

With pitch angle θ, the gravity torque about y is:

$$
\tau_g(\theta)= m g\,(x_c\cos\theta + z_c\sin\theta)\quad\text{(sign per convention)}
$$

**Requirement:** x_c = z_c = 0.

y_c does not load the pitch motor. It does shift the CoM seen by the roll and yaw stages, so set it to the roll-axis plane, usually y_c = 0.

### 2.2 Dynamic balance (rotation coupling)

For pitch rate ω_y and acceleration ω̇_y, the angular momentum is:

$$
\mathbf{H}= \mathbf{I}_O\,\boldsymbol\omega = \omega_y(-P_{xy},\ I_{yy},\ -P_{yz})^T
$$

The reaction moment transmitted to the roll frame is:

$$
\mathbf{M}= \mathbf{I}_O\dot{\boldsymbol\omega} + \boldsymbol\omega\times\mathbf{H}
$$

This gives off-axis moments proportional to:

$$
P_{xy}\dot\omega_y,\quad P_{yz}\dot\omega_y,\quad P_{xy}\omega_y^2,\quad P_{yz}\omega_y^2
$$

These disturb the roll and yaw loops.

**Requirement:** P_xy = P_yz = 0, meaning y_C is a principal axis.

P_xz does not couple pitch motion. It does matter for roll/yaw dynamics, so record it, but it is a secondary target.

### 2.3 Inertia magnitude tuning (I_yy)

The plant is G(s) = 1 / ((I_yy + I_rotor) s²). Tune for:

| Goal | Target |
|---|---|
| Control bandwidth / authority | ω_bw ≤ √(τ_max,cont / (I_yy·θ̈_req)) margin ≥ `[2×]` |
| Inertia ratio | I_load / I_rotor ≤ `[5–10]` for direct drive with good stiffness |
| Structural separation | First cradle mode f₁ ≥ `[5–10×]` the loop crossover |
| Cross-axis ratio | I_xx, I_zz chosen so roll/yaw loads stay within those stages' envelopes |
| Payload-swap robustness | ΔI_yy across lens/config variants ≤ `[±15%]`, or gain-scheduled |

---

## 3. Analytical Pre-Balance (CAD Stage)

1. **Build the mass model.** Use measured masses for COTS items (camera, lens, battery, cable bundle) and override CAD densities.
2. **Extract m, r_c, and I_O** in {C} for every payload configuration (lens variants, zoom extremes, focus extremes if the CoM shifts).
3. **Compute the trim requirement for configuration k.** The residuals are e_k = [x_c, z_c, y_c, P_xy, P_yz]_k.
4. **Choose trim-mass sites.** Use at least two axial planes (±y) so product terms can be cancelled independently of the CoM. Use sliding rails along x and z for fine CoM adjustment.
5. **Set up the trim solve.** Each trim mass m_i at (x_i, y_i, z_i) shifts the totals:

$$
\Delta(m x_c)=\sum m_i x_i,\quad \Delta(m z_c)=\sum m_i z_i,\quad \Delta(m y_c)=\sum m_i y_i
$$

$$
\Delta P_{xy}=\sum m_i x_i y_i,\quad \Delta P_{yz}=\sum m_i y_i z_i
$$

$$
\Delta I_{yy}=\sum m_i (x_i^2+z_i^2)\ (+\text{own } I_{cm})
$$

6. **Solve the constrained least-squares problem** over all configurations:

$$
\min_{m_i,\,\mathbf r_i}\ \sum_k \| W(\mathbf e_k + A\,\mathbf u) \|^2 + \lambda \sum m_i
$$

   subject to:
   - rail/slot travel limits
   - Σm_i ≤ `[trim budget]`
   - ΔI_yy within budget

   The mass penalty λ keeps you from "balancing with lead" at the expense of inertia.
7. **Prefer moving existing mass over adding mass.** Shift the camera on the dovetail along x and z, and relocate the battery or electronics. Fix fixed-mass errors by design change if the trim needed exceeds `[20%]` of the budget.

---

## 4. Physical Balancing Procedure

### Step 0: Preparation
- Assemble the final flight configuration: camera, lens, filters, caps removed, cables routed and tied exactly as flown.
- Log the configuration ID, serial numbers, and temperature.
- Lock the roll and yaw axes level, or remove the cradle to a bearing fixture.

### Step 1: Coarse static balance (motor off)
1. Unpower or free-wheel the pitch motor. Check that cogging is negligible, or use a fixture with low-friction bearings.
2. Set the cradle at 0° (lens horizontal). Slide the camera fore/aft along x until there is no tendency to rotate. This sets z_c ≈ 0.
3. Set the cradle at 90° (lens down). Adjust height (z rail) until neutral. This sets x_c ≈ 0.
4. Check at 45° and 135°. The cradle should hold any angle.

### Step 2: Fine static balance (motor-current method)
1. Close the position loop with a low-bandwidth gain. Command θ_j over −90° to +90° in 15° steps, holding quasi-statically.
2. Record the mean motor torque τ_j = K_t · i_q,j. Average over 2 s or more, and sweep in both directions to cancel friction.
3. Fit the gravity-torque model:

$$
\tau_j = a\cos\theta_j + b\sin\theta_j + c
$$

   This gives x_c = a/(mg) and z_c = b/(mg). The term c captures cable/spring bias.
4. Move trim by Δx = −x_c and Δz = −z_c. Iterate until |τ_g|_max ≤ `[2% of continuous motor torque]`.
5. Check the cable bias c. If it exceeds `[x mN·m]`, re-route the service loop.

### Step 3: Dynamic balance (product of inertia)

**Method A: Oscillation/spin test (preferred)**
1. Mount a 3-axis accelerometer or gyro on the roll frame, or use a force/torque sensor at the cradle mount.
2. Drive pitch sinusoidally: θ = Θ sin(2πft) at f = `[2, 5, 10 Hz]`, with amplitude chosen for ω̇ near the operating maximum.
3. Measure the roll and yaw reaction moments M_x and M_z, synchronous with ω̇_y and ω_y².
4. Estimate the products of inertia by regression:

$$
M_x \approx P_{xy}\,\dot\omega_y + \ldots,\qquad M_z \approx P_{yz}\,\dot\omega_y + \ldots
$$

   (signs per frame).
5. Correct with two-plane trim. Place masses at y = ±d, with opposite-sign offsets in x (for P_xy) or z (for P_yz). This is a couple, so the CoM does not move:

$$
m_t\,(2d)\,\delta = -P
$$

   Here δ is the offset of each mass from the axis, in opposite directions.

**Method B: Roll/yaw-loop disturbance**
- With all axes active, command pitch steps. Measure the roll and yaw error transients.
- Use these results to iterate the trim until the cross-axis error is ≤ `[X mdeg]`.

### Step 4: Re-verify static balance
Two-plane couples ideally leave the CoM unchanged. Re-run Step 2 to confirm, since mounting tolerances leak into it.

### Step 5: Configuration sweep
Repeat Steps 2–3 at each lens/zoom/focus extreme.

If a single trim set cannot meet the criteria for all configurations, either:
- define per-configuration trim positions (marked detents), or
- add a motorized CoM slide or feed-forward gravity compensation, τ_ff = m g(x̂_c cos θ + ẑ_c sin θ).

---

## 5. Inertia Measurement and Tuning

### 5.1 Measuring I_yy

**Torsional/trifilar pendulum** (cradle removed, pitch axis vertical):

$$
I = \frac{m_{tot}\, g\, R^2\, T^2}{4\pi^2 L} - I_{platform}
$$

Use the parallel-axis theorem if the axis is offset.

**In-situ chirp identification:**
1. Inject a current chirp from `[1–200 Hz]`.
2. Fit the low-frequency asymptote of the plant, θ/τ = 1/(J s²), to get J = I_yy + I_rotor.
3. Subtract the datasheet rotor inertia.

**Cross-check:** CAD versus measured I_yy should agree within `[10%]`. If it does not, update the mass model before further tuning.

### 5.2 Tuning actions

| Need | Action |
|---|---|
| Reduce I_yy | Move dense items (battery, electronics) toward the axis. Use a lighter cradle with carbon arms. Avoid large trim masses at radius. |
| Increase I_yy (rarely, for disturbance rejection or noise filtering) | Add mass at radius symmetric about the axis. This has no CoM or product effect if placed in ±x/±z quads. |
| Restore principal axis after tuning | Any added inertia mass must be placed symmetrically, as a quad, about y. |
| Raise structural mode | Stiffen the arm-to-bearing joint. Check that f₁ ∝ √(k/I) still meets separation. |

### 5.3 Final tensor record

Report I_O and the principal moments/directions via eigen-decomposition. Then report the principal-axis misalignment:

$$
\phi = \angle(\hat{\mathbf y}_C,\ \hat{\mathbf e}_{principal})\ \le [0.5^\circ]
$$

---

## 6. Acceptance Criteria (Template)

| Parameter | Limit | Measured |
|---|---|---|
| Static residual torque, max over θ | ≤ `[2% τ_cont]` | |
| x_c, z_c | ≤ `[0.2 mm]` | |
| y_c (roll plane) | ≤ `[1 mm]` | |
| P_xy, P_yz | ≤ `[1% of I_yy·(rad)]`, or principal misalignment ≤ `[0.5°]` | |
| I_yy (incl. rotor) | `[target ± tol]` kg·m² | |
| Load/rotor inertia ratio | ≤ `[N]` | |
| Cross-axis disturbance on pitch step | ≤ `[X mdeg]` roll/yaw | |
| First structural mode | ≥ `[Y Hz]` | |
| Cable bias torque c | ≤ `[Z mN·m]` | |

---

## 7. Documentation for Cycle 21

1. **Configuration matrix:** each payload variant with its trim-mass positions and serials.
2. **Data:** fit coefficients (a, b, c), regression P_xy and P_yz before/after each iteration, and the I_yy measurement method and value.
3. **Deltas from Cycle 20:** mass, CoM, and tensor changes, with the root cause of any balance shifts (design change, cable routing, component substitution).
4. **Open items:** e.g., temperature-dependent CoM drift, feed-forward compensation parameters, rail detent marking.

---

If you share the Cycle 21 specifics (payload mass and variants, motor K_t and τ_cont, rotor inertia, CAD tensor, available trim locations), I can turn the placeholders into concrete numbers and solve the trim-mass placement directly.