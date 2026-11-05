# Pitch-Axis Camera Cradle: Balancing Procedure and Inertia Tensor Tuning (Cycle 16)

I don't have your program's Cycle 16 baseline data (cradle mass properties, motor specs, prior-cycle residuals, or acceptance limits). The formulation below is general. Fields marked **[TBD]** should be filled in from your configuration records.

---

## 1. Reference Frame and Definitions

**Frame P (cradle body frame):**
- Origin O at the intersection of the pitch axis and the mid-plane between the bearings.
- **y-axis** along the pitch rotation axis.
- **x-axis** along the optical axis at θ = 0.
- **z-axis** completes the right-handed set (up at θ = 0).

**Payload state.** Cradle, camera, lens, filters, cables, and connectors in flight configuration, with lens zoom and focus at the declared balance position **[TBD]**.

**Quantities:**
- m: total rotating mass about pitch.
- **r_c** = (x_c, y_c, z_c): center of mass in frame P.
- Inertia tensor about O:

$$
\mathbf{I}_O=\begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz}\\ -I_{xy} & I_{yy} & -I_{yz}\\ -I_{xz} & -I_{yz} & I_{zz}\end{bmatrix},\quad I_{xy}=\int xy\,dm,\ \text{etc.}
$$

---

## 2. Balance Objectives

### 2.1 Static balance (first moment)

The gravity torque about pitch is:

$$
\tau_g(\theta) = m g\,(x_c\sin\theta + z_c\cos\theta) \quad (\text{sign per convention})
$$

**Requirement:** x_c → 0 and z_c → 0. The y_c offset produces no pitch torque, but it does load the roll and yaw axes, so trim it as a secondary objective.

**Acceptance:** m·√(x_c² + z_c²) ≤ **[TBD]**, for example ≤ 2–5 % of continuous motor torque divided by g.

### 2.2 Dynamic balance (products of inertia)

For pitch motion (ω_y, α_y), the reaction torques transmitted to the outer axes are:

$$
\tau_x = -I_{xy}\,\alpha_y + I_{yz}\,\omega_y^2,\qquad
\tau_z = -I_{yz}\,\alpha_y - I_{xy}\,\omega_y^2
$$

**Requirement:** I_xy → 0 and I_yz → 0, so that the y-axis is a principal axis. This decouples pitch slews from roll and yaw disturbances.

The residual principal-axis tilt in the x–y plane is:

$$
\phi_{xy}=\tfrac12\arctan\!\frac{2I_{xy}}{I_{xx}-I_{yy}}
$$

The same form applies in the y–z plane. **Acceptance:** |φ| ≤ **[TBD]**, typically ≤ 0.5°.

### 2.3 Inertia tuning (I_yy)

The plant is G(s) = 1/(I_yy s²), so I_yy sets the achievable stabilization performance.

| Goal | Criterion |
|---|---|
| Disturbance rejection | Larger I_yy gives more inertial filtering of base jitter above bandwidth |
| Slew/acceleration | α_max = τ_max / I_yy ≥ **[TBD]** rad/s² |
| Motor-to-load inertia ratio | J_load/J_rotor within driver-tuned range (direct drive: per vendor) |
| Structural separation | First cradle/camera mode f_s ≥ 3–5× position-loop crossover f_c |
| Loop gain consistency | I_yy within ±**[TBD]** % of the value used in controller tuning (re-tune if outside) |

---

## 3. Measurement Methods

### 3.1 Static: motor-current sweep (in situ)

1. Put the pitch loop in position hold at slow-rate stations θ_k (for example every 15° over the full travel).
2. Record the averaged torque-producing current i_q,k at each station, averaging both approach directions to cancel friction.
3. Fit:
$$
\tau_k = K_t\, i_{q,k} = A\sin\theta_k + B\cos\theta_k + C
$$
4. Solve: m g x_c = A, m g z_c = B, and C is the cable/spring bias.

Cable stiffness shows up as a linear-in-θ term. Add D·θ to the fit if the residuals show a trend.

### 3.2 Static: free-hang (motors unpowered, quick check)

Release the cradle at several angles. It should exhibit neutral equilibrium. The settling angle indicates the direction of the heavy side.

### 3.3 Inertia I_yy

- **Option A, torsional pendulum:** a calibrated torsion fixture on the pitch axis. I_yy = k_t T² / 4π², minus fixture inertia.
- **Option B, in-loop identification:** inject a chirp or PRBS torque and fit the 1/(Js²) asymptote of the measured frequency response between 2–10× crossover, after subtracting rotor inertia.

### 3.4 Products of inertia I_xy and I_yz

- **Option A, reaction measurement:** drive pitch with a sinusoidal acceleration α_y = α₀ sin ωt while holding roll and yaw. Measure the roll and yaw holding torques, then fit the τ_x and τ_z equations from 2.2. The in-phase-with-α component gives the I terms. Use two frequencies to separate the ω² terms.
- **Option B, CAD/mass-properties:** good for predictions. Validate against Option A for the flight configuration.

---

## 4. Counterweight Solution

Place n trim masses m_i at allowable hard-points **r_i** = (x_i, y_i, z_i). The constraints are linear in m_i for fixed locations:

$$
\begin{aligned}
\sum m_i x_i &= -m\,x_c && \text{(static, x)}\\
\sum m_i z_i &= -m\,z_c && \text{(static, z)}\\
\sum m_i x_i y_i &= -I_{xy} && \text{(dynamic)}\\
\sum m_i y_i z_i &= -I_{yz} && \text{(dynamic)}
\end{aligned}
$$

Solve as **A m = b**:

$$
\min_{m}\ \|\mathbf{m}\|_1 \text{ or } \sum m_i \quad \text{s.t. } \mathbf{A}\mathbf{m}=\mathbf{b},\ 0\le m_i\le m_{i,\max}
$$

Use a linear or quadratic program.

**Practical notes:**
- At least **two axial planes** (y_i of opposite sign, near the bearings) are required to null both products. Static trim alone is single-plane.
- Prefer sliding-rail adjustments (continuous x_i, z_i) for fine static trim, and fixed slugs for coarse and dynamic trim.
- Update I_yy afterward: ΔI_yy = Σ m_i(x_i² + z_i²). Check this against the 2.3 targets. Every added gram increases I_yy and lowers acceleration margin.
- If I_yy must be **raised** for jitter rejection, add symmetric mass pairs at ±x or ±z (paired so first moments and products stay zero).

---

## 5. Cycle 16 Procedure

| Step | Action | Record |
|---|---|---|
| 0 | Configuration freeze: payload P/N, lens state, cable routing per drawing **[TBD]** | Config ID |
| 1 | Mass all components; update CAD mass properties | m, r_c, **I_O** (predicted) |
| 2 | Compute counterweight solution (Sec. 4) from CAD | m_i, r_i |
| 3 | Install trim; torque-seal fasteners per spec | Install log |
| 4 | Current-sweep static test (3.1), both directions, 3 repeats | A, B, C, residual |
| 5 | Fine static trim via rails; repeat step 4 until within limit | Final x_c, z_c |
| 6 | Dynamic reaction test (3.4A) at ≥ 2 frequencies | I_xy, I_yz, φ |
| 7 | Dynamic trim (two-plane); re-verify static (step 4) | Final products |
| 8 | I_yy identification (3.3B); compare to controller design value | I_yy, % deviation |
| 9 | Modal tap or sine sweep of cradle | f_s, ratio f_s/f_c |
| 10 | Thermal check: repeat step 4 at temperature extremes **[TBD]** | Drift of r_c |
| 11 | Lens-state sensitivity: repeat step 4 at zoom/focus extremes | Δr_c envelope |
| 12 | If I_yy is outside tolerance: re-tune gains, verify phase/gain margins | Margins |
| 13 | Sign-off against acceptance table; archive for Cycle 17 baseline | Report |

---

## 6. Acceptance Table (fill per program)

| Parameter | Limit | Cycle 15 | Cycle 16 |
|---|---|---|---|
| Static unbalance m·r (g·mm) | [TBD] | | |
| \|I_xy\|, \|I_yz\| (kg·m²) | [TBD] | | |
| Principal-axis tilt φ | ≤ 0.5° [TBD] | | |
| I_yy vs design | ±10 % [TBD] | | |
| Gravity torque / continuous torque | ≤ 5 % [TBD] | | |
| f_s / f_c | ≥ 4 [TBD] | | |
| Thermal r_c drift | [TBD] | | |
| Lens-state unbalance envelope | [TBD] | | |

---

## 7. Common Pitfalls

- **Cables:** cable spring torque often mimics unbalance. Fit and remove the linear-in-θ term, and balance with cables in their flight routing.
- **Friction:** single-direction sweeps bias the result by the friction torque. Always average CW and CCW approaches.
- **Changing optics:** zoom lenses shift r_c. Balance at the most-used state and verify that the worst case is still within limits.
- **Outer-axis dependence:** pitch static balance must be completed before roll and yaw balancing, because the pitch cradle's CoM feeds the outer axes' mass properties.

If you can share the Cycle 16 payload mass properties, available trim hard-points, motor torque constant, and loop bandwidth, I can compute the specific counterweight solution and the inertia tolerance band.