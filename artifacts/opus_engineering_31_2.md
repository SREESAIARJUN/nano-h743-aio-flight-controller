# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning (Cycle 31)

> **Note:** I don't have your program's Cycle 30 results, CAD data, or requirement values. The procedure below is complete and general. Bracketed fields such as `[ ]` are where your Cycle 31 numbers go. If you share the Cycle 30 residuals, payload configuration, and motor specs, I can turn this into concrete trim-mass values.

---

## 1. Reference Frame and Definitions

- **Origin O:** the intersection of the pitch bearing axis with the cradle mid-plane. This is the bearing-centered origin.
- **Axes:**
  - **y** lies along the pitch rotation axis.
  - **x** points along the optical axis at θ = 0.
  - **z** completes the right-handed set (up at level).
- **Cradle assembly:** the camera, lens, cradle, filters, cables, and any rotating encoder or motor parts.

**Inertia tensor about O:**

$$
\mathbf{J}_O=\begin{bmatrix}J_{xx}&J_{xy}&J_{xz}\\J_{xy}&J_{yy}&J_{yz}\\J_{xz}&J_{yz}&J_{zz}\end{bmatrix},\qquad J_{ij}=-\!\int r_i r_j\,dm\ (i\neq j)
$$

**Balance targets:**

| Quantity | Condition | Reason |
|---|---|---|
| Static (CG) | x_c = 0, z_c = 0 | Removes the gravity torque about pitch at every angle |
| Dynamic | J_xy = 0, J_yz = 0 | Makes y a principal axis, so pitch motion does not couple into roll or yaw |
| Axial CG | y_c = y_target (usually 0) | Keeps the roll and yaw stages balanced |
| Pitch inertia | J_yy within [J_min, J_max] | Meets the control bandwidth and disturbance rejection requirements |

## 2. Governing Equations

**Gravity torque about pitch** (z up in the world frame, cradle pitched by θ):

$$
\tau_g(\theta)= m g\,(x_c\cos\theta - z_c\sin\theta)
$$

A residual CG offset therefore produces a sinusoidal torque. Its amplitude is $m g\sqrt{x_c^2+z_c^2}$.

**Dynamic coupling** for pure pitch motion (ω = ω ŷ, α = α ŷ), from Euler's equation τ = Jα + ω × (Jω):

$$
\boldsymbol\tau=\begin{bmatrix}J_{xy}\alpha + J_{yz}\omega^2\\ J_{yy}\alpha\\ J_{yz}\alpha - J_{xy}\omega^2\end{bmatrix}
$$

The x and z components are reaction torques that the roll and yaw motors must reject. This is why J_xy and J_yz are driven to zero.

**Coupling acceptance check:**

$$
|J_{xy}|,|J_{yz}| \le \frac{\tau_{\text{allow,roll/yaw}}}{\alpha_{\max}+\omega_{\max}^2}
$$

## 3. Trim Formulation (Two-Plane Balancing)

Add trim masses $m_k$ at positions $(x_k, y_k, z_k)$, usually at two y-stations on opposite sides of the cradle (left and right planes). The baseline has mass $m_0$, first moments $S_x = m_0 x_c$ and $S_z = m_0 z_c$, and products $J_{xy}^0, J_{yz}^0$.

**Constraint set** (linear in $m_k$ when positions are fixed):

$$
\begin{aligned}
S_x+\sum m_k x_k &= 0\\
S_z+\sum m_k z_k &= 0\\
J_{xy}^0-\sum m_k x_k y_k &= 0\\
J_{yz}^0-\sum m_k y_k z_k &= 0\\
S_y+\sum m_k y_k &= m_{tot}\,y_{target}
\end{aligned}
$$

There are 5 equations, so you need at least 5 mass degrees of freedom. In practice, use 2 planes × (x, z) adjustable positions, or 4–6 discrete trim pockets.

**Solve as a constrained least-squares problem:**

$$
\min_{m_k\ge0}\ \sum m_k \quad\text{s.t.}\quad A\mathbf m=\mathbf b,\ \ J_{yy}^0+\sum m_k(x_k^2+z_k^2)\in[J_{min},J_{max}]
$$

- If $J_{yy}$ is too high, prefer slide-to-balance (repositioning the camera on its dovetail) over adding mass. Moving the payload along x by Δx shifts $x_c$ without adding mass. Move first, then trim the residual.
- **Parallel-axis update for each added mass:**
  - $\Delta J_{xx}=m(y^2+z^2)$
  - $\Delta J_{yy}=m(x^2+z^2)$
  - $\Delta J_{zz}=m(x^2+y^2)$
  - $\Delta J_{xy}=-mxy$, and similarly for the other products.

## 4. Inertia Tuning Criteria for J_yy

- **Bandwidth:** for a target closed-loop bandwidth $\omega_{bw}$ and peak motor torque $\tau_{pk}$, require $J_{yy}\,\alpha_{req} \le 0.7\,\tau_{pk}$ (30% margin).
- **Disturbance rejection:** a higher $J_{yy}$ attenuates high-frequency base disturbances. Do not reduce it below the stability floor set by the structural mode.
- **Structural separation:** keep the first cradle mode, $f_1 \approx \frac{1}{2\pi}\sqrt{k_{\theta}/J_{yy}}$, at least 3–5× above the control bandwidth.
- **Inertia ratio:** keep $J_{yy}/J_{rotor}$ within the motor or driver's recommended range, typically ≤ 10–30 for direct drive.

## 5. Measurement Procedure

**Step 0: Preparation**
1. Use the final flight or production payload configuration: lens, focus motor, filters, hood, and cables dressed and tied.
2. Set focus and zoom to the balance reference position `[ ]`. Record the CG shift across the zoom range separately.
3. Thermally soak the assembly at `[ ]` °C. Bearing preload affects friction readings.

**Step 1: Baseline from CAD**
- Export mass, CG, and the full tensor about O. Log $m_0$, $x_c$, $y_c$, $z_c$, and all six J terms.

**Step 2: Static residual (hardware)**
1. Unpower the motor, or use current-sense mode.
2. Measure the hold current $i(\theta)$ at θ = −90° to +90° in 15° steps. Average forward and reverse sweeps to cancel friction.
3. Compute $\tau(\theta)=K_t\,i(\theta)$, then fit $\tau = a\cos\theta + b\sin\theta + c$. This gives:
   - $x_c = a/(mg)$
   - $z_c = -b/(mg)$
   - c is the friction or cable-bias term
4. **Alternative:** a free-release test. The cradle should remain at rest at any angle within the friction band.

**Step 3: Moment of inertia J_yy**
- **Method A (spin-up):** apply a known torque step and measure α from the encoder. $J_{yy} = (\tau - \tau_f)/\alpha$.
- **Method B (pendulum):** add a known offset mass $m_p$ at radius r and measure the period T.

  $$
  J_{yy} + m_p r^2 = \frac{m_p g r T^2}{4\pi^2}
  $$

**Step 4: Products of inertia**
- Run a sinusoidal pitch sweep at amplitude A and frequency ω. Measure the reaction torque on the locked roll and yaw axes using motor current or a load cell.
- Fit the result to the Section 2 coupling equations to extract J_xy and J_yz.
- If there is no test rig, use the CAD values corrected for measured payload CG offsets.

**Step 5: Solve and install trim**
- Solve the Section 3 problem.
- Round to the available trim increments `[ ]` g.
- Secure with threadlocker or witness marks.

**Step 6: Verify**
- Repeat Steps 2–4.
- Iterate until within limits. The typical maximum is 2 iterations, given linear sensitivity.

## 6. Acceptance Criteria (Template)

| Parameter | Limit | Cycle 30 | Cycle 31 pre-trim | Cycle 31 post-trim |
|---|---|---|---|---|
| Residual unbalance $m\sqrt{x_c^2+z_c^2}$ | ≤ [ ] g·mm | | | |
| Peak gravity torque | ≤ [ ]% of τ_cont | | | |
| \|J_xy\| | ≤ [ ] kg·m² | | | |
| \|J_yz\| | ≤ [ ] kg·m² | | | |
| y_c offset | ≤ [ ] mm | | | |
| J_yy | [J_min, J_max] | | | |
| First mode f₁ | ≥ [ ] Hz | | | |
| CG shift across zoom range | ≤ [ ] mm | | | |
| Total trim mass added | ≤ [ ] g | | | |

## 7. Cycle 31 Iteration Notes

- **Carry-over check:** compare the measured-minus-CAD deltas from Cycle 30. If they are consistent, apply them as a correction bias to the Cycle 31 CAD prediction before trimming.
- **Sensitivity matrix:** record ∂(residual)/∂(m_k) from this cycle's measurements. Reusing it lets future cycles trim in a single pass.
- **Configuration control:** log trim mass part numbers, locations, and torque values. Re-balance whenever the lens, filter, or cable routing changes.
- **Downstream effect:** after pitch balancing, re-check the roll and then yaw stage balance with the trimmed cradle installed. Changes in pitch-stage mass shift their CGs.

---

If you provide the payload mass and properties, the Cycle 30 residuals, the trim pocket locations, and the motor $K_t$ and torque limits, I can solve the trim-mass set and fill in the acceptance table numerically.