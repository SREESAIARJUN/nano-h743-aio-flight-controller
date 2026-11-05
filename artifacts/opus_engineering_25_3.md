# Pitch-Axis Camera Cradle: Balancing Procedure and Inertia Tensor Tuning

I don't have your Cycle 25 baseline: the CAD, the payload mass properties, or the requirements doc. This is therefore written as a general formulation with placeholders such as `[m_payload]` and `[ω_bw]`. Swap in your Cycle 25 values where marked.

---

## 1. Coordinate Frame and Definitions

- **Frame C (cradle frame):** origin on the pitch bearing axis, at the midpoint between the bearings.
  - **y_C:** along the pitch rotation axis.
  - **x_C:** along the nominal optical axis (boresight) at zero pitch.
  - **z_C:** completes the right-handed set (up at zero pitch).
- **Rotating assembly:** cradle structure, camera body, lens, cables in the service loop (the rotating fraction), connectors, and balance masses.

Mass properties about the frame C origin:

$$
m = \sum m_i, \qquad \mathbf{r}_{cg} = \frac{1}{m}\sum m_i \mathbf{r}_i
$$

$$
\mathbf{I}_C =
\begin{bmatrix}
I_{xx} & -I_{xy} & -I_{xz}\\
-I_{xy} & I_{yy} & -I_{yz}\\
-I_{xz} & -I_{yz} & I_{zz}
\end{bmatrix},
\quad I_{xy} = \sum m_i x_i y_i \text{ (etc.)}
$$

---

## 2. Balance Objectives

### 2.1 Static balance (gravity torque about pitch)

The gravity torque about the pitch axis at pitch angle θ is:

$$
\tau_g(\theta) = m g \left( x_{cg}\cos\theta + z_{cg}\sin\theta \right) \quad (\text{sign per convention})
$$

**Requirement:** keep the residual below a fraction of continuous motor torque. A typical value is ≤ 5–10%.

$$
m g \sqrt{x_{cg}^2 + z_{cg}^2} \le \epsilon_s \, \tau_{cont}
$$

From this, the allowable CG offset is:

$$
e_{max} = \frac{\epsilon_s \tau_{cont}}{m g}
$$

For example, with m = 1.2 kg, τ_cont = 0.15 N·m and ε_s = 0.05, e_max ≈ 0.64 mm.

The axial CG coordinate y_cg does not produce pitch torque. However, it loads the bearings unevenly and couples into the roll/yaw axes, so keep |y_cg| within `[bearing/outer-axis limit]`.

### 2.2 Dynamic balance (cross-axis coupling)

When the outer gimbal rotates at rate **Ω** with acceleration **Ω̇**, the products of inertia involving y produce pitch-axis disturbance torques:

$$
\tau_{y,dist} = -I_{xy}\dot\Omega_x - I_{yz}\dot\Omega_z + (I_{xx}-I_{zz})\Omega_x\Omega_z + I_{xz}(\Omega_x^2-\Omega_z^2) + \dots
$$

Conversely, I_xy and I_yz couple pitch acceleration back into the roll and yaw axes.

**Requirements:**
- **Products of inertia:** |I_xy|, |I_yz| ≤ ε_d · I_yy, with ε_d typically 1–2%. This makes y_C a principal axis.
- **Asymmetry term:** the term (I_xx − I_zz) Ω_x Ω_z should stay below the stabilization error budget at the expected base-motion spectrum. Small |I_xx − I_zz| is desirable.

### 2.3 Inertia magnitude (control tuning)

I_yy sets the plant pole. For a rigid axis, G(s) = 1/(I_yy s²). Three constraints apply:

1. **Torque/acceleration:** I_yy ≤ τ_peak / α_req, where α_req is the required pitch slew acceleration.
2. **Disturbance rejection:** higher I_yy passively attenuates friction and cable torque disturbances, giving lower jitter above bandwidth. This argues *against* minimizing inertia blindly.
3. **Structural separation:** the first cradle structural mode f_s should satisfy f_s ≥ 3–5 × f_bw. With a collocated gyro, f_s ≥ 5 × f_bw is preferred.

This gives the target band:

$$
\frac{\tau_{dist,rms}}{\alpha_{jitter,allow}} \;\le\; I_{yy} \;\le\; \frac{\tau_{peak}}{\alpha_{req}}
$$

Choose I_yy within this band, then set the controller crossover:

$$
\omega_c \approx \sqrt{K_p / I_{yy}}
$$

---

## 3. Tuning Formulation (Design Phase)

**Design variables:** for each balance mass j (on N sliding or slotted mounts), choose its mass m_j and position **p_j**. The camera fore/aft slide position s_cam is also a variable.

**Optimization problem:**

$$
\min_{m_j,\mathbf{p}_j,s_{cam}} \; \sum_j m_j \quad (\text{minimize added mass})
$$

subject to:

$$
\begin{aligned}
&|x_{cg}|, |z_{cg}| \le e_{max} \\
&|y_{cg}| \le e_{y,max} \\
&|I_{xy}|, |I_{yz}| \le \epsilon_d I_{yy} \\
&I_{yy,min} \le I_{yy} \le I_{yy,max} \\
&|I_{xx} - I_{zz}| \le \Delta_{max} \\
&\mathbf{p}_j \in \text{allowable envelope (no FOV or clearance intrusion over full pitch range)}
\end{aligned}
$$

**Closed-form starting point** (point masses, two-plane balance):

1. Solve the static balance for the total corrective first moment:
   $$\sum_j m_j \mathbf{p}_j = -m_0 \mathbf{r}_{cg,0}$$
2. Split the correction across two axial planes, y = ±b, so the products of inertia cancel:
   $$\sum_j m_j x_j y_j = -I_{xy,0}, \qquad \sum_j m_j y_j z_j = -I_{yz,0}$$

   This is the classic two-plane balance. One plane cannot null both the force and the moment.
3. Update I_yy with the added masses:
   $$I_{yy} = I_{yy,0} + \sum_j m_j(x_j^2 + z_j^2)$$

   Check I_yy against the band. If it is too low, place balance masses farther out. If it is too high, use tungsten (ρ ≈ 19 g/cm³) closer in.

**Lens configurations:** treat each lens or zoom configuration as a separate load case. Either size adjustable masses to cover the worst case, or define a per-configuration balance table.

---

## 4. Physical Balancing Procedure (Integration Phase)

**Prerequisites:**
- Final flight/production payload installed, including the lens hood, filters, cable service loop and connectors.
- Pitch motor de-energized.
- Bearing friction characterized.

### Step 1: Coarse static balance, fore/aft (x)
- Set pitch θ = 0 (boresight horizontal).
- Adjust the camera slide or fore/aft masses until the cradle holds position when released.

### Step 2: Coarse static balance, vertical (z)
- Set θ = 90° (boresight vertical).
- Adjust the vertical masses or riser until the cradle holds.
- Repeat Steps 1–2 iteratively, since the two adjustments interact through the mount geometry.

### Step 3: Neutral-stability check
- Release the cradle at θ = −60°, −30°, 0°, +30°, +60°. It should stay put at each angle, within the bearing friction breakaway.
- If it always settles to one orientation, the residual offset is still large.

### Step 4: Fine static balance (powered method)
- Close the position loop and slow-sweep the pitch range at constant low rate, in both directions.
- Log motor current, convert to torque with Kt, and fit:
  $$\tau(\theta) = A\cos\theta + B\sin\theta + \tau_{fric}\,\text{sgn}(\dot\theta) + \tau_{cable}(\theta)$$
- Average the forward and reverse sweeps to remove Coulomb friction.
- Read the offsets from the fit:
  $$x_{cg} = A/(mg), \qquad z_{cg} = B/(mg)$$
- Correct with known masses and repeat until within e_max.

### Step 5: Dynamic balance
- Mount the gimbal on a rate table, or drive the outer axis with a sinusoid while holding pitch in closed loop.
- Measure the pitch-motor torque that is coherent with the outer-axis acceleration. The ratio gives I_xy (roll drive) or I_yz (yaw drive).
- Correct using paired masses at ±y planes: equal and opposite in x (or z) so static balance is preserved.
- Re-verify static balance.

### Step 6: Inertia identification
- With pitch in closed loop, inject a swept-sine or chirp torque.
- Fit the low-frequency rigid-body region of the plant FRF, |θ̇/τ| = 1/(I_yy ω), to obtain I_yy.
- Alternative method: bifilar or trifilar pendulum, or compound-pendulum period with a known offset mass.
- Identify the first structural resonance from the same FRF and confirm the separation ratio.

### Step 7: Controller retune
- Rescale the gains for the measured I_yy: K ∝ I_yy to keep the same crossover.
- Verify phase margin ≥ 45° and gain margin ≥ 6–10 dB.
- Confirm the notch filter, if any, is centered on the measured structural mode.

### Step 8: Lock and record
- Torque-stripe or loctite the masses.
- Record the final mass, positions, CG, I-tensor, and FRF in the Cycle 25 configuration log.

---

## 5. Acceptance Criteria

Fill in the Cycle 25 values:

| Parameter | Requirement | Measured |
|---|---|---|
| Gravity torque residual | ≤ `[ε_s·τ_cont]` N·m | |
| √(x_cg² + z_cg²) | ≤ `[e_max]` mm | |
| \|y_cg\| | ≤ `[ ]` mm | |
| \|I_xy\|/I_yy, \|I_yz\|/I_yy | ≤ `[1–2]` % | |
| I_yy | `[min]`–`[max]` kg·m² | |
| First structural mode | ≥ `[5×f_bw]` Hz | |
| Phase / gain margin | ≥ 45° / ≥ 6 dB | |
| Added balance mass | ≤ `[budget]` g | |

---

## 6. Common Pitfalls

- **Cable service loop:** often the largest unmodeled θ-dependent torque. Characterize it separately (cables-only sweep) so it isn't misread as CG offset.
- **Temperature:** thermal shifts in focus or zoom elements move the CG. Balance at the mid operating temperature and check the extremes.
- **Lens zoom/focus travel:** can shift the CG by millimetres. Use the worst-case or a configuration table.
- **Static-only balancing:** this leaves I_xy, which shows up as jitter only under vehicle motion and not on the bench.
- **Over-minimizing inertia:** this degrades passive jitter rejection. Target the band in §2.3, not the minimum.

If you share the Cycle 25 mass properties (CAD export of m, r_cg, I_C), the motor specs, and the bandwidth target, I can compute the specific balance mass sizes, positions, and the resulting inertia band.