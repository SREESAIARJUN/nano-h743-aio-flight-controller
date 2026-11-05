# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning (Cycle 2)

I don't have your Cycle 1 records, so this procedure assumes the following. Replace them with your own baseline:

- **Configuration:** 3-axis gimbal with yaw → roll → pitch order, so pitch is the innermost axis.
- **Hardware:** direct-drive BLDC motor with encoder feedback.
- **Cycle 1 status:** static balance was coarse and done by hand. Cycle 2 refines it to quantified static + dynamic balance and shapes the inertia tensor for control bandwidth.

---

## 1. Frames and Definitions

**Pitch frame {P}**
- Origin O on the pitch motor axis, at the midpoint between the bearings.
- x_P lies along the pitch rotation axis.
- z_P points along the optical axis at zero pitch.
- y_P completes the right-handed set.

**Payload assembly**
Cradle + camera + lens + accessories (filters, follow-focus, cables), with total mass m.

**CG offset vector:** r_G = [x_G, y_G, z_G]ᵀ in {P}.

**Inertia tensor about O:**

$$
\mathbf{I}_O=\begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz}\\ -I_{xy} & I_{yy} & -I_{yz}\\ -I_{xz} & -I_{yz} & I_{zz}\end{bmatrix},\qquad
\mathbf{I}_O=\mathbf{I}_G+m\left[(\mathbf{r}_G^\top\mathbf{r}_G)\mathbf{E}-\mathbf{r}_G\mathbf{r}_G^\top\right]
$$

**Role of each term about the pitch axis (x):**
- **I_xx** is the plant inertia the pitch controller sees.
- **I_xy, I_xz** are the products of inertia that couple pitch acceleration into roll and yaw reaction torques. These are the dynamic imbalance.
- **y_G, z_G** produce gravity torque about pitch (static imbalance).
- **x_G** does not create pitch torque, but it loads the roll axis. Fix it here because it is cheapest to fix at the cradle.

---

## 2. Balance Objectives

| Quantity | Physical effect | Cycle 2 target (adjust to your motor) |
|---|---|---|
| √(y_G² + z_G²) | Gravity torque τ_g = m·g·r_⊥·cos(θ+φ) | τ_g,max ≤ 2–3 % of motor continuous torque; typically r_⊥ ≤ 0.3 mm |
| x_G | Roll-axis static load | ≤ 0.5 mm, or within the roll motor budget |
| I_xy, I_xz | Pitch→roll/yaw cross-coupling torque I_xz·θ̈ | ≤ 5 % of I_xx |
| I_xx | Sets the pitch plant pole and the torque needed for a given slew | Within the band derived in §5 |
| First structural mode of cradle | Must sit above the control loop | f_n ≥ 4–5 × the pitch loop crossover frequency |

---

## 3. Static Balancing Procedure

### 3.1 Pre-conditions
1. Lock the final payload configuration: lens, focus position, filters, battery, and cable dress. Zoom and focus breathing shift z_G, so balance at the mid-focus position and log the full range.
2. Disable or de-energize the motor. If the motor has cogging, measure and subtract the cogging torque map from Cycle 1. Otherwise use a cogging-free test spindle.
3. Level the base. Use a calibrated inclinometer on the cradle reference datum.

### 3.2 Coarse balance (position adjustments)
1. **Fore/aft (z_G):** set the pitch axis horizontal with the lens pointing level. Slide the camera plate along z until the cradle holds at any angle with no drift.
2. **Vertical (y_G):** tilt the lens to point straight up (θ = 90°). Adjust the cradle arm height until the cradle holds at 90° and at 45°.
3. Iterate steps 1–2. They interact when the axes are not orthogonal.

### 3.3 Fine balance (quantitative, via motor current)
Use the motor as a torque sensor.

1. Energize the motor in position-hold mode with low gains.
2. Step through θ = 0° to 360° in 15° increments.
3. At each step, record the steady quadrature current i_q(θ) averaged over 2 s.
4. Fit the model:

$$
\tau(\theta)=K_t\,i_q(\theta)= \underbrace{m g\,r_\perp}_{A}\cos(\theta+\varphi)+\tau_{cog}(\theta)+\tau_{fric}\,\text{sgn}(\dot\theta)
$$

5. Run sweeps in both directions to separate Coulomb friction. Least-squares fitting gives A and φ:
   - **Correction distance:** r_⊥ = A / (m·g)
   - **Direction:** given by φ
   - **Correction:** move the payload by −r_⊥ along φ.
6. If the slide resolution is insufficient, add trim mass instead. The mass m_t placed at radius r_t opposite the imbalance is:

$$
m_t\,r_t = m\,r_\perp \quad\Rightarrow\quad m_t=\frac{m\,r_\perp}{r_t}
$$

   Keep r_t large to minimize added mass. The trim mass also adds m_t·r_t² to I_xx, so account for it in §5.
7. Repeat until the static target in §2 is met.

### 3.4 x_G (axial) balance
1. Rotate the roll axis to 90° so the pitch axis is vertical.
2. Measure the roll motor holding current.
3. Shift the cradle laterally along the pitch shaft clamp until roll holding current is at a minimum.
4. Re-verify §3.3. Lateral shifts usually don't disturb it, but check.

---

## 4. Dynamic Balance (Products of Inertia)

Static balance does not remove I_xy or I_xz. Asymmetric lenses, side-mounted monitors, and offset batteries create them.

### 4.1 Measurement
**Option A: Spin test**
1. Rotate the pitch axis at constant ω (e.g., 2–3 rev/s on a test rig, if the cradle allows full rotation).
2. Measure the bearing reaction forces with load cells, or read the roll/yaw motor holding currents.
3. Interpret the result: a rotating couple M = ω²·[I_xz, −I_xy]ᵀ (in rotating coordinates) appears as a 1× sinusoid.

**Option B: Step-acceleration test (gimbal-in-situ)**
1. Command pitch acceleration steps θ̈.
2. Record the roll and yaw motor currents.
3. Compute the products of inertia (with the outer axes held stiff):

$$
\tau_{roll}\approx -I_{xy}\,\ddot\theta,\qquad \tau_{yaw}\approx -I_{xz}\,\ddot\theta
$$

**Option C: CAD + verification**
1. Compute the full I_O from CAD with measured component masses.
2. Validate I_xx using the pendulum test in §5.2.

### 4.2 Correction
Use two trim planes separated along the pitch axis (left and right cradle arms). Place masses m₁ and m₂ at positions (x₁, ρ₁, ψ₁) and (x₂, ρ₂, ψ₂). Solve:

$$
\text{Static: } \sum m_i \rho_i e^{j\psi_i} = -m\,(y_G + j z_G)
$$

$$
\text{Couple: } \sum m_i x_i \rho_i e^{j\psi_i} = -(I_{xy} + j I_{xz})
$$

This gives two complex equations in two complex unknowns (m_iρ_i e^{jψ_i}). Solve directly, then choose ρ_i to fit the geometry.

---

## 5. Inertia Tensor Tuning (I_xx Shaping)

### 5.1 Target band for I_xx
1. **Upper bound (torque/slew):** for the required peak acceleration α_max (from the slew profile):

$$
I_{xx}\le \frac{\tau_{peak}-\tau_{fric}-\tau_{g,res}}{\alpha_{max}}
$$

2. **Lower bound (disturbance rejection):** larger I_xx passively filters base vibration and torque ripple. For a ripple torque τ_r at frequency ω_r, the angular jitter is:

$$
\theta_{jit}\approx \frac{\tau_r}{I_{xx}\,\omega_r^2}\ \le\ \theta_{spec}
\ \Rightarrow\ I_{xx}\ge \frac{\tau_r}{\theta_{spec}\,\omega_r^2}
$$

3. **Inertia ratio:** keep J_load/J_rotor within your controller's tested range. Direct drive tolerates high ratios, but compliance-induced resonance gets worse as the ratio grows.

**Choosing the target:** take I_xx in the lower third of the band. This preserves torque headroom while meeting the jitter limit.

### 5.2 Measurement of I_xx (validation)
**Compound pendulum method**
1. Add a known offset mass m_p at distance d to create a pendulum.
2. Measure the small-oscillation period T with the motor unpowered.
3. Compute:

$$
I_{xx}=\frac{(m_p d)\,g\,T^2}{4\pi^2}-m_p d^2
$$

**Motor sine-sweep / chirp method**
1. Apply a torque chirp with a balanced payload.
2. Fit the measured response G(s) = 1/(I_xx·s² + b·s).
3. This also yields the damping b and exposes structural resonances.

Cross-check both methods. They should agree within 5 %.

### 5.3 Tuning levers
- **To reduce I_xx:** move mass radially inward. For example, relocate the battery or monitor onto the roll frame, or shorten cradle arms.
- **To increase I_xx:** add symmetric mass pairs at equal radius on opposite sides. This adds inertia with zero static or couple imbalance.
- **After any change:** re-run §3.3 and §4.

---

## 6. Structural Check
1. Perform a tap test or chirp to find the first cradle bending/torsion mode f_n.
2. If f_n is below the target in §2:
   - Stiffen the cradle arms. Adding a gusset or closing a section beats adding thickness.
   - Do not simply add mass, which lowers f_n.

---

## 7. Cycle 2 Acceptance Record

| Item | Cycle 1 | Cycle 2 measured | Target | Pass |
|---|---|---|---|---|
| r_⊥ (mm) | | | ≤ 0.3 | |
| x_G (mm) | | | ≤ 0.5 | |
| τ_g,max (% cont.) | | | ≤ 3 | |
| \|I_xy\|/I_xx | | | ≤ 0.05 | |
| \|I_xz\|/I_xx | | | ≤ 0.05 | |
| I_xx (kg·m²) | | | band §5.1 | |
| f_n (Hz) | | | ≥ 4–5 × f_c | |
| Balance drift across focus/zoom range | | | log | |

**Deliverables:**
- Updated I_O tensor
- Trim mass map (masses, positions, part numbers)
- Revised plant model (I_xx, b) handed to controls so loop gains can be retuned
- Cycle 3 notes: residuals and any payload-configuration sensitivity

---

If you share your actual payload mass, motor torque constants, Cycle 1 results, and control bandwidth target, I can turn the targets above into specific numbers and compute the trim masses.