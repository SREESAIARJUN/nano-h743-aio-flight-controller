# Pitch-Axis Camera Cradle: Balancing and Inertia Tensor Tuning Procedure (Cycle 3)

I don't have your project's Cycle 1–2 records, so this procedure is self-contained. It assumes Cycle 3 is a refinement iteration with the camera, lens and accessories in their final configuration. Replace the bracketed targets with your program's values.

---

## 1. Reference Frame and Definitions

**Cradle body frame {C}**, fixed to the rotating cradle:
- Origin on the pitch bearing axis, at the motor-side bearing face.
- **y**: pitch rotation axis.
- **x**: optical axis, at pitch angle θ = 0.
- **z**: completes the right-handed set (up at θ = 0).

**Rotating assembly** includes the cradle, camera, lens, accessories, motor rotor, encoder disc, the rotating half of the cabling, and the trim masses.

**Mass properties:**

$$
m,\qquad \mathbf r_c=\tfrac{1}{m}\!\int \mathbf r\,dm,\qquad
\mathbf I_O=\begin{bmatrix}I_{xx}&I_{xy}&I_{xz}\\ I_{xy}&I_{yy}&I_{yz}\\ I_{xz}&I_{yz}&I_{zz}\end{bmatrix},\quad I_{xy}=-\!\int xy\,dm \text{ (etc.)}
$$

---

## 2. Balance Objectives

### 2.1 Static balance (gravity torque about pitch)

With the base level, the gravity torque about the pitch axis is:

$$
\tau_g(\theta) = -mg\,(x_c\cos\theta + z_c\sin\theta)
$$

- **Requirement:** $x_c = z_c = 0$, so the centre of mass lies on the pitch axis.
- $y_c$ does not affect pitch torque. It does set the static moment seen by the roll and yaw axes, so it should match the outer-gimbal design value $y_c^{*}$, usually the roll-axis intersection.

### 2.2 Dynamic balance (pitch axis as a principal axis)

For rotation purely about y, $\boldsymbol\omega = (0,\omega,0)$. Euler's equation $\boldsymbol\tau = \mathbf I\dot{\boldsymbol\omega} + \boldsymbol\omega\times\mathbf I\boldsymbol\omega$ gives the off-axis reaction torques on the bearings and outer axes:

$$
\tau_x = I_{xy}\,\dot\omega + I_{yz}\,\omega^2,\qquad
\tau_z = I_{yz}\,\dot\omega - I_{xy}\,\omega^2
$$

- **Requirement:** $I_{xy} = I_{yz} = 0$. Otherwise every pitch manoeuvre disturbs roll and yaw, and the pitch loop couples into the outer loops.

### 2.3 Inertia shaping

**(a) Pitch plant inertia $I_{yy}$.** Set it to the control-design target:

$$
I_{yy}\in[I_{yy}^{*}(1-\epsilon),\;I_{yy}^{*}]
$$

- The lower bound gives disturbance rejection.
- The upper bound follows from torque and acceleration margin: $\tau_{\text{cont}} \ge I_{yy}\,\ddot\theta_{\max} + \tau_{\text{fric}} + \tau_{g,\text{res}}$.

**(b) Transverse isotropy.** The outer (roll/yaw) axes see the cradle's x–z inertia rotated by θ:

$$
I_{\text{outer}}(\theta) = \tfrac{I_{xx}+I_{zz}}{2} + \tfrac{I_{xx}-I_{zz}}{2}\cos 2\theta \mp I_{xz}\sin 2\theta
$$

- **Requirement:** $I_{xx}\approx I_{zz}$ and $I_{xz}\approx 0$.
- This makes the outer-axis plant independent of pitch angle, so no gain scheduling is needed.
- This is a secondary objective; trade it against the mass budget.

---

## 3. Tuning Model (Linear in Trim Masses)

Define N candidate trim hard-points with positions $\mathbf r_i = (x_i,y_i,z_i)$ and masses $m_i \ge 0$. Each trim mass contributes linearly:

$$
\Delta(m x_c) = \sum m_i x_i,\quad \Delta(m z_c)=\sum m_i z_i,\quad
\Delta I_{xy} = -\sum m_i x_i y_i,\quad \Delta I_{yz} = -\sum m_i y_i z_i,
$$
$$
\Delta I_{yy} = \sum m_i (x_i^2+z_i^2),\quad \Delta(I_{xx}-I_{zz}) = \sum m_i (z_i^2 - x_i^2),\quad \Delta I_{xz} = -\sum m_i x_i z_i
$$

Stacking these gives $\mathbf A\mathbf m = \mathbf b$, where $\mathbf b$ is the negative of the measured residuals and target errors.

**Optimisation:** solve the weighted non-negative least-squares problem

$$
\min_{\mathbf m\ge 0}\ \|\mathbf W(\mathbf A\mathbf m-\mathbf b)\|^2 + \lambda\,\mathbf 1^{\!T}\mathbf m
\quad\text{s.t.}\quad \textstyle\sum m_i \le m_{\text{budget}},\; m_i\in\{\text{available increments}\}
$$

- Weight priority: static > products of inertia > $I_{yy}$ > isotropy.
- Round the result to the available increments (washers or slugs) and re-check the residuals.

**Practical decoupling devices:**

| Device | Configuration | Effect |
|---|---|---|
| Static slide | Camera fore/aft on dovetail ($x_c$); riser/shim height ($z_c$) | Moves the CoM, coarse adjustment |
| Couple pair | Equal masses at $(a,b,0)$ and $(-a,-b,0)$ | Zero net first moment; $\Delta I_{xy} = -2mab$, corrects $I_{xy}$ only |
| Couple pair | Equal masses at $(0,b,c)$ and $(0,-b,-c)$ | Corrects $I_{yz}$ only |
| Symmetric pair | Masses at $\pm z$, or at $\pm x$ | Shapes $I_{xx}-I_{zz}$ and raises $I_{yy}$ without moving the CoM |

---

## 4. Procedure

### Step 0: Preparation
1. Fit the final Cycle 3 payload: camera, lens, filter, follow-focus, cables routed and strain-relieved in their flight positions.
2. Lock or brake the roll and yaw axes, and level the base to < 0.1°.
3. Load the CAD mass model and update it with the weighed component masses. This gives the predicted $\mathbf r_c$ and $\mathbf I_O$, which serve as the initial $\mathbf b$.

### Step 1: Coarse static balance (motor de-energised)
1. Slide the camera in x until the cradle holds at θ = 0 without drift.
2. Rotate to θ = 90° and adjust z (riser or shims) until it holds there as well.
3. Confirm neutral equilibrium at θ = ±45° and ±135°.

### Step 2: Fine static identification (motor energised)
1. Hold the cradle at quasi-static angles $\theta_k$, for example every 15° over the full travel, using a slow sweep in both directions to cancel Coulomb friction.
2. Log holding torque: $\tau_k = K_t i_k$.
3. Fit $\tau(\theta) = A\cos\theta + B\sin\theta + \tau_0$. Then $x_c = -A/mg$ and $z_c = -B/mg$.
4. Interpret the remainder:
   - $\tau_0$ (offset) indicates cable spring or bias.
   - Higher harmonics indicate cable stiffness. Fix routing rather than trimming it out.

### Step 3: Dynamic identification
1. Apply a pitch chirp or multisine excitation, for example 0.5–30 Hz, below the first structural mode.
2. Fit the pitch plant model $\tau = I_{yy}\ddot\theta + b\dot\theta + \tau_c\,\text{sgn}\dot\theta + \tau_g(\theta)$ to obtain $I_{yy}$.
3. Simultaneously log the roll and yaw reaction torques, via holding-motor currents or a 6-axis load cell under the pitch yoke.
4. Regress against $\dot\omega$ and $\omega^2$ using the equations in §2.2 to extract $I_{xy}$ and $I_{yz}$:
   - The $\dot\omega$ terms dominate at low speed; use both terms for consistency.
5. **Optional:** identify $I_{xx}$, $I_{zz}$ and $I_{xz}$ with a bifilar or trifilar pendulum, or by exciting the roll axis at θ = 0° and θ = 90°, then fitting $I_{\text{outer}}(\theta)$.

### Step 4: Solve and install trim
1. Form $\mathbf b$ from the measured residuals and solve §3.
2. Install the masses using threadlocker and witness marks.
3. Record the part numbers and positions.

### Step 5: Verify
1. Repeat Steps 2–3.
2. Iterate Steps 4–5, typically 2–3 loops, until the acceptance criteria are met.

### Step 6: Robustness checks
1. Repeat the static fit at the temperature extremes and after a vibration or shock exposure, to check for slip.
2. Check sensitivity to configuration changes: measure $\partial\mathbf r_c$ for lens focus and zoom extremes, and for battery and accessory variants.
3. If the sensitivity is significant, define a per-configuration trim table or add a motorised balance slide.

---

## 5. Acceptance Criteria

Values in brackets are typical starting points; set them per program.

| Parameter | Criterion |
|---|---|
| Residual gravity torque $mg\sqrt{x_c^2+z_c^2}$ | ≤ [5%] of continuous motor torque; CoM offset ≲ [0.2 mm] |
| $\lvert I_{xy}\rvert,\ \lvert I_{yz}\rvert$ | ≤ [2%] of $I_{yy}$ |
| Peak cross-axis torque at $\ddot\theta_{\max}$ and $\omega_{\max}$ | ≤ [10%] of outer-axis disturbance budget |
| $I_{yy}$ | Within [−10%, 0%] of $I_{yy}^{*}$ |
| $\lvert I_{xx}-I_{zz}\rvert / \bar I$, $\lvert I_{xz}\rvert/\bar I$ | ≤ [10%] |
| Trim mass added | ≤ $m_{\text{budget}}$ |
| First structural mode, cradle and payload | ≥ [3–5×] the pitch loop bandwidth (re-check after trim) |

---

## 6. Cycle 3 Deliverables

1. **Measured values:** final $m$, $\mathbf r_c$ and $\mathbf I_O$ with uncertainties, compared against the CAD prediction. Feed the delta back into the CAD model for Cycle 4.
2. **Trim log:** positions, masses and the residual after each iteration.
3. **Configuration sensitivity table:** effect of lens, focus and accessory variants.
4. **Updated pitch plant parameters** ($I_{yy}$, b, $\tau_c$) for controller retuning.
5. **Outstanding issues**, such as cable-stiffness harmonics and configuration-dependent imbalance, with proposed fixes.

If you share the Cycle 1–2 results, the payload mass and geometry, or your motor torque figures, I can turn the bracketed targets into concrete numbers and set up the trim-mass solver for your hard-point layout.