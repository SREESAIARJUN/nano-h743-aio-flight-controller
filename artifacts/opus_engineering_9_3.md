# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning

I don't have your program's specific documentation, so I don't know what Cycle 9 adds or changes, such as a new payload, lens, or cradle revision. Below is a general procedure for a pitch cradle in a 3-axis gimbal (yaw → roll → pitch, with pitch as the innermost axis). Substitute your actual masses, tolerances, and acceptance limits where indicated.

---

## 1. Coordinate Frame and Definitions

Define the **cradle frame {P}**:
- **Origin O** on the pitch bearing axis, at the midpoint between bearings.
- **y_P** along the pitch rotation axis.
- **x_P** along the optical axis (boresight) at zero pitch.
- **z_P** completes a right-handed set (up at zero pitch).

The rotating assembly includes the cradle, camera body, lens, filters, cables within the service loop, the motor rotor, the encoder disk, and any counterweights.

**Inertia tensor about O in {P}:**

$$
\mathbf{I}_O =
\begin{bmatrix}
I_{xx} & -I_{xy} & -I_{xz}\\
-I_{xy} & I_{yy} & -I_{yz}\\
-I_{xz} & -I_{yz} & I_{zz}
\end{bmatrix}
$$

**Balance objectives:**

| Objective | Condition | Physical consequence if violated |
|---|---|---|
| Static balance | CoM on the pitch axis: r_x = r_z = 0 | Gravity torque varies with pitch angle, so the motor holds a steady load |
| Dynamic balance | I_xy ≈ 0, I_yz ≈ 0 | Pitch acceleration couples reaction torques into the roll and yaw axes |
| Axial CoM | r_y ≈ 0 (centered between bearings) | Unequal bearing loads, plus a roll-axis imbalance moment |
| Inertia tuning | I_yy within target band | Sets control bandwidth, motor sizing, and disturbance rejection |

---

## 2. Required Inputs

1. A mass properties report from CAD for all rotating parts, with materials verified.
2. Measured masses of the actual camera, lens, and accessories. Do not trust datasheet values.
3. The payload configuration matrix for this cycle, including all lens/zoom/focus states. Zoom lenses shift CoM along x as they extend.
4. Motor parameters: torque constant K_t, rotor inertia J_m, gear ratio N if geared (direct-drive is typical), cogging torque, and continuous current limit.
5. The control requirement: the target pitch bandwidth ω_bw or the required disturbance rejection.

---

## 3. Static Balancing Procedure

### 3.1 Analytical pre-balance
Compute the composite CoM:

$$
\mathbf{r}_{cm} = \frac{\sum m_i \mathbf{r}_i}{\sum m_i}
$$

Solve for the counterweight mass and position needed to null r_x and r_z:

$$
m_c\,x_c = -\sum m_i x_i,\qquad m_c\,z_c = -\sum m_i z_i
$$

Prefer **adjustment by translating the payload** on the cradle's sliding plate (fore/aft and up/down) over adding mass. Translation balances without increasing inertia. Use counterweights only for residuals that exceed the slide's travel.

### 3.2 Physical static balance (motors unpowered or disabled)
1. Lock the roll and yaw axes level. Release pitch.
2. **Fore/aft (x) balance.** Set pitch to 0° (boresight horizontal). Adjust the payload slide until the cradle holds position without drift.
3. **Vertical (z) balance.** Rotate pitch to ±90° (pointing up and down). Adjust the vertical offset until the cradle holds at both orientations.
4. **Iterate.** The x and z adjustments interact through mechanical tolerances. Repeat steps 2–3 until the cradle remains stationary when released at 0°, ±45°, and ±90°.
5. **Axial (y) balance.** Check with the roll axis released. The cradle should not induce roll. Shift the payload laterally if needed.

### 3.3 Quantitative verification with motor current
Bearing friction masks small imbalances, so verify electrically. Enable the pitch loop in position-hold mode. Sweep the pitch angle θ slowly from −90° to +90°, record the steady-state current i(θ), and fit:

$$
K_t\, i(\theta) = \tau_f\,\text{sgn}(\dot\theta) + m g\,(r_x\cos\theta - r_z\sin\theta)
$$

Run the sweep in both directions to separate friction τ_f from gravity. The fitted r_x and r_z are the residual offsets.

**Acceptance criterion (example):** the peak residual gravity torque m g √(r_x² + r_z²) should be at most about 5–10% of the motor's continuous torque, and below the cogging amplitude where possible.

---

## 4. Dynamic Balancing (Products of Inertia)

### 4.1 Why it matters
Angular momentum about O is **H** = **I**_O **ω**. For pure pitch rate ω_y with acceleration α_y, the cross-axis reaction torques are:

$$
\tau_x = -I_{xy}\alpha_y + I_{yz}\,\omega_y^2,\qquad
\tau_z = -I_{yz}\alpha_y - I_{xy}\,\omega_y^2
$$

(Signs depend on convention.) These torques appear as roll and yaw disturbances during aggressive pitch slews, and as jitter at line-of-sight.

### 4.2 Procedure
1. **Compute the products of inertia from CAD.** The main contributors are usually asymmetric camera bodies (side-mounted grips, displays), offset lens motors, and connectors.
2. **Correct with paired masses.** A pair of masses at (x, ±y, z) cancels I_xy without shifting the CoM:

$$
\Delta I_{xy} = -\sum m_k x_k y_k
$$

Place equal masses at diagonally opposite positions, (+x, +y) and (−x, −y), so that Σm x = 0 and Σm y = 0 but Σm xy ≠ 0. Apply the same approach to I_yz.

3. **Verify experimentally.** Command a pitch acceleration step or chirp with the roll and yaw axes held by their controllers. Measure the roll and yaw motor current responses, then estimate I_xy and I_yz:

$$
\hat I_{xy} \approx -\frac{K_{t,roll}\,\Delta i_{roll}}{\alpha_y}
$$

**Acceptance criterion (example):** |I_xy| and |I_yz| ≤ 1–2% of I_yy, or the residual cross-axis LOS jitter is within the error budget.

---

## 5. Pitch Inertia Tuning (I_yy)

### 5.1 Target selection
The total reflected inertia is:

$$
J_{tot} = I_{yy} + J_m N^2
$$

Considerations:

- **Torque/acceleration capability.** τ_max ≥ J_tot α_req + τ_f + τ_residual,gravity.
- **Bandwidth.** For a PD/PID loop, ω_bw ≈ √(K_p / J_tot). Lower inertia raises achievable bandwidth for the same gain, but structural modes limit how far you can push it.
- **Disturbance rejection.** Higher inertia passively filters high-frequency base disturbances, since θ_err ∝ τ_d / (J s²). For stabilization-dominated gimbals, moderate inertia is beneficial.
- **Structural mode separation.** Keep the first cradle/payload resonance f_n at least 3–5× the loop bandwidth. Adding mass far from the axis lowers f_n.
- **Payload-variation robustness.** Across all payload configurations, J_tot should vary less than the controller's gain margin allows. Typically a variation of ±30% or less is tolerable without gain scheduling.

### 5.2 Tuning levers (in order of preference)
1. **Payload position along the slide.** This shifts both the CoM and I_yy, via the parallel axis theorem I_yy = I_cm,yy + m(x² + z²). Balancing to the axis already minimizes I_yy.
2. **Cradle structure redesign.** Use material removal and pocketing to reduce I_yy, or deliberate mass at small radius to add inertia without hurting f_n.
3. **Inertia trim ring or flywheel on the pitch shaft.** Use this when a minimum J is required for disturbance filtering. A trim ring adds I_yy with zero CoM or product-of-inertia penalty, as long as it is axisymmetric.

### 5.3 Measuring I_yy
- **Torsional/compound pendulum.** Add a known offset mass m_t at radius r_t, then measure the oscillation period T:

$$
J_{tot} + m_t r_t^2 = \frac{m_t g r_t\,T^2}{4\pi^2}
$$

- **System identification.** Apply a chirp or PRBS torque input, fit the plant G(s) = 1 / (J s² + b s), and extract J. This also reveals the structural modes.
- **Cross-check against CAD.** Treat a discrepancy greater than 10% as a flag for missing parts (cables, adhesives, fasteners) or for errors in material assignment.

---

## 6. Cycle Procedure Summary

| Step | Action | Output | Pass criterion (example; substitute your spec) |
|---|---|---|---|
| 1 | Update the CAD mass model with weighed components | Predicted r_cm, **I**_O | Model complete, all parts weighed |
| 2 | Analytic pre-balance: set slide positions and counterweights | Balance configuration | Predicted |r_cm| < 0.2 mm |
| 3 | Physical static balance at 0°, ±45°, ±90° | Slide settings recorded | No drift when released |
| 4 | Current-sweep verification | Fitted r_x, r_z, τ_f | Residual gravity torque < 5% of continuous torque |
| 5 | Dynamic balance (products of inertia) | Î_xy, Î_yz | Each < 2% of I_yy |
| 6 | Measure I_yy | J_tot measured | Within ±10% of the target band |
| 7 | System identification of structural modes | f_n | f_n > 4× ω_bw |
| 8 | Repeat steps 3–6 for each payload configuration | Configuration table | All configurations meet criteria |
| 9 | Retune or confirm controller gains | Updated gain set or schedule | Phase margin > 45°, gain margin > 6 dB |
| 10 | Document as-built mass properties and balance settings | Cycle report | Signed off |

---

## 7. Common Pitfalls

- **Cable service loops** produce angle-dependent spring torque that is easily mistaken for imbalance. Characterize it separately by repeating the current sweep with the cables disconnected, if possible.
- **Zoom/focus state changes** shift the CoM. Balance at the most-used state, or add an automated CoM compensation (a motorized slide or a feedforward gravity term).
- **Bearing preload changes** after rebalancing alter friction. Re-run the friction fit.
- **Thermal drift** in lens mass distribution matters for some long lenses. Check balance at operating temperature if your environment is wide-range.

If you share the Cycle 9 specifics (payload masses, motor model, bandwidth target, and what changed from Cycle 8), I can turn this into concrete numbers: counterweight sizes, slide positions, and the predicted I_yy and acceptance values.