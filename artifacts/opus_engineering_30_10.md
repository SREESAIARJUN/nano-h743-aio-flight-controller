# Pitch-Axis Camera Cradle: Balancing and Inertia Tensor Tuning Procedure (Cycle 30)

I don't have records from earlier cycles. This is a complete, self-contained procedure. Where it calls for Cycle 29 baseline values, substitute your own logged data. Tolerances are typical starting points for a brushless direct-drive gimbal and should be adjusted to your motor and controller specifications.

---

## 1. Coordinate Frame and Definitions

Define the cradle body frame **C** with its origin on the pitch bearing axis, at the midpoint between the two bearings:

- **y**: pitch rotation axis (bearing axis)
- **x**: optical axis (forward) at θ = 0
- **z**: completes the right-handed frame (up at θ = 0)

The payload is the cradle, camera, lens, accessories, and cables inside the pitch joint. It has:

- mass M
- center of gravity **r**_c = (x_c, y_c, z_c)
- inertia tensor about the origin:

$$
\mathbf{I}_C=\begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz}\\ -I_{xy} & I_{yy} & -I_{yz}\\ -I_{xz} & -I_{yz} & I_{zz}\end{bmatrix}
$$

**Balancing goals:**

| Goal | Condition | Why it matters |
|---|---|---|
| Static balance | x_c = z_c = 0 | No gravity torque on the pitch motor at any angle |
| Lateral centering | y_c ≈ 0 | Equal bearing loads; reduces roll-axis moment |
| Dynamic balance | I_xy = I_yz = 0 | Pitch motion produces no reaction torques on roll/yaw |
| Inertia tuning | I_yy within target band | Sets plant gain and achievable control bandwidth |

---

## 2. Governing Equations

**Gravity torque about pitch** (g in the inertial frame, θ = pitch angle):

$$
\tau_g(\theta) = M g\,(x_c\cos\theta + z_c\sin\theta)
$$

**Reaction torques from pitch motion** (ω = θ̇ about y). Angular momentum is **H** = **I**_C ω ŷ = ω(−I_xy, I_yy, −I_yz). The bearings must supply:

$$
\boldsymbol{\tau} = \dot{\mathbf{H}} =
\begin{bmatrix} -I_{xy}\dot\omega + I_{yz}\,\omega^2 \\ I_{yy}\dot\omega \\ -I_{yz}\dot\omega - I_{xy}\,\omega^2 \end{bmatrix}
$$

The x and z components are disturbances injected into the roll and yaw loops. Dynamic balance drives them to zero.

**Pitch plant** (rigid-body approximation):

$$
G(s)=\frac{\theta(s)}{\tau(s)}=\frac{1}{(I_{yy}+I_{rotor})\,s^2 + b\,s}
$$

---

## 3. Procedure

### Phase A: Configuration Freeze
1. Lock the payload build: camera body, lens (and focus/zoom position), filters, follow-focus, cables, and battery.
2. Dress and tie down cables exactly as they will be in operation. Cable stiffness and drape are the most common causes of balance drift between cycles.
3. Record the configuration ID, payload mass M, and the lens state.

### Phase B: Coarse Static Balance (motors unpowered or free-wheeling)
4. **Fore/aft (x_c).** With the optical axis horizontal (θ = 0), slide the camera plate along x until the cradle holds level with no drift.
5. **Vertical (z_c).** Rotate to θ = 90° (lens up) and release. Adjust the vertical riser until the cradle holds at any angle.
6. **Iterate steps 4 and 5.** The adjustments interact if the plate slide is not perfectly orthogonal to the riser. Stop when the cradle stays put at 0°, ±45°, and ±90°.
7. **Lateral (y_c).** Center the payload laterally using a reference or a bearing-load check. Then verify that the roll axis stays balanced, since y_c shifts the roll-axis CG.

### Phase C: Fine Static Balance (powered, current-based)
8. Command a slow quasi-static pitch sweep from −90° to +90° at no more than 5°/s. Log the motor torque, τ = K_t·i_q.
9. Fit the logged torque to:

$$
\tau(\theta)= a\cos\theta + b\sin\theta + c + \tau_f\,\mathrm{sgn}(\dot\theta)
$$

   Sweep in both directions so that friction (τ_f) separates from the gravity terms.
10. Extract the residual offsets: x_c = a/(Mg) and z_c = b/(Mg). The constant term c indicates cable spring torque or encoder/commutation offset.
11. Trim mechanically (Phase E) until the gravity amplitude meets spec:

$$
\sqrt{a^2+b^2} \le 0.05\,\tau_{cont}
$$

   A typical spec is ≤ 5% of the motor's continuous torque, with a stretch goal of ≤ 2%.

### Phase D: Dynamic Balance Measurement
12. Measure the products of inertia using either method:
    - **Analytic/CAD:** Export **I**_C from a CAD model with measured component masses. Use this as the initial estimate.
    - **Experimental (preferred for verification):** Drive pitch with a sinusoid, θ = Θ sin(Ωt), at 2–5 Hz with the roll and yaw motors holding position. Record the roll and yaw motor currents. The component at Ω gives I_xy and I_yz through the −I·ω̇ terms. The component at 2Ω gives the ω² terms and serves as a cross-check.
13. Run the test at two amplitudes to confirm linearity and rule out mount compliance.

### Phase E: Two-Plane Trim Mass Solution
Place trim masses in two planes along the pitch axis, at y = y₁ and y = y₂ (for example, the left and right cradle side plates). For each plane i, the unknowns are the products u_i = m_i x_i and w_i = m_i z_i.

The four conditions are:

$$
\begin{aligned}
u_1 + u_2 &= -M x_c &&\text{(static, x)}\\
w_1 + w_2 &= -M z_c &&\text{(static, z)}\\
u_1 y_1 + u_2 y_2 &= -I_{xy}^{res} &&\text{(dynamic, xy)}\\
w_1 y_1 + w_2 y_2 &= -I_{yz}^{res} &&\text{(dynamic, yz)}
\end{aligned}
$$

These decouple into two 2×2 systems. With L = y₂ − y₁:

$$
u_2=\frac{-I_{xy}^{res}+M x_c\,y_1}{L},\quad u_1=-Mx_c-u_2
$$

The w terms follow identically, using z_c and I_yz.

14. For each plane, choose the trim radius ρ_i and solve for mass and angle:
    - m_i = √(u_i² + w_i²)/ρ_i
    - φ_i = atan2(w_i, u_i)
15. Choose the largest ρ the cradle envelope permits. This minimizes added mass, but note the inertia penalty in Phase F.
16. Install the masses using tapped tungsten slugs or a threaded-rod rider. Torque-seal all fasteners.

### Phase F: Inertia Tensor Tuning
17. **Update I_yy.** Each trim mass adds approximately m_i ρ_i² (point mass) to I_yy. Recompute the total.
18. **Check against the target band.** For a target closed-loop bandwidth ω_bw with the controller tuned for phase margin PM:
    - I_yy,max: set by the motor's peak torque and the required angular acceleration, α_max (I_yy ≤ τ_peak/α_max, with ≥ 30% margin).
    - I_yy,min: set by the inertia ratio to the rotor (keep I_yy/I_rotor ≲ 10–20 for a stiff loop with a compliant coupling), or by sensor noise amplification at the chosen gain.
    - Structural mode: the first cradle/plate resonance should satisfy f_n ≥ 5·f_bw. If added mass lowers f_n, use stiffer, closer trim locations.
19. **If I_yy is too high:** move the trim inward, use denser trim material (tungsten instead of brass) to shorten the arm, or relocate payload accessories toward the axis.
20. **If I_yy is too low** (rare, usually due to noise or limit cycling): add symmetric inertia rings that are balanced and centered, so they leave x_c, z_c, I_xy, and I_yz unchanged.
21. **Check the cross-axis inertias.** I_xx and I_zz of the cradle are part of the load for the roll and yaw axes. Pass the updated values to those loops and re-verify their balance. Moving pitch-axis mass shifts the roll and yaw CGs.

### Phase G: Experimental Inertia Verification
22. Inject a chirp torque (0.5–50 Hz) on the pitch axis and fit the frequency response to G(s). The high-frequency asymptote gives I_yy + I_rotor, and the low-frequency phase gives b.
23. As an optional cross-check, measure the unpowered cradle on a trifilar pendulum:

$$
I = \frac{M g R^2 T^2}{4\pi^2 L}
$$

### Phase H: Controller Retune and Acceptance
24. Rescale the pitch loop gains by the ratio I_yy,new / I_yy,old to preserve bandwidth. Re-check the notch filters against the measured structural mode.
25. Run the acceptance tests in Section 4 and log everything.

---

## 4. Acceptance Criteria (Typical)

| Parameter | Limit |
|---|---|
| Gravity torque amplitude √(a²+b²) | ≤ 5% τ_cont (goal ≤ 2%) |
| Residual CG offset √(x_c²+z_c²) | ≤ 0.5 mm (scale with M) |
| Constant torque offset c (cable spring) | ≤ 1% τ_cont |
| Cross-coupling: roll/yaw torque per unit pitch accel | ≤ 3% of I_yy·ω̇ |
| I_yy deviation from target | within ±10% |
| Structural first mode | ≥ 5× pitch bandwidth |
| Idle pitch motor current at ±90° | ≤ 1.2× current at 0° |
| Repeatability after payload remove/reinstall | gravity amplitude change ≤ 1% τ_cont |

---

## 5. Cycle 30 Record Template

```
Cycle: 30        Date:        Config ID:        Lens state:
M [kg]:          I_rotor [kg·m²]:
Pre-trim:   x_c=   z_c=   y_c=   Ixy=   Iyz=   Iyy=
Trim plane 1 (y1=  ): m=   ρ=   φ=
Trim plane 2 (y2=  ): m=   ρ=   φ=
Post-trim:  x_c=   z_c=   Ixy=   Iyz=   Iyy=   f_n=
Fit coeffs: a=   b=   c=   τ_f=
Gains rescaled by:       Bandwidth achieved:
Δ vs Cycle 29:
Pass/Fail:       Notes (cable dress, anomalies):
```

---

## 6. Common Failure Modes

- **Cable drag** shows up as a nonzero c term or as hysteresis that friction does not explain. Re-dress cables before trimming any mass.
- **Lens focus/zoom breathing** shifts x_c. Balance at the most-used lens state, or characterize the range.
- **Trim mass creep** comes from unsecured fasteners. Use thread-locker and witness marks.
- **Correcting static balance only** can leave large products of inertia that make the roll and yaw axes jitter during fast tilts. Always complete Phase D.
- **Ignoring the inertia penalty** is a problem because large trim masses at large radius can push I_yy out of band.

If you share the Cycle 29 baseline data (mass, measured offsets, motor specs, target bandwidth), I can work through the trim-mass solution and inertia check numerically.