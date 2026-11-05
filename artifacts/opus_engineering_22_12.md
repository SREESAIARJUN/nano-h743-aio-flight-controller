# Pitch-Axis Camera Cradle: Balancing and Inertia Tuning Procedure (Cycle 22)

> **Note:** I don't have your program's Cycle 22 baseline (CAD mass properties, motor spec, previous-cycle residuals). This procedure is therefore a complete, parameterized template. Replace the bracketed values `[ ]` with your numbers. If you share the Cycle 21 results and the hardware specs, I can tailor the tolerances and correction masses.

---

## 1. Frames and Definitions

**Pitch frame {P}:**
- Origin on the pitch bearing axis, at the midpoint between the bearings.
- **y** lies along the pitch axis.
- **x** points along the optical axis at θ = 0.
- **z** completes a right-handed set.

**Cradle payload properties** (camera, lens, cradle, cabling, and anything else that rotates with pitch), expressed in {P}:

- Mass: M
- Center of mass (CoM): **r**_c = (x_c, y_c, z_c)
- Inertia tensor about the {P} origin:

$$
\mathbf{J}=\begin{bmatrix} J_{xx} & J_{xy} & J_{xz}\\ J_{xy} & J_{yy} & J_{yz}\\ J_{xz} & J_{yz} & J_{zz}\end{bmatrix},
\qquad J_{xy}=-\!\int xy\,dm,\ \text{etc.}
$$

### Balance Objectives for the Pitch Axis

| Condition | Requirement | Physical meaning |
|---|---|---|
| Static balance | x_c = z_c = 0 | No gravity torque at any pitch angle |
| Dynamic balance | J_xy = J_yz = 0 | Pitch motion produces no roll/yaw reaction torque. Pitch is then a principal axis. |
| y_c | Not critical for pitch | Affects roll/yaw balance. Coordinate with the outer-axis budget. |
| J_yy | Tuned to a target value, not minimized | Sets plant gain, bandwidth, and disturbance rejection |

---

## 2. Governing Equations

**Gravity torque from static imbalance** (pitch angle θ, gravity along −z of the base):

$$
\tau_g(\theta) = M g\,(x_c\cos\theta - z_c\sin\theta)
$$

**Cross-axis reaction torques** for pure pitch motion (ω = (0, ω, 0), α = ω̇). From **τ** = **J**α + ω × (**J**ω):

$$
\tau_x = J_{xy}\,\alpha + J_{yz}\,\omega^2,\qquad
\tau_z = J_{yz}\,\alpha - J_{xy}\,\omega^2
$$

These torques feed directly into the roll and yaw loops as disturbances. This is the reason dynamic balance matters as much as static balance.

**Pitch plant:**

$$
G(s)=\frac{1}{J_{yy}s^2 + b s},\qquad
\tau_{\text{motor,req}} \ge J_{yy}\alpha_{\max} + \tau_{\text{fric}} + \tau_{\text{cable}} + M g\,|r_{c,\perp}|
$$

---

## 3. Procedure

### Step 0 – Configuration Freeze
1. Lock the payload configuration for Cycle 22: lens, zoom/focus position, filters, sunshade, cable routing, and connector strain relief.
2. Record mass and part numbers.
3. Zoom and focus groups shift the CoM. Balance at the **[nominal / mid-zoom]** position and record the shift at both extremes (Step 6).

### Step 1 – Predict from CAD
1. Export M, **r**_c, and **J** from CAD in frame {P}.
2. Add non-modeled items (cables, tape, fasteners) as point masses using measured weights.
3. Compute the predicted correction (Section 4). This is your starting trim.

### Step 2 – Coarse Static Balance (Unpowered)
1. Mount the cradle with the motor de-energized and decoupled (or with cogging characterized).
2. **x-offset:** At θ = 0, slide the payload along x on the dovetail or rail until the cradle stays neutral.
3. **z-offset:** Rotate to θ = 90°, then adjust z (vertical shim or riser) until the cradle stays neutral.
4. **Check:** Release the cradle at 0°, ±45°, and ±90°. It should not drift.
   - Drift that depends on angle indicates a z_c error.
   - Constant-direction drift indicates an x_c error.

### Step 3 – Fine Static Balance (Powered, Holding-Torque Method)
1. Command a slow position sweep from −θ_max to +θ_max in steps of **[5°]**, dwelling for **[2 s]** at each angle.
2. Log the motor torque (current × K_t) at each dwell. Average the forward and reverse sweeps to cancel Coulomb friction.
3. Fit the data by least squares:
   $$\tau(\theta)=a\cos\theta + c\sin\theta + d$$
   - x_c = a / (Mg)
   - z_c = −c / (Mg)
   - d is the cable/spring bias. Track it, because it is not a balance error.
4. Correct, then repeat until residuals meet the Section 5 criteria.

### Step 4 – Inertia Measurement
Use two independent methods and require agreement within **[5%]**.

**(a) Trifilar or bifilar pendulum (offline, cradle removed):**
$$
J_{yy}=\frac{M g R^2 T^2}{4\pi^2 L}
$$
where R is the suspension radius, L is the wire length, and T is the period. Subtract the fixture inertia.

**(b) In-situ system identification:**
1. Inject a chirp or PRBS torque from **[0.5–200 Hz]** in closed loop at low amplitude.
2. Fit the measured frequency response to 1/(J_yy s² + b s).
3. The mid-band slope gives J_yy and the low-frequency corner gives b.
4. Note the first structural mode (f₁). Your control bandwidth must stay below **[f₁/3 to f₁/5]**.

**Products of inertia J_xy and J_yz:**
1. Run sinusoidal pitch motion at amplitude A and frequency ω₀.
2. Measure the reaction on the roll and yaw axes. Use motor current with those axes locked, or a base 6-axis load cell.
3. The α-correlated component (in phase with −A ω₀² sin ω₀t) gives the products of inertia. The ω²-component appears at 2ω₀ and serves as a cross-check.

### Step 5 – Dynamic Balance (Two-Plane Correction)
1. Choose two correction planes on the cradle at y = y₁ and y = y₂, as far apart as possible.
2. Place trim mass m_k at (x_k, z_k) in each plane. Let u_k = m_k x_k and w_k = m_k z_k.
3. Null static and dynamic imbalance together by solving:

$$
\begin{bmatrix}1&1\\ y_1&y_2\end{bmatrix}\!\begin{bmatrix}u_1\\u_2\end{bmatrix}=\begin{bmatrix}-Mx_c\\ J_{xy}\end{bmatrix},
\qquad
\begin{bmatrix}1&1\\ y_1&y_2\end{bmatrix}\!\begin{bmatrix}w_1\\w_2\end{bmatrix}=\begin{bmatrix}-Mz_c\\ J_{yz}\end{bmatrix}
$$

(The sign follows from J_xy = −∫xy dm: added masses must contribute +J_xy to Σm x y.)

4. Convert to hardware for each plane k:
   - Mass: m_k = √(u_k² + w_k²) / r_k, at the available mounting radius r_k
   - Clocking angle: atan2(w_k, u_k)
5. Use discrete trim weights in **[1 g / 5 g]** increments, and re-solve after rounding.

### Step 6 – J_yy Tuning
1. **Lower bound:** Choose J_yy high enough to reject torque disturbances (friction ripple, cable, wind). Pointing jitter scales roughly as τ_dist / (J_yy ω_bw²).
2. **Upper bound:** Ensure J_yy α_max + margins ≤ **[70%]** of motor continuous torque.
3. **Adjustment:** Add symmetric ballast pairs at ±x or ±z, at equal radius, in both correction planes. Paired symmetric masses change J_yy without disturbing static or dynamic balance.
   $$\Delta J_{yy} = \sum m_i (x_i^2+z_i^2)$$
4. **Inertia ratio:** Keep the pitch inertia ratio to the reflected outer-axis inertia within **[target]** to limit kinematic coupling.
5. **Retune the controller:** Update the loop gains, which scale with J_yy, and re-verify the phase margin (≥ **[45°]**) and gain margin (≥ **[6 dB]**).

### Step 7 – Configuration Sensitivity
1. Repeat Step 3 at the zoom and focus extremes and with any optional accessories installed.
2. Record Δx_c and Δz_c.
3. If the variation exceeds tolerance, choose one of:
   - Balance at the centroid of the operating range.
   - Add active compensation: a gravity feedforward term, τ_ff = Mg(x̂_c cos θ − ẑ_c sin θ), scheduled on zoom.

### Step 8 – Verification and Lock
1. Run the full Step 3 sweep and the Step 4b identification again.
2. Run the pointing-stability test under the operational disturbance profile.
3. Torque-stripe or thread-lock all trim hardware, then photograph and record positions.
4. Update the CAD mass model so Cycle 23 starts from measured truth.

---

## 4. Acceptance Criteria

Example values are shown. Set yours from the jitter and torque budgets.

| Parameter | Criterion |
|---|---|
| Residual static torque, M g \|r_c⊥\| | ≤ **[2–5%]** of motor continuous torque, and ≤ **[x] mN·m** |
| \|x_c\|, \|z_c\| | ≤ **[0.1–0.3 mm]** |
| \|J_xy\|, \|J_yz\| | ≤ **[1–2%]** of J_yy |
| J_yy | Target **[value]** ± **[5%]**. Pendulum and system ID agree within 5%. |
| Fit residual (Step 3) | RMS ≤ **[10%]** of the cos/sin amplitude. Larger residuals indicate cable stiffness or cogging, not imbalance. |
| Structural mode f₁ | ≥ **[3–5×]** control bandwidth |
| Torque headroom | J_yy α_max + disturbances ≤ **[70%]** of continuous torque |

---

## 5. Cycle 22 Record Sheet

| Item | CAD prediction | Measured, pre-trim | Correction applied | Measured, post-trim | Pass/Fail |
|---|---|---|---|---|---|
| M | | | | | |
| x_c, z_c | | | | | |
| J_xy, J_yz | | | | | |
| J_yy | | | | | |
| b, cable bias d | | | | | |
| f₁ | | | | | |
| Δr_c over zoom range | | | | | |
| Trim masses (plane, mass, radius, angle) | | | | | |

---

## 6. Common Pitfalls

- **Cable torque mistaken for imbalance.** Cable torque is usually angle-dependent but not sinusoidal. Check the Step 3 fit residuals, and route cables near the axis.
- **Coulomb friction biasing holding torque.** Always average bidirectional sweeps.
- **Balancing with the wrong configuration.** Lens caps, sunshades, and connector backshells add mass that must be included or removed consistently.
- **Single-plane correction.** It fixes static balance but can worsen the products of inertia. Use two planes.
- **Not retuning after a J_yy change.** Loop gain shifts in direct proportion to J_yy.

If you share the payload mass, CAD inertia tensor, motor K_t and continuous torque, target bandwidth, and the Cycle 21 residuals, I can compute the specific trim masses, J_yy target, and tolerances for Cycle 22.