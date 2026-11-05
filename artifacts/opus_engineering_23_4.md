# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning Procedure

**Scope note:** I don't have project-specific data for "Cycle 23", such as payload mass, cradle geometry, trim hardware, or prior-cycle residuals. This procedure is therefore written generically, and the quantities you need to fill in are marked **[ ]**. If you share the previous cycle's results or the CAD mass properties, I can turn this into numeric targets.

---

## 1. Objectives

For a cradle rotating about pitch axis **x̂**:

1. **Static balance.** Place the center of mass (CoM) on the pitch axis, so that y_c = z_c = 0. This eliminates the gravity torque τ_g(θ) that the pitch motor would otherwise hold continuously.
2. **Dynamic balance.** Drive the products of inertia I_xy and I_xz to zero. This makes **x̂** a principal axis, so pitch acceleration and rate produce no reaction torques on the roll and yaw axes.
3. **Inertia tuning.** Set I_xx to the design target **[ ] kg·m²**. This gives the required control bandwidth, keeps the motor torque margin, and places structural modes at least 3–5× above the loop crossover.
4. **Outer-axis coupling.** Control the axial CoM offset x_c, which loads the roll and yaw axes. It does not matter for pitch alone.

---

## 2. Coordinate Frame and Definitions

- **Origin:** the intersection of the pitch axis with the cradle's mid-plane between the bearings.
- **Axes:** x̂ along the pitch axis, ẑ along the optical axis at θ = 0, and ŷ completing a right-handed frame.
- **Inertia tensor about the origin:**

$$
\mathbf{I} = \begin{bmatrix} I_{xx} & I_{xy} & I_{xz}\\ I_{xy} & I_{yy} & I_{yz}\\ I_{xz} & I_{yz} & I_{zz}\end{bmatrix},\quad I_{xy} = -\int xy\,dm \;\;(\text{tensor convention; state it explicitly in the log})
$$

- **Gravity torque about pitch** at pitch angle θ, for total mass M:

$$
\tau_g(\theta) = M g\,(z_c \sin\theta + y_c\cos\theta) \quad(\text{sign per frame orientation})
$$

- **Cross-axis reaction torques** for pitch-only motion (ω = ω x̂):

$$
\boldsymbol{\tau} = \mathbf{I}\dot{\boldsymbol\omega} + \boldsymbol\omega\times\mathbf{I}\boldsymbol\omega
\;\Rightarrow\;
\tau_y = I_{xy}\dot\omega - I_{xz}\omega^2,\quad \tau_z = I_{xz}\dot\omega + I_{xy}\omega^2
$$

This is why I_xy and I_xz must be minimized.

---

## 3. Pre-Balance Preparation

1. **Flight configuration.** Install the payload exactly as it will fly: camera body, lens at its operational focus/zoom, filters, cables with strain relief dressed and tied, and connectors. Zoom and focus shift the CoM, so balance at the worst-case or nominal position and record which one you used.
2. **Disconnect the pitch drive.** Remove motor cogging and drag, or use an encoder-only mode, or power the motor off with the magnets decoupled if possible.
3. **Measure breakaway friction.** Record the bearing friction torque τ_f **[ ] N·m**. It sets the resolution limit of the static test.
4. **Record the baseline.** Log total mass M, the CAD-predicted CoM, and the CAD-predicted inertia tensor for comparison.

---

## 4. Static Balance Procedure

### 4.1 Coarse balance (manual)

1. Place the cradle at θ = 0 (lens horizontal) and release it. If it rotates, adjust the **fore/aft slide (z)** until it stays put.
2. Rotate to θ = 90° (lens pointing down) and release. If it rotates, adjust the **vertical / riser trim (y)**.
3. Iterate until the cradle holds at 0°, ±45°, and ±90°.

### 4.2 Fine balance (quantitative, motor-current method)

1. Close the position loop and step through θ = −90° to +90° in 15° increments. At each step, hold for **[ ] s** and average the motor q-axis current.
2. Convert current to torque using τ = K_t·i, then average the forward and reverse sweeps to cancel friction.
3. Least-squares fit the model

$$
\tau(\theta) = A\sin\theta + B\cos\theta + C
$$

4. Recover the offsets:
   - z_c = A/(Mg)
   - y_c = B/(Mg)
   - C is a cable or spring bias. Investigate it if |C| > **[ ]**.

**Acceptance criterion:** the peak residual torque Mg·√(y_c² + z_c²) must be ≤ **[e.g., 5–10 % of continuous motor torque]**, equivalent to |r_c,⊥| ≤ **[ ] mm**.

---

## 5. Inertia Tensor Measurement

Use at least two methods so they cross-check each other.

| Quantity | Method | Notes |
|---|---|---|
| I_xx | Torsion pendulum or bifilar suspension about the pitch axis: I = kT²/(4π²) − I_fixture | Calibrate k with a reference body |
| I_xx (in situ) | Chirp or swept-sine excitation from the pitch motor; fit τ = I_xx·θ̈ + b·θ̇ + τ_c | Also yields damping and friction |
| Full tensor | Measure the moment of inertia about ≥ 6 non-coplanar axes n̂ᵢ in a trifilar or torsion fixture; I_n = n̂ᵀ**I**n̂; solve the 6-unknown linear system by least squares | Choose axes such as x, y, z, and the three face diagonals for good conditioning |
| I_xy, I_xz (dynamic) | Spin or oscillate about pitch while measuring reaction torques with a load cell or a powered-off roll/yaw motor; fit to the §2 equations | Most sensitive to the quantities you actually care about |

**Parallel-axis correction.** Fixture measurements are taken about the CoM. Transfer them to the pitch-axis origin with

$$
\mathbf{I}_O = \mathbf{I}_{cm} + M\left(|\mathbf r_c|^2\mathbf E - \mathbf r_c\mathbf r_c^{T}\right)
$$

---

## 6. Trim-Mass Tuning (Optimization Formulation)

**Setup.** Define a set of discrete trim stations k = 1…N at fixed positions **r**_k = (x_k, y_k, z_k), with masses m_k ≥ 0 and an upper limit m_k ≤ m_max.

Adding a point mass changes the system as follows:

$$
\Delta(M\mathbf r_c) = m_k\mathbf r_k,\qquad \Delta\mathbf I = m_k\left(|\mathbf r_k|^2\mathbf E - \mathbf r_k\mathbf r_k^{T}\right)
$$

All of these effects are **linear in m_k**. You can therefore solve for the trim masses directly:

$$
\min_{\mathbf m\ge 0}\;\; \left\| \mathbf W\left(\mathbf A\,\mathbf m - \mathbf b\right)\right\|^2 + \lambda\sum_k m_k
$$

The rows of **A** and **b** are:

| Row | Expression | Target |
|---|---|---|
| Static y | Σ m_k y_k | = −M y_c |
| Static z | Σ m_k z_k | = −M z_c |
| Product I_xy | −Σ m_k x_k y_k | = −I_xy,0 |
| Product I_xz | −Σ m_k x_k z_k | = −I_xz,0 |
| Axial CoM (optional) | Σ m_k x_k | = −M x_c |
| Inertia target (soft) | Σ m_k (y_k² + z_k²) | = I_xx,target − I_xx,0 |

Solve with NNLS, or with LP/MILP if the trim masses come in discrete washer sizes. The λ term penalizes total added mass.

**Design guidance:**

- **Pairs for products of inertia.** Correcting I_xy and I_xz without disturbing static balance requires mass **pairs** at ±x, for example +m at (x, y) and −m (mass removed) at (−x, y). This creates a pure couple.
- **Lowering I_xx.** If I_xx exceeds the target, trim masses cannot help. You must remove mass far from the axis, for example by moving the battery or electronics closer to it, or by using lightweight lens supports.
- **Keep trims inside the structure.** Place trim stations within the cradle envelope so they do not reduce the stiffness margin.

---

## 7. Iteration Loop (Cycle Structure)

For each cycle, including Cycle 23:

1. Measure the static offsets (§4.2) and the tensor (§5). Compare against the previous cycle and against CAD.
2. Solve the trim problem (§6) and install the trims. Torque the fasteners to spec and apply threadlocker.
3. Re-measure, then compute residuals and the predicted versus achieved change.
4. Update the model. If the achieved change is less than about 80 % of the predicted change, recheck the station coordinates and the sign conventions.
5. Verify closed-loop performance:
   - holding current at ±90°
   - step response and overshoot
   - cross-axis disturbance on roll/yaw during a **[ ] rad/s²** pitch slew
   - no new resonance within 3× bandwidth
6. Stop when all acceptance criteria pass. Otherwise, proceed to the next cycle.

---

## 8. Acceptance Criteria (to Be Filled In)

| Parameter | Limit |
|---|---|
| √(y_c² + z_c²) | ≤ [ ] mm |
| \|x_c\| | ≤ [ ] mm |
| \|I_xy\|, \|I_xz\| | ≤ [ ]% of I_xx (typ. 1–2 %) |
| I_xx | target ± [ ]% |
| Holding current over ±90° | ≤ [ ] A |
| Cross-axis reaction at max slew | ≤ [ ] N·m |
| Added trim mass (cumulative) | ≤ [ ] g |

---

## 9. Cycle 23 Record Template

- Configuration: payload, lens, focus/zoom state, cable routing revision
- Measured before trimming: M, **r**_c, **I**_O, friction τ_f
- Trim solution: station, mass, and position for each trim
- Measured after trimming: the same quantities
- Residuals versus criteria: PASS/FAIL per row
- Closed-loop verification data
- Anomalies, and changes to carry into Cycle 24

If you provide the Cycle 22 residuals, your trim station layout, and the motor K_t and torque limits, I can compute the specific trim masses and the predicted Cycle 23 residuals.