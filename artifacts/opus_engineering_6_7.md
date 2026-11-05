# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning (Cycle 6)

**Note on context:** I don't have your project's Cycle 5 results, CAD data, or requirement IDs. The procedure below is a complete general formulation. Placeholders like `[TBD]` mark where your program-specific values go. If you share the prior cycle's residuals or the cradle mass properties, I can tailor the numbers.

---

## 1. Objectives

| # | Objective | Physical meaning |
|---|-----------|------------------|
| O1 | **Static balance** | Payload center of mass (CoM) lies on the pitch axis. Gravity torque ≈ 0 at all pitch angles. |
| O2 | **Dynamic balance** | Products of inertia coupling pitch to the other axes ≈ 0. Pitch is a principal axis. |
| O3 | **Inertia tuning** | Pitch inertia J_yy is set to the target band required by the control loop bandwidth and motor torque budget. |
| O4 | **Robustness** | Balance holds across the payload configuration envelope (lens/zoom, filters, cables). |

---

## 2. Coordinate Frame and Definitions

Pitch frame **P**:
- Origin O on the pitch bearing axis, at the midplane between the bearings.
- **y** along the pitch rotation axis.
- **x** along the optical (boresight) axis at zero pitch.
- **z** completes the right-handed set.

**Inertia tensor about O:**

$$
\mathbf{I}_O=\begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz}\\ -I_{xy} & I_{yy} & -I_{yz}\\ -I_{xz} & -I_{yz} & I_{zz}\end{bmatrix},\qquad I_{ij}=\int \rho\, x_i x_j\, dV \;(i\neq j)
$$

**Composite assembly** of n bodies, where each body k has mass m_k, CoM r_k, and centroidal tensor I_k (rotated into P by R_k):

$$
\mathbf{I}_O=\sum_k\Big[\mathbf{R}_k\mathbf{I}_k\mathbf{R}_k^T + m_k\big(\|\mathbf{r}_k\|^2\mathbf{E}-\mathbf{r}_k\mathbf{r}_k^T\big)\Big]
$$

**Net CoM offset:**

$$
\mathbf{r}_c=\frac{\sum m_k\mathbf{r}_k}{M}
$$

---

## 3. Governing Equations

### 3.1 Gravity disturbance torque about pitch

With pitch angle θ, and only the x and z offsets mattering:

$$
\tau_g(\theta)=M g\,(x_c\cos\theta - z_c\sin\theta)\;\Rightarrow\;|\tau_g|_{max}=Mg\sqrt{x_c^2+z_c^2}
$$

The y offset (y_c) does not produce pitch torque. It does load the bearings and the roll/yaw axes, so trim it too.

### 3.2 Inertial coupling

Euler's equation about the pitch axis, for a cradle with base angular rate ω:

$$
\tau_y = I_{yy}\dot\omega_y - I_{xy}(\dot\omega_x - \omega_y\omega_z) - I_{yz}(\dot\omega_z+\omega_x\omega_y) + (I_{xx}-I_{zz})\omega_x\omega_z + I_{xz}(\omega_x^2-\omega_z^2)
$$

Implications:
- Driving I_xy and I_yz to zero removes coupling of roll/yaw acceleration into pitch, and vice versa.
- The term (I_xx − I_zz) is a geometric property that is only partly tunable.

### 3.3 Linear base-acceleration coupling

A base acceleration **a** produces pitch torque M(**r**_c × **a**)_y. Static balance therefore also rejects vibration and maneuver loads, not just gravity.

---

## 4. Acceptance Criteria (fill from requirements)

| Parameter | Criterion | Typical starting value |
|-----------|-----------|------------------------|
| √(x_c² + z_c²) | ≤ τ_allow / (M g) | ≤ 0.05–0.2 mm, or ≤ 5% of continuous motor torque |
| \|y_c\| | ≤ `[TBD]` | ≤ 0.5 mm |
| \|I_xy\|, \|I_yz\| | ≤ ε · I_yy | ε = 0.5–2% |
| I_yy | within [J_min, J_max] | from loop bandwidth / torque budget |
| Config spread Δr_c over envelope | ≤ `[TBD]` | ≤ 2× the nominal tolerance |

**Deriving the inertia band:**
- **Upper bound (torque budget):** J_max ≈ τ_motor,peak / α_req − J_rotor.
- **Lower bound (stability):** J_min is set by structural/control margins. For example, it keeps the inertia ratio to motor rotor ≥ 3–5. It may also be set by required angular momentum to reject friction and cable stiffness: J ≥ K_cable / ω_bw².

---

## 5. Balancing Procedure

### Step 0 — Configuration freeze
- Define the configuration matrix C_j: lens/focal position, filters, cable dressing, and any accessories.
- Choose the **nominal balance configuration**, i.e. the centroid of the envelope rather than an extreme.

### Step 1 — Analytical baseline (CAD/mass properties)
1. Build a mass-property table for all parts, with *measured* masses replacing CAD estimates wherever available.
2. Compute **r**_c and **I**_O for each C_j.
3. Record the Cycle 5 residuals and their deltas to the model. Use these as a model-correction bias:

$$
\mathbf{r}_{c,\text{pred}}^{(6)}=\mathbf{r}_{c,\text{model}}+\big(\mathbf{r}_{c,\text{meas}}-\mathbf{r}_{c,\text{model}}\big)^{(5)}
$$

### Step 2 — Trim mass synthesis (optimization)

Decision variables:
- Trim masses m_i at candidate hard-points p_i (discrete set, e.g. threaded bosses).
- Continuous slide positions s along adjustment rails.

Formulation:

$$
\min_{m_i,\,s}\; w_1\|\mathbf{r}_c\|_{xz}^2+w_2 y_c^2+w_3\!\left(\frac{I_{xy}}{I_{yy}}\right)^{\!2}+w_4\!\left(\frac{I_{yz}}{I_{yy}}\right)^{\!2}+w_5\!\left(\frac{I_{yy}-J^*}{J^*}\right)^{\!2}+w_6\sum m_i
$$

Subject to:
- Σm_i ≤ m_trim,max
- 0 ≤ m_i ≤ m_i,max
- Envelope/clearance constraints at all pitch angles
- Every C_j meeting the static tolerance (min-max robust form)

Key observations:
- **Static balance** is linear in m_i, since Σm_i p_i = −M_0 **r**_c0. It needs at least 2 independent trims in the x–z plane.
- **Each product of inertia** needs trims with independent lever arms:
  - Nulling I_xy without disturbing x_c requires a **couple**: two masses at ±y with opposite x offsets.
  - I_yz is handled the same way, with ±y and opposite z.
- **I_yy tuning** is achieved with symmetric mass pairs placed at radius ρ from the axis. Because they are symmetric, they leave the CoM and products unchanged:

$$
\Delta I_{yy}=2m\rho^2
$$

Recommended trim architecture (minimum 6 effective DOF):

| Trim | Location | Corrects |
|------|----------|----------|
| T1 | Slide along x (fore/aft rail) | x_c |
| T2 | Slide along z (vertical rail) | z_c |
| T3a/T3b | Couple pair at ±y, ±x | I_xy |
| T4a/T4b | Couple pair at ±y, ±z | I_yz |
| T5 | Symmetric radial pair | I_yy (coarse) |
| — | Lateral shim | y_c |

### Step 3 — Physical static balance
1. Unpower the motor. Remove or decouple cogging, and use a low-friction bearing or a jig with air bearings if available.
2. **Rough balance (neutral equilibrium test):**
   - Rotate to θ = 0°, ±45°, ±90° and release.
   - Adjust T1 until the cradle holds at θ = 0° vs 90°, then T2 until it holds at ±45°.
3. **Fine balance (motor current method):**
   - Under slow-rate closed-loop control, sweep θ over the full range in both directions.
   - Record the torque from current, τ = K_t·i.
   - Fit:

$$
\tau(\theta)=\tau_f\,\text{sgn}(\dot\theta)+Mg(x_c\cos\theta - z_c\sin\theta)+\tau_{cable}(\theta)
$$

   - Averaging the two sweep directions removes friction. Fitting the sinusoid gives x_c and z_c directly.
   - Characterize τ_cable separately with the payload removed, or include it as a polynomial term.
4. Compute trim corrections:

$$
\Delta s_x = -\frac{M x_c}{m_{T1}},\qquad \Delta s_z = -\frac{M z_c}{m_{T2}}
$$

5. Iterate until within tolerance, usually 2–3 passes.

### Step 4 — Inertia tensor identification

**Option A — Swept-sine / chirp excitation (on-gimbal, preferred):**
- Excite pitch and fit G(s) = 1/(J s² + b s + k) to identify I_yy, damping, and cable stiffness.
- Excite roll/yaw with pitch locked and measure the pitch reaction torque. This yields I_xy and I_yz from the cross terms in §3.2. With ω ≈ 0: τ_y ≈ −I_xy ω̇_x − I_yz ω̇_z.

**Option B — Trifilar/bifilar pendulum or inertia measurement machine (off-gimbal):**
- Measure about at least 6 independent axis orientations.
- Reconstruct the full tensor by least squares, using the moment about unit axis **n**:

$$
I_n=\mathbf{n}^T\mathbf{I}\mathbf{n}
$$

**Option C — Spin balancer (if the cradle permits rotation about pitch at speed):**
- Measure bearing reaction forces at both bearings.
- The static unbalance gives the in-phase common-mode component. The couple unbalance gives the antiphase component, which is proportional to the products I_xy and I_yz.
- Correct in two planes per the ISO 21940 two-plane method.

### Step 5 — Dynamic (couple) correction
- Apply couple trims T3 and T4:

$$
m_c\,(\Delta x)(2\,y_p) = I_{xy},\qquad m_c\,(\Delta z)(2\,y_p) = I_{yz}
$$

- Re-verify static balance, since couples are CoM-neutral only if placed exactly symmetrically.

### Step 6 — I_yy tuning
- Add or remove the T5 pair to reach J*.
- If J_yy > J_max with no removable mass, flag it for a structural redesign (lightening) in Cycle 7.

### Step 7 — Envelope verification
- Repeat the Step 3 current sweep for each C_j.
- Confirm max over j of |τ_g| ≤ τ_allow.
- If zoom/focus shifts the CoM beyond tolerance, choose between:
  - (a) Motorized active balance (lead-screw trim mass driven from the lens position).
  - (b) Feedforward gravity compensation, τ_ff = Mg(x̂_c(z_lens) cosθ − …), using a lookup table.

### Step 8 — Lock and record
- Apply threadlocker or witness marks to the trims, and record the final positions.
- Document the final **r**_c, the full tensor **I**_O, uncertainties, and residual-vs-model deltas. These feed the Cycle 7 model correction.

---

## 6. Uncertainty Budget (template)

| Source | Effect on r_c | Effect on I | Mitigation |
|--------|---------------|-------------|------------|
| Current sensor / K_t error | ±% of τ_g | — | Calibrate K_t on a dyno |
| Bearing friction asymmetry | bias in x_c | — | Bidirectional sweep averaging |
| Cable torque | bias, θ-dependent | apparent stiffness | Separate characterization |
| Trim position resolution | ±m·δs/M | ±2mρδρ | Fine-pitch screws |
| Pendulum period error | — | ∝ 2δT/T | ≥ 50 cycle averaging |
| Payload variation | Δr_c over C_j | ΔI | Robust optimization (Step 2) |

Combine the sources by root-sum-square and confirm the result is less than or equal to 1/3 of the tolerance (3:1 test-uncertainty ratio).

---

## 7. Cycle 6 Deliverables
1. Updated mass-property model with the Cycle 5 correction bias applied.
2. Trim optimization results (mass, location, predicted residuals per configuration).
3. Measured static balance data: τ(θ) fits and residual x_c, z_c, y_c.
4. Identified inertia tensor with uncertainty, and its eigen-decomposition (principal-axis misalignment angle from pitch: ψ ≈ ½·atan2(2I_xy, I_xx − I_yy)).
5. Pass/fail matrix against §4.
6. Open issues and recommendations for Cycle 7: redesign if trim mass exceeds ~5–8% of payload, or if the principal-axis misalignment exceeds `[TBD]`°.

If you provide the payload mass, the Cycle 5 residuals, the motor torque constant, and the target bandwidth, I can compute the actual trim masses, the J band, and the tolerances.