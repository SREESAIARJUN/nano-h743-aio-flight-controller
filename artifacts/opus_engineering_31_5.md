# 10-State EKF for Payload/Handle Gyro Fusion in Gimbal Vibration Isolation

I don't have the details of your "Cycle 31" design iteration. The formulation below is a self-contained baseline. Adjust the state choices or noise models to match your earlier cycles.

---

## 1. Problem Setup

**Frames**
- **{W}** world (NED or ENU; gravity is **g** = [0, 0, g]ᵀ)
- **{H}** handle frame, with the handle IMU rigidly mounted
- **{P}** payload/camera frame, with the payload IMU rigidly mounted

**Sensors**

| Sensor | Signal | Rate (typ.) |
|---|---|---|
| Payload gyro | ω̃ₚ = ωₚ + bₚ + ηₚ | 1–8 kHz |
| Payload accel | ãₚ ≈ Cᵀ(q)·g + aₗᵢₙ + ηₐ | 1 kHz |
| Handle gyro | ω̃ₕ = ωₕ + ηₕ (bias pre-calibrated) | 1–8 kHz |
| Joint encoders | θ, θ̇ (3 axes) | ≥ 1 kHz |

**Purpose of the fusion**
- The payload gyro measures the controlled quantity directly, but it is biased.
- The handle gyro observes the disturbance before it propagates through the motors and structure.
- Mapped through the gimbal kinematics, the handle gyro gives an unbiased (but noisier) second observation of ωₚ. This makes bₚ observable without waiting for slow accelerometer leveling.

---

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \boldsymbol{\omega}_p \\ \mathbf{b}_p \end{bmatrix}
\in \mathbb{R}^{10},
\qquad
\begin{aligned}
\mathbf{q} &= [q_0, q_1, q_2, q_3]^T && \text{attitude of P w.r.t. W (4)}\\
\boldsymbol{\omega}_p &\in \mathbb{R}^3 && \text{true payload angular rate in P (3)}\\
\mathbf{b}_p &\in \mathbb{R}^3 && \text{payload gyro bias (3)}
\end{aligned}
$$

ωₚ is modeled as an explicit state, rather than being consumed as a control input. This lets the filter produce a fused, low-latency rate estimate ω̂ₚ for the stabilization loop, which is the vibration isolation feedback signal.

---

## 3. Process Model

### 3.1 Continuous-time dynamics

$$
\dot{\mathbf{q}} = \tfrac{1}{2}\,\boldsymbol{\Omega}(\boldsymbol{\omega}_p)\,\mathbf{q}
$$

$$
\dot{\boldsymbol{\omega}}_p = -\frac{1}{\tau_\omega}\boldsymbol{\omega}_p + \mathbf{w}_\omega
\quad\text{(first-order Gauss–Markov: band-limited vibration)}
$$

$$
\dot{\mathbf{b}}_p = \mathbf{w}_b \quad\text{(random walk)}
$$

where

$$
\boldsymbol{\Omega}(\boldsymbol{\omega}) =
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z\\
\omega_x & 0 & \omega_z & -\omega_y\\
\omega_y & -\omega_z & 0 & \omega_x\\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

### 3.2 Discrete-time propagation (step Δt)

$$
\hat{\mathbf{q}}_{k+1}^- = \left[\cos\!\left(\tfrac{\|\hat{\boldsymbol\omega}_k\|\Delta t}{2}\right)\mathbf{I}_4 + \frac{\sin\!\left(\tfrac{\|\hat{\boldsymbol\omega}_k\|\Delta t}{2}\right)}{\|\hat{\boldsymbol\omega}_k\|}\,\boldsymbol\Omega(\hat{\boldsymbol\omega}_k)\right]\hat{\mathbf{q}}_k
$$

$$
\hat{\boldsymbol\omega}_{p,k+1}^- = \alpha\,\hat{\boldsymbol\omega}_{p,k},\qquad \alpha = e^{-\Delta t/\tau_\omega}
$$

$$
\hat{\mathbf{b}}_{p,k+1}^- = \hat{\mathbf{b}}_{p,k}
$$

After every predict and update step, renormalize the quaternion: q̂ ← q̂ / ‖q̂‖.

### 3.3 State-transition Jacobian

$$
\mathbf{F}_k =
\begin{bmatrix}
\mathbf{I}_4 + \tfrac{\Delta t}{2}\boldsymbol\Omega(\hat{\boldsymbol\omega}) & \tfrac{\Delta t}{2}\,\boldsymbol\Xi(\hat{\mathbf q}) & \mathbf{0}_{4\times3}\\
\mathbf{0}_{3\times4} & \alpha\,\mathbf{I}_3 & \mathbf{0}_3\\
\mathbf{0}_{3\times4} & \mathbf{0}_3 & \mathbf{I}_3
\end{bmatrix}
$$

$$
\boldsymbol\Xi(\mathbf q)=
\begin{bmatrix}
-q_1 & -q_2 & -q_3\\
q_0 & -q_3 & q_2\\
q_3 & q_0 & -q_1\\
-q_2 & q_1 & q_0
\end{bmatrix}
$$

### 3.4 Process noise

$$
\mathbf{Q}_k = \mathrm{blkdiag}\!\left(
\sigma_q^2\Delta t\,\mathbf I_4,\;
\sigma_\omega^2(1-\alpha^2)\,\mathbf I_3,\;
\sigma_b^2\Delta t\,\mathbf I_3\right)
$$

- **σ_q** is small and covers integration error only.
- **σ_ω** is the RMS vibration rate. Set it from a PSD of logged handle data.
- **σ_b** is the bias-instability random walk from the gyro Allan deviation.

### 3.5 Covariance prediction

$$
\mathbf P_{k+1}^- = \mathbf F_k\mathbf P_k\mathbf F_k^T + \mathbf Q_k
$$

---

## 4. Measurement Models

### 4.1 Payload gyro (every sample)

$$
\mathbf z_1 = \tilde{\boldsymbol\omega}_p = \boldsymbol\omega_p + \mathbf b_p + \boldsymbol\eta_p
$$

$$
\mathbf H_1 = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf I_3\end{bmatrix},
\qquad \mathbf R_1 = \sigma_{gp}^2\mathbf I_3
$$

### 4.2 Handle gyro mapped through gimbal kinematics (every sample)

For a gimbal with joint angles θ, the rigid-body rate composition is:

$$
\boldsymbol\omega_p = \mathbf C_{PH}(\boldsymbol\theta)\,\boldsymbol\omega_h + \mathbf J(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
$$

- **C_PH(θ)** is the rotation from H to P, built from the joint angles (e.g., yaw → roll → pitch chain).
- **J(θ)** maps joint rates to payload-frame angular velocity. Its columns are the joint axes expressed in P.

The pseudo-measurement is:

$$
\mathbf z_2 = \mathbf C_{PH}(\boldsymbol\theta)\,\tilde{\boldsymbol\omega}_h + \mathbf J(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
= \boldsymbol\omega_p + \boldsymbol\eta_2
$$

$$
\mathbf H_2 = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf 0_3\end{bmatrix}
$$

$$
\mathbf R_2 = \mathbf C_{PH}\,\sigma_{gh}^2\,\mathbf C_{PH}^T + \mathbf J\,\Sigma_{\dot\theta}\,\mathbf J^T + \sigma_{flex}^2\mathbf I_3
$$

- σ_flex accounts for structural compliance and backlash between the handle and payload. It should be inflated near arm or plate resonances.
- Because z₂ has no bₚ term while z₁ does, the difference z₁ − z₂ makes **bₚ directly observable**. This is the key benefit of the dual-IMU arrangement.

### 4.3 Payload accelerometer (gravity reference, gated)

With q = [q₀, q₁, q₂, q₃] and gravity along +z:

$$
\mathbf z_3 = \tilde{\mathbf a}_p,\qquad
h_3(\mathbf q) = g\begin{bmatrix}
2(q_1q_3 - q_0q_2)\\
2(q_2q_3 + q_0q_1)\\
q_0^2 - q_1^2 - q_2^2 + q_3^2
\end{bmatrix}
$$

$$
\mathbf H_3 = \begin{bmatrix}
\dfrac{\partial h_3}{\partial\mathbf q} & \mathbf 0_3 & \mathbf 0_3
\end{bmatrix},\quad
\frac{\partial h_3}{\partial\mathbf q} = 2g\begin{bmatrix}
-q_2 & q_3 & -q_0 & q_1\\
q_1 & q_0 & q_3 & q_2\\
q_0 & -q_1 & -q_2 & q_3
\end{bmatrix}
$$

**Adaptive noise:**

$$
\mathbf R_3 = \left(\sigma_a^2 + k_a\big(\|\tilde{\mathbf a}_p\| - g\big)^2\right)\mathbf I_3
$$

Skip the z₃ update entirely if |‖ãₚ‖ − g| exceeds a threshold, such as 0.1g during walking or running shots.

---

## 5. Update Equations (for each available measurement i)

1. **Innovation:**
   $$\mathbf y_i = \mathbf z_i - h_i(\hat{\mathbf x}^-)$$
2. **Innovation covariance:**
   $$\mathbf S_i = \mathbf H_i\mathbf P^-\mathbf H_i^T + \mathbf R_i$$
3. **Kalman gain:**
   $$\mathbf K_i = \mathbf P^-\mathbf H_i^T\mathbf S_i^{-1}$$
4. **State update:**
   $$\hat{\mathbf x}^+ = \hat{\mathbf x}^- + \mathbf K_i\mathbf y_i$$
   Then renormalize q̂.
5. **Covariance update (Joseph form, for numerical robustness in fixed-point or float32):**
   $$\mathbf P^+ = (\mathbf I-\mathbf K_i\mathbf H_i)\mathbf P^-(\mathbf I-\mathbf K_i\mathbf H_i)^T + \mathbf K_i\mathbf R_i\mathbf K_i^T$$

**Implementation notes**
- z₁ and z₂ can be stacked into a single 6-D update. Alternatively, process them sequentially, which is cheaper since only 3×3 inverses are needed.
- **Outlier gating:** reject any measurement with Mahalanobis distance yᵢᵀSᵢ⁻¹yᵢ > χ²₃(0.999) ≈ 16.3.

---

## 6. Outputs to the Vibration-Isolation Controller

| Signal | Use |
|---|---|
| ω̂ₚ | Rate-loop feedback. Lower noise and lag than the raw gyro. |
| ω̃ₚ − b̂ₚ | Alternative high-bandwidth feedback, if the filter's τ_ω lowpass adds too much phase lag. |
| q̂ | Outer attitude loop and horizon leveling. |
| C_PH·ω̃ₕ