# 10-State EKF for Gimbal Vibration Isolation: Payload and Handle IMU Fusion

## 1. Problem Setup

| Symbol | Meaning |
|---|---|
| {W} | World frame (gravity-aligned, z up) |
| {P} | Payload (camera) frame. The payload IMU is rigidly mounted here. |
| {H} | Handle frame. The handle IMU is mounted here and sees the full operator and walking vibration. |
| θ = [θ_yaw, θ_roll, θ_pitch] | Gimbal joint angles from the encoders |
| θ̇ | Joint rates (from encoder differentiation or motor observers) |

**Objective.** Estimate the payload inertial attitude and angular rate with minimal phase lag. The stabilization loop rejects base vibration using two signals:
- **Feedback** from the estimated payload rate ω̂.
- **Feed-forward** from the handle rate, which arrives "before" the disturbance propagates through the joints.

---

## 2. State Vector (n = 10)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \boldsymbol{\omega} \\ \mathbf{b}_p \end{bmatrix}
\in \mathbb{R}^{10}
$$

- **q** = [q_w, q_x, q_y, q_z]ᵀ: payload attitude, rotating P→W. It uses the Hamilton convention and has unit norm. (4 states)
- **ω**: payload angular velocity in {P}, in rad/s. (3 states)
- **b_p**: payload gyro bias in {P}. (3 states)

Handle gyro bias is treated as pre-calibrated. The handle gyro mainly contributes high-bandwidth rate information, so its slowly varying bias is absorbed into the measurement noise. See §8 for extensions.

---

## 3. Process Model

### Continuous time

$$
\dot{\mathbf{q}} = \tfrac12\,\mathbf{q}\otimes\begin{bmatrix}0\\ \boldsymbol{\omega}\end{bmatrix} = \tfrac12\,\Omega(\boldsymbol{\omega})\,\mathbf{q}
$$

$$
\dot{\boldsymbol{\omega}} = -\frac{1}{\tau_\omega}\boldsymbol{\omega} + \mathbf{B}\,\mathbf{u} + \mathbf{w}_\omega
$$

$$
\dot{\mathbf{b}}_p = \mathbf{w}_b
$$

The rate model is a first-order Gauss–Markov process:
- τ_ω is the rate correlation time, typically 5–50 ms.
- **u** is the optional commanded motor torque, with **B** ≈ J⁻¹ (payload inertia).
- If no torque input is used, set **u** = 0 and the model becomes a pure Gauss–Markov process.

The quaternion rate matrix is:

$$
\Omega(\boldsymbol{\omega}) =
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z\\
\omega_x & 0 & \omega_z & -\omega_y\\
\omega_y & -\omega_z & 0 & \omega_x\\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

### Discrete time (step Δt)

$$
\mathbf{q}_{k+1|k} = \left[\cos\!\tfrac{\|\boldsymbol\omega_k\|\Delta t}{2}\,\mathbf{I}_4 + \frac{\sin(\|\boldsymbol\omega_k\|\Delta t/2)}{\|\boldsymbol\omega_k\|}\,\Omega(\boldsymbol\omega_k)\right]\mathbf{q}_k
$$

$$
\boldsymbol{\omega}_{k+1|k} = \phi\,\boldsymbol{\omega}_k + \Gamma\,\mathbf{u}_k,
\qquad \phi = e^{-\Delta t/\tau_\omega},\quad \Gamma = \tau_\omega(1-\phi)\mathbf{B}
$$

$$
\mathbf{b}_{p,k+1|k} = \mathbf{b}_{p,k}
$$

After prediction, normalize: **q** ← **q** / ‖**q**‖.

### State-transition Jacobian

$$
\mathbf{F}_k = \frac{\partial f}{\partial \mathbf{x}} =
\begin{bmatrix}
\mathbf{I}_4 + \tfrac{\Delta t}{2}\Omega(\boldsymbol\omega) & \tfrac{\Delta t}{2}\Xi(\mathbf{q}) & \mathbf{0}_{4\times3}\\
\mathbf{0}_{3\times4} & \phi\,\mathbf{I}_3 & \mathbf{0}_{3}\\
\mathbf{0}_{3\times4} & \mathbf{0}_{3} & \mathbf{I}_3
\end{bmatrix}
$$

where Ξ(**q**) = ∂(**q** ⊗ [0, ω])/∂ω:

$$
\Xi(\mathbf{q}) =
\begin{bmatrix}
-q_x & -q_y & -q_z\\
\ \ q_w & -q_z & \ \ q_y\\
\ \ q_z & \ \ q_w & -q_x\\
-q_y & \ \ q_x & \ \ q_w
\end{bmatrix}
$$

### Process noise

$$
\mathbf{Q}_k = \begin{bmatrix}
\tfrac{\Delta t^2}{4}\Xi\,\mathbf{Q}_\omega\,\Xi^\top & \tfrac{\Delta t}{2}\Xi\,\mathbf{Q}_\omega & 0\\
\tfrac{\Delta t}{2}\mathbf{Q}_\omega\Xi^\top & \mathbf{Q}_\omega & 0\\
0 & 0 & \sigma_b^2\Delta t\,\mathbf{I}_3
\end{bmatrix},
\qquad
\mathbf{Q}_\omega = \sigma_{\dot\omega}^2\,\tfrac{\tau_\omega}{2}(1-\phi^2)\,\mathbf{I}_3
$$

Here σ_ω̇ sets how aggressively ω may change. Tune it to the highest disturbance frequency you intend to track.

### Covariance prediction

$$
\mathbf{P}_{k+1|k} = \mathbf{F}_k\mathbf{P}_{k|k}\mathbf{F}_k^\top + \mathbf{Q}_k
$$

---

## 4. Measurement Models

Apply the updates sequentially, since each sensor runs at its own rate.

### 4a. Payload gyro (e.g. 1–8 kHz)

$$
\mathbf{z}_g = \boldsymbol\omega + \mathbf{b}_p + \mathbf{v}_g,\qquad \mathbf{v}_g\sim\mathcal N(0,\sigma_g^2\mathbf I_3)
$$

$$
\mathbf{H}_g = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf I_3\end{bmatrix}
$$

### 4b. Handle gyro, mapped through the gimbal kinematics (same rate)

Rigid-body kinematics of the serial gimbal chain give:

$$
\boldsymbol\omega = \mathbf R_{PH}(\boldsymbol\theta)\,\boldsymbol\omega_h + \mathbf J_\theta(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
$$

- **J_θ** stacks the joint axes expressed in {P}. Its columns are each joint axis rotated by the downstream joint rotations.

Form the **pseudo-measurement**:

$$
\tilde{\mathbf z}_h = \mathbf R_{PH}(\boldsymbol\theta)\,(\mathbf z_h - \hat{\mathbf b}_h) + \mathbf J_\theta\,\dot{\boldsymbol\theta}_{\text{meas}} = \boldsymbol\omega + \tilde{\mathbf v}_h
$$

$$
\mathbf H_h = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf 0_3\end{bmatrix}
$$

$$
\tilde{\mathbf R}_h = \mathbf R_{PH}\,\mathbf R_h\,\mathbf R_{PH}^\top + \mathbf J_\theta\,\mathbf R_{\dot\theta}\,\mathbf J_\theta^\top + \mathbf R_{\text{flex}}
$$

The term **R_flex** covers structural compliance between handle and payload. Make it frequency- or innovation-adaptive (§6).

Because the handle path is bias-free with respect to **b_p**, the difference between 4a and 4b makes **b_p** observable at all times, not only when the payload is still.

### 4c. Payload accelerometer (gravity tilt reference, e.g. 200–1000 Hz)

$$
\mathbf z_a = \mathbf R(\mathbf q)^\top\begin{bmatrix}0\\0\\g\end{bmatrix} + \mathbf v_a
= g\begin{bmatrix}2(q_xq_z - q_wq_y)\\ 2(q_yq_z + q_wq_x)\\ q_w^2 - q_x^2 - q_y^2 + q_z^2\end{bmatrix} + \mathbf v_a
$$

Columns are ordered (q_w, q_x, q_y, q_z):

$$
\mathbf H_a = \begin{bmatrix}
2g\begin{bmatrix}
-q_y & \ \ q_z & -q_w & q_x\\
\ \ q_x & \ \ q_w & \ \ q_z & q_y\\
\ \ q_w & -q_x & -q_y & q_z
\end{bmatrix} & \mathbf 0_{3} & \mathbf 0_{3}
\end{bmatrix}
$$

**Linear-acceleration gating.** When the payload is accelerating, the gravity reference is corrupted, so inflate the noise:

$$
\mathbf R_a = \sigma_a^2\mathbf I_3\left(1 + k_a\,\big|\,\|\mathbf z_a\| - g\,\big| + k_\omega\|\hat{\boldsymbol\omega}\|^2 r\right)
$$

Skip the update entirely if ‖**z_a**‖ − g exceeds a threshold. Yaw is unobservable from the accelerometer, so either add a magnetometer or accept slow yaw drift (typical for handheld gimbals).

---

## 5. Update Equations (for each measurement j ∈ {g, h, a})

1. **Innovation:** $\mathbf y = \mathbf z_j - h_j(\hat{\mathbf x}_{k+1|k})$
2. **Innovation covariance:** $\mathbf S = \mathbf H_j\mathbf P\mathbf H_j^\top + \mathbf R_j$
3. **Gain:** $\mathbf