# 10-State EKF for Payload/Handle Gyro Fusion in Gimbal Vibration Isolation (Cycle 1)

## 1. Problem Setup and Frames

| Frame | Description |
|---|---|
| **N** | Navigation frame, z-up, gravity **g**ₙ = [0, 0, −g]ᵀ |
| **P** | Camera payload body frame (payload IMU mounted here) |
| **H** | Handle frame (handle IMU mounted here) |
| θ = [θ₁, θ₂, θ₃] | Gimbal joint angles from the encoders (yaw, roll, pitch chain) |

**Goal.** Estimate the payload attitude and the *true* payload angular rate with minimal lag. The rate-loop controller can then reject handle vibration before it reaches the payload.

**Roles of the two IMUs:**
- The **handle IMU** observes the disturbance *upstream* of the motors. It is a leading indicator that gives early warning of vibration.
- The **payload IMU** observes the residual motion that actually reaches the camera.

---

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \boldsymbol{\omega} \\ \mathbf{b}_p \end{bmatrix}
\in \mathbb{R}^{10},\qquad
\begin{aligned}
\mathbf{q} &= [q_0, q_1, q_2, q_3]^T && \text{payload attitude (Hamilton, P→N)} \\
\boldsymbol{\omega} &= [\omega_x, \omega_y, \omega_z]^T && \text{true payload angular rate in P (rad/s)} \\
\mathbf{b}_p &= [b_x, b_y, b_z]^T && \text{payload gyro bias (rad/s)}
\end{aligned}
$$

ω is carried as a **state** rather than used as an input. This is the key design choice: it lets two independent gyro measurements, plus a dynamics model, be fused into one rate estimate.

**Inputs:**
- **u** = τₘ, the commanded motor torque mapped into P.
- θ, θ̇ from the encoders.

---

## 3. Continuous-Time Process Model

$$
\begin{aligned}
\dot{\mathbf{q}} &= \tfrac{1}{2}\,\boldsymbol{\Omega}(\boldsymbol{\omega})\,\mathbf{q} \\
\dot{\boldsymbol{\omega}} &= \mathbf{I}_p^{-1}\big(\boldsymbol{\tau}_m - \boldsymbol{\omega}\times \mathbf{I}_p\boldsymbol{\omega}\big) + \mathbf{w}_\omega \\
\dot{\mathbf{b}}_p &= -\tfrac{1}{\tau_b}\mathbf{b}_p + \mathbf{w}_b \quad(\text{or } \tau_b\to\infty \text{ for a random walk})
\end{aligned}
$$

with

$$
\boldsymbol{\Omega}(\boldsymbol{\omega}) =
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z\\
\omega_x & 0 & \omega_z & -\omega_y\\
\omega_y & -\omega_z & 0 & \omega_x\\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

where **I**ₚ is the payload inertia. The term **w**_ω captures unmodeled torques: cable drag, friction, imbalance, and vibration coupled through the motors.

---

## 4. Discrete Prediction (step Δt)

**Attitude** (exact integration for constant ω over the step):

$$
\mathbf{q}_{k+1|k} = \mathbf{q}_{k|k} \otimes
\begin{bmatrix}\cos(\tfrac{\|\boldsymbol\omega\|\Delta t}{2}) \\ \frac{\boldsymbol\omega}{\|\boldsymbol\omega\|}\sin(\tfrac{\|\boldsymbol\omega\|\Delta t}{2})\end{bmatrix},
\qquad \mathbf{q}\leftarrow \mathbf{q}/\|\mathbf{q}\|
$$

**Rate:**

$$
\boldsymbol{\omega}_{k+1|k} = \boldsymbol{\omega}_{k} + \Delta t\, \mathbf{I}_p^{-1}\big(\boldsymbol{\tau}_{m,k} - \boldsymbol{\omega}_k\times\mathbf{I}_p\boldsymbol{\omega}_k\big)
$$

**Bias:**

$$
\mathbf{b}_{p,k+1|k} = e^{-\Delta t/\tau_b}\,\mathbf{b}_{p,k}
$$

**Covariance:**

$$
\mathbf{P}_{k+1|k} = \mathbf{F}_k\mathbf{P}_{k|k}\mathbf{F}_k^T + \mathbf{Q}_k,\qquad \mathbf{F}_k \approx \mathbf{I}_{10} + \mathbf{A}\,\Delta t
$$

### Process Jacobian A = ∂f/∂x (10×10)

$$
\mathbf{A} =
\begin{bmatrix}
\tfrac12\boldsymbol\Omega(\boldsymbol\omega) & \tfrac12\boldsymbol\Xi(\mathbf q) & \mathbf 0_{4\times3}\\
\mathbf 0_{3\times4} & \mathbf{I}_p^{-1}\big([\mathbf{I}_p\boldsymbol\omega]_\times - [\boldsymbol\omega]_\times\mathbf{I}_p\big) & \mathbf 0_{3\times3}\\
\mathbf 0_{3\times4} & \mathbf 0_{3\times3} & -\tfrac{1}{\tau_b}\mathbf I_3
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

Here [·]ₓ denotes the skew-symmetric (cross-product) matrix.

### Process Noise

$$
\mathbf{Q}_k = \operatorname{diag}\big(\sigma_q^2\mathbf I_4,\;\sigma_\omega^2\mathbf I_3,\;\sigma_b^2\mathbf I_3\big)\Delta t
$$

- σ_q is small, just enough to cover numerical integration error.
- σ_ω sets the rate-estimate bandwidth. A large value trusts the gyros; a small value trusts the torque model.
- σ_b is the bias instability.

---

## 5. Measurement Models

### (a) Payload gyro (every IMU sample, e.g. 1–8 kHz)

$$
\mathbf{y}_g = \boldsymbol\omega + \mathbf b_p + \mathbf n_g
\qquad
\mathbf H_g = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf I_3\end{bmatrix},\quad \mathbf R_g=\sigma_g^2\mathbf I_3
$$

### (b) Handle gyro, mapped through the gimbal kinematics (every sample)

Rate composition along the joint chain:

$$
\boldsymbol\omega = \mathbf C_{PH}(\boldsymbol\theta)\,\boldsymbol\omega_H + \mathbf J(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
$$

Inverting this gives the predicted handle-gyro reading:

$$
\mathbf{y}_h = \mathbf C_{HP}(\boldsymbol\theta)\big(\boldsymbol\omega - \mathbf J(\boldsymbol\theta)\dot{\boldsymbol\theta}\big) + \mathbf n_h
$$

$$
\mathbf H_h = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf C_{HP}(\boldsymbol\theta) & \mathbf 0_{3\times3}\end{bmatrix}
$$

The encoder-rate noise is folded into the measurement covariance:

$$
\mathbf R_h = \sigma_h^2\mathbf I_3 + \mathbf C_{HP}\mathbf J\,\boldsymbol\Sigma_{\dot\theta}\,\mathbf J^T\mathbf C_{HP}^T
$$

- **J**(θ) is the joint-axis matrix: each column is a motor axis expressed in P.
- The handle gyro bias is assumed pre-calibrated at power-on in Cycle 1 (see §8).

### (c) Payload accelerometer (gravity reference, gated, e.g. 100–500 Hz)

$$
\mathbf y_a = \mathbf C_{PN}(\mathbf q)\begin{bmatrix}0\\0\\g\end{bmatrix} + \mathbf n_a
= g\begin{bmatrix}2(q_1q_3-q_0q_2)\\ 2(q_2q_3+q_0q_1)\\ q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix}+\mathbf n_a
$$

$$
\mathbf H_a = g\begin{bmatrix}
-2q_2 & 2q_3 & -2q_0 & 2q_1 & \\
2q_1 & 2q_0 & 2q_3 & 2q_2 & \mathbf 0_{3\times6}\\
2q_0 & -2q_1 & -2q_2 & 2q_3 &
\end{bmatrix}
$$

**Gating rule:** if |‖**y**ₐ‖ − g| > ε (typically 0.05g–0.1g), either skip the update or inflate **R**ₐ, for example **R**ₐ = σₐ²(1 + k·|‖**y**ₐ‖ − g|)**I**₃.

---

## 6. Update Equations (applied sequentially per sensor j ∈ {g, h, a})

$$
\begin{aligned}
\tilde{\mathbf y}_j &= \mathbf y_j - h_j(\hat{\mathbf x}_{k|k-1}) \\
\mathbf S_j &= \mathbf H_j\mathbf P_{k|k-1}\mathbf H_j^T + \mathbf R_j\\
\mathbf K_j &= \mathbf P_{k|k-1}\mathbf H_j^T\mathbf S_j^{-1}\\
\hat{\mathbf x}_{k|k} &= \hat{\mathbf x}_{k|k-1} + \mathbf K_j\tilde{\mathbf y}_j\\
\mathbf P_{k|k} &= (\mathbf I-\mathbf K_j\mathbf H_j)\mathbf P_{k|k-1}(\mathbf I-\mathbf K_j\mathbf H_j)^T + \mathbf K_j\mathbf R_j\mathbf K_j^T \quad(\text{Joseph form})
\end{aligned}
$$

After each update, renormalize the quaternion: **q** ← **q**/‖**q**‖.

Because each measurement block is 3×3, sequential processing keeps the S⁻¹ inversion cheap enough for an MCU. Optionally, reject outliers with an innovation gate: ỹᵀS⁻¹ỹ > χ²₃(0.997) ≈