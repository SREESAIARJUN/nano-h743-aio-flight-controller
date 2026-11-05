# 10-State EKF for Dual-IMU Gimbal Vibration Isolation (Cycle 2)

## 1. Scope and Assumptions

This formulation assumes the following, since "Cycle 2" can mean different things in different design flows. Adjust these if your Cycle 1 baseline differs.

- **Cycle 1 baseline:** payload IMU only. The payload attitude and payload gyro bias were estimated (7 states).
- **Cycle 2 addition:** the handle (base) IMU gyro is added. The two IMUs are linked through the gimbal joint kinematics (motor encoders), and the handle gyro bias is estimated. The estimated handle rate is used as a feed-forward disturbance signal for vibration isolation.
- **Frames:**
  - **n**: navigation frame, z-up, gravity **g**ₙ = [0, 0, −g]ᵀ.
  - **p**: payload (camera) frame.
  - **h**: handle frame.
- **Quaternion convention:** Hamilton, scalar-first, q = [q_w, q_x, q_y, q_z]ᵀ. The quaternion maps p → n.

---

## 2. State Vector (10 states)

$$
\mathbf{x} =
\begin{bmatrix}
\mathbf{q}_{np} \\ \mathbf{b}_p \\ \mathbf{b}_h
\end{bmatrix}
\in \mathbb{R}^{10},
\qquad
\begin{aligned}
&\mathbf{q}_{np} \in \mathbb{R}^4 &&\text{payload attitude}\\
&\mathbf{b}_p \in \mathbb{R}^3 &&\text{payload gyro bias}\\
&\mathbf{b}_h \in \mathbb{R}^3 &&\text{handle gyro bias}
\end{aligned}
$$

---

## 3. Sensor Models

| Sensor | Model |
|---|---|
| Payload gyro | $\boldsymbol{\omega}_p^m = \boldsymbol{\omega}_p + \mathbf{b}_p + \mathbf{n}_{gp}$ |
| Handle gyro | $\boldsymbol{\omega}_h^m = \boldsymbol{\omega}_h + \mathbf{b}_h + \mathbf{n}_{gh}$ |
| Payload accelerometer | $\mathbf{a}_p^m = \mathbf{R}(\mathbf{q})^{T}(\mathbf{a}_n - \mathbf{g}_n) + \mathbf{n}_a$ |
| Joint encoders | $\boldsymbol{\theta}^m = \boldsymbol{\theta} + \mathbf{n}_\theta$, with rate $\dot{\boldsymbol\theta}$ obtained by differentiation |

The joint kinematics couple the two IMUs:

$$
\boldsymbol{\omega}_p = \mathbf{R}_{ph}(\boldsymbol\theta)\,\boldsymbol{\omega}_h + \mathbf{J}(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
$$

- **R**ₚₕ(θ) is the yaw-roll-pitch chain of the gimbal axes, mapping handle → payload.
- **J**(θ) is the joint-rate Jacobian, which expresses the motor axis rates in the payload frame.

---

## 4. Prediction (Time Update)

### Process model

The bias-corrected payload rate is

$$
\hat{\boldsymbol\omega} = \boldsymbol{\omega}_p^m - \hat{\mathbf{b}}_p .
$$

The continuous dynamics are

$$
\dot{\mathbf q} = \tfrac12\,\boldsymbol\Omega(\hat{\boldsymbol\omega})\,\mathbf q,\qquad
\dot{\mathbf b}_p = \mathbf n_{bp},\qquad
\dot{\mathbf b}_h = \mathbf n_{bh}
$$

where

$$
\boldsymbol\Omega(\boldsymbol\omega)=
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z\\
\omega_x & 0 & \omega_z & -\omega_y\\
\omega_y & -\omega_z & 0 & \omega_x\\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}.
$$

### Discrete propagation (step Δt, exact for a constant rate)

$$
\hat{\mathbf q}_{k+1|k} =
\Big[\cos\!\big(\tfrac{\|\hat{\boldsymbol\omega}\|\Delta t}{2}\big)\mathbf I_4
+ \tfrac{\sin(\|\hat{\boldsymbol\omega}\|\Delta t/2)}{\|\hat{\boldsymbol\omega}\|}\boldsymbol\Omega(\hat{\boldsymbol\omega})\Big]\hat{\mathbf q}_{k|k}
$$

$$
\hat{\mathbf b}_{p,k+1|k} = \hat{\mathbf b}_{p,k|k},\qquad
\hat{\mathbf b}_{h,k+1|k} = \hat{\mathbf b}_{h,k|k}
$$

After propagation, normalize the quaternion: **q** ← **q**/‖**q**‖.

### State transition Jacobian (10×10)

$$
\mathbf F_k =
\begin{bmatrix}
\mathbf I_4 + \tfrac{\Delta t}{2}\boldsymbol\Omega(\hat{\boldsymbol\omega}) & -\tfrac{\Delta t}{2}\boldsymbol\Xi(\hat{\mathbf q}) & \mathbf 0_{4\times3}\\
\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf 0_3\\
\mathbf 0_{3\times4} & \mathbf 0_3 & \mathbf I_3
\end{bmatrix},
\qquad
\boldsymbol\Xi(\mathbf q)=
\begin{bmatrix}
-q_x & -q_y & -q_z\\
q_w & -q_z & q_y\\
q_z & q_w & -q_x\\
-q_y & q_x & q_w
\end{bmatrix}
$$

This uses the identity **q** ⊗ [0, **ω**] = **Ξ**(**q**) **ω**.

### Process noise

$$
\mathbf Q_k =
\begin{bmatrix}
\tfrac{\Delta t^2}{4}\boldsymbol\Xi\,(\sigma_{gp}^2\mathbf I_3)\,\boldsymbol\Xi^T & \mathbf 0 & \mathbf 0\\
\mathbf 0 & \sigma_{bp}^2\Delta t\,\mathbf I_3 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \sigma_{bh}^2\Delta t\,\mathbf I_3
\end{bmatrix}
$$

### Covariance propagation

$$
\mathbf P_{k+1|k} = \mathbf F_k\mathbf P_{k|k}\mathbf F_k^T + \mathbf Q_k
$$

---

## 5. Measurement Updates

### 5a. Payload accelerometer (gravity reference for roll and pitch)

$$
\mathbf h_a(\mathbf x) = \mathbf R(\mathbf q)^T(-\mathbf g_n)
= g\begin{bmatrix}
2(q_xq_z - q_wq_y)\\
2(q_yq_z + q_wq_x)\\
q_w^2 - q_x^2 - q_y^2 + q_z^2
\end{bmatrix}
$$

The Jacobian is

$$
\mathbf H_a =
\Big[\;2g\begin{bmatrix}
-q_y & q_z & -q_w & q_x\\
q_x & q_w & q_z & q_y\\
q_w & -q_x & -q_y & q_z
\end{bmatrix}\;\Big|\;\mathbf 0_{3\times3}\;\Big|\;\mathbf 0_{3\times3}\Big].
$$

Use adaptive noise to reject linear acceleration from walking and handheld motion:

$$
\mathbf R_a = \sigma_a^2\mathbf I_3\left(1 + k_a\,\big|\,\|\mathbf a^m\| - g\big|\right)
$$

Skip this update entirely if ‖**a**ᵐ‖ − g exceeds a threshold.

### 5b. Cross-IMU rate consistency (the Cycle 2 fusion step)

Form a pseudo-measurement from both gyros and the encoders:

$$
\mathbf z_\omega = \boldsymbol\omega_p^m - \mathbf R_{ph}(\boldsymbol\theta^m)\,\boldsymbol\omega_h^m - \mathbf J(\boldsymbol\theta^m)\,\dot{\boldsymbol\theta}^m
$$

$$
\mathbf h_\omega(\mathbf x) = \mathbf b_p - \mathbf R_{ph}\,\mathbf b_h
$$

$$
\mathbf H_\omega = \big[\;\mathbf 0_{3\times4}\;\big|\;\mathbf I_3\;\big|\;-\mathbf R_{ph}(\boldsymbol\theta)\;\big]
$$

The measurement noise combines all three sensor contributions:

$$
\mathbf R_\omega = \sigma_{gp}^2\mathbf I_3 + \mathbf R_{ph}\,\sigma_{gh}^2\,\mathbf R_{ph}^T + \mathbf J\,\boldsymbol\Sigma_{\dot\theta}\,\mathbf J^T
$$

Because **R**ₚₕ(θ) varies as the gimbal moves, both bias vectors become individually observable over time. With a fixed θ, only their difference would be observable.

### 5c. Optional: zero-rate update (ZUPT) when the handle is at rest

If ‖**ω**ₕᵐ‖ stays below a threshold for N samples:

$$
\mathbf z = \boldsymbol\omega_h^m,\qquad
\mathbf H = [\mathbf 0_{3\times4}\;|\;\mathbf 0_3\;|\;\mathbf I_3]
$$

### Standard EKF update (applied for each measurement type)

$$
\begin{aligned}
\mathbf y &= \mathbf z - \mathbf h(\hat{\mathbf x}_{k|k-1})\\
\mathbf S &= \mathbf H\mathbf P_{k|k-1}\mathbf H^T + \mathbf R\\
\mathbf K &= \mathbf P_{k|k-1}\mathbf H^T\mathbf S^{-1}\\
\hat{\mathbf x}_{k|k} &= \hat{\mathbf x}_{k|k-1} + \mathbf K\mathbf y,\qquad \mathbf q \leftarrow \mathbf q/\|\mathbf q\|\\
\mathbf P_{k|k} &= (\mathbf I-\mathbf K\mathbf H)\mathbf P_{k|k-1}(\mathbf I-\mathbf K\mathbf H)^T + \mathbf K\mathbf R\mathbf K^T
\end{aligned}
$$

The covariance update uses the Joseph form for numerical robustness on embedded floating-point hardware.

To reject outliers such as encoder glitches or shocks, gate on the Mahalanobis distance:

$$
\mathbf y^T\mathbf S^{-1}\mathbf y > \chi^2_{3,\,0.99} = 11.34 \;\Rightarrow\; \text{reject}
$$

---

## 6. Outputs for the Vibration-Isolation Controller

| Output | Equation | Use |
|---|---|---|
| Payload attitude | $\hat{\mathbf q}_{np}$ | Outer attitude-hold loop |
| Payload rate (feedback) | $\hat{\boldsymbol\omega}_p = \boldsymbol\omega_p^m - \hat{\mathbf b}_p$ | Inner rate loop (PID or LQR) |
| Handle rate (feed-forward) | $\hat{\boldsymbol\omega}_h = \boldsymbol\omega_h^m - \hat{\mathbf b}_h$ | Disturbance cancellation |

The feed-forward motor-rate command