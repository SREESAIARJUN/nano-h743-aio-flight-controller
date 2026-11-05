# 10-State EKF for Gimbal Vibration Isolation (Payload IMU + Handle IMU Fusion)

## 1. Problem Setup

A handheld 3-axis gimbal has two IMUs:

- **Payload IMU** on the camera, frame *p*. Its gyro measures the camera's inertial angular rate.
- **Handle IMU** on the grip, frame *h*. Its gyro measures the operator's hand motion, including tremor, walking shock and vibration.

The two are linked through the gimbal joints. The motor encoders give the joint angles θ and joint rates θ̇.

The filter has three jobs:

1. Estimate the payload attitude.
2. Estimate both gyro biases, so the inner rate loop and the disturbance feedforward are drift-free.
3. Produce clean estimates of the payload rate ω̂ₚ and the handle disturbance d̂ for the stabilization controller.

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \mathbf{b}_p \\ \mathbf{b}_h \end{bmatrix}
\in \mathbb{R}^{10},\qquad
\mathbf{q} = [q_w\ q_x\ q_y\ q_z]^T
$$

| States | Symbol | Description |
|---|---|---|
| 1–4 | **q** | Payload attitude quaternion, payload → nav (NED), Hamilton convention |
| 5–7 | **b**ₚ | Payload gyro bias (rad/s) |
| 8–10 | **b**ₕ | Handle gyro bias (rad/s) |

## 3. Sensor Models

**Payload gyro** (used as the process input):
$$
\boldsymbol{\omega}_{gp} = \boldsymbol{\omega}_p + \mathbf{b}_p + \mathbf{n}_{gp},\qquad \mathbf{n}_{gp}\sim\mathcal N(0,\sigma_{gp}^2 I)
$$

**Handle gyro** (used as a measurement):
$$
\boldsymbol{\omega}_{gh} = \boldsymbol{\omega}_h + \mathbf{b}_h + \mathbf{n}_{gh}
$$

**Gimbal kinematic chain**, linking the two bodies through the encoders:
$$
\boldsymbol{\omega}_p = \mathbf{C}_{ph}(\boldsymbol\theta)\,\boldsymbol{\omega}_h + \mathbf{J}(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
$$

- **C**ₚₕ(θ) is the rotation from the handle frame to the payload frame, built from the encoder angles.
- **J**(θ) is the joint-rate Jacobian, expressed in the payload frame. For a yaw–roll–pitch gimbal, its columns are the joint axes rotated into the payload frame.

**Payload accelerometer** (used as a measurement):
$$
\mathbf{f}_p = \mathbf{C}(\mathbf{q})^T(\mathbf{a}_n - \mathbf{g}_n) + \mathbf{n}_a,\qquad \mathbf{g}_n = [0\ 0\ g]^T
$$

## 4. Process Model

### Continuous time
$$
\dot{\mathbf q} = \tfrac12\,\mathbf q\otimes\begin{bmatrix}0\\ \boldsymbol\omega_{gp}-\mathbf b_p - \mathbf n_{gp}\end{bmatrix}
$$
$$
\dot{\mathbf b}_p = -\frac{1}{\tau_p}\mathbf b_p + \mathbf w_{bp},\qquad
\dot{\mathbf b}_h = -\frac{1}{\tau_h}\mathbf b_h + \mathbf w_{bh}
$$

The biases are modeled as first-order Gauss-Markov processes. Setting τ → ∞ reduces them to random walks.

### Discrete prediction (step Δt, e.g. 1 kHz)

Define the bias-corrected rate and the rotation angle over one step:
$$
\bar{\boldsymbol\omega}_k = \boldsymbol\omega_{gp,k} - \hat{\mathbf b}_{p,k},\qquad \phi=\|\bar{\boldsymbol\omega}_k\|\Delta t
$$

**Attitude propagation:**
$$
\hat{\mathbf q}_{k+1}^- = \underbrace{\left[\cos\tfrac{\phi}{2}\,I_4 + \frac{\sin(\phi/2)}{\|\bar{\boldsymbol\omega}_k\|}\,\boldsymbol\Omega(\bar{\boldsymbol\omega}_k)\right]}_{\boldsymbol\Phi_q}\hat{\mathbf q}_k
$$

where
$$
\boldsymbol\Omega(\boldsymbol\omega)=\begin{bmatrix}0&-\omega_x&-\omega_y&-\omega_z\\ \omega_x&0&\omega_z&-\omega_y\\ \omega_y&-\omega_z&0&\omega_x\\ \omega_z&\omega_y&-\omega_x&0\end{bmatrix}
$$

**Bias propagation:**
$$
\hat{\mathbf b}_{p,k+1}^- = e^{-\Delta t/\tau_p}\hat{\mathbf b}_{p,k},\qquad
\hat{\mathbf b}_{h,k+1}^- = e^{-\Delta t/\tau_h}\hat{\mathbf b}_{h,k}
$$

### State transition Jacobian (10×10)

$$
\mathbf F_k=\begin{bmatrix}
\boldsymbol\Phi_q & -\tfrac{\Delta t}{2}\boldsymbol\Xi(\hat{\mathbf q}_k) & \mathbf 0_{4\times3}\\
\mathbf 0_{3\times4} & e^{-\Delta t/\tau_p}I_3 & \mathbf 0\\
\mathbf 0_{3\times4} & \mathbf 0 & e^{-\Delta t/\tau_h}I_3
\end{bmatrix}
$$

The matrix **Ξ**(q) is defined by q ⊗ [0; v] = **Ξ**(q) v:
$$
\boldsymbol\Xi(\mathbf q)=\begin{bmatrix}-q_x&-q_y&-q_z\\ q_w&-q_z&q_y\\ q_z&q_w&-q_x\\ -q_y&q_x&q_w\end{bmatrix}
$$

### Process noise

The noise vector is w = [n_gp; w_bp; w_bh]. Its input matrix is
$$
\mathbf G_k=\begin{bmatrix}-\tfrac12\boldsymbol\Xi(\hat{\mathbf q}_k)&0&0\\0&I_3&0\\0&0&I_3\end{bmatrix}
$$

The discrete process noise covariance is
$$
\mathbf Q_k \approx \mathbf G_k\,\mathrm{diag}\!\left(\sigma_{gp}^2 I,\ \sigma_{bp}^2 I,\ \sigma_{bh}^2 I\right)\mathbf G_k^T\,\Delta t
$$

### Covariance prediction
$$
\mathbf P_{k+1}^- = \mathbf F_k\mathbf P_k\mathbf F_k^T + \mathbf Q_k
$$

## 5. Measurement Models

### (a) Handle gyro via the kinematic chain (every cycle, 1 kHz)

Substitute ω̂ₚ = ω_gp − **b**ₚ into the inverted kinematic chain:
$$
\mathbf z_1 = \boldsymbol\omega_{gh},\qquad
h_1(\mathbf x)=\mathbf C_{hp}(\boldsymbol\theta)\big(\boldsymbol\omega_{gp}-\mathbf b_p-\mathbf J\dot{\boldsymbol\theta}\big)+\mathbf b_h
$$

The Jacobian is
$$
\mathbf H_1=\begin{bmatrix}\mathbf 0_{3\times4} & -\mathbf C_{hp}(\boldsymbol\theta) & I_3\end{bmatrix}
$$

The payload gyro noise enters this measurement through ω_gp, so the effective measurement covariance includes it. It also includes encoder-rate noise:
$$
\mathbf R_1 = \sigma_{gh}^2 I_3 + \mathbf C_{hp}\,\sigma_{gp}^2\,\mathbf C_{hp}^T + \mathbf C_{hp}\mathbf J\,\Sigma_{\dot\theta}\,\mathbf J^T\mathbf C_{hp}^T
$$

This measurement makes the bias difference observable: **C**ₕₚ**b**ₚ − **b**ₕ. The diversity of θ as the gimbal moves helps separate the two biases.

### (b) Payload accelerometer gravity reference (gated, e.g. 200 Hz)

$$
h_2(\mathbf x) = -g\begin{bmatrix}2(q_xq_z-q_wq_y)\\ 2(q_yq_z+q_wq_x)\\ q_w^2-q_x^2-q_y^2+q_z^2\end{bmatrix}
$$

The Jacobian is
$$
\mathbf H_2=\Big[\,-2g\begin{bmatrix}-q_y&q_z&-q_w&q_x\\ q_x&q_w&q_z&q_y\\ q_w&-q_x&-q_y&q_z\end{bmatrix}\;\;\mathbf 0_{3\times3}\;\;\mathbf 0_{3\times3}\Big]
$$

Use this measurement only when the camera is not accelerating much:

- **Gate:** apply the update only if |‖f_p‖ − g| < ε_a.
- **Adaptive noise:** otherwise, or in addition, inflate the noise as R₂ = (σ_a² + k·(‖f_p‖ − g)²) I₃.

This measurement anchors roll and pitch, which makes **b**ₚ observable on the x and y axes. Through measurement (a), it then also anchors **b**ₕ.

### (c) Optional yaw reference

Yaw bias is unobservable from gravity alone. Options:

- A handle magnetometer combined with the encoder yaw angle.
- A "