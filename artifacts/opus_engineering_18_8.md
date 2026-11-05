# 10-State EKF: Payload/Handle Gyro Fusion for Gimbal Vibration Isolation

I don't have the specific conventions from your "Cycle 18" documents, so this formulation uses a standard handheld 3-axis gimbal architecture. Adjust the frames, joint order and sensor rates to match your design.

## 1. Architecture and Frames

| Symbol | Meaning |
|---|---|
| **n** | Navigation frame (gravity along +z, so the accelerometer at rest reads [0, 0, +g]) |
| **p** | Payload (camera) body frame, containing IMU_p (gyro + accelerometer) |
| **h** | Handle body frame, containing IMU_h (gyro) |
| θ = [θ₁ θ₂ θ₃]ᵀ | Joint angles from the motor encoders |
| R_hp(θ) | Rotation from payload frame to handle frame (forward kinematics) |
| J(θ) | Joint-rate Jacobian, giving the relative rate ω_rel = J(θ) θ̇ (payload with respect to handle, expressed in **p**) |

The core idea is a rigid kinematic chain. Because both gyros see the same motion through the gimbal, the handle gyro and the encoders provide a redundant measurement of payload rate. This makes the biases of both gyros observable and gives a clean handle-disturbance rate for feedforward.

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \mathbf{b}_p \\ \mathbf{b}_h \end{bmatrix}
\in \mathbb{R}^{10},\qquad
\mathbf{q} = [q_w\; q_x\; q_y\; q_z]^T
$$

- **q**: payload attitude quaternion (Hamilton convention, rotating p to n). 4 states.
- **b_p**: payload gyro bias. 3 states.
- **b_h**: handle gyro bias. 3 states.

Payload gyro measurement model:

$$
\tilde{\boldsymbol\omega}_p = \boldsymbol\omega_p + \mathbf b_p + \mathbf n_p
$$

Handle gyro measurement model:

$$
\tilde{\boldsymbol\omega}_h = \boldsymbol\omega_h + \mathbf b_h + \mathbf n_h
$$

## 3. Process Model (Prediction)

### 3.1 Continuous-time dynamics

The payload gyro is used as the control input.

$$
\dot{\mathbf q} = \tfrac12\,\boldsymbol\Omega(\tilde{\boldsymbol\omega}_p - \mathbf b_p - \mathbf n_p)\,\mathbf q,\qquad
\dot{\mathbf b}_p = \mathbf w_{bp},\qquad
\dot{\mathbf b}_h = \mathbf w_{bh}
$$

The rate matrix is:

$$
\boldsymbol\Omega(\boldsymbol\omega)=\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z\\
\omega_x & 0 & \omega_z & -\omega_y\\
\omega_y & -\omega_z & 0 & \omega_x\\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

An equivalent form is useful for the Jacobians:

$$
\boldsymbol\Omega(\boldsymbol\omega)\mathbf q = \boldsymbol\Xi(\mathbf q)\boldsymbol\omega,\qquad
\boldsymbol\Xi(\mathbf q)=\begin{bmatrix}
-q_x & -q_y & -q_z\\
q_w & -q_z & q_y\\
q_z & q_w & -q_x\\
-q_y & q_x & q_w
\end{bmatrix}
$$

### 3.2 Discrete propagation (step Δt, at the IMU rate, e.g. 1–4 kHz)

First compute the bias-corrected rate and its magnitude:

$$
\hat{\boldsymbol\omega}_k = \tilde{\boldsymbol\omega}_{p,k} - \hat{\mathbf b}_{p,k},\qquad
\phi = \|\hat{\boldsymbol\omega}_k\|\Delta t
$$

Then propagate the states:

$$
\hat{\mathbf q}_{k+1}^- = \Big[\cos\tfrac{\phi}{2}\,\mathbf I_4 + \tfrac{\sin(\phi/2)}{\|\hat{\boldsymbol\omega}_k\|}\,\boldsymbol\Omega(\hat{\boldsymbol\omega}_k)\Big]\hat{\mathbf q}_k
\;\equiv\;\boldsymbol\Phi_q\,\hat{\mathbf q}_k
$$

$$
\hat{\mathbf b}_{p,k+1}^- = \hat{\mathbf b}_{p,k},\qquad \hat{\mathbf b}_{h,k+1}^- = \hat{\mathbf b}_{h,k}
$$

When φ → 0, use the limit sin(φ/2)/‖ω̂‖ → Δt/2.

### 3.3 State transition Jacobian (10×10)

$$
\mathbf F_k=\begin{bmatrix}
\boldsymbol\Phi_q & -\tfrac{\Delta t}{2}\boldsymbol\Xi(\hat{\mathbf q}_k) & \mathbf 0_{4\times3}\\
\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf 0_3\\
\mathbf 0_{3\times4} & \mathbf 0_3 & \mathbf I_3
\end{bmatrix}
$$

### 3.4 Process noise

Noise input matrix:

$$
\mathbf G_k=\begin{bmatrix}
-\tfrac{\Delta t}{2}\boldsymbol\Xi(\hat{\mathbf q}_k) & \mathbf 0 & \mathbf 0\\
\mathbf 0 & \mathbf I_3 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \mathbf I_3
\end{bmatrix}
$$

Noise covariance:

$$
\mathbf Q_k = \mathbf G_k\,\mathrm{diag}\!\big(\sigma_{gp}^2\mathbf I_3/\Delta t,\;\sigma_{bp}^2\Delta t\,\mathbf I_3,\;\sigma_{bh}^2\Delta t\,\mathbf I_3\big)\,\mathbf G_k^T
$$

Here σ_gp is the payload gyro angle random walk (ARW) density, and σ_bp, σ_bh are the bias instability random-walk densities.

### 3.5 Covariance prediction

$$
\mathbf P_{k+1}^- = \mathbf F_k\mathbf P_k\mathbf F_k^T + \mathbf Q_k
$$

## 4. Measurement Models (Correction)

### 4.1 Handle gyro and encoder kinematic constraint (the fusion step)

Rigid-chain kinematics give:

$$
\boldsymbol\omega_h = \mathbf R_{hp}(\theta)\big(\boldsymbol\omega_p - \mathbf J(\theta)\dot\theta\big)
$$

This yields the predicted handle gyro reading:

$$
\mathbf h_1(\mathbf x) = \mathbf R_{hp}(\theta)\big(\tilde{\boldsymbol\omega}_p - \mathbf b_p - \mathbf J(\theta)\dot\theta\big) + \mathbf b_h,
\qquad \mathbf z_1 = \tilde{\boldsymbol\omega}_h
$$

The measurement Jacobian is:

$$
\mathbf H_1 = \begin{bmatrix}\mathbf 0_{3\times4} & -\mathbf R_{hp}(\theta) & \mathbf I_3\end{bmatrix}
$$

The measurement noise includes payload gyro noise mapped through the kinematics, plus encoder-rate noise:

$$
\mathbf R_1 = \sigma_{gh}^2\mathbf I_3 + \mathbf R_{hp}\big(\sigma_{gp}^2\mathbf I_3 + \mathbf J\,\Sigma_{\dot\theta}\,\mathbf J^T\big)\mathbf R_{hp}^T
$$

Notes on this update:

- The payload gyro noise appears both here and in the process input, so the two are correlated. Inflating R₁ (by roughly 1.5–2×) is the usual practical fix. Alternatively, apply this update at a decimated rate.
- θ̇ should come from a differentiated, filtered encoder signal. It must be time-aligned with both IMUs; sub-sample skew directly corrupts this update.
- This update constrains **b_h − R_hp b_p**. The individual biases become observable as the gimbal angles θ vary and when combined with the gravity update.

### 4.2 Payload accelerometer (gravity reference for roll and pitch)

$$
\mathbf h_2(\mathbf q) = \mathbf C(\mathbf q)^T\begin{bmatrix}0\\0\\g\end{bmatrix}
= g\begin{bmatrix}2(q_xq_z - q_wq_y)\\ 2(q_yq_z + q_wq_x)\\ q_w^2-q_x^2-q_y^2+q_z^2\end{bmatrix},
\qquad \mathbf z_2 = \tilde{\mathbf a}_p
$$

$$
\mathbf H_2 = \begin{bmatrix}
2g\begin{bmatrix}
-q_y & q_z & -q_w & q_x\\
q_x & q_w & q_z & q_y\\
q_w & -q_x & -q_y & q_z
\end{bmatrix} & \mathbf 0_{3\times6}\end{bmatrix}
$$

The measurement noise is adaptive, to reject linear acceleration:

$$
\mathbf R_2 = \big(\sigma_a^2 + k_a\,\big|\|\tilde{\mathbf a}_p\| - g\big|^2\big)\mathbf I_3
