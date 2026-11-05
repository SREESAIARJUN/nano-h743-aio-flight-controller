# 10-State EKF for Payload/Handle IMU Fusion in Gimbal Vibration Isolation (Cycle 10)

## 1. System Setup and Frames

| Symbol | Meaning |
|---|---|
| **W** | World frame (NED or ENU), gravity $\mathbf g_W = [0,0,g]^T$ |
| **P** | Camera payload frame. The payload IMU is rigidly mounted here. |
| **H** | Handle frame. The handle IMU is rigidly mounted here. |
| $\boldsymbol\theta = [\theta_1,\theta_2,\theta_3]$ | Gimbal joint angles from the motor encoders (yaw, roll, pitch chain) |
| $\mathbf R_{PH}(\boldsymbol\theta)$ | Rotation from H to P from forward kinematics |
| $\mathbf J(\boldsymbol\theta)$ | Joint-rate Jacobian, which maps $\dot{\boldsymbol\theta}$ to the relative angular rate expressed in P |

**Rigid-chain kinematic constraint.** This is the key relation that couples the two IMUs:

$$
\boldsymbol\omega_P = \mathbf R_{PH}(\boldsymbol\theta)\,\boldsymbol\omega_H + \mathbf J(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
$$

---

## 2. State Vector (10 nominal states, 9 error states)

$$
\mathbf x = \begin{bmatrix} \mathbf q_{WP} \\ \mathbf b_P \\ \mathbf b_H \end{bmatrix}\in\mathbb R^{10}
\qquad
\begin{aligned}
&\mathbf q_{WP}\in\mathbb R^4 &&\text{payload attitude quaternion}\\
&\mathbf b_P\in\mathbb R^3 &&\text{payload gyro bias}\\
&\mathbf b_H\in\mathbb R^3 &&\text{handle gyro bias}
\end{aligned}
$$

The quaternion has a unit-norm constraint, so the filter uses an **error-state (multiplicative) EKF** with a 9-dimensional error state:

$$
\delta\mathbf x = \begin{bmatrix}\delta\boldsymbol\phi \\ \delta\mathbf b_P \\ \delta\mathbf b_H\end{bmatrix}\in\mathbb R^9,
\qquad
\mathbf q_{WP} = \hat{\mathbf q}_{WP}\otimes \begin{bmatrix}1\\ \tfrac12\delta\boldsymbol\phi\end{bmatrix}
$$

The covariance $\mathbf P$ is therefore 9×9.

---

## 3. Sensor Models

**Gyroscopes**

$$
\tilde{\boldsymbol\omega}_P = \boldsymbol\omega_P + \mathbf b_P + \mathbf n_{gP},\qquad
\tilde{\boldsymbol\omega}_H = \boldsymbol\omega_H + \mathbf b_H + \mathbf n_{gH}
$$

**Bias random walks**

$$
\dot{\mathbf b}_P = \mathbf n_{bP},\qquad \dot{\mathbf b}_H = \mathbf n_{bH}
$$

**Accelerometers**, assuming quasi-static conditions:

$$
\tilde{\mathbf a}_P = \mathbf R_{WP}^T\mathbf g_W + \mathbf a_{dyn,P} + \mathbf n_{aP}
$$

**Encoders**

$$
\tilde{\boldsymbol\theta} = \boldsymbol\theta + \mathbf n_\theta
$$

$\dot{\tilde{\boldsymbol\theta}}$ is obtained by differentiating the encoder signal and low-pass filtering it.

---

## 4. Prediction Step (driven by the payload gyro, rate $f_s$ ≈ 1–4 kHz)

### 4.1 Nominal state propagation

$$
\hat{\boldsymbol\omega}_P = \tilde{\boldsymbol\omega}_P - \hat{\mathbf b}_P
$$

$$
\hat{\mathbf q}_{k+1} = \hat{\mathbf q}_k\otimes
\begin{bmatrix}\cos\!\left(\tfrac{\|\hat{\boldsymbol\omega}_P\|\Delta t}{2}\right)\\[2pt]
\dfrac{\hat{\boldsymbol\omega}_P}{\|\hat{\boldsymbol\omega}_P\|}\sin\!\left(\tfrac{\|\hat{\boldsymbol\omega}_P\|\Delta t}{2}\right)\end{bmatrix},
\qquad
\hat{\mathbf b}_{P,k+1}=\hat{\mathbf b}_{P,k},\quad
\hat{\mathbf b}_{H,k+1}=\hat{\mathbf b}_{H,k}
$$

### 4.2 Error-state dynamics

$$
\delta\dot{\mathbf x} = \mathbf F\,\delta\mathbf x + \mathbf G\,\mathbf w,\qquad
\mathbf w = \begin{bmatrix}\mathbf n_{gP}\\ \mathbf n_{bP}\\ \mathbf n_{bH}\end{bmatrix}
$$

$$
\mathbf F=\begin{bmatrix}
-[\hat{\boldsymbol\omega}_P]_\times & -\mathbf I_3 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \mathbf 0
\end{bmatrix},
\qquad
\mathbf G=\begin{bmatrix}
-\mathbf I_3 & \mathbf 0 & \mathbf 0\\
\mathbf 0 & \mathbf I_3 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \mathbf I_3
\end{bmatrix}
$$

### 4.3 Discretization

$$
\boldsymbol\Phi_k \approx \begin{bmatrix}
\mathrm{Exp}(-\hat{\boldsymbol\omega}_P\Delta t) & -\mathbf I_3\Delta t & \mathbf 0\\
\mathbf 0 & \mathbf I_3 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \mathbf I_3
\end{bmatrix}
$$

$$
\mathbf Q_d \approx \mathrm{diag}\!\left(\sigma_{gP}^2\Delta t\,\mathbf I_3,\;\; \sigma_{bP}^2\Delta t\,\mathbf I_3,\;\; \sigma_{bH}^2\Delta t\,\mathbf I_3\right)
$$

Here $\sigma_g$ is the angle random walk (rad/s/√Hz) and $\sigma_b$ is the bias instability (rad/s²/√Hz).

### 4.4 Covariance propagation

$$
\mathbf P_{k+1}^- = \boldsymbol\Phi_k\,\mathbf P_k^+\,\boldsymbol\Phi_k^T + \mathbf Q_d
$$

---

## 5. Measurement Updates

### 5.1 Inter-IMU kinematic consistency (fusion of the two gyros)

Run this at the gyro rate, or decimated to about 500 Hz. Substituting the gyro models into the kinematic constraint gives the pseudo-measurement:

$$
\mathbf z_\omega = \tilde{\boldsymbol\omega}_P - \mathbf R_{PH}(\tilde{\boldsymbol\theta})\,\tilde{\boldsymbol\omega}_H - \mathbf J(\tilde{\boldsymbol\theta})\,\dot{\tilde{\boldsymbol\theta}}
= \mathbf b_P - \mathbf R_{PH}\mathbf b_H + \mathbf v_\omega
$$

The vibration content $\boldsymbol\omega_H$ cancels. This measurement is linear in the biases:

$$
\hat{\mathbf z}_\omega = \hat{\mathbf b}_P - \mathbf R_{PH}\hat{\mathbf b}_H,\qquad
\mathbf H_\omega = \begin{bmatrix}\mathbf 0_3 & \mathbf I_3 & -\mathbf R_{PH}\end{bmatrix}
$$

The measurement noise is propagated from all three sensors:

$$
\mathbf R_\omega = \sigma_{gP}^2 f_s\,\mathbf I_3 + \mathbf R_{PH}\,(\sigma_{gH}^2 f_s\,\mathbf I_3)\,\mathbf R_{PH}^T + \mathbf J\,\boldsymbol\Sigma_{\dot\theta}\,\mathbf J^T + \boldsymbol\Sigma_{\text{flex}}
$$

$\boldsymbol\Sigma_{\text{flex}}$ accounts for structural compliance and the damper. The rigid-chain assumption breaks down near the damper resonance, so this term is important.

**Observability.** This update supplies 3 equations for 6 bias unknowns. The biases become separately observable when either:

- $\mathbf R_{PH}$ varies over time (the gimbal articulates), or
- the accelerometer update (5.2) pins the roll and pitch components of $\mathbf b_P$.

Yaw bias remains only weakly observable unless a magnetometer or visual heading source is added.

### 5.2 Payload gravity (tilt) update (100–200 Hz)

$$
\mathbf h_a(\mathbf x) = \hat{\mathbf R}_{WP}^T\mathbf g_W \equiv \hat{\mathbf g}_P
$$

Using $\mathbf R = \hat{\mathbf R}\,\mathrm{Exp}(\delta\boldsymbol\phi)$, we have $\mathbf R^T\mathbf g_W \approx \hat{\mathbf g}_P + [\hat{\mathbf g}_P]_\times\delta\boldsymbol\phi$. Therefore:

$$
\mathbf H_a = \begin{bmatrix}[\hat{\mathbf g}_P]_\times & \mathbf 0_3 & \mathbf 0_3\end{bmatrix}
$$

The accelerometer is corrupted during walking and running, so its noise is set adaptively:

$$
\mathbf R_a = \sigma_a^2\,\mathbf I_3\cdot\Big(1 + \kappa\,\big|\,\|\tilde{\mathbf a}_P\| - g\,\big|^2 + \kappa_\omega\|\hat{\boldsymbol\omega}_P\|^2\Big)
$$

Additionally, gate the update (skip it) if $\big|\|\tilde{\mathbf a}_P\| - g\big| > 0.15g$.

### 5.3 Optional: handle gravity update via the kinematic chain

The handle accelerometer can also observe payload tilt through the encoders:

$$
\mathbf h_{aH} = \mathbf R_{PH}^T\,\hat{\mathbf R}_{WP}^T\mathbf g_W,\qquad
\mathbf H_{aH} = \begin{bmatrix}\mathbf R_{PH}^T[\hat{\mathbf g}_P]_\times & \mathbf 0 & \mathbf 0\end{bmatrix}
$$

Give it a much larger noise ($\mathbf R_{aH} \gg \mathbf R_a$), because the handle carries the raw vibration.

### 5.4 Standard EKF update (common to all measurements)

$$
\mathbf S = \mathbf H\mathbf P^-\mathbf H^T+\mathbf R,\qquad
\mathbf K = \mathbf P^-\mathbf H^T\mathbf S^{-1}
$$

$$
\delta\hat{\mathbf x} = \mathbf K\,(\mathbf z-\mathbf h(\hat{\mathbf x}))
$$

Use the Joseph form for numerical robustness:

$$
\mathbf P^+ = (\mathbf I-\mathbf K\mathbf H)\mathbf P^-(\mathbf I-\mathbf K\mathbf H)^T + \mathbf K\mathbf R\mathbf K^T
$$

**Innovation gating.** Reject the measurement if the Mahalanobis distance exceeds the 99% chi-square threshold for 3 degrees