# 10-State EKF for Payload/Handle Gyro Fusion in Gimbal Vibration Isolation

I don't have the specifics of your "Cycle 12" program, so this formulation uses a standard 3-axis handheld or stabilized gimbal architecture:

- **Payload IMU**: gyro and accelerometer on the camera, after the gimbal joints.
- **Handle IMU**: gyro on the base or handle, before the joints.
- **Joint encoders**: measure the joint angles θ and the joint rates θ̇.

If your hardware or state choice differs, the structure below adapts directly.

---

## 1. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \boldsymbol{\omega} \\ \mathbf{b}_g \end{bmatrix} \in \mathbb{R}^{10}
$$

| States | Symbol | Meaning |
|---|---|---|
| 1–4 | $\mathbf{q} = [q_w, q_x, q_y, q_z]^T$ | Payload attitude quaternion (navigation frame → payload body frame) |
| 5–7 | $\boldsymbol{\omega}$ | True payload angular velocity (body frame) |
| 8–10 | $\mathbf{b}_g$ | Payload gyro bias |

Carrying $\boldsymbol{\omega}$ as a state matters for vibration isolation. The filter then blends the payload gyro, the handle-derived rate and the dynamics model, rather than passing raw gyro noise straight to the stabilization loop.

---

## 2. Process Model

### Continuous-time dynamics

$$
\dot{\mathbf{q}} = \tfrac{1}{2}\,\mathbf{q}\otimes\begin{bmatrix}0\\ \boldsymbol{\omega}\end{bmatrix} = \tfrac12\,\Xi(\mathbf{q})\,\boldsymbol{\omega}
$$

$$
\dot{\boldsymbol{\omega}} = \mathbf{J}_p^{-1}\big(\boldsymbol{\tau}_m - \boldsymbol{\omega}\times \mathbf{J}_p\boldsymbol{\omega}\big) + \mathbf{w}_\omega
$$

$$
\dot{\mathbf{b}}_g = \mathbf{w}_b
$$

Where:

- $\mathbf{J}_p$ is the payload inertia.
- $\boldsymbol{\tau}_m$ is the commanded motor torque, mapped to the payload frame.
- $\mathbf{w}_\omega$ absorbs unmodeled torques: friction, cable drag and residual vibration coupling.
- $\Xi(\mathbf{q})$ is defined as:

$$
\Xi(\mathbf{q}) = \begin{bmatrix} -q_x & -q_y & -q_z \\ q_w & -q_z & q_y \\ q_z & q_w & -q_x \\ -q_y & q_x & q_w \end{bmatrix}
$$

If no torque model is available, set $\boldsymbol{\tau}_m = 0$ and the inertia term to zero. $\boldsymbol{\omega}$ then becomes a random walk driven by $\mathbf{w}_\omega$.

### Discrete prediction (step $\Delta t$)

$$
\hat{\mathbf{q}}_{k|k-1} = \hat{\mathbf{q}}_{k-1}\otimes
\begin{bmatrix}\cos(\tfrac{\|\boldsymbol\omega\|\Delta t}{2})\\[2pt] \frac{\boldsymbol\omega}{\|\boldsymbol\omega\|}\sin(\tfrac{\|\boldsymbol\omega\|\Delta t}{2})\end{bmatrix},
\qquad \text{then normalize } \hat{\mathbf q}
$$

$$
\hat{\boldsymbol\omega}_{k|k-1} = \hat{\boldsymbol\omega}_{k-1} + \Delta t\,\mathbf{J}_p^{-1}\big(\boldsymbol\tau_m - \hat{\boldsymbol\omega}\times\mathbf{J}_p\hat{\boldsymbol\omega}\big)
$$

$$
\hat{\mathbf b}_{g,k|k-1} = \hat{\mathbf b}_{g,k-1}
$$

$$
\mathbf P_{k|k-1} = \mathbf F_k \mathbf P_{k-1}\mathbf F_k^T + \mathbf Q_k
$$

### State-transition Jacobian

$$
\mathbf F_k = \begin{bmatrix}
\mathbf I_4 + \tfrac{\Delta t}{2}\Omega(\boldsymbol\omega) & \tfrac{\Delta t}{2}\Xi(\mathbf q) & \mathbf 0_{4\times3}\\
\mathbf 0_{3\times4} & \mathbf I_3 + \Delta t\,\mathbf J_p^{-1}\big([\mathbf J_p\boldsymbol\omega]_\times - [\boldsymbol\omega]_\times \mathbf J_p\big) & \mathbf 0_{3}\\
\mathbf 0_{3\times4} & \mathbf 0_3 & \mathbf I_3
\end{bmatrix}
$$

with

$$
\Omega(\boldsymbol\omega)=\begin{bmatrix}0&-\omega_x&-\omega_y&-\omega_z\\ \omega_x&0&\omega_z&-\omega_y\\ \omega_y&-\omega_z&0&\omega_x\\ \omega_z&\omega_y&-\omega_x&0\end{bmatrix}
$$

### Process noise

$$
\mathbf Q_k = \mathrm{diag}\big(\sigma_q^2\mathbf I_4,\; \sigma_{\dot\omega}^2\Delta t\,\mathbf I_3,\; \sigma_{b}^2\Delta t\,\mathbf I_3\big)
$$

The parameter $\sigma_{\dot\omega}$ sets how quickly the filter tracks real motion versus smoothing vibration.

---

## 3. Measurement Models

Measurements are applied sequentially, each at its own sample rate.

### (a) Payload gyro: direct but noisy and biased

$$
\mathbf z_p = \boldsymbol\omega + \mathbf b_g + \mathbf v_p,\qquad
\mathbf H_p = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf I_3\end{bmatrix},\qquad
\mathbf R_p = \sigma_{gp}^2\mathbf I_3
$$

### (b) Handle IMU propagated through the gimbal kinematics: the fusion path

The payload rate equals the handle rate rotated into the payload frame, plus the joint-induced rates:

$$
\mathbf z_h = \mathbf C_{ph}(\boldsymbol\theta)\,\boldsymbol\omega_h + \mathbf J_\theta(\boldsymbol\theta)\,\dot{\boldsymbol\theta} = \boldsymbol\omega + \mathbf v_h
$$

$$
\mathbf H_h = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf 0_3\end{bmatrix}
$$

The measurement noise is propagated from the handle gyro and encoder noise:

$$
\mathbf R_h = \mathbf C_{ph}\,\Sigma_{gh}\,\mathbf C_{ph}^T + \mathbf J_\theta\,\Sigma_{\dot\theta}\,\mathbf J_\theta^T
$$

The terms in these equations are:

- $\mathbf C_{ph}(\boldsymbol\theta)$ is the rotation from the handle frame to the payload frame. It is the product of the per-joint rotations, for example yaw → roll → pitch.
- $\mathbf J_\theta(\boldsymbol\theta)$ maps joint rates into payload-frame angular velocity. Its columns are the joint axes expressed in the payload frame.
- $\boldsymbol\omega_h$ should be the handle gyro with a pre-calibrated bias removed. If handle bias drift matters, the state can be extended to 13 states with $\mathbf b_h$.

### (c) Payload accelerometer: gravity reference that bounds attitude drift

$$
\mathbf z_a = \mathbf R(\mathbf q)^T\mathbf g + \mathbf v_a
= g\begin{bmatrix}2(q_xq_z - q_wq_y)\\ 2(q_yq_z + q_wq_x)\\ q_w^2-q_x^2-q_y^2+q_z^2\end{bmatrix} + \mathbf v_a
$$

$$
\mathbf H_a = \begin{bmatrix} 2g\begin{bmatrix}-q_y & q_z & -q_w & q_x\\ q_x & q_w & q_z & q_y\\ q_w & -q_x & -q_y & q_z\end{bmatrix} & \mathbf 0_{3\times6}\end{bmatrix}
$$

Use adaptive noise to de-weight this update during linear acceleration:

$$
\mathbf R_a = \big(\sigma_a^2 + k\,\big|\,\|\mathbf a\| - g\,\big|\big)\mathbf I_3
$$

---

## 4. Update Equations (for each measurement $i \in \{p, h, a\}$)

**Innovation**

$$
\mathbf y = \mathbf z_i - h_i(\hat{\mathbf x}_{k|k-1})
$$

**Innovation covariance**

$$
\mathbf S = \mathbf H_i\mathbf P_{k|k-1}\mathbf H_i^T + \mathbf R_i
$$

**Gain**

$$
\mathbf K = \mathbf P_{k|k-1}\mathbf H_i^T\mathbf S^{-1}
$$

**State update**

$$
\hat{\mathbf x}_k = \hat{\mathbf x}_{k|k-1} + \mathbf K\mathbf y,\qquad \hat{\mathbf q}\leftarrow \hat{\mathbf q}/\|\hat{\mathbf q}\|
$$

**Covariance update (Joseph form)**

$$
\mathbf P_k = (\mathbf I - \mathbf K\mathbf H_i)\mathbf P_{k|k-1}(\mathbf I-\mathbf K\mathbf H_i)^T + \mathbf K\mathbf R_i\mathbf K^T
$$

**Innovation gating:** reject the measurement if $\mathbf y^T\mathbf S^{-1}\mathbf y > \chi^2_{3,0.997}\approx 14.2$. This protects against shock spikes and gyro saturation.

---

## 5. Using the Estimate for Vibration Isolation

**Rate loop feedback.** Feed back $\hat{\boldsymbol\omega}$, the filtered payload rate, instead of the raw gyro signal. This gives lower noise and no bias.

**Feedforward cancellation.** The handle IMU senses base vibration before it propagates through the joints. Command each motor with a rate that cancels the handle motion's contribution in the joint space:

$$
\dot{\boldsymbol\theta}_{ff} = -\mathbf J_\theta^{-1}\,\mathbf C_{ph}\,\boldsymbol\omega_h
$$

**Disturbance monitoring.** The residual $\mathbf y_h$ indicates mechanical coupling, for example a loose mount or a damper resonance. Spectral analysis of this residual helps tune isolators.

---

## 6. Practical Notes

- **Quaternion covariance.** The 4×4 quaternion block of P is rank-deficient. For best numerical behavior, consider an error-state (multiplicative) EKF: keep a 10-element nominal state but a 9×9 error covariance, using $\delta\boldsymbol\theta$ in place of $\delta\mathbf q$.
- **Time synchronization.** The two IMUs must be time-aligned to well under one vibration period. A 1 ms skew at 100 Hz already causes a 36° phase error. Use hardware sync or timestamp-based interpolation.
- **Sample rates.** Typical rates are 1–4 kHz for prediction and both gyro updates, and 100–500 Hz for the accelerometer update.
- **Tuning order:**
  1. Set the R values from the gyro datasheet noise densities multiplied by √bandwidth.
  2. Raise $\sigma_{\dot\omega}$ until the tracking lag during intentional pans is acceptable.
  3. Set $\sigma_b$ from the gyro's Allan-variance bias instability.