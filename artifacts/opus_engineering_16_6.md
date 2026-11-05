# 10-State EKF for Payload/Handle Gyro Fusion (Gimbal Vibration Isolation)

## 1. Frames and Sensors

| Symbol | Meaning |
|---|---|
| **W** | World/navigation frame (z-up, gravity **g**_W = [0, 0, g]ᵀ) |
| **P** | Payload (camera) body frame. Payload IMU is mounted here. |
| **H** | Handle frame. Handle IMU is mounted here. |
| **θ** = [θ₁, θ₂, θ₃] | Gimbal joint angles from encoders (yaw–roll–pitch chain) |
| R_PH(θ) | Rotation from handle frame to payload frame via the joint chain |

**Objective.** Estimate the payload attitude and a clean payload angular rate for the stabilization loop. Two sensor paths are fused:

- **Payload gyro:** a direct measurement, but biased and noisy.
- **Handle gyro + encoder kinematics:** an independent route to the payload rate. It sees the handle vibration before it propagates through the motors, so it is effectively a feedforward disturbance measurement.

---

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \boldsymbol{\omega} \\ \mathbf{b} \end{bmatrix}
= \begin{bmatrix} q_w & q_x & q_y & q_z & \omega_x & \omega_y & \omega_z & b_x & b_y & b_z \end{bmatrix}^T
$$

- **q**: unit quaternion, payload attitude (P→W), Hamilton convention (4 states)
- **ω**: true payload angular velocity expressed in P (3 states)
- **b**: payload gyro bias (3 states)

The handle gyro bias is assumed pre-calibrated at startup. Section 9 shows how to estimate it online instead, by expanding to 13 states.

---

## 3. Continuous-Time Process Model

$$
\dot{\mathbf{q}} = \tfrac{1}{2}\,\Omega(\boldsymbol{\omega})\,\mathbf{q} = \tfrac{1}{2}\,\Xi(\mathbf{q})\,\boldsymbol{\omega}
$$

$$
\dot{\boldsymbol{\omega}} = \boldsymbol{\alpha}_{u} + \mathbf{w}_\omega, \qquad
\dot{\mathbf{b}} = \mathbf{w}_b
$$

Here **α**_u is an optional known angular-acceleration input (for example J⁻¹τ_motor from the motor current model). Set it to 0 to get a pure random-walk rate model.

The two quaternion-rate matrices are:

$$
\Omega(\boldsymbol\omega)=\begin{bmatrix}
0&-\omega_x&-\omega_y&-\omega_z\\
\omega_x&0&\omega_z&-\omega_y\\
\omega_y&-\omega_z&0&\omega_x\\
\omega_z&\omega_y&-\omega_x&0\end{bmatrix},
\qquad
\Xi(\mathbf q)=\begin{bmatrix}
-q_x&-q_y&-q_z\\
q_w&-q_z&q_y\\
q_z&q_w&-q_x\\
-q_y&q_x&q_w\end{bmatrix}
$$

---

## 4. Discrete Prediction (step Δt)

**Quaternion propagation (exact for constant ω over the step):**

$$
\mathbf{q}_{k+1|k} = \Big[\cos\!\big(\tfrac{\|\boldsymbol\omega\|\Delta t}{2}\big)\mathbf{I}_4 + \tfrac{\sin(\|\boldsymbol\omega\|\Delta t/2)}{\|\boldsymbol\omega\|}\,\Omega(\boldsymbol\omega_k)\Big]\mathbf{q}_{k|k} \;\equiv\; \Phi_q\,\mathbf{q}_{k|k}
$$

When ‖ω‖ is near 0, use the limit Φ_q ≈ I₄ + ½Ω(ω)Δt.

**Rate and bias propagation:**

$$
\boldsymbol\omega_{k+1|k} = \boldsymbol\omega_{k|k} + \boldsymbol\alpha_u\Delta t, \qquad
\mathbf b_{k+1|k} = \mathbf b_{k|k}
$$

**Then renormalize the quaternion:** q ← q / ‖q‖.

**State transition Jacobian F (10×10):**

$$
\mathbf F_k=\begin{bmatrix}
\Phi_q & \tfrac{\Delta t}{2}\,\Xi(\mathbf q_k) & \mathbf 0_{4\times3}\\
\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf 0_3\\
\mathbf 0_{3\times4} & \mathbf 0_3 & \mathbf I_3
\end{bmatrix}
$$

**Covariance propagation:**

$$
\mathbf P_{k+1|k} = \mathbf F_k\mathbf P_{k|k}\mathbf F_k^T + \mathbf Q_k
$$

$$
\mathbf Q_k = \operatorname{diag}\!\big(\sigma_q^2\Delta t\,\mathbf I_4,\;\; \sigma_{\alpha}^2\Delta t\,\mathbf I_3,\;\; \sigma_{b}^2\Delta t\,\mathbf I_3\big)
$$

- σ_α is the angular-acceleration spectral density. It is the main tuning knob for the bandwidth of the ω estimate.
- σ_q is a small regularizer for numerical stability.

---

## 5. Measurement Models

### 5a. Payload gyro (runs at IMU rate, e.g. 1–4 kHz)

$$
\mathbf z_1 = \tilde{\boldsymbol\omega}_P = \boldsymbol\omega + \mathbf b + \mathbf v_1,\qquad
\mathbf H_1 = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf I_3\end{bmatrix},\qquad
\mathbf R_1 = \sigma_{gP}^2\mathbf I_3
$$

### 5b. Handle gyro mapped through the gimbal kinematics

This measurement carries the vibration-isolation information. The payload rate equals the handle rate rotated into P, plus the joint rates:

$$
\mathbf z_2 = R_{PH}(\boldsymbol\theta)\,\tilde{\boldsymbol\omega}_H + \mathbf J(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
= \boldsymbol\omega + \mathbf v_2
$$

- J(θ) maps the encoder joint rates into P-frame angular velocity (columns are the joint axes expressed in P).
- θ̇ comes from differentiated or filtered encoder readings.

$$
\mathbf H_2 = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf 0_3\end{bmatrix}
$$

$$
\mathbf R_2 = \sigma_{gH}^2\,\mathbf I_3 \;+\; \mathbf J\,\Sigma_{\dot\theta}\,\mathbf J^T \;+\; \mathbf R_{\text{flex}}
$$

R_flex covers mounting and structural flex between the handle IMU and the joint chain. Inflate it near the structural resonance band.

> **Why this works:** z₁ and z₂ observe ω with different bias structure. Their difference makes **b** observable on all three axes, even while the gimbal is moving. Gravity alone cannot provide this for yaw.

### 5c. Payload accelerometer (gravity reference, e.g. 100–500 Hz)

$$
\mathbf z_3 = \tilde{\mathbf a}_P \approx C(\mathbf q)^T\mathbf g_W + \mathbf v_3
$$

C(q)ᵀ rotates world vectors into the payload frame P. The predicted measurement is:

$$
\mathbf h_3(\mathbf q) = g\begin{bmatrix}
2(q_xq_z - q_wq_y)\\
2(q_yq_z + q_wq_x)\\
q_w^2 - q_x^2 - q_y^2 + q_z^2
\end{bmatrix}
$$

$$
\mathbf H_3 = \begin{bmatrix}\dfrac{\partial\mathbf h_3}{\partial\mathbf q} & \mathbf 0_3 & \mathbf 0_3\end{bmatrix},\qquad
\frac{\partial\mathbf h_3}{\partial\mathbf q}= 2g\begin{bmatrix}
-q_y & q_z & -q_w & q_x\\
q_x & q_w & q_z & q_y\\
q_w & -q_x & -q_y & q_z\end{bmatrix}
$$

**Adaptive noise.** Linear acceleration and vibration corrupt this measurement, so scale its noise:

$$
\mathbf R_3 = \sigma_a^2\big(1 + k_a\,\big|\|\tilde{\mathbf a}_P\| - g\big|\big)\mathbf I_3
$$

Alternatively, gate (skip) the update entirely when the deviation from g exceeds a threshold.

---

## 6. Update Step (each available measurement i)

$$
\mathbf y = \mathbf z_i - \mathbf h_i(\mathbf x_{k+1|k})
$$

$$
\mathbf S = \mathbf H_i\mathbf P_{k+1|k}\mathbf H_i^T + \mathbf R_i
$$

$$
\mathbf K = \mathbf P_{k+1|k}\mathbf H_i^T\mathbf S^{-1}
$$

$$
\mathbf x_{k+1|k+1} = \mathbf x_{k+1|k} + \mathbf K\mathbf y \quad\text{(then renormalize q)}
$$

$$
\mathbf P_{k+1|k+1} = (\mathbf I-\mathbf K\mathbf H_i)\mathbf P_{k+1|k}(\mathbf I-\mathbf K\mathbf H_i)^T + \mathbf K\mathbf R_i\mathbf K^T \quad\text{(Joseph form)}
$$

**Implementation notes:**

- Apply the updates sequentially, as each sensor's data arrives.
- Reject outliers with a χ² gate on yᵀS⁻¹y.
- Time-align the handle IMU, payload IMU, and encoder samples. Any skew shows up directly as phase error in the vibration path.

---

## 7. Use in the Vibration Isolation Loop

- **Rate loop feedback:** ω̂ (bias-free, low-lag) replaces the raw gyro reading.
- **Attitude loop:** use the error quaternion q_ref⁻¹ ⊗ q̂.
- **Feedforward:** the handle-derived disturbance rate R_PH ω̃_H gives a command that cancels handle motion before it reaches the payload:

$$
\tau_{ff} = -K_{ff}\,\mathbf J^{-1}\big(R_{PH}\tilde{\boldsymbol\omega}_H\big)
$$

---

## 8. Tuning Guidance

| Parameter | Effect |
|---|---|
| σ_α ↑ | Faster ω tracking, less smoothing. Set the ω estimate bandw