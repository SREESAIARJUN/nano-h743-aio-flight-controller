# 10-State EKF for Payload/Handle IMU Fusion in Gimbal Vibration Isolation

## 1. Problem Setup

**Frames**
- **W**: world (NED or ENU, z-axis aligned with gravity)
- **P**: camera payload body, where IMU_P is mounted
- **H**: handle body, where IMU_H is mounted
- **θ = [θ₁ θ₂ θ₃]ᵀ**: gimbal joint angles from the motor encoders. θ̇ is their rates.

**Goal.** Estimate the payload attitude and the *true* payload angular rate with low latency and low noise. The handle IMU senses the vibration *before* it propagates through the gimbal. This gives the controller a feedforward disturbance path and a redundant rate measurement.

---

## 2. State Vector (n = 10)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \boldsymbol{\omega} \\ \mathbf{b}_p \end{bmatrix}
\in \mathbb{R}^{10}
$$

| States | Symbol | Description |
|---|---|---|
| 1–4 | **q** = [q_w q_x q_y q_z]ᵀ | Payload attitude quaternion (P→W, Hamilton convention) |
| 5–7 | **ω** | True payload angular rate, expressed in P (rad/s) |
| 8–10 | **b_p** | Payload gyro bias (rad/s) |

The handle gyro bias is assumed calibrated at startup. Section 10 shows how to extend to 13 states if it must be estimated online.

---

## 3. Process Model

### Continuous time

$$
\dot{\mathbf{q}} = \tfrac{1}{2}\,\Omega(\boldsymbol{\omega})\,\mathbf{q}
$$

$$
\dot{\boldsymbol{\omega}} = \mathbf{J}_p^{-1}\big(\boldsymbol{\tau}_m - \boldsymbol{\omega}\times \mathbf{J}_p\boldsymbol{\omega}\big) + \mathbf{w}_\omega
\quad\text{(or } \dot{\boldsymbol{\omega}}=\mathbf{w}_\omega \text{ random walk if torque is unmodeled)}
$$

$$
\dot{\mathbf{b}}_p = \mathbf{w}_b
$$

where

$$
\Omega(\boldsymbol{\omega}) =
\begin{bmatrix}
0 & -\boldsymbol{\omega}^T \\
\boldsymbol{\omega} & -[\boldsymbol{\omega}]_\times
\end{bmatrix},
\qquad
[\boldsymbol{\omega}]_\times =
\begin{bmatrix}
0 & -\omega_z & \omega_y\\
\omega_z & 0 & -\omega_x\\
-\omega_y & \omega_x & 0
\end{bmatrix}
$$

### Discrete time (step Δt)

$$
\mathbf{q}_{k+1} = \Big[\cos\!\big(\tfrac{\|\boldsymbol{\omega}_k\|\Delta t}{2}\big)\mathbf{I}_4 + \frac{\sin\!\big(\tfrac{\|\boldsymbol{\omega}_k\|\Delta t}{2}\big)}{\|\boldsymbol{\omega}_k\|}\,\Omega(\boldsymbol{\omega}_k)\Big]\mathbf{q}_k \;\equiv\; \Phi_q(\boldsymbol{\omega}_k)\,\mathbf{q}_k
$$

$$
\boldsymbol{\omega}_{k+1} = \boldsymbol{\omega}_k + \Delta t\,\mathbf{J}_p^{-1}\big(\boldsymbol{\tau}_{m,k} - \boldsymbol{\omega}_k\times\mathbf{J}_p\boldsymbol{\omega}_k\big)
$$

$$
\mathbf{b}_{p,k+1} = \mathbf{b}_{p,k}
$$

Use the small-angle limit Φ_q ≈ I + ½Ω Δt when ‖ω‖ → 0.

### State transition Jacobian

$$
\mathbf{F}_k = \frac{\partial f}{\partial \mathbf{x}}\Big|_{\hat{\mathbf{x}}_{k|k}} =
\begin{bmatrix}
\Phi_q(\hat{\boldsymbol\omega}) & \tfrac{\Delta t}{2}\,\Xi(\hat{\mathbf q}) & \mathbf{0}_{4\times3}\\
\mathbf{0}_{3\times4} & \mathbf{I}_3 + \Delta t\,\mathbf{A}_\omega & \mathbf{0}_{3}\\
\mathbf{0}_{3\times4} & \mathbf{0}_3 & \mathbf{I}_3
\end{bmatrix}
$$

The matrix Ξ satisfies Ω(ω)q = Ξ(q)ω:

$$
\Xi(\mathbf q)=\begin{bmatrix}-q_x & -q_y & -q_z\\ q_w & -q_z & q_y\\ q_z & q_w & -q_x\\ -q_y & q_x & q_w\end{bmatrix}
$$

The gyroscopic term linearizes to

$$
\mathbf A_\omega = -\mathbf J_p^{-1}\big([\boldsymbol\omega]_\times\mathbf J_p - [\mathbf J_p\boldsymbol\omega]_\times\big)
$$

which is 0 under the random-walk model.

### Process noise

$$
\mathbf{Q}_k = \operatorname{blkdiag}\!\Big(\mathbf{Q}_q,\; \sigma_{\dot\omega}^2\Delta t\,\mathbf I_3,\; \sigma_{b}^2\Delta t\,\mathbf I_3\Big),
\qquad \mathbf Q_q \approx \tfrac{\Delta t^2}{4}\,\Xi\,(\sigma_{\dot\omega}^2\Delta t\,\mathbf I_3)\,\Xi^T + \epsilon\mathbf I_4
$$

**Tuning:** σ_ω̇ sets the tracking bandwidth of the rate state. Set it high enough to pass the vibration band you intend to reject (typically 5–150 Hz on handheld rigs).

---

## 4. Measurement Models

Process the measurements sequentially, each at its own rate.

### (a) Payload gyro

$$
\mathbf z_p = \boldsymbol\omega + \mathbf b_p + \mathbf v_p,
\qquad
\mathbf H_p = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf I_3\end{bmatrix},
\quad \mathbf R_p = \sigma_{gp}^2\mathbf I_3
$$

### (b) Handle gyro, mapped through the gimbal kinematics

The payload rate equals the handle rate rotated into P, plus the joint-induced rate:

$$
\boldsymbol\omega = \mathbf C_{PH}(\boldsymbol\theta)\,\boldsymbol\omega_H + \mathbf J_\theta(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
$$

Inverting this gives the measurement in the handle frame:

$$
\mathbf z_h = \mathbf C_{HP}(\boldsymbol\theta)\big(\boldsymbol\omega - \mathbf J_\theta(\boldsymbol\theta)\dot{\boldsymbol\theta}\big) + \mathbf v_h,
\qquad
\mathbf H_h = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf C_{HP}(\boldsymbol\theta) & \mathbf 0_3\end{bmatrix}
$$

Here C_HP = C_PHᵀ is the product of the axis rotations (e.g., yaw·roll·pitch for your motor order). The columns of J_θ are the joint axes expressed in P.

Encoder-rate noise must be included in the measurement covariance:

$$
\mathbf R_h = \sigma_{gh}^2\mathbf I_3 + \mathbf C_{HP}\mathbf J_\theta\,\boldsymbol\Sigma_{\dot\theta}\,\mathbf J_\theta^T\mathbf C_{HP}^T
$$

### (c) Payload accelerometer (gravity reference, attitude observability)

$$
\mathbf z_a = \mathbf C(\mathbf q)^T\begin{bmatrix}0\\0\\g\end{bmatrix} + \mathbf v_a
= 2g\begin{bmatrix} q_xq_z - q_wq_y\\ q_yq_z + q_wq_x\\ \tfrac12(q_w^2-q_x^2-q_y^2+q_z^2)\end{bmatrix} + \mathbf v_a
$$

$$
\mathbf H_a = \begin{bmatrix} 2g\begin{bmatrix}-q_y & q_z & -q_w & q_x\\ q_x & q_w & q_z & q_y\\ q_w & -q_x & -q_y & q_z\end{bmatrix} & \mathbf 0_3 & \mathbf 0_3\end{bmatrix}
$$

Make R_a adaptive so that linear acceleration from walking or vibration does not corrupt attitude:

$$
\mathbf R_a = \sigma_a^2\big(1 + \kappa\,\big|\|\mathbf z_a\| - g\big|\big)\mathbf I_3
$$

Alternatively, gate the update with a χ² test on the innovation.

### (d) Quaternion norm pseudo-measurement (optional)

$$
z_n = 1 = \mathbf q^T\mathbf q,
\qquad
\mathbf H_n = \begin{bmatrix}2\hat{\mathbf q}^T & \mathbf 0_{1\times6}\end{bmatrix},
\quad R_n \approx 10^{-6}
$$

---

## 5. EKF Recursion

**Predict**

$$
\hat{\mathbf x}_{k+1|k} = f(\hat{\mathbf x}_{k|k}, \boldsymbol\tau_{m,k})
$$

$$
\mathbf P_{k+1|k} = \mathbf F_k\mathbf P_{k|k}\mathbf F_k^T + \mathbf Q_k
$$

**Update.** For each available measurement i ∈ {p, h, a, n}:

$$
\mathbf y_i = \mathbf z_i - h_i(\hat{\mathbf x}_{k+1|k})
$$

$$
\mathbf S_i = \mathbf H_i\mathbf P_{k+1|k}\mathbf H_i^T + \mathbf R_i
$$

$$
\mathbf K_i = \mathbf P_{k+1|k}\mathbf H_i^T\mathbf S_i^{-1}
$$

$$
\hat{\mathbf x}_{k+1|k+1} = \hat{\mathbf x}_{k+1|k} + \mathbf K_i\mathbf y_i
$$

$$
\mathbf P_{k+1|k+1} = (\mathbf I-\mathbf K_i\mathbf H_i)\mathbf P_{k+1|k}(\mathbf I-\mathbf K_i\mathbf H_