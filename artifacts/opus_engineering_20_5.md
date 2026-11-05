# 10-State EKF for Gimbal Vibration Isolation: Payload IMU + Handle IMU Fusion

I don't have the context for "Cycle 20" from your project, so this is a self-contained formulation. Adapt the frames, sensor rates and noise values to your hardware.

---

## 1. Problem Setup and Frames

| Frame | Symbol | Description |
|---|---|---|
| Navigation (ENU) | **n** | Gravity-aligned world frame |
| Payload (camera) | **p** | Rigidly attached to the camera IMU |
| Handle | **h** | Rigidly attached to the handle IMU (vibration source) |

The gimbal joint angles θ = [θ₁, θ₂, θ₃]ᵀ are measured by the motor encoders. They give the relative rotation **R**ₕₚ(θ) and the joint-rate Jacobian **J**(θ).

**Goal:** estimate the payload attitude and a clean, bias-free payload angular rate. Use the handle IMU so that handle vibrations become a measured disturbance for feed-forward rejection, rather than an unknown one.

---

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \mathbf{b}_p \\ \mathbf{b}_h \end{bmatrix}
= \big[\,q_w\ q_x\ q_y\ q_z \;\big|\; b_{p,x}\ b_{p,y}\ b_{p,z} \;\big|\; b_{h,x}\ b_{h,y}\ b_{h,z}\,\big]^T \in \mathbb{R}^{10}
$$

- **q**: payload attitude quaternion (Hamilton convention, rotates p → n), 4 states
- **b**ₚ: payload gyro bias (rad/s), 3 states
- **b**ₕ: handle gyro bias (rad/s), 3 states

---

## 3. Sensor Models

**Payload gyro:**
$$\tilde{\boldsymbol\omega}_p = \boldsymbol\omega_p + \mathbf{b}_p + \mathbf{n}_{gp}$$

**Handle gyro:**
$$\tilde{\boldsymbol\omega}_h = \boldsymbol\omega_h + \mathbf{b}_h + \mathbf{n}_{gh}$$

**Payload accelerometer** (specific force):
$$\tilde{\mathbf{a}}_p = \mathbf{R}(\mathbf{q})^T(\mathbf{a}^n - \mathbf{g}^n) + \mathbf{n}_a, \qquad \mathbf{g}^n = [0,\ 0,\ -g]^T$$

**Kinematic coupling through the gimbal** (payload rate equals handle rate plus joint rates):
$$\boldsymbol\omega_p = \mathbf{R}_{ph}(\theta)\,\boldsymbol\omega_h + \mathbf{J}(\theta)\,\dot\theta$$

Bias dynamics are modelled as random walks: $\dot{\mathbf{b}}_p = \mathbf{w}_{bp}$ and $\dot{\mathbf{b}}_h = \mathbf{w}_{bh}$.

---

## 4. Process Model (Prediction)

The payload gyro is used as the control input:
$$\boldsymbol\omega = \tilde{\boldsymbol\omega}_p - \hat{\mathbf{b}}_p$$

### Continuous time
$$
\dot{\mathbf{q}} = \tfrac12\,\boldsymbol\Omega(\boldsymbol\omega)\,\mathbf{q} = \tfrac12\,\boldsymbol\Xi(\mathbf{q})\,\boldsymbol\omega,
\qquad \dot{\mathbf{b}}_p = \mathbf{0},\qquad \dot{\mathbf{b}}_h = \mathbf{0}
$$

where

$$
\boldsymbol\Omega(\boldsymbol\omega)=\begin{bmatrix}0&-\omega_x&-\omega_y&-\omega_z\\ \omega_x&0&\omega_z&-\omega_y\\ \omega_y&-\omega_z&0&\omega_x\\ \omega_z&\omega_y&-\omega_x&0\end{bmatrix},
\qquad
\boldsymbol\Xi(\mathbf{q})=\begin{bmatrix}-q_x&-q_y&-q_z\\ q_w&-q_z&q_y\\ q_z&q_w&-q_x\\ -q_y&q_x&q_w\end{bmatrix}
$$

### Discrete time (sample period Δt, zeroth-order hold on ω)
$$
\hat{\mathbf{q}}_{k+1|k} = \Big[\cos\!\big(\tfrac{\|\boldsymbol\omega\|\Delta t}{2}\big)\mathbf{I}_4 + \frac{\sin(\|\boldsymbol\omega\|\Delta t/2)}{\|\boldsymbol\omega\|}\boldsymbol\Omega(\boldsymbol\omega)\Big]\hat{\mathbf{q}}_{k|k},
\qquad \hat{\mathbf{b}}_{k+1|k} = \hat{\mathbf{b}}_{k|k}
$$

Normalize **q** after every step.

### State-transition Jacobian (10×10)
$$
\mathbf{F}_k = \begin{bmatrix}
\mathbf{A}_k & -\tfrac{\Delta t}{2}\boldsymbol\Xi(\hat{\mathbf q}_k) & \mathbf{0}_{4\times3}\\
\mathbf{0}_{3\times4} & \mathbf{I}_3 & \mathbf{0}_{3\times 3}\\
\mathbf{0}_{3\times4} & \mathbf{0}_{3\times3} & \mathbf{I}_3
\end{bmatrix},\qquad
\mathbf{A}_k = \exp\!\big(\tfrac12\boldsymbol\Omega(\boldsymbol\omega)\Delta t\big)
$$

### Process noise
$$
\mathbf{Q}_k = \begin{bmatrix}
\tfrac{\Delta t^2}{4}\boldsymbol\Xi\,(\sigma_{gp}^2\mathbf I_3)\,\boldsymbol\Xi^T & \mathbf 0 & \mathbf 0\\
\mathbf 0 & \sigma_{bp}^2\Delta t\,\mathbf I_3 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \sigma_{bh}^2\Delta t\,\mathbf I_3
\end{bmatrix}
$$

### Covariance prediction
$$\mathbf{P}_{k+1|k} = \mathbf{F}_k\mathbf{P}_{k|k}\mathbf{F}_k^T + \mathbf{Q}_k$$

---

## 5. Measurement Models (Update)

### 5a. Gravity reference (payload accelerometer)
$$
\mathbf{h}_a(\mathbf{x}) = \mathbf{R}(\mathbf q)^T\begin{bmatrix}0\\0\\g\end{bmatrix}
= g\begin{bmatrix}2(q_xq_z - q_wq_y)\\ 2(q_yq_z + q_wq_x)\\ q_w^2 - q_x^2 - q_y^2 + q_z^2\end{bmatrix}
$$

$$
\mathbf{H}_a = \Big[\;2g\begin{bmatrix}-q_y & q_z & -q_w & q_x\\ q_x & q_w & q_z & q_y\\ q_w & -q_x & -q_y & q_z\end{bmatrix}\;\Big|\;\mathbf 0_{3\times3}\;\Big|\;\mathbf 0_{3\times3}\Big]
$$

Use adaptive measurement noise to reject handle-induced linear vibration:
$$
\mathbf R_a = \sigma_a^2\mathbf I_3\left(1 + \kappa\,\big|\,\|\tilde{\mathbf a}_p\| - g\,\big|\right)^2
$$

Alternatively, gate (skip) the update when $\big|\|\tilde{\mathbf a}_p\| - g\big| > \epsilon$.

### 5b. Handle-gyro rate consistency (the cross-IMU fusion term)

The handle gyro is predicted from the payload gyro, the encoders and both biases:
$$
\mathbf{h}_h(\mathbf x) = \mathbf R_{hp}(\theta)\big[(\tilde{\boldsymbol\omega}_p - \mathbf b_p) - \mathbf J(\theta)\dot\theta\big] + \mathbf b_h
$$

$$
\mathbf H_h = \big[\;\mathbf 0_{3\times4}\;\big|\;-\mathbf R_{hp}(\theta)\;\big|\;\mathbf I_3\;\big]
$$

The effective noise includes both gyros plus encoder-rate noise:
$$
\mathbf R_h = \sigma_{gh}^2\mathbf I_3 + \mathbf R_{hp}\,\sigma_{gp}^2\,\mathbf R_{hp}^T + \mathbf R_{hp}\mathbf J\,\sigma_{\dot\theta}^2\,\mathbf J^T\mathbf R_{hp}^T
$$

This measurement makes the *relative* bias $\mathbf b_h - \mathbf R_{hp}\mathbf b_p$ observable. The accelerometer anchors the roll/pitch part of $\mathbf b_p$. As the gimbal articulates, $\mathbf R_{hp}(\theta)$ changes, which improves observability of each individual bias.

**Caveat:** $\tilde{\boldsymbol\omega}_p$ appears in both the propagation and this measurement, which correlates process and measurement noise. In practice this is handled by the inflated $\mathbf R_h$ above. For a rigorous treatment, use cross-covariance $\mathbf S$ terms or a delayed-state formulation.

### Standard EKF update (for either measurement)
$$
\mathbf y = \mathbf z - \mathbf h(\hat{\mathbf x}_{k+1|k}),\qquad
\mathbf S = \mathbf H\mathbf P\mathbf H^T + \mathbf R,\qquad
\mathbf K = \mathbf P\mathbf H^T\mathbf S^{-1}
$$

$$
\hat{\mathbf x}_{k+1|k+1} = \hat{\mathbf x}_{k+1|k} + \mathbf K\mathbf y,\qquad
\mathbf P_{k+1|k+1} = (\mathbf I - \mathbf K\mathbf H)\mathbf P(\mathbf I - \mathbf K\mathbf H)^T + \mathbf K\mathbf R\mathbf K^T \quad\text{(Joseph form)}
$$

After each update, renormalize **q**. To remove the radial (non-physical) covariance direction, also project **P**:
$$
\mathbf P_{qq} \leftarrow \mathbf\Pi\,\mathbf P_{qq}\,\mathbf\Pi^T,\qquad \mathbf\Pi = \mathbf I_4 - \hat{\mathbf q}\hat{\mathbf q}^T
$$

---

## 6. Outputs for Vibration Isolation

| Output | Expression | Use |
|---|---|---|
| Clean payload rate | $\hat{\boldsymbol\omega}_p = \tilde{\boldsymbol\omega}_p - \hat{\mathbf b}_p$ | Inner rate-loop feedback |
| Payload attitude | $\hat{\mathbf q}$ | Outer attitude/stabilization loop |
| Handle disturbance rate | $\hat{\boldsymbol\omega}_h = \tilde{\boldsymbol\omega}_h - \hat{\mathbf b}_h$ | Feed-forward |

**Feed-forward joint-rate command** (cancels the handle motion before it reaches the camera):
$$
\dot\theta_{ff} = -\,\mathbf J(\theta)^{-1}\,\mathbf R_{ph}(\the