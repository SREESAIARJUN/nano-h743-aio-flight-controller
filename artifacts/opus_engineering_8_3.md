# 10-State EKF for Gimbal Vibration Isolation: Fusing Payload and Handle IMUs

## 1. Problem Setup

A handheld gimbal has two IMUs:

| Sensor | Location | Role in the filter |
|---|---|---|
| Payload IMU (gyro + accelerometer) | Camera / payload | Primary attitude and rate sensor |
| Handle IMU (gyro) | Handle (vibration source) | Measures the disturbance, mapped through the joint kinematics to the payload frame |
| Joint encoders θ, θ̇ | Motor axes | Provide the handle-to-payload rotation and relative rates |

The goal is a clean estimate of payload attitude **q** and payload angular velocity **ω** for the attitude and rate loops. The handle-rate path gives an independent, low-latency view of the disturbance. Fusing it lets the filter separate true payload motion from payload gyro noise and bias.

**Frames:**
- n: navigation (ENU)
- p: payload body
- h: handle body

**Quaternion convention:** Hamilton, q = [q_w, q_x, q_y, q_z]ᵀ, rotating p → n. C(q) denotes the corresponding direction cosine matrix.

---

## 2. State Vector (10 States)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \boldsymbol{\omega} \\ \mathbf{b}_g \end{bmatrix} \in \mathbb{R}^{10}
\qquad
\begin{aligned}
\mathbf{q} &\in \mathbb{R}^4 &&\text{payload attitude (p→n)}\\
\boldsymbol{\omega} &\in \mathbb{R}^3 &&\text{true payload angular rate, in p}\\
\mathbf{b}_g &\in \mathbb{R}^3 &&\text{payload gyro bias}
\end{aligned}
$$

Two design notes:
- ω is carried as a state rather than used as an input. This lets both gyros act as measurements, which is the fusion mechanism.
- The handle gyro bias is assumed calibrated at startup. If it is not, augment to 13 states with **b_h** (see §8).

---

## 3. Process Model

### 3.1 Continuous-time dynamics

$$
\dot{\mathbf{q}} = \tfrac{1}{2}\,\boldsymbol{\Omega}(\boldsymbol{\omega})\,\mathbf{q}
$$

$$
\dot{\boldsymbol{\omega}} = \mathbf{J}_p^{-1}\big(\boldsymbol{\tau}_m - \boldsymbol{\omega}\times \mathbf{J}_p\boldsymbol{\omega}\big) + \mathbf{w}_\omega
$$

$$
\dot{\mathbf{b}}_g = \mathbf{w}_b
$$

where:
- **J_p** is the payload inertia.
- **τ_m** is the known motor torque (from the current command times the torque constant K_t, mapped to payload axes).
- **w_ω** captures unmodeled disturbance torque transmitted from the handle through friction and cable stiffness.
- **w_b** is the bias random walk.

The skew operator and quaternion rate matrix are:

$$
\boldsymbol{\Omega}(\boldsymbol{\omega}) =
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z\\
\omega_x & 0 & \omega_z & -\omega_y\\
\omega_y & -\omega_z & 0 & \omega_x\\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

If the torque model is poor, set τ_m = 0. This reduces the ω dynamics to a random walk (or a first-order Gauss–Markov process with ω̇ = −ω/τ_c + w_ω).

### 3.2 Jacobian F = ∂f/∂x (10×10)

$$
\mathbf{F} =
\begin{bmatrix}
\tfrac12\boldsymbol{\Omega}(\boldsymbol{\omega}) & \tfrac12\boldsymbol{\Xi}(\mathbf{q}) & \mathbf{0}_{4\times3}\\
\mathbf{0}_{3\times4} & \mathbf{F}_{\omega\omega} & \mathbf{0}_{3\times3}\\
\mathbf{0}_{3\times4} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3}
\end{bmatrix}
$$

The quaternion-rate block and the gyroscopic block are:

$$
\boldsymbol{\Xi}(\mathbf{q}) =
\begin{bmatrix}
-q_x & -q_y & -q_z\\
q_w & -q_z & q_y\\
q_z & q_w & -q_x\\
-q_y & q_x & q_w
\end{bmatrix},
\qquad
\mathbf{F}_{\omega\omega} = \mathbf{J}_p^{-1}\big([\mathbf{J}_p\boldsymbol{\omega}]_\times - [\boldsymbol{\omega}]_\times\mathbf{J}_p\big)
$$

For the Gauss–Markov variant, use **F_ωω** = −(1/τ_c)**I**.

### 3.3 Discretization (step Δt, run at the gyro rate, e.g. 1–2 kHz)

**State propagation:**

$$
\mathbf{q}_{k+1}^- = \Big[\cos\!\big(\tfrac{\|\boldsymbol\omega\|\Delta t}{2}\big)\mathbf{I}_4 + \tfrac{\sin(\|\boldsymbol\omega\|\Delta t/2)}{\|\boldsymbol\omega\|}\boldsymbol{\Omega}(\boldsymbol\omega)\Big]\mathbf{q}_k^+,\quad \text{then normalize}
$$

$$
\boldsymbol{\omega}_{k+1}^- = \boldsymbol{\omega}_k^+ + \Delta t\,\mathbf{J}_p^{-1}(\boldsymbol\tau_m - \boldsymbol\omega_k^+\times\mathbf{J}_p\boldsymbol\omega_k^+),
\qquad
\mathbf{b}_{g,k+1}^- = \mathbf{b}_{g,k}^+
$$

**Covariance propagation:**

$$
\boldsymbol{\Phi}_k \approx \mathbf{I}_{10} + \mathbf{F}\Delta t + \tfrac12(\mathbf{F}\Delta t)^2
$$

$$
\mathbf{P}_{k+1}^- = \boldsymbol{\Phi}_k\mathbf{P}_k^+\boldsymbol{\Phi}_k^\top + \mathbf{Q}_d
$$

$$
\mathbf{Q}_d \approx \mathbf{G}\mathbf{Q}_c\mathbf{G}^\top\Delta t,\qquad
\mathbf{G}=\begin{bmatrix}\mathbf{0}_{4\times3}&\mathbf{0}\\ \mathbf{I}_3&\mathbf{0}\\ \mathbf{0}&\mathbf{I}_3\end{bmatrix},\quad
\mathbf{Q}_c=\mathrm{diag}(\sigma_{\omega}^2\mathbf{I}_3,\ \sigma_{b}^2\mathbf{I}_3)
$$

Add a small εI₄ to the quaternion block to keep P well-conditioned.

---

## 4. Measurement Models

### 4.1 Payload gyro (every sample)

$$
\mathbf{z}_p = \tilde{\boldsymbol\omega}_p = \boldsymbol\omega + \mathbf{b}_g + \mathbf{v}_p,
\qquad
\mathbf{H}_p = [\,\mathbf{0}_{3\times4}\ \ \mathbf{I}_3\ \ \mathbf{I}_3\,],\quad
\mathbf{R}_p = \sigma_{gp}^2\mathbf{I}_3
$$

### 4.2 Handle gyro mapped through gimbal kinematics (every sample)

The payload rate equals the handle rate rotated into the payload frame plus the joint-rate contribution:

$$
\boldsymbol\omega = \mathbf{C}_{ph}(\boldsymbol\theta)\,\boldsymbol\omega_h + \mathbf{S}(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
$$

**Definitions:**
- **C_ph(θ)** is the handle-to-payload rotation from forward kinematics of the yaw–roll–pitch chain.
- **S(θ)** maps the joint rates θ̇ to body rates in p (the joint-axis Jacobian).

**Pseudo-measurement:**

$$
\mathbf{z}_h = \mathbf{C}_{ph}(\boldsymbol\theta)\,(\tilde{\boldsymbol\omega}_h - \hat{\mathbf{b}}_h) + \mathbf{S}(\boldsymbol\theta)\,\dot{\boldsymbol\theta}_{enc}
= \boldsymbol\omega + \mathbf{v}_h
$$

$$
\mathbf{H}_h = [\,\mathbf{0}_{3\times4}\ \ \mathbf{I}_3\ \ \mathbf{0}_{3\times3}\,]
$$

**Noise covariance** (propagated through the mapping):

$$
\mathbf{R}_h = \mathbf{C}_{ph}\,\sigma_{gh}^2\mathbf{I}_3\,\mathbf{C}_{ph}^\top + \mathbf{S}\,\boldsymbol\Sigma_{\dot\theta}\,\mathbf{S}^\top + \mathbf{R}_{flex}
$$

**R_flex** accounts for arm and bracket compliance between the two IMUs. Tune it upward near structural resonances.

**Why this isolates vibration:** **H_p** observes ω + b_g, while **H_h** observes ω alone. Together they make **b_g** observable at high bandwidth, and they average the two independent gyro noise sources on ω.

### 4.3 Payload accelerometer (gravity reference, 100–500 Hz)

$$
\mathbf{z}_a = \tilde{\mathbf f}_p = \mathbf{h}_a(\mathbf q) + \mathbf v_a,\qquad
\mathbf{h}_a(\mathbf q) = g\begin{bmatrix}
2(q_xq_z - q_wq_y)\\
2(q_yq_z + q_wq_x)\\
q_w^2 - q_x^2 - q_y^2 + q_z^2
\end{bmatrix}
$$

$$
\mathbf{H}_a = \Big[\,2g\begin{bmatrix}
-q_y & q_z & -q_w & q_x\\
q_x & q_w & q_z & q_y\\
q_w & -q_x & -q_y & q_z
\end{bmatrix}\ \ \mathbf 0_{3\times3}\ \ \mathbf 0_{3\times3}\,\Big]
$$

**Adaptive noise:** inflate **R_a** when the norm deviates from gravity, to reject operator linear acceleration:

$$
\mathbf R_a = \sigma_a^2\big(1 + k_a\,\big|\,\|\tilde{\mathbf f}_p\| - g\,\big|\big)\mathbf I_3
$$

Yaw is unobservable from these sensors. It is either held by the follow-mode reference or corrected by an optional magnetometer or encoder-yaw pseudo-measurement.

---

## 5. Update Equations (common to each sensor block)

For the measurement set i ∈ {p, h, a}, which may be stacked or applied sequentially:

$$
\mathbf{y} = \mathbf z_i - \mathbf h_i(\hat{\mathbf x}^-)
$$

$$
\mathbf S = \mathbf H_i\mathbf P^-\mathbf H_i^\top + \mathbf R_i
$$

$$
\mathbf K = \mathbf P^-\mathbf H_i^\top\mathbf S^{-1}
$$

$$
\hat{\mathbf x