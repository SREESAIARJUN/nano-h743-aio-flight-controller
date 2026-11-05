# 10-State EKF: Payload/Handle IMU Fusion for Vibration Isolation

I don't have your project's Cycle 11 documents, so this formulation uses a standard 3-axis handheld gimbal architecture. You may need to adapt frame names, sign conventions, and the gimbal Jacobian to your hardware.

---

## 1. Problem Setup

**Frames**
- **n**: navigation frame, z-up. Gravity is **g**ⁿ = [0, 0, g]ᵀ.
- **p**: payload (camera) body frame. It carries IMU_p (gyro and accelerometer).
- **h**: handle body frame. It carries IMU_h (gyro, optionally an accelerometer).
- **θ** = [θ₁, θ₂, θ₃]ᵀ: gimbal joint angles from the encoders. **θ̇** is the joint rates.

**Goals**
1. Estimate the payload attitude for the stabilization loop.
2. Estimate a clean, bias-free payload rate for the rate loop.
3. Estimate the handle disturbance rate for feedforward cancellation of hand and walking vibration.

---

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \mathbf{b}_p \\ \boldsymbol{\omega}_d \end{bmatrix}
\in \mathbb{R}^{10}
\qquad
\begin{aligned}
\mathbf{q} &= [q_0, q_1, q_2, q_3]^T && \text{payload attitude quaternion } (p \to n) \quad (4)\\
\mathbf{b}_p &&& \text{payload gyro bias} \quad (3)\\
\boldsymbol{\omega}_d &&& \text{handle angular velocity in } h \text{ (disturbance)} \quad (3)
\end{aligned}
$$

---

## 3. Process Model

**Inputs**
- Payload gyro: **ω**ₘ = **ω**ₚ + **b**ₚ + **n**_g
- Bias-corrected rate: **ω̄** = **ω**ₘ − **b̂**ₚ

**Continuous-time dynamics**

$$
\dot{\mathbf{q}} = \tfrac{1}{2}\,\mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega}_m - \mathbf{b}_p - \mathbf{n}_g \end{bmatrix},
\qquad
\dot{\mathbf{b}}_p = \mathbf{n}_b,
\qquad
\dot{\boldsymbol{\omega}}_d = -\frac{1}{\tau_d}\,\boldsymbol{\omega}_d + \mathbf{n}_d
$$

The disturbance is modeled as a first-order Gauss–Markov process. The correlation time τ_d sets the vibration bandwidth: a corner frequency of about 1/(2πτ_d) placed at the upper edge of the hand-tremor and gait band (≈ 5–20 Hz) is a good starting point.

**Discrete propagation (step Δt)**

$$
\mathbf{q}_{k+1}^- = \Big[\cos\!\big(\tfrac{\|\bar{\boldsymbol\omega}\|\Delta t}{2}\big)\mathbf{I}_4 + \frac{\sin(\|\bar{\boldsymbol\omega}\|\Delta t/2)}{\|\bar{\boldsymbol\omega}\|}\,\boldsymbol{\Omega}(\bar{\boldsymbol\omega})\Big]\mathbf{q}_k^+ ,
\quad
\boldsymbol{\Omega}(\boldsymbol\omega)=\begin{bmatrix}0 & -\boldsymbol\omega^T\\ \boldsymbol\omega & -[\boldsymbol\omega]_\times\end{bmatrix}
$$

$$
\mathbf{b}_{p,k+1}^- = \mathbf{b}_{p,k}^+,
\qquad
\boldsymbol\omega_{d,k+1}^- = \phi\,\boldsymbol\omega_{d,k}^+,\quad \phi = e^{-\Delta t/\tau_d}
$$

After propagation, renormalize the quaternion: **q** ← **q** / ‖**q**‖.

**State-transition Jacobian**

$$
\mathbf{F}_k=\begin{bmatrix}
\mathbf{I}_4+\tfrac{\Delta t}{2}\boldsymbol\Omega(\bar{\boldsymbol\omega}) & -\tfrac{\Delta t}{2}\boldsymbol\Xi(\mathbf{q}) & \mathbf{0}_{4\times3}\\
\mathbf{0}_{3\times4} & \mathbf{I}_3 & \mathbf{0}\\
\mathbf{0}_{3\times4} & \mathbf{0} & \phi\,\mathbf{I}_3
\end{bmatrix},
\qquad
\boldsymbol\Xi(\mathbf q)=\begin{bmatrix}-\mathbf q_v^T\\ q_0\mathbf I_3+[\mathbf q_v]_\times\end{bmatrix}
$$

Here **q**_v = [q₁, q₂, q₃]ᵀ, and Ξ(**q**) is defined so that **q** ⊗ [0; **v**] = Ξ(**q**)**v**.

**Process noise**

$$
\mathbf{Q}_k=\mathrm{blkdiag}\Big(\tfrac{\Delta t^2}{4}\boldsymbol\Xi\,\sigma_g^2\mathbf I_3\,\boldsymbol\Xi^T,\;\; \sigma_b^2\Delta t\,\mathbf I_3,\;\; \sigma_d^2(1-\phi^2)\,\mathbf I_3\Big)
$$

**Covariance prediction**

$$
\mathbf P_{k+1}^- = \mathbf F_k\mathbf P_k^+\mathbf F_k^T + \mathbf Q_k
$$

---

## 4. Measurement Models

### 4a. Handle gyro (direct observation of the disturbance)

$$
\mathbf z_h = \boldsymbol\omega_{h,m} = \boldsymbol\omega_d + \mathbf n_h,
\qquad
\mathbf H_h = [\,\mathbf 0_{3\times4}\;\;\mathbf 0_{3\times3}\;\;\mathbf I_3\,],
\quad
\mathbf R_h=\sigma_h^2\mathbf I_3
$$

### 4b. Gimbal kinematic coupling (pseudo-measurement fusing both IMUs)

The payload rate equals the handle rate rotated into the payload frame, plus the gimbal joint contribution:

$$
\boldsymbol\omega_p = \mathbf C_{ph}(\boldsymbol\theta)\,\boldsymbol\omega_d + \mathbf J(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
$$

Here **C**_ph is the handle-to-payload rotation from the encoders, and **J**(**θ**) is the gimbal joint-rate Jacobian expressed in the payload frame. The pseudo-measurement is:

$$
\mathbf z_k = \boldsymbol\omega_m - \mathbf J(\boldsymbol\theta)\dot{\boldsymbol\theta}
= \mathbf C_{ph}(\boldsymbol\theta)\,\boldsymbol\omega_d + \mathbf b_p + \mathbf n_k
$$

$$
\mathbf H_k = [\,\mathbf 0_{3\times4}\;\;\mathbf I_3\;\;\mathbf C_{ph}(\boldsymbol\theta)\,],
\qquad
\mathbf R_k = \sigma_g^2\mathbf I_3 + \mathbf J\,\sigma_{\dot\theta}^2\mathbf J^T
$$

This coupling is what makes the payload gyro bias **b**ₚ observable. It also ties the two IMUs together.

> **Note:** **n**_g appears both in the propagation input and in this pseudo-measurement, so the noises are correlated. Common practical options:
> - Inflate **R**_k.
> - Apply this update at a decimated rate.
> - Use the cross-correlation term **S** = E[**w** **v**ᵀ] in the gain.

### 4c. Payload accelerometer (gravity reference for roll and pitch)

$$
\mathbf z_a=\mathbf f_p = \mathbf C^T(\mathbf q)\,\mathbf g^n+\mathbf n_a
= g\begin{bmatrix}2(q_1q_3-q_0q_2)\\ 2(q_2q_3+q_0q_1)\\ q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix}+\mathbf n_a
$$

$$
\mathbf H_a = \Big[\;2g\begin{bmatrix}-q_2 & q_3 & -q_0 & q_1\\ q_1 & q_0 & q_3 & q_2\\ q_0 & -q_1 & -q_2 & q_3\end{bmatrix}\;\;\mathbf 0_{3\times3}\;\;\mathbf 0_{3\times3}\Big]
$$

Use adaptive noise to reject linear acceleration, for example while walking:

$$
\mathbf R_a=\sigma_a^2\big(1+\kappa\,\big|\,\|\mathbf f_p\|-g\big|\big)\mathbf I_3
$$

Skip the update entirely if |‖**f**ₚ‖ − g| exceeds a gate threshold.

*(Yaw is unobservable from these sensors. Add a magnetometer or a handle-referenced yaw-follow pseudo-measurement if needed.)*

---

## 5. Update Equations (applied for each available measurement *i* ∈ {h, k, a})

$$
\begin{aligned}
\mathbf y &= \mathbf z_i - h_i(\hat{\mathbf x}^-)\\
\mathbf S &= \mathbf H_i\mathbf P^-\mathbf H_i^T+\mathbf R_i\\
\mathbf K &= \mathbf P^-\mathbf H_i^T\mathbf S^{-1}\\
\hat{\mathbf x}^+ &= \hat{\mathbf x}^- + \mathbf K\mathbf y,\qquad \mathbf q\leftarrow \mathbf q/\|\mathbf q\|\\
\mathbf P^+ &= (\mathbf I-\mathbf K\mathbf H_i)\mathbf P^-(\mathbf I-\mathbf K\mathbf H_i)^T+\mathbf K\mathbf R_i\mathbf K^T \quad\text{(Joseph form)}
\end{aligned}
$$

Before each update, gate on the innovation using the Mahalanobis distance: reject the measurement if **y**