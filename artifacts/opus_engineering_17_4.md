# 10-State EKF for Gimbal Vibration Isolation: Fusing Payload IMU and Handle IMU Angular Velocity

I don't have the context of your earlier cycles, so this formulation is self-contained. The states and frame conventions below are explicit so you can map them onto your existing notation.

---

## 1. Problem Setup and Frames

| Frame | Meaning |
|---|---|
| **{N}** | Navigation frame (local level, z-up, gravity **g**ₙ = [0, 0, g]ᵀ) |
| **{C}** | Camera / payload body frame (payload IMU mounted here) |
| **{H}** | Handle frame (handle IMU mounted here) |
| **θ** = [θ₁, θ₂, θ₃] | Gimbal joint angles from the encoders (yaw, roll, pitch chain) |

**Goal.** Estimate the camera attitude and the *true* camera angular velocity with low latency and low noise. The rate estimate feeds the rate loop, and the handle-rate path acts as a disturbance feedforward for vibration isolation.

**Sensor roles:**
- **Payload gyro:** high bandwidth, but biased. It directly sees the residual vibration at the camera.
- **Payload accelerometer:** provides a gravity reference for roll and pitch. It is corrupted by translational vibration.
- **Handle gyro + encoders:** measure the disturbance at its source. Mapped through the gimbal kinematics, they give an independent measurement of camera rate, which makes the payload gyro bias observable.

---

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \boldsymbol{\omega} \\ \mathbf{b}_c \end{bmatrix}
\in \mathbb{R}^{10},
\qquad
\begin{aligned}
\mathbf{q} &= [q_w, q_x, q_y, q_z]^T &&\text{camera attitude, } C\!\to\!N \;(4)\\
\boldsymbol{\omega} &= [\omega_x, \omega_y, \omega_z]^T &&\text{true camera angular rate in } \{C\} \;(3)\\
\mathbf{b}_c &= [b_x, b_y, b_z]^T &&\text{payload gyro bias} \;(3)
\end{aligned}
$$

**Why ω is a state rather than an input:** treating ω as a state lets the filter blend both gyros optimally. Its output ω̂ is the filtered rate used by the stabilization loop.

---

## 3. Continuous-Time Process Model

$$
\dot{\mathbf{q}} = \tfrac{1}{2}\,\boldsymbol{\Omega}(\boldsymbol{\omega})\,\mathbf{q}, \qquad
\boldsymbol{\Omega}(\boldsymbol{\omega}) =
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z\\
\omega_x & 0 & \omega_z & -\omega_y\\
\omega_y & -\omega_z & 0 & \omega_x\\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

$$
\dot{\boldsymbol{\omega}} = -\frac{1}{\tau_\omega}\boldsymbol{\omega} + \mathbf{w}_\omega
\qquad\text{(first-order Gauss–Markov; } \tau_\omega \text{ sets the expected vibration bandwidth)}
$$

$$
\dot{\mathbf{b}}_c = \mathbf{w}_b \qquad\text{(random walk)}
$$

with $\mathbf{w}_\omega \sim \mathcal{N}(0, \sigma_{\omega}^2 \mathbf{I})$ and $\mathbf{w}_b \sim \mathcal{N}(0, \sigma_{b}^2 \mathbf{I})$.

**Optional extension:** if the motor torque command **u** is available, replace the Gauss–Markov model with a rigid-body model:
$\dot{\boldsymbol\omega} = \mathbf{J}_c^{-1}(\mathbf{u} - \boldsymbol\omega\times\mathbf{J}_c\boldsymbol\omega) + \mathbf{w}_\omega$.

---

## 4. Discrete Prediction (step Δt)

**State propagation:**

$$
\hat{\mathbf{q}}_{k|k-1} = \left[\cos\!\left(\tfrac{\|\boldsymbol\omega\|\Delta t}{2}\right)\mathbf{I}_4 + \frac{\sin\!\left(\tfrac{\|\boldsymbol\omega\|\Delta t}{2}\right)}{\|\boldsymbol\omega\|}\boldsymbol{\Omega}(\hat{\boldsymbol\omega})\right]\hat{\mathbf{q}}_{k-1}, \quad \text{then normalize}
$$

$$
\hat{\boldsymbol\omega}_{k|k-1} = \phi\,\hat{\boldsymbol\omega}_{k-1}, \quad \phi = e^{-\Delta t/\tau_\omega}
\qquad\qquad
\hat{\mathbf b}_{c,k|k-1} = \hat{\mathbf b}_{c,k-1}
$$

**Covariance propagation:**

$$
\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1} \mathbf{F}_k^T + \mathbf{Q}_k
$$

### State Transition Jacobian F (10×10)

$$
\mathbf{F} =
\begin{bmatrix}
\mathbf{I}_4 + \tfrac{\Delta t}{2}\boldsymbol{\Omega}(\hat{\boldsymbol\omega}) & \tfrac{\Delta t}{2}\boldsymbol{\Xi}(\hat{\mathbf q}) & \mathbf{0}_{4\times3}\\
\mathbf{0}_{3\times4} & \phi\,\mathbf{I}_3 & \mathbf{0}_{3}\\
\mathbf{0}_{3\times4} & \mathbf{0}_{3} & \mathbf{I}_3
\end{bmatrix}
$$

where Ξ(q) satisfies $\boldsymbol\Omega(\boldsymbol\omega)\mathbf q = \boldsymbol\Xi(\mathbf q)\boldsymbol\omega$:

$$
\boldsymbol{\Xi}(\mathbf q) =
\begin{bmatrix}
-q_x & -q_y & -q_z\\
q_w & -q_z & q_y\\
q_z & q_w & -q_x\\
-q_y & q_x & q_w
\end{bmatrix}
$$

### Process Noise Q

$$
\mathbf{Q} = \mathrm{blkdiag}\Big(\sigma_q^2\Delta t\,\mathbf I_4,\;\; \sigma_\omega^2\tfrac{\tau_\omega}{2}(1-\phi^2)\,\mathbf I_3,\;\; \sigma_b^2\Delta t\,\mathbf I_3\Big)
$$

Here $\sigma_q$ is a small regularization term that keeps P well-conditioned.

---

## 5. Measurement Models

There are three measurement channels. Each can be applied as a sequential update at its own rate.

### 5.1 Payload Gyro (e.g., 1–8 kHz)

$$
\mathbf z_{g} = \boldsymbol\omega + \mathbf b_c + \mathbf v_g,
\qquad
\mathbf H_g = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf I_3\end{bmatrix},
\qquad
\mathbf R_g = \sigma_{g}^2\mathbf I_3
$$

### 5.2 Handle Gyro Mapped Through Gimbal Kinematics (vibration source channel)

The camera rate equals the handle rate rotated into {C} plus the joint-rate contribution:

$$
\mathbf z_{h} \triangleq \mathbf R_{CH}(\boldsymbol\theta)\,\boldsymbol\omega_h^{meas} + \mathbf J_\theta(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
= \boldsymbol\omega + \mathbf v_h
$$

$$
\mathbf H_h = \begin{bmatrix}\mathbf 0_{3\times4} & \mathbf I_3 & \mathbf 0_3\end{bmatrix},
\qquad
\mathbf R_h = \mathbf R_{CH}\,\boldsymbol\Sigma_{h}\,\mathbf R_{CH}^T + \mathbf J_\theta \boldsymbol\Sigma_{\dot\theta}\mathbf J_\theta^T + \sigma_{flex}^2\mathbf I_3
$$

**Components of the handle channel:**
- **Joint-rate Jacobian.** For a yaw–roll–pitch chain, $\mathbf J_\theta$ maps the joint rates into {C}. Its columns are the joint axes expressed in {C}.
- **Derivation of θ̇.** Obtain θ̇ by differentiating the encoders, filtered at the same rate the measurement is used.
- **Flex term.** $\sigma_{flex}$ accounts for arm/bracket compliance and the encoder–IMU time skew. This inflation term is important because the rigid-link assumption breaks down at structural resonances.

**Why this channel makes the bias observable:** the payload gyro sees ω + b_c, while the handle channel sees ω alone. Their difference isolates b_c.

**Handle gyro bias:** the handle gyro bias is not in this 10-state model.
- Either calibrate it at startup with a static-handle check,
- or extend to 13 states by adding $\mathbf b_h$, with $\mathbf H_h = [\mathbf 0\;\; \mathbf I\;\; \mathbf 0\;\; \mathbf R_{CH}]$.

### 5.3 Payload Accelerometer (gravity reference, e.g., 100–500 Hz)

$$
\mathbf z_a = \mathbf C(\mathbf q)^T\mathbf g_n + \mathbf v_a
= g\begin{bmatrix}
2(q_xq_z - q_wq_y)\\
2(q_yq_z + q_wq_x)\\
q_w^2 - q_x^2 - q_y^2 + q_z^2
\end{bmatrix} + \mathbf v_a
$$

$$
\mathbf H_a = \begin{bmatrix}
\dfrac{\partial \mathbf h_a}{\partial \mathbf q} & \mathbf 0_3 & \mathbf 0_3
\end{bmatrix},
\qquad
\frac{\partial \mathbf h_a}{\partial \mathbf q} = 2g
\begin{bmatrix}
-q_y & q_z & -q_w & q_x\\
q_x & q_w & q_z & q_y\\
q_w & -q_x & -q_y & q_z
\end{bmatrix}
$$

**Adaptive R_a for vibration rejection.** Translational vibration corrupts the gravity reference, so scale the noise with the deviation of the measured specific force from g:

$$
\mathbf R_a = \sigma_a^2\left(1 + k_a\,\big|\,\|\mathbf a^{meas}\| - g\,\big|^2\right)\mathbf I_3,
$$

Skip the update entirely when $\big|\|\mathbf a\| - g\big| > \epsilon_a$.

**Yaw observability:** the accelerometer gives no information about yaw. Yaw is held by the gyro and bias states, or by an optional magnetometer or encoder-yaw update.

---

## 6. Update Equations (each channel i ∈ {g, h, a})

$$
\begin{aligned}
\mathbf y_k &= \mathbf z_{i,k} - \mathbf h_i(\hat{\mathbf x}_{k|k-1}) &&\text{innovation}\\
\mathbf S_k &= \mathbf H_i \mathbf P_{k|k-1}\mathbf H_i^T + \mathbf R_i &&\text{innovation covariance}\\
\mathbf K_k &= \mathbf P_{k|k-1}\mathbf H_i^T \mathbf S_k^{-1} &&\text{Kalman gain (10×3)}\\
\hat{\mathbf x}_{k} &= \hat{\mathbf x}_{k|k-1} + \mathbf K_k\mathbf y_k\\
\mathbf P_k &= (\mathbf I - \mathbf K_k\mathbf H_i)\mathbf P_{k|k-1}(\mathbf I - \mathbf K_k