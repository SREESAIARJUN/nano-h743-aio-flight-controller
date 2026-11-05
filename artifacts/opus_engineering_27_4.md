# 10-State EKF for Payload/Handle IMU Fusion (Gimbal Vibration Isolation)

I don't have your project's "Cycle 27" spec, so this formulation uses standard 3-axis handheld-gimbal assumptions. The places where your hardware may differ are flagged at the end.

---

## 1. State Definition

Nominal state (10 elements):

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q}_{WP} \\ \boldsymbol{\omega}_P \\ \mathbf{b}_P \end{bmatrix} \in \mathbb{R}^{10}
$$

| State | Dim | Meaning |
|---|---|---|
| $\mathbf{q}_{WP}$ | 4 | Payload attitude quaternion (payload → world), unit norm |
| $\boldsymbol{\omega}_P$ | 3 | True payload angular velocity in the payload frame (rad/s) |
| $\mathbf{b}_P$ | 3 | Payload gyro bias (rad/s) |

Because the quaternion carries a unit-norm constraint, the covariance is propagated on a **9-dimensional error state**. This is the multiplicative / error-state EKF form:

$$
\delta\mathbf{x} = \begin{bmatrix} \delta\boldsymbol{\theta} \\ \delta\boldsymbol{\omega} \\ \delta\mathbf{b} \end{bmatrix} \in \mathbb{R}^9,
\qquad
\mathbf{q} = \hat{\mathbf{q}} \otimes \begin{bmatrix} 1 \\ \tfrac12\delta\boldsymbol{\theta} \end{bmatrix}
$$

$\mathbf{P} \in \mathbb{R}^{9\times 9}$.

---

## 2. Process Model

### Continuous-time nominal dynamics

$$
\dot{\mathbf{q}}_{WP} = \tfrac12\, \mathbf{q}_{WP} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega}_P \end{bmatrix}
$$

$$
\dot{\boldsymbol{\omega}}_P = -\frac{1}{\tau_\omega}\boldsymbol{\omega}_P + \underbrace{\mathbf{I}_P^{-1}\big(\boldsymbol{\tau}_m - \boldsymbol{\omega}_P \times \mathbf{I}_P\boldsymbol{\omega}_P\big)}_{\text{optional motor-torque input}} + \mathbf{w}_\omega
$$

$$
\dot{\mathbf{b}}_P = \mathbf{w}_b
$$

- The angular-velocity state is a first-order Gauss–Markov process. The correlation time $\tau_\omega$ is set relative to the dominant vibration band, typically $\tau_\omega \approx 1/(2\pi f_{\text{vib}})$ or longer.
- Include the torque term if commanded motor torque $\boldsymbol{\tau}_m$ and payload inertia $\mathbf{I}_P$ are known. It noticeably sharpens prediction during active correction.
- $\mathbf{w}_\omega \sim \mathcal{N}(0, \sigma_\omega^2 \mathbf{I})$ represents unmodeled vibration torque.
- $\mathbf{w}_b \sim \mathcal{N}(0, \sigma_b^2 \mathbf{I})$ is the bias random walk.

### Error-state dynamics

$$
\dot{\delta\mathbf{x}} = \mathbf{F}\,\delta\mathbf{x} + \mathbf{G}\,\mathbf{w},
\qquad
\mathbf{F} = \begin{bmatrix}
-[\hat{\boldsymbol{\omega}}_P]_\times & \mathbf{I}_3 & \mathbf{0} \\
\mathbf{0} & -\tfrac{1}{\tau_\omega}\mathbf{I}_3 + \mathbf{F}_{\omega\omega} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0}
\end{bmatrix}
$$

If the gyroscopic term is modeled, its Jacobian is:

$$
\mathbf{F}_{\omega\omega} = \mathbf{I}_P^{-1}\big([\mathbf{I}_P\hat{\boldsymbol\omega}]_\times - [\hat{\boldsymbol\omega}]_\times \mathbf{I}_P\big)
$$

Otherwise $\mathbf{F}_{\omega\omega} = \mathbf{0}$.

The noise mapping and continuous noise density are:

$$
\mathbf{G} = \begin{bmatrix}\mathbf{0} & \mathbf{0}\\ \mathbf{I}_3 & \mathbf{0} \\ \mathbf{0} & \mathbf{I}_3\end{bmatrix},
\quad
\mathbf{Q}_c = \mathrm{diag}(\sigma_\omega^2\mathbf{I}_3,\ \sigma_b^2\mathbf{I}_3)
$$

### Discrete propagation (period $\Delta t$)

**Nominal state:**

$$
\hat{\mathbf{q}}_{k+1|k} = \hat{\mathbf{q}}_k \otimes \begin{bmatrix}\cos(\|\hat{\boldsymbol\omega}\|\Delta t/2)\\ \dfrac{\hat{\boldsymbol\omega}}{\|\hat{\boldsymbol\omega}\|}\sin(\|\hat{\boldsymbol\omega}\|\Delta t/2)\end{bmatrix}
$$

$$
\hat{\boldsymbol\omega}_{k+1|k} = e^{-\Delta t/\tau_\omega}\hat{\boldsymbol\omega}_k + \Delta t\,\mathbf{I}_P^{-1}(\cdots)
$$

$$
\hat{\mathbf{b}}_{k+1|k} = \hat{\mathbf{b}}_k
$$

**Covariance:**

$$
\boldsymbol\Phi_k \approx \mathbf{I}_9 + \mathbf{F}\Delta t + \tfrac12(\mathbf{F}\Delta t)^2,
\qquad
\mathbf{Q}_k \approx \boldsymbol\Phi_k \mathbf{G}\mathbf{Q}_c\mathbf{G}^\top\boldsymbol\Phi_k^\top \Delta t
$$

Van Loan's method gives an exact $\mathbf{Q}_k$ if preferred.

$$
\mathbf{P}_{k+1|k} = \boldsymbol\Phi_k\mathbf{P}_k\boldsymbol\Phi_k^\top + \mathbf{Q}_k
$$

---

## 3. Measurement Models

Apply the three updates **sequentially** as each sensor sample arrives. They usually run at different rates.

### (a) Payload gyro: $\mathbf{z}_g = \tilde{\boldsymbol\omega}_P$

$$
\mathbf{h}_g(\mathbf{x}) = \boldsymbol\omega_P + \mathbf{b}_P,
\qquad
\mathbf{H}_g = \begin{bmatrix}\mathbf{0} & \mathbf{I}_3 & \mathbf{I}_3\end{bmatrix},
\qquad
\mathbf{R}_g = \sigma_{g,P}^2\mathbf{I}_3
$$

### (b) Handle IMU mapped through the gimbal kinematics

This is the fusion channel. The handle gyro plus the joint encoders give an independent estimate of payload rate:

$$
\mathbf{z}_h = \mathbf{R}_{PH}(\boldsymbol\theta)\,\big(\tilde{\boldsymbol\omega}_H - \hat{\mathbf{b}}_H\big) + \mathbf{J}(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
$$

$$
\mathbf{h}_h(\mathbf{x}) = \boldsymbol\omega_P,
\qquad
\mathbf{H}_h = \begin{bmatrix}\mathbf{0} & \mathbf{I}_3 & \mathbf{0}\end{bmatrix}
$$

$$
\mathbf{R}_h = \mathbf{R}_{PH}\,\sigma_{g,H}^2\,\mathbf{R}_{PH}^\top + \mathbf{J}\,\boldsymbol\Sigma_{\dot\theta}\,\mathbf{J}^\top + \mathbf{R}_{\text{flex}}
$$

The terms in this measurement:
- $\boldsymbol\theta$ are the yaw/roll/pitch joint angles from the encoders.
- $\mathbf{R}_{PH}(\boldsymbol\theta)$ is the handle-to-payload rotation from forward kinematics.
- $\mathbf{J}(\boldsymbol\theta)$ is the joint-rate Jacobian, whose columns are the joint axes expressed in the payload frame.
- $\boldsymbol\Sigma_{\dot\theta}$ is the encoder-differentiation noise. It is dominated by quantization, roughly $(\Delta\theta_q/\Delta t)^2/12$.
- $\mathbf{R}_{\text{flex}}$ inflates noise to cover structural compliance and damper deflection between handle and payload. Increase it in the frequency bands where the dampers decouple the two bodies.
- $\hat{\mathbf{b}}_H$ is the handle gyro bias, calibrated at startup while stationary (see §6).

### (c) Payload accelerometer (gravity reference, gated)

$$
\mathbf{h}_a(\mathbf{x}) = \mathbf{R}(\mathbf{q}_{WP})^\top\begin{bmatrix}0\\0\\g\end{bmatrix},
\qquad
\mathbf{H}_a = \begin{bmatrix}\big[\mathbf{R}^\top\mathbf{g}\big]_\times & \mathbf{0} & \mathbf{0}\end{bmatrix}
$$

Apply this update only when the accelerometer is plausibly reading gravity alone:

$$
\big|\,\|\tilde{\mathbf{a}}\| - g\,\big| < \epsilon_a
$$

Otherwise inflate $\mathbf{R}_a$. This update makes roll/pitch and the corresponding bias components observable. Yaw is not observable without a magnetometer or vision reference.

---

## 4. Update Equations (per measurement $i \in \{g, h, a\}$)

$$
\mathbf{y} = \mathbf{z}_i - \mathbf{h}_i(\hat{\mathbf{x}}_{k|k-1})
$$

$$
\mathbf{S} = \mathbf{H}_i\mathbf{P}_{k|k-1}\mathbf{H}_i^\top + \mathbf{R}_i
$$

$$
\mathbf{K} = \mathbf{P}_{k|k-1}\mathbf{H}_i^\top\mathbf{S}^{-1}
$$

$$
\delta\hat{\mathbf{x}} = \mathbf{K}\mathbf{y}
$$

$$
\mathbf{P}_{k|k} = (\mathbf{I}-\mathbf{K}\mathbf{H}_i)\mathbf{P}_{k|k-1}(\mathbf{I}-\mathbf{K}\mathbf{H}_i)^\top + \mathbf{K}\mathbf{R}_i\mathbf{K}^\top
$$

The covariance update uses the Joseph form for numerical robustness.

**Optional outlier gate:** reject the measurement if $\mathbf{y}^\top\mathbf{S}^{-1}\mathbf{y} > \chi^2_{3,\,0.997}$. This catches impacts and encoder glitches.

### Error injection and reset

$$
\hat{\mathbf{q}} \leftarrow \text{normalize}\!\left(\hat{\mathbf{q}} \otimes \begin{bmatrix}1\\ \tfrac12\delta\hat{\boldsymbol\theta}\end{bmatrix}\right),
\quad
\hat{\boldsymbol\omega}_P \leftarrow \hat{\boldsymbol\omega}_P + \delta\hat{\boldsymbol\omega},
\quad
\hat{\mathbf{b}}_P \leftarrow \hat{\mathbf{b}}_P + \delta\hat{\mathbf{b}}
$$

$$
\mathbf{P} \leftarrow \mathbf{\Gamma}\mathbf{P}\mathbf{\Gamma}^\top,
\qquad
\mathbf{\Gamma} = \mathrm{blkdiag}\!\left(\mathbf{I}_3 - \big[\tfrac