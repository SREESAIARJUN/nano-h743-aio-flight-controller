# 10-State EKF for Payload/Handle IMU Fusion (Gimbal Vibration Isolation)

## 1. Problem Setup

**Sensors**

| Sensor | Frame | Measures | Typical role |
|---|---|---|---|
| Payload IMU (gyro + accel) | {P} camera body | ω_p, f_p | Primary attitude/rate feedback |
| Handle IMU (gyro + accel) | {H} handle body | ω_h, f_h | Disturbance (vibration) feedforward |
| Joint encoders | gimbal axes | θ, θ̇ (3-axis) | Kinematic link {H}→{P} |

**Objective:** estimate payload attitude and *true* payload angular rate with low latency and bias, so the stabilization loop rejects handle vibration (feedback) while the handle IMU provides a model-based prediction of incoming disturbance (feedforward).

**Frames:** {N} navigation (local level), {H} handle, {P} payload. R_PH(θ) is the rotation from handle to payload given by gimbal kinematics.

---

## 2. State Vector (10 states)

$$
\mathbf{x} =
\begin{bmatrix}
\mathbf{q}_{NP} \\ \boldsymbol{\omega}_p \\ \mathbf{b}_p
\end{bmatrix}
\in \mathbb{R}^{10}
$$

- **q_NP** (4): payload attitude quaternion, Hamilton convention, scalar first, ‖q‖ = 1
- **ω_p** (3): true payload angular rate in {P}
- **b_p** (3): payload gyro bias

> Implementation note: because q has a unit-norm constraint, the covariance is carried in **error-state form** (multiplicative EKF), so **P is 9×9** while the nominal state is 10-dimensional:
> $$\delta\mathbf{x} = [\delta\boldsymbol{\theta}^T,\ \delta\boldsymbol{\omega}^T,\ \delta\mathbf{b}^T]^T \in \mathbb{R}^9,\qquad \mathbf{q} = \hat{\mathbf{q}} \otimes \begin{bmatrix}1\\ \tfrac12\delta\boldsymbol{\theta}\end{bmatrix}$$

Handle gyro bias b_h is treated as pre-calibrated (or estimated in a slow outer loop) to keep the dimension at 10. If handle bias drift is significant, extend to 13 states.

---

## 3. Process Model (Continuous Time)

### 3.1 Attitude kinematics
$$
\dot{\mathbf{q}}_{NP} = \tfrac12\, \mathbf{q}_{NP} \otimes \begin{bmatrix}0\\ \boldsymbol{\omega}_p\end{bmatrix} = \tfrac12\,\boldsymbol{\Omega}(\boldsymbol{\omega}_p)\,\mathbf{q}_{NP}
$$
$$
\boldsymbol{\Omega}(\boldsymbol{\omega}) =
\begin{bmatrix}
0 & -\boldsymbol{\omega}^T\\
\boldsymbol{\omega} & -[\boldsymbol{\omega}]_\times
\end{bmatrix}
$$

### 3.2 Payload rate dynamics (vibration transmission model)
The payload rate is modelled as the stabilized response to handle motion transmitted through the gimbal (motor stiffness, damping, cable/bearing friction):

$$
\dot{\boldsymbol{\omega}}_p = -\mathbf{A}_\omega\,\boldsymbol{\omega}_p + \mathbf{K}_c\big(\mathbf{R}_{PH}(\theta)\,\tilde{\boldsymbol{\omega}}_h + \mathbf{J}(\theta)\dot{\theta} - \boldsymbol{\omega}_p\big) + \mathbf{B}_u\,\boldsymbol{\tau}_m + \mathbf{w}_\omega
$$

- **A_ω** = diag(1/τ_x, 1/τ_y, 1/τ_z): Gauss–Markov decay (closed-loop bandwidth)
- **K_c**: transmissibility coupling gain (identified from handle→payload frequency response)
- **ω̃_h** = ω_h,meas − b_h: handle gyro, used as a **known input** (this is the feedforward fusion path)
- **J(θ)θ̇**: gimbal joint-rate contribution
- **B_u τ_m**: commanded motor torque mapped through the inverse payload inertia, B_u = I_p⁻¹
- **w_ω** ~ N(0, Q_ω): unmodelled vibration and friction

### 3.3 Bias
$$
\dot{\mathbf{b}}_p = \mathbf{w}_b,\qquad \mathbf{w}_b \sim \mathcal{N}(0, \mathbf{Q}_b)\quad\text{(random walk)}
$$

---

## 4. Discrete Prediction (step k → k+1, sample time Δt)

**Nominal propagation**
$$
\hat{\mathbf{q}}_{k+1}^- = \hat{\mathbf{q}}_k \otimes \exp\!\Big(\tfrac12\hat{\boldsymbol{\omega}}_{p,k}\Delta t\Big),\qquad
\exp\!\Big(\tfrac12\boldsymbol{\phi}\Big) = \begin{bmatrix}\cos\frac{\|\boldsymbol{\phi}\|}{2}\\ \frac{\boldsymbol{\phi}}{\|\boldsymbol{\phi}\|}\sin\frac{\|\boldsymbol{\phi}\|}{2}\end{bmatrix}
$$
$$
\hat{\boldsymbol{\omega}}_{p,k+1}^- = \hat{\boldsymbol{\omega}}_{p,k} + \Delta t\Big[-(\mathbf{A}_\omega+\mathbf{K}_c)\hat{\boldsymbol{\omega}}_{p,k} + \mathbf{K}_c\big(\mathbf{R}_{PH}\tilde{\boldsymbol{\omega}}_{h,k} + \mathbf{J}\dot\theta_k\big) + \mathbf{B}_u\boldsymbol{\tau}_{m,k}\Big]
$$
$$
\hat{\mathbf{b}}_{p,k+1}^- = \hat{\mathbf{b}}_{p,k}
$$

**Error-state Jacobian (9×9)**
$$
\mathbf{F} =
\begin{bmatrix}
-[\hat{\boldsymbol{\omega}}_p]_\times & \mathbf{I}_3 & \mathbf{0}\\
\mathbf{0} & -(\mathbf{A}_\omega + \mathbf{K}_c) & \mathbf{0}\\
\mathbf{0} & \mathbf{0} & \mathbf{0}
\end{bmatrix},
\qquad
\boldsymbol{\Phi}_k \approx \mathbf{I}_9 + \mathbf{F}\Delta t + \tfrac12\mathbf{F}^2\Delta t^2
$$

**Noise mapping**
$$
\mathbf{G} = \begin{bmatrix}\mathbf{0} & \mathbf{0} & \mathbf{0}\\ \mathbf{I}_3 & \mathbf{K}_c\mathbf{R}_{PH} & \mathbf{0}\\ \mathbf{0} & \mathbf{0} & \mathbf{I}_3\end{bmatrix},
\qquad
\mathbf{Q}_c = \mathrm{blkdiag}(\mathbf{Q}_\omega,\ \sigma_{g,h}^2\mathbf{I}_3,\ \mathbf{Q}_b)
$$

The middle column propagates **handle gyro noise** into payload-rate uncertainty, so the filter automatically down-weights the feedforward path when the handle IMU is noisy.

$$
\mathbf{Q}_k \approx \boldsymbol{\Phi}_k\mathbf{G}\mathbf{Q}_c\mathbf{G}^T\boldsymbol{\Phi}_k^T\,\Delta t
$$

**Covariance prediction**
$$
\mathbf{P}_{k+1}^- = \boldsymbol{\Phi}_k\mathbf{P}_k\boldsymbol{\Phi}_k^T + \mathbf{Q}_k
$$

---

## 5. Measurement Models

### 5.1 Payload gyro (every sample, e.g. 1–8 kHz)
$$
\mathbf{z}_g = \boldsymbol{\omega}_p + \mathbf{b}_p + \mathbf{v}_g,\qquad \mathbf{v}_g\sim\mathcal{N}(0,\sigma_{g,p}^2\mathbf{I}_3)
$$
$$
\mathbf{H}_g = \begin{bmatrix}\mathbf{0}_3 & \mathbf{I}_3 & \mathbf{I}_3\end{bmatrix}
$$

### 5.2 Payload accelerometer (gravity reference, e.g. 100–500 Hz)
$$
\mathbf{z}_a = \mathbf{R}(\mathbf{q}_{NP})^T\begin{bmatrix}0\\0\\g\end{bmatrix} + \mathbf{v}_a,\qquad
\mathbf{H}_a = \begin{bmatrix}\big[\hat{\mathbf{R}}^T\mathbf{g}\big]_\times & \mathbf{0}_3 & \mathbf{0}_3\end{bmatrix}
$$

**Adaptive gating against linear acceleration from walking and vibration:**
$$
\mathbf{R}_a = \sigma_a^2\mathbf{I}_3\Big(1 + \kappa\,\big|\,\|\mathbf{z}_a\| - g\,\big|\Big)
$$
Reject the update entirely if the absolute value of ‖z_a‖ − g exceeds a threshold (for example, 0.5 m/s²).

### 5.3 Handle-derived rate pseudo-measurement (cross-IMU fusion)
$$
\mathbf{z}_h = \mathbf{R}_{PH}(\theta)\,\tilde{\boldsymbol{\omega}}_h + \mathbf{J}(\theta)\dot\theta = \boldsymbol{\omega}_p + \mathbf{v}_h
$$
$$
\mathbf{H}_h = \begin{bmatrix}\mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3\end{bmatrix},\qquad
\mathbf{R}_h = \mathbf{R}_{PH}\,\sigma_{g,h}^2\,\mathbf{R}_{PH}^T + \mathbf{J}\,\boldsymbol{\Sigma}_{\dot\theta}\,\mathbf{J}^T + \sigma_{\text{flex}}^2\mathbf{I}_3
$$

This rigid-kinematic measurement makes **b_p observable from the rate channel**: payload gyro measures ω + b, while the handle path measures ω alone. This sharply speeds up bias convergence compared with relying on the accelerometer alone, and it gives yaw-bias observability that gravity cannot provide. The σ_flex term covers arm/bracket compliance; set it from the measured structural mode amplitude.

**Stacked update (when all are available):**
$$
\mathbf{H} = \begin{bmatrix}\mathbf{H}_g\\ \mathbf{H}_a\\ \mathbf{H}_h\end{bmatrix},\quad
\mathbf{R} = \mathrm{blkdiag}(\mathbf{R}_g, \mathbf{R}_a, \mathbf{R}_h)
$$

---

## 6. Update

$$
\mathbf{y} = \mathbf{z} - h(\hat{\mathbf{x}}^-)
$$
$$
\mathbf{S} = \mathbf{H}\mathbf{P}^-\mathbf{H}^T + \mathbf{R},\qquad
\mathbf{K} = \mathbf{P}^-\mathbf{H}^T\mathbf{S}^{-1}
$$
$$
\delta\hat{\mathbf{x}} = \mathbf{K}\mathbf{y} = [\delta\boldsymbol{\theta}^T,\ \delta\boldsymbol{\omega}^T,\ \delta\mathbf{b}^T]^T
$$

**Inject the error into the nominal state and reset**
$$
\hat{\mathbf{q}}^+ = \hat{\mathbf{q}}^- \otimes \begin{bmatrix}1\\ \tfrac12\delta\boldsymbol{\theta}\end{bmatrix},\quad \hat{\mathbf{q}}^+ \leftarrow \frac{\hat{\mathbf{q}}^+}{\|\hat{\mathbf{q}}^+\|}
$$
$$
\hat{\boldsymbol{\omega}}_p^+ = \hat{\boldsymbol{\omega}}_p^- + \delta\boldsymbol{\omega},\qquad
\hat{\mathbf{b}}_p^+ = \hat{\mathbf{b}}_p^- + \delta\mathbf{b}
$$

**Covariance update (Joseph form, for numerical robustness)**
$$
\mathbf{P}^+ = (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}^-(\mathbf{I}-\mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}\mathbf{K}^T
$$

**Reset Jacobian** (first-order, after attitude injection)
$$
\mathbf{P}^+ \leftarrow \mathbf{G}_r\mathbf{P}^+\mathbf{G}_r^T,\qquad \mathbf{G}_r = \mathrm{blkdiag}\big(\mathbf{I}_3 - [\tfrac12\delta\boldsymbol{\theta}]_\times,\ \mathbf{I}_3,\ \mathbf{I}_3\big)
$$

---

## 7. Use in the Vibration-Isolation Loop

**Feedback** uses the filtered estimates:
$$
\boldsymbol{\tau}_{fb} = \mathbf{K}_p\,\mathrm{Log}(\mathbf{q}_{ref}^{-1}\otimes\hat{\mathbf{q}}_{NP}) + \mathbf{K}_d(\hat{\boldsymbol{\omega}}_p - \boldsymbol{\omega}_{ref})
$$

**Feedforward** cancels the predicted transmitted disturbance before it appears at the payload:
$$
\boldsymbol{\tau}_{ff} = -\mathbf{B}_u^{-1}\mathbf{K}_c\big(\