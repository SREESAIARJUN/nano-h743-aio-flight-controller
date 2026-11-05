# 10-State EKF for Gimbal Vibration Isolation: Fusing Payload and Handle IMUs

## 1. State Vector

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \mathbf{b}_p \\ \boldsymbol{\omega}_h \end{bmatrix} \in \mathbb{R}^{10}
$$

| States | Symbol | Dim | Meaning |
|---|---|---|---|
| 1–4 | $\mathbf{q} = [q_0, q_1, q_2, q_3]^T$ | 4 | Payload attitude, body→nav, scalar-first |
| 5–7 | $\mathbf{b}_p$ | 3 | Payload gyro bias (rad/s) |
| 8–10 | $\boldsymbol{\omega}_h$ | 3 | Handle angular velocity in the handle frame (the disturbance) |

**Design rationale.** The payload gyro drives attitude propagation. The handle IMU observes the disturbance $\boldsymbol{\omega}_h$ directly. The gimbal kinematics link the two IMUs, which makes $\mathbf{b}_p$ observable even when the accelerometer is unusable because of vibration.

## 2. Continuous-Time Process Model

$$
\begin{aligned}
\dot{\mathbf{q}} &= \tfrac{1}{2}\,\Omega(\boldsymbol{\omega}_m - \mathbf{b}_p - \mathbf{n}_g)\,\mathbf{q} \\
\dot{\mathbf{b}}_p &= \mathbf{n}_b \\
\dot{\boldsymbol{\omega}}_h &= -\frac{1}{\tau_h}\boldsymbol{\omega}_h + \mathbf{n}_h
\end{aligned}
$$

Where:
- $\boldsymbol{\omega}_m$ is the raw payload gyro reading.
- The handle motion is modelled as a first-order Gauss–Markov process with correlation time $\tau_h$ (typically 5–30 ms for handheld or vehicle vibration).
- The quaternion rate matrix is

$$
\Omega(\boldsymbol{\omega}) = \begin{bmatrix} 0 & -\boldsymbol{\omega}^T \\ \boldsymbol{\omega} & -[\boldsymbol{\omega}\times] \end{bmatrix}
$$

An equivalent form, used in the Jacobian below, is

$$
\tfrac12\Omega(\boldsymbol{\omega})\mathbf{q} = \tfrac12\Xi(\mathbf{q})\boldsymbol{\omega},
\qquad
\Xi(\mathbf{q}) = \begin{bmatrix} -\mathbf{q}_v^T \\ q_0 I_3 + [\mathbf{q}_v\times]\end{bmatrix}
$$

## 3. Discrete Prediction (step $\Delta t$)

Define the bias-corrected rate $\bar{\boldsymbol{\omega}} = \boldsymbol{\omega}_{m,k} - \hat{\mathbf{b}}_{p,k}$.

**State propagation:**

$$
\hat{\mathbf{q}}_{k+1}^- = \underbrace{\left[\cos\!\tfrac{\|\bar{\boldsymbol\omega}\|\Delta t}{2}\, I_4 + \frac{\sin\!\tfrac{\|\bar{\boldsymbol\omega}\|\Delta t}{2}}{\|\bar{\boldsymbol\omega}\|}\,\Omega(\bar{\boldsymbol\omega})\right]}_{\Phi_q}\hat{\mathbf{q}}_k
$$

$$
\hat{\mathbf{b}}_{p,k+1}^- = \hat{\mathbf{b}}_{p,k},
\qquad
\hat{\boldsymbol\omega}_{h,k+1}^- = \alpha\,\hat{\boldsymbol\omega}_{h,k},
\quad \alpha = e^{-\Delta t/\tau_h}
$$

**State transition Jacobian:**

$$
F_k = \begin{bmatrix}
\Phi_q & -\tfrac{\Delta t}{2}\,\Xi(\hat{\mathbf{q}}_k) & 0_{4\times3}\\
0_{3\times4} & I_3 & 0_{3\times3}\\
0_{3\times4} & 0_{3\times3} & \alpha I_3
\end{bmatrix}_{10\times10}
$$

**Process noise:**

$$
Q_k = \begin{bmatrix}
\tfrac{\Delta t^2}{4}\,\Xi\,(\sigma_g^2 I)\,\Xi^T & 0 & 0\\
0 & \sigma_b^2\Delta t\, I_3 & 0\\
0 & 0 & \sigma_h^2\tfrac{\tau_h}{2}(1-\alpha^2) I_3
\end{bmatrix}
$$

**Covariance propagation:**

$$
P_{k+1}^- = F_k P_k F_k^T + Q_k
$$

## 4. Measurement Models

### (a) Handle gyro: direct disturbance observation

$$
\mathbf{z}_h = \boldsymbol\omega_h + \mathbf{v}_h,
\qquad
H_h = \begin{bmatrix} 0_{3\times4} & 0_{3\times3} & I_3 \end{bmatrix},
\qquad
R_h = \sigma_{hg}^2 I_3
$$

The handle gyro bias should be pre-calibrated at startup while stationary. If it is not, extend the state to 13.

### (b) Kinematic coupling through the gimbal encoders: the fusion link

The payload rate equals the handle rate rotated into the payload frame plus the joint-induced rate:

$$
\boldsymbol\omega_p = R_{ph}(\boldsymbol\theta)\,\boldsymbol\omega_h + J(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
$$

Here $\boldsymbol\theta$ are the encoder angles, $R_{ph}$ is the handle→payload rotation, and $J$ is the gimbal joint-axis Jacobian, for example yaw-roll-pitch axes expressed in the payload frame.

The pseudo-measurement subtracts the known joint contribution from the payload gyro:

$$
\mathbf{z}_c = \boldsymbol\omega_m - J(\boldsymbol\theta)\dot{\boldsymbol\theta} = R_{ph}(\boldsymbol\theta)\,\boldsymbol\omega_h + \mathbf{b}_p + \mathbf{v}_c
$$

$$
H_c = \begin{bmatrix} 0_{3\times4} & I_3 & R_{ph}(\boldsymbol\theta) \end{bmatrix},
\qquad
R_c = \left(\sigma_g^2 + \sigma_{\dot\theta}^2\|J\|^2\right) I_3
$$

This equation is what makes $\mathbf{b}_p$ observable against the handle IMU.

### (c) Payload accelerometer: gravity reference (gated)

The model is

$$
\mathbf{z}_a = R(\mathbf{q})^T \mathbf{g} + \mathbf{v}_a,
\qquad
\mathbf{g} = [0,0,g]^T
$$

which expands to

$$
h_a(\mathbf q) = g\begin{bmatrix} 2(q_1q_3 - q_0q_2)\\ 2(q_2q_3 + q_0q_1)\\ q_0^2 - q_1^2 - q_2^2 + q_3^2\end{bmatrix}
$$

$$
H_a = \Big[\; 2g\begin{bmatrix} -q_2 & q_3 & -q_0 & q_1\\ q_1 & q_0 & q_3 & q_2\\ q_0 & -q_1 & -q_2 & q_3\end{bmatrix} \;\Big|\; 0_{3\times3} \;\Big|\; 0_{3\times3}\Big]
$$

**Gating.** Apply this update only when $\big|\|\mathbf{z}_a\| - g\big| < \epsilon_g$ (e.g., 0.05 g). Inflate $R_a$ adaptively with the vibration level:

$$
R_a = \left(\sigma_a^2 + k_v\big|\|\mathbf z_a\|-g\big|^2\right) I_3
$$

## 5. Update (applied sequentially for each available measurement $i$)

$$
\begin{aligned}
\mathbf{y} &= \mathbf{z}_i - h_i(\hat{\mathbf x}^-) \\
S &= H_i P^- H_i^T + R_i \\
K &= P^- H_i^T S^{-1} \\
\hat{\mathbf x}^+ &= \hat{\mathbf x}^- + K\mathbf y \\
P^+ &= (I - KH_i)P^-(I-KH_i)^T + K R_i K^T \quad \text{(Joseph form)}
\end{aligned}
$$

**Quaternion normalization and covariance projection** (required for a 4-parameter quaternion inside an EKF):

$$
\hat{\mathbf q} \leftarrow \frac{\hat{\mathbf q}}{\|\hat{\mathbf q}\|},
\qquad
N = \mathrm{blkdiag}\!\left(I_4 - \hat{\mathbf q}\hat{\mathbf q}^T,\; I_6\right),
\qquad
P \leftarrow N P N^T + \epsilon I
$$

**Outlier rejection.** Reject a measurement if its Mahalanobis distance $\mathbf y^T S^{-1}\mathbf y$ exceeds $\chi^2_{3,0.99} = 11.34$.

## 6. Use in Vibration Isolation

The controller consumes three filter outputs:

1. **Stabilization feedback:** attitude error $\mathbf e = 2\,\mathrm{vec}(\mathbf q_{ref}^{-1}\otimes\hat{\mathbf q})$ and rate $\hat{\boldsymbol\omega}_p = \boldsymbol\omega_m - \hat{\mathbf b}_p$.
2. **Disturbance feedforward:** command the joints to cancel the handle motion,
   $$
   \dot{\boldsymbol\theta}_{ff} = -J(\boldsymbol\theta)^{-1}R_{ph}(\boldsymbol\theta)\,\hat{\boldsymbol\omega}_h
   $$
   This feedforward is the main benefit of the handle IMU. It rejects disturbance before it reaches the payload, which extends isolation bandwidth beyond the feedback loop.
3. **Predicted disturbance for latency compensation:**
   $$
   \hat{\boldsymbol\omega}_h(t+t_d) = e^{-t_d/\tau_h}\,\hat{\boldsymbol\omega}_h
   $$

## 7. Suggested Tuning Starting Point

| Parameter | Typical value (MEMS, 1 kHz) |
|---|---|
| $\Delta t$ | 1 ms (gyros), accelerometer update at 100–200 Hz |
| $\sigma_g$ | 0.005 rad/s/√Hz × √(1/Δt) |
| $\sigma_b$ | 1e-5 rad/s/√s |
| $\tau_h$ | 10 ms |
| $\sigma_h$ | 5–20 rad/s² (scale with expected vibration energy) |
| $\sigma_{hg}$ | Same as $\sigma_g$ |
| $\sigma_a$ | 0.05 m/s², adaptively inflated |
| $P_0$ | diag(1e-2·I₄, 1e-4·I₃, 1e-1·I₃) |

## 8. Notes

- **Error-state alternative.** A multiplicative error-state EKF (9 states: $\delta\boldsymbol\theta$, $\delta\mathbf b_p$, $\delta\boldsymbol\omega_h$) avoids the quaternion covariance singularity and is numerically preferable. The 10-state form above follows the requested structure.
- **Rich handle vibration spectra.** If the handle vibration has dominant resonances, for example from a propeller or engine, replace the Gauss–Markov model with a second-order resonator per axis. That adds 3 states.
- **Time synchronization.** Timestamp-align the two IMUs to within about 100 µs. Any misalignment appears directly as an error in the coupling resid