# 10-State EKF for Gimbal Vibration Isolation: Payload + Handle IMU Fusion

I don't have context for what "Cycle 25" refers to in your project. Below is a self-contained formulation. Adjust the assumptions if they differ from your design.

---

## 1. System Assumptions

| Item | Assumption |
|---|---|
| Payload IMU (P) | 3-axis gyro + 3-axis accelerometer, rigidly mounted on the camera plate |
| Handle IMU (H) | 3-axis gyro on the handle/base, which carries the vibration source |
| Joint kinematics | Motor encoders give the gimbal angles θ, so the rotation R_PH(θ) from the handle to the payload frame is known |
| Coupling | Handle motion leaks into the payload through bearing friction, cable stiffness and motor cogging. This is modeled as first-order viscous coupling. |
| Goal | Estimate clean payload attitude and angular rate, and predict the disturbance from handle motion so the controller can feed it forward |

**Key design choice:** The handle gyro is used as a **process-model input** (the disturbance driver). It is not used as a measurement. This lets the filter predict payload disturbance *before* it appears in the payload gyro, which is the basis of vibration isolation.

---

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \boldsymbol{\omega}_p \\ \mathbf{b}_p \end{bmatrix} \in \mathbb{R}^{10}
\qquad
\begin{aligned}
\mathbf{q} &= [q_w, q_x, q_y, q_z]^T &&\text{payload attitude (nav ← body)} \\
\boldsymbol{\omega}_p &\in \mathbb{R}^3 &&\text{true payload angular rate (body)} \\
\mathbf{b}_p &\in \mathbb{R}^3 &&\text{payload gyro bias}
\end{aligned}
$$

**Inputs:**
- $\mathbf{u}_h = R_{PH}(\theta)\,\tilde{\boldsymbol{\omega}}_h$ is the handle gyro rate rotated into the payload frame. Handle bias is removed beforehand with a static calibration or slow high-pass filter.
- $\boldsymbol{\tau}_m$ is the commanded motor torque.

---

## 3. Continuous-Time Process Model

$$
\begin{aligned}
\dot{\mathbf{q}} &= \tfrac{1}{2}\,\mathbf{q}\otimes\begin{bmatrix}0\\ \boldsymbol{\omega}_p\end{bmatrix} = \tfrac{1}{2}\,\Xi(\mathbf{q})\,\boldsymbol{\omega}_p \\[4pt]
\dot{\boldsymbol{\omega}}_p &= -C\,(\boldsymbol{\omega}_p - \mathbf{u}_h) + J^{-1}\boldsymbol{\tau}_m + \mathbf{w}_\omega \\[4pt]
\dot{\mathbf{b}}_p &= \mathbf{w}_b
\end{aligned}
$$

The terms are defined as follows:
- $C = J^{-1}D$ is the diagonal coupling-rate matrix [1/s]. Here $D$ is the effective viscous/friction coupling and $J$ is the payload inertia.
- $\mathbf{w}_\omega$ absorbs unmodeled torques (cogging, cable spring, air).
- $\mathbf{w}_b$ is the bias random walk.
- $\Xi(\mathbf{q})$ is the 4×3 matrix:

$$
\Xi(\mathbf{q}) = \begin{bmatrix}
-q_x & -q_y & -q_z\\
\ \ q_w & -q_z & \ \ q_y\\
\ \ q_z & \ \ q_w & -q_x\\
-q_y & \ \ q_x & \ \ q_w
\end{bmatrix}
$$

---

## 4. Discrete Prediction (sample period Δt)

### 4.1 State propagation

$$
\begin{aligned}
\hat{\mathbf{q}}_{k|k-1} &= \exp\!\left(\tfrac{1}{2}\Omega(\hat{\boldsymbol{\omega}}_{p,k-1})\Delta t\right)\hat{\mathbf{q}}_{k-1}, \quad\text{then normalize} \\
\hat{\boldsymbol{\omega}}_{p,k|k-1} &= \hat{\boldsymbol{\omega}}_{p,k-1} + \Delta t\left[-C(\hat{\boldsymbol{\omega}}_{p,k-1} - \mathbf{u}_{h,k-1}) + J^{-1}\boldsymbol{\tau}_{m,k-1}\right] \\
\hat{\mathbf{b}}_{p,k|k-1} &= \hat{\mathbf{b}}_{p,k-1}
\end{aligned}
$$

The rate matrix is:

$$
\Omega(\boldsymbol{\omega}) = \begin{bmatrix} 0 & -\boldsymbol{\omega}^T \\ \boldsymbol{\omega} & -[\boldsymbol{\omega}]_\times \end{bmatrix}
$$

The matrix exponential has a closed form:

$$
\exp(\cdot) = \cos\!\left(\tfrac{\|\omega\|\Delta t}{2}\right) I_4 + \frac{\sin(\|\omega\|\Delta t/2)}{\|\omega\|}\,\Omega(\omega)
$$

### 4.2 State-transition Jacobian $F_k$ (10×10)

$$
F_k = \begin{bmatrix}
I_4 + \tfrac{1}{2}\Omega(\hat{\boldsymbol{\omega}}_p)\Delta t & \tfrac{1}{2}\Xi(\hat{\mathbf{q}})\Delta t & 0_{4\times3}\\
0_{3\times4} & I_3 - C\Delta t & 0_{3\times3}\\
0_{3\times4} & 0_{3\times3} & I_3
\end{bmatrix}
$$

### 4.3 Covariance propagation

$$
P_{k|k-1} = F_k P_{k-1} F_k^T + Q_k,
\qquad
Q_k = \text{blkdiag}\!\left(\sigma_q^2 I_4,\; \sigma_\omega^2 I_3,\; \sigma_b^2 I_3\right)\Delta t
$$

- $\sigma_q^2$ is small. It provides regularization only.
- $\sigma_\omega^2$ is the main tuning knob. Set it from the residual (unmodeled) disturbance spectrum in the vibration band.
- $\sigma_b^2$ is taken from the gyro bias instability or Allan deviation.

---

## 5. Measurement Models

### 5.1 Payload gyro (every sample, e.g. 1–8 kHz)

$$
\mathbf{z}_g = \boldsymbol{\omega}_p + \mathbf{b}_p + \mathbf{v}_g,
\qquad
H_g = \begin{bmatrix} 0_{3\times4} & I_3 & I_3 \end{bmatrix},
\qquad
R_g = \sigma_g^2 I_3
$$

### 5.2 Payload accelerometer (gravity reference, gated)

Use NED or ENU with $\mathbf{g}_n = [0,0,g_0]^T$ as the reference specific-force direction:

$$
h_a(\mathbf{q}) = R(\mathbf{q})^T \mathbf{g}_n = g_0\begin{bmatrix}
2(q_xq_z - q_wq_y)\\
2(q_yq_z + q_wq_x)\\
q_w^2 - q_x^2 - q_y^2 + q_z^2
\end{bmatrix}
$$

The Jacobian with respect to $(q_w, q_x, q_y, q_z)$ is:

$$
\frac{\partial h_a}{\partial \mathbf{q}} = 2g_0\begin{bmatrix}
-q_y & \ \ q_z & -q_w & q_x\\
\ \ q_x & \ \ q_w & \ \ q_z & q_y\\
\ \ q_w & -q_x & -q_y & q_z
\end{bmatrix},
\qquad
H_a = \begin{bmatrix} \dfrac{\partial h_a}{\partial \mathbf{q}} & 0_{3\times3} & 0_{3\times3} \end{bmatrix}
$$

**Adaptive noise:** Linear acceleration from walking or vehicles corrupts the gravity reference, so scale the noise with the deviation from 1 g:

$$
R_a = \sigma_a^2\left(1 + k_a\,\big|\,\|\tilde{\mathbf{a}}\| - g_0\big|\right)^2 I_3
$$

Skip the accelerometer update entirely if $\big|\|\tilde{\mathbf a}\|-g_0\big| > \epsilon$.

---

## 6. Update (applied to each measurement set)

$$
\begin{aligned}
\mathbf{y}_k &= \mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1}) \\
S_k &= H_k P_{k|k-1} H_k^T + R_k \\
K_k &= P_{k|k-1} H_k^T S_k^{-1} \\
\hat{\mathbf{x}}_{k} &= \hat{\mathbf{x}}_{k|k-1} + K_k \mathbf{y}_k \\
P_k &= (I - K_kH_k)P_{k|k-1}(I-K_kH_k)^T + K_kR_kK_k^T \quad\text{(Joseph form)}
\end{aligned}
$$

Then normalize the quaternion and project the covariance to keep it consistent:

$$
\hat{\mathbf{q}} \leftarrow \frac{\hat{\mathbf{q}}}{\|\hat{\mathbf{q}}\|},
\qquad
P \leftarrow G P G^T,
\quad
G = \text{blkdiag}\!\left(I_4 - \hat{\mathbf q}\hat{\mathbf q}^T,\; I_6\right)
$$

---

## 7. Outputs for the Vibration-Isolation Controller

| Signal | Use |
|---|---|
| $\hat{\mathbf q}$ | Attitude-hold loop (outer) |
| $\hat{\boldsymbol\omega}_p$ | Low-noise, bias-free rate for the inner damping loop |
| $\hat{\boldsymbol\tau}_{dist} = J\,C(\mathbf u_h - \hat{\boldsymbol\omega}_p)$ | Disturbance torque, cancelled with motor feedforward $\boldsymbol\tau_{ff} = -\hat{\boldsymbol\tau}_{dist}$ |
| Innovation $\mathbf y_g$ | Online health check and adaptive $\sigma_\omega$ tuning (NIS test) |

---

## 8. Implementation Notes

1. **Rank deficiency.** The 4-component quaternion makes $P$ rank-deficient along $\hat{\mathbf q}$, which is why the projection step is needed. If numerical issues appear, switch to an error-state (multiplicative) EKF. That uses 9 error states $[\delta\boldsymbol\theta, \delta\boldsymbol\omega, \delta\mathbf b]$ while keeping 10 nominal states.
2. **Timing.**
   - Synchronize both IMUs, using a shared SPI clock or timestamp interpolation. A handle-to-payload latency mismatch of only 1 ms produces large phase error at 100–200 Hz vibration.
   - Run prediction and the gyro update at the IMU rate. Run the accelerometer update at a lower rate (100–500 Hz