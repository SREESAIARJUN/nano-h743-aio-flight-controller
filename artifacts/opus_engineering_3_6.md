# 10-State EKF for Dual-IMU Gimbal Vibration Isolation

I don't have the earlier cycles of your project, so this formulation uses a standard 3-axis gimbal setup:
- a payload (camera) IMU mounted on the stabilized platform,
- a handle IMU mounted on the hand-held base,
- joint encoders between them.

Adjust frames and signs to match your hardware.

---

## 1. Frames and Notation

| Symbol | Meaning |
|---|---|
| $\{n\}$ | Navigation frame, z-up |
| $\{p\}$ | Payload (camera) body frame |
| $\{h\}$ | Handle body frame |
| $q = [q_w, q_x, q_y, q_z]^T$ | Payload attitude (Hamilton, scalar-first), $\{p\}\to\{n\}$ |
| $C(q)$ | Rotation matrix of $q$ |
| $\theta = [\theta_1,\theta_2,\theta_3]^T$ | Gimbal joint angles from the encoders |
| $R_{ph}(\theta)$ | Rotation $\{h\}\to\{p\}$ from the gimbal forward kinematics |
| $J(\theta)$ | Joint-rate Jacobian; relative rate $= J(\theta)\dot\theta$, expressed in $\{p\}$ |
| $\tilde\omega_p, \tilde\omega_h$ | Raw gyro readings (payload, handle) |
| $\tilde a_p$ | Payload accelerometer reading |

---

## 2. State Vector (10 states)

$$
\mathbf{x} =
\begin{bmatrix} q \\ b_p \\ b_h \end{bmatrix}
\in \mathbb{R}^{10},
\qquad
\begin{aligned}
q &\in \mathbb{R}^4 &&\text{payload attitude}\\
b_p &\in \mathbb{R}^3 &&\text{payload gyro bias}\\
b_h &\in \mathbb{R}^3 &&\text{handle gyro bias}
\end{aligned}
$$

**Sensor models:**

$$
\tilde\omega_p = \omega_p + b_p + n_{g,p},
\qquad
\tilde\omega_h = \omega_h + b_h + n_{g,h}
$$

**Why this state choice:**
- The payload gyro is the propagation input.
- The handle gyro enters as a kinematic rate-consistency measurement through the gimbal joints.
- This makes **both** biases observable.
- The filter outputs a clean payload inertial rate for the feedback loop and a clean handle rate for disturbance feed-forward.

---

## 3. Process Model

### 3.1 Continuous time

$$
\dot q = \tfrac12\, q \otimes \begin{bmatrix}0\\ \bar\omega\end{bmatrix} = \tfrac12\,\Xi(q)\,\bar\omega,
\qquad
\bar\omega = \tilde\omega_p - b_p - n_{g,p}
$$

$$
\dot b_p = w_{b,p},\qquad \dot b_h = w_{b,h}
\qquad\text{(random walks)}
$$

The matrix $\Xi(q)$ is:

$$
\Xi(q)=\begin{bmatrix}
-q_x & -q_y & -q_z\\
\ \ q_w & -q_z & \ \ q_y\\
\ \ q_z & \ \ q_w & -q_x\\
-q_y & \ \ q_x & \ \ q_w
\end{bmatrix}
$$

The skew operator $\Omega(\bar\omega)$ is:

$$
\Omega(\bar\omega)=\begin{bmatrix}
0 & -\bar\omega^T\\
\bar\omega & -[\bar\omega]_\times
\end{bmatrix}
$$

### 3.2 Discrete prediction (sample period $\Delta t$)

Define $\hat{\bar\omega}_k = \tilde\omega_{p,k} - \hat b_{p,k}$.

**Quaternion propagation (exact for constant rate over the step):**

$$
\hat q_{k+1}^- =
\Big[\cos\!\big(\tfrac{\|\hat{\bar\omega}\|\Delta t}{2}\big) I_4
+ \frac{\sin\!\big(\tfrac{\|\hat{\bar\omega}\|\Delta t}{2}\big)}{\|\hat{\bar\omega}\|}\,\Omega(\hat{\bar\omega})\Big]\hat q_k
\;\equiv\; \Phi_q\,\hat q_k
$$

**Bias propagation:**

$$
\hat b_{p,k+1}^- = \hat b_{p,k},
\qquad
\hat b_{h,k+1}^- = \hat b_{h,k}
$$

### 3.3 State-transition Jacobian $F_k$ (10×10)

$$
F_k=\begin{bmatrix}
\Phi_q & -\tfrac{\Delta t}{2}\,\Xi(\hat q_k) & 0_{4\times3}\\
0_{3\times4} & I_3 & 0_{3}\\
0_{3\times4} & 0_3 & I_3
\end{bmatrix}
$$

### 3.4 Process noise $Q_k$

$$
Q_k=\begin{bmatrix}
\tfrac{\Delta t^2}{4}\,\Xi\,(\sigma_{g,p}^2 I_3)\,\Xi^T & 0 & 0\\
0 & \sigma_{b,p}^2\Delta t\, I_3 & 0\\
0 & 0 & \sigma_{b,h}^2\Delta t\, I_3
\end{bmatrix}
$$

### 3.5 Covariance prediction

$$
P_{k+1}^- = F_k P_k F_k^T + Q_k
$$

---

## 4. Measurement Models

### 4.1 Measurement A: dual-IMU rate consistency (the core fusion)

**Kinematic constraint.** The payload rate equals the handle rate rotated into $\{p\}$, plus the joint-induced relative rate:

$$
\omega_p = R_{ph}(\theta)\,\omega_h + J(\theta)\dot\theta
$$

**Pseudo-measurement.** Substitute the sensor models and form:

$$
z_\omega = \tilde\omega_p - R_{ph}(\theta)\,\tilde\omega_h - J(\theta)\dot\theta
$$

The true angular motion cancels. This includes the handle vibration common to both sensors, which leaves only the biases:

$$
h_\omega(\mathbf{x}) = b_p - R_{ph}(\theta)\, b_h,
\qquad
H_\omega=\begin{bmatrix}0_{3\times4} & I_3 & -R_{ph}(\theta)\end{bmatrix}
$$

**Measurement noise:**

$$
R_\omega=\sigma_{g,p}^2 I_3 + R_{ph}\,\sigma_{g,h}^2\,R_{ph}^T + J\,\Sigma_{\dot\theta}\,J^T
\;(+\;\sigma_{\text{flex}}^2 I_3)
$$

Notes on $R_\omega$:
- $\Sigma_{\dot\theta}$ is the noise of encoder-differentiated joint rates.
- $\sigma_{\text{flex}}^2$ absorbs arm and mount compliance and residual vibration that is not rigid-body.
- **Bias separability.** $b_p$ and $b_h$ are individually observable only when $R_{ph}(\theta)$ varies over time (gimbal articulation or handle rotation). Otherwise only the combination $b_p - R_{ph}b_h$ is observable. The accelerometer update in 4.2 additionally anchors the roll/pitch components of $b_p$ through attitude.
- **Correlated noise.** $n_{g,p}$ appears in both the process and this measurement. Either inflate $R_\omega$ by about 1.5–2× empirically, or run this update at a decimated rate (e.g., 1:4) to decorrelate.

### 4.2 Measurement B: payload accelerometer gravity reference

At quasi-static acceleration, the specific force is $C(q)^T g_n$ with $g_n = [0,0,g]^T$:

$$
h_a(q) = g\begin{bmatrix}
2(q_xq_z - q_wq_y)\\
2(q_yq_z + q_wq_x)\\
q_w^2 - q_x^2 - q_y^2 + q_z^2
\end{bmatrix}
$$

The Jacobian is nonzero only in the quaternion columns:

$$
H_a=\Big[\,\underbrace{2g\begin{bmatrix}
-q_y & \ \ q_z & -q_w & q_x\\
\ \ q_x & \ \ q_w & \ \ q_z & q_y\\
\ \ q_w & -q_x & -q_y & q_z
\end{bmatrix}}_{\partial h_a/\partial q}\;\Big|\;0_{3\times3}\;\Big|\;0_{3\times3}\Big]
$$

**Adaptive noise and gating.** Linear and vibratory acceleration corrupt the gravity reference, so scale the noise with the deviation from 1 g:

$$
R_a = \big(\sigma_a^2 + k_a\,\big|\|\tilde a_p\| - g\big|\big)\, I_3
$$

Skip the update entirely when $\big|\|\tilde a_p\|-g\big| > \epsilon_a$ (typically 0.05–0.1 g). Low-pass filter $\tilde a_p$ at about 5–10 Hz before use.

---

## 5. Update Equations (apply to each measurement $i \in \{\omega, a\}$)

**Innovation and its covariance:**

$$
\nu = z_i - h_i(\hat{\mathbf{x}}^-),
\qquad
S = H_i P^- H_i^T + R_i
$$

**Outlier gate (χ², 3 DOF; 99% threshold ≈ 11.34):**

$$
\nu^T S^{-1}\nu < \chi^2_{3,\,0.99}
$$

**Gain and state update:**

$$
K = P^- H_i^T S^{-1},
\qquad
\hat{\mathbf{x}}^+ = \hat{\mathbf{x}}^- + K\nu
$$

**Covariance update (Joseph form, for numerical stability):**

$$
P^+ = (I - KH_i)\,P^-\,(I-KH_i)^T + K R_i K^T
$$

### 5.1 Quaternion normalization and covariance projection

The full-state quaternion parameterization adds one redundant dimension. After each update:

$$
\hat q \leftarrow \frac{\hat q}{\|\hat q\|},
\qquad
N = \mathrm{blkdiag}\!\Big(I_4 - \hat q\hat q^T,\; I_3,\; I_3\Big),
\qquad
P \leftarrow N P N^T
$$

This removes covariance along the unit-norm direction, which keeps $P$ consistent. Alternatively, implement the equivalent 9-dimensional error-state (multiplicative) EKF with $\delta\theta \in \mathbb{R}^3$. The structure of the equations is the same.

---

## 6. Filter Outputs for Vibration Isolation

| Output | Expression | Use |
|---|---|---|
| Payload inertial rate | $\hat\omega_p = \tilde\omega_p - \hat b_p$ | Rate-loop feedback (target: 0 in stabilize mode) |
| Payload attitude | $\hat q$ | Outer attitude / foll