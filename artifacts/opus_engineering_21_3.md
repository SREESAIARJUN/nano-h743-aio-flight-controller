# 10-State EKF for Payload/Handle IMU Fusion in a Gimbal Vibration Isolation Loop

I don't have the earlier design cycles, so this is a self-contained formulation using the most common architecture for a handheld 3-axis gimbal:

- The **payload (camera) IMU** propagates attitude.
- The **handle IMU**, mapped through the joint kinematics, supplies a gyro-consistency measurement.
- The **accelerometer** supplies a gravity reference.

Adjust the frames and conventions if your earlier cycles fixed them differently.

---

## 1. Conventions

| Symbol | Meaning |
|---|---|
| $\{W\}, \{P\}, \{H\}$ | World (z-up), payload/camera body, handle body frames |
| $q = [q_w, q_x, q_y, q_z]^T$ | Hamilton unit quaternion, payload → world |
| $R(q)$ | Corresponding rotation matrix $R_{WP}$ |
| $\theta = [\theta_1,\theta_2,\theta_3]^T$ | Gimbal joint angles from the motor encoders |
| $R_{PH}(\theta)$ | Rotation handle → payload, from forward kinematics |
| $J(\theta)$ | Joint-rate Jacobian, so that $\omega_{\text{rel}} = J(\theta)\dot\theta$ in $\{P\}$ |
| $[v]_\times$ | Skew-symmetric matrix of $v$ |

---

## 2. State Vector (10 states)

$$
x = \begin{bmatrix} q \\ b_p \\ b_h \end{bmatrix} \in \mathbb{R}^{10},
\qquad
\begin{aligned}
q &\in \mathbb{R}^4 &&\text{payload attitude}\\
b_p &\in \mathbb{R}^3 &&\text{payload gyro bias}\\
b_h &\in \mathbb{R}^3 &&\text{handle gyro bias}
\end{aligned}
$$

---

## 3. Sensor Models

$$
\begin{aligned}
\omega_{p,m} &= \omega_p + b_p + n_{gp}, & n_{gp}&\sim\mathcal N(0,\Sigma_{gp})\\
\omega_{h,m} &= \omega_h + b_h + n_{gh}, & n_{gh}&\sim\mathcal N(0,\Sigma_{gh})\\
a_{p,m} &= R(q)^T\!\left(a_W + g\,e_3\right) + n_a, & n_a&\sim\mathcal N(0,\Sigma_a)
\end{aligned}
$$

The kinematic coupling through the gimbal joints is:

$$
\omega_p = R_{PH}(\theta)\,\omega_h + J(\theta)\,\dot\theta
$$

---

## 4. Process Model

**Continuous time:**

$$
\dot q = \tfrac12\, q \otimes \begin{bmatrix}0\\ \hat\omega\end{bmatrix} = \tfrac12\,\Omega(\hat\omega)\,q,
\qquad \hat\omega = \omega_{p,m} - b_p
$$

$$
\dot b_p = w_{bp},\qquad \dot b_h = w_{bh}\qquad (\text{random walks})
$$

where

$$
\Omega(\omega)=\begin{bmatrix}0 & -\omega^T\\ \omega & -[\omega]_\times\end{bmatrix}.
$$

It is also useful to define the 4×3 matrix $\Xi(q)$, which satisfies $\Omega(\omega)q = \Xi(q)\,\omega$:

$$
\Xi(q)=\begin{bmatrix}-q_v^T\\ q_w I_3 + [q_v]_\times\end{bmatrix}
$$

**Discrete prediction** (step $\Delta t$, with $\phi = \|\hat\omega\|\Delta t$):

$$
\Phi_q = \cos\!\tfrac{\phi}{2}\, I_4 + \frac{\sin(\phi/2)}{\|\hat\omega\|}\,\Omega(\hat\omega)
$$

$$
\hat q_{k+1}^- = \Phi_q\,\hat q_k^+ \;\;(\text{then normalize}),\qquad
\hat b_{p,k+1}^- = \hat b_{p,k}^+,\qquad
\hat b_{h,k+1}^- = \hat b_{h,k}^+
$$

**State-transition Jacobian (10×10):**

$$
F_k=\begin{bmatrix}
\Phi_q & -\tfrac{\Delta t}{2}\,\Xi(\hat q_k) & 0_{4\times3}\\
0_{3\times4} & I_3 & 0_3\\
0_{3\times4} & 0_3 & I_3
\end{bmatrix}
$$

**Process noise:**

$$
Q_k=\begin{bmatrix}
\tfrac{\Delta t^2}{4}\,\Xi\,\Sigma_{gp}\,\Xi^T & 0 & 0\\
0 & \sigma_{bp}^2\Delta t\, I_3 & 0\\
0 & 0 & \sigma_{bh}^2\Delta t\, I_3
\end{bmatrix}
$$

**Covariance prediction:**

$$
P_{k+1}^- = F_k P_k^+ F_k^T + Q_k
$$

---

## 5. Measurement Updates

### 5a. Gyro cross-consistency (every sample, typically 1–2 kHz)

This form uses the measured relative rate. Its residual depends only on the biases:

$$
z_\omega = \omega_{p,m} - R_{PH}(\theta)\,\omega_{h,m} - J(\theta)\dot\theta_m
$$

$$
h_\omega(x) = b_p - R_{PH}(\theta)\,b_h
$$

$$
H_\omega = \begin{bmatrix} 0_{3\times4} & I_3 & -R_{PH}(\theta)\end{bmatrix}
$$

$$
R_\omega = \Sigma_{gp} + R_{PH}\Sigma_{gh}R_{PH}^T + J\,\Sigma_{\dot\theta}\,J^T
$$

Because $R_{PH}(\theta)$ changes as the joints move, $b_p$ and $b_h$ become separately observable. When the gimbal is stationary, only their difference is observable.

### 5b. Gravity reference (payload accelerometer, gated)

$$
h_a(q) = g\,R(q)^T e_3 = 2g\begin{bmatrix} q_xq_z - q_wq_y\\ q_yq_z + q_wq_x\\ \tfrac12(q_w^2-q_x^2-q_y^2+q_z^2)\end{bmatrix}
$$

$$
H_a=\Big[\;2g\begin{bmatrix}-q_y & q_z & -q_w & q_x\\ q_x & q_w & q_z & q_y\\ q_w & -q_x & -q_y & q_z\end{bmatrix}\;\;0_{3\times3}\;\;0_{3\times3}\Big]
$$

Apply this update only when both conditions hold:
- $\big|\,\|a_{p,m}\|-g\,\big| < \epsilon_a$, and
- the high-pass vibration energy is below a threshold.

Otherwise, inflate $R_a$ adaptively, for example:

$$
R_a = \Sigma_a\left(1 + \kappa\,\big|\|a_{p,m}\|-g\big|\right)
$$

### 5c. Standard EKF update (for either measurement)

$$
S = HP^-H^T + R,\qquad K = P^-H^TS^{-1}
$$

$$
\hat x^+ = \hat x^- + K\,(z - h(\hat x^-)),\qquad \hat q^+ \leftarrow \hat q^+/\|\hat q^+\|
$$

$$
P^+ = (I-KH)P^-(I-KH)^T + KRK^T \quad(\text{Joseph form})
$$

---

## 6. Outputs for Vibration Isolation

**Stabilization feedback** (payload inertial rate and attitude):

$$
\hat\omega_p = \omega_{p,m} - \hat b_p,\qquad \hat q
$$

**Base-disturbance feedforward** (handle vibration expressed in the payload frame):

$$
\hat d = R_{PH}(\theta)\,(\omega_{h,m} - \hat b_h)
$$

**Motor joint-rate command** that cancels the handle motion while tracking the desired rate $\omega_{\text{ref}}$:

$$
\dot\theta_{\text{cmd}} = J(\theta)^{-1}\big(\omega_{\text{ref}} - \hat d\big) + C(s)\,\big(\omega_{\text{ref}} - \hat\omega_p\big)
$$

The feedforward term $\hat d$ removes handle vibration before it appears on the payload gyro. This extends rejection beyond the feedback loop bandwidth. Unbiased $\hat b_h$ is what keeps this term from injecting drift.

---

## 7. Implementation Notes

- **Timing:** Hardware-sync both IMUs, or interpolate the handle samples to payload timestamps. A 1 ms skew at 100 Hz vibration produces about 0.6 rad/s of phase error in $\hat d$.
- **Observability:** Yaw has no absolute reference without a magnetometer. Expect $q$-yaw and $b_{p,z}$ (in world terms) to be only weakly observable. Keep $\sigma_{bp}$ small so the filter does not chase noise.
- **Quaternion covariance:** The 10×10 $P$ is rank-deficient along $q$. If numerical issues appear, switch to a multiplicative error-state EKF: keep the same 10 nominal states, but use a 9×9 error covariance with $\delta\theta \in \mathbb{R}^3$.
- **Starting values** (for MEMS gyros such as BMI088 or ICM-42688):

| Parameter | Value |
|---|---|
| $\sigma_{g}$ | ≈ 0.003–0.005 rad/s/√Hz |
| $\sigma_{b}$ | ≈ 1e-5 rad/s²/√Hz |
| $\sigma_{a}$ | ≈ 0.02–0.05 m/s² |
| Initial $P_0$, bias block | (0.01 rad/s)² |
| Initial $P_0$, quaternion block | 0.01 |

If you share the frame conventions or decisions from your earlier cycles, I can align the equations to them. That includes, for example, whether motor encoder rates are trusted or estimated.