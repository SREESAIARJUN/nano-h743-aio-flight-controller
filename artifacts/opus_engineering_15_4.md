# 10-State EKF for Dual-IMU Gimbal Vibration Isolation

I don't have the earlier design cycles, so this formulation is self-contained. Notation and assumptions are stated explicitly so they can be adapted to your existing conventions.

---

## 1. System Overview

| Sensor | Location | Signals | Role in filter |
|---|---|---|---|
| Payload IMU | Camera / payload frame **{P}** | gyro ω̃ₚ, accel ãₚ | Propagation input and gravity reference |
| Handle IMU | Handle frame **{H}** | gyro ω̃ₕ | Kinematic rate measurement |
| Joint encoders | Gimbal axes | θ, θ̇ | Maps between {H} and {P} |

**Goals**
- Estimate payload attitude.
- Estimate the bias-free payload rate ω̂ₚ for the feedback loop.
- Estimate the bias-free handle rate ω̂ₕ, the disturbance, for feedforward vibration rejection.

---

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \mathbf{b}_p \\ \mathbf{b}_h \end{bmatrix}
\in \mathbb{R}^{10},\qquad
\mathbf{q} = [q_0\; q_1\; q_2\; q_3]^T
$$

- **q**: unit quaternion, payload → world (4)
- **b_p**: payload gyro bias (3)
- **b_h**: handle gyro bias (3)

**Sensor models**

$$
\tilde{\boldsymbol\omega}_p = \boldsymbol\omega_p + \mathbf{b}_p + \mathbf{n}_{g,p}, \qquad
\tilde{\boldsymbol\omega}_h = \boldsymbol\omega_h + \mathbf{b}_h + \mathbf{n}_{g,h}
$$

$$
\tilde{\mathbf{a}}_p = R(\mathbf q)^T(\mathbf{a}^W - \mathbf g^W) + \mathbf n_a
$$

---

## 3. Process Model (Prediction)

### Continuous-time

$$
\dot{\mathbf q} = \tfrac12\,\Xi(\mathbf q)\,(\tilde{\boldsymbol\omega}_p - \mathbf b_p - \mathbf n_{g,p}),\qquad
\dot{\mathbf b}_p = \mathbf n_{b,p},\qquad
\dot{\mathbf b}_h = \mathbf n_{b,h}
$$

The quaternion kinematics matrix is:

$$
\Xi(\mathbf q)=\begin{bmatrix}-q_1 & -q_2 & -q_3\\ q_0 & -q_3 & q_2\\ q_3 & q_0 & -q_1\\ -q_2 & q_1 & q_0\end{bmatrix},
\qquad
\Omega(\boldsymbol\omega)=\begin{bmatrix}0 & -\boldsymbol\omega^T\\ \boldsymbol\omega & -[\boldsymbol\omega]_\times\end{bmatrix}
$$

### Discrete propagation (at gyro rate, e.g. 1 kHz)

Let $\bar{\boldsymbol\omega}_k = \tilde{\boldsymbol\omega}_{p,k} - \hat{\mathbf b}_{p,k}$.

$$
\hat{\mathbf q}_{k+1}^- = \Big[\cos\!\tfrac{\|\bar\omega\|\Delta t}{2}\,I_4 + \tfrac{\sin(\|\bar\omega\|\Delta t/2)}{\|\bar\omega\|}\,\Omega(\bar{\boldsymbol\omega}_k)\Big]\hat{\mathbf q}_k,
\qquad
\hat{\mathbf b}_{p,k+1}^-=\hat{\mathbf b}_{p,k},\quad
\hat{\mathbf b}_{h,k+1}^-=\hat{\mathbf b}_{h,k}
$$

### State-transition Jacobian

$$
F_k = \begin{bmatrix}
I_4 + \tfrac{\Delta t}{2}\Omega(\bar{\boldsymbol\omega}_k) & -\tfrac{\Delta t}{2}\Xi(\hat{\mathbf q}_k) & 0_{4\times3}\\
0_{3\times4} & I_3 & 0_3\\
0_{3\times4} & 0_3 & I_3
\end{bmatrix}
$$

### Process noise

With $G_q = \tfrac{\Delta t}{2}\Xi(\hat{\mathbf q}_k)$:

$$
Q_k = \operatorname{blkdiag}\!\Big(G_q\,\sigma_{g,p}^2 I_3\,G_q^T,\;\; \sigma_{b,p}^2\Delta t\, I_3,\;\; \sigma_{b,h}^2\Delta t\, I_3\Big)
$$

### Covariance prediction

$$
P_{k+1}^- = F_k P_k F_k^T + Q_k
$$

---

## 4. Measurement Models

### 4.1 Gravity direction (payload accelerometer, 100–200 Hz)

Use the normalized accelerometer $\mathbf z_1 = \tilde{\mathbf a}_p / \|\tilde{\mathbf a}_p\|$, with the world z-axis pointing up.

$$
h_1(\mathbf x) = R(\mathbf q)^T\hat{\mathbf z}=
\begin{bmatrix} 2(q_1q_3 - q_0q_2)\\ 2(q_2q_3 + q_0q_1)\\ q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix}
$$

$$
H_1 = \Big[\;2\begin{bmatrix}-q_2 & q_3 & -q_0 & q_1\\ q_1 & q_0 & q_3 & q_2\\ q_0 & -q_1 & -q_2 & q_3\end{bmatrix}\;\Big|\; 0_{3\times3}\;\Big|\;0_{3\times3}\Big]
$$

**Adaptive noise.** This de-weights the accelerometer during operator walking or shake:

$$
R_1 = \big(\sigma_a^2 + k_a\,(\|\tilde{\mathbf a}_p\| - g)^2\big) I_3
$$

Skip the update entirely if $\big|\|\tilde{\mathbf a}_p\|-g\big| > \epsilon_a$.

### 4.2 Handle-gyro kinematic rate constraint (1 kHz) — the fusion core

The gimbal kinematics relate the two frames:

$$
\boldsymbol\omega_p = R_{PH}(\boldsymbol\theta)\,\boldsymbol\omega_h + J(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
\;\;\Rightarrow\;\;
\boldsymbol\omega_h = R_{HP}(\boldsymbol\theta)\big(\boldsymbol\omega_p - J(\boldsymbol\theta)\dot{\boldsymbol\theta}\big)
$$

Here $J(\boldsymbol\theta)$ is the joint-rate Jacobian, which maps encoder rates to relative angular velocity expressed in {P}.

**Measurement:** $\mathbf z_2 = \tilde{\boldsymbol\omega}_h$

$$
h_2(\mathbf x) = R_{HP}(\boldsymbol\theta)\big(\tilde{\boldsymbol\omega}_p - \mathbf b_p - J(\boldsymbol\theta)\dot{\boldsymbol\theta}\big) + \mathbf b_h
$$

$$
H_2 = \big[\,0_{3\times4}\;\big|\; -R_{HP}(\boldsymbol\theta)\;\big|\; I_3\,\big]
$$

$$
R_2 = \sigma_{g,h}^2 I_3 + R_{HP}\,\sigma_{g,p}^2\,R_{HP}^T + J\,\Sigma_{\dot\theta}\,J^T
$$

**Note on correlation.** $\tilde{\boldsymbol\omega}_p$ appears in both propagation and $h_2$, which creates correlation between process and measurement noise. Inflating $R_2$ as shown is the practical fix. For a rigorous treatment, augment the cross-covariance term $S = G_q\sigma_{g,p}^2(-R_{HP})^T$ into the gain.

### 4.3 Quaternion-norm pseudo-measurement (optional, every cycle)

$$
z_3 = 1,\qquad h_3 = \mathbf q^T\mathbf q,\qquad H_3 = [\,2\mathbf q^T\;\; 0_{1\times6}\,],\qquad R_3 \approx 10^{-6}
$$

---

## 5. Update Equations (for each available measurement *i*)

$$
\mathbf y = \mathbf z_i - h_i(\hat{\mathbf x}^-)
$$

$$
S = H_i P^- H_i^T + R_i
$$

$$
K = P^- H_i^T S^{-1}
$$

$$
\hat{\mathbf x}^+ = \hat{\mathbf x}^- + K\mathbf y
$$

$$
P^+ = (I-KH_i)P^-(I-KH_i)^T + K R_i K^T \quad\text{(Joseph form)}
$$

Then normalize: $\hat{\mathbf q}^+ \leftarrow \hat{\mathbf q}^+/\|\hat{\mathbf q}^+\|$.

**Innovation gating.** Reject the update if $\mathbf y^T S^{-1}\mathbf y > \chi^2_{3,\,0.99} = 11.34$. This guards against encoder glitches and impacts.

---

## 6. Outputs for Vibration Isolation

| Output | Equation | Use |
|---|---|---|
| Payload rate | $\hat{\boldsymbol\omega}_p = \tilde{\boldsymbol\omega}_p - \hat{\mathbf b}_p$ | Inner rate-loop feedback |
| Handle disturbance rate | $\hat{\boldsymbol\omega}_h = \tilde{\boldsymbol\omega}_h - \hat{\mathbf b}_h$ | Feedforward |
| Attitude | $\hat{\mathbf q}$ | Outer angle loop and horizon lock |

**Feedforward joint-rate command.** This cancels handle motion before it reaches the payload:

$$
\dot{\boldsymbol\theta}_{ff} = -J(\boldsymbol\theta)^{-1} R_{PH}(\boldsymbol\theta)\,\hat{\boldsymbol\omega}_h
$$

Guard the inverse near gimbal lock. Use a damped pseudo-inverse, $J^T(JJ^T+\lambda I)^{-1}$.

---

## 7. Observability Notes

- **Roll/pitch and $b_{p,x}, b_{p,y}$** are observable from gravity.
- **$b_h - R_{HP}\,b_p$** is directly observed by $h_2$. As the gimbal angles change, $R_{HP}$ varies, which separates $b_h$ from $b_p$.
- **Yaw and $b_{p,z}$ (world-vertical component)** are unobservable without a heading reference. Options:
  - Add a magnetometer, which is often unreliable near gimbal motors.
  - Apply a zero-rate update when the system is stationary.
  - Accept yaw drift if the gimbal operates in follow mode, which is typical.

---

## 8. Initial Tuning Values (MEMS-class IMUs, e.g. ICM-42688 / BMI088)

| Par