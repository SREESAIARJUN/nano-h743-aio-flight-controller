# 10-State EKF for Payload/Handle Gyro Fusion in Gimbal Vibration Isolation

I don't have the details of your "Cycle 29" requirements, so this formulation uses standard 3-axis handheld-gimbal conventions. Adjust the frame conventions and noise values to match your hardware.

---

## 1. Frames and Sensors

| Symbol | Meaning |
|---|---|
| **W** | World (NED or ENU, gravity along z) |
| **P** | Payload (camera) body frame, where payload IMU is mounted |
| **H** | Handle body frame, where handle IMU is mounted |
| θ = [θ₁ θ₂ θ₃]ᵀ | Joint angles from motor encoders (yaw–roll–pitch chain) |
| R_PH(θ) | Rotation from handle frame to payload frame (forward kinematics) |
| J(θ) | Joint-rate Jacobian, mapping θ̇ to relative angular velocity expressed in P |

**Sensor models:**

$$
\begin{aligned}
\tilde{\omega}_p &= \omega_p + b_p + n_{gp} \\
\tilde{\omega}_h &= \omega_h + b_h + n_{gh} \\
\tilde{a}_p &= R_{WP}^{T}\, g_W + a_{p,\text{lin}} + n_{ap}
\end{aligned}
$$

**Rigid-body kinematic constraint** (the key to the fusion):

$$
\omega_p = R_{PH}(\theta)\,\omega_h + J(\theta)\,\dot{\theta}
$$

---

## 2. State Vector (10 states)

$$
x = \begin{bmatrix} q \\ b_p \\ b_h \end{bmatrix} \in \mathbb{R}^{10},
\qquad
q = [q_w\; q_x\; q_y\; q_z]^T \;(\text{payload attitude } R_{WP}),\quad
b_p, b_h \in \mathbb{R}^3
$$

- **q (4):** payload attitude quaternion.
- **b_p (3):** payload gyro bias.
- **b_h (3):** handle gyro bias.

The handle gyro bias must be estimated because the handle rate is used as a disturbance feedforward. An unestimated bias there turns into a slow drift that the stabilization loop has to fight.

---

## 3. Process Model (payload gyro as control input)

Define the bias-corrected payload rate:

$$
\hat{\omega} = \tilde{\omega}_p - b_p
$$

The continuous-time dynamics are:

$$
\dot{q} = \tfrac{1}{2}\,\Omega(\hat{\omega})\,q = \tfrac{1}{2}\,\Xi(q)\,\hat{\omega},
\qquad
\dot{b}_p = w_{bp},
\qquad
\dot{b}_h = w_{bh}
$$

where:

$$
\Omega(\omega)=\begin{bmatrix}0&-\omega^T\\ \omega&-[\omega]_\times\end{bmatrix},
\qquad
\Xi(q)=\begin{bmatrix}-q_v^T\\ q_w I_3+[q_v]_\times\end{bmatrix}\;(4\times3)
$$

### Discrete Propagation (step Δt)

Use the exact zeroth-order-hold quaternion update:

$$
q_{k+1}^- = \left[\cos\!\left(\tfrac{\|\hat\omega\|\Delta t}{2}\right) I_4 + \frac{\sin\!\left(\tfrac{\|\hat\omega\|\Delta t}{2}\right)}{\|\hat\omega\|}\,\Omega(\hat\omega)\right] q_k^+
$$

The biases are held constant:

$$
b_{p,k+1}^- = b_{p,k}^+, \qquad b_{h,k+1}^- = b_{h,k}^+
$$

### Jacobians

The continuous-time state Jacobian is:

$$
F=\begin{bmatrix}
\tfrac12\Omega(\hat\omega) & -\tfrac12\Xi(q) & 0_{4\times3}\\
0_{3\times4} & 0_3 & 0_3\\
0_{3\times4} & 0_3 & 0_3
\end{bmatrix},
\qquad
\Phi_k \approx I_{10} + F\,\Delta t
$$

The noise input matrix and continuous noise covariance are:

$$
G=\begin{bmatrix}-\tfrac12\Xi(q) & 0 & 0\\ 0 & I_3 & 0\\ 0&0&I_3\end{bmatrix},
\qquad
Q_c=\operatorname{diag}(\sigma_{gp}^2 I_3,\;\sigma_{bp}^2 I_3,\;\sigma_{bh}^2 I_3)
$$

Discretize and propagate:

$$
Q_k \approx G\,Q_c\,G^T\Delta t,
\qquad
P_{k+1}^- = \Phi_k P_k^+ \Phi_k^T + Q_k
$$

---

## 4. Measurement Models

### (a) Gravity Reference from the Payload Accelerometer

With $g_W=[0,0,g]^T$ (specific-force convention, so the sensor reads +g when at rest):

$$
h_1(x) = R(q)^T g_W = g\begin{bmatrix}
2(q_xq_z - q_wq_y)\\
2(q_yq_z + q_wq_x)\\
q_w^2-q_x^2-q_y^2+q_z^2
\end{bmatrix}
$$

The measurement Jacobian is:

$$
H_1=\Big[\;2g\begin{bmatrix}
-q_y & q_z & -q_w & q_x\\
q_x & q_w & q_z & q_y\\
q_w & -q_x & -q_y & q_z
\end{bmatrix}\;\Big|\;0_{3\times3}\;\Big|\;0_{3\times3}\Big]
$$

**Gating:** apply this update only when

$$
\big|\,\|\tilde a_p\| - g\,\big| < \varepsilon_a
$$

Otherwise, inflate R₁ adaptively, for example:

$$
R_1 = \sigma_a^2 I + k_a\big(\|\tilde a_p\|-g\big)^2 I
$$

This rejects walking shocks and linear acceleration.

### (b) Handle/Payload Rate Consistency (the IMU fusion)

Substitute the sensor models into the kinematic constraint and form the residual pseudo-measurement:

$$
z_2 = \tilde\omega_p - R_{PH}(\theta)\,\tilde\omega_h - J(\theta)\,\dot\theta_{enc}
$$

This gives a model that is linear in the biases:

$$
h_2(x) = b_p - R_{PH}(\theta)\,b_h,
\qquad
H_2 = \big[\,0_{3\times4}\;\big|\;I_3\;\big|\;-R_{PH}(\theta)\,\big]
$$

The measurement noise combines both gyros, the encoders, and an allowance for non-rigidity:

$$
R_2 = \sigma_{gp}^2 I + R_{PH}\,\sigma_{gh}^2 I\,R_{PH}^T + J\,\Sigma_{\dot\theta}\,J^T + R_{flex}
$$

- **R_flex** covers structural flex and backlash, which violate the rigid-body assumption at vibration frequencies.
- Make R_flex large, or low-pass z₂ (e.g., 5–20 Hz) before the update, so that only the quasi-static bias information is used.
- As the gimbal rotates through different θ, R_PH(θ) changes. This lets the filter separate the individual components of b_p and b_h, rather than just their difference.

---

## 5. Update Step (for each available measurement i)

$$
\begin{aligned}
y &= z_i - h_i(x^-)\\
S &= H_i P^- H_i^T + R_i\\
K &= P^- H_i^T S^{-1}\\
x^+ &= x^- + K y\\
P^+ &= (I-KH_i)P^-(I-KH_i)^T + K R_i K^T \quad\text{(Joseph form)}
\end{aligned}
$$

Then renormalize the quaternion:

$$
q \leftarrow \frac{q}{\|q\|}
$$

Optionally, project P onto the tangent space of the unit quaternion:

$$
P \leftarrow \Pi P \Pi^T,\qquad \Pi = I - \tfrac{q q^T}{\|q\|^2} \;\text{(on the q block)}
$$

This removes the redundant norm direction. If numerical rank issues appear, switch to a 9-state error-state (multiplicative) EKF using the same models.

---

## 6. Outputs to the Stabilization Controller

| Output | Use |
|---|---|
| $\hat\omega_p = \tilde\omega_p - \hat b_p$ | Rate-loop feedback (inner loop, ~1 kHz) |
| $\hat\omega_h^{P} = R_{PH}(\tilde\omega_h - \hat b_h)$ | **Disturbance feedforward**: command motors to cancel handle motion before it reaches the payload |
| $\hat q$ | Attitude-loop feedback (outer loop) |

A typical feedforward torque command is:

$$
\tau_{ff} = -J^{-1}(\theta)\,\hat\omega_h^{P}\;\text{(rate sense)}
$$

shaped with notch filters at the motor/structure resonances.

---

## 7. Suggested Tuning Starting Points

| Parameter | Typical (MEMS gyro, 1 kHz) |
|---|---|
| σ_gp, σ_gh | 0.005–0.01 rad/s/√Hz |
| σ_bp, σ_bh (bias random walk) | 1e-5 – 1e-4 rad/s²/√Hz |
| σ_a | 0.05–0.2 m/s² |
| ε_a (accel gate) | 0.5–1.0 m/s² |
| Propagation rate | 500 Hz–1 kHz |
| Accel update rate | 100–200 Hz |
| Rate-consistency update rate | 50–100 Hz (after low-pass) |
| P₀ | q: 1e-2; b: (0.02 rad/s)² |

---

## 8. Observability Notes

- **Roll/pitch** are observable from gravity.
- **Yaw** is unobservable without a magnetometer or heading reference, so its covariance grows. This is acceptable for follow-mode gimbals.
- **b_p − R_PH b_h** is always observable through measurement (b).
- **b_p and b_h individually** become observable through two routes: gravity (which pins the payload roll/pitch bias), and variation of θ over time.

If you share the Cycle 29 specifics (sensor ODR, joint ordering, vibration bands), I can tighten the noise models and notch/low-pass placement.