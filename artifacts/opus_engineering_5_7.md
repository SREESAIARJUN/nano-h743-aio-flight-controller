# 10-State EKF for Payload/Handle IMU Fusion in Gimbal Vibration Isolation

I don't have the earlier cycles of this design, so I've defined the state, frames and assumptions explicitly. Map them onto your Cycle 1–4 conventions where they differ.

---

## 1. Frames, Sensors and Assumptions

| Symbol | Meaning |
|---|---|
| N | Navigation frame (NED or ENU, gravity along the z axis) |
| P | Camera payload frame (payload IMU is mounted here) |
| H | Handle frame (handle IMU is mounted here) |
| θ ∈ ℝ³ | Gimbal joint angles from the motor encoders (yaw, roll, pitch) |
| R_PH(θ) | Rotation from H to P, from the gimbal forward kinematics |
| J(θ) | Joint-rate Jacobian, which maps θ̇ to the payload angular rate in P |

The design rests on three assumptions:

- **Kinematic chain linking the two IMUs.** The payload rate is the handle rate carried through the gimbal plus the motor-induced relative rate:
  ω_P = R_PH(θ)·ω_H + J(θ)·θ̇
- **Handle gyro bias is calibrated out at power-up** (stationary average). This keeps the filter at 10 states. A 13-state variant adds b_H.
- **Encoders are treated as known inputs.** They are high-rate and low-noise compared with the MEMS gyros.

---

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \boldsymbol{\omega}_H \\ \mathbf{b}_P \end{bmatrix}
\quad
\begin{array}{l}
\mathbf{q}=[q_0,q_1,q_2,q_3]^T \text{ (payload attitude, P→N, scalar first)} \;(4)\\
\boldsymbol{\omega}_H \text{ (handle angular velocity in H, the vibration disturbance)} \;(3)\\
\mathbf{b}_P \text{ (payload gyro bias)} \;(3)
\end{array}
$$

The handle rate ω_H is what the filter isolates. Its estimate drives the disturbance feedforward to the motors (Section 7).

---

## 3. Process Model (continuous time)

**Attitude kinematics:**
$$
\dot{\mathbf{q}} = \tfrac12\,\Omega(\boldsymbol{\omega}_P)\,\mathbf{q},\qquad
\boldsymbol{\omega}_P = R_{PH}(\theta)\,\boldsymbol{\omega}_H + J(\theta)\dot\theta
$$

$$
\Omega(\boldsymbol\omega)=\begin{bmatrix}
0&-\omega_x&-\omega_y&-\omega_z\\
\omega_x&0&\omega_z&-\omega_y\\
\omega_y&-\omega_z&0&\omega_x\\
\omega_z&\omega_y&-\omega_x&0\end{bmatrix}
$$

**Handle rate** is modelled as a first-order Gauss–Markov process. The correlation time τ_H is set from the dominant vibration and hand-motion bandwidth.
$$
\dot{\boldsymbol\omega}_H = -\frac{1}{\tau_H}\boldsymbol\omega_H + \mathbf{w}_\omega
$$

**Payload gyro bias** is a random walk:
$$
\dot{\mathbf b}_P = \mathbf w_b
$$

The process noise covariance is
$$
Q_c=\mathrm{diag}(\sigma_q^2 I_4,\;\sigma_\omega^2 I_3,\;\sigma_b^2 I_3),
$$
where σ_q is small and only absorbs kinematic and encoder errors.

---

## 4. State Jacobian F = ∂f/∂x

$$
F=\begin{bmatrix}
\tfrac12\Omega(\boldsymbol\omega_P) & \tfrac12\,\Xi(\mathbf q)\,R_{PH}(\theta) & 0_{4\times3}\\
0_{3\times4} & -\tfrac{1}{\tau_H}I_3 & 0_{3\times3}\\
0_{3\times4} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix}
$$

Here Ξ(q) is the 4×3 matrix satisfying Ω(ω)q = Ξ(q)ω:

$$
\Xi(\mathbf q)=\begin{bmatrix}
-q_1&-q_2&-q_3\\
q_0&-q_3&q_2\\
q_3&q_0&-q_1\\
-q_2&q_1&q_0\end{bmatrix}
$$

---

## 5. Discrete Prediction (step Δt, typically at the IMU rate of 1–8 kHz)

**Quaternion propagation** uses the exact exponential, followed by normalisation:
$$
\mathbf q_{k+1}^- = \Big[\cos\tfrac{\|\boldsymbol\omega_P\|\Delta t}{2}\,I_4+\tfrac{\sin(\|\boldsymbol\omega_P\|\Delta t/2)}{\|\boldsymbol\omega_P\|}\,\Omega(\boldsymbol\omega_P)\Big]\mathbf q_k^+,\qquad \mathbf q\leftarrow \mathbf q/\|\mathbf q\|
$$

**Handle rate and bias:**
$$
\boldsymbol\omega_{H,k+1}^- = e^{-\Delta t/\tau_H}\,\boldsymbol\omega_{H,k}^+,\qquad
\mathbf b_{P,k+1}^- = \mathbf b_{P,k}^+
$$

**Covariance:**
$$
\Phi_k \approx I_{10}+F_k\Delta t,\qquad Q_k\approx Q_c\Delta t
$$
$$
P_{k+1}^- = \Phi_k P_k^+ \Phi_k^T + Q_k
$$

---

## 6. Measurement Models

Apply the updates sequentially, one block at a time, each with its own rate and gating.

### (a) Payload gyro (3)
$$
\mathbf z_P = R_{PH}(\theta)\boldsymbol\omega_H + J(\theta)\dot\theta + \mathbf b_P + \mathbf v_P
$$
$$
H_P = \begin{bmatrix}0_{3\times4} & R_{PH}(\theta) & I_3\end{bmatrix},\qquad R_P=\sigma_{gP}^2 I_3
$$

### (b) Handle gyro (3)
$$
\mathbf z_H = \boldsymbol\omega_H + \mathbf v_H,\qquad
H_H=\begin{bmatrix}0_{3\times4} & I_3 & 0_{3\times3}\end{bmatrix},\qquad R_H=\sigma_{gH}^2 I_3
$$

Taken together, (a) and (b) make b_P observable. The handle gyro pins ω_H, so any residual in the payload gyro beyond the kinematic prediction is attributed to the bias.

### (c) Payload accelerometer (3), for gravity reference and roll/pitch drift correction

With g_N = [0, 0, g]^T:
$$
\mathbf z_a = R(\mathbf q)^T\mathbf g_N + \mathbf v_a
= g\begin{bmatrix}2(q_1q_3-q_0q_2)\\ 2(q_2q_3+q_0q_1)\\ q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix}+\mathbf v_a
$$
$$
H_a=\begin{bmatrix}
g\begin{bmatrix}
-2q_2 & 2q_3 & -2q_0 & 2q_1\\
2q_1 & 2q_0 & 2q_3 & 2q_2\\
2q_0 & -2q_1 & -2q_2 & 2q_3
\end{bmatrix} & 0_{3\times3} & 0_{3\times3}\end{bmatrix}
$$

**Gating:** only apply this update when | ‖a‖ − g | < ε_a, so the filter rejects walking and vibration-induced linear acceleration. As an alternative, inflate R_a in proportion to that deviation.

Yaw is unobservable from these measurements. Add a magnetometer, or rely on the yaw-follow mode, if absolute heading matters.

### Update (for each block i ∈ {P, H, a})

$$
\mathbf y = \mathbf z_i - h_i(\mathbf x^-)
$$
$$
S = H_i P^- H_i^T + R_i
$$
$$
K = P^- H_i^T S^{-1}
$$
$$
\mathbf x^+ = \mathbf x^- + K\mathbf y,\qquad \mathbf q^+\leftarrow \mathbf q^+/\|\mathbf q^+\|
$$
$$
P^+ = (I-KH_i)P^-(I-KH_i)^T + K R_i K^T \quad\text{(Joseph form)}
$$

**Outlier rejection:** reject an update if the normalised innovation squared, yᵀS⁻¹y, exceeds the 3-DOF χ² threshold (about 11.3 at 99%).

---

## 7. Outputs for the Stabilisation Loop

**Estimated payload inertial rate**, which feeds the rate loop:
$$
\hat{\boldsymbol\omega}_P = \mathbf z_P - \hat{\mathbf b}_P
$$

**Disturbance feedforward.** The goal is to cancel the handle motion before it reaches the payload, i.e. drive R_PH ω_H + Jθ̇ toward ω_P^ref:
$$
\dot\theta_{ff} = J(\theta)^{-1}\big[\boldsymbol\omega_P^{ref} - R_{PH}(\theta)\,\hat{\boldsymbol\omega}_H\big]
$$
Avoid gimbal-lock regions where J is singular, and saturate near singularities.

**Attitude error** for the outer loop is taken from q̂ against the commanded attitude.

---

## 8. Tuning Notes

- **τ_H:** start around 0.01–0.05 s for handheld vibration (10–100 Hz content). If the vibration is narrowband, for example walking at about 2 Hz plus a motor or drone resonance, replace the Gauss–Markov model with a second-order resonator. That raises the filter to 13 states.
- **Noise ratios:** keep σ_gH² ≪ σ_ω²Δt·(steps) so the filter tracks ω_H tightly. A large σ_b slows bias convergence but tolerates temperature drift.
- **Covariance rank:** the 4-state quaternion makes P rank-deficient along the norm direction. For better numerical behaviour, switch to the error-state (multiplicative) form. That gives 9 error states (δθ, δω_H, δb_P) with an identical structure, where Ξ(q)/2 is replaced by I₃ for attitude.
- **Time alignment:** both IMUs and the encoders must be timestamp-synchronised. A 1 ms skew at 50 Hz vibration is roughly 18° of phase error and degrades the feedforward significantly.