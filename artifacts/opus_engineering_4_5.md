# 10-State EKF for Payload/Handle IMU Fusion in Gimbal Vibration Isolation

I don't have your project's specific Cycle 4 conventions, so this is a self-consistent formulation. Rename frames and symbols to match your documentation.

---

## 1. Frames and Sensors

| Symbol | Meaning |
|---|---|
| **W** | World (NED or ENU, gravity along ±z) |
| **C** | Camera/payload body frame, which carries the payload IMU |
| **H** | Handle frame, which carries the handle IMU |
| θ, θ̇ | Gimbal joint angles and rates (from encoders) |
| R_HC(θ) | Rotation from C to H, computed by forward kinematics of the joint chain |

**Sensors (sampled synchronously at Δt):**
- Payload gyro: ω̃_c = ω_c + b_c + n_c
- Payload accelerometer: ã_c
- Handle gyro: ω̃_h = ω_h + b_h + n_h

---

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q}_{WC} \\ \mathbf{b}_c \\ \mathbf{b}_h \end{bmatrix}
\in \mathbb{R}^{10},
\qquad
\mathbf{q}=[q_w,\,q_x,\,q_y,\,q_z]^T
$$

- **q_WC (4):** camera attitude quaternion
- **b_c (3):** payload gyro bias
- **b_h (3):** handle gyro bias

The payload gyro drives the prediction step. The handle gyro and payload accelerometer are used as measurements. The handle gyro enters through the gimbal kinematics, which makes both biases observable and yields a clean estimate of handle (disturbance) motion.

---

## 3. Process Model

**Bias-corrected camera rate:**
$$
\hat{\boldsymbol{\omega}}_c = \tilde{\boldsymbol{\omega}}_c - \hat{\mathbf{b}}_c
$$

**Quaternion propagation (exact zero-order hold):**
$$
\mathbf{q}_{k+1} = \underbrace{\left[\cos\!\left(\tfrac{\|\hat\omega\|\Delta t}{2}\right) I_4 + \frac{\sin\!\left(\tfrac{\|\hat\omega\|\Delta t}{2}\right)}{\|\hat\omega\|}\,\Omega(\hat\omega)\right]}_{\Phi_q}\mathbf{q}_k
$$

$$
\Omega(\omega)=\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z\\
\omega_x & 0 & \omega_z & -\omega_y\\
\omega_y & -\omega_z & 0 & \omega_x\\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

When ‖ω̂‖ is near zero, use Φ_q ≈ I + ½Ω(ω̂)Δt.

**Biases (random walk):**
$$
\mathbf{b}_{c,k+1} = \mathbf{b}_{c,k} + \mathbf{w}_{bc},\qquad
\mathbf{b}_{h,k+1} = \mathbf{b}_{h,k} + \mathbf{w}_{bh}
$$

### State Transition Jacobian F (10×10)

$$
F_k = \begin{bmatrix}
\Phi_q & -\tfrac{\Delta t}{2}\,\Xi(\mathbf{q}_k) & 0_{4\times3}\\
0_{3\times4} & I_3 & 0_3\\
0_{3\times4} & 0_3 & I_3
\end{bmatrix},
\qquad
\Xi(\mathbf{q}) = \begin{bmatrix}
-q_x & -q_y & -q_z\\
q_w & -q_z & q_y\\
q_z & q_w & -q_x\\
-q_y & q_x & q_w
\end{bmatrix}
$$

### Process Noise Q (10×10)

$$
Q_k = \begin{bmatrix}
\tfrac{\Delta t^2}{4}\,\Xi\,(\sigma_{gc}^2 I_3)\,\Xi^T & 0 & 0\\
0 & \sigma_{bc}^2\Delta t\, I_3 & 0\\
0 & 0 & \sigma_{bh}^2\Delta t\, I_3
\end{bmatrix}
$$

Here σ_gc is the payload gyro white noise, and σ_bc and σ_bh are the bias random-walk densities. All come from the datasheet or an Allan variance analysis.

### Prediction Step

$$
\hat{\mathbf{x}}_{k+1}^- = f(\hat{\mathbf{x}}_k^+, \tilde\omega_c),\qquad
P_{k+1}^- = F_k P_k^+ F_k^T + Q_k
$$

Normalize the quaternion after prediction: q ← q / ‖q‖.

---

## 4. Measurement Models

### 4a. Payload Accelerometer (gravity reference, 3 outputs)

$$
\mathbf{z}_a = \tilde{\mathbf{a}}_c \approx C(\mathbf{q})^T\mathbf{g}_W + \mathbf{v}_a,\qquad
h_a(\mathbf{q}) = g\begin{bmatrix}2(q_xq_z - q_wq_y)\\ 2(q_yq_z + q_wq_x)\\ q_w^2-q_x^2-q_y^2+q_z^2\end{bmatrix}
$$

This uses g_W = [0, 0, g]^T. Flip the sign convention to match your accelerometer.

$$
H_a = \Big[\; 2g\begin{bmatrix}
-q_y & q_z & -q_w & q_x\\
q_x & q_w & q_z & q_y\\
q_w & -q_x & -q_y & q_z
\end{bmatrix}\;\Big|\; 0_{3\times3}\;\Big|\; 0_{3\times3}\Big]
$$

**Adaptive noise (essential under walking or vibration):**
$$
R_a = \sigma_a^2 I_3\left(1 + \kappa\,\big|\,\|\tilde{\mathbf a}_c\| - g\,\big|\right)
$$

Skip the update entirely if |‖ã‖ − g| exceeds a gate (for example 0.15 g).

### 4b. Handle Gyro via Gimbal Kinematics (3 outputs), the Fusion Step

Rigid-body kinematics through the joint chain gives

$$
\boldsymbol\omega_c = R_{CH}\,\boldsymbol\omega_h + \boldsymbol\omega_{rel}^C(\theta,\dot\theta)
$$

where ω_rel^C = Σᵢ θ̇ᵢ · (axis i expressed in C).

Solving for the handle rate gives the predicted handle-gyro reading:

$$
h_h(\mathbf{x}) = R_{HC}(\theta)\big(\tilde\omega_c - \mathbf b_c - \omega^C_{rel}\big) + \mathbf b_h
$$

$$
H_h = \big[\,0_{3\times4}\;\big|\; -R_{HC}(\theta)\;\big|\; I_3\,\big]
$$

**Measurement noise** must account for the payload gyro noise and encoder-rate noise, since ω̃_c appears in both the process and this measurement:

$$
R_h = \sigma_{gh}^2 I_3 + \sigma_{gc}^2 I_3 + R_{HC}\,\Sigma_{\dot\theta,C}\,R_{HC}^T
$$

### 4c. Update (sequential or stacked, z = [z_a; z_h])

$$
\begin{aligned}
\mathbf{y} &= \mathbf z - h(\hat{\mathbf x}^-)\\
S &= HP^-H^T + R\\
K &= P^-H^T S^{-1}\\
\hat{\mathbf x}^+ &= \hat{\mathbf x}^- + K\mathbf y,\quad \mathbf q \leftarrow \mathbf q/\|\mathbf q\|\\
P^+ &= (I-KH)P^-(I-KH)^T + KRK^T \quad\text{(Joseph form)}
\end{aligned}
$$

Add a χ² innovation gate (yᵀS⁻¹y < threshold) to reject shocks and saturated samples.

---

## 5. Outputs for Vibration Isolation

| Estimate | Use |
|---|---|
| q̂_WC | Attitude error for the outer stabilization loop: q_err = q_ref⁻¹ ⊗ q̂_WC |
| ω̂_c = ω̃_c − b̂_c | Rate feedback for the inner loop (low-latency, bias-free) |
| ω̂_h = ω̃_h − b̂_h | **Disturbance feedforward**: the handle motion to be rejected |

**Feedforward joint-rate command** (cancels handle motion before it reaches the camera):
$$
\dot\theta_{ff} = -J(\theta)^{-1} R_{CH}\,\hat\omega_h
$$

Here J(θ) maps joint rates to ω_rel^C. The total command is θ̇_cmd = θ̇_ff + PID(q_err, ω̂_c). Feedforward handles hand tremor and walking (about 1–15 Hz), while feedback cleans up the residual.

---

## 6. Practical Notes

1. **Observability.** Roll and pitch are corrected by gravity, but yaw and the yaw component of b_c are unobservable without a magnetometer or vision. Expect slow yaw drift, and keep σ_bc small in yaw or add a heading reference.
2. **Bias separation.** Only b_h − R_HC b_c is observed directly by the handle measurement. Joint motion (changing R_HC) and the gravity update decorrelate the two biases.
3. **Covariance singularity.** The additive 4-element quaternion makes P rank-deficient along q. For robustness, move to a 9-state error-state (multiplicative) EKF with δθ in place of δq, while keeping the same 10-element nominal state.
4. **Timing.** Timestamp-align both IMUs and the encoders to within 1 ms. At 10 Hz vibration, a 1 ms skew introduces roughly 6% rate error in the feedforward path.
5. **Rates.** Run the EKF at 500 Hz to 1 kHz. Pre-filter the gyros with an anti-alias filter only, and notch out motor-PWM or structural resonances before they reach the filter.
6. **Lever arm.** Gyros need no lever-arm correction (angular rate is uniform across a rigid body), but the accelerometer does if it sits off the rotation center. Gating handles most of that error.