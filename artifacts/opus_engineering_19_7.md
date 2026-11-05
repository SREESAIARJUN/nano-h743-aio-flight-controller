# 10-State EKF for Payload/Handle IMU Fusion (Gimbal Vibration Isolation)

I don't have the context for "Cycle 19", so this is a self-contained formulation with the assumptions stated explicitly. Adjust the frame conventions, noise values and joint kinematics to match your project.

---

## 1. Problem Setup

**Frames**
- **n**: navigation/world frame (z-up, gravity **g** = [0, 0, g]ᵀ).
- **p**: camera payload body frame (payload IMU).
- **h**: handle body frame (handle IMU).

**Sensors**

| Sensor | Signal | Rate (typical) |
|---|---|---|
| Payload gyro | ω̃ₚ | 1–2 kHz |
| Payload accelerometer | ãₚ | 1–2 kHz |
| Handle gyro | ω̃ₕ | 1–2 kHz |
| Joint encoders | θ, θ̇ (3 axes) | ≥ 1 kHz |

**Kinematic coupling.** The payload rate equals the handle rate rotated into the payload frame, plus the gimbal joint contribution:

$$
\boldsymbol\omega_p = \mathbf R_{ph}(\boldsymbol\theta)\,\boldsymbol\omega_h + \mathbf J(\boldsymbol\theta)\,\dot{\boldsymbol\theta}
$$

Here R_ph(θ) is the handle→payload rotation from the joint angles, and J(θ) is the gimbal joint Jacobian. For yaw–roll–pitch ordering, J's columns are the joint axes expressed in frame p.

---

## 2. State Vector (10 states)

$$
\mathbf x = \begin{bmatrix} \mathbf q \\ \mathbf b_p \\ \boldsymbol\omega_h \end{bmatrix}
\in \mathbb R^{10},
\qquad
\begin{aligned}
\mathbf q &= [q_0\; q_1\; q_2\; q_3]^T &&\text{payload attitude (p→n)} &(4)\\
\mathbf b_p &&&\text{payload gyro bias} &(3)\\
\boldsymbol\omega_h &&&\text{true handle angular velocity (disturbance)} &(3)
\end{aligned}
$$

**Design rationale**
- ω_h is the disturbance to be rejected. Estimating it gives a clean, low-lag feedforward signal.
- The payload gyro is used as a *measurement*, not as the propagation input. This avoids using the same data twice and makes **b_p** observable through the two-IMU coupling.
- The handle gyro bias **b_h** is calibrated at startup (stationary) and removed. If it drifts significantly, extend to 13 states.

---

## 3. Process Model

### Continuous time

$$
\begin{aligned}
\dot{\mathbf q} &= \tfrac12\, \mathbf q \otimes \begin{bmatrix}0\\ \boldsymbol\omega_p\end{bmatrix},
\qquad \boldsymbol\omega_p = \mathbf R_{ph}(\boldsymbol\theta)\boldsymbol\omega_h + \mathbf J(\boldsymbol\theta)\dot{\boldsymbol\theta}\\[4pt]
\dot{\mathbf b}_p &= \mathbf n_{b}\\[4pt]
\dot{\boldsymbol\omega}_h &= -\boldsymbol\Lambda\,\boldsymbol\omega_h + \mathbf n_h,
\qquad \boldsymbol\Lambda = \mathrm{diag}(1/\tau_x,1/\tau_y,1/\tau_z)
\end{aligned}
$$

- The handle rate is modeled as a first-order Gauss–Markov process.
- τ sets the disturbance bandwidth. Typically τ ≈ 5–20 ms for hand tremor and walking vibration.
- The encoder terms (θ, θ̇) enter as known inputs **u**.

### Discrete propagation (step Δt)

Define ω̂_p = R_ph ω̂_h + J θ̇ and the integration angle φ = ‖ω̂_p‖Δt.

$$
\hat{\mathbf q}_{k+1|k} = \hat{\mathbf q}_{k|k} \otimes
\begin{bmatrix}\cos(\phi/2)\\[2pt] \dfrac{\hat{\boldsymbol\omega}_p}{\|\hat{\boldsymbol\omega}_p\|}\sin(\phi/2)\end{bmatrix},
\qquad
\hat{\mathbf b}_{p,k+1|k} = \hat{\mathbf b}_{p,k|k},
\qquad
\hat{\boldsymbol\omega}_{h,k+1|k} = \boldsymbol\Phi_h\,\hat{\boldsymbol\omega}_{h,k|k}
$$

where Φ_h = e^{−ΛΔt}. Normalize q̂ after every step.

---

## 4. State Jacobian F (10×10)

Helper matrices:

$$
\boldsymbol\Omega(\boldsymbol\omega)=
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z\\
\omega_x & 0 & \omega_z & -\omega_y\\
\omega_y & -\omega_z & 0 & \omega_x\\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix},
\qquad
\boldsymbol\Xi(\mathbf q)=
\begin{bmatrix}
-q_1 & -q_2 & -q_3\\
q_0 & -q_3 & q_2\\
q_3 & q_0 & -q_1\\
-q_2 & q_1 & q_0
\end{bmatrix}
$$

These satisfy q ⊗ [0; ω] = Ω(ω)q = Ξ(q)ω.

Continuous Jacobian:

$$
\mathbf A = \frac{\partial \dot{\mathbf x}}{\partial \mathbf x}=
\begin{bmatrix}
\tfrac12\boldsymbol\Omega(\hat{\boldsymbol\omega}_p) & \mathbf 0_{4\times3} & \tfrac12\boldsymbol\Xi(\hat{\mathbf q})\mathbf R_{ph}\\
\mathbf 0_{3\times4} & \mathbf 0_{3} & \mathbf 0_{3}\\
\mathbf 0_{3\times4} & \mathbf 0_{3} & -\boldsymbol\Lambda
\end{bmatrix}
$$

Discrete transition (first order is adequate at kHz rates):

$$
\mathbf F_k \approx \mathbf I_{10} + \mathbf A\,\Delta t
$$

Use the exact Φ_h block for ω_h.

### Process noise

The noise vector is w = [n_ω (gyro/kinematic noise on ω_p), n_b, n_h], with noise Jacobian:

$$
\mathbf G = \begin{bmatrix}\tfrac12\boldsymbol\Xi(\hat{\mathbf q}) & \mathbf 0 & \mathbf 0\\ \mathbf 0 & \mathbf I_3 & \mathbf 0\\ \mathbf 0 & \mathbf 0 & \mathbf I_3\end{bmatrix}
$$

$$
\mathbf Q_k \approx \mathbf G\,\mathrm{diag}\!\left(\sigma_\omega^2\mathbf I,\ \sigma_b^2\mathbf I,\ \tfrac{2\sigma_{\omega h}^2}{\tau}\mathbf I\right)\mathbf G^T\Delta t
$$

- σ_ωh is the steady-state RMS handle vibration rate.
- σ_b is the bias random walk, taken from the Allan variance.

---

## 5. Measurement Models

### (a) Handle gyro, bias-compensated

$$
\mathbf z_h = \tilde{\boldsymbol\omega}_h - \mathbf b_h^{cal} = \boldsymbol\omega_h + \mathbf v_h,
\qquad
\mathbf H_h = [\,\mathbf 0_{3\times4}\ \ \mathbf 0_3\ \ \mathbf I_3\,],
\quad \mathbf R_h=\sigma_{gh}^2\mathbf I
$$

### (b) Payload gyro (the coupling measurement)

$$
\mathbf z_p = \tilde{\boldsymbol\omega}_p = \mathbf R_{ph}(\boldsymbol\theta)\boldsymbol\omega_h + \mathbf J(\boldsymbol\theta)\dot{\boldsymbol\theta} + \mathbf b_p + \mathbf v_p
$$

$$
\mathbf H_p = [\,\mathbf 0_{3\times4}\ \ \mathbf I_3\ \ \mathbf R_{ph}(\boldsymbol\theta)\,],
\quad \mathbf R_p=\sigma_{gp}^2\mathbf I
$$

This measurement makes b_p observable. The handle gyro pins ω_h, so any persistent residual in z_p is attributed to b_p. Encoder quantization noise on θ̇ can be folded into R_p as σ²_gp + J σ²_θ̇ Jᵀ.

### (c) Payload accelerometer (gravity reference for tilt)

$$
\mathbf z_a = \tilde{\mathbf a}_p = \mathbf C(\mathbf q)^T\mathbf g + \mathbf v_a
= g\begin{bmatrix}2(q_1q_3 - q_0q_2)\\ 2(q_2q_3 + q_0q_1)\\ q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix} + \mathbf v_a
$$

$$
\mathbf H_a = \Big[\ 2g\begin{bmatrix}-q_2 & q_3 & -q_0 & q_1\\ q_1 & q_0 & q_3 & q_2\\ q_0 & -q_1 & -q_2 & q_3\end{bmatrix}\ \ \mathbf 0_3\ \ \mathbf 0_3\Big]
$$

Use adaptive noise to reject linear acceleration from walking or running:

$$
\mathbf R_a = \sigma_a^2\mathbf I\cdot\Big(1 + \kappa\,\big|\,\|\tilde{\mathbf a}_p\| - g\,\big|\Big)^2
$$

Gate the update entirely if | ‖ã_p‖ − g | exceeds a threshold, e.g. 0.15g.

*Yaw is unobservable from gravity. Add a magnetometer or hold yaw via the joint follow-mode reference if heading is required.*

---

## 6. EKF Recursion

**Predict**

$$
\hat{\mathbf x}_{k+1|k} = f(\hat{\mathbf x}_{k|k},\mathbf u_k),\qquad
\mathbf P_{k+1|k} = \mathbf F_k\mathbf P_{k|k}\mathbf F_k^T + \mathbf Q_k
$$

**Update** (sequentially for each available z_i ∈ {z_h, z_p, z_a})

$$
\begin{aligned}
\mathbf y