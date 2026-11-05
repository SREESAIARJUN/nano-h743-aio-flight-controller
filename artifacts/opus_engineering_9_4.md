# 10-State EKF for Dual-IMU Gimbal Vibration Isolation

I don't have the earlier design cycles, so this formulation states its conventions and assumptions explicitly. If earlier cycles fixed a different state set, sensor suite or frame convention, the derivation adapts directly.

## 1. Conventions and Assumptions

- **Frames:**
  - **N**: navigation (local level, z-up).
  - **P**: camera payload (where the payload IMU is mounted).
  - **H**: handle (where the handle IMU is mounted).
- **Quaternion:** Hamilton, scalar-first, q = [w x y z]ᵀ. It rotates P to N, giving R(q) = R_NP.
- **Gimbal kinematics:** joint encoders give angles θ and rates θ̇. From these:
  - R_PH(θ) is the rotation from H to P.
  - J(θ) is the joint-rate Jacobian.
  - The relative-rate identity is **ω_P = R_PH(θ)·ω_H + J(θ)·θ̇**.
- **Sensors:** both IMUs are gyro plus accelerometer. Both are time-synchronised and sampled at period Δt (e.g., 1 kHz).

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \mathbf{b}_P \\ \mathbf{b}_H \end{bmatrix} \in \mathbb{R}^{10},
\qquad
\begin{aligned}
\mathbf{q}&\in\mathbb{R}^4 &&\text{payload attitude}\\
\mathbf{b}_P&\in\mathbb{R}^3 &&\text{payload gyro bias}\\
\mathbf{b}_H&\in\mathbb{R}^3 &&\text{handle gyro bias}
\end{aligned}
$$

**Gyro models:**
- ω_P,m = ω_P + b_P + n_gP
- ω_H,m = ω_H + b_H + n_gH

## 3. Process Model

**Continuous time:**

$$
\dot{\mathbf{q}} = \tfrac12\,\Xi(\mathbf{q})\,(\boldsymbol\omega_{P,m}-\mathbf{b}_P-\mathbf{n}_{gP}),\quad
\dot{\mathbf{b}}_P=\mathbf{n}_{bP},\quad
\dot{\mathbf{b}}_H=\mathbf{n}_{bH}
$$

$$
\Xi(\mathbf q)=\begin{bmatrix}-x&-y&-z\\ w&-z&y\\ z&w&-x\\ -y&x&w\end{bmatrix}
$$

**Discrete prediction (zeroth-order hold on the bias-corrected rate):**

Define ω̄ = ω_P,m − b̂_P and the rotation-angle magnitude φ = ‖ω̄‖Δt.

$$
\hat{\mathbf q}_{k|k-1} = \Big[\cos\tfrac{\phi}{2}\,I_4 + \tfrac{\sin(\phi/2)}{\|\bar\omega\|}\,\Omega(\bar\omega)\Big]\hat{\mathbf q}_{k-1},
\qquad
\Omega(\omega)=\begin{bmatrix}0&-\omega^T\\ \omega&-[\omega]_\times\end{bmatrix}
$$

$$
\hat{\mathbf b}_{P,k|k-1}=\hat{\mathbf b}_{P,k-1},\qquad \hat{\mathbf b}_{H,k|k-1}=\hat{\mathbf b}_{H,k-1}
$$

Renormalise q after every prediction and every update.

**State-transition Jacobian (10×10):**

$$
F_k=\begin{bmatrix}
\Phi_q & -\tfrac{\Delta t}{2}\Xi(\hat{\mathbf q}) & 0_{4\times3}\\
0_{3\times4} & I_3 & 0\\
0_{3\times4} & 0 & I_3
\end{bmatrix},
\qquad \Phi_q=\cos\tfrac{\phi}{2}I_4+\tfrac{\sin(\phi/2)}{\|\bar\omega\|}\Omega(\bar\omega)
$$

**Process noise:**

$$
Q_k \approx G\,\mathrm{diag}(\sigma_{gP}^2 I_3,\ \sigma_{bP}^2 I_3,\ \sigma_{bH}^2 I_3)\,G^T\Delta t,
\qquad
G=\begin{bmatrix}-\tfrac12\Xi(\hat{\mathbf q}) & 0 & 0\\ 0& I_3&0\\0&0&I_3\end{bmatrix}
$$

**Covariance propagation:**

$$
P_{k|k-1}=F_kP_{k-1}F_k^T+Q_k
$$

## 4. Measurement Models

### (a) Payload accelerometer: gravity reference for roll and pitch

The predicted accelerometer reading is gravity rotated into the payload frame:

$$
h_a(\mathbf q)=R(\mathbf q)^T\begin{bmatrix}0\\0\\g\end{bmatrix}
= g\begin{bmatrix}2(xz-wy)\\ 2(yz+wx)\\ w^2-x^2-y^2+z^2\end{bmatrix}
$$

$$
H_a=\Big[\,2g\begin{bmatrix}-y&z&-w&x\\ x&w&z&y\\ w&-x&-y&z\end{bmatrix}\ \ 0_{3\times3}\ \ 0_{3\times3}\Big]
$$

**Adaptive R_a for vibration and handheld motion.** Inflate the noise when the measured specific force departs from gravity:

$$
R_a=\sigma_a^2\big(1+\alpha\,\big|\|\mathbf a_m\|-g\big|\big)\,I_3
$$

Reject the update outright when |‖a_m‖ − g| exceeds a threshold, or when the normalised innovation νᵀS⁻¹ν > χ²₃(0.99).

### (b) Dual-gyro rate consistency: the fusion step

Substitute both gyro models into the kinematic identity. The true rates cancel, leaving a bias-only residual:

$$
\mathbf z_\omega=\boldsymbol\omega_{P,m}-R_{PH}(\theta)\,\boldsymbol\omega_{H,m}-J(\theta)\dot\theta
=\mathbf b_P-R_{PH}(\theta)\,\mathbf b_H+\mathbf v_\omega
$$

$$
H_\omega=\big[\,0_{3\times4}\ \ I_3\ \ -R_{PH}(\theta)\,\big]
$$

$$
R_\omega=\sigma_{gP}^2I_3+R_{PH}\,\sigma_{gH}^2\,R_{PH}^T+\sigma_{enc}^2 JJ^T
$$

Practical points for this update:

- **Pre-filtering:** low-pass both gyro signals identically, below the dominant handle vibration band, before forming z_ω. Otherwise sensor-timing and lever-arm effects alias into bias.
- **Update rate:** run this update at a decimated rate, e.g., 100–200 Hz.

### (c) Optional: yaw reference

Use a magnetometer, or a zero-rate update when the gimbal is detected stationary. Without one, yaw attitude is unobservable, and the absolute bias component along gravity is only weakly observable.

## 5. Update Equations (for each measurement j ∈ {a, ω, …})

$$
\begin{aligned}
\boldsymbol\nu &= \mathbf z_j - h_j(\hat{\mathbf x}_{k|k-1})\\
S &= H_jP_{k|k-1}H_j^T+R_j\\
K &= P_{k|k-1}H_j^TS^{-1}\\
\hat{\mathbf x}_{k} &= \hat{\mathbf x}_{k|k-1}+K\boldsymbol\nu,\qquad \hat{\mathbf q}\leftarrow\hat{\mathbf q}/\|\hat{\mathbf q}\|\\
P_k &= (I-KH_j)P_{k|k-1}(I-KH_j)^T+KR_jK^T \quad\text{(Joseph form)}
\end{aligned}
$$

**Quaternion-norm caveat.** The 4-component quaternion makes P rank-deficient along q. Use one of these:

- Project P after normalisation with Π = I − q̂q̂ᵀ on the q block.
- Implement the equivalent **9-state error-state (multiplicative) EKF**, with δθ ∈ ℝ³ replacing δq. This is numerically preferred, and it is a drop-in change: replace Ξ(q)/2 by I₃ in the error dynamics.

## 6. Outputs for Vibration Isolation

The filter supplies three signals to the gimbal controller:

| Signal | Equation | Use |
|---|---|---|
| Payload rate (feedback) | ω̂_P = ω_P,m − b̂_P | Inner rate loop |
| Handle disturbance (feedforward) | ω̂_H = ω_H,m − b̂_H | Disturbance cancellation |
| Payload attitude | q̂ | Outer attitude loop |

**Feedforward joint-rate command.** Choose joint rates that cancel the handle rate seen at the payload:

$$
\dot\theta_{ff}=-J(\theta)^{-1}R_{PH}(\theta)\,\hat{\boldsymbol\omega}_H
$$

Add this to the attitude-loop command. The handle IMU then provides phase-lead disturbance rejection, while the payload IMU closes the loop.

## 7. Suggested Initial Tuning

| Parameter | Typical value (MEMS) |
|---|---|
| σ_gP, σ_gH | 0.005–0.01 rad/s/√Hz |
| σ_bP, σ_bH (random walk) | 1e-5–1e-4 rad/s²/√Hz |
| σ_a | 0.05–0.2 m/s² |
| P₀ (q block) | (5°)² |
| P₀ (bias blocks) | (0.5°/s)² |

Validate the tuning with NEES/NIS consistency tests, using recorded handle-vibration profiles (walking at 1–3 Hz, plus motor/rotor harmonics).