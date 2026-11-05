# 10-State EKF for Payload/Handle IMU Fusion (Gimbal Vibration Isolation)

I don't have the earlier cycles of your project, so I've made the design choices explicit below. If Cycle 13 fixed a different state vector, frame convention, or coupling model, the structure still applies and only the corresponding blocks change.

## 1. Frames and Notation

| Symbol | Meaning |
|---|---|
| {n} | Navigation frame (z up). Gravity is gₙ = [0, 0, −g]ᵀ |
| {p} | Payload (camera) body frame, with IMU_p |
| {h} | Handle frame, with IMU_h |
| R_ph | Rotation from handle to payload, computed from the gimbal joint encoders (known input) |
| q | Payload attitude quaternion, Hamilton convention, scalar first, q = [q_w, q_x, q_y, q_z]ᵀ |
| [a]ₓ | Skew-symmetric (cross-product) matrix of vector a |

## 2. State Vector (10 states)

$$
\mathbf{x}=\begin{bmatrix}\mathbf{q}\\ \boldsymbol{\omega}_p\\ \mathbf{b}_p\end{bmatrix}\in\mathbb{R}^{10}
\qquad
\begin{aligned}
\mathbf{q}&\in\mathbb{R}^4 &&\text{payload attitude}\\
\boldsymbol{\omega}_p&\in\mathbb{R}^3 &&\text{true payload angular rate in }\{p\}\\
\mathbf{b}_p&\in\mathbb{R}^3 &&\text{payload gyro bias}
\end{aligned}
$$

Angular rate is an explicit state rather than a raw gyro input. This lets the handle IMU enter the model as a **measured disturbance input**, so the filter predicts the vibration transmitted through the gimbal before the payload gyro sees it.

## 3. Inputs

$$
\mathbf{u}_k=\begin{bmatrix}\boldsymbol{\omega}_h\\ \boldsymbol{\tau}_m\end{bmatrix}
$$

- **ω_h** is the handle gyro reading, bias-compensated offline or by a separate slow estimator.
- **τ_m** is the commanded motor torque from the stabilization controller.

## 4. Continuous Process Model

**Attitude kinematics**

$$
\dot{\mathbf{q}}=\tfrac12\,\Omega(\boldsymbol{\omega}_p)\,\mathbf{q}=\tfrac12\,\Xi(\mathbf{q})\,\boldsymbol{\omega}_p
$$

$$
\Omega(\boldsymbol\omega)=\begin{bmatrix}0&-\boldsymbol\omega^T\\ \boldsymbol\omega&-[\boldsymbol\omega]_\times\end{bmatrix},\qquad
\Xi(\mathbf q)=\begin{bmatrix}-\mathbf q_v^T\\ q_w\mathbf I_3+[\mathbf q_v]_\times\end{bmatrix}
$$

**Payload rate dynamics (handle-to-payload vibration coupling)**

$$
\dot{\boldsymbol{\omega}}_p=-\frac{1}{\tau_c}\Big(\boldsymbol\omega_p-\Gamma\,R_{ph}\,\boldsymbol\omega_h\Big)+J^{-1}\boldsymbol\tau_m+\mathbf w_\omega
$$

- **τ_c** is the coupling time constant, set by the mechanical bandwidth of the joint friction/compliance path.
- **Γ** = diag(γ₁, γ₂, γ₃) holds the per-axis transmissibility, where 0 means perfect isolation and 1 means rigid coupling. You can identify it from a sweep test (handle shaker, motors off).
- **J** is the payload inertia about the gimbal axes.
- **w_ω** captures unmodeled disturbance torque.

**Gyro bias (random walk)**

$$
\dot{\mathbf b}_p=\mathbf w_b
$$

## 5. Discrete Prediction (sample time Δt)

$$
\begin{aligned}
\hat{\mathbf q}_{k|k-1}&=\Big[\cos\!\tfrac{\|\boldsymbol\omega\|\Delta t}{2}\,\mathbf I_4+\tfrac{\sin(\|\boldsymbol\omega\|\Delta t/2)}{\|\boldsymbol\omega\|}\,\Omega(\boldsymbol\omega)\Big]\hat{\mathbf q}_{k-1},\quad \boldsymbol\omega=\hat{\boldsymbol\omega}_{p,k-1}\\[4pt]
\hat{\boldsymbol\omega}_{p,k|k-1}&=\alpha\,\hat{\boldsymbol\omega}_{p,k-1}+(1-\alpha)\,\Gamma R_{ph}\boldsymbol\omega_{h,k}+\Delta t\,J^{-1}\boldsymbol\tau_{m,k},\qquad \alpha=e^{-\Delta t/\tau_c}\\[4pt]
\hat{\mathbf b}_{p,k|k-1}&=\hat{\mathbf b}_{p,k-1}\\[4pt]
\mathbf P_{k|k-1}&=\mathbf F_k\mathbf P_{k-1}\mathbf F_k^T+\mathbf Q_k
\end{aligned}
$$

**State transition Jacobian (10×10)**

$$
\mathbf F_k=\begin{bmatrix}
\mathbf I_4+\tfrac{\Delta t}{2}\Omega(\hat{\boldsymbol\omega}_p) & \tfrac{\Delta t}{2}\Xi(\hat{\mathbf q}) & \mathbf 0_{4\times3}\\
\mathbf 0_{3\times4} & \alpha\,\mathbf I_3 & \mathbf 0_{3\times3}\\
\mathbf 0_{3\times4} & \mathbf 0_{3\times3} & \mathbf I_3
\end{bmatrix}
$$

**Process noise**

$$
\mathbf Q_k=\mathrm{blkdiag}\Big(\sigma_q^2\Delta t\,\mathbf I_4,\;\; \sigma_{\omega}^2\Delta t\,\mathbf I_3+(1-\alpha)^2\Gamma R_{ph}\Sigma_{h}R_{ph}^T\Gamma^T,\;\; \sigma_b^2\Delta t\,\mathbf I_3\Big)
$$

The extra term in the ω block propagates the handle gyro noise Σ_h through the coupling input. A small σ_q only absorbs integration and normalization error.

## 6. Measurement Models

### 6a. Payload gyro (every sample, high rate)

$$
\mathbf z_g=\boldsymbol\omega_p+\mathbf b_p+\mathbf v_g,\qquad
\mathbf H_g=\begin{bmatrix}\mathbf 0_{3\times4}&\mathbf I_3&\mathbf I_3\end{bmatrix},\quad \mathbf R_g=\sigma_g^2\mathbf I_3
$$

### 6b. Payload accelerometer (gravity reference, gated)

Under quasi-static motion, the accelerometer measures f ≈ R(q)ᵀ[0, 0, g]ᵀ:

$$
\mathbf h_a(\mathbf q)=g\begin{bmatrix}2(q_xq_z-q_wq_y)\\ 2(q_yq_z+q_wq_x)\\ q_w^2-q_x^2-q_y^2+q_z^2\end{bmatrix}
$$

$$
\mathbf H_a=\Big[\;2g\begin{bmatrix}-q_y&q_z&-q_w&q_x\\ q_x&q_w&q_z&q_y\\ q_w&-q_x&-q_y&q_z\end{bmatrix}\;\;\mathbf 0_{3\times3}\;\;\mathbf 0_{3\times3}\Big]
$$

The accelerometer noise is made adaptive, which provides vibration gating:

$$
\mathbf R_a=\sigma_a^2\big(1+\kappa\,\big|\|\mathbf z_a\|-g\big|\big)\mathbf I_3
$$

Skip the update entirely when the deviation |‖z_a‖ − g| exceeds a threshold, for example 0.5 m/s².

## 7. Update (applied to each measurement block in sequence)

$$
\begin{aligned}
\mathbf y&=\mathbf z-\mathbf h(\hat{\mathbf x}_{k|k-1})\\
\mathbf S&=\mathbf H\mathbf P_{k|k-1}\mathbf H^T+\mathbf R\\
\mathbf K&=\mathbf P_{k|k-1}\mathbf H^T\mathbf S^{-1}\\
\hat{\mathbf x}_{k}&=\hat{\mathbf x}_{k|k-1}+\mathbf K\mathbf y\\
\mathbf P_k&=(\mathbf I-\mathbf K\mathbf H)\mathbf P_{k|k-1}(\mathbf I-\mathbf K\mathbf H)^T+\mathbf K\mathbf R\mathbf K^T \quad\text{(Joseph form)}
\end{aligned}
$$

**Quaternion renormalization**

$$
\hat{\mathbf q}\leftarrow\hat{\mathbf q}/\|\hat{\mathbf q}\|,\qquad
\mathbf P_{qq}\leftarrow \mathbf N\mathbf P_{qq}\mathbf N^T,\quad \mathbf N=\mathbf I_4-\hat{\mathbf q}\hat{\mathbf q}^T
$$

Apply the projection N to the cross-covariance blocks P_{qω} and P_{qb} as well. This removes the unobservable norm direction and keeps P well-conditioned.

## 8. Observability Notes

- **Separating ω_p from b_p.** Both appear additively in z_g. They become separable because ω_p is driven by the known inputs (ω_h, τ_m) and constrained by attitude evolution through the accelerometer, while b_p is quasi-constant.
- **Yaw bias b_{p,z}.** This component is weakly observable without a heading reference. Either add a magnetometer, or hold σ_b,z small so the bias is updated only during stationary periods (ZUPT: when ‖z_g‖ and ‖ω_h‖ are both below threshold, inject the pseudo-measurement ω_p = 0).

## 9. Output to the Vibration Isolation Loop

- **Rate feedback:** ω̂_p goes to the inner loop. It is bias-free and phase-advanced, because the handle feedforward term predicts the disturbance.
- **Attitude error:** the attitude loop uses

$$
\mathbf e_\theta=2\,\mathrm{vec}\!\left(\mathbf q_{ref}^{-1}\otimes\hat{\mathbf q}\right)
$$

- **Feedforward torque:** the predicted disturbance can be cancelled directly:

$$
\boldsymbol\tau_{ff}=-\frac{J}{\tau_c}\,\Gamma R_{ph}\boldsymbol\omega_h
$$

## 10. Typical Tuning Starting Points

| Parameter | Value |
|---|---|
| Δt | 1 ms (1 kHz) |
| σ_g | 0.005 rad/s |
| σ_a | 0.05 m/s² |
| σ_b | 1e-5 rad/s/√s |
| σ_ω | 0.5–5 rad/s²/√Hz (raise until the innovation y_g is white) |
| τ_c | 5–20 ms |
| Γ | From sweep identification |
| Initial covariance P₀ | diag(1e-2·I₄, 1e-2·I₃, 1e-4·I₃) |

**Consistency checks:**
- Normalized innovation squared, yᵀS⁻¹y, should stay inside the χ²₃ bounds.
- The payload-rate PSD at handle-vib