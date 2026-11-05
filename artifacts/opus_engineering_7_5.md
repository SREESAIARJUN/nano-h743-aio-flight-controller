# 10-State EKF for Payload/Handle IMU Fusion in Gimbal Vibration Isolation

I don't have the details of your "Cycle 7" baseline. This is a self-contained formulation using a common 10-state choice for handheld or vehicle gimbals. Adjust the frames or noise values to match your hardware.

---

## 1. Frames and Sensors

| Symbol | Meaning |
|---|---|
| $\{n\}$ | Navigation frame (z-up) |
| $\{p\}$ | Camera payload frame (payload IMU) |
| $\{h\}$ | Handle/base frame (handle IMU) |
| $\boldsymbol\theta = [\theta_1,\theta_2,\theta_3]^T$ | Gimbal joint angles (encoders) |
| $R_{ph}(\boldsymbol\theta)$ | Rotation from handle frame to payload frame (forward kinematics) |
| $J(\boldsymbol\theta)$ | Joint-rate Jacobian: payload rate relative to handle, expressed in $\{p\}$ |

**Sensor models:**

$$
\boldsymbol\omega_{p,m} = \boldsymbol\omega_p + \mathbf b_p + \boldsymbol\eta_{gp},\qquad
\boldsymbol\omega_{h,m} = \boldsymbol\omega_h + \mathbf b_h + \boldsymbol\eta_{gh}
$$

$$
\mathbf a_{p,m} = C(\mathbf q)^T\mathbf g_n + \mathbf a_{lin} + \boldsymbol\eta_a,\qquad \mathbf g_n=[0,0,g]^T
$$

---

## 2. State Vector (10 states)

$$
\mathbf x = \begin{bmatrix}\mathbf q \\ \mathbf b_p \\ \mathbf b_h\end{bmatrix}
= [\,q_w\; q_x\; q_y\; q_z \;|\; b_{px}\; b_{py}\; b_{pz} \;|\; b_{hx}\; b_{hy}\; b_{hz}\,]^T \in \mathbb R^{10}
$$

- $\mathbf q$: payload attitude ($\{p\}\to\{n\}$), Hamilton convention.
- $\mathbf b_p$: payload gyro bias.
- $\mathbf b_h$: handle gyro bias.

Both biases must be estimated. The handle rate is used as a feedforward disturbance signal, so any bias in it injects a drift torque into the stabilization loop.

---

## 3. Process Model

### Continuous time

$$
\dot{\mathbf q} = \tfrac12\,\Omega(\hat{\boldsymbol\omega}_p)\,\mathbf q,\qquad
\hat{\boldsymbol\omega}_p = \boldsymbol\omega_{p,m}-\mathbf b_p
$$

$$
\dot{\mathbf b}_p = \mathbf w_{bp},\qquad \dot{\mathbf b}_h = \mathbf w_{bh}
$$

These are random-walk biases. Alternatively, use a Gauss–Markov model, $\dot{\mathbf b} = -\mathbf b/\tau + \mathbf w$.

The rate matrix is

$$
\Omega(\boldsymbol\omega)=\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z\\
\omega_x & 0 & \omega_z & -\omega_y\\
\omega_y & -\omega_z & 0 & \omega_x\\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

It is related to the matrix $\Xi(\mathbf q)$ by the identity $\Omega(\boldsymbol\omega)\mathbf q = \Xi(\mathbf q)\boldsymbol\omega$, where

$$
\Xi(\mathbf q)=\begin{bmatrix}
-q_x & -q_y & -q_z\\
q_w & -q_z & q_y\\
q_z & q_w & -q_x\\
-q_y & q_x & q_w
\end{bmatrix}
$$

### Discrete prediction (sample period $\Delta t$)

$$
\hat{\mathbf q}_{k|k-1} = \left[\cos\!\tfrac{\|\hat{\boldsymbol\omega}\|\Delta t}{2}\,I_4 + \frac{\sin\!\tfrac{\|\hat{\boldsymbol\omega}\|\Delta t}{2}}{\|\hat{\boldsymbol\omega}\|}\,\Omega(\hat{\boldsymbol\omega})\right]\hat{\mathbf q}_{k-1|k-1}
$$

$$
\hat{\mathbf b}_{p,k|k-1}=\hat{\mathbf b}_{p,k-1|k-1},\qquad \hat{\mathbf b}_{h,k|k-1}=\hat{\mathbf b}_{h,k-1|k-1}
$$

### State-transition Jacobian

$$
F_k = \begin{bmatrix}
I_4 + \tfrac{\Delta t}{2}\Omega(\hat{\boldsymbol\omega}) & -\tfrac{\Delta t}{2}\Xi(\hat{\mathbf q}) & 0_{4\times3}\\
0_{3\times4} & I_3 & 0_{3}\\
0_{3\times4} & 0_3 & I_3
\end{bmatrix}
$$

For the Gauss–Markov variant, replace each bias block with $e^{-\Delta t/\tau}I_3$.

### Process noise

The noise input matrix and continuous noise covariance are

$$
G_k=\begin{bmatrix}-\tfrac{\Delta t}{2}\Xi(\hat{\mathbf q}) & 0 & 0\\ 0 & \Delta t\, I_3 & 0\\ 0&0&\Delta t\, I_3\end{bmatrix},\qquad
Q_c=\mathrm{diag}(\sigma_{gp}^2 I_3,\;\sigma_{bp}^2 I_3,\;\sigma_{bh}^2 I_3)
$$

The discrete process noise and covariance prediction are

$$
Q_k = G_k\,Q_c\,G_k^T/\Delta t
$$

$$
P_{k|k-1}=F_kP_{k-1|k-1}F_k^T+Q_k
$$

---

## 4. Measurement Models

### (a) Gravity reference from the payload accelerometer

$$
\mathbf z_a=\mathbf a_{p,m},\qquad
h_a(\mathbf q)=C(\mathbf q)^T\mathbf g_n = g\begin{bmatrix}2(q_xq_z-q_wq_y)\\2(q_yq_z+q_wq_x)\\q_w^2-q_x^2-q_y^2+q_z^2\end{bmatrix}
$$

$$
H_a = 2g\begin{bmatrix}
-q_y & q_z & -q_w & q_x\\
q_x & q_w & q_z & q_y\\
q_w & -q_x & -q_y & q_z
\end{bmatrix}\;\Big|\;0_{3\times6}
$$

**Adaptive noise.** Inflate the measurement noise during handle shake:

$$
R_a=\sigma_a^2 I_3\cdot\left(1+k_a\,\big|\|\mathbf a_{p,m}\|-g\big|^2\right)
$$

Alternatively, skip the update when $\big|\|\mathbf a\|-g\big|>\epsilon$. Either choice is essential for vibration rejection.

### (b) Kinematic rate-consistency (fuses the two gyros)

Rigid-body kinematics through the gimbal give

$$
\boldsymbol\omega_p = R_{ph}(\boldsymbol\theta)\,\boldsymbol\omega_h + J(\boldsymbol\theta)\dot{\boldsymbol\theta}
$$

Substituting the sensor models gives a pseudo-measurement:

$$
\mathbf z_\omega=\boldsymbol\omega_{p,m}-R_{ph}\boldsymbol\omega_{h,m}-J\dot{\boldsymbol\theta}_{enc}
= \mathbf b_p - R_{ph}\mathbf b_h + \mathbf v_\omega
$$

$$
H_\omega=\big[\,0_{3\times4}\;\;I_3\;\;-R_{ph}(\boldsymbol\theta)\,\big]
$$

$$
R_\omega=\sigma_{gp}^2I_3+R_{ph}\,\sigma_{gh}^2I_3\,R_{ph}^T+J\,\Sigma_{\dot\theta}\,J^T
$$

Notes on this update:
- $\dot{\boldsymbol\theta}_{enc}$ is the differentiated encoder signal. Low-pass filter it, and include its noise in $\Sigma_{\dot\theta}$.
- Time-align the two IMUs, ideally with hardware-synced sampling. Otherwise, a timing skew $\delta t$ adds an error $\approx \dot{\boldsymbol\omega}_h\delta t$, which is large under vibration. Inflate $R_\omega$ accordingly.
- Apply this update at the gyro rate, or at a decimated rate after anti-alias filtering.

### (c) Quaternion constraint

After each update, normalize the quaternion:

$$
\hat{\mathbf q}\leftarrow\hat{\mathbf q}/\|\hat{\mathbf q}\|
$$

Optionally, also apply the pseudo-measurement $z=1=\|\mathbf q\|^2$ with $H=2\hat{\mathbf q}^T$.

---

## 5. Update Equations (for each measurement $i \in \{a,\omega\}$)

$$
\mathbf y = \mathbf z_i - h_i(\hat{\mathbf x}_{k|k-1})
$$

$$
S = H_iP_{k|k-1}H_i^T+R_i
$$

$$
K = P_{k|k-1}H_i^TS^{-1}
$$

$$
\hat{\mathbf x}_{k|k}=\hat{\mathbf x}_{k|k-1}+K\mathbf y
$$

$$
P_{k|k}=(I-KH_i)P_{k|k-1}(I-KH_i)^T+KR_iK^T \quad\text{(Joseph form)}
$$

Add a chi-square innovation gate, rejecting the update when $\mathbf y^TS^{-1}\mathbf y>\chi^2_{3,0.99}\approx11.3$, to discard impact or shock samples.

---

## 6. Outputs to the Stabilization Controller

| Signal | Expression | Use |
|---|---|---|
| Payload attitude | $\hat{\mathbf q}$ | Outer attitude loop |
| Payload inertial rate | $\hat{\boldsymbol\omega}_p=\boldsymbol\omega_{p,m}-\hat{\mathbf b}_p$ | Inner rate loop feedback |
| Handle disturbance rate (in $\{p\}$) | $\hat{\mathbf d}=R_{ph}(\boldsymbol\omega_{h,m}-\hat{\mathbf b}_h)$ | Feedforward |
| Joint-rate feedforward | $\dot{\boldsymbol\theta}_{ff}=-J^{-1}\hat{\mathbf d}$ | Cancels base motion before it reaches the payload |

The feedforward path is what extends isolation bandwidth beyond the payload-gyro feedback loop alone.

---

## 7. Observability and Tuning Notes

- **Yaw.** Yaw and the yaw component of $\mathbf b_p$ are unobservable from grav