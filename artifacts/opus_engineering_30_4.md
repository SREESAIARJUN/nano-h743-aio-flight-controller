# 10-State EKF for Payload/Handle IMU Fusion in Gimbal Vibration Isolation

I don't have context for what "Cycle 30" refers to in your project, so this is a self-contained formulation. Adapt the frame conventions and noise values to your hardware.

---

## 1. Problem Setup

**Frames**
- **n**: navigation frame (NED or ENU, gravity along ±z)
- **p**: camera payload body frame (payload IMU)
- **h**: handle body frame (handle IMU)

**Sensors**
- Payload gyro: $\boldsymbol{\omega}_{p,m}$
- Payload accelerometer: $\mathbf{a}_{p,m}$
- Handle gyro: $\boldsymbol{\omega}_{h,m}$
- Joint encoders: $\boldsymbol{\theta}$, $\dot{\boldsymbol{\theta}}$

**Goals**
1. Estimate the payload attitude.
2. Estimate both gyro biases.
3. Supply bias-free handle rates for feedforward cancellation of handle vibration before it reaches the payload.

---

## 2. State Vector (10 states)

$$
\mathbf{x} =
\begin{bmatrix}
\mathbf{q} \\ \mathbf{b}_p \\ \mathbf{b}_h
\end{bmatrix}
\in \mathbb{R}^{10},
\qquad
\mathbf{q} = [q_w\; q_x\; q_y\; q_z]^T \;(\text{payload attitude, } p \to n)
$$

| States | Symbol | Dimension |
|---|---|---|
| Payload attitude quaternion | $\mathbf{q}$ | 4 |
| Payload gyro bias | $\mathbf{b}_p$ | 3 |
| Handle gyro bias | $\mathbf{b}_h$ | 3 |

---

## 3. Sensor Models

$$
\boldsymbol{\omega}_{p,m} = \boldsymbol{\omega}_p + \mathbf{b}_p + \mathbf{n}_p, \qquad
\boldsymbol{\omega}_{h,m} = \boldsymbol{\omega}_h + \mathbf{b}_h + \mathbf{n}_h
$$

$$
\dot{\mathbf{b}}_p = \mathbf{w}_{bp}, \qquad \dot{\mathbf{b}}_h = \mathbf{w}_{bh} \quad (\text{random walk})
$$

**Gimbal kinematic coupling.** The payload rate equals the handle rate rotated into the payload frame, plus the motor-induced rate:

$$
\boldsymbol{\omega}_p = \mathbf{C}_{ph}(\boldsymbol{\theta})\,\boldsymbol{\omega}_h + \mathbf{J}(\boldsymbol{\theta})\,\dot{\boldsymbol{\theta}}
$$

- $\mathbf{C}_{ph}$ is the handle-to-payload rotation from the encoder angles, including the IMU mounting calibration.
- $\mathbf{J}$ is the gimbal joint-rate Jacobian expressed in the payload frame.

---

## 4. Process Model (Prediction)

The payload gyro is the control input:

$$
\bar{\boldsymbol{\omega}} = \boldsymbol{\omega}_{p,m} - \hat{\mathbf{b}}_p
$$

**Continuous-time dynamics**

$$
\dot{\mathbf{q}} = \tfrac12\,\boldsymbol{\Omega}(\bar{\boldsymbol\omega})\,\mathbf{q} = \tfrac12\,\boldsymbol{\Xi}(\mathbf{q})\,\bar{\boldsymbol\omega},
\qquad \dot{\mathbf{b}}_p = \mathbf{0},\quad \dot{\mathbf{b}}_h=\mathbf{0}
$$

where

$$
\boldsymbol{\Omega}(\boldsymbol\omega)=
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z\\
\omega_x & 0 & \omega_z & -\omega_y\\
\omega_y & -\omega_z & 0 & \omega_x\\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix},
\qquad
\boldsymbol{\Xi}(\mathbf{q})=
\begin{bmatrix}
-q_x & -q_y & -q_z\\
q_w & -q_z & q_y\\
q_z & q_w & -q_x\\
-q_y & q_x & q_w
\end{bmatrix}
$$

**Discrete propagation** (exact under zero-order hold of $\bar{\boldsymbol\omega}$ over $\Delta t$):

$$
\boldsymbol{\Phi}_q = \cos\!\left(\tfrac{\|\bar{\boldsymbol\omega}\|\Delta t}{2}\right)\mathbf{I}_4 + \frac{\sin\!\left(\tfrac{\|\bar{\boldsymbol\omega}\|\Delta t}{2}\right)}{\|\bar{\boldsymbol\omega}\|}\,\boldsymbol{\Omega}(\bar{\boldsymbol\omega})
$$

$$
\hat{\mathbf{q}}_{k+1}^- = \boldsymbol{\Phi}_q\,\hat{\mathbf{q}}_k^+, \qquad
\hat{\mathbf{b}}_{p,k+1}^- = \hat{\mathbf{b}}_{p,k}^+, \qquad
\hat{\mathbf{b}}_{h,k+1}^- = \hat{\mathbf{b}}_{h,k}^+
$$

Then normalize $\hat{\mathbf{q}}$.

**State transition Jacobian** ($10\times10$):

$$
\mathbf{F}_k=
\begin{bmatrix}
\boldsymbol{\Phi}_q & -\tfrac{\Delta t}{2}\boldsymbol{\Xi}(\hat{\mathbf{q}}) & \mathbf{0}_{4\times3}\\
\mathbf{0}_{3\times4} & \mathbf{I}_3 & \mathbf{0}\\
\mathbf{0}_{3\times4} & \mathbf{0} & \mathbf{I}_3
\end{bmatrix}
$$

**Process noise**

$$
\mathbf{Q}_k=
\begin{bmatrix}
\tfrac{\Delta t^2}{4}\,\boldsymbol\Xi\,(\sigma_{gp}^2\mathbf{I}_3)\,\boldsymbol\Xi^T & \mathbf 0 & \mathbf 0\\
\mathbf 0 & \sigma_{bp}^2\Delta t\,\mathbf I_3 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \sigma_{bh}^2\Delta t\,\mathbf I_3
\end{bmatrix}
$$

**Covariance prediction**

$$
\mathbf{P}_{k+1}^- = \mathbf{F}_k\mathbf{P}_k^+\mathbf{F}_k^T + \mathbf{Q}_k
$$

---

## 5. Measurement Models

### 5a. Payload accelerometer (gravity reference)

$$
\mathbf{z}_a = \mathbf{a}_{p,m} \approx \mathbf{h}_a(\mathbf q) = \mathbf{C}(\mathbf q)^T\begin{bmatrix}0\\0\\g\end{bmatrix}
= g\begin{bmatrix}2(q_xq_z-q_wq_y)\\ 2(q_yq_z+q_wq_x)\\ q_w^2-q_x^2-q_y^2+q_z^2\end{bmatrix}
$$

$$
\mathbf{H}_a=\Big[\;2g\begin{bmatrix}-q_y & q_z & -q_w & q_x\\ q_x & q_w & q_z & q_y\\ q_w & -q_x & -q_y & q_z\end{bmatrix}\;\Big|\;\mathbf 0_{3\times3}\;\Big|\;\mathbf 0_{3\times3}\Big]
$$

**Adaptive noise gating.** Handheld motion and vibration corrupt the gravity reference, so inflate the noise when the specific force deviates from gravity:

$$
\mathbf R_a = \left(\sigma_a^2 + k_a\big(\|\mathbf a_{p,m}\|-g\big)^2\right)\mathbf I_3
$$

Skip the update entirely if $\big|\|\mathbf a_{p,m}\|-g\big| > \epsilon$.

### 5b. Gyro rate-consistency measurement (the IMU fusion step)

This is the core of the payload/handle fusion. Form the pseudo-measurement:

$$
\mathbf z_\omega = \boldsymbol\omega_{p,m} - \mathbf C_{ph}(\boldsymbol\theta)\,\boldsymbol\omega_{h,m} - \mathbf J(\boldsymbol\theta)\,\dot{\boldsymbol\theta}_{enc}
$$

Substituting the sensor models, the true rates cancel:

$$
\mathbf z_\omega = \mathbf b_p - \mathbf C_{ph}\mathbf b_h + \underbrace{\mathbf n_p - \mathbf C_{ph}\mathbf n_h - \mathbf J\,\mathbf n_{\dot\theta}}_{\boldsymbol\nu_\omega}
$$

$$
\mathbf H_\omega = \big[\;\mathbf 0_{3\times4}\;\big|\;\mathbf I_3\;\big|\;-\mathbf C_{ph}(\boldsymbol\theta)\;\big]
$$

$$
\mathbf R_\omega = \sigma_{gp}^2\mathbf I_3 + \sigma_{gh}^2\,\mathbf C_{ph}\mathbf C_{ph}^T + \mathbf J\,\boldsymbol\Sigma_{\dot\theta}\,\mathbf J^T
$$

**Practical requirements for this update:**
- **Time alignment.** Timestamp-align both IMUs to under 1 sample. At vibration frequencies (50–300 Hz), even 0.5 ms of skew produces large spurious residuals.
- **Matched filtering.** Pass $\boldsymbol\omega_{p,m}$, $\boldsymbol\omega_{h,m}$, and $\dot{\boldsymbol\theta}$ through the same low-pass filter before forming $\mathbf z_\omega$. Otherwise structural flex between the IMU mounts is misinterpreted as bias.
- **Update rate.** Run this update at a reduced rate (e.g., 100–200 Hz).

### 5c. Quaternion norm pseudo-measurement (optional)

$$
z_n = 1,\qquad h_n = \mathbf q^T\mathbf q,\qquad \mathbf H_n = [\,2\mathbf q^T\;|\;\mathbf 0\;|\;\mathbf 0\,],\qquad R_n \approx 10^{-6}
$$

---

## 6. Update Equations (for each measurement $j \in \{a,\omega,n\}$)

$$
\mathbf S = \mathbf H_j\mathbf P^-\mathbf H_j^T + \mathbf R_j
$$

$$
\mathbf K = \mathbf P^-\mathbf H_j^T\mathbf S^{-1}
$$

$$
\hat{\mathbf x}^+ = \hat{\mathbf x}^- + \mathbf K\big(\mathbf z_j - \mathbf h_j(\hat{\mathbf x}^-)\big)
$$

$$
\mathbf P^+ = (\mathbf I-\mathbf K\mathbf H_j)\,\mathbf P^-(\mathbf I-\mathbf K\mathbf H_j)^T + \mathbf K\mathbf R_j\mathbf K^T \quad\text{(Joseph form)}
$