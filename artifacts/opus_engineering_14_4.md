# 10-State EKF for Gimbal Vibration Isolation: Payload IMU + Handle IMU Fusion

I don't have the context for "Cycle 14," so this formulation is self-contained. The structure is a standard one: the **payload IMU** provides the measurements, and the **handle IMU** acts as a known disturbance input that is transmitted through the gimbal joints. If your previous cycles used different conventions, the notation can be adapted.

---

## 1. State, Inputs, and Measurements

**State vector (10×1):**

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \boldsymbol{\omega}_p \\ \mathbf{b}_p \end{bmatrix}
= \begin{bmatrix} q_0 & q_1 & q_2 & q_3 & \omega_x & \omega_y & \omega_z & b_x & b_y & b_z \end{bmatrix}^T
$$

| State | Dim | Meaning |
|---|---|---|
| $\mathbf{q}$ | 4 | Payload attitude quaternion (body→world, Hamilton, scalar-first) |
| $\boldsymbol{\omega}_p$ | 3 | True payload angular velocity (body frame) |
| $\mathbf{b}_p$ | 3 | Payload gyro bias |

**Inputs:**
- $\boldsymbol{\tau}_m$: commanded motor torque, mapped to the payload frame.
- $\boldsymbol{\omega}_h^p = \mathbf{C}_{hp}(\boldsymbol{\theta}_{enc})\,\tilde{\boldsymbol{\omega}}_h$: the handle gyro rate, rotated into the payload frame using the joint encoder angles.

**Measurements (payload IMU):**
- Gyro: $\tilde{\boldsymbol{\omega}}_p$
- Accelerometer: $\tilde{\mathbf{a}}_p$

---

## 2. Continuous-Time Process Model

$$
\dot{\mathbf{q}} = \tfrac{1}{2}\,\boldsymbol{\Omega}(\boldsymbol{\omega}_p)\,\mathbf{q}
$$

$$
\dot{\boldsymbol{\omega}}_p = \mathbf{J}^{-1}\Big[\boldsymbol{\tau}_m - \boldsymbol{\omega}_p \times \mathbf{J}\boldsymbol{\omega}_p + \mathbf{D}\big(\boldsymbol{\omega}_h^p - \boldsymbol{\omega}_p\big)\Big] + \mathbf{w}_\omega
$$

$$
\dot{\mathbf{b}}_p = \mathbf{w}_b
$$

**Parameters:**
- $\mathbf{J}$: payload inertia.
- $\mathbf{D} = \mathrm{diag}(d_x, d_y, d_z)$: effective joint coupling (bearing friction, cable drag, motor back-EMF damping). This is the path by which handle vibration leaks into the payload.

**Quaternion rate matrix:**

$$
\boldsymbol{\Omega}(\boldsymbol{\omega}) =
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z\\
\omega_x & 0 & \omega_z & -\omega_y\\
\omega_y & -\omega_z & 0 & \omega_x\\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

---

## 3. Discrete Prediction (step $\Delta t$)

**Quaternion (exact zero-order-hold exponential):**

$$
\mathbf{q}_{k+1|k} = \left[\cos\!\tfrac{\|\boldsymbol{\omega}\|\Delta t}{2}\,\mathbf{I}_4 + \frac{\sin\!\tfrac{\|\boldsymbol{\omega}\|\Delta t}{2}}{\|\boldsymbol{\omega}\|}\,\boldsymbol{\Omega}(\boldsymbol{\omega})\right]\mathbf{q}_{k|k},
\quad \boldsymbol{\omega} = \hat{\boldsymbol{\omega}}_{p,k|k}
$$

**Angular velocity:**

$$
\hat{\boldsymbol{\omega}}_{p,k+1|k} = \hat{\boldsymbol{\omega}}_{p} + \Delta t\,\mathbf{J}^{-1}\Big[\boldsymbol{\tau}_m - \hat{\boldsymbol{\omega}}_p\times\mathbf{J}\hat{\boldsymbol{\omega}}_p + \mathbf{D}(\boldsymbol{\omega}_h^p - \hat{\boldsymbol{\omega}}_p)\Big]
$$

**Bias:**

$$
\hat{\mathbf{b}}_{p,k+1|k} = \hat{\mathbf{b}}_{p,k|k}
$$

**Covariance:**

$$
\mathbf{P}_{k+1|k} = \mathbf{F}_k \mathbf{P}_{k|k}\mathbf{F}_k^T + \mathbf{G}_k \mathbf{Q}_h \mathbf{G}_k^T + \mathbf{Q}_k
$$

### State Transition Jacobian $\mathbf{F}_k$ (10×10)

$$
\mathbf{F}_k =
\begin{bmatrix}
\mathbf{I}_4 + \tfrac{\Delta t}{2}\boldsymbol{\Omega}(\hat{\boldsymbol{\omega}}_p) & \tfrac{\Delta t}{2}\boldsymbol{\Xi}(\hat{\mathbf{q}}) & \mathbf{0}_{4\times3}\\
\mathbf{0}_{3\times4} & \mathbf{F}_{\omega\omega} & \mathbf{0}_{3}\\
\mathbf{0}_{3\times4} & \mathbf{0}_{3} & \mathbf{I}_3
\end{bmatrix}
$$

The quaternion-to-rate block uses:

$$
\boldsymbol{\Xi}(\mathbf{q}) =
\begin{bmatrix}
-q_1 & -q_2 & -q_3\\
q_0 & -q_3 & q_2\\
q_3 & q_0 & -q_1\\
-q_2 & q_1 & q_0
\end{bmatrix},
\qquad \dot{\mathbf{q}} = \tfrac{1}{2}\boldsymbol{\Xi}(\mathbf{q})\boldsymbol{\omega}
$$

The rate dynamics block is:

$$
\mathbf{F}_{\omega\omega} = \mathbf{I}_3 + \Delta t\,\mathbf{J}^{-1}\Big(-[\hat{\boldsymbol{\omega}}_p\times]\mathbf{J} + [(\mathbf{J}\hat{\boldsymbol{\omega}}_p)\times] - \mathbf{D}\Big)
$$

### Handle-Gyro Noise Injection

The handle gyro noise enters through the coupling term:

$$
\mathbf{G}_k = \begin{bmatrix}\mathbf{0}_{4\times3}\\ \Delta t\,\mathbf{J}^{-1}\mathbf{D}\,\mathbf{C}_{hp}\\ \mathbf{0}_{3}\end{bmatrix},
\qquad \mathbf{Q}_h = \sigma_{g,h}^2\mathbf{I}_3
$$

### Process Noise

$$
\mathbf{Q}_k = \mathrm{blkdiag}\big(\sigma_q^2\mathbf{I}_4,\ \sigma_\omega^2\Delta t\,\mathbf{I}_3,\ \sigma_b^2\Delta t\,\mathbf{I}_3\big)
$$

Keep $\sigma_q$ small; it is only needed for numerical conditioning.

---

## 4. Measurement Models

### (a) Payload Gyro (every sample, e.g., 1–8 kHz)

$$
\mathbf{z}_g = \boldsymbol{\omega}_p + \mathbf{b}_p + \mathbf{v}_g,
\qquad
\mathbf{H}_g = \begin{bmatrix}\mathbf{0}_{3\times4} & \mathbf{I}_3 & \mathbf{I}_3\end{bmatrix},
\qquad
\mathbf{R}_g = \sigma_{g,p}^2\mathbf{I}_3
$$

Bias and rate are separable only because the rate dynamics are driven by known inputs ($\boldsymbol{\tau}_m$, $\boldsymbol{\omega}_h$) and the attitude is anchored by the accelerometer. An accurate $\mathbf{J}$ and $\mathbf{D}$ therefore matter.

### (b) Payload Accelerometer (gravity reference, lower rate)

With $\mathbf{g}_w = [0, 0, g]^T$ and $\mathbf{C}(\mathbf{q})$ body→world:

$$
\mathbf{h}_a(\mathbf{q}) = \mathbf{C}(\mathbf{q})^T\mathbf{g}_w = g\begin{bmatrix}2(q_1q_3 - q_0q_2)\\ 2(q_2q_3 + q_0q_1)\\ q_0^2 - q_1^2 - q_2^2 + q_3^2\end{bmatrix}
$$

$$
\mathbf{H}_a = \begin{bmatrix}\mathbf{H}_{a,q} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3}\end{bmatrix},
\qquad
\mathbf{H}_{a,q} = 2g\begin{bmatrix}
-q_2 & q_3 & -q_0 & q_1\\
q_1 & q_0 & q_3 & q_2\\
q_0 & -q_1 & -q_2 & q_3
\end{bmatrix}
$$

**Adaptive accelerometer noise (vibration gating):**

$$
\mathbf{R}_a = \sigma_a^2\left(1 + \kappa\,\big|\,\|\tilde{\mathbf{a}}_p\| - g\,\big|^2\right)\mathbf{I}_3
$$

Skip the accelerometer update entirely if $\big|\,\|\tilde{\mathbf{a}}_p\| - g\,\big| > \epsilon$ (e.g., 0.15 g).

---

## 5. Update Step (for each measurement $j \in \{g, a\}$)

$$
\mathbf{y} = \mathbf{z}_j - \mathbf{h}_j(\hat{\mathbf{x}}_{k+1|k})
$$

$$
\mathbf{S} = \mathbf{H}_j\mathbf{P}_{k+1|k}\mathbf{H}_j^T + \mathbf{R}_j
$$

$$
\mathbf{K} = \mathbf{P}_{k+1|k}\mathbf{H}_j^T\mathbf{S}^{-1}
$$

$$
\hat{\mathbf{x}}_{k+1|k+1} = \hat{\mathbf{x}}_{k+1|k} + \mathbf{K}\mathbf{y}
$$

**Covariance update (Joseph form, for numerical robustness):**

$$
\mathbf{P}_{k+1|k+1} = (\mathbf{I} - \mathbf{K}\mathbf{H}_j)\mathbf{P}_{k+1|k}(\mathbf{I} - \mathbf{K}\mathbf{H}_j)^T + \mathbf{K}\mathbf{R}_j\mathbf{K}^T
$$

**Quaternion normalization (after each update):**

$$
\hat{\mathbf