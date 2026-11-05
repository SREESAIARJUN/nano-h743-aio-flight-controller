# 10-State EKF for Payload/Handle IMU Fusion in Gimbal Vibration Isolation

I don't have your earlier design cycles, so this formulation is self-contained. It states its own assumptions. Map the symbols to your Cycle 1–5 conventions as needed.

---

## 1. Problem Setup

**Hardware assumed:**
- A 3-axis stabilized gimbal.
- **Payload IMU** (gyro + accelerometer) rigidly mounted to the camera.
- **Handle IMU** (gyro) mounted on the operator handle, the source of the vibration disturbance.
- Joint encoders that provide the handle-to-payload rotation $R_{ph}$.

**Objective:** estimate payload attitude and angular rate with low latency. The handle gyro serves as a measured disturbance input, so transmitted vibration can be predicted (feedforward) rather than only reacted to.

**Fusion strategy:**
- The handle gyro enters the **process model** as a known disturbance input, with its noise mapped into $Q$.
- The payload gyro and accelerometer are **measurements**.

---

## 2. State Vector (10 states)

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \boldsymbol{\omega}_p \\ \mathbf{b}_p \end{bmatrix}
\in \mathbb{R}^{10}
\qquad
\begin{aligned}
\mathbf{q} &= [q_0\ q_1\ q_2\ q_3]^T &&\text{payload attitude (world}\to\text{payload), 4}\\
\boldsymbol{\omega}_p &&&\text{true payload angular rate, payload frame, 3}\\
\mathbf{b}_p &&&\text{payload gyro bias, 3}
\end{aligned}
$$

**Inputs:**
- $\mathbf{u} = \{\boldsymbol{\tau}_m,\ \boldsymbol{\omega}_h^{meas}\}$: motor torque command and handle gyro reading.
- $\boldsymbol{\omega}_h^{meas} = \boldsymbol{\omega}_h + \mathbf{n}_h$.

---

## 3. Continuous-Time Process Model

**Attitude kinematics:**
$$
\dot{\mathbf{q}} = \tfrac12\,\Omega(\boldsymbol{\omega}_p)\,\mathbf{q} = \tfrac12\,\Xi(\mathbf{q})\,\boldsymbol{\omega}_p
$$
$$
\Omega(\boldsymbol\omega)=\begin{bmatrix}0 & -\boldsymbol\omega^T\\ \boldsymbol\omega & -[\boldsymbol\omega]_\times\end{bmatrix},\qquad
\Xi(\mathbf q)=\begin{bmatrix}-\mathbf q_v^T\\ q_0 I_3+[\mathbf q_v]_\times\end{bmatrix}
$$

**Payload rotational dynamics:** handle motion couples through bearing friction and cable drag, modeled as viscous coupling $D$:
$$
\dot{\boldsymbol\omega}_p = J^{-1}\Big[\boldsymbol\tau_m + D\big(R_{ph}\,\boldsymbol\omega_h^{meas} - \boldsymbol\omega_p\big) - \boldsymbol\omega_p\times J\boldsymbol\omega_p\Big] + \mathbf w_\omega
$$
- $J$ is the payload inertia.
- $D = \mathrm{diag}(d_x,d_y,d_z)$ is the coupling damping, identified from a handle-shake test.
- $\mathbf w_\omega$ absorbs unmodeled torques.

**Gyro bias (random walk):**
$$
\dot{\mathbf b}_p = \mathbf w_b
$$

---

## 4. Jacobians

### Process Jacobian $F = \partial f/\partial \mathbf x$

$$
F=\begin{bmatrix}
\tfrac12\Omega(\boldsymbol\omega_p) & \tfrac12\Xi(\mathbf q) & 0_{4\times3}\\
0_{3\times4} & F_{\omega\omega} & 0_{3\times3}\\
0_{3\times4} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix}
$$
$$
F_{\omega\omega} = J^{-1}\big(-D - [\boldsymbol\omega_p]_\times J + [J\boldsymbol\omega_p]_\times\big)
$$

### Noise Input Matrix

The noise vector is $\mathbf w = [\mathbf n_h;\ \mathbf w_\omega;\ \mathbf w_b]$:
$$
G=\begin{bmatrix}
0_{4\times3} & 0_{4\times3} & 0_{4\times3}\\
J^{-1}DR_{ph} & I_3 & 0\\
0 & 0 & I_3
\end{bmatrix},\qquad
Q_c=\mathrm{diag}(\sigma_h^2 I,\ \sigma_\omega^2 I,\ \sigma_b^2 I)
$$

---

## 5. Discretization (sample period $\Delta t$, e.g. 1–2 kHz)

$$
\Phi_k \approx I_{10} + F_k\Delta t + \tfrac12 F_k^2\Delta t^2,\qquad
Q_k \approx G_k Q_c G_k^T\,\Delta t
$$

For the quaternion propagation itself, use the exact zeroth-order-hold form:
$$
\mathbf q_{k+1} = \Big[\cos\!\tfrac{\|\boldsymbol\omega\|\Delta t}{2}\,I_4 + \tfrac{\sin(\|\boldsymbol\omega\|\Delta t/2)}{\|\boldsymbol\omega\|}\,\Omega(\boldsymbol\omega)\Big]\mathbf q_k
$$

---

## 6. Measurement Models

### (a) Payload Gyro (every sample)

$$
\mathbf z_g = \boldsymbol\omega_p + \mathbf b_p + \mathbf v_g,\qquad
H_g = \begin{bmatrix}0_{3\times4} & I_3 & I_3\end{bmatrix},\quad R_g=\sigma_g^2 I_3
$$

### (b) Payload Accelerometer (gravity reference, gated)

The quasi-static specific force is $\mathbf f = R(\mathbf q)^T[0\ 0\ g]^T$:
$$
h_a(\mathbf q)= g\begin{bmatrix}2(q_1q_3-q_0q_2)\\ 2(q_2q_3+q_0q_1)\\ q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix}
$$
$$
H_a = \Big[\,2g\begin{bmatrix}-q_2 & q_3 & -q_0 & q_1\\ q_1 & q_0 & q_3 & q_2\\ q_0 & -q_1 & -q_2 & q_3\end{bmatrix}\quad 0_{3\times3}\quad 0_{3\times3}\Big]
$$

**Adaptive gating:** inflate $R_a$ when the payload is accelerating or vibrating.
$$
R_a = \sigma_a^2 I_3\,\big(1 + k_a\,\big|\,\|\mathbf z_a\| - g\,\big|\big)
$$
Reject the update entirely if $\big|\|\mathbf z_a\|-g\big| > \epsilon$.

---

## 7. EKF Recursion

**Predict:**
$$
\hat{\mathbf x}_{k|k-1} = \hat{\mathbf x}_{k-1} + \int f(\hat{\mathbf x},\mathbf u)\,dt
\quad (\text{RK4, or the exact quaternion step above})
$$
$$
P_{k|k-1} = \Phi_k P_{k-1}\Phi_k^T + Q_k
$$

**Update** (stack the gyro measurements, plus the accelerometer measurements when accepted):
$$
\mathbf y_k = \mathbf z_k - h(\hat{\mathbf x}_{k|k-1}),\qquad
S_k = H_k P_{k|k-1} H_k^T + R_k
$$
$$
K_k = P_{k|k-1}H_k^T S_k^{-1}
$$
$$
\hat{\mathbf x}_k = \hat{\mathbf x}_{k|k-1} + K_k\mathbf y_k
$$
$$
P_k = (I-K_kH_k)P_{k|k-1}(I-K_kH_k)^T + K_kR_kK_k^T \quad\text{(Joseph form)}
$$

**Post-update:**
- Normalize the quaternion: $\hat{\mathbf q}\leftarrow\hat{\mathbf q}/\|\hat{\mathbf q}\|$.
- Symmetrize: $P\leftarrow\tfrac12(P+P^T)$.

---

## 8. Use in Vibration Isolation

The estimator supplies three control signals:

| Signal | Expression | Role |
|---|---|---|
| Feedforward disturbance torque | $\hat{\boldsymbol\tau}_d = D\big(R_{ph}\boldsymbol\omega_h^{meas} - \hat{\boldsymbol\omega}_p\big)$ | Cancel before it reaches the payload |
| Rate feedback | $\hat{\boldsymbol\omega}_p$ (bias-free, low-lag) | Inner rate loop |
| Attitude feedback | $\hat{\mathbf q}$ | Outer stabilization loop |

The resulting motor command is
$$
\boldsymbol\tau_m = -\hat{\boldsymbol\tau}_d + K_\omega(\boldsymbol\omega_{ref}-\hat{\boldsymbol\omega}_p) + K_q\,\mathbf e_q(\mathbf q_{ref},\hat{\mathbf q})
$$

---

## 9. Implementation and Tuning Notes

- **Quaternion covariance:** with 4 quaternion states, $P$ is rank-deficient along the unit-norm constraint. Initialize carefully and normalize after every update. If numerical issues appear, switch to a 9-state error-state (MEKF) form with a 3-element attitude error.
- **Tuning knobs:**
  - $\sigma_h$ comes from the handle gyro datasheet's ARW, scaled to the sample rate.
  - $\sigma_\omega$ sets how much the filter trusts the dynamics. Raise it if $D$ or $J$ are uncertain.
  - $\sigma_b$ comes from bias instability, typically $10^{-5}$–$10^{-4}$ rad/s/√s.
- **Time alignment:** timestamp both IMUs on a common clock. A handle/payload skew of more than about 0.2 ms noticeably degrades feedforward above roughly 50 Hz.
- **Consistency check:** monitor the normalized innovation squared, $\mathbf y^TS^{-1}\mathbf y$. It should follow a $\chi^2$ distribution with 3 or 6 degrees of freedom. A persistent excess indicates a wrong $D$ or unmodeled structural resonance.
- **Optional extension:** if handle gyro bias matters (long shoots, low-rate pans), extend to 13 states by adding $\mathbf b_h$. That goes beyond the 10-state requirement.