# Autonomous Gimbal Visual-Follow Controller: Cycle 11 Formulation

The controller is a cascaded **PID + velocity feedforward** design with **rate and acceleration limiting**, latency compensation, and anti-windup. It is designed for pan/tilt (yaw/pitch) gimbals tracking a target detected in the camera image.

---

## 1. System Definition

**States per axis** (for yaw ψ and pitch θ; written below for a generic axis q):

| Symbol | Meaning |
|---|---|
| q, q̇ | Gimbal joint angle and rate (encoder/IMU) |
| (uₜ, vₜ) | Target pixel centroid from the detector |
| (c_x, c_y), f_x, f_y | Principal point and focal lengths (px) |
| ω_cmd | Commanded gimbal rate (output to the inner rate loop) |
| T_s | Control period |
| τ_d | Vision pipeline latency (capture → detection) |

**Angular tracking error** (pixel → angle, exact pinhole form):

$$
e_\psi = \arctan\!\left(\frac{u_t - c_x}{f_x}\right), \qquad
e_\theta = -\arctan\!\left(\frac{v_t - c_y}{f_y}\right)
$$

For pitch, a sign flip is applied because image v increases downward.

---

## 2. Target Motion Estimation (Feedforward Source)

The target's **inertial bearing** is reconstructed so that gimbal motion is not mistaken for target motion:

$$
\beta_k = q_{k-d} + e_k
$$

Here q_{k−d} is the gimbal angle **at image capture time**, taken from a timestamped encoder buffer with d = ⌈τ_d / T_s⌉.

A **constant-velocity Kalman filter** runs on β with state x = [β, β̇]ᵀ:

$$
x_{k+1} = \begin{bmatrix}1 & T_s\\0 & 1\end{bmatrix} x_k + w_k,\quad
Q = \sigma_a^2\begin{bmatrix}T_s^4/4 & T_s^3/2\\ T_s^3/2 & T_s^2\end{bmatrix},\quad
z_k = \beta_k + \nu_k
$$

**Latency compensation.** The estimate is propagated forward by τ_d to obtain the present-time prediction:

$$
\hat\beta_{now} = \hat\beta + \hat{\dot\beta}\,\tau_d,\qquad
\hat e_{now} = \hat\beta_{now} - q_k
$$

**Missed detections.** When a detection is missing, only the prediction step runs. After N_lost frames, the feedforward gain decays: ŝ ← γ·ŝ with γ ≈ 0.9.

---

## 3. Control Law

$$
\omega_{raw} = \underbrace{K_{ff}\,\hat{\dot\beta}}_{\text{feedforward}}
+ K_p\,\hat e_{now}
+ K_i \!\int \hat e_{now}\,dt
+ K_d\,\dot e_f
$$

**Derivative on filtered error** (first-order low-pass, cutoff ω_c):

$$
\dot e_{f,k} = \frac{\alpha\,\dot e_{f,k-1} + (\hat e_k - \hat e_{k-1})/T_s \cdot (1-\alpha)}{1},\quad \alpha = e^{-\omega_c T_s}
$$

In practice, set K_d small or zero. With feedforward active, the P term mostly handles residual error.

**Deadband with smooth blending.** This prevents jitter on a centered target:

$$
e' = \begin{cases} 0 & |e|<e_{db}\\ \text{sign}(e)\,(|e|-e_{db}) & \text{otherwise}\end{cases}
$$

The blended error e' replaces ê_now in the P and I terms.

---

## 4. Rate, Acceleration, and Jerk Limiting

**Rate saturation:**

$$
\omega_{sat} = \text{clip}(\omega_{raw},\,-\omega_{max},\,\omega_{max})
$$

**Acceleration limit** (slew-rate limiter on the command):

$$
\omega_{cmd,k} = \omega_{cmd,k-1} + \text{clip}\!\left(\omega_{sat}-\omega_{cmd,k-1},\; -a_{max}T_s,\; a_{max}T_s\right)
$$

**Optional jerk limit** (for cinematic smoothness):

$$
a_k = a_{k-1} + \text{clip}\!\left(\frac{\omega_{sat}-\omega_{cmd,k-1}}{T_s} - a_{k-1},\,-j_{max}T_s,\,j_{max}T_s\right),\quad a_k \leftarrow \text{clip}(a_k,\pm a_{max})
$$

$$
\omega_{cmd,k} = \omega_{cmd,k-1} + a_k T_s
$$

**Braking-aware limit (overshoot prevention).** The command is capped so the gimbal can stop at the target:

$$
|\omega_{sat}| \le \sqrt{2\,a_{max}\,|\hat e_{now}|} + |\hat{\dot\beta}|
$$

---

## 5. Anti-Windup (Back-Calculation)

$$
I_k = I_{k-1} + T_s\left[K_i\, e'_k + K_{aw}\,(\omega_{cmd,k} - \omega_{raw,k})\right]
$$

- Choose K_aw ≈ 1/T_i to 2/T_i.
- Freeze I when the target is lost or a joint limit is reached.

---

## 6. Mechanical Limits and Coupling

**Joint limits.** Taper ω_max near the end-stops:

$$
\omega_{max}^{\pm}(q) = \min\!\left(\omega_{max},\sqrt{2a_{max}|q_{lim}^{\pm}-q|}\right)
$$

**Yaw–pitch coupling.** Near high pitch, yaw rate requirements grow as 1/cos θ:

- Apply ω_ψ,cmd · cos θ scaling to the image-space error mapping.
- Alternatively, cap the yaw demand when |θ| > 75°.

**Base motion (vehicle-mounted).** Add base-rate rejection using the IMU:

$$
\omega_{cmd} \mathrel{+}= -\,\omega_{base,\text{axis}}
$$

---

## 7. Discrete Implementation (Pseudocode)

```python
def step(det, q, q_buf, imu_base, dt):
    # 1. Measurement (latency-aligned)
    if det.valid:
        e_meas = atan((det.u - cx)/fx)
        beta   = q_buf.at(det.t_capture) + e_meas
        kf.update(beta)
        lost = 0
    else:
        lost += 1
    kf.predict(dt)
    beta_hat, beta_dot = kf.x
    beta_now = beta_hat + beta_dot * tau_d
    e = wrap(beta_now - q)

    # 2. Deadband
    e_db = sign(e) * max(abs(e) - E_DB, 0.0)

    # 3. Feedforward gain fade on loss
    ff_gain = KFF * (GAMMA ** max(0, lost - N_LOST))

    # 4. PID + FF
    d_f = alpha*d_f + (1-alpha)*(e - e_prev)/dt
    w_raw = ff_gain*beta_dot + KP*e_db + I + KD*d_f

    # 5. Limits: braking, joint, rate
    w_brake = sqrt(2*A_MAX*abs(e)) + abs(beta_dot)
    w_lim   = min(W_MAX, w_brake, joint_taper(q))
    w_sat   = clip(w_raw, -w_lim, w_lim)

    # 6. Jerk + accel limit
    a_des = (w_sat - w_cmd)/dt
    a     = clip(a + clip(a_des - a, -J_MAX*dt, J_MAX*dt), -A_MAX, A_MAX)
    w_cmd = w_cmd + a*dt

    # 7. Anti-windup integrator
    if lost < N_LOST and not at_joint_limit(q):
        I += dt*(KI*e_db + KAW*(w_cmd - w_raw))

    e_prev = e
    return w_cmd - imu_base   # base-motion rejection
```

---

## 8. Tuning Guide (Cycle 11 Defaults)

These values are a starting point for a 50–100 Hz loop with ~60 ms vision latency.

| Parameter | Yaw | Pitch | Notes |
|---|---|---|---|
| K_p (1/s) | 4.0 | 4.5 | Bandwidth ≈ K_p; keep < 1/(3τ_d) |
| K_i (1/s²) | 0.8 | 1.0 | T_i ≈ 5/K_p |
| K_d (–) | 0.05 | 0.05 | Often 0 with feedforward |
| K_ff | 0.95 | 0.95 | <1 avoids overshoot from estimation lag |
| ω_max (°/s) | 120 | 90 | |
| a_max (°/s²) | 300 | 250 | Lower values give a more cinematic feel |
| j_max (°/s³) | 1500 | 1200 | |
| e_db (°) | 0.3 | 0.3 | About 1–2 px equivalent |
| σ_a (°/s²) | 50 | 40 | KF process noise (target agility) |
| ω_c (Hz) | 8 | 8 | Derivative filter |

**Procedure:**
1. Tune the inner rate loop first, to a bandwidth at least 5× the outer loop.
2. Set K_ff = 0 and raise K_p until slight overshoot appears, then back off 30%.
3. Enable feedforward and increase K_ff toward 1 while tracking a constant-velocity target. Steady lag should approach zero.
4. Add K_i only for residual bias such as cable drag or imbalance.
5. Reduce a_max and j_max until motion looks smooth, while keeping the target inside the safe frame region (e.g., the central 60%).

---

## 9. Stability and Performance Notes

- **Delay margin.** With loop gain ≈ K_p/s and delay τ_d, the phase margin is approximately 90° − K_p·τ_d·(180/π). Keep K_p·τ_d < 0.5 rad for a margin above 60°.
- **Steady-state error.** With ideal feedforward, ramp-tracking error is zero. With K_ff < 1, the residual ramp error is approximately (1 − K_ff)·β̇ / K_p, and the integrator removes it.
- **Limiter interaction.** Acceleration limiting adds effective lag. The braking-aware cap and back-calculation anti-windup together prevent limit cycles when the limits are active.
- **Recommended metrics to log each cycle:** RMS pixel error, peak overshoot, jerk RMS, time-to-reacquire after occlusion, and the percentage of time spent saturated.

---

**Cycle 11 changes vs. a typical prior iteration:**
- Latency-aligned bearing reconstruction
- Braking-aware rate cap
- Jerk limiter
- Feedforward fade on target loss
- Joint-limit tapering