# Autonomous Gimbal Tracking Controller (Cycle 12)

**Scope:** A two-axis (pan/tilt) visual-servo controller. It combines PID on image-space error, feedforward of the estimated target angular velocity, a jerk-bounded acceleration/rate limiter, and anti-windup.

---

## 1. Problem Setup

**State per axis** (yaw ψ, pitch θ; treated independently and identically):

| Symbol | Meaning |
|---|---|
| θ_g | gimbal joint angle (encoder) |
| ω_g | gimbal angular rate (IMU/gyro) |
| (u, v) | target centroid in image (px) |
| (c_x, c_y) | principal point |
| f_x, f_y | focal lengths (px) |
| T_s | control period (e.g., 5 ms at 200 Hz) |
| T_v | vision period (e.g., 33 ms) plus latency τ_v |

**Angular error** (pinhole model):

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

The objective is to drive e → 0 smoothly, with |ω̇_cmd| ≤ a_max and bounded jerk.

---

## 2. Target Line-of-Sight (LOS) Estimation

The camera is mounted on the gimbal, so measured error mixes target motion with gimbal motion. Reconstruct the inertial LOS angle:

$$
\lambda_k = \theta_g(t_k - \tau_v) + e_k
$$

Use the gimbal angle **time-stamped at frame capture**, not at frame arrival.

Then run a constant-velocity Kalman filter on λ.

- State: x = [λ, λ̇]ᵀ
- Transition:

$$
F = \begin{bmatrix}1 & T_s\\ 0 & 1\end{bmatrix}, \quad
Q = q\begin{bmatrix}T_s^3/3 & T_s^2/2\\ T_s^2/2 & T_s\end{bmatrix}, \quad
H = [1\ 0]
$$

- Predict every T_s. Update only when a vision frame arrives.
- After the update, forward-predict by τ_v to compensate latency.
- Output: λ̂ and λ̂̇. The rate λ̂̇ is the **feedforward rate**.

Lost-target handling: coast on prediction for up to N frames while decaying λ̂̇ (e.g., multiplying by 0.9 each frame), then hold or hand off to the search mode.

---

## 3. Control Law (Rate-Command Gimbal)

The inner loop is the gimbal's own rate controller (motor driver plus gyro). The outer loop commands a rate:

$$
\omega_{raw} = \underbrace{K_{ff}\,\hat{\dot\lambda}}_{\text{feedforward}}
+ K_p\,\tilde e + K_i\!\int \tilde e\,dt + K_d\,\dot{\tilde e}_f
$$

where:
- **ẽ = λ̂ − θ_g** is the filtered, latency-compensated tracking error.
- **Derivative on error rate**: ė̃_f = λ̂̇ − ω_g. This avoids differentiating noisy pixels. Apply an optional first-order low-pass with cutoff f_d ≈ 10–20 Hz.
- **K_ff ≈ 0.9–1.0.** Slightly below 1 avoids overshoot when the λ̂̇ estimate is noisy.

**Deadband (smooth follow):** soft-scale the error to suppress jitter near center:

$$
\tilde e \leftarrow \tilde e \cdot \min\!\left(1, \left(\frac{|\tilde e|}{e_{db}}\right)^2\right)
$$

with e_db ≈ 0.2–0.5°.

---

## 4. Acceleration and Jerk Limiting

Rate-limit the commanded rate in two stages:

$$
\begin{aligned}
a_{des} &= \frac{\omega_{raw} - \omega_{cmd,k-1}}{T_s} \\
a_k &= \mathrm{clamp}\!\left(a_{des},\ a_{k-1} - j_{max}T_s,\ a_{k-1} + j_{max}T_s\right) \\
a_k &= \mathrm{clamp}(a_k,\ -a_{max},\ a_{max}) \\
\omega_{cmd,k} &= \mathrm{clamp}(\omega_{cmd,k-1} + a_k T_s,\ -\omega_{max},\ \omega_{max})
\end{aligned}
$$

**Overshoot prevention (braking-aware):** cap the rate so the gimbal can stop within the remaining error:

$$
|\omega_{cmd}| \le \sqrt{2\,a_{max}\,|\tilde e|} + |\hat{\dot\lambda}|
$$

**Mechanical limits:** near joint stops θ_min/θ_max, scale ω toward zero using the same braking relation.

---

## 5. Anti-Windup (Back-Calculation)

$$
I_k = I_{k-1} + T_s\left[K_i\,\tilde e_k + K_{aw}(\omega_{cmd,k} - \omega_{raw,k})\right],
\qquad K_{aw} \approx 1/T_s \ \text{to}\ \sqrt{K_i/K_p}
$$

Additionally:
- Freeze I while the target is lost.
- Clamp |I| ≤ I_max.

---

## 6. Reference Implementation (per axis)

```python
class AxisTracker:
    def __init__(s, Kp, Ki, Kd, Kff, Kaw, a_max, j_max, w_max, e_db, fd, Ts):
        s.__dict__.update(locals()); s.I = s.a = s.w = s.d_f = 0.0
        s.alpha = Ts / (Ts + 1/(2*3.14159*fd))

    def step(s, lam_hat, lam_dot_hat, theta_g, omega_g, target_valid):
        e = lam_hat - theta_g
        e *= min(1.0, (abs(e)/s.e_db)**2)                 # soft deadband
        s.d_f += s.alpha*((lam_dot_hat - omega_g) - s.d_f) # filtered D

        w_raw = s.Kff*lam_dot_hat + s.Kp*e + s.I + s.Kd*s.d_f

        # braking-aware rate cap
        w_brk = (2*s.a_max*abs(e))**0.5 + abs(lam_dot_hat)
        w_raw = max(-w_brk, min(w_brk, w_raw))

        # jerk + accel + rate limiting
        a_des = (w_raw - s.w)/s.Ts
        s.a = max(s.a - s.j_max*s.Ts, min(s.a + s.j_max*s.Ts, a_des))
        s.a = max(-s.a_max, min(s.a_max, s.a))
        s.w = max(-s.w_max, min(s.w_max, s.w + s.a*s.Ts))

        if target_valid:                                   # back-calc anti-windup
            s.I += s.Ts*(s.Ki*e + s.Kaw*(s.w - w_raw))
        return s.w
```

The feedforward term is the most important element: with K_ff ≈ 1, the PID only corrects residual error instead of chasing the target.

---

## 7. Tuning Procedure

1. **Inner rate loop first.** Verify bandwidth ≥ 5× the outer loop (typically 30–50 Hz).
2. **Set limits from hardware.** Choose a_max, ω_max, and j_max from motor torque and payload inertia, at roughly 70% of capability.
3. **P only** (K_ff = K_i = K_d = 0). Raise K_p until mild oscillation, then back off 50%. Typical value is 2–6 s⁻¹, with outer bandwidth ≈ K_p / 2π ≤ 1/(4 τ_v).
4. **Add K_ff.** Increase from 0.5 toward 1.0 while tracking a constant-velocity target. Steady lag should approach zero.
5. **Add K_i** for residual bias, using K_i ≈ K_p²/10.
6. **Add K_d** only if overshoot persists, with K_d ≈ 0.05–0.2.
7. **Tune deadband and KF noise** (q, R) for the trade-off between stillness and responsiveness.

---

## 8. Performance Targets

| Metric | Target |
|---|---|
| Steady tracking error (constant-ω target) | < 0.3° |
| Step settle (10°) | < 0.8 s, overshoot < 5% |
| Peak acceleration | ≤ a_max (enforced) |
| Jitter at rest | < 0.05° RMS |
| Re-acquisition after 0.5 s occlusion | < 1 frame of drift beyond predicted path |

---

## 9. Cycle 12 Additions (relative to a basic PID baseline)

- Latency-compensated LOS reconstruction using frame-time gimbal pose
- Feedforward from the Kalman-filtered target rate
- Jerk-bounded S-curve limiting plus braking-aware rate cap
- Back-calculation anti-windup with freeze on target loss
- Soft quadratic deadband for cinematic smoothness

**Next steps:**
- Add base-motion compensation by feeding the vehicle IMU rate into the feedforward term.
- Add gain scheduling on target range and zoom level: scale K_p by f_ref/f, because angular error per pixel shrinks as zoom increases.