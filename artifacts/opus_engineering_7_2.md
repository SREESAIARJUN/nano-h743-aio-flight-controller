# Autonomous Gimbal Tracking Controller: PID + Velocity Feedforward + Acceleration Limiting

*(Cycle 7 formulation. Self-contained, so it does not depend on earlier cycles.)*

---

## 1. Problem Setup

**Goal:** keep a visually detected target at the image center (or a framing offset) with smooth, cinematic motion. The controller must reject detector noise, compensate camera latency, and respect actuator limits.

**Axes:** pan (yaw, ψ) and tilt (pitch, θ). They are controlled independently with identical structure. Roll is usually stabilized separately by the IMU loop.

**Architecture (cascaded):**

```
Vision (20–60 Hz) → Target Estimator → Outer Angle Loop (PID + FF) → Accel/Jerk Limiter → Inner Rate Loop (motor/IMU, 500–2000 Hz)
```

The design rests on two choices:

- The outer loop commands **angular rate**, not torque.
- The gimbal's inner stabilization loop tracks that rate.

---

## 2. Measurement Model: Pixel → Angle Error

Given the target centroid $(u, v)$, principal point $(c_x, c_y)$, and focal lengths $(f_x, f_y)$ in pixels:

$$
e_\psi = \arctan\!\left(\frac{u - c_x - u_{\text{off}}}{f_x}\right), \qquad
e_\theta = -\arctan\!\left(\frac{v - c_y - v_{\text{off}}}{f_y}\right)
$$

- $u_{\text{off}}, v_{\text{off}}$ are framing offsets, e.g. rule-of-thirds or lead room in the direction of motion.
- For a zoom lens, $f$ is looked up from the current zoom level. This keeps loop gain constant in angle space.

The absolute target bearing in the world/base frame is:

$$
\psi_T = \psi_G(t_{\text{img}}) + e_\psi, \qquad \theta_T = \theta_G(t_{\text{img}}) + e_\theta
$$

Here $\psi_G(t_{\text{img}})$ is the gimbal angle **at the image timestamp**, taken from a buffered encoder/IMU history. This step removes the gimbal's own motion from the measurement.

---

## 3. Target State Estimator (Velocity Source for Feedforward)

Use a constant-velocity Kalman filter per axis, with state $x = [\phi_T,\ \dot\phi_T]^\top$.

**Prediction:**

$$
x_{k|k-1} = F x_{k-1}, \quad F=\begin{bmatrix}1 & \Delta t\\ 0 & 1\end{bmatrix}, \quad
Q = q\begin{bmatrix}\Delta t^3/3 & \Delta t^2/2\\ \Delta t^2/2 & \Delta t\end{bmatrix}
$$

**Update** with measurement $z = \phi_T$, $H=[1\ \ 0]$, and noise $R=\sigma_{\text{pix}}^2/f^2$.

**Latency compensation.** Propagate the state to the current time:

$$
\hat\phi_T(t_{\text{now}}) = \hat\phi_T + \hat{\dot\phi}_T\,(t_{\text{now}} - t_{\text{img}})
$$

**Outlier gating.** Reject any measurement with Mahalanobis distance $d^2 > 9$ (≈3σ).

**Detection loss.** On a missed detection, coast on prediction and decay velocity:

$$
\hat{\dot\phi}_T \leftarrow \lambda\,\hat{\dot\phi}_T, \quad \lambda \approx 0.95 \text{ per cycle}
$$

After a timeout of about 1 s, enter hold/search mode.

**Tuning knob:** $q$ (process noise).

- Low $q$ gives smooth but laggy tracking.
- High $q$ gives responsive but jittery tracking.

---

## 4. Control Law

Tracking error in angle space, wrapped to $[-\pi, \pi]$ for pan:

$$
e(t) = \hat\phi_T(t) - \phi_G(t)
$$

**Deadband (smooth, continuous):**

$$
e_{db} = \operatorname{sign}(e)\,\max(0,\ |e| - \delta)
$$

A typical value is $\delta \approx 0.3$–$1°$. It suppresses micro-hunting from detector jitter.

**Rate command (PID + velocity feedforward):**

$$
\boxed{\ \omega_{\text{raw}} = \underbrace{K_{ff}\,\hat{\dot\phi}_T}_{\text{feedforward}} + K_p\, e_{db} + K_i \!\int e_{db}\,dt + K_d\,\dot e_f\ }
$$

**Term details:**

- **$K_{ff} \approx 0.9$–$1.0$.** The feedforward carries most of the tracking effort. The PID only corrects residual error, which is why the motion stays smooth.
- **Derivative on measurement, filtered.** Use

  $$
  \dot e_f = \text{LPF}_{\tau_d}\!\left(-\frac{d\phi_G}{dt}\right) + \hat{\dot\phi}_T
  $$

  or simply the filtered error derivative with $\tau_d \approx 3$–$5\,T_s$. In practice, with strong feedforward, $K_d$ is often small or zero.
- **Error-dependent gain scheduling (optional, cinematic).** Make the proportional gain grow with error:

  $$
  K_p(e) = K_{p0}\left(1 + \alpha \frac{|e|}{e_{\max}}\right)
  $$

  The result is gentle near center and assertive when the target nears the frame edge.

---

## 5. Acceleration and Jerk Limiting

Apply the limiter to the commanded rate before it is sent to the inner loop. All quantities are per axis, with control period $T_s$.

**1. Velocity saturation:**

$$
\omega_{\text{sat}} = \text{clip}(\omega_{\text{raw}},\ -\omega_{\max},\ \omega_{\max})
$$

**2. Acceleration limit (rate limiter):**

$$
a_{\text{des}} = \frac{\omega_{\text{sat}} - \omega_{k-1}}{T_s}, \qquad
a_k = \text{clip}(a_{\text{des}},\ -a_{\max},\ a_{\max})
$$

**3. Jerk limit (optional, recommended for video):**

$$
a_k \leftarrow a_{k-1} + \text{clip}(a_k - a_{k-1},\ -j_{\max}T_s,\ j_{\max}T_s)
$$

**4. Integrate to the final command:**

$$
\omega_k = \omega_{k-1} + a_k T_s
$$

**Overshoot-free braking** (prevents rate-limited overshoot near the target). Cap the speed toward the target so the gimbal can still stop in the remaining distance:

$$
|\omega_{\text{sat}}| \le \sqrt{2\,a_{\max}\,|e|} + |\hat{\dot\phi}_T|
$$

**Anti-windup (conditional integration).** Freeze the integrator when the limiter is active and the error pushes further into saturation:

$$
\text{if } (\omega_k \ne \omega_{\text{raw}}) \wedge (\operatorname{sign}(e)=\operatorname{sign}(\omega_{\text{raw}}-\omega_k)):\ \ \dot I = 0
$$

Alternatively, use back-calculation:

$$
\dot I = K_i e + \frac{1}{T_t}(\omega_k - \omega_{\text{raw}}), \qquad T_t \approx \sqrt{T_i T_d}
$$

---

## 6. Reference Implementation (Python, per axis)

```python
import math

class AxisTracker:
    def __init__(s, Ts, Kp, Ki, Kd, Kff, w_max, a_max, j_max,
                 deadband, tau_d, Tt, wrap=False):
        s.__dict__.update(locals()); del s.__dict__['s']
        s.I = 0.0; s.w = 0.0; s.a = 0.0; s.e_prev = 0.0; s.de_f = 0.0

    def _wrap(s, x):
        return (x + math.pi) % (2*math.pi) - math.pi if s.wrap else x

    def step(s, tgt_ang, tgt_rate, gimbal_ang):
        e = s._wrap(tgt_ang - gimbal_ang)
        e_db = math.copysign(max(0.0, abs(e) - s.deadband), e)

        # filtered derivative
        de = (e_db - s.e_prev) / s.Ts; s.e_prev = e_db
        alpha = s.Ts / (s.tau_d + s.Ts)
        s.de_f += alpha * (de - s.de_f)

        w_raw = s.Kff*tgt_rate + s.Kp*e_db + s.I + s.Kd*s.de_f

        # braking envelope + velocity saturation
        w_brake = math.sqrt(2*s.a_max*abs(e)) + abs(tgt_rate)
        w_sat = max(-min(s.w_max, w_brake), min(min(s.w_max, w_brake), w_raw))

        # accel + jerk limiting
        a_des = (w_sat - s.w) / s.Ts
        a_des = max(-s.a_max, min(s.a_max, a_des))
        da = max(-s.j_max*s.Ts, min(s.j_max*s.Ts, a_des - s.a))
        s.a += da
        s.w += s.a * s.Ts

        # back-calculation anti-windup
        s.I += s.Ts * (s.Ki*e_db + (s.w - w_raw)/s.Tt)
        return s.w   # rate command to inner stabilization loop
```

**Main loop sketch:**

```python
on_image(det, t_img):
    z = gimbal_hist(t_img) + pixel_to_angle(det)
    kf.update(z, t_img)

every Ts:
    phi_T, rate_T = kf.predict_to(now)
    cmd = tracker.step(phi_T, rate_T, gimbal_angle())
    send_rate(cmd)
```

---

## 7. Tuning Procedure

1. **Inner rate loop first.** Bandwidth should be at least 5× the outer loop (e.g. ≥15 Hz inner for a ~2–3 Hz outer loop).
2. **Limits.** Set $\omega_{\max}, a_{\max}, j_{\max}$ from mechanical limits and desired aesthetics. Typical cinematic values are 90°/s, 180°/s², and 1000°/s³.
3. **Feedforward.** Set $K_i=K_d=0$ and $K_{ff}=1$. Tune $K_p$ for about 1–2 Hz crossover, keeping it below ⅓ of the vision rate for stability with latency.
4. **Latency check.** Measure end-to-end latency $L$ (typically 50–150 ms). Phase margin requires $K_p L < \approx 0.5$ rad.
5. **Integral.** Add $K_i \approx K_p^2/4$ (start low) to eliminate bias from mounting or steady drift.
6. **Estimator.** Raise $q$ until the feedforward follows maneuvers without visible jitter.
7. **Deadband.** Size it from detector noise: $\delta \approx 2\sigma_{\text{pix}}/f$.

---

## 8. Stability Note

With accurate feedforward ($K_{ff}\hat{\dot\phi}_T \approx \dot\phi_T$), the closed-loop error dynamics reduce to:

$$
\dot e \approx -K_p e - K_i\!\int e \quad\Rightarrow\quad s^2 + K_p s + K_i = 0
$$

This system is stable for $K_p, K_i > 0$, and is critically damped when $K_i = K_p^2/4$.

The limiters are passive (they never inject energy), and the braking envelope guarantees no limit-induced overshoot. The remaining stability risk is the vision delay $L$, which is handled by the predictor and the constraint $K_p L < 0.5$.

---

## 9. Summary of Parameters

| Symbol | Role | Typical |
|---|---|---|
| $K_{ff}$ | Velocity feedforward | 0.9–1.0 |
| $K_p$ | Proportional (1/s) | 2–6 |
| $K_i$ | Integral (1/s²) | $K_p^2/4$ |
| $K_d$ | Derivative | 0–0.1 |
| $\delta$ | Deadband | 0.3–1° |
| $\omega_{\max}$ | Rate limit | 60–180°/s |
| $a_{\max}$ | Accel limit | 120–360°/s² |
| $j_{\max}$ | Jerk limit | 500–2000°/s³ |
| $q$ | KF process noise | tune to target agility |
| $L$ | Latency compensation | measured |

If you share specific hardware constraints (motor type, vision rate, latency, payload inertia), I can turn this into a concrete tuned parameter set or a discrete-time stability analysis for those values.