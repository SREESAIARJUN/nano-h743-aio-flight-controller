# Autonomous Gimbal Tracking Controller: PID + Velocity Feedforward + Acceleration Limiting (Cycle 20)

I'm treating "Cycle 20" as the 20th design iteration. The aim of this iteration is to remove jerky starts and stops and steady-state lag on moving targets, while keeping the camera image smooth.

---

## 1. System Definition

**Axes:** Pan (yaw, ψ) and Tilt (pitch, θ). The axes are treated as decoupled SISO loops, with cross-coupling compensated in §6.

**Signals per axis (shown for pan; tilt is identical):**

| Symbol | Meaning |
|---|---|
| $u_k, v_k$ | Target centroid in image (px) |
| $c_x, c_y$ | Principal point (px) |
| $f_x, f_y$ | Focal length (px), zoom-dependent |
| $\psi_k$ | Gimbal pan angle (encoder) |
| $\omega_k$ | Commanded pan rate (rad/s) |
| $\omega_b$ | Base (vehicle) yaw rate from IMU |
| $T_s$ | Control period (e.g. 5 ms at 200 Hz) |
| $T_v$ | Vision period (e.g. 33 ms at 30 Hz), latency $\tau_v$ |

The plant is the inner gimbal rate loop, normally run by the motor driver (FOC + gyro). It is modeled as first order:

$$G_\omega(s) = \frac{1}{\tau_m s + 1}, \quad \tau_m \approx 10\text{–}30\ \text{ms}$$

The outer tracking loop designed here outputs a **rate command** $\omega_k$.

---

## 2. Tracking Error (Image → Angle)

Use a pinhole model with an exact arctangent. This avoids the small-angle gain error at wide FOV:

$$e_{\psi,k} = \arctan\!\left(\frac{u_k - c_x}{f_x}\right), \qquad e_{\theta,k} = \arctan\!\left(\frac{v_k - c_y}{f_y}\right)$$

**Latency compensation.** The detection describes the scene at time $k - \tau_v$, but the gimbal has moved since then. Re-reference the error using buffered encoder history:

$$\tilde e_{\psi,k} = e_{\psi,k} - \big(\psi_k - \psi_{k-\tau_v}\big)$$

**Deadband (smooth, not hard).** This suppresses jitter near center without a discontinuity:

$$e^{db} = \begin{cases} 0 & |e| < \delta \\ \operatorname{sign}(e)\,\dfrac{(|e|-\delta)^2}{2\delta} & \delta \le |e| < 2\delta \\ e - \operatorname{sign}(e)\,\tfrac{3}{2}\delta & \text{else}\end{cases}$$

A typical value is $\delta \approx 0.1^\circ$ to $0.3^\circ$. The two branches join with matching value and slope, so the output stays C¹-continuous.

---

## 3. Target Motion Estimator (for Feedforward)

Estimate the target's **inertial** bearing $\lambda$ and its rate. The measurement is:

$$\lambda_k^{meas} = \psi_{k-\tau_v} + e_{\psi,k}$$

Run a constant-velocity Kalman filter (or an α-β filter as a lightweight option) at the vision rate. The state is $x = [\lambda,\ \dot\lambda]^T$:

$$x_{k+1} = \begin{bmatrix}1 & T_v\\0 & 1\end{bmatrix}x_k + w_k, \qquad Q = q\begin{bmatrix}T_v^3/3 & T_v^2/2\\ T_v^2/2 & T_v\end{bmatrix}, \qquad z_k = [1\ \ 0]\,x_k + n_k$$

- **Tuning:** $q$ sets agility. Small $q$ gives smooth feedforward; large $q$ follows maneuvers faster.
- **Between frames:** propagate at $T_s$ so the controller always has a fresh $\hat\lambda$ and $\hat{\dot\lambda}$.
- **Target loss:** if no detection arrives for more than $N$ frames, decay $\hat{\dot\lambda} \leftarrow \hat{\dot\lambda}\, e^{-T_s/\tau_{coast}}$ (coast mode). Then freeze it and hand off to search mode.

---

## 4. Control Law

### 4.1 Total command

$$\omega^{cmd}_k = \underbrace{K_{ff}\,\hat{\dot\lambda}_k}_{\text{target FF}} \;-\; \underbrace{\omega_{b,k}}_{\text{base comp.}} \;+\; \underbrace{K_p\,\tilde e_k + K_i\, I_k + K_d\, D_k}_{\text{PID correction}}$$

- **Target feedforward** carries most of the motion. With $K_{ff}\approx 1$, the PID only corrects residual error, which removes the steady lag a pure PID shows on a ramp.
- **Base compensation** is needed when the gimbal is not already gyro-stabilized in an inertial frame. Omit it if the inner loop is inertially stabilized.

### 4.2 Integral with anti-windup (conditional integration + back-calculation)

$$I_k = I_{k-1} + T_s\left[\tilde e_k + \frac{1}{T_t K_i}\big(\omega^{out}_{k-1} - \omega^{cmd}_{k-1}\big)\right]$$

- Here $T_t \approx \sqrt{T_i T_d}$ is the tracking time constant.
- Freeze $I$ when the rate or acceleration limiter is active **and** $\operatorname{sign}(\tilde e) = \operatorname{sign}(I)$.
- Clamp $|K_i I| \le \omega_{I,max}$.

### 4.3 Derivative on measurement, filtered

Take the derivative of the measurement rather than the error. This avoids a "derivative kick" when the target jumps or re-acquires.

$$D_k = \frac{\tau_d}{\tau_d + T_s} D_{k-1} - \frac{1}{\tau_d + T_s}\big(y_k - y_{k-1}\big), \qquad y_k = -\tilde e_k$$

Use $\tau_d = T_d/N$ with $N \approx 8$ to $20$. Because of vision noise, the derivative term is often small or zero once feedforward is present.

---

## 5. Rate and Acceleration Limiting (Smoothness Stage)

Apply the limiters in this order after computing $\omega^{cmd}_k$:

**(a) Rate saturation**
$$\omega^{sat}_k = \operatorname{clip}(\omega^{cmd}_k,\ -\omega_{max},\ \omega_{max})$$

**(b) Acceleration limit (slew-rate on rate)**
$$\Delta = \operatorname{clip}\!\left(\omega^{sat}_k - \omega^{out}_{k-1},\ -a_{max}T_s,\ a_{max}T_s\right), \qquad \omega^{out}_k = \omega^{out}_{k-1} + \Delta$$

**(c) Optional jerk limit (S-curve).** Apply the same clip to the acceleration itself:
$$a_k = \operatorname{clip}\!\left(\tfrac{\Delta}{T_s},\ a_{k-1} - j_{max}T_s,\ a_{k-1} + j_{max}T_s\right), \qquad \omega^{out}_k = \omega^{out}_{k-1} + a_k T_s$$

**(d) Error-adaptive acceleration.** This keeps small corrections gentle and large slews fast:
$$a_{max}(\tilde e) = a_{min} + (a_{hi} - a_{min})\cdot \operatorname{sat}\!\left(\frac{|\tilde e|}{e_{big}}\right)$$

**(e) Braking-distance check** to prevent overshoot when closing on a target. Require
$$|\omega^{out} - \hat{\dot\lambda}| \le \sqrt{2\, a_{max}\, |\tilde e|}$$
and clip $\omega^{out}$ accordingly. This is the time-optimal approach curve.

**(f) Mechanical soft limits.** Near a joint limit $\psi_{lim}$, allow only
$$|\omega| \le \sqrt{2\,a_{max}\,|\psi_{lim} - \psi|}$$
in the direction of the limit.

---

## 6. Cross-Axis and Geometry Corrections

- **Pan gain scheduling for tilt elevation.** Near the zenith the pan axis is ineffective, so pan feedforward needs $\hat{\dot\lambda}_\psi / \cos\theta$. Clamp $\cos\theta \ge \cos 80^\circ$ to bound the gain near gimbal lock.
- **Zoom gain scheduling.** The error is already in radians via $f_x(\text{zoom})$. Additionally scale $K_p$ down by about $\sqrt{f/f_0}$ at long focal lengths, where pixel noise maps to tiny angles and the image appears to shake more.

---

## 7. Tuning Procedure

1. **Inner rate loop:** verify a bandwidth of at least 15–30 Hz and identify $\tau_m$.
2. **Feedforward first:** set $K_p = K_i = K_d = 0$ and track a constant-speed target. Adjust $K_{ff}$ (≈0.9–1.0) until the drift is minimal.
3. **$K_p$:** choose an outer bandwidth $\omega_c \approx 1$–3 rad/s, kept well below the vision rate and below $1/(3\tau_v)$. Then $K_p \approx \omega_c$ (in 1/s, since the command is rate per error angle).
4. **$K_i$:** set $T_i \approx 4/\omega_c$, i.e. $K_i = K_p/T_i$. Use it only to remove bias such as calibration offset or FF mismatch.
5. **$K_d$:** start at 0. Add a small amount only if overshoot remains after acceleration limiting.
6. **Limits:** set $a_{max}$ from the desired visual comfort, typically 60–200 °/s² for cinematic motion and higher for surveillance. Set $j_{max}$ to about $5$–$10 \times a_{max}$ per second.
7. **Phase margin:** verify at least 45° including the latency term $e^{-s\tau_v}$:
$$L(s) = \frac{K_p + K_i/s}{s(\tau_m s + 1)}\, e^{-s\tau_v}$$

**Example starting values** (30 Hz vision, $\tau_v$ = 60 ms):

| Parameter | Value |
|---|---|
| $K_p$ | 2.0 s⁻¹ |
| $K_i$ | 0.4 s⁻² |
| $K_d$ | 0.05 |
| $K_{ff}$ | 0.95 |
| $\omega_{max}$ | 90 °/s |
| $a_{max}$ | 30–250 °/s² (adaptive) |
| $j_{max}$ | 1500 °/s³ |
| $\delta$ | 0.15° |

---

## 8. Reference Implementation (per axis, runs at $T_s$)

```python
class AxisTracker:
    def __init__(s, p):
        s.p = p; s.I = 0.0; s.D = 0.0; s.y_prev = 0.0
        s.w_out = 0.0; s.a_prev = 0.0; s.w_cmd_prev = 0.0

    def step(s, e_raw, lam_dot_hat, w_base, Ts, sat_info=None):
        p = s.p
        e = smooth_deadband(e_raw, p.delta)          # latency-compensated input

        # Derivative on measurement (filtered)
        y = -e
        s.D = (p.tau_d*s.D - (y - s.y_prev)) / (p.tau_d + Ts)
        s.y_prev = y

        # Feedforward + PID
        ff = p.Kff * lam_dot_hat - w_base
        w_cmd = ff + p.Kp*e + p.Ki*s.I + p.Kd*s.D

        # Rate limit
        w_sat = clip(w_cmd, -p.w_max, p.w_max)

        # Error-adaptive accel limit + jerk limit
        a_max = p.a_min + (p.a_hi - p.a_min) * min(abs(e)/p.e_big, 1.0)
        a_req = (w_sat - s.w_out) / Ts
        a_req = clip(a_req, -a_max, a_max)
        a = clip(a_req, s.a_prev - p.j_max*Ts, s.a_prev + p.j_max*Ts)
        w_new = s.w_out + a*Ts

        # Braking envelope relative to target rate
        v_rel_max = (2*a_max*abs(e))**0.5
        w_new = clip(w_new, lam_dot_hat - v_rel_max - p.v_slack,
                            lam_dot_hat + v_rel_max + p.v_slack)

        # Anti-windup: back-calculation + conditional freeze
        limited = abs(w_new - w_cmd) > 1e-6
        if not (limited and sign(e) == sign(s.I)):
            s.I += Ts * (e + (w_new - w_cmd) / (p.Tt * p.Ki))
        s.I = clip(s.I, -p.I_max/p.Ki, p.I_max/p.Ki)

        s.a_prev = (w_new - s.w_out) / Ts
        s.w_out = w_new
        return w_out_to_motor(w_new)
```

---

## 9. Mode Logic

| Mode | Behavior |
|---|---|
| **ACQUIRE** | High $a_{max}$, $K_i$ disabled, braking envelope active. Switch to TRACK when $\lvert e\rvert < 2^\circ$ for 0.3 s. |
| **TRACK** | Full law as above. Adaptive $a_{max}$, deadband on. |
| **COAST** | No detections. Feedforward only with decaying $\hat{\dot\lambda}$; PID frozen. |
| **LOST / SEARCH** | Scripted scan pattern at low $a_{max}$. Reset $I$ and $D$, and re-initialize the Kalman filter on reacquire. |

Mode transitions are **bumpless**: initialize `w_out` to the current measured rate and preload $I$ so that $\omega^{cmd} = \omega^{out}$ at the switch instant.

---

## 10. Cycle-20 Acceptance Metrics

- **Steady ramp tracking error:** below 0.2° RMS at 20 °/s target speed, because feedforward removes the velocity lag.
- **Peak acceleration:** never above $a_{max}$; peak jerk never above $j_{max}$.
- **Step reacquire:** overshoot below 5% and settling to 0.5° in under 1.0 s for a 20° step.
- **Image smoothness:** angular jitter of the gimbal rate command in the 1–10 Hz band below 0.5 °/s RMS while tracking a stationary target.
- **Stability:** gain margin of at least 6 dB and phase margin of at least 45° with $+30\%$ added vision latency.