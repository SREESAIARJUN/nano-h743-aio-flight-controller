# Autonomous Gimbal Tracking Controller: PID + Velocity Feedforward + Acceleration Limiting (Cycle 30)

## 1. System Architecture

```
 Camera ─► Detector/Tracker ─► Angular Error ─► Target State Estimator ─┐
   ▲            (u,v,t_img)        (e_ψ, e_θ)     (α-β / Kalman, latency │
   │                                               compensation)        │
   │                                                                    ▼
   │     ┌──────────────── Outer Visual Loop (30–60 Hz) ──────────────────┐
   │     │ ω_ff (feedforward) + PID(e) → saturate → accel/jerk limit →    │
   │     │ anti-windup back-calculation                                   │
   │     └────────────────────────────────┬───────────────────────────────┘
   │                                      ▼ ω_cmd
   │                    Inner Rate Loop (IMU/encoder, 500–2000 Hz)
   └──────────────────────────── Gimbal motors (pan ψ, tilt θ)
```

The outer loop outputs **rate commands**. The inner loop (usually provided by the gimbal firmware) handles motor dynamics, which decouples slow vision latency from fast stabilization.

---

## 2. Measurement Model

### 2.1 Pixel → angular error
With intrinsics (f_x, f_y, c_x, c_y) and detected target center (u, v):

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

(The sign on tilt accounts for image v increasing downward.)

### 2.2 Inertial target bearing
Using the gimbal angles **at image capture time** t_img, not at processing time:

$$
\psi_T = \psi_g(t_{img}) + e_\psi, \qquad \theta_T = \theta_g(t_{img}) + e_\theta
$$

Store a ring buffer of gimbal angles indexed by timestamp so you can look up ψ_g(t_img).

---

## 3. Target State Estimation (Feedforward Source)

Run a constant-velocity Kalman filter per axis, with state x = [φ_T, ω_T]ᵀ:

$$
x_{k+1} = \begin{bmatrix}1 & \Delta t\\ 0 & 1\end{bmatrix}x_k + w_k,\quad
Q = q\begin{bmatrix}\Delta t^3/3 & \Delta t^2/2\\ \Delta t^2/2 & \Delta t\end{bmatrix},\quad
z_k = [1\;\;0]\,x_k + v_k
$$

A lighter alternative is the α-β filter:

$$
\hat\phi^- = \hat\phi + \hat\omega\Delta t,\quad r = z - \hat\phi^-,\quad
\hat\phi = \hat\phi^- + \alpha r,\quad \hat\omega = \hat\omega + \tfrac{\beta}{\Delta t} r
$$

Typical values are α ≈ 0.4–0.6 and β ≈ 0.05–0.15.

### Latency compensation
Total latency is τ = t_now − t_img (capture, inference, and transport). Predict the target forward to the present:

$$
\hat\phi_T(t_{now}) = \hat\phi_T + \hat\omega_T\,\tau
$$

The controller's error then becomes e = φ̂_T(t_now) − φ_g(t_now).

---

## 4. Control Law

For each axis i ∈ {ψ, θ}:

$$
\boxed{\;\omega_{raw} = K_{ff}\,\hat\omega_T \;+\; K_p\, e \;+\; K_i\!\int e\,dt \;+\; K_d\,\dot e_f\;}
$$

- **Feedforward** (K_ff ≈ 0.8–1.0) carries most of the tracking effort. PID then only corrects residual error, which is what makes the follow smooth.
- **Derivative on filtered error**, using a first-order filter with time constant T_f:

$$
\dot e_f[k] = \frac{T_f}{T_f+\Delta t}\dot e_f[k-1] + \frac{1}{T_f+\Delta t}\big(e[k]-e[k-1]\big)
$$

- **Smoothing deadband** (prevents hunting on detector jitter). Inside the deadband, P and I are suppressed but feedforward remains active:

$$
e' = \begin{cases}0 & |e| < \epsilon_{db}\\ e - \epsilon_{db}\,\text{sgn}(e) & \text{otherwise}\end{cases}
$$

- **Pan coupling compensation** keeps a world-frame yaw rate correct at high tilt:

$$
\omega_{\psi,cmd} \leftarrow \omega_{\psi,cmd}/\max(\cos\theta_g,\ c_{min})
$$

---

## 5. Saturation, Acceleration and Jerk Limiting

Apply these in order each cycle.

**(a) Velocity saturation**

$$
\omega_{sat} = \text{clip}(\omega_{raw},\ -\omega_{max},\ \omega_{max})
$$

**(b) Acceleration limit (slew rate)**

$$
a_{req} = \frac{\omega_{sat} - \omega_{cmd}[k-1]}{\Delta t},\qquad
a = \text{clip}(a_{req},\ -a_{max},\ a_{max})
$$

**(c) Optional jerk limit** (S-curve, for cinematic smoothness)

$$
a = \text{clip}\big(a,\ a[k-1]-j_{max}\Delta t,\ a[k-1]+j_{max}\Delta t\big)
$$

$$
\omega_{cmd}[k] = \omega_{cmd}[k-1] + a\,\Delta t
$$

**(d) Anti-windup by back-calculation**

$$
I[k] = I[k-1] + \Big(K_i\, e' + \tfrac{1}{T_t}\big(\omega_{cmd}[k] - \omega_{raw}\big)\Big)\Delta t,\qquad T_t \approx \sqrt{T_i T_d}
$$

This bleeds off the integrator whenever the limiters clip the command, so the gimbal does not overshoot after a fast slew.

**(e) Mechanical limits**: if a joint is near its end stop and ω_cmd points into it, set ω_cmd = 0 and freeze the integrator.

---

## 6. Target Loss and Reacquisition

| State | Condition | Behavior |
|---|---|---|
| TRACK | confidence ≥ c_hi | Full law above |
| COAST | lost < T_coast (~0.5–1 s) | ω_cmd = K_ff ω̂_T·e^(−t/τ_c); predict forward; P and I disabled |
| HOLD/SEARCH | lost > T_coast | Decelerate to 0 (accel-limited), then optional search pattern |
| REACQUIRE | detected again | Reset D-term history; reinitialize the filter with the new bearing; ramp gains over ~0.3 s (bumpless transfer) |

---

## 7. Reference Implementation (Python)

```python
import math
from dataclasses import dataclass

@dataclass
class AxisParams:
    kp: float = 2.5      # 1/s
    ki: float = 0.4      # 1/s^2
    kd: float = 0.08     # s
    kff: float = 0.95
    tf: float = 0.05     # D filter time constant [s]
    tt: float = 0.3      # anti-windup tracking time [s]
    w_max: float = math.radians(180)
    a_max: float = math.radians(400)
    j_max: float = math.radians(4000)
    deadband: float = math.radians(0.3)

class AlphaBeta:
    def __init__(self, a=0.5, b=0.1):
        self.a, self.b = a, b
        self.x, self.v, self.init = 0.0, 0.0, False

    def update(self, z, dt):
        if not self.init:
            self.x, self.v, self.init = z, 0.0, True
            return
        xp = self.x + self.v * dt
        r = z - xp
        self.x = xp + self.a * r
        self.v += (self.b / dt) * r

class AxisController:
    def __init__(self, p: AxisParams):
        self.p = p
        self.I = 0.0
        self.e_prev = 0.0
        self.d_f = 0.0
        self.w_cmd = 0.0
        self.a_prev = 0.0

    def reset_derivative(self, e):
        self.e_prev, self.d_f = e, 0.0

    def step(self, e, w_ff, dt, track=True):
        p = self.p
        # Deadband shaping
        e_s = 0.0 if abs(e) < p.deadband else e - math.copysign(p.deadband, e)

        # Filtered derivative
        self.d_f = (p.tf * self.d_f + (e - self.e_prev)) / (p.tf + dt)
        self.e_prev = e

        if track:
            w_raw = p.kff * w_ff + p.kp * e_s + self.I + p.kd * self.d_f
        else:
            w_raw = p.kff * w_ff  # coast mode

        # Velocity saturation
        w_sat = max(-p.w_max, min(p.w_max, w_raw))

        # Acceleration and jerk limits
        a = (w_sat - self.w_cmd) / dt
        a = max(-p.a_max, min(p.a_max, a))
        a = max(self.a_prev - p.j_max * dt, min(self.a_prev + p.j_max * dt, a))
        self.a_prev = a
        self.w_cmd += a * dt

        # Back-calculation anti-windup
        if track:
            self.I += (p.ki * e_s + (self.w_cmd - w_raw) / p.tt) * dt
        return self.w_cmd

class GimbalTracker:
    def __init__(self, fx, fy, cx, cy):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.est = {"pan": AlphaBeta(), "tilt": AlphaBeta()}
        self.ctl = {"pan": AxisController(AxisParams()),
                    "tilt": AxisController(AxisParams(w_max=math.radians(120)))}

    def update(self, det, gimbal_at_capture, gimbal_now, t_img, t_now, dt):
        """det: (u, v, conf) or None; gimbal_*: (pan, tilt) in rad."""
        tau = t_now - t_img
        track = det is not None and det[2] > 0.5
        if track:
            u, v, _ = det
            e_pan = math.atan((u - self.cx) / self.fx)
            e_tilt = -math.atan((v - self.cy) / self.fy)
            self.est["pan"].update(gimbal_at_capture[0] + e_pan, dt)
            self.est["tilt"].update(gimbal_at_capture[1] + e_tilt, dt)

        out = {}
        for i, ax in enumerate(("pan", "tilt")):
            f = self.est[ax]
            target_now = f.x + f.v * tau            # latency compensation
            e = target_now - gimbal_now[i]
            if ax == "pan":
                e = math.atan2(math.sin(e), math.cos(e))  # wrap to [-pi, pi]
            out[ax] = self.ctl[ax].step(e, f.v, dt, track)

        out["pan"] /= max(math.cos(gimbal_now[1]), 0.2)  # pan/tilt coupling
        return out["pan"], out["tilt"]  # rate commands to inner loop
```

---

## 8. Tuning Procedure

1. **Inner rate loop first.** Verify a closed-loop bandwidth of at least 5× the outer loop's target bandwidth.
2. **Feedforward only** (Kp = Ki = Kd = 0, K_ff = 1). Pan at a constant-speed target and tune α/β until ω̂_T is clean. Lower K_ff slightly if you see overshoot from estimator lag.
3. **Kp.** Raise it until the step response settles in ~0.3–0.5 s without oscillation. As a rule of thumb, keep the outer-loop crossover ω_c ≲ 1/(3τ) because vision latency τ dominates phase loss.
4. **Kd.** Add a small amount only if Kp alone overshoots, and keep T_f ≥ 2Δt.
5. **Ki.** Add it last and keep it small; it only needs to remove steady bias such as mount misalignment or wind.
6. **Limits.** Set a_max to about 60–70% of the motor torque capability divided by payload inertia. Use j_max for footage smoothness: lower values give more cinematic motion.
7. **Deadband.** Set ε_db to roughly 1–2σ of the detector's pixel jitter converted to angle.

### Stability check
Approximate the linearized outer loop as L(s) = (Kp + Ki/s + Kd·s)·e^(−τs)/s. You want a phase margin of at least 45°:

$$
\text{PM} \approx 90^\circ - \omega_c\tau\cdot\tfrac{180}{\pi} + \angle C(j\omega_c)
$$

For τ = 80 ms, this keeps ω_c below roughly 4 rad/s, which matches Kp ≈ 2–3.

---

## 9. Performance Metrics to Log

- RMS angular tracking error (the target is typically < 1° on moving subjects)
- Peak |ω̇| and |ω̈|, to confirm the limits are respected
- Integrator saturation time fraction
- Reacquisition time after occlusion
- Frame-to-frame jitter (std. dev. of ω_cmd differences), a proxy for smoothness

This combination produces a follow behavior that is predictive and lag-free because of the latency-compensated feedforward, smooth because of the acceleration/jerk limits and deadband, and robust to saturation and occlusion because of back-calculation anti-windup and the coast state machine.