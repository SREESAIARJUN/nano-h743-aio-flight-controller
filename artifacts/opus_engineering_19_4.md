# Gimbal Visual-Follow Controller: PID + Velocity Feedforward + Acceleration Limiting (Cycle 19)

## 1. System Overview

```
 Camera ──► Detector/Tracker ──► Pixel error ──► Angular error ──┐
                                                                 │
              Target-rate estimator (α-β / KF) ──► ω_ff ─────────┤
                                                                 ▼
                       ┌────────── Outer loop (per axis: pan, tilt) ─────────┐
                       │  ω_cmd = ω_ff + PID(e_θ)                            │
                       │  → deadband/soft-zone → rate clamp → accel clamp    │
                       └─────────────────────────────┬───────────────────────┘
                                                     ▼
                              Gimbal inner rate loop (motor/IMU, ~1 kHz)
```

The outer loop runs at the vision rate (30–60 Hz) and outputs **angular rate commands**. The inner stabilization loop is assumed to be fast (bandwidth ≥ 5–10× outer).

---

## 2. Measurement Model

### 2.1 Pixel → Angular Error
With image center $(c_x, c_y)$, focal lengths $(f_x, f_y)$ in pixels, and target centroid $(u, v)$:

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\phi = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

($\psi$ = pan/yaw, $\phi$ = tilt/pitch; sign chosen so positive error → positive rate.) Under zoom, $f_x, f_y$ must be updated from the current zoom level so loop gain stays constant.

### 2.2 Latency Compensation
Images arrive with latency $\tau_d$ (capture + inference). The gimbal has moved since capture:

$$
e_\theta^{\text{comp}}(k) = e_\theta(k) - \left[\theta_g(k) - \theta_g(k - \tau_d)\right]
$$

where $\theta_g$ is the gimbal encoder/IMU angle, buffered and interpolated at the image timestamp. This removes the largest source of overshoot in visual servoing.

### 2.3 Target Angular Position in World Frame
$$
\theta_t(k) = \theta_g(k-\tau_d) + e_\theta(k)
$$
Expressing target bearing in an inertial frame (gimbal angle + error) decouples target motion from gimbal motion, which is what makes feedforward valid.

---

## 3. Target Velocity Estimation (Feedforward Source)

Use an α-β filter (or a 2-state constant-velocity Kalman filter) on $\theta_t$:

$$
\begin{aligned}
\hat\theta^-_k &= \hat\theta_{k-1} + T\,\hat\omega_{k-1} \\
r_k &= \theta_t(k) - \hat\theta^-_k \\
\hat\theta_k &= \hat\theta^-_k + \alpha\, r_k \\
\hat\omega_k &= \hat\omega_{k-1} + \frac{\beta}{T}\, r_k
\end{aligned}
$$

Typical: $\alpha \approx 0.3\!-\!0.5$, $\beta \approx 0.05\!-\!0.15$ (critically damped choice: $\beta = \alpha^2/(2-\alpha)$).

Predict forward to cover latency:
$$
\hat\theta_t^{\text{pred}} = \hat\theta_k + \hat\omega_k \,\tau_d
$$

---

## 4. Control Law

### 4.1 Composite Rate Command (per axis)

$$
\boxed{
\omega_{\text{raw}}(k) = \underbrace{K_{ff}\,\hat\omega_k}_{\text{feedforward}}
+ K_p\, e(k) + K_i \sum_{j\le k} e(j)\,T - K_d\, \dot{\theta}^{f}_{m}(k)
}
$$

- $e(k)$: latency-compensated angular error (Sec. 2.2), after soft deadband.
- $K_{ff} \in [0.8, 1.0]$: feedforward gain (start at 0.9; 1.0 only with a well-filtered $\hat\omega$).
- **Derivative on measurement**, not error, to avoid kicks on target re-acquisition:
$$
\dot\theta^f_m(k) = \lambda\,\dot\theta^f_m(k-1) + (1-\lambda)\frac{-e(k) + e(k-1)}{T}\Big|_{\text{measurement part}}
$$
In practice, since the setpoint is "zero error" and target motion is handled by FF, filtered $\Delta e/T$ with a first-order low-pass ($\lambda = e^{-T/\tau_f}$, $\tau_f \approx 2$–$4T$) works well.

### 4.2 Soft Deadband (Smoothness Near Center)
Prevents hunting from detector jitter without a hard discontinuity:

$$
e_s = \begin{cases}
0, & |e| \le d_0 \\
\operatorname{sign}(e)\,\dfrac{(|e|-d_0)^2}{2(d_1-d_0)}, & d_0 < |e| \le d_1 \\
\operatorname{sign}(e)\left(|e| - \dfrac{d_0+d_1}{2}\right), & |e| > d_1
\end{cases}
$$

This is $C^1$-continuous. Typical: $d_0 \approx 0.3°$, $d_1 \approx 1.5°$ (scale with FOV).

### 4.3 Rate Saturation
$$
\omega_{\text{sat}} = \operatorname{clip}(\omega_{\text{raw}}, -\omega_{\max}, \omega_{\max})
$$

### 4.4 Acceleration Limiting (Slew Limiter)
$$
\omega_{\text{cmd}}(k) = \omega_{\text{cmd}}(k-1) + \operatorname{clip}\!\big(\omega_{\text{sat}} - \omega_{\text{cmd}}(k-1),\; -a_{\max}T,\; a_{\max}T\big)
$$

**Optional jerk limiting** (cinematic smoothness): track an acceleration state $a(k)$ and clip $\Delta a$ to $\pm j_{\max}T$, giving S-curve rate profiles.

### 4.5 Anti-Windup (Back-Calculation)
Because both rate and accel limits saturate the output, the integrator must know:

$$
I(k) = I(k-1) + T\Big[K_i\, e(k) + K_{aw}\big(\omega_{\text{cmd}}(k) - \omega_{\text{raw}}(k)\big)\Big]
$$

with $K_{aw} \approx 1/\sqrt{T_i T_d}$ or simply $K_{aw} \approx K_p/T_i$. Also clamp $|I| \le I_{\max}$, and **freeze** integration when the target is lost.

---

## 5. Target-State Handling

| State | Condition | Behavior |
|---|---|---|
| TRACKING | confidence ≥ c_hi | Full law above |
| COASTING | lost < t_coast (≈0.5–1 s) | $\omega_{\text{cmd}} \to K_{ff}\hat\omega$ decayed by $e^{-t/\tau_c}$; I frozen |
| SEARCH/HOLD | lost > t_coast | Ramp to zero rate under accel limit; reset I, filter |
| REACQUIRE | detection returns | Re-init α-β with $\hat\omega=0$ or last value; bumpless transfer: $I \leftarrow \omega_{\text{cmd}} - K_{ff}\hat\omega - K_p e$ |

Bumpless transfer prevents a jolt when tracking resumes.

---

## 6. Reference Implementation (Python)

```python
import math
from dataclasses import dataclass, field

@dataclass
class AxisParams:
    kp: float = 2.5          # 1/s
    ki: float = 0.4          # 1/s^2
    kd: float = 0.08         # s
    kff: float = 0.9
    kaw: float = 1.5
    w_max: float = math.radians(90)    # rad/s
    a_max: float = math.radians(180)   # rad/s^2
    j_max: float = math.radians(1200)  # rad/s^3 (None to disable)
    d0: float = math.radians(0.3)
    d1: float = math.radians(1.5)
    i_max: float = math.radians(20)
    tau_f: float = 0.06      # derivative filter (s)
    alpha: float = 0.4
    beta: float = 0.1

def soft_deadband(e, d0, d1):
    a = abs(e)
    if a <= d0: return 0.0
    s = math.copysign(1.0, e)
    if a <= d1: return s * (a - d0)**2 / (2*(d1 - d0))
    return s * (a - 0.5*(d0 + d1))

def clip(x, lo, hi): return max(lo, min(hi, x))

@dataclass
class AxisController:
    p: AxisParams
    I: float = 0.0
    e_prev: float = 0.0
    d_f: float = 0.0
    th_hat: float = None
    w_hat: float = 0.0
    w_cmd: float = 0.0
    a_cmd: float = 0.0

    def update_estimator(self, theta_t, T):
        if self.th_hat is None:
            self.th_hat, self.w_hat = theta_t, 0.0
            return
        pred = self.th_hat + T*self.w_hat
        r = theta_t - pred
        self.th_hat = pred + self.p.alpha*r
        self.w_hat += (self.p.beta/T)*r

    def step(self, e_comp, theta_t, T, tracking=True):
        p = self.p
        if tracking:
            self.update_estimator(theta_t, T)
            e = soft_deadband(e_comp, p.d0, p.d1)
            lam = math.exp(-T/p.tau_f)
            self.d_f = lam*self.d_f + (1-lam)*(e - self.e_prev)/T
            self.e_prev = e
            w_raw = p.kff*self.w_hat + p.kp*e + self.I + p.kd*self.d_f
        else:
            e = 0.0
            self.w_hat *= 0.9                # coast decay
            w_raw = p.kff*self.w_hat

        # rate limit
        w_sat = clip(w_raw, -p.w_max, p.w_max)
        # accel (+ optional jerk) limit
        a_des = (w_sat - self.w_cmd)/T
        if p.j_max:
            a_des = clip(a_des, self.a_cmd - p.j_max*T, self.a_cmd + p.j_max*T)
        self.a_cmd = clip(a_des, -p.a_max, p.a_max)
        self.w_cmd += self.a_cmd*T

        # integrator with back-calculation anti-windup
        if tracking:
            self.I += T*(p.ki*e + p.kaw*(self.w_cmd - w_raw))
            self.I = clip(self.I, -p.i_max, p.i_max)
        return self.w_cmd

class GimbalTracker:
    def __init__(self, fx, fy, cx, cy):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.pan  = AxisController(AxisParams())
        self.tilt = AxisController(AxisParams(w_max=math.radians(60)))

    def step(self, det, gimbal_now, gimbal_at_capture, T):
        """det: (u,v) or None; gimbal_*: (pan, tilt) radians."""
        if det is None:
            return (self.pan.step(0,0,T,False), self.tilt.step(0,0,T,False))
        u, v = det
        e_pan  =  math.atan((u - self.cx)/self.fx)
        e_tilt = -math.atan((v - self.cy)/self.fy)
        # latency compensation
        dp = gimbal_now[0] - gimbal_at_capture[0]
        dt = gimbal_now[1] - gimbal_at_capture[1]
        th_pan  = gimbal_at_capture[0] + e_pan
        th_tilt = gimbal_at_capture[1] + e_tilt
        return (self.pan.step(e_pan - dp,  th_pan,  T),
                self.tilt.step(e_tilt - dt, th_tilt, T))
```

---

## 7. Stability and Tuning

**Loop model** (rate-commanded gimbal ≈ integrator with delay):
$$
G(s) = \frac{e^{-s\tau_d}}{s(\tau_g s + 1)}
$$
Crossover from $K_p$: $\omega_c \approx K_p$. Phase margin requirement:
$$
\text{PM} \approx 90° - \arctan(\omega_c\tau_g) - \omega_c\tau_d\cdot\tfrac{180°}{\pi} \ge 45°
$$
→ For $\tau_d = 80$ ms, $\tau_g = 20$ ms: $K_p \lesssim 3$ s⁻¹.

**Tuning order:**
1. $K_{ff}=K_i=K_d=0$. Raise $K_p$ until slight overshoot on a step, then back off ~30%.
2. Enable FF; tune α/β on a constant-speed target until steady error ≈ 0 with low jitter in $\hat\omega$. Raise $K_{ff}$ toward 1.0.
3. Add $K_i \approx K_p^2/10$ to remove residual bias (e.g. mount drift).
4. Add small $K_d$ only if overshoot persists; increase $\tau_f$ if output gets noisy.
5. Set $a_{\max}$ from the desired feel: smooth cinematic ~90–180°/s², aggressive sports ~400–720°/s². Ensure $a_{\max}$ ≥ expected target angular acceleration, otherwise tracking lag grows.
6. Set deadband $d_0$ ≈ 2–3σ of detection jitter (in angle).

**Performance metrics to log each cycle:** RMS angular error, peak error during maneuvers, rate-command jerk (smoothness), saturation duty cycle, time-to-recenter after reacquisition.

---

## 8. Cycle 19 Refinements Summary
- Feedforward from an **inertial-frame** target-rate estimate, not raw pixel velocity (avoids positive feedback from gimbal motion).
- **Latency compensation** using timestamped gimbal angle history.
- **C¹ soft deadband** instead of a hard one.
- **Rate → accel → (jerk)** limiting cascade with **back-calculation anti-windup** that sees all limits.
- **Bumpless transfer** and explicit coast/search states for occlusions.

If you share your gimbal's inner-loop bandwidth, camera latency, and target type, I can compute specific gain values and limits.