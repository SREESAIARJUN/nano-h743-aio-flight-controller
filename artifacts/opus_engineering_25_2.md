# Autonomous Gimbal Tracking Controller: PID + Velocity Feedforward + Acceleration Limiting (Cycle 25)

## 1. System Overview

```
 Camera ──► Detector ──► Pixel→Angle ──► Target Estimator ──► Reference Shaper ──► PID+FF ──► Accel/Rate Limiter ──► Gimbal Motor
   ▲          (bbox)      (+ gimbal pose)   (KF: θ, ω, α)       (smooth follow)      (per axis)                         │
   └──────────────────────────────────── encoder / IMU feedback ◄───────────────────────────────────────────────────────┘
```

Each axis (yaw ψ, pitch θ) is controlled independently with an identical structure. The notation below uses a generic axis angle **q**.

---

## 2. Measurement Model: Pixel Error to Angular Target

Given a detection centroid (u, v), principal point (cₓ, c_y), and focal lengths (fₓ, f_y) in pixels:

$$
e_{u}^{\text{ang}} = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_{v}^{\text{ang}} = \arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

**Latency compensation.** The image was captured at time t − τ, where τ is the exposure, inference, and transport delay. Reconstruct the absolute target angle using the gimbal pose *at capture time*, taken from a buffered encoder history:

$$
q^{\text{tgt}}_{\text{meas}}(t-\tau) = q_{\text{gimbal}}(t-\tau) + e^{\text{ang}}
$$

Doing this decouples target motion from the gimbal's own motion. It is the single biggest fix for oscillation in visual servoing.

---

## 3. Target State Estimator (Constant-Velocity Kalman Filter)

The state is $x = [q_t,\ \omega_t]^T$, with sample period Δt.

$$
F=\begin{bmatrix}1&\Delta t\\0&1\end{bmatrix},\quad
Q=\sigma_a^2\begin{bmatrix}\Delta t^4/4&\Delta t^3/2\\ \Delta t^3/2&\Delta t^2\end{bmatrix},\quad
H=[1\ \ 0],\quad R=\sigma_{\text{pix}}^2
$$

**Prediction to the present.** After the update at t − τ, propagate the state forward by τ:

$$
\hat q_t(t) = \hat q_t(t-\tau) + \hat\omega_t\,\tau
$$

**Measurement handling:**
- **Gating:** reject the measurement if the Mahalanobis distance d² > 9.21 (χ², 2 DOF, 99%).
- **Target loss:** coast on prediction for T_coast ≈ 0.5 s, decaying ω̂ₜ toward 0. After that, hold position or hand off to a search behavior.

---

## 4. Reference Shaper (Smooth Follow)

The raw estimate is shaped before it drives the loop. This produces a cinematic, non-twitchy follow.

**4a. Deadband with smooth entry.** This prevents micro-hunting when the target is near center:

$$
e_{db} = \operatorname{sign}(e)\cdot\max(0,\ |e| - \delta)\cdot\left(1 - e^{-|e|/\delta}\right)
$$

**4b. Second-order reference filter.** Use ζ = 1 (critically damped) and bandwidth ω_n:

$$
\ddot q_r = \omega_n^2\,(\hat q_t - q_r) - 2\zeta\omega_n\,(\dot q_r - \hat\omega_t)
$$

Including ω̂ₜ in the damping term means the reference matches the target's velocity, not merely its position. Without it, the reference produces steady lag on moving targets.

The filter outputs are:
- **q_r**: reference angle
- **q̇_r**: reference velocity, used for feedforward
- **q̈_r**: reference acceleration, used for optional acceleration feedforward

---

## 5. Control Law: PID + Feedforward

Define the tracking error $e = q_r - q$.

$$
\boxed{\;\omega_{\text{cmd}}^{\text{raw}} = \underbrace{K_{ff}\,\dot q_r}_{\text{velocity FF}} + K_p\,e + K_i\!\int e\,dt + K_d\,\dot e_f \;+\; K_{a}\,\ddot q_r\;}
$$

The output is a **rate command** to the inner gimbal rate loop, which is typically the motor driver or brushless gimbal controller.

**Component notes:**
- **Feedforward K_ff ≈ 1.0.** This term carries most of the tracking. The PID only corrects residuals, so the gains can stay low and the motion smooth.
- **Derivative on measurement.** Compute $\dot e_f = \dot q_r - \dot q_{\text{meas},f}$ using the reference velocity and the filtered gyro or encoder rate. Do not differentiate the error directly, since that amplifies detector noise.
- **Derivative filter:** first-order low-pass with N ≈ 10–20:
  $$D_k = \frac{T_f}{T_f+\Delta t}D_{k-1} + \frac{K_d}{T_f+\Delta t}(\dot e_k\,\Delta t), \quad T_f = K_d/(K_pN)$$
- **Acceleration FF K_a:** use only on high-inertia or payload-heavy gimbals. Otherwise set it to 0.

---

## 6. Acceleration and Rate Limiting

Apply the limits in this order:

1. **Rate limit:**
   $$\omega_{\text{sat}} = \operatorname{clip}(\omega_{\text{cmd}}^{\text{raw}},\ -\omega_{\max},\ \omega_{\max})$$

2. **Acceleration limit (slew):**
   $$\omega_k = \omega_{k-1} + \operatorname{clip}(\omega_{\text{sat}} - \omega_{k-1},\ -a_{\max}\Delta t,\ a_{\max}\Delta t)$$

3. **Optional jerk limit**, for smoother cinematic motion:
   $$a_k = a_{k-1} + \operatorname{clip}\!\left(\tfrac{\omega_{\text{sat}}-\omega_{k-1}}{\Delta t} - a_{k-1},\ -j_{\max}\Delta t,\ j_{\max}\Delta t\right),\quad \omega_k = \omega_{k-1} + a_k\Delta t$$

4. **Mechanical soft limits.** Near the joint limits, taper the allowed velocity toward the limit so the gimbal stops exactly at q_max:
   $$\omega_{\max}^{\text{eff}} = \min\!\left(\omega_{\max},\ \sqrt{2a_{\max}(q_{\max}-q)}\right)$$

---

## 7. Anti-Windup (Back-Calculation)

The limiters make the plant input differ from the controller output, so the integrator must be corrected:

$$
I_k = I_{k-1} + \left[K_i\,e_k + \frac{1}{T_t}\left(\omega_k - \omega_{\text{cmd}}^{\text{raw}}\right)\right]\Delta t, \qquad T_t \approx \sqrt{T_i T_d}\ \text{(or } 0.5\,T_i\text{)}
$$

Additionally:
- Freeze the integrator while the target is lost.
- Bleed the integrator (I ← 0.98·I) during coasting.

---

## 8. Reference Implementation (Python, per axis)

```python
import math
from dataclasses import dataclass

@dataclass
class AxisParams:
    kp: float = 4.0; ki: float = 0.8; kd: float = 0.15
    kff: float = 1.0; ka: float = 0.0
    d_filt_N: float = 15.0
    w_max: float = math.radians(180)      # rad/s
    a_max: float = math.radians(600)      # rad/s^2
    j_max: float = math.radians(6000)     # rad/s^3
    deadband: float = math.radians(0.3)
    wn: float = 6.0; zeta: float = 1.0    # reference shaper
    q_min: float = -math.pi; q_max: float = math.pi
    Tt: float = 0.3

class GimbalAxisController:
    def __init__(self, p: AxisParams):
        self.p = p
        self.qr = self.qr_d = 0.0
        self.I = self.D = 0.0
        self.w_prev = self.a_prev = 0.0

    def _deadband(self, e):
        d = self.p.deadband
        return math.copysign(max(0.0, abs(e) - d) * (1 - math.exp(-abs(e) / d)), e)

    def update(self, q_tgt, w_tgt, q, q_dot, dt, target_valid=True):
        p = self.p
        # --- Reference shaper (2nd order, velocity-matched) ---
        err_t = self._deadband(q_tgt - self.qr)
        qr_dd = p.wn**2 * err_t - 2 * p.zeta * p.wn * (self.qr_d - w_tgt)
        self.qr_d += qr_dd * dt
        self.qr   += self.qr_d * dt

        # --- PID + FF ---
        e = self.qr - q
        e_dot = self.qr_d - q_dot
        Tf = p.kd / (p.kp * p.d_filt_N) if p.kd > 0 else 0.0
        self.D = (Tf * self.D + p.kd * e_dot * dt) / (Tf + dt) if Tf > 0 else 0.0

        u_raw = p.kff * self.qr_d + p.kp * e + self.I + self.D + p.ka * qr_dd

        # --- Rate limit with soft joint limits ---
        w_hi = min(p.w_max, math.sqrt(max(0.0, 2 * p.a_max * (p.q_max - q))))
        w_lo = -min(p.w_max, math.sqrt(max(0.0, 2 * p.a_max * (q - p.q_min))))
        w_sat = min(max(u_raw, w_lo), w_hi)

        # --- Jerk + acceleration limit ---
        a_des = (w_sat - self.w_prev) / dt
        a = self.a_prev + max(-p.j_max*dt, min(p.j_max*dt, a_des - self.a_prev))
        a = max(-p.a_max, min(p.a_max, a))
        w = self.w_prev + a * dt
        self.w_prev, self.a_prev = w, a

        # --- Integrator with back-calculation anti-windup ---
        if target_valid:
            self.I += (p.ki * e + (w - u_raw) / p.Tt) * dt
        else:
            self.I *= 0.98
        return w   # rate command to inner loop
```

---

## 9. Tuning Procedure

| Step | Action | Target |
|---|---|---|
| 1 | Tune the inner rate loop (motor driver) first | Bandwidth ≥ 5× outer loop |
| 2 | Set Kp = Ki = Kd = 0, K_ff = 1, move the target at constant speed | Lag should be small; adjust K_ff to between 0.9 and 1.1 |
| 3 | Raise Kp until slight overshoot, then back off 30% | Crossover ≈ ¼ of the vision rate (e.g., 30 Hz → ~1.2 Hz) |
| 4 | Add Ki ≈ Kp/5 | Removes steady offset from bias or wind |
| 5 | Add small Kd only if overshoot persists | Watch for noise chatter |
| 6 | Set a_max and j_max from the payload or framing aesthetic | Cinematic: low j_max. Sports: high a_max. |
| 7 | Set ω_n of the shaper | 2–4 rad/s smooth, 6–10 rad/s aggressive |

**Stability check:** the phase margin must hold with the loop delay τ included:

$$
\text{PM} \approx \text{PM}_0 - \omega_c\,\tau\cdot\frac{180}{\pi} \ \ge 45^\circ
$$

---

## 10. Cycle 25 Refinements Checklist

- ☐ Pose-at-capture latency compensation (timestamp-synchronized encoder buffer)
- ☐ Feedforward uses the **shaped reference velocity**, not the raw KF velocity, which avoids noise injection
- ☐ Jerk-limited slew with joint-limit-aware braking
- ☐ Back-calculation anti-windup plus integrator bleed on target loss
- ☐ Velocity-matched reference shaper, giving zero steady lag on ramps
- ☐ Chi-square gating for detector outliers and ID switches
- ☐ Adaptive option: scale ω_n and Kp by detection confidence or target size (closer, larger targets warrant lower gain)