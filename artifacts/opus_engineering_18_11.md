# Autonomous Gimbal Tracking Controller: PID with Velocity Feedforward and Acceleration Limiting (Cycle 18)

I don't have the earlier cycles, so this is a self-contained formulation. Each section maps to a separable module, so you can swap it in against your current baseline.

---

## 1. System Overview

```
 Camera ──► Detector ──► Pixel→Angle ──► Target State ──► PID + FF ──► Accel/Jerk ──► Gimbal
 (frame)    (bbox)       (latency comp)   Estimator (KF)   (per axis)   Limiter         Rate Loop
                                ▲                                                         │
                                └──────────── gimbal encoder angles + IMU rates ◄─────────┘
```

The design has three loops:
- **Inner loop (gimbal ESC/FOC):** rate control at roughly 500 Hz to 2 kHz. This is assumed to exist already.
- **Outer loop (this design):** produces an angular-rate command at the vision rate (30 to 120 Hz), interpolated up to the control rate (200 to 500 Hz).
- **Estimator:** fuses delayed vision measurements with high-rate gimbal and IMU data.

The pan (yaw, ψ) and tilt (pitch, θ) axes are controlled independently. Roll is held level by the stabilizer.

---

## 2. Measurement Model: Pixel Error to Angular Error

Let the target centroid be $(u, v)$, with intrinsics $f_x, f_y, c_x, c_y$:

$$
e_\psi^{cam} = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta^{cam} = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

For lenses with significant distortion, undistort $(u,v)$ first.

**Line-of-sight (LOS) angle in the stabilized/world frame**, using the gimbal angle *at image capture time* $t_c = t_{now} - \tau_{lat}$:

$$
\lambda_\psi(t_c) = \psi_g(t_c) + e_\psi^{cam}, \qquad \lambda_\theta(t_c) = \theta_g(t_c) + e_\theta^{cam}
$$

Gimbal angles are stored in a ring buffer and interpolated at $t_c$. This is the main latency fix. Without it, the controller "chases" errors that the gimbal has already corrected.

---

## 3. Target LOS Estimator (Constant-Velocity Kalman Filter, per axis)

State $x = [\lambda,\ \dot\lambda]^T$:

$$
x_{k+1} = \begin{bmatrix}1 & \Delta t\\ 0 & 1\end{bmatrix} x_k + w_k,\quad
Q = q\begin{bmatrix}\Delta t^3/3 & \Delta t^2/2\\ \Delta t^2/2 & \Delta t\end{bmatrix},\quad
z_k = \lambda_{meas} + v_k,\ R = \sigma_{px}^2/f^2
$$

- **Delayed-measurement handling:** predict to $t_c$, update, then re-predict to $t_{now}$. For low latency, the simple approach is to update with the delayed measurement and forward-predict by $\tau_{lat}$.
- **Innovation gating:** reject a measurement if $\nu^2/S > \chi^2_{1,0.99} \approx 6.63$. This catches detector glitches and ID switches.
- **Adaptive $q$:** increase $q$ when the normalized innovation squared stays high for N frames, which indicates the target is maneuvering.

Outputs at control time: $\hat\lambda(t)$ and $\hat{\dot\lambda}(t)$.

---

## 4. Control Law (per axis)

**Tracking error**, computed against the estimated LOS predicted to now rather than the raw stale pixel error:

$$
e(t) = \hat\lambda(t) - \phi_g(t)
$$

where $\phi_g$ is the current gimbal angle on that axis.

**Velocity feedforward plus PID:**

$$
\boxed{\ \omega_{des} = \underbrace{K_{ff}\,\hat{\dot\lambda}}_{\text{feedforward}} \;+\; K_p\,\mathcal{S}(e) \;+\; K_i\!\int\! e\,dt \;+\; K_d\,\dot e_f\ }
$$

Each term has a specific role:

| Term | Role | Notes |
|---|---|---|
| $K_{ff}\hat{\dot\lambda}$ | Matches target angular velocity, giving near-zero lag on constant-rate motion | $K_{ff}\in[0.8, 1.0]$. Fade it in with confidence: $K_{ff}\cdot c$, where $c = $ track confidence. |
| $K_p\,\mathcal S(e)$ | Re-centers the target | $\mathcal S$ is the smooth-shaping function below |
| $K_i\int e$ | Removes steady bias (mount misalignment, friction) | Small; anti-windup is required |
| $K_d\dot e_f$ | Damping | $\dot e_f$ from a 1st-order filtered derivative; use $-\dot\phi_g$ (gimbal IMU rate) to avoid differentiating noisy vision |

If the gimbal is body-mounted without world stabilization, add base-rate compensation: $\omega_{des} \mathrel{-}= \omega_{body}$ (projected onto the axis).

### 4.1 Smooth Error Shaping ("cinematic follow")

A soft deadzone plus saturation prevents micro-jitter near center and harsh slews on large errors:

$$
\mathcal S(e) = \operatorname{sign}(e)\cdot e_{sat}\tanh\!\left(\frac{\max(|e| - e_{dz},\,0)}{e_{sat}}\right)
$$

- $e_{dz}$: deadzone, typically 0.3° to 1° (the "framing tolerance").
- $e_{sat}$: soft-saturation scale, typically 10° to 20°.

The function is continuous with a continuous derivative outside the deadzone, so it introduces no chatter at the boundary as long as the deadzone is smaller than the noise-free tracking accuracy.

### 4.2 Derivative Filter

$$
\dot e_f[k] = \alpha\,\dot e_f[k-1] + (1-\alpha)\frac{e[k]-e[k-1]}{\Delta t},\qquad \alpha = \frac{\tau_d}{\tau_d+\Delta t}
$$

---

## 5. Acceleration and Jerk Limiting (Command Shaping)

The raw $\omega_{des}$ is passed through a jerk-limited acceleration limiter to produce the actual command $\omega_{cmd}$.

**Acceleration-limited (minimum version):**

$$
\omega_{cmd}[k] = \omega_{cmd}[k-1] + \operatorname{clamp}\!\big(\omega_{des}[k]-\omega_{cmd}[k-1],\ -a_{max}\Delta t,\ a_{max}\Delta t\big)
$$

**Jerk-limited (recommended for smooth video):**

$$
a_{des} = \operatorname{clamp}\!\left(\frac{\omega_{des}-\omega_{cmd}}{T_a},\ -a_{max},\ a_{max}\right),\quad
a[k] = a[k-1] + \operatorname{clamp}(a_{des}-a[k-1],\ -j_{max}\Delta t,\ j_{max}\Delta t)
$$
$$
\omega_{cmd}[k] = \operatorname{clamp}(\omega_{cmd}[k-1] + a[k]\Delta t,\ -\omega_{max},\ \omega_{max})
$$

To avoid overshoot from jerk limiting, allow deceleration to begin early. If $|\omega_{des}-\omega_{cmd}| < a^2/(2j_{max})$, drive $a_{des}\to 0$.

**Mechanical limits:** if the gimbal approaches an angle stop, cap the speed so it can still stop in time: $|\omega| \le \sqrt{2 a_{max}\,d_{stop}}$.

### 5.1 Anti-Windup (Back-Calculation)

When the limiter clips the command, bleed the integrator:

$$
I[k] = I[k-1] + \Delta t\left(K_i e + K_{aw}(\omega_{cmd} - \omega_{des})\right),\qquad K_{aw}\approx 1/T_i
$$

Also freeze integration while $|e| < e_{dz}$ and while the target is lost.

---

## 6. Track-State Behaviour Machine

| State | Condition | Behaviour |
|---|---|---|
| **ACQUIRE** | new track, confidence < c₁ | $K_{ff}=0$, reduced $K_p$, gentle $a_{max}$ |
| **TRACK** | confidence ≥ c₁, gated updates OK | Full law |
| **COAST** | no valid measurement for ≤ $T_{coast}$ (e.g. 1 s) | KF prediction only; $K_{ff}$ decays as $e^{-t/\tau_c}$; no integration |
| **LOST** | > $T_{coast}$ | $\omega_{des}\to 0$ through the limiter (smooth stop); hold or return-to-home |

Transitions go through the same limiter, so no state change can produce a step in rate.

---

## 7. Stability and Tuning Guidance

The effective plant from rate command to angle is approximately $\frac{1}{s}e^{-s\tau}$, where $\tau$ is the total loop delay (vision latency plus inner-loop lag).

- **Proportional gain:** for ≥ 45° phase margin, keep the crossover $\omega_c \approx K_p \le \frac{1}{2\tau}$ to $\frac{1}{3\tau}$. With τ = 80 ms, $K_p \lesssim 4$ to 6 s⁻¹.
- **Integral:** $K_i \approx K_p^2/10$ for a corner well below crossover.
- **Derivative:** start at 0. Add $K_d \approx 0.05$ to $0.15$ only if the inner loop is underdamped.
- **Feedforward:** tune on a constant-rate target (turntable or walking subject). Increase $K_{ff}$ until the steady lag approaches 0, then back off 5 to 10% if noise is visible.
- **Limiter:** $a_{max}$ sets the "feel." Use 100 to 300 °/s² for cinematic motion and 600 to 1500 °/s² for sport or fast targets. Set $j_{max} \approx 5$ to $10\times a_{max}$ s⁻¹.

**Starting parameter set (30 fps camera, ~80 ms latency):**

| Param | Pan | Tilt |
|---|---|---|
| $K_p$ (1/s) | 3.5 | 3.0 |
| $K_i$ (1/s²) | 0.8 | 0.6 |
| $K_d$ | 0.05 | 0.05 |
| $K_{ff}$ | 0.9 | 0.85 |
| $e_{dz}$ / $e_{sat}$ (°) | 0.5 / 15 | 0.5 / 12 |
| $\omega_{max}$ (°/s) | 180 | 120 |
| $a_{max}$ (°/s²) | 400 | 300 |
| $j_{max}$ (°/s³) | 3000 | 2500 |
| KF $q$ / $\sigma_{px}$ | 50 / 3 px | 30 / 3 px |

---

## 8. Reference Implementation (Python, per axis)

```python
import math
from collections import deque

def clamp(x, lo, hi): return max(lo, min(hi, x))

class LOSKalman:
    def __init__(self, q, r):
        self.x = [0.0, 0.0]; self.P = [[1e2, 0], [0, 1e2]]
        self.q, self.r, self.init = q, r, False
    def predict(self, dt):
        l, ld = self.x; P = self.P; q = self.q
        self.x = [l + ld*dt, ld]
        p00 = P[0][0] + dt*(P[1][0]+P[0][1]) + dt*dt*P[1][1] + q*dt**3/3
        p01 = P[0][1] + dt*P[1][1] + q*dt**2/2
        p11 = P[1][1] + q*dt
        self.P = [[p00, p01], [p01, p11]]
    def update(self, z, gate=6.63):
        if not self.init:
            self.x = [z, 0.0]; self.init = True; return True
        nu = z - self.x[0]; S = self.P[0][0] + self.r
        if nu*nu/S > gate: return False            # reject outlier
        K0, K1 = self.P[0][0]/S, self.P[1][0]/S
        self.x = [self.x[0] + K0*nu, self.x[1] + K1*nu]
        p00, p01, p11 = self.P[0][0], self.P[0][1], self.P[1][1]
        self.P = [[(1-K0)*p00, (1-K0)*p01], [p01 - K1*p00, p11 - K1*p01]]
        return True

class GimbalAxisTracker:
    def __init__(s, kp, ki, kd, kff, e_dz, e_sat, w_max, a_max, j_max,
                 q, r, tau_d=0.03, coast_T=1.0):
        s.kp, s.ki, s.kd, s.kff = kp, ki, kd, kff
        s.e_dz, s.e_sat = e_dz, e_sat
        s.w_max, s.a_max, s.j_max = w_max, a_max, j_max
        s.kf = LOSKalman(q, r); s.tau_d = tau_d; s.coast_T = coast_T
        s.I = 0.0; s.de_f = 0.0; s.e_prev = None
        s.w_cmd = 0.0; s.a = 0.0; s.t_since_meas = 1e9
        s.angle_hist = deque(maxlen=200)            # (t, gimbal_angle)

    def log_gimbal(self, t, ang): self.angle_hist.append((t, ang))

    def _angle_at(self, t):
        h = self.angle_hist
        if not h: return 0.0
        for (t0, a0), (t1, a1) in zip(h, list(h)[1:]):
            if t0 <= t <= t1:
                return a0 + (a1-a0)*(t-t0)/max(t1-t0, 1e-6)
        return h[-1][1]

    def vision_update(self, t_capture, cam_err):
        """cam_err: angular error from pixel offset at capture time."""
        los = self._angle_at(t_capture) + cam_err
        if self.kf.update(los): self.t_since_meas = 0.0

    def shape(self, e):
        m = max(abs(e) - self.e_dz, 0.0)
        return math.copysign(self.e_sat*math.tanh(m/self.e_sat), e)

    def step(self, dt, gimbal_angle, confidence=1.0):
        self.kf.predict(dt); self.t_since_meas += dt
        tracking = self.kf.init and self.t_since_meas < self.coast_T
        if not tracking:
            w_des = 0.0                                    # L