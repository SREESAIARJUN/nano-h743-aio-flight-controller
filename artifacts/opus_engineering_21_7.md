# Autonomous Gimbal Tracking Controller (Cycle 21)
### PID + Velocity Feedforward + Acceleration/Jerk Limiting for Smooth Visual Follow

---

## 1. System Overview

```
 Camera ──► Detector/Tracker ──► Pixel→Angle ──► LOS Estimator ──► Controller ──► Rate/Accel Limiter ──► Gimbal Rate Loop
   ▲          (u,v,t_img)          (e_ψ, e_θ)     (λ̂, λ̂̇)        PID + FF        (smooth ω_cmd)          (motor FOC)
   └───────────────────────────────── gimbal encoders / IMU (ψ_g, θ_g, ω_g) ◄─────────────────────────────┘
```

The vision controller is an **outer angle loop** that outputs **angular rate commands** to the gimbal's internal (fast, stabilized) rate loop. Yaw (pan, ψ) and pitch (tilt, θ) are controlled independently with identical structure.

---

## 2. Measurement Model

### 2.1 Pixel error → angular error
With intrinsics (f_x, f_y, c_x, c_y) and target centroid (u, v):

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

(Use the undistorted centroid; sign of pitch depends on mount convention.)

### 2.2 Line-of-sight (LOS) angle in the stabilized frame
The error alone mixes target motion and gimbal motion. Reconstruct the target's absolute bearing using the gimbal angle **at image capture time** t_img:

$$
\lambda_k = \psi_g(t_{img}) + e_{\psi,k}
$$

This decoupling is what makes feedforward valid: λ̇ is the target's true angular velocity, independent of what the gimbal is doing.

### 2.3 Latency compensation
Total pipeline delay τ = exposure/2 + transfer + inference + comms. Predict forward:

$$
\lambda_{pred} = \hat\lambda + \hat{\dot\lambda}\,\tau, \qquad
e_{pred} = \lambda_{pred} - \psi_g(t_{now})
$$

---

## 3. LOS State Estimator (α-β-γ / constant-acceleration Kalman)

State x = [λ, λ̇, λ̈]ᵀ, updated at the vision rate (asynchronous, variable Δt):

**Predict**
$$
\hat\lambda^- = \hat\lambda + \hat{\dot\lambda}\Delta t + \tfrac12\hat{\ddot\lambda}\Delta t^2,\quad
\hat{\dot\lambda}^- = \hat{\dot\lambda} + \hat{\ddot\lambda}\Delta t,\quad
\hat{\ddot\lambda}^- = \hat{\ddot\lambda}
$$

**Correct** (residual r = λ_meas − λ̂⁻)
$$
\hat\lambda = \hat\lambda^- + \alpha r,\quad
\hat{\dot\lambda} = \hat{\dot\lambda}^- + \frac{\beta}{\Delta t} r,\quad
\hat{\ddot\lambda} = \hat{\ddot\lambda}^- + \frac{2\gamma}{\Delta t^2} r
$$

Typical: α = 0.5, β = 0.15, γ = 0.01 (lower = smoother, laggier). A full Kalman filter with Q tuned to target maneuverability is preferred if the detector reports confidence (scale R by 1/confidence).

**Outlier gating:** reject the measurement if |r| > g·σ_r (g ≈ 3–4); treat as a missed frame.

---

## 4. Control Law

Commanded gimbal rate (per axis):

$$
\boxed{\;
\omega_{raw} = \underbrace{K_{ff}\,\hat{\dot\lambda}}_{\text{velocity FF}}
+ \underbrace{K_p\,e}_{\text{P}}
+ \underbrace{K_i \!\int e\,dt}_{\text{I}}
+ \underbrace{K_d\,\dot e_f}_{\text{D (filtered)}}
\;}
$$

### 4.1 Velocity feedforward
- K_ff ∈ [0.8, 1.0]. At K_ff = 1 the gimbal matches target angular velocity, and the PID only corrects residual position error → near-zero steady-state lag on ramp motion.
- Optional acceleration feedforward into the rate-loop torque: τ_ff = J·λ̈̂ (only if the inner loop accepts torque/current FF).

### 4.2 Proportional term with soft deadband
To avoid hunting around center (detector jitter):

$$
e_{db} = \operatorname{sign}(e)\,\max(0,\,|e| - d)\cdot\frac{|e|}{|e|+\epsilon}
$$

d ≈ 1–2 detector σ (e.g., 0.1–0.3°). The smooth form avoids a step at the deadband edge.

### 4.3 Derivative on measurement (filtered)
Use derivative on −λ_g-relative error, low-pass filtered:

$$
\dot e_f[k] = \frac{T_f}{T_f+\Delta t}\dot e_f[k-1] + \frac{1}{T_f+\Delta t}(e[k]-e[k-1])
$$

with T_f ≈ K_d/(N·K_p), N = 5–10. Because FF already handles target velocity, K_d is often small or zero.

### 4.4 Gain scheduling (optional smooth-follow behavior)
Reduce aggressiveness near center, increase on large errors:

$$
K_p(e) = K_{p,0}\left(1 + \kappa\,\tanh\!\frac{|e|}{e_s}\right)
$$

---

## 5. Rate, Acceleration, and Jerk Limiting

Applied every control tick (run at gimbal loop rate, e.g. 200–500 Hz, holding last vision estimate with prediction between frames).

**Rate limit**
$$
\omega_{sat} = \operatorname{clip}(\omega_{raw},\,-\omega_{max},\,\omega_{max})
$$

**Jerk-limited acceleration (S-curve smoothing)**
$$
a_{des} = \frac{\omega_{sat} - \omega_{cmd}[k-1]}{\Delta t}
$$
$$
a_{cmd}[k] = \operatorname{clip}\!\Big(a_{des},\; a_{cmd}[k-1] - j_{max}\Delta t,\; a_{cmd}[k-1] + j_{max}\Delta t\Big)
$$
$$
a_{cmd}[k] = \operatorname{clip}(a_{cmd}[k],\,-a_{max},\,a_{max})
$$
$$
\omega_{cmd}[k] = \omega_{cmd}[k-1] + a_{cmd}[k]\,\Delta t
$$

**Overshoot-safe braking:** when approaching a stop, limit velocity so the gimbal can decelerate in time:
$$
|\omega| \le \sqrt{2\,a_{max}\,|e|}
$$
(apply to the PID portion, not to FF, so tracking of moving targets is unaffected).

---

## 6. Anti-Windup (Back-Calculation)

Since limiting is substantial, the integrator must know the actually-commanded rate:

$$
I[k] = I[k-1] + \Big(K_i\,e[k] + \tfrac{1}{T_t}\big(\omega_{cmd}[k] - \omega_{raw}[k]\big)\Big)\Delta t
$$

T_t ≈ √(T_i·T_d) or ~T_i if D is unused. Also clamp |I| ≤ I_max and freeze integration while the target is lost.

---

## 7. Target Loss & Reacquisition

| State | Condition | Behavior |
|---|---|---|
| **TRACK** | valid detection | full controller |
| **COAST** | missed ≤ T_coast (≈0.5–1 s) | ω = K_ff·λ̇̂·e^{−t/τ_c}, predict λ̂, no I update |
| **SEARCH** | lost > T_coast | decelerate (a_max), optional scan pattern around last λ̂ |
| **REACQUIRE** | new detection | reset estimator to measurement, keep λ̇̂ = 0 or last, blend gains in over ~0.3 s |

Gain blending avoids a jump on reacquisition: K_eff = K·min(1, t/T_blend).

---

## 8. Reference Implementation (Python, single axis)

```python
import math

class AxisTracker:
    def __init__(self, p):
        self.p = p
        # estimator
        self.lam = self.lamd = self.lamdd = 0.0
        self.init = False
        # controller
        self.I = 0.0
        self.e_prev = 0.0
        self.ed_f = 0.0
        self.w_cmd = 0.0
        self.a_cmd = 0.0
        self.t_last_meas = None
        self.blend_t = 0.0

    # --- called when a vision frame arrives ---
    def on_measurement(self, e_meas, gimbal_angle_at_capture, t_img):
        lam_meas = gimbal_angle_at_capture + e_meas
        if not self.init:
            self.lam, self.lamd, self.lamdd = lam_meas, 0.0, 0.0
            self.init, self.t_last_meas, self.blend_t = True, t_img, 0.0
            return
        dt = max(1e-3, t_img - self.t_last_meas)
        # predict
        lam_p  = self.lam + self.lamd*dt + 0.5*self.lamdd*dt*dt
        lamd_p = self.lamd + self.lamdd*dt
        r = lam_meas - lam_p
        if abs(r) > self.p['gate']:          # outlier
            return
        a, b, g = self.p['alpha'], self.p['beta'], self.p['gamma']
        self.lam   = lam_p + a*r
        self.lamd  = lamd_p + (b/dt)*r
        self.lamdd = self.lamdd + (2*g/(dt*dt))*r
        self.t_last_meas = t_img

    # --- called at control rate ---
    def update(self, gimbal_angle_now, t_now, dt):
        p = self.p
        if not self.init:
            return self._limit(0.0, dt, 0.0)

        age = t_now - self.t_last_meas
        lost = age > p['t_coast']
        coasting = age > p['t_miss']

        # predict LOS to now (+ actuation latency)
        h = age + p['tau_act']
        lam_pred  = self.lam + self.lamd*h + 0.5*self.lamdd*h*h
        lamd_pred = self.lamd + self.lamdd*h

        if lost:
            return self._limit(0.0, dt, 0.0)

        e = lam_pred - gimbal_angle_now
        # soft deadband
        d = p['deadband']
        e_db = math.copysign(max(0.0, abs(e)-d), e) * abs(e)/(abs(e)+1e-6)

        # feedforward (decays while coasting)
        ff = p['Kff'] * lamd_pred
        if coasting:
            ff *= math.exp(-(age - p['t_miss'])/p['tau_coast'])

        # filtered derivative
        Tf = p['Tf']
        self.ed_f = (Tf*self.ed_f + (e - self.e_prev)) / (Tf + dt)
        self.e_prev = e

        # gain blend after (re)acquisition
        self.blend_t += dt
        k = min(1.0, self.blend_t / p['T_blend'])

        fb = k*(p['Kp']*e_db + self.I + p['Kd']*self.ed_f)
        # braking envelope on feedback portion
        vmax_fb = math.sqrt(2*p['a_max']*abs(e)) + 1e-3
        fb = max(-vmax_fb, min(vmax_fb, fb))

        w_raw = ff + fb
        w_out = self._limit(w_raw, dt, e if not coasting else None)
        return w_out

    def _limit(self, w_raw, dt, e):
        p = self.p
        w_sat = max(-p['w_max'], min(p['w_max'], w_raw))
        a_des = (w_sat - self.w_cmd) / dt
        jd = p['j_max']*dt
        a = max(self.a_cmd - jd, min(self.a_cmd + jd, a_des))
        a = max(-p['a_max'], min(p['a_max'], a))
        self.a_cmd = a
        self.w_cmd += a*dt
        # back-calculation anti-windup
        if e is not None:
            self.I += (p['Ki']*e + (self.w_cmd - w_raw)/p['Tt']) * dt
            self.I = max(-p['I_max'], min(p['I_max'], self.I))
        return self.w_cmd
```

### Example parameters (handheld/drone 3-axis gimbal, 30 Hz detector, angles in rad)

```python
params = dict(
    Kp=3.0, Ki=0.4, Kd=0.05, Kff=0.95, Tf=0.03, Tt=0.5, I_max=0.3,
    w_max=math.radians(120), a_max=math.radians(400), j_max=math.radians(4000),
    deadband=math.radians(0.2), tau_act=0.06,
    alpha=0.5, beta=0.15, gamma=0.01, gate=math.radians(8),
    t_miss=0.07, t_coast=0.8, tau_coast=0.3, T_blend=0.3,
)
```

---

## 9. Stability & Tuning Guidance

1. **Bandwidth vs latency:** with total delay τ, keep the outer-loop crossover ω_c ≲ 1/(3τ). For τ = 80 ms → ω_c ≈ 4 rad/s → K_p ≈ 3–4 s⁻¹ (pure-integrator plant from rate cmd to angle).
2. **Loop separation:** inner rate-loop bandwidth should be ≥ 5× outer loop.
3. **Tuning order:**
   1. K_ff = 0, K_i = K_d = 0; raise K_p until slight overshoot on step, back off 30%.
   2. Enable K_ff, increase toward 1.0 while tracking a constant-velocity target; stop when lag error is ~0 without overshoot at direction reversals.
   3. Add K_i ≈ K_p²/10 for bias rejection (mount drift, wind).
   4. Set a_max to ~70% of actuator capability; j_max ≈ 10·a_max for cinematic smoothness (lower = smoother, more lag).
   5. Add K_d only if residual oscillation remains.
4. **Smoothness metrics:** RMS pixel error, peak jerk of ω_cmd, overshoot after target stop, time-to-reacquire.
5. **Cross-axis:** at high pitch, yaw rate in the camera frame scales by cos θ; divide yaw command by max(cos θ, 0.2) or control in the stabilized frame to avoid gimbal-lock gain blow-up.

---

## 10. Cycle 21 Summary of Design Choices

| Feature | Purpose |
|---|---|
| LOS reconstruction with capture-time gimbal angle | Decouples target motion from gimbal motion → clean FF |
| α-β-γ estimator + gating | Smooth λ̇ for FF, robust to