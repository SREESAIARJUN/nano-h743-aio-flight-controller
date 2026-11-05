# Autonomous Gimbal Tracking Controller: PID with Velocity Feedforward and Acceleration Limiting (Cycle 24)

## 1. System Overview

```
 Camera ──► Detector ──► Pixel→Angle ──► Target State Estimator ──┐
                                           (world-frame α-β / KF) │
                                                                  ▼
 Gimbal IMU/encoders ──────────────────────────────►  ┌─────────────────────┐
                                                      │ FF + PID (per axis) │
                                                      │ + soft deadband     │
                                                      │ + anti-windup       │
                                                      └─────────┬───────────┘
                                                                ▼
                                          Velocity limit → Accel limit → (Jerk limit)
                                                                ▼
                                                    Gimbal rate loop (motor driver)
```

The design has two loops:

* **Outer loop (this design):** produces an angular-rate command ω_cmd for each axis (pan ψ, tilt θ). It runs at the vision rate (typically 30–120 Hz).
* **Inner loop:** the existing gimbal motor or IMU rate loop. It must be at least 5–10× faster than the outer loop.

---

## 2. Measurement Model

### 2.1 Pixel to angular error

The camera intrinsics are (f_x, f_y, c_x, c_y). The detected target center is (u, v).

$$
e_\psi^{cam} = \operatorname{atan}\!\left(\frac{u - c_x}{f_x}\right),\qquad
e_\theta^{cam} = -\operatorname{atan}\!\left(\frac{v - c_y}{f_y}\right)
$$

Use the zoom-dependent f_x and f_y if the lens zooms.

### 2.2 World-frame target bearing (decouples gimbal self-motion)

Let q_g(t_img) be the gimbal angles at the **image capture timestamp**, not the processing time:

$$
\psi_T = \psi_g(t_{img}) + e_\psi^{cam},\qquad \theta_T = \theta_g(t_{img}) + e_\theta^{cam}
$$

Estimating the target in the world (or base) frame is the key step for smooth following. If you differentiate the image error directly, the gimbal's own motion corrupts the velocity estimate and causes oscillation.

For a 2-axis gimbal, also scale the pan-rate requirement by $1/\cos\theta$ near high tilt. Clamp this factor near ±90° to avoid gimbal-lock blow-up.

---

## 3. Target State Estimator

Use one per axis. The state is $x = [\phi_T,\ \dot\phi_T]^T$ under a constant-velocity model.

**Prediction** (period Δt):

$$
\hat\phi^- = \hat\phi + \Delta t\,\hat{\dot\phi},\qquad \hat{\dot\phi}^- = \hat{\dot\phi}
$$

**Update** (α-β filter, the steady-state Kalman equivalent):

$$
r = \phi_T^{meas} - \hat\phi^-,\qquad
\hat\phi = \hat\phi^- + \alpha r,\qquad
\hat{\dot\phi} = \hat{\dot\phi}^- + \frac{\beta}{\Delta t} r
$$

Typical values are α ≈ 0.3–0.6 and β ≈ 0.05–0.2. Lower values give smoother tracking with more lag.

**Latency compensation:** if the total pipeline latency is τ (exposure + inference + communication), predict forward:

$$
\phi_T^{ref} = \hat\phi + \tau\,\hat{\dot\phi}
$$

**Outlier gate:** reject the measurement and coast on prediction if $|r| > k\sigma$ (k ≈ 3–4).

---

## 4. Control Law (per axis)

### 4.1 Tracking error

$$
e_k = \operatorname{wrap}\!\left(\phi_T^{ref} - \phi_{g,k}\right)
$$

Wrap to (−π, π] for continuous pan.

### 4.2 Soft deadband

A soft deadband stops micro-jitter without creating a discontinuity:

$$
\tilde e = \begin{cases}
0 & |e| < d_0\\
\operatorname{sign}(e)\,\dfrac{(|e|-d_0)^2}{2(d_1-d_0)} & d_0 \le |e| < d_1\\
e - \operatorname{sign}(e)\,\dfrac{d_0+d_1}{2} & |e|\ge d_1
\end{cases}
$$

This is C¹-continuous. Typical values are d₀ ≈ 0.1–0.3° and d₁ ≈ 3·d₀.

### 4.3 PID with feedforward

$$
\omega_{raw,k} = \underbrace{K_{ff}\,\hat{\dot\phi}_T}_{\text{feedforward}}
+ K_p \tilde e_k + K_i I_k + K_d D_k
$$

**Filtered derivative on measurement:** differentiate the world-frame bearing, not the raw error, to avoid derivative kick when the target is reacquired:

$$
D_k = \frac{T_f}{T_f+\Delta t}D_{k-1} - \frac{1}{T_f+\Delta t}\left(\phi_{g,k}-\phi_{g,k-1}\right) + \ldots
$$

In practice, $\hat{\dot\phi}_T$ already supplies the predictive term, so K_d is often small or zero.

**Feedforward gain:** K_ff ≈ 0.9–1.0. Slightly below 1 avoids overshoot if the velocity estimate is noisy.

### 4.4 Saturation chain

**Velocity limit:**

$$
\omega_{sat} = \operatorname{clamp}(\omega_{raw}, -\omega_{max}, \omega_{max})
$$

**Acceleration limit (slew-rate):**

$$
\omega_{cmd,k} = \omega_{cmd,k-1} + \operatorname{clamp}\!\left(\omega_{sat}-\omega_{cmd,k-1},\ -a_{max}\Delta t,\ a_{max}\Delta t\right)
$$

**Optional jerk limit** for cinematic smoothness: maintain a commanded acceleration state a_k with $|a_k - a_{k-1}| \le j_{max}\Delta t$. Then integrate $\omega_k = \omega_{k-1} + a_k\Delta t$.

**Overshoot-aware braking:** limit the speed so the gimbal can stop within the remaining error:

$$
|\omega| \le \sqrt{2\,a_{max}\,|e|} + |\hat{\dot\phi}_T|
$$

This prevents acceleration-limited overshoot on large steps.

### 4.5 Anti-windup (back-calculation)

$$
I_{k} = I_{k-1} + \Delta t\left[\tilde e_k + \frac{1}{T_t}\left(\omega_{cmd,k}-\omega_{raw,k}\right)\right]
$$

Set $T_t \approx \sqrt{T_i T_d}$, or about $T_i$ if there is no D term. Additionally, freeze the integrator when the target is lost or when |e| exceeds a large-error threshold.

---

## 5. Target-Loss / Mode Logic

| Mode | Condition | Behavior |
|---|---|---|
| TRACK | valid detection | full law above |
| COAST | no detection for < T_coast (≈0.5–1 s) | ω = K_ff·φ̂̇ decaying: $\hat{\dot\phi}\leftarrow\lambda\hat{\dot\phi}$, λ≈0.9; integrator frozen |
| SEARCH/HOLD | lost > T_coast | decelerate at a_max to 0, or run a search pattern |
| REACQUIRE | detection returns | reset estimator to the measurement, keep ω_cmd (bumpless), reset D state |

---

## 6. Reference Implementation (Python)

```python
import math
from dataclasses import dataclass

def wrap(a):  # (-pi, pi]
    return (a + math.pi) % (2*math.pi) - math.pi

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

@dataclass
class AxisParams:
    kp: float = 2.5          # 1/s
    ki: float = 0.4          # 1/s^2
    kd: float = 0.0
    kff: float = 0.95
    tt: float = 1.0          # anti-windup tracking time
    w_max: float = math.radians(120)
    a_max: float = math.radians(300)
    j_max: float = math.radians(3000)   # set None to disable
    d0: float = math.radians(0.15)
    d1: float = math.radians(0.45)
    alpha: float = 0.45
    beta: float = 0.10
    latency: float = 0.06    # s
    gate: float = math.radians(8)
    coast_time: float = 0.8
    coast_decay: float = 0.9
    wrap_axis: bool = True

class AxisTracker:
    def __init__(self, p: AxisParams):
        self.p = p
        self.phi = None; self.dphi = 0.0
        self.I = 0.0; self.w = 0.0; self.a = 0.0
        self.t_lost = 0.0

    def _soft_deadband(self, e):
        d0, d1, s, ae = self.p.d0, self.p.d1, math.copysign(1, e), abs(e)
        if ae < d0: return 0.0
        if ae < d1: return s*(ae-d0)**2/(2*(d1-d0))
        return s*(ae-(d0+d1)/2)

    def step(self, dt, gimbal_angle, meas_bearing=None):
        p = self.p
        # ---- estimator ----
        if self.phi is not None:
            self.phi += dt*self.dphi
        valid = meas_bearing is not None
        if valid:
            if self.phi is None:                       # (re)acquire
                self.phi, self.dphi = meas_bearing, 0.0
            r = meas_bearing - self.phi
            if p.wrap_axis: r = wrap(r)
            if abs(r) > p.gate and self.t_lost == 0.0:
                valid = False                          # outlier → coast
            else:
                self.phi += p.alpha*r
                self.dphi += p.beta/dt*r
                self.t_lost = 0.0
        if not valid:
            self.t_lost += dt
            self.dphi *= p.coast_decay

        if self.phi is None or self.t_lost > p.coast_time:
            target_w = 0.0                             # HOLD: brake
            self.I = 0.0
            if self.t_lost > p.coast_time: self.phi = None
        else:
            ref = self.phi + p.latency*self.dphi
            e = ref - gimbal_angle
            if p.wrap_axis: e = wrap(e)
            et = self._soft_deadband(e)
            w_raw = p.kff*self.dphi + p.kp*et + p.ki*self.I
            # braking envelope
            w_brk = math.sqrt(2*p.a_max*abs(e)) + abs(self.dphi)
            lim = min(p.w_max, w_brk)
            target_w = clamp(w_raw, -lim, lim)
            # anti-windup (back-calculation), freeze during coast
            if valid:
                self.I += dt*(et + (target_w - w_raw)/(p.tt*max(p.ki,1e-9)))

        # ---- accel (+jerk) limiting ----
        a_des = clamp((target_w - self.w)/dt, -p.a_max, p.a_max)
        if p.j_max:
            self.a += clamp(a_des - self.a, -p.j_max*dt, p.j_max*dt)
            self.a = clamp(self.a, -p.a_max, p.a_max)
        else:
            self.a = a_des
        self.w += self.a*dt
        return self.w      # rate command to inner loop

class GimbalTracker:
    def __init__(self, fx, fy, cx, cy):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.pan = AxisTracker(AxisParams())
        self.tilt = AxisTracker(AxisParams(wrap_axis=False,
                                           w_max=math.radians(90)))

    def step(self, dt, pan_now, tilt_now, det=None, pan_img=None, tilt_img=None):
        mp = mt = None
        if det is not None:
            u, v = det
            mp = pan_img  + math.atan((u - self.cx)/self.fx)
            mt = tilt_img - math.atan((v - self.cy)/self.fy)
        wp = self.pan.step(dt, pan_now, mp)
        wt = self.tilt.step(dt, tilt_now, mt)
        wp /= max(math.cos(tilt_now), 0.2)   # 2-axis pan scaling, clamped
        return wp, wt
```

---

## 7. Tuning Procedure

1. **Inner loop first.** Verify the rate-loop bandwidth ω_bw,in. Set the outer loop crossover to $K_p \lesssim \omega_{bw,in}/5$ and $K_p \lesssim 1/(3\tau)$, where τ is the vision latency.
2. **Feedforward only** (K_p = K_i = 0): move the target at constant speed. Tune α, β, and the latency term until the gimbal matches the target velocity with constant lag.
3. **Add K_p** until steady lag is removed and the step response is fast without overshoot. A reasonable starting point is K_p ≈ 2–4 s⁻¹.
4. **Add K_i** small, around $K_i \approx K_p^2/10$. It removes bias from mounting misalignment or wind.
5. **Set the limits** from mechanics and aesthetics. A cinematic look uses roughly a_max ≈ 60–150 °/s² and j_max ≈ 10·a_max. Aggressive sports tracking uses roughly a_max ≈ 500+ °/s².
6. **Deadband.** Set d₀ just above the detector's jitter σ, about 2σ in angle.

### Stability check (linearized, limits inactive)

Model the plant as an integrator with delay, $G(s)=e^{-\tau s}/s$, and the controller as $C(s)=K_p + K_i/s$. Then:

* Crossover is approximately $\omega_c \approx K_p$.
* Phase margin is $PM \approx 90^\circ - \arctan(K_i/(K_p\omega_c)) - \omega_c\tau\cdot\frac{180}{\pi}$.

Target PM ≥ 45°. With τ = 60 ms, this gives K_p ≲ 4 s⁻¹ with a small K_i.

---

## 8. Performance Metrics (log every cycle)

* RMS and peak angular error (°) and pixel error.
* Commanded vs. measured rate and acceleration. This shows saturation duty and whether the limits are too tight.
* Jerk RMS, as a smoothness and cinematic quality metric.
* Time-to-center after reacquisition, and percentage of time in COAST.

This formulation gives zero-lag tracking of constant-velocity targets via feedforward and latency prediction, bias rejection via the integrator, jitter immunity via the estimator and soft deadband, and physically smooth motion through the velocity, braking, acceleration, and jerk constraints.