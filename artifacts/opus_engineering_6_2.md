# Autonomous Gimbal Tracking Controller: PID + Velocity Feedforward + Acceleration Limiting (Cycle 6)

I don't have the earlier cycles, so this is a self-contained formulation. Where it refines a basic PID tracker, the change is marked **[C6]**. The controller runs independently on the **yaw (pan)** and **pitch (tilt)** axes. The gimbal is assumed to accept **angular rate commands**, which is typical for brushless gimbals with an inner rate loop.

---

## 1. Signal Chain

```
Camera → Detector → Pixel error → Angular LOS error ─┐
                                                      ├→ Target estimator (α-β / KF, latency comp.)
Gimbal encoders/IMU → Gimbal angle θ_g ───────────────┘
        │
        ▼
   ω_ff (target LOS rate)  +  PID(e)  →  Velocity limit  →  Accel/Jerk limit  →  Gimbal rate loop
```

---

## 2. Measurement Model

**Pixel to angle.** Use the camera intrinsics rather than a linear scale:

$$
e_{\psi}^{meas} = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_{\phi}^{meas} = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

**World-frame target line of sight [C6].** Track the target in an inertial or base frame, not in the image. This decouples target motion from gimbal motion:

$$
\lambda_k = \theta_g(t_k - \tau) + e^{meas}_k
$$

Here τ is the total vision latency (exposure + inference + transport). Take θ_g from a timestamped encoder buffer at the image capture time, not at the time the detection arrives.

---

## 3. Target State Estimator

The state is $x = [\lambda,\ \dot\lambda]^T$. An α-β filter is shown; a constant-velocity Kalman filter is a drop-in replacement.

**Predict:**
$$
\hat\lambda^- = \hat\lambda + \hat{\dot\lambda}\,T_v, \qquad \hat{\dot\lambda}^- = \hat{\dot\lambda}
$$

**Update** with residual $r = \lambda_k - \hat\lambda^-$:
$$
\hat\lambda = \hat\lambda^- + \alpha r,\qquad \hat{\dot\lambda} = \hat{\dot\lambda}^- + \frac{\beta}{T_v} r
$$

Typical values are α ≈ 0.5–0.8 and β ≈ 0.1–0.3. Lower values give a smoother but laggier estimate.

**Latency compensation [C6].** Propagate the estimate to the present control time:
$$
\lambda^\*(t) = \hat\lambda + \hat{\dot\lambda}\,(t - t_k + \tau)
$$

**Control error:**
$$
e(t) = \operatorname{wrap}\big(\lambda^\*(t) - \theta_g(t)\big)
$$

---

## 4. Control Law

The control loop runs at a fixed rate $T_s$ (e.g. 200–500 Hz). This is faster than vision, which runs at 30–60 Hz. The estimator interpolates between frames.

### 4.1 Velocity Feedforward
$$
\omega_{ff} = K_{ff}\cdot \hat{\dot\lambda}\cdot c, \qquad K_{ff}\approx 0.9\text{–}1.0
$$

The confidence weight $c \in [0,1]$ comes from detection confidence and track age.

- With feedforward, the PID only corrects *residual* error.
- The steady-state lag on a constant-velocity target, which is $\dot\lambda / K_p$ for P-only control, is mostly eliminated.

### 4.2 PID Feedback (discrete)

**Proportional:**
$$
P_k = K_p\, e_k
$$

**Integral, with conditional integration and back-calculation anti-windup [C6]:**
$$
I_k = I_{k-1} + K_i\, e_k T_s + K_{aw}\,(\omega_{out,k-1} - \omega_{raw,k-1})\,T_s
$$

Freeze the integral when |e| > $e_{I,max}$ (large reacquisition errors) or when the target is lost.

**Derivative on filtered error rate [C6].** Feedforward already handles target velocity, so D acts as damping on the error rate only:
$$
\dot e_k = \hat{\dot\lambda} - \dot\theta_{g,k}
$$
$$
D_k = K_d\,\big[\gamma D_{k-1}/K_d + (1-\gamma)\,\dot e_k\big], \qquad \gamma = \frac{\tau_d}{\tau_d + T_s}
$$

Using the estimator rate and the gimbal IMU rate avoids differentiating noisy pixel jumps.

**Raw command:**
$$
\omega_{raw,k} = \omega_{ff} + P_k + I_k + D_k
$$

### 4.3 Smoothness Shaping

**1. Soft deadband.** This removes jitter when the target is centered without creating a hard discontinuity:
$$
e' = \operatorname{sign}(e)\cdot\max(0, |e| - e_{db})\cdot\frac{|e|}{|e| + e_{db}}
$$

**2. Error-scheduled gain [C6].** This keeps small errors gentle and large errors snappy:
$$
K_p(e) = K_{p0}\left(1 + k_s \tanh\!\frac{|e|}{e_s}\right)
$$

**3. Velocity saturation:**
$$
\omega_{sat} = \operatorname{clip}(\omega_{raw}, -\omega_{max}, \omega_{max})
$$

**4. Acceleration limiting.** Use a rate limiter on the command:
$$
\Delta\omega = \operatorname{clip}(\omega_{sat} - \omega_{out,k-1},\ -a_{max}T_s,\ a_{max}T_s)
$$

**5. Optional jerk limit [C6].** This gives an S-curve and is the main lever for "cinematic" smoothness:
$$
a_k = \operatorname{clip}\!\left(\frac{\Delta\omega}{T_s},\ a_{k-1} - j_{max}T_s,\ a_{k-1} + j_{max}T_s\right)
$$
$$
\omega_{out,k} = \omega_{out,k-1} + a_k T_s
$$

**6. Braking-aware limit [C6].** Without this, accel limiting can cause overshoot. Cap the velocity so the gimbal can still stop at the target:
$$
|\omega_{out}| \le \sqrt{2\,a_{max}\,|e|} + |\hat{\dot\lambda}|
$$

---

## 5. Target-Loss Behaviour

| State | Behaviour |
|---|---|
| Tracking (c > 0.5) | Full law |
| Coasting (lost < T_coast, e.g. 0.5 s) | Predict λ\* with constant velocity; decay ω_ff by $e^{-t/\tau_c}$; freeze I |
| Lost | Ramp ω to 0 under a_max; hold or begin a search pattern |
| Reacquire | Reset estimator to the new λ, set $\hat{\dot\lambda}$ = 0, reset I, keep ω_out (bumpless) |

---

## 6. Reference Implementation (Python, per axis)

```python
import math

class AxisTracker:
    def __init__(self, Ts, Kp=4.0, Ki=0.5, Kd=0.15, Kff=0.95, Kaw=2.0,
                 w_max=math.radians(180), a_max=math.radians(400),
                 j_max=math.radians(3000), e_db=math.radians(0.3),
                 e_Imax=math.radians(5), tau_d=0.02, ks=0.5, es=math.radians(5)):
        self.__dict__.update(locals()); del self.__dict__['self']
        self.I = 0.0; self.D = 0.0
        self.w_out = 0.0; self.a = 0.0; self.w_raw_prev = 0.0
        self.lam = None; self.lam_dot = 0.0; self.t_meas = 0.0

    # Called at vision rate
    def vision_update(self, lam_meas, t_capture, Tv, alpha=0.6, beta=0.2):
        if self.lam is None:
            self.lam, self.lam_dot = lam_meas, 0.0
            self.I = 0.0
        else:
            lp = self.lam + self.lam_dot * Tv
            r = wrap(lam_meas - lp)
            self.lam = lp + alpha * r
            self.lam_dot += beta / Tv * r
        self.t_meas = t_capture

    # Called at control rate
    def step(self, t, theta_g, theta_g_dot, conf=1.0):
        Ts = self.Ts
        if self.lam is None:
            return self._limit(0.0)

        lam_star = self.lam + self.lam_dot * (t - self.t_meas)  # latency comp.
        e = wrap(lam_star - theta_g)

        # soft deadband
        ae = abs(e)
        e_s = math.copysign(max(0.0, ae - self.e_db) * ae / (ae + self.e_db + 1e-9), e)

        # feedforward
        w_ff = self.Kff * self.lam_dot * conf

        # scheduled P
        Kp = self.Kp * (1 + self.ks * math.tanh(ae / self.es))
        P = Kp * e_s

        # I with conditional integration + back-calculation
        if ae < self.e_Imax and conf > 0.5:
            self.I += self.Ki * e_s * Ts
        self.I += self.Kaw * (self.w_out - self.w_raw_prev) * Ts

        # D on error rate (filtered)
        e_dot = self.lam_dot - theta_g_dot
        g = self.tau_d / (self.tau_d + Ts)
        self.D = g * self.D + (1 - g) * self.Kd * e_dot

        w_raw = w_ff + P + self.I + self.D
        self.w_raw_prev = w_raw

        # braking-aware velocity cap
        w_brake = math.sqrt(2 * self.a_max * ae) + abs(self.lam_dot)
        w_cap = min(self.w_max, w_brake)
        return self._limit(max(-w_cap, min(w_cap, w_raw)))

    def _limit(self, w_target):
        Ts = self.Ts
        a_des = (w_target - self.w_out) / Ts
        a_des = max(-self.a_max, min(self.a_max, a_des))
        self.a = max(self.a - self.j_max * Ts, min(self.a + self.j_max * Ts, a_des))
        self.w_out += self.a * Ts
        return self.w_out

def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi
```

---

## 7. Tuning Procedure

1. **Identify the plant.** Measure the gimbal rate-loop bandwidth $\omega_{bw}$ and the total vision latency τ.
2. **Set the outer-loop crossover.** Use $\omega_c \le \min(\omega_{bw}/5,\ 1/(3\tau))$, then start with $K_p \approx \omega_c$. For example, τ = 80 ms gives $K_p$ ≈ 3–4 s⁻¹.
3. **Tune feedforward.** Track a constant-speed target and raise $K_{ff}$ until the steady lag is ≈ 0, then back off 5%.
4. **Add integral action.** Use $K_i \approx K_p\,\omega_c/10$ to remove the residual bias (gravity, cable drag).
5. **Add derivative action.** Increase $K_d$ only if overshoot appears and $\tau_d \approx 1/(5\omega_c)$.
6. **Set the limits.**
   - Set $a_{max}$ from the payload and motor torque, at about 70% of capability.
   - Set $j_{max}$ for the desired feel: lower values look more cinematic.
   - Verify that braking distance $\omega^2/(2a_{max})$ stays inside half the FOV.
7. **Set the deadband.** Use 1–2× the detector's centroid jitter.

**Stability check.** Phase margin must stay above 45° including the latency term $e^{-s\tau}$. Feedforward does not affect closed-loop stability; it only improves tracking.

---

## 8. Cycle 6 Summary of Refinements

- World-frame LOS estimation with timestamp-aligned encoder data.
- Latency-compensated prediction of the target position to the present control time.
- Derivative on error *rate* from the estimator and IMU, rather than on raw pixels.
- Back-calculation anti-windup combined with conditional integration.
- Error-scheduled $K_p$ and a soft deadband.
- Jerk-limited S-curve output with a braking-aware velocity cap.
- Explicit coast, lost, and reacquire state handling with bumpless transfer.

If you share the specifics from earlier cycles (plant model, rates, hardware), I can adapt the gains and verify margins analytically.