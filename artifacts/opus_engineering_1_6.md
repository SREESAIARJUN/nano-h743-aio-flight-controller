# Autonomous Gimbal Visual-Tracking Controller — Cycle 1

**PID + Velocity Feedforward + Rate/Acceleration Limiting**

---

## 1. Problem Statement

Keep a visually detected target centered in the camera frame using a 2-axis (pan/tilt) gimbal. Motion should be **smooth**: no jerks, no hunting, no overshoot on target maneuvers. The controller must also be **responsive**, with minimal steady lag when the target moves at constant velocity.

Cycle 1 establishes the baseline architecture, the math, the discrete implementation and the initial tuning. Later cycles refine it (see §10).

---

## 2. System Architecture

```
            ┌──────────────┐  e_px   ┌─────────────┐  e_θ, ê̇_θ
 Camera ───▶│ Detector /   │───────▶│ Pixel→Angle │──────────┐
            │ Tracker      │         │ + Latency   │          │
            └──────────────┘         │ Compensation│          ▼
                                     └─────────────┘   ┌──────────────┐
 Gimbal encoders/IMU ─── ω_g, θ_g ────────────────────▶│ LOS-Rate     │ ω̂_T
                                                       │ Estimator    │────┐
                                                       └──────────────┘    │
                                                                           ▼
                     ┌────────┐    u_PID   ┌───────────┐ ω_raw ┌────────────────┐ ω_cmd
         e_θ ───────▶│  PID   │──────────▶│ Σ (+ FF)  │──────▶│ Rate & Accel   │──────▶ Gimbal
                     └────────┘            └───────────┘       │ Limiter        │       rate loop
                         ▲ anti-windup feedback                └────────────────┘
                         └────────────────────────────────────────────┘
```

**Design assumption.** The gimbal has an inner **rate (velocity) loop**. This is standard for brushless gimbals. Our outer loop therefore commands angular rate ω_cmd per axis.

The axes are treated as **decoupled**: pan uses the horizontal pixel error, tilt uses the vertical. Coupling is addressed in Cycle 2.

---

## 3. Plant Model (per axis)

The inner rate loop is modeled as first-order:

$$
\dot\omega_g = \frac{1}{\tau_r}(\omega_{cmd} - \omega_g), \qquad \dot\theta_g = \omega_g
$$

Typical value: τ_r ≈ 15–40 ms.

The vision measurement is delayed by the pipeline latency L (exposure + inference + transport, typically 40–120 ms):

$$
e_\theta^{meas}(t) = \theta_T(t-L) - \theta_g(t-L)
$$

Here θ_T is the inertial line-of-sight (LOS) angle to the target.

---

## 4. Measurement Formulation

### 4.1 Pixel → angular error

With focal length f in pixels (f = W / (2·tan(HFOV/2))) and principal point (c_x, c_y):

$$
e_{\theta,pan} = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_{\theta,tilt} = \arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

Working in **angles rather than pixels** makes the gains independent of zoom level and resolution.

### 4.2 Latency compensation

Each vision frame carries a capture timestamp t_c. Gimbal angle history is stored in a ring buffer, which lets us reconstruct the target's inertial LOS angle at capture time:

$$
\theta_T(t_c) = \theta_g(t_c) + e_\theta(t_c)
$$

The LOS angle is then predicted forward to the present:

$$
\hat\theta_T(t) = \theta_T(t_c) + \hat\omega_T \,(t - t_c)
$$

The **compensated error** used by the controller is:

$$
e(t) = \hat\theta_T(t) - \theta_g(t)
$$

This step removes most of the phase lag that otherwise causes oscillation at high gain.

---

## 5. Target LOS-Rate Estimator (for feedforward)

Differentiating noisy vision data directly is unusable. Instead, an **α-β filter** runs on θ_T, updated at the vision rate (period T_v):

$$
\begin{aligned}
\theta^- &= \hat\theta_T + \hat\omega_T T_v \\
r &= \theta_T^{meas} - \theta^- \\
\hat\theta_T &= \theta^- + \alpha\, r \\
\hat\omega_T &= \hat\omega_T + \frac{\beta}{T_v}\, r
\end{aligned}
$$

Starting values are α ≈ 0.5 and β ≈ 0.1. Lower values give a smoother ω̂_T at the cost of slower response. (Cycle 2 upgrades this to a constant-velocity Kalman filter with adaptive Q.)

**Key point.** Because θ_T is reconstructed in the inertial frame, ω̂_T is the target's own angular rate. It is not contaminated by the gimbal's motion or by base (vehicle) rotation, provided θ_g is inertial (IMU-referenced).

---

## 6. Control Law

### 6.1 Feedforward

$$
u_{ff} = K_{ff}\,\hat\omega_T, \qquad K_{ff}\in[0.8,\,1.0]
$$

A value of K_ff slightly below 1 trades a little lag for robustness against estimator noise.

### 6.2 PID feedback (on compensated angular error e)

**Proportional:**

$$
u_P = K_p\, e
$$

**Integral, with conditional integration (anti-windup):**

$$
I_k = \begin{cases} I_{k-1} + K_i\, e_k\, T_s & \text{if not saturated, or } \operatorname{sign}(e_k) \neq \operatorname{sign}(\text{saturation}) \\ I_{k-1} & \text{otherwise}\end{cases}
\qquad I_k \in [-I_{max},\, I_{max}]
$$

**Derivative, filtered, on error rate:**

$$
D_k = \frac{\tau_d}{\tau_d+T_s}D_{k-1} + \frac{K_d}{\tau_d+T_s}(e_k - e_{k-1})
$$

With feedforward present, the residual error e is small and slowly varying, so K_d is usually small or zero. It mainly damps residual motion.

### 6.3 Smooth-follow shaping (deadband + soft gain)

This prevents the gimbal from twitching on detector jitter while the target is near center. A smooth deadband replaces e before the PID:

$$
\tilde e = \begin{cases}
0 & |e| < e_{db} \\[2pt]
\operatorname{sign}(e)\,\dfrac{(|e|-e_{db})^2}{2\,e_{soft}} & e_{db} \le |e| < e_{db}+e_{soft} \\[6pt]
\operatorname{sign}(e)\left(|e| - e_{db} - \tfrac{e_{soft}}{2}\right) & \text{otherwise}
\end{cases}
$$

This function is C¹-continuous, so there is no kick at the deadband edge. Typical values: e_db ≈ 0.2–0.5°, e_soft ≈ 1°.

### 6.4 Total raw command

$$
\omega_{raw} = u_{ff} + K_p\tilde e + I + D
$$

---

## 7. Rate and Acceleration Limiting

### 7.1 Rate saturation

$$
\omega_{sat} = \operatorname{clip}(\omega_{raw},\, -\omega_{max},\, \omega_{max})
$$

### 7.2 Acceleration (slew-rate) limiting

$$
\omega_{cmd,k} = \omega_{cmd,k-1} + \operatorname{clip}\!\left(\omega_{sat}-\omega_{cmd,k-1},\; -a_{max}T_s,\; a_{max}T_s\right)
$$

### 7.3 Braking-aware limiting (prevents overshoot)

A plain slew limiter can overshoot. When the error is closing, the gimbal must be able to decelerate within the remaining angle. The command is capped by the stopping-distance constraint, measured relative to the target's motion:

$$
|\omega_{sat} - u_{ff}| \le \sqrt{2\, a_{max}\, |e|}
$$

This produces a **trapezoidal / time-optimal-like approach** for large reacquisitions and stays linear PID behavior for small errors. Optionally, the √ curve can be blended into the linear K_p·e region where K_p·e < √(2·a_max·|e|), i.e. for |e| < 2·a_max / K_p².

### 7.4 Saturation flag for anti-windup

```
saturated = (ω_cmd ≠ ω_raw)
sat_sign  = sign(ω_raw − ω_cmd)
```

---

## 8. Reference Implementation (Python, per axis)

```python
import math

class AxisTracker:
    def __init__(self, Kp=4.0, Ki=0.8, Kd=0.05, Kff=0.9, tau_d=0.03,
                 w_max=math.radians(180), a_max=math.radians(400),
                 e_db=math.radians(0.3), e_soft=math.radians(1.0),
                 I_max=math.radians(30), alpha=0.5, beta=0.1):
        self.__dict__.update(locals()); del self.__dict__['self']
        self.I = self.D = self.e_prev = 0.0
        self.w_cmd = 0.0
        self.th_T = None; self.w_T = 0.0
        self.sat_sign = 0

    # ---- vision update (≈30–60 Hz), with latency-compensated reconstruction
    def vision_update(self, e_meas, theta_g_at_capture, Tv):
        th_meas = theta_g_at_capture + e_meas            # inertial LOS
        if self.th_T is None:
            self.th_T, self.w_T = th_meas, 0.0; return
        pred = self.th_T + self.w_T * Tv
        r = th_meas - pred
        self.th_T = pred + self.alpha * r
        self.w_T += (self.beta / Tv) * r

    def _shape(self, e):
        a, db, s = abs(e), self.e_db, self.e_soft
        if a < db: return 0.0
        if a < db + s: return math.copysign((a - db)**2 / (2*s), e)
        return math.copysign(a - db - s/2, e)

    # ---- control step (≈200–500 Hz)
    def step(self, theta_g, dt_since_vision, Ts):
        if self.th_T is None: return 0.0
        th_T_now = self.th_T + self.w_T * dt_since_vision   # predict to now
        e = th_T_now - theta_g
        es = self._shape(e)

        u_ff = self.Kff * self.w_T

        # integral with conditional anti-windup
        if self.sat_sign == 0 or math.copysign(1, es) != self.sat_sign:
            self.I = max(-self.I_max, min(self.I_max, self.I + self.Ki * es * Ts))

        # filtered derivative
        self.D = (self.tau_d*self.D + self.Kd*(es - self.e_prev)) / (self.tau_d + Ts)
        self.e_prev = es

        w_raw = u_ff + self.Kp*es + self.I + self.D

        # braking-aware cap on feedback relative to feedforward
        v_brake = math.sqrt(2 * self.a_max * abs(e))
        fb = max(-v_brake, min(v_brake, w_raw - u_ff))
        w_sat = max(-self.w_max, min(self.w_max, u_ff + fb))

        # acceleration limit
        dv = max(-self.a_max*Ts, min(self.a_max*Ts, w_sat - self.w_cmd))
        self.w_cmd += dv

        diff = w_raw - self.w_cmd
        self.sat_sign = 0 if abs(diff) < 1e-6 else math.copysign(1, diff)
        return self.w_cmd

    def target_lost(self, Ts, decay_tau=0.5):
        # coast on feedforward, decaying to stop
        self.w_T *= math.exp(-Ts / decay_tau)
        self.I *= math.exp(-Ts / decay_tau)
```

Instantiate one tracker each for pan and tilt.

---

## 9. Cycle-1 Tuning Procedure

| Step | Action | Target behavior |
|---|---|---|
| 1 | Set K_ff=0, K_i=0, K_d=0. Raise K_p until a slight overshoot appears on a 10° step, then back off 30%. | Crossover ω_c ≈ 1/(3·(L+τ_r)), typically 2–5 rad/s |
| 2 | Enable K_ff = 0.9. Track a constant-velocity target (e.g. 20°/s). | Steady-state lag ≤ 0.5° |
| 3 | Add K_i ≈ K_p·ω_c/5. | Removes residual bias without slow oscillation |
| 4 | Add K_d only if there is ringing; keep τ_d ≈ 2–3·T_s. | Damped approach |
| 5 | Set a_max to ~60% of mechanical capability and ω_max to the motor/payload limit. | No visible jerk on reacquisition |
| 6 | Tune α, β, e_db on a stationary target, then on a jittery detection. | No hunting at center |

**Stability guideline.** Total loop delay (L_residual + τ_r) limits the bandwidth. Keep the phase margin above 45°:

$$
\omega_c \lesssim \frac{\pi/4}{L_{res} + \tau_r}
$$

Latency compensation reduces L_residual, which is what allows higher K_p.

**Starting parameters (typical 1080p camera, 60 ms latency):**

| Parameter | Value |
|---|---|
| K_p | 4 s⁻¹ |
| K_i | 0.8 s⁻² |
| K_d | 0.05 s |
| K_ff | 0.9 |
| ω_max | 180°/s |
| a_max | 400°/s² |
| e_db | 0.3° |
| e_soft | 1° |
| Control rate | 250 Hz |
| Vision rate | 30–60 Hz |

---

## 10. Cycle-1 Evaluation Metrics and Next Cycles

**Metrics to log:**
- RMS centering error, for both static and moving targets
- Peak overshoot on reacquisition
- Command jerk (dω_cmd/dt²) RMS, as a smoothness measure
- Settling time
- Lag at constant velocity

**Planned for Cycle 2+:**
1. Replace the α-β filter with a Kalman filter that has adaptive process noise for maneuvering targets.
2. Add jerk limiting (S-curve) on top of acceleration limiting for cinematic smoothness.
3. Model pan/tilt kinematic coupling, including the near-zenith singularity and the cos(tilt) scaling of pan rate.
4. Gain-schedule by zoom level, target range and detection confidence.
5. Add base-motion disturbance feedforward from the vehicle IMU.
6. Add a composition offset, i.e. tracking toward a rule-of-thirds point instead of the image center.