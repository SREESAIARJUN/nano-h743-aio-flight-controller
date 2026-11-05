# Autonomous Gimbal Tracking Controller (Cycle 13)
**PID + velocity feedforward + rate/acceleration limiting, for smooth visual follow**

---

## 1. Problem Setup

The controller runs independently for each axis, pan (yaw, ψ) and tilt (pitch, θ). The gimbal accepts **angular rate commands** ω_cmd. The goal is to keep the detected target at the image center (or at an offset framing point) while producing cinematically smooth motion: no jerks, no oscillation, and minimal lag on moving targets.

| Symbol | Meaning |
|---|---|
| (u, v) | Target centroid in pixels (from detector/tracker) |
| (c_x, c_y) | Desired framing point (image center by default) |
| f_x, f_y | Focal lengths in pixels (from intrinsics, scaled by zoom) |
| Ω_g | Gimbal inertial angular rate (IMU/encoder) |
| τ_d | Total vision latency (exposure → detection → command) |
| T_s | Control period |

---

## 2. Measurement Model: Pixel Error → Angular Error

Convert pixel error to angular error so that gains are independent of resolution and zoom:

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

The sign on tilt accounts for image v increasing downward. With zoom, f_x = f_{x,0} · z, which automatically reduces angular error per pixel at high zoom.

**Latency alignment:** the error e_k was measured at time t − τ_d. Store a short history of gimbal angles and compute the error relative to the gimbal pose *at capture time*. This avoids treating the gimbal's own motion as target motion.

---

## 3. Target Line-of-Sight (LOS) Rate Estimation (Feedforward Source)

The inertial LOS angle to the target is

$$
\lambda = \phi_g(t-\tau_d) + e
$$

where φ_g is the gimbal angle at capture time. The target's inertial angular rate is λ̇. It is estimated with a constant-velocity Kalman filter (or an α-β filter) on λ:

**State:** x = [λ, λ̇]ᵀ

**Prediction step:**

$$
x_{k|k-1} = \begin{bmatrix}1 & T\\ 0 & 1\end{bmatrix}x_{k-1}, \quad
Q = q\begin{bmatrix}T^3/3 & T^2/2\\ T^2/2 & T\end{bmatrix}
$$

**Measurement:** z = λ_meas, H = [1, 0], R = σ_pix² / f_x².

**α-β equivalent (simpler):**

$$
\begin{aligned}
\hat\lambda^- &= \hat\lambda + T\hat{\dot\lambda},\\
r &= \lambda_{meas} - \hat\lambda^-,\\
\hat\lambda &= \hat\lambda^- + \alpha r,\\
\hat{\dot\lambda} &= \hat{\dot\lambda} + \tfrac{\beta}{T} r
\end{aligned}
$$

Typical values are α ≈ 0.3–0.5 and β ≈ 0.05–0.15. Lower values give smoother motion but more lag.

**Latency prediction:** project the error forward to "now":

$$
\hat e_{now} = e + \hat{\dot\lambda}\,\tau_d - (\phi_g(t) - \phi_g(t-\tau_d))
$$

---

## 4. Control Law

$$
\boxed{\;
\omega_{raw} = \underbrace{K_{ff}\,\hat{\dot\lambda}}_{\text{feedforward}}
+ \underbrace{K_p\,\tilde e}_{\text{P}}
+ \underbrace{K_i \!\int\! \tilde e\,dt}_{\text{I}}
+ \underbrace{K_d\,\dot{\tilde e}_f}_{\text{D (filtered)}}
\;}
$$

### 4.1 Error Shaping (Smooth-Follow Behavior)

Shaping the error before it reaches the PID gives a "lazy cameraman" feel near center and firm correction far from center.

- **Soft deadband** (radius e₀) suppresses micro-jitter from detection noise:

$$
\tilde e = \begin{cases} 0 & |e|<e_0 \\ \operatorname{sgn}(e)\,(|e|-e_0) & \text{otherwise}\end{cases}
$$

  A smooth alternative is ẽ = e − e₀ tanh(e/e₀).
- **Gain scheduling:** use a lower K_p near center and a higher K_p near the frame edge, for example K_p(e) = K_{p0}(1 + k_s|e|/e_max). This prevents losing fast targets while keeping centered shots calm.

### 4.2 Term Details

| Term | Role | Notes |
|---|---|---|
| **K_ff** | Matches target motion so the PID only corrects residuals. This eliminates steady-state lag on constant-velocity targets. | Set K_ff ≈ 0.9–1.0. Slightly below 1 avoids overshoot from estimator noise. |
| **K_p** | Main centering stiffness. | Bandwidth ≈ K_p (rad/s per rad). Keep it at or below roughly 1/(4τ_d) for latency robustness. |
| **K_i** | Removes bias (mount offset, drag, imperfect FF). | Keep it small. Use anti-windup (§5.3). |
| **K_d** | Damping. Uses the **derivative of the filtered error**, not the raw error. | Use a first-order filter with N ≈ 5–10. Often small or zero because FF already damps. |

**Filtered derivative:**

$$
\dot{\tilde e}_{f,k} = \frac{T_f}{T_f+T_s}\dot{\tilde e}_{f,k-1} + \frac{1}{T_f+T_s}(\tilde e_k - \tilde e_{k-1}), \quad T_f = K_d/(K_p N)
$$

---

## 5. Output Conditioning: Rate, Acceleration, and Jerk Limiting

### 5.1 Rate Limit

$$
\omega_{sat} = \operatorname{clip}(\omega_{raw}, -\omega_{max}, \omega_{max})
$$

### 5.2 Acceleration Limit (Core Smoothness Mechanism)

$$
\omega_{cmd,k} = \omega_{cmd,k-1} + \operatorname{clip}\!\left(\omega_{sat}-\omega_{cmd,k-1},\; -a_{max}T_s,\; a_{max}T_s\right)
$$

**Optional jerk limit** (for S-curve, cinematic motion): track α_k = acceleration and limit Δα to j_max·T_s.

**Stopping-aware limiting:** to avoid overshoot when approaching center at speed, cap the commanded rate by the braking distance:

$$
|\omega| \le \sqrt{2\,a_{max}\,|\tilde e|} + |\hat{\dot\lambda}|
$$

### 5.3 Anti-Windup (Back-Calculation)

Integrate only what was actually delivered:

$$
I_{k} = I_{k-1} + T_s\left[K_i\tilde e_k + \frac{1}{T_t}(\omega_{cmd,k} - \omega_{raw,k})\right], \quad T_t \approx \sqrt{T_i T_d}\ \text{or}\ \approx T_i/2
$$

Also clamp |I| ≤ I_max, and freeze the integrator when the target is lost.

---

## 6. Target-Loss and Re-Acquisition Logic

| State | Behavior |
|---|---|
| **TRACKING** | Full control law. |
| **COAST** (lost < t_coast, e.g., 0.5–1 s) | ω = K_ff·λ̇̂ decaying: λ̇̂ ← λ̇̂·e^{−T_s/τ_c}. Integrator frozen. |
| **SEARCH/HOLD** (lost > t_coast) | Decelerate to zero at a_max, or run a search pattern. Reset I and the derivative state. |
| **RE-ACQUIRE** | Re-initialize the estimator with λ̇̂ = 0 (or the coast value). Ramp K_p from 0 to nominal over ~0.3 s to avoid a snap. |

Also gate measurements with a **confidence threshold** and an innovation gate: reject the measurement if |r| > 3√S, which catches detector glitches.

---

## 7. Discrete Implementation (Per Axis)

```python
import math

class AxisTracker:
    def __init__(self, Kp, Ki, Kd, Kff, w_max, a_max, j_max=None,
                 e0=0.005, alpha=0.4, beta=0.1, N=8, Tt=0.5, I_max=0.3):
        self.Kp, self.Ki, self.Kd, self.Kff = Kp, Ki, Kd, Kff
        self.w_max, self.a_max, self.j_max = w_max, a_max, j_max
        self.e0, self.alpha, self.beta, self.N = e0, alpha, beta, N
        self.Tt, self.I_max = Tt, I_max
        self.reset()

    def reset(self):
        self.lam = None; self.lam_dot = 0.0
        self.I = 0.0; self.e_prev = 0.0; self.d_f = 0.0
        self.w_cmd = 0.0; self.acc = 0.0

    def shape(self, e):
        return e - self.e0 * math.tanh(e / self.e0)      # soft deadband

    def step(self, Ts, e_meas, gimbal_angle_at_capture,
             gimbal_angle_now, latency, valid):
        # --- LOS estimation (alpha-beta) ---
        if valid:
            lam_meas = gimbal_angle_at_capture + e_meas
            if self.lam is None:
                self.lam, self.lam_dot = lam_meas, 0.0
            lam_pred = self.lam + Ts * self.lam_dot
            r = lam_meas - lam_pred
            self.lam = lam_pred + self.alpha * r
            self.lam_dot += (self.beta / Ts) * r
            # latency-compensated error relative to current pose
            e = (self.lam + self.lam_dot * latency) - gimbal_angle_now
        else:
            self.lam_dot *= math.exp(-Ts / 0.4)            # coast decay
            e = 0.0

        e_s = self.shape(e)

        # --- PID with filtered derivative ---
        Tf = self.Kd / (self.Kp * self.N) if self.Kd > 0 else 0.0
        self.d_f = (Tf * self.d_f + (e_s - self.e_prev)) / (Tf + Ts)
        self.e_prev = e_s

        w_raw = (self.Kff * self.lam_dot + self.Kp * e_s
                 + self.I + self.Kd * self.d_f)

        # --- braking-aware rate cap ---
        w_brake = math.sqrt(2 * self.a_max * abs(e_s)) + abs(self.lam_dot)
        w_lim = min(self.w_max, w_brake)
        w_sat = max(-w_lim, min(w_lim, w_raw))

        # --- acceleration (and optional jerk) limiting ---
        a_des = (w_sat - self.w_cmd) / Ts
        a_des = max(-self.a_max, min(self.a_max, a_des))
        if self.j_max:
            da = max(-self.j_max * Ts, min(self.j_max * Ts, a_des - self.acc))
            a_des = self.acc + da
        self.acc = a_des
        self.w_cmd += self.acc * Ts

        # --- integrator with back-calculation anti-windup ---
        if valid:
            self.I += Ts * (self.Ki * e_s + (self.w_cmd - w_raw) / self.Tt)
            self.I = max(-self.I_max, min(self.I_max, self.I))

        return self.w_cmd
```

**Integration notes:**
- Run the control loop at 100–500 Hz. Vision updates arrive at 30–60 Hz. Between frames, run only the prediction step (valid=False path but *without* coast decay), and hold the last error propagated by λ̇̂.
- Pan/tilt coupling: at high tilt, the yaw rate needed in the camera frame scales by 1/cos θ. Apply ω_ψ,cmd ← ω_ψ,cmd / max(cos θ, 0.2) if commanding a yaw-outer gimbal.

---

## 8. Suggested Starting Gains

These assume a rate-controlled gimbal, 30 fps vision, and τ_d ≈ 80 ms.

| Parameter | Pan | Tilt | Unit |
|---|---|---|---|
| K_p | 2.5 | 2.0 | 1/s |
| K_i | 0.3 | 0.3 | 1/s² |
| K_d | 0.05 | 0.05 | s·(1/s) |
| K_ff | 0.95 | 0.95 | – |
| ω_max | 3.0 | 2.0 | rad/s |
| a_max | 6.0 | 4.0 | rad/s² |
| j_max | 40 | 30 | rad/s³ |
| e₀ (deadband) | 0.005 | 0.005 | rad (~0.3°) |
| α, β | 0.4, 0.1 | 0.4, 0.1 | – |

**Cinematic preset:** halve K_p and a_max, double e₀, and lower β to 0.05.
**Sport/fast preset:** raise K_p to 4 and a_max to 12, and lower e₀ to 0.002.

---

## 9. Tuning Procedure

1. **Inner loop check:** verify that the gimbal rate loop tracks steps with bandwidth at least 5× the planned outer-loop bandwidth.
2. **Measure latency τ_d** with an LED-flash or a known motion test. Set the latency compensation accordingly.
3. **P only:** with K_ff = K_i = K_d = 0, increase K_p until slight overshoot appears, then back off 30%.
4. **Add feedforward:** track a constant-speed target (turntable or walking subject). Raise K_ff toward 1 until steady-state lag is below 2–3% of frame width.
5. **Tune the estimator:** if motion looks jittery, lower α and β. If the camera lags on direction changes, raise β.
6. **Add K_i** only if a residual bias remains. Then add a small K_d if overshoot persists.
7. **Set limits:** set a_max and j_max for the desired "feel," then confirm that fast targets are not lost at the frame edge (adjust gain scheduling k_s).
8. **Test robustness** against occlusion (coast behavior), false detections (innovation gate), and zoom changes (angular normalization).

---

## 10. Stability and Performance Notes

- **Phase margin:** latency adds phase lag of ω_c·τ_d. Keep the crossover frequency ω_c ≈ K_p small enough that ω_c·τ_d < ~0.5 rad (≈30°).
- **Feedforward stability:** feedforward does not affect closed-loop stability, but it does inject estimator noise. This is why K_ff < 1 and the β filtering matter.
- **Limit-induced behavior:** acceleration limiting makes the loop nonlinear. The braking cap and back-calculation anti-windup prevent the limit-cycle oscillation common with saturated PIDs.
- **Steady-state error for a ramp target:** this is ≈ (1 − K_ff)·λ̇ / K_p before the integrator acts, and zero once I converges.