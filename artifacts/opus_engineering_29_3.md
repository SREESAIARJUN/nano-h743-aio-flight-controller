# Autonomous Gimbal Tracking Controller: PID with Velocity Feedforward and Acceleration Limiting

*(Cycle 29 is treated here as a design-iteration label. If it refers to a specific prior spec, share the deltas and I'll fold them in.)*

---

## 1. System Definition

**Axes:** pan (yaw, ψ) and tilt (pitch, θ). Each axis has an independent controller; the derivation is shown per axis.

**Inner loop (assumed):** The gimbal motor driver accepts **angular rate commands** ω_cmd and runs its own fast rate loop (bandwidth ≥ 5–10× the outer loop). The design below is the **outer visual loop**.

**Signals**

| Symbol | Meaning |
|---|---|
| (u, v) | Target centroid in the image (px) |
| (c_x, c_y) | Principal point (px) |
| f_x, f_y | Focal lengths (px) |
| ψ_g, θ_g | Gimbal joint angles (encoders) |
| ω_base | Vehicle/base angular rate (IMU) |
| T_s | Control period |
| τ_d | Total vision latency (exposure + inference + transport) |

---

## 2. Error Formulation (Pixel → Angle)

Convert pixel error to angular error so the gains are independent of zoom and resolution:

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

The sign on tilt accounts for image v increasing downward.

**Deadband (smoothness):** suppress micro-jitter near center with a smooth deadband, not a hard cutoff:

$$
\tilde e = \operatorname{sign}(e)\cdot\max(0,\ |e| - \delta)
$$

A typical value is δ ≈ 0.2–0.5°, or about 1–2 detector-noise σ.

**Latency compensation:** the measurement describes the scene τ_d seconds ago. Since then the gimbal has moved, so correct the error using the logged gimbal angle history:

$$
e_k^{\text{comp}} = \tilde e_k - \big(\psi_g(t_k) - \psi_g(t_k - \tau_d)\big)
$$

---

## 3. Target Velocity Estimation (Feedforward Source)

Form the target's **inertial bearing** (line-of-sight angle):

$$
\lambda_k = \psi_g(t_k - \tau_d) + e_k
$$

Estimate the LOS rate with an **α-β filter**, or a 2-state constant-velocity Kalman filter:

$$
\begin{aligned}
\hat\lambda^-_k &= \hat\lambda_{k-1} + \hat{\dot\lambda}_{k-1} T_s \\
r_k &= \lambda_k - \hat\lambda^-_k \\
\hat\lambda_k &= \hat\lambda^-_k + \alpha\, r_k \\
\hat{\dot\lambda}_k &= \hat{\dot\lambda}_{k-1} + \tfrac{\beta}{T_s}\, r_k
\end{aligned}
$$

Typical gains are α ≈ 0.3–0.5 and β ≈ 0.05–0.15. Lower values give smoother output but more lag.

**Latency-predicted feedforward:**

$$
\omega_{ff} = k_{ff}\,\hat{\dot\lambda}_k - \omega_{\text{base}}
$$

- k_ff ∈ [0.8, 1.0]; values slightly below 1 add robustness to estimate overshoot.
- Subtracting ω_base cancels vehicle rotation if the gimbal is not already stabilized in the inertial frame. Omit this term if the inner loop already stabilizes.

---

## 4. PID Feedback Law

$$
\omega_{fb} = K_p\, e + K_i \!\int e\, dt + K_d\, \dot e_f
$$

**Derivative on filtered error.** Use a first-order low-pass on the derivative to avoid amplifying detector noise:

$$
\dot e_{f,k} = \frac{\tau_f}{\tau_f + T_s}\,\dot e_{f,k-1} + \frac{1}{\tau_f + T_s}\,(e_k - e_{k-1})
$$

Use τ_f ≈ T_s to 3T_s. With feedforward carrying the tracking load, K_d is often small or zero.

**Integral with anti-windup.** Use back-calculation:

$$
I_k = I_{k-1} + T_s\left[K_i e_k + \frac{1}{T_t}(\omega_{\text{out},k-1} - \omega_{\text{des},k-1})\right]
$$

Here T_t ≈ √(T_i T_d), or about T_i if D is unused. Also **freeze the integrator** while the target is lost or while acceleration limiting is active.

**Total desired rate:**

$$
\omega_{\text{des}} = \omega_{ff} + \omega_{fb}
$$

---

## 5. Velocity and Acceleration Limiting (Smooth Follow)

**Velocity saturation:**

$$
\omega_{\text{sat}} = \operatorname{clamp}(\omega_{\text{des}},\ -\omega_{\max},\ \omega_{\max})
$$

**Acceleration limiting** is a rate limiter on the command:

$$
\Delta\omega = \operatorname{clamp}\big(\omega_{\text{sat}} - \omega_{\text{out},k-1},\ -a_{\max}T_s,\ a_{\max}T_s\big)
$$
$$
\omega_{\text{out},k} = \omega_{\text{out},k-1} + \Delta\omega
$$

**Optional jerk limit** for cinematic smoothness: apply the same clamp structure one level up. Track a_k, limit its change to ±j_max T_s, then integrate.

**Braking-aware limit.** This prevents overshoot when approaching the target under an acceleration cap. Cap the speed so the gimbal can still stop within the remaining error:

$$
|\omega_{fb}| \le \sqrt{2\,a_{\max}\,|e|}
$$

This makes large re-acquisitions follow a time-optimal trapezoidal profile instead of overshooting.

---

## 6. Mode Logic

| State | Condition | Behavior |
|---|---|---|
| **TRACK** | Valid detection, confidence ≥ c_min | Full law above |
| **COAST** | Lost for < T_coast (≈0.5–1 s) | ω = ω_ff only (decaying: ω_ff · e^{-t/τ_c}), integrator frozen |
| **HOLD/SEARCH** | Lost for > T_coast | Decelerate to 0 at a_max, then hold or run a search pattern |
| **REACQUIRE** | Detection returns | Reset derivative state, re-seed the α-β filter, and keep ω_out continuous (no reset) for a bumpless transfer |

Mechanical limits: near a joint limit, scale ω_max down linearly over the last few degrees. Never command into a hard stop.

---

## 7. Reference Implementation (Python, per axis)

```python
import math

class AxisTracker:
    def __init__(self, Ts, Kp, Ki, Kd, kff=0.9, tau_f=None,
                 w_max=math.radians(120), a_max=math.radians(400),
                 deadband=math.radians(0.3), alpha=0.4, beta=0.1, Tt=0.5):
        self.Ts, self.Kp, self.Ki, self.Kd, self.kff = Ts, Kp, Ki, Kd, kff
        self.tau_f = tau_f or 2 * Ts
        self.w_max, self.a_max, self.db = w_max, a_max, deadband
        self.alpha, self.beta, self.Tt = alpha, beta, Tt
        self.I = 0.0; self.e_prev = 0.0; self.de_f = 0.0
        self.lam = None; self.lam_dot = 0.0
        self.w_out = 0.0; self.w_des_prev = 0.0

    def _deadband(self, e):
        return math.copysign(max(0.0, abs(e) - self.db), e)

    def step(self, e_meas, gimbal_angle_then, gimbal_delta_since,
             w_base=0.0, valid=True):
        Ts = self.Ts
        if valid:
            # LOS estimation (alpha-beta)
            lam_meas = gimbal_angle_then + e_meas
            if self.lam is None:
                self.lam = lam_meas
            pred = self.lam + self.lam_dot * Ts
            r = lam_meas - pred
            self.lam = pred + self.alpha * r
            self.lam_dot += (self.beta / Ts) * r

            # Latency-compensated, deadbanded error
            e = self._deadband(e_meas) - gimbal_delta_since

            # Filtered derivative
            self.de_f = (self.tau_f * self.de_f + (e - self.e_prev)) / (self.tau_f + Ts)
            self.e_prev = e

            # Integral with back-calculation anti-windup
            self.I += Ts * (self.Ki * e + (self.w_out - self.w_des_prev) / self.Tt)

            w_fb = self.Kp * e + self.I + self.Kd * self.de_f
            # Braking-aware cap on feedback
            w_brake = math.sqrt(2 * self.a_max * abs(e))
            w_fb = max(-w_brake, min(w_brake, w_fb))

            w_ff = self.kff * self.lam_dot - w_base
        else:
            # COAST: decay feedforward, freeze integrator
            self.lam_dot *= math.exp(-Ts / 0.5)
            w_ff, w_fb = self.kff * self.lam_dot - w_base, 0.0

        w_des = w_ff + w_fb
        self.w_des_prev = w_des

        # Velocity saturation
        w_sat = max(-self.w_max, min(self.w_max, w_des))
        # Acceleration limit
        dw_max = self.a_max * Ts
        self.w_out += max(-dw_max, min(dw_max, w_sat - self.w_out))
        return self.w_out
```

Instantiate one `AxisTracker` for pan and one for tilt. Tilt typically gets lower ω_max and a_max because of payload inertia.

---

## 8. Tuning Procedure

1. **Feedforward off, K_i = K_d = 0.** Raise K_p until a step response reaches about 10% overshoot, then back off 30%. A starting point is K_p ≈ 2–4 s⁻¹, which gives an outer-loop bandwidth of about 0.3–0.6 Hz for smooth filming. Raise it for aggressive tracking.
2. **Enable feedforward.** Track a constant-velocity target, such as a turntable or a walking person. Raise k_ff until steady-state lag ≈ 0 with no overshoot on stops.
3. **Add K_i** only if a residual bias remains (base drift, gravity-induced tilt sag): K_i ≈ K_p / (5–10 s).
4. **Add K_d** only if the step response rings. Keep τ_f ≥ T_s.
5. **Set a_max** from the motor torque budget: a_max ≈ 0.6 · τ_motor / J_payload. Lower it further for a cinematic feel.
6. **Check latency margin.** The outer crossover must satisfy ω_c · τ_d ≲ 0.3 rad (≥ 17° of phase lost at most). For 100 ms latency, keep ω_c ≲ 3 rad/s.

**Starting values (30 Hz vision, 100–200 Hz control, 1–2 kg payload)**

| Param | Pan | Tilt |
|---|---|---|
| K_p (s⁻¹) | 3.0 | 2.5 |
| K_i (s⁻²) | 0.4 | 0.3 |
| K_d (–) | 0.05 | 0.05 |
| k_ff | 0.9 | 0.9 |
| ω_max (°/s) | 120 | 90 |
| a_max (°/s²) | 400 | 250 |
| deadband (°) | 0.3 | 0.3 |
| α / β | 0.4 / 0.1 | 0.4 / 0.1 |

---

## 9. Key Properties

- **Zero steady-state lag** on constant-velocity targets via feedforward, without relying on high integral gain.
- **Bounded, smooth motion:** speed, acceleration, and optionally jerk are all limited, and the braking-aware cap prevents overshoot.
- **Robust to latency and dropouts:** gimbal history compensation, α-β prediction, coast mode, and bumpless reacquisition.
- **No windup** under saturation or acceleration limiting, thanks to back-calculation and integrator freezing.

If you share the hardware (motor driver interface, IMU availability, frame rate, latency) or what changed from the previous cycle, I can refine the gains or switch the estimator to a full Kalman/IMM for maneuvering targets.