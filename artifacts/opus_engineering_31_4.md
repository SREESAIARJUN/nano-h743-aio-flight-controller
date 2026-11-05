# 3-Axis Rigid Body Simulator with Motor Torque Curves and Gyro Drift

I don't have context for what "Cycle 31" refers to (a course module, sprint, or earlier conversation), so I've treated it as a version label in the file header. If it implies specific requirements, tell me and I'll adapt the code.

The program is a single file that needs only `numpy`, plus optional `matplotlib` for plots. It models:

- **Rigid body dynamics:** full 3D Euler equations with an inertia tensor, quaternion attitude, and RK4 integration.
- **Actuators:** three reaction wheels, each driven by a DC/BLDC motor model. Each motor has a torque-speed curve, back-EMF, current and voltage limits, viscous and Coulomb friction, and electrical lag.
- **Gyro sensor:** constant bias, bias instability (random walk), angle random walk (white noise), scale-factor error, axis misalignment, quantization, and saturation.
- **Flight software:** an attitude estimator that integrates the gyro and can apply a periodic star-tracker correction. A quaternion PD controller tracks a slew target.

```python
#!/usr/bin/env python3
"""
rigid_body_sim.py  --  Cycle 31
3-axis rigid-body attitude simulator with reaction-wheel motor torque curves
and a realistic MEMS/FOG gyro drift model.

Usage:
    python rigid_body_sim.py              # run default scenario + plots
    python rigid_body_sim.py --no-plot    # text summary only
    python rigid_body_sim.py --no-startracker   # watch pure gyro drift
"""
from __future__ import annotations
import argparse
from dataclasses import dataclass, field
import numpy as np

# ============================================================================
# Quaternion utilities  (scalar-first: q = [w, x, y, z], body->inertial)
# ============================================================================
def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])

def q_norm(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)

def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    s = np.sin(angle / 2)
    return np.array([np.cos(angle / 2), *(axis * s)])

def q_from_rotvec(rv: np.ndarray) -> np.ndarray:
    ang = np.linalg.norm(rv)
    if ang < 1e-12:
        return q_norm(np.array([1.0, *(0.5 * rv)]))
    return q_from_axis_angle(rv / ang, ang)

def q_to_euler(q: np.ndarray) -> np.ndarray:
    """Returns roll, pitch, yaw (ZYX) in radians."""
    w, x, y, z = q
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])

def q_angle_between(a: np.ndarray, b: np.ndarray) -> float:
    d = abs(np.dot(a, b))
    return 2 * np.arccos(np.clip(d, -1, 1))

def q_derivative(q: np.ndarray, w_body: np.ndarray) -> np.ndarray:
    return 0.5 * q_mul(q, np.array([0.0, *w_body]))

def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

# ============================================================================
# Motor model with torque-speed curve
# ============================================================================
@dataclass
class MotorParams:
    kt: float = 0.025            # torque constant [N*m/A]
    ke: float = 0.025            # back-EMF constant [V*s/rad] (=kt in SI)
    R: float = 1.2               # winding resistance [ohm]
    L: float = 0.4e-3            # inductance [H]
    V_bus: float = 12.0          # supply voltage [V]
    I_max: float = 2.0           # driver current limit [A]
    b_visc: float = 2e-6         # viscous friction [N*m*s/rad]
    tau_coulomb: float = 4e-4    # Coulomb friction [N*m]
    J_rotor: float = 2.5e-4      # wheel + rotor inertia [kg*m^2]
    omega_max: float = 600.0     # mechanical speed limit [rad/s] (~5700 rpm)

class Motor:
    """
    Brushed-DC / BLDC equivalent motor driven by a current-mode controller.

    The torque-speed envelope emerges naturally from the model:
      - Low speed: torque is current-limited  -> tau = kt * I_max
      - High speed: voltage-limited           -> I = (V_bus - ke*w)/R
      - Free speed: w_nl = V_bus / ke
    """
    def __init__(self, p: MotorParams):
        self.p = p
        self.omega = 0.0      # wheel speed relative to body [rad/s]
        self.current = 0.0    # winding current [A]

    def max_torque_at_speed(self, omega: float, direction: float) -> float:
        """Static torque-speed curve (motoring, in commanded direction)."""
        p = self.p
        v_avail = p.V_bus - direction * p.ke * omega   # headroom vs back-EMF
        i_avail = np.clip(v_avail / p.R, 0.0, p.I_max)
        return p.kt * i_avail

    def step(self, tau_cmd: float, dt: float) -> float:
        """
        Advance the motor electrical + mechanical state.
        tau_cmd: requested torque ON THE WHEEL [N*m].
        Returns the reaction torque applied to the BODY (= -motor torque).
        """
        p = self.p
        i_cmd = np.clip(tau_cmd / p.kt, -p.I_max, p.I_max)

        # Current loop: PI-like voltage demand, saturated by the bus
        v_demand = p.R * i_cmd + p.ke * self.omega + p.L * (i_cmd - self.current) / max(dt, 1e-6)
        v = np.clip(v_demand, -p.V_bus, p.V_bus)

        # Electrical dynamics (sub-stepped for stiffness: L/R ~ 0.33 ms)
        n_sub = max(1, int(np.ceil(dt / (p.L / p.R / 4))))
        h = dt / n_sub
        for _ in range(n_sub):
            di = (v - p.R * self.current - p.ke * self.omega) / p.L
            self.current += di * h
        self.current = np.clip(self.current, -p.I_max, p.I_max)

        tau_motor = p.kt * self.current
        tau_fric = p.b_visc * self.omega + p.tau_coulomb * np.tanh(self.omega / 0.5)
        tau_net = tau_motor - tau_fric

        # Hard speed limit (overspeed protection)
        if abs(self.omega) >= p.omega_max and np.sign(tau_net) == np.sign(self.omega):
            tau_net = 0.0
            tau_motor = tau_fric

        self.omega += tau_net / p.J_rotor * dt
        # Body feels the reaction to the *motor* torque (friction is internal too)
        return -tau_net

    @property
    def momentum(self) -> float:
        return self.p.J_rotor * self.omega

# ============================================================================
# Gyro model with drift
# ============================================================================
@dataclass
class GyroParams:
    # Values representative of a tactical-grade MEMS gyro
    bias_initial_sigma: float = np.deg2rad(0.05)          # turn-on bias [rad/s]
    bias_instability: float = np.deg2rad(5.0) / 3600      # [rad/s] (5 deg/hr)
    bias_corr_time: float = 300.0                         # Gauss-Markov tau [s]
    arw: float = np.deg2rad(0.15) / 60                    # ARW [rad/sqrt(s)] (0.15 deg/sqrt(hr))
    rrw: float = np.deg2rad(0.5) / 3600 / np.sqrt(3600)   # rate random walk
    scale_factor_ppm: float = 300.0
    misalignment_rad: float = 1e-3
    quantization: float = np.deg2rad(0.004)               # LSB [rad/s]
    full_scale: float = np.deg2rad(300.0)                 # saturation [rad/s]
    temp_coeff: float = np.deg2rad(0.002)                 # bias vs temp [rad/s/degC]

class Gyro:
    def __init__(self, p: GyroParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng
        self.bias_const = rng.normal(0, p.bias_initial_sigma, 3)
        self.bias_gm = np.zeros(3)        # first-order Gauss-Markov (flicker approx.)
        self.bias_rw = np.zeros(3)        # rate random walk
        sf = 1 + rng.normal(0, p.scale_factor_ppm * 1e-6, 3)
        mis = rng.normal(0, p.misalignment_rad, (3, 3))
        np.fill_diagonal(mis, 0.0)
        self.M = np.diag(sf) + mis        # scale factor + cross-axis matrix

    @property
    def total_bias(self) -> np.ndarray:
        return self.bias_const + self.bias_gm + self.bias_rw

    def measure(self, w_true: np.ndarray, dt: float, temp_c: float = 25.0) -> np.ndarray:
        p = self.p
        # Bias instability: Gauss-Markov process
        beta = 1.0 / p.bias_corr_time
        sigma_gm = p.bias_instability * np.sqrt(2 * beta * dt)
        self.bias_gm += -beta * self.bias_gm * dt + self.rng.normal(0, sigma_gm, 3)
        # Rate random walk
        self.bias_rw += self.rng.normal(0, p.rrw * np.sqrt(dt), 3)
        # Thermal bias
        bias_temp = p.temp_coeff * (temp_c - 25.0)
        # White noise (ARW) -> rate noise sigma = ARW / sqrt(dt)
        noise = self.rng.normal(0, p.arw / np.sqrt(dt), 3)

        w = self.M @ w_true + self.total_bias + bias_temp + noise
        w = np.clip(w, -p.full_scale, p.full_scale)
        return np.round(w / p.quantization) * p.quantization

# ============================================================================
# Rigid body
# ============================================================================
@dataclass
class RigidBody:
    I: np.ndarray                                   # 3x3 inertia tensor [kg*m^2]
    q: np.ndarray = field(default_factory=lambda: np.array([1.0, 0, 0, 0]))
    w: np.ndarray = field(default_factory=lambda: np.zeros(3))  # body rate [rad/s]

    def __post_init__(self):
        self.I_inv = np.linalg.inv(self.I)

    def derivs(self, q, w, tau_ext, h_wheels, tau_wheel_on_body):
        """
        Euler equation with internal momentum (reaction wheels):
          I*dw/dt = tau_ext + tau_wheels_on_body - w x (I*w + h_wheels)
        """
        H = self.I @ w + h_wheels
        dw = self.I_inv @ (tau_ext + tau_wheel_on_body - np.cross(w, H))
        dq = q_derivative(q, w)
        return dq, dw

    def step_rk4(self, dt, tau_ext, h_wheels, tau_wb):
        q0, w0 = self.q, self.w
        k1q, k1w = self.derivs(q0, w0, tau_ext, h_wheels, tau_wb)
        k2q, k2w = self.derivs(q0 + 0.5*dt*k1q, w0 + 0.5*dt*k1w, tau_ext, h_wheels, tau_wb)
        k3q, k3w = self.derivs(q0 + 0.5*dt*k2q, w0 + 0.5*dt*k2w, tau_ext, h_wheels, tau_wb)
        k4q, k4w = self.derivs(q0 + dt*k3q, w0 + dt*k3w, tau_ext, h_wheels, tau_wb)
        self.q = q_norm(q0 + dt/6 * (k1q + 2*k2q + 2*k3q + k4q))
        self.w = w0 + dt/6 * (k1w + 2*k2w + 2*k3w + k4w)

    def kinetic_energy(self) -> float:
        return 0.5 * self.w @ self.I @ self.w

# ============================================================================
# Disturbances
# ============================================================================
def disturbance_torque(t: float, q: np.ndarray) -> np.ndarray:
    """Small environmental torques: constant + orbital-periodic + gravity-gradient-like."""
    const = np.array([2e-6, -1.5e-6, 1e-6])
    periodic = 3e-6 * np.array([np.sin(2*np.pi*t/5400),
                                np.cos(2*np.pi*t/5400),
                                0.5*np.sin(2*np.pi*t/2700)])
    return const + periodic

# ============================================================================
# Flight software: estimator + controller
# ============================================================================
class AttitudeEstimator:
    """Gyro propagation with optional star-tracker resets and bias estimation."""
    def __init__(self, q0: np.ndarray, st_period: float | None, st_noise: float,
                 bias_gain: float, rng):
        self.q = q0.copy()
        self.bias_est = np.zeros(3)
        self.st_period = st_period
        self.st_noise = st_noise
        self.bias_gain = bias_gain
        self.rng = rng
        self.last_st = 0.0

    def propagate(self, w_meas: np.ndarray, dt: float) -> np.ndarray:
        w = w_meas - self.bias_est
        self.q = q_norm(q_mul(self.q, q_from_rotvec(w * dt)))
        return w

    def maybe_star_tracker(self, t: float, q_true: np.ndarray):
        if self.st_period is None or t - self.last_st < self.st_period:
            return
        dt_st = t - self.last_st
        self.last_st = t
        q_meas = q_norm(q_mul(q_true, q_from_rotvec(self.rng.normal(0, self.st_noise, 3))))
        dq = q_mul(q_conj(self.q), q_meas)
        if dq[0] < 0:
            dq = -dq
        err = 2 * dq[1:]                     # small-angle error in body frame
        self.q = q_norm(q_mul(self.q, q_from_rotvec(0.8 * err)))
        self.bias_est -= self.bias_gain * err / dt_st   # complementary bias update

class PDController:
    def __init__(self, I: np.ndarray, wn: float = 0.15, zeta: float = 0.9,
                 tau_limit: float = 0.04):
        self.Kp = I * wn**2
        self.Kd = I * 2 * zeta * wn
        self.tau_limit = tau_limit

    def command(self, q_est, w_est, q_target, h_wheels) -> np.ndarray:
        dq = q_mul(q_conj(q_target), q_est)
        if dq[0] < 0:
            dq = -dq
        e = 2 * dq