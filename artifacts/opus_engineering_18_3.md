# 3D Rigid Body Physics Simulator: Reaction Wheels, Motor Torque Curves, and Gyro Drift

I'm treating "Cycle 18" as a version label for this build. This is a self-contained Python simulator. It needs only `numpy`, and `matplotlib` is optional for plots. It models:

- **3-axis rigid body dynamics.** It uses Euler's equations with a full inertia tensor, gyroscopic coupling, and reaction-wheel momentum exchange. The state is integrated with RK4 and the quaternion is renormalized after each step.
- **Motor torque curves.** Each wheel motor has a brushless DC motor model with:
  - a linear torque-speed envelope (stall torque to no-load speed),
  - current and power limits,
  - back-EMF,
  - Coulomb plus viscous friction,
  - a first-order torque response lag,
  - electrical power and energy accounting.
- **Gyro drift.** Each gyro axis models:
  - angle random walk (white noise),
  - bias instability (random walk),
  - temperature-dependent bias,
  - scale-factor error,
  - axis misalignment,
  - quantization and saturation.
- **Closed-loop control.** A quaternion PD controller acts on the *estimated* attitude, which comes from integrating the gyro. Over time the true attitude drifts away from the commanded one. An optional star-tracker correction can be enabled to show the difference.

```python
#!/usr/bin/env python3
"""
rigid3d_sim.py  —  Cycle 18
3-axis rigid-body spacecraft/vehicle simulator with reaction-wheel motors,
torque-speed curves, and realistic gyro error modelling.

Usage:
    python rigid3d_sim.py                 # default 600 s run, prints summary
    python rigid3d_sim.py --plot          # with matplotlib plots
    python rigid3d_sim.py --star-tracker  # enable periodic attitude correction
    python rigid3d_sim.py --duration 3600 --dt 0.01 --seed 7
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

import numpy as np

# ============================================================================
# Quaternion utilities  (scalar-first convention: q = [w, x, y, z])
# Quaternion maps body -> inertial frame.
# ============================================================================

def q_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0.0, 0.0, 0.0])


def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), *(axis * s)])


def q_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX (yaw-pitch-roll) Euler angles -> quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def q_to_euler(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def q_deriv(q: np.ndarray, omega_body: np.ndarray) -> np.ndarray:
    """dq/dt = 0.5 * q ⊗ [0, ω]."""
    return 0.5 * q_mul(q, np.array([0.0, *omega_body]))


def q_angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Smallest rotation angle (rad) between two attitudes."""
    d = abs(float(np.dot(q_normalize(a), q_normalize(b))))
    return 2.0 * math.acos(min(1.0, d))


def q_integrate_exact(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
    """Closed-form propagation assuming constant ω over dt (used by estimator)."""
    angle = np.linalg.norm(omega) * dt
    if angle < 1e-12:
        dq = np.array([1.0, *(0.5 * omega * dt)])
    else:
        dq = q_from_axis_angle(omega, angle)
    return q_normalize(q_mul(q, dq))


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


# ============================================================================
# Motor model with torque-speed curve
# ============================================================================

@dataclass
class MotorParams:
    stall_torque: float = 0.20        # N·m at zero speed (rated envelope)
    no_load_speed: float = 6000 * 2 * math.pi / 60   # rad/s
    kt: float = 0.03                  # N·m/A torque constant
    ke: float = 0.03                  # V·s/rad back-EMF constant (== kt in SI)
    resistance: float = 1.2           # Ω winding resistance
    bus_voltage: float = 28.0         # V
    max_current: float = 6.0          # A (driver limit)
    max_power: float = 120.0          # W electrical input limit
    coulomb_friction: float = 0.0015  # N·m
    viscous_friction: float = 2.0e-6  # N·m/(rad/s)
    torque_time_const: float = 0.02   # s (current-loop / electrical lag)
    rotor_inertia: float = 0.012      # kg·m² (wheel + rotor)


class Motor:
    """
    Brushless DC motor driving a reaction wheel.

    Torque-speed envelope (motoring quadrant):
        τ_avail(Ω) = min( τ_stall·(1 - |Ω|/Ω_nl),  kt·I_max,  P_max/|Ω| )
    The voltage-limited line comes from V = I·R + ke·Ω.
    When decelerating (regenerating) the back-EMF assists, so the
    available braking torque is only limited by current.
    """

    def __init__(self, p: MotorParams, name: str = "motor"):
        self.p = p
        self.name = name
        self.speed = 0.0          # wheel speed rad/s (relative to body)
        self.torque = 0.0         # actual electromagnetic torque N·m
        self.current = 0.0
        self.power = 0.0
        self.energy = 0.0         # J consumed
        self.saturated = False

    def torque_limits(self, speed: float) -> tuple[float, float]:
        """Return (min, max) achievable electromagnetic torque at given speed."""
        p = self.p
        i_lim = p.kt * p.max_current
        # Voltage-limited torque: I = (V - ke·Ω)/R
        tau_v_pos = p.kt * (p.bus_voltage - p.ke * speed) / p.resistance
        tau_v_neg = p.kt * (-p.bus_voltage - p.ke * speed) / p.resistance
        # Linear rated envelope
        frac = max(0.0, 1.0 - abs(speed) / p.no_load_speed)
        env = p.stall_torque * frac
        # Power limit (only meaningful when motoring)
        pw = p.max_power / max(abs(speed), 1e-3)

        if speed >= 0:
            t_max = min(i_lim, tau_v_pos, env if env > 0 else 0.0, pw)
            t_min = max(-i_lim, tau_v_neg)          # braking: current-limited
        else:
            t_max = min(i_lim, tau_v_pos)           # braking
            t_min = max(-i_lim, tau_v_neg, -(env if env > 0 else 0.0), -pw)
        return min(t_min, 0.0), max(t_max, 0.0)

    def friction(self, speed: float) -> float:
        p = self.p
        if abs(speed) < 1e-6:
            return 0.0
        return math.copysign(p.coulomb_friction, speed) + p.viscous_friction * speed

    def update(self, torque_cmd: float, dt: float) -> float:
        """Advance electrical state; returns net torque on the wheel (N·m)."""
        p = self.p
        t_min, t_max = self.torque_limits(self.speed)
        cmd = min(max(torque_cmd, t_min), t_max)
        self.saturated = not math.isclose(cmd, torque_cmd, abs_tol=1e-9)

        # First-order lag on torque
        alpha = 1.0 - math.exp(-dt / max(p.torque_time_const, 1e-6))
        self.torque += alpha * (cmd - self.torque)

        self.current = self.torque / p.kt
        voltage = self.current * p.resistance + p.ke * self.speed
        self.power = voltage * self.current
        self.energy += max(self.power, 0.0) * dt   # no regen credit

        return self.torque - self.friction(self.speed)


# ============================================================================
# Gyro model with drift
# ============================================================================

@dataclass
class GyroParams:
    arw: float = math.radians(0.15) / 60.0          # rad/√s  (0.15 °/√hr)
    bias_instability: float = math.radians(1.0) / 3600.0  # rad/s (1 °/hr)
    bias_corr_time: float = 300.0                   # s (Gauss-Markov)
    rate_random_walk: float = math.radians(0.01) / 3600.0  # rad/s/√s
    initial_bias_sigma: float = math.radians(3.0) / 3600.0  # rad/s turn-on bias
    temp_coeff: float = math.radians(0.05) / 3600.0  # rad/s per °C
    temp_ref: float = 20.0                          # °C
    scale_factor_sigma: float = 300e-6              # 300 ppm
    misalignment_sigma: float = 1e-4                # rad
    quantization: float = 1e-6                      # rad/s LSB
    max_rate: float = math.radians(400.0)           # rad/s saturation


class Gyro:
    """
    3-axis rate gyro.
        ω_meas = (I + S + M)·ω_true + b_GM + b_RW + b_0 + k_T·(T - T0) + n_ARW
    """

    def __init__(self, p: GyroParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng
        self.b0 = rng.normal(0, p.initial_bias_sigma, 3)
        self.b_gm = np.zeros(3)          # Gauss-Markov bias instability
        self.b_rw = np.zeros(3)          # rate random walk
        S = np.diag(rng.normal(0, p.scale_factor_sigma, 3))
        M = rng.normal(0, p.misalignment_sigma, (3, 3))
        np.fill_diagonal(M, 0.0)
        self.error_matrix = np.eye(3) + S + M
        self.temperature = p.temp_ref

    @property
    def total_bias(self) -> np.ndarray:
        return (self.b0 + self.b_gm + self.b_rw
                + self.p.temp_coeff * (self.temperature - self.p.temp_ref))

    def measure(self, omega_true: np.ndarray, dt: float) -> np.ndarray:
        p = self.p
        # Gauss-Markov bias instability
        beta = 1.0 / p.bias_corr_time
        phi = math.exp(-beta * dt)
        q_gm = p.bias_instability * math.sqrt(1 - phi * phi)
        self.b_gm = phi * self.b_gm + self.rng.normal(0, q_gm, 3)
        # Rate random walk
        self.b_rw += self.rng.normal(0, p.rate_random_walk * math.sqrt(dt), 3)
        # Angle random walk -> white rate noise
        n = self.rng.normal(0, p.arw / math.sqrt(dt), 3)

        w = self.error_matrix @ omega_true + self.total_bias + n
        w = np.clip(w, -p.max_rate, p.max_rate)
        if p.quantization > 0:
            w = np.round(w / p.quantization) * p.quantization
        return w


# ============================================================================
# Rigid body with reaction wheels
# ============================================================================

@dataclass
class BodyParams:
    inertia: np.ndarray = field(default_factory=lambda: np.array([
        [12.0, 0.15, -0.10],
        [0.15, 9.5, 0.08],
        [-0.10, 0.08, 14.0]]))                   # kg·m² (body frame)
    # Wheel spin axes in body frame (columns), here aligned with body axes
    wheel_axes: np.ndarray = field(default_factory=lambda: np.eye(3))


class RigidBody:
    """
    State: q (body->inertial), ω (body rate, body frame), Ω (wheel speeds).
    Dynamics:
        h_w   = A · (J_w Ω)                     wheel angular momentum (body)
        I ω̇  = -ω × (I ω + h_w) - A τ_w + τ_ext
        J_w Ω̇ = τ_w                              (τ_w = net torque on wheels)
    """

    def __init__(self, bp: BodyParams, motors: list[Motor]):
        self.I = np.array(bp.inertia, dtype=float)
        self.I_inv = np.linalg.inv(self.I)
        self.