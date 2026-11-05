# 3-Axis Rigid Body Simulator: Reaction Wheels and Gyro Drift (Cycle 12)

I don't have context on what earlier cycles contained, so this is a complete, self-contained build. It covers three areas:

- **Rigid body mechanics**: full 3-axis attitude dynamics using Euler's equations with gyroscopic coupling and quaternion kinematics, integrated with RK4.
- **Motor torque curves**: brushless DC reaction-wheel models with back-EMF, current limiting, voltage saturation, and Coulomb plus viscous friction.
- **Gyro drift**: a MEMS/FOG-style gyro model with angle random walk, rate random walk, Gauss-Markov bias instability, temperature-dependent bias, scale-factor error, axis misalignment, quantization, and range limits.

A closed-loop PD controller steers the spacecraft using the *gyro-propagated* attitude estimate. You can therefore watch gyro drift turn into true pointing error over time.

Requirements: `numpy`, and optionally `matplotlib` for plots.

```python
#!/usr/bin/env python3
"""
rigidsim3d.py - 3D rigid body physics simulator (Cycle 12)

Models:
  * 3-axis rigid body rotational dynamics (Euler's equations + quaternion kinematics)
  * Reaction wheel motors with DC torque-speed curves, current limits, friction
  * Gyroscope with bias drift (ARW, RRW, bias instability, thermal, scale factor,
    misalignment, quantization, saturation)
  * PD attitude controller driven by gyro-integrated attitude estimate

Usage:
  python rigidsim3d.py                       # default slew simulation + plots
  python rigidsim3d.py --duration 600 --csv out.csv --no-plot
  python rigidsim3d.py --selftest            # conservation-law checks
  python rigidsim3d.py --torque-curve        # print/plot motor torque curve
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field

import numpy as np

# ============================================================================
# Quaternion utilities (scalar-first convention: q = [w, x, y, z])
# q rotates vectors from BODY frame to INERTIAL frame.
# ============================================================================


def q_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-15:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


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


def q_from_axis_angle(axis, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle_rad / 2.0)
    return np.array([math.cos(angle_rad / 2.0), *(axis * s)])


def q_exp_rotvec(rv: np.ndarray) -> np.ndarray:
    """Quaternion from a rotation vector (axis * angle)."""
    angle = np.linalg.norm(rv)
    if angle < 1e-12:
        return q_normalize(np.array([1.0, *(0.5 * rv)]))
    return q_from_axis_angle(rv / angle, angle)


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """Rotation matrix body->inertial."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def q_to_euler_zyx(q: np.ndarray) -> np.ndarray:
    """Return roll, pitch, yaw (rad)."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def q_angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Smallest rotation angle (rad) between two attitudes."""
    d = abs(float(np.dot(q_normalize(a), q_normalize(b))))
    return 2.0 * math.acos(min(1.0, d))


def omega_matrix(w: np.ndarray) -> np.ndarray:
    """Matrix so that q_dot = 0.5 * Omega(w) @ q (w in body frame)."""
    wx, wy, wz = w
    return np.array([
        [0.0, -wx, -wy, -wz],
        [wx, 0.0, wz, -wy],
        [wy, -wz, 0.0, wx],
        [wz, wy, -wx, 0.0],
    ])


# ============================================================================
# Motor / reaction wheel model
# ============================================================================


@dataclass
class MotorParams:
    Kt: float = 0.025          # torque constant [N*m/A]
    Ke: float = 0.025          # back-EMF constant [V*s/rad] (=Kt in SI)
    R: float = 1.2             # winding resistance [ohm]
    V_max: float = 12.0        # supply voltage [V]
    I_max: float = 2.0         # current limit [A]
    J_wheel: float = 2.0e-4    # wheel rotor inertia [kg*m^2]
    coulomb: float = 1.0e-4    # Coulomb friction [N*m]
    viscous: float = 2.0e-7    # viscous friction [N*m/(rad/s)]
    stribeck_w: float = 2.0    # smoothing speed for Coulomb friction [rad/s]
    max_speed: float = 600.0   # hard overspeed protection [rad/s] (~5700 rpm)


class ReactionWheel:
    """
    DC motor torque-speed curve:
        i   = clip((V - Ke*Omega)/R, -I_max, I_max)
        tau = Kt * i
    The curve is linear between stall torque (Kt*V/R, capped by Kt*I_max)
    and no-load speed (V/Ke). Friction opposes wheel rotation.
    """

    def __init__(self, params: MotorParams, axis):
        self.p = params
        a = np.asarray(axis, dtype=float)
        self.axis = a / np.linalg.norm(a)
        self.voltage = 0.0  # zero-order-hold command

    def current(self, V: float, omega: float) -> float:
        p = self.p
        i = (V - p.Ke * omega) / p.R
        return float(np.clip(i, -p.I_max, p.I_max))

    def motor_torque(self, V: float, omega: float) -> float:
        tau = self.p.Kt * self.current(V, omega)
        # Overspeed protection: refuse to accelerate further past the limit
        if abs(omega) >= self.p.max_speed and np.sign(tau) == np.sign(omega):
            tau = 0.0
        return tau

    def friction(self, omega: float) -> float:
        p = self.p
        # Smooth (tanh) Coulomb friction avoids chattering at zero speed
        return p.coulomb * math.tanh(omega / p.stribeck_w) + p.viscous * omega

    def net_torque(self, omega: float) -> float:
        """Net torque on the wheel rotor (body feels the negative of this)."""
        return self.motor_torque(self.voltage, omega) - self.friction(omega)

    def voltage_for_torque(self, tau_des: float, omega: float) -> float:
        """Invert the motor model (with friction feed-forward) to get voltage."""
        p = self.p
        tau_total = tau_des + self.friction(omega)
        i = np.clip(tau_total / p.Kt, -p.I_max, p.I_max)
        V = i * p.R + p.Ke * omega
        return float(np.clip(V, -p.V_max, p.V_max))

    def torque_curve(self, V: float | None = None, n: int = 50):
        """Return (speeds, torques) for the motor's torque-speed curve."""
        V = self.p.V_max if V is None else V
        no_load = V / self.p.Ke
        speeds = np.linspace(0.0, no_load, n)
        torques = np.array([self.p.Kt * self.current(V, w) - self.friction(w)
                            for w in speeds])
        return speeds, torques


# ============================================================================
# Gyroscope model with drift
# ============================================================================


@dataclass
class GyroParams:
    arw: float = 0.15 * math.pi / 180 / 60           # angle random walk [rad/sqrt(s)] (0.15 deg/sqrt(hr))
    rrw: float = 0.5 * math.pi / 180 / 3600 / 60     # rate random walk [rad/s/sqrt(s)]
    bias_instability: float = 3.0 * math.pi / 180 / 3600   # [rad/s] (3 deg/hr)
    bias_corr_time: float = 300.0                    # Gauss-Markov tau [s]
    turn_on_bias_sigma: float = 10.0 * math.pi / 180 / 3600  # [rad/s]
    thermal_coeff: float = 0.05 * math.pi / 180 / 3600  # bias per deg C [rad/s/C]
    scale_factor_ppm: float = 300.0                  # 1-sigma scale factor error
    misalignment_rad: float = 0.5e-3                 # 1-sigma axis misalignment
    quantization: float = 1.0e-6                     # LSB [rad/s]
    range_max: float = math.radians(300.0)           # saturation [rad/s]
    T_ref: float = 25.0                              # reference temperature [C]


class Gyro:
    """
    measured = (I + S + M) @ omega_true + b_on + b_rw + b_gm + b_T(T) + n_arw
      S     : diagonal scale-factor errors
      M     : off-diagonal misalignment
      b_on  : fixed turn-on bias
      b_rw  : rate random walk (integrated white noise)
      b_gm  : first-order Gauss-Markov bias instability
      b_T   : linear thermal bias sensitivity
      n_arw : white rate noise (angle random walk)
    """

    def __init__(self, params: GyroParams, rng: np.random.Generator):
        self.p = params
        self.rng = rng
        S = np.diag(rng.normal(0.0, params.scale_factor_ppm * 1e-6, 3))
        M = rng.normal(0.0, params.misalignment_rad, (3, 3))
        np.fill_diagonal(M, 0.0)
        self.C = np.eye(3) + S + M
        self.b_on = rng.normal(0.0, params.turn_on_bias_sigma, 3)
        self.b_rw = np.zeros(3)
        self.b_gm = np.zeros(3)
        self.thermal_dir = rng.normal(0.0, 1.0, 3)

    def total_bias(self, temp_c: float) -> np.ndarray:
        b_T = self.p.thermal_coeff * (temp_c - self.p.T_ref) * self.thermal_dir
        return self.b_on + self.b_rw + self.b_gm + b_T

    def measure(self, omega_true: np.ndarray, dt: float, temp_c: float) -> np.ndarray:
        p, rng = self.p, self.rng
        # Rate random walk
        self.b_rw += rng.normal(0.0, p.rrw * math.sqrt(dt), 3)
        # Gauss-Markov bias instability (exact discretization)
        phi = math.exp(-dt / p.bias_corr_time)
        q_std = p.bias_instability * math.sqrt(1.0 - phi * phi)
        self.b_gm = phi * self.b_gm + rng.normal(0.0, q_std, 3)
        # White rate noise (ARW): sigma_rate = N / sqrt(