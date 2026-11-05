# 3-Axis Rigid Body Attitude Simulator

This is a complete, self-contained Python 3D attitude dynamics simulator. It models:

- **Rigid body mechanics**: full Euler equations with gyroscopic coupling, quaternion kinematics and RK4 integration.
- **Reaction wheel motors**: a DC motor torque/speed curve with back-EMF, current limiting, regenerative braking, Coulomb and viscous friction, and speed saturation.
- **Gyro errors**: bias instability as a random walk, angle random walk (ARW), scale factor error, axis misalignment, temperature-dependent bias and quantization.
- **Estimation and control**: a gyro-propagated attitude estimator with star tracker corrections and online bias estimation, plus a PD slew controller with gyroscopic feed-forward.
- **Disturbances**: constant, orbital-periodic and random torques.

**Requirements:** `numpy` is required. `matplotlib` is optional and only needed for plots.

```python
#!/usr/bin/env python3
"""
rigidbody_sim.py  -  3-axis rigid body attitude simulator (Cycle 26)

Models:
  * Rigid body dynamics (Euler's equations w/ reaction-wheel momentum coupling)
  * Quaternion kinematics, RK4 integration
  * Reaction wheel DC motors: torque-speed curve, current limit, back-EMF,
    regenerative braking, Coulomb + viscous friction
  * MEMS/FOG gyro: bias random walk, ARW, scale factor, misalignment,
    thermal bias drift, quantization
  * Star tracker + complementary attitude/bias estimator
  * PD attitude controller with gyroscopic feed-forward

Usage:
  python rigidbody_sim.py --tf 300 --dt 0.01 --plot --csv out.csv
"""
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field

import numpy as np


# ----------------------------------------------------------------------------
# Quaternion utilities (scalar-first: q = [w, x, y, z], body->inertial)
# ----------------------------------------------------------------------------
def qmul(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw * qw - px * qx - py * qy - pz * qz,
        pw * qx + px * qw + py * qz - pz * qy,
        pw * qy - px * qz + py * qw + pz * qx,
        pw * qz + px * qy - py * qx + pz * qw,
    ])


def qconj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qnorm(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0, 0, 0])


def qfrom_rotvec(v: np.ndarray) -> np.ndarray:
    ang = np.linalg.norm(v)
    if ang < 1e-12:
        return qnorm(np.array([1.0, 0.5 * v[0], 0.5 * v[1], 0.5 * v[2]]))
    ax = v / ang
    s = math.sin(ang / 2)
    return np.array([math.cos(ang / 2), ax[0] * s, ax[1] * s, ax[2] * s])


def qfrom_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX Euler angles (rad) -> quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def qto_euler(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def qerror_vec(q_ref: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Small-angle rotation vector of q relative to q_ref (body frame), rad."""
    qe = qmul(qconj(q_ref), q)
    if qe[0] < 0:          # shortest path
        qe = -qe
    return 2.0 * qe[1:]


def qangle_deg(q_ref: np.ndarray, q: np.ndarray) -> float:
    qe = qmul(qconj(q_ref), q)
    return math.degrees(2 * math.acos(min(1.0, abs(qe[0]))))


# ----------------------------------------------------------------------------
# Reaction wheel motor model
# ----------------------------------------------------------------------------
@dataclass
class MotorParams:
    Jw: float = 2.0e-3          # wheel spin inertia [kg m^2]
    Kt: float = 0.025           # torque constant [N m / A]
    Ke: float = 0.025           # back-EMF constant [V s / rad]
    R: float = 1.2              # winding resistance [ohm]
    V_bus: float = 28.0         # bus voltage [V]
    I_max: float = 4.0          # current limit [A]
    c_visc: float = 2.0e-6      # viscous friction [N m s / rad]
    c_coul: float = 1.0e-4      # Coulomb friction [N m]
    w_max: float = 6000 * 2 * math.pi / 60  # software speed limit [rad/s]
    tau_lag: float = 0.02       # current loop time constant [s]


class ReactionWheel:
    """Brushless DC reaction wheel with a realistic torque-speed envelope."""

    def __init__(self, p: MotorParams):
        self.p = p
        self.omega = 0.0          # wheel speed relative to body [rad/s]
        self.torque = 0.0         # actual motor electromagnetic torque [N m]
        self.current = 0.0

    def torque_limits(self, omega: float) -> tuple[float, float]:
        """Return (min, max) achievable motor torque at speed omega.

        Motoring (torque same sign as speed) is limited by back-EMF:
            tau <= Kt*(V - Ke|w|)/R
        Braking (torque opposite speed) is aided by back-EMF:
            tau <= Kt*(V + Ke|w|)/R
        Both are clipped by the current limit.
        """
        p = self.p
        t_cur = p.Kt * p.I_max
        motoring = max(0.0, p.Kt * (p.V_bus - p.Ke * abs(omega)) / p.R)
        braking = p.Kt * (p.V_bus + p.Ke * abs(omega)) / p.R
        if omega >= 0:
            return -min(t_cur, braking), min(t_cur, motoring)
        return -min(t_cur, motoring), min(t_cur, braking)

    def friction(self, omega: float) -> float:
        p = self.p
        # tanh smooths the Coulomb discontinuity for integrator stability
        return -p.c_visc * omega - p.c_coul * math.tanh(omega / 0.5)

    def command(self, tau_cmd: float, dt: float) -> None:
        """Apply commanded torque through the current loop and envelope."""
        p = self.p
        # Speed protection: refuse to accelerate past w_max
        if abs(self.omega) >= p.w_max and tau_cmd * self.omega > 0:
            tau_cmd = 0.0
        lo, hi = self.torque_limits(self.omega)
        tau_cmd = min(max(tau_cmd, lo), hi)
        a = dt / (p.tau_lag + dt)      # first-order current loop lag
        self.torque += a * (tau_cmd - self.torque)
        self.current = self.torque / p.Kt

    def net_torque(self, omega: float) -> float:
        """Net torque accelerating the wheel (motor + friction)."""
        return self.torque + self.friction(omega)

    def power(self) -> float:
        p = self.p
        return self.current ** 2 * p.R + self.torque * self.omega


# ----------------------------------------------------------------------------
# Gyro model
# ----------------------------------------------------------------------------
@dataclass
class GyroParams:
    arw: float = math.radians(0.15) / 60          # angle random walk [rad/sqrt(s)]
    rrw: float = math.radians(0.5) / 3600 / 30    # bias random walk [rad/s/sqrt(s)]
    bias0_sigma: float = math.radians(10) / 3600  # turn-on bias [rad/s]
    scale_sigma: float = 300e-6                   # scale factor error [-]
    misalign_sigma: float = math.radians(0.05)    # axis misalignment [rad]
    temp_coeff: float = math.radians(0.02) / 3600 # bias per degC [rad/s/C]
    quant: float = math.radians(0.001)            # angle quantum [rad]
    bias_max: float = math.radians(50) / 3600     # random walk clamp


class Gyro:
    def __init__(self, p: GyroParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng
        self.bias = rng.normal(0, p.bias0_sigma, 3)
        sf = rng.normal(0, p.scale_sigma, 3)
        m = rng.normal(0, p.misalign_sigma, (3, 3))
        np.fill_diagonal(m, 0.0)
        self.M = np.eye(3) + np.diag(sf) + m
        self.temp_axis = rng.normal(0, 1, 3)
        self._angle_residual = np.zeros(3)

    def measure(self, omega_true: np.ndarray, dt: float, temp_c: float) -> np.ndarray:
        p = self.p
        # Bias random walk (bias instability)
        self.bias += p.rrw * math.sqrt(dt) * self.rng.standard_normal(3)
        self.bias = np.clip(self.bias, -p.bias_max, p.bias_max)
        thermal = p.temp_coeff * (temp_c - 20.0) * self.temp_axis
        # White rate noise -> angle random walk
        noise = p.arw / math.sqrt(dt) * self.rng.standard_normal(3)
        rate = self.M @ omega_true + self.bias + thermal + noise
        # Quantization on integrated angle (delta-theta output)
        if p.quant > 0:
            dtheta = rate * dt + self._angle_residual
            q = np.round(dtheta / p.quant) * p.quant
            self._angle_residual = dtheta - q
            rate = q / dt
        return rate


# ----------------------------------------------------------------------------
# Star tracker + attitude estimator
# ----------------------------------------------------------------------------
class StarTracker:
    def __init__(self, rng, sigma_cross=math.radians(5 / 3600),
                 sigma_roll=math