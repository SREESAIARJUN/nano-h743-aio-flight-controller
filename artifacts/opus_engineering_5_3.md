# 3‑Axis Rigid Body Physics Simulator (Cycle 5)

I'm treating "Cycle 5" as an iteration label. This is a complete, self‑contained build that covers the three requested areas:

1. **3‑axis rigid body mechanics**: full quaternion kinematics, Euler's equations with gyroscopic coupling, and reaction‑wheel momentum exchange, integrated with RK4.
2. **Motor torque curves**: a linear DC/BLDC torque‑speed envelope, an optional lookup‑table curve, current saturation, back‑EMF limiting, Coulomb and viscous friction, and torque ripple.
3. **Gyro drift**: bias instability (random walk), angle random walk (white noise), a Gauss‑Markov bias term, temperature‑dependent bias, scale factor error, axis misalignment, quantization and saturation.

The simulator also includes a quaternion PD attitude controller and an estimator that integrates the gyro. The estimator can optionally use star‑tracker corrections and online bias estimation, so you can watch drift accumulate or be corrected.

**Dependencies:** `numpy` (required) and `matplotlib` (optional, for plots).

```python
#!/usr/bin/env python3
"""
rigidsim.py - 3D rigid-body physics simulator (Cycle 5)

Models:
  * 3-axis rigid body dynamics (quaternion attitude, Euler's equations,
    gyroscopic coupling, reaction-wheel momentum exchange), RK4 integration
  * Motor torque-speed curves (linear DC envelope or lookup table),
    current limiting, friction (Coulomb + viscous), torque ripple
  * MEMS/FOG gyro error model: bias random walk, ARW white noise,
    Gauss-Markov bias, thermal bias, scale factor, misalignment,
    quantization, saturation
  * Attitude estimator (gyro propagation + optional star tracker w/ bias est.)
  * Quaternion PD attitude controller

Usage:
  python rigidsim.py                  # default slew scenario, plots if available
  python rigidsim.py --no-startracker # pure gyro integration -> visible drift
  python rigidsim.py --test           # run self-checks
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

# ---------------------------------------------------------------------------
# Quaternion utilities  (scalar-first: q = [w, x, y, z], body->inertial)
# ---------------------------------------------------------------------------

def q_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0, 0, 0])


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


def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-15 or abs(angle) < 1e-15:
        return np.array([1.0, 0, 0, 0])
    axis = axis / n
    s = math.sin(angle / 2)
    return np.array([math.cos(angle / 2), *(axis * s)])


def q_from_rotvec(v: np.ndarray) -> np.ndarray:
    ang = np.linalg.norm(v)
    return q_from_axis_angle(v, ang) if ang > 1e-15 else q_normalize(
        np.array([1.0, *(0.5 * v)]))


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """Rotation matrix mapping body vectors to inertial frame."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def q_to_euler(q: np.ndarray) -> np.ndarray:
    """ZYX Euler angles [roll, pitch, yaw] in rad."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def q_error_angle(q_a: np.ndarray, q_b: np.ndarray) -> float:
    """Smallest rotation angle between two attitudes (rad)."""
    dq = q_mul(q_conj(q_a), q_b)
    return 2.0 * math.acos(min(1.0, abs(dq[0])))


def omega_matrix(w: np.ndarray) -> np.ndarray:
    """q_dot = 0.5 * Omega(w) @ q for body-frame rates."""
    wx, wy, wz = w
    return np.array([
        [0, -wx, -wy, -wz],
        [wx, 0, wz, -wy],
        [wy, -wz, 0, wx],
        [wz, wy, -wx, 0],
    ])


# ---------------------------------------------------------------------------
# Motor model with torque-speed curve
# ---------------------------------------------------------------------------

@dataclass
class MotorParams:
    stall_torque: float = 0.05        # N*m at zero speed
    no_load_speed: float = 650.0      # rad/s (~6200 rpm)
    max_current: float = 2.0          # A
    torque_constant: float = 0.025    # N*m/A
    coulomb_friction: float = 2.0e-4  # N*m
    viscous_friction: float = 1.0e-6  # N*m/(rad/s)
    ripple_fraction: float = 0.02     # cogging/commutation ripple (fraction)
    ripple_poles: int = 6             # ripple cycles per revolution
    regen_braking: bool = True        # allow braking torque up to stall
    # Optional lookup-table curve: list of (speed rad/s, max torque N*m)
    curve_table: Optional[list] = None


class Motor:
    """DC/BLDC motor driving a reaction wheel, with a speed-dependent torque envelope."""

    def __init__(self, p: MotorParams):
        self.p = p
        if p.curve_table:
            tbl = np.array(sorted(p.curve_table), dtype=float)
            self._tbl_speed, self._tbl_torque = tbl[:, 0], tbl[:, 1]
        else:
            self._tbl_speed = self._tbl_torque = None

    def envelope(self, speed: float) -> float:
        """Max motoring torque magnitude available at |speed|."""
        s = abs(speed)
        if self._tbl_speed is not None:
            return float(np.interp(s, self._tbl_speed, self._tbl_torque,
                                   right=0.0))
        # Linear DC torque-speed line (back-EMF limited)
        return max(0.0, self.p.stall_torque * (1.0 - s / self.p.no_load_speed))

    def friction(self, speed: float) -> float:
        # Smooth sign via tanh avoids stiff chatter at zero crossing
        return (self.p.coulomb_friction * math.tanh(speed / 0.5)
                + self.p.viscous_friction * speed)

    def ripple(self, angle: float) -> float:
        return 1.0 + self.p.ripple_fraction * math.sin(self.p.ripple_poles * angle)

    def electromagnetic_torque(self, cmd: float, speed: float, angle: float) -> float:
        """Achievable electromagnetic torque given command and current wheel speed."""
        # Current limit
        i_lim = self.p.max_current * self.p.torque_constant
        cmd = max(-i_lim, min(i_lim, cmd))
        motoring = cmd * speed >= 0.0  # torque in the direction of rotation
        if motoring:
            lim = self.envelope(speed)
        else:
            lim = self.p.stall_torque if self.p.regen_braking else self.envelope(speed)
        tau = max(-lim, min(lim, cmd))
        return tau * self.ripple(angle)

    def net_torque(self, cmd: float, speed: float, angle: float) -> float:
        """Net torque accelerating the wheel (EM torque minus friction)."""
        return self.electromagnetic_torque(cmd, speed, angle) - self.friction(speed)


# ---------------------------------------------------------------------------
# Gyro error model
# ---------------------------------------------------------------------------

@dataclass
class GyroParams:
    arw: float = 0.15 * math.pi / 180 / 60          # rad/sqrt(s) (0.15 deg/sqrt(hr))
    rrw: float = 3.0 * math.pi / 180 / 3600 / 60    # rad/s/sqrt(s) bias random walk
    bias_initial_sigma: float = 10 * math.pi / 180 / 3600  # rad/s (10 deg/hr)
    gm_sigma: float = 2 * math.pi / 180 / 3600      # Gauss-Markov bias steady-state
    gm_tau: float = 300.0                           # s correlation time
    thermal_coeff: float = 0.5 * math.pi / 180 / 3600  # rad/s per degC
    scale_factor_ppm: float = 300.0
    misalignment_rad: float = 1e-3
    quantization: float = 1e-6                      # rad/s LSB
    saturation: float = math.radians(400.0)        # rad/s full scale
    sample_rate: float = 100.0                      # Hz


class Gyro:
    def __init__(self, p: GyroParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng
        self.bias_rw = rng.normal(0, p.bias_initial_sigma, 3)
        self.bias_gm = np.zeros(3)
        sf = rng.normal(0, p.scale_factor_ppm * 1e-6, 3)
        mis = rng.normal(0, p.misalignment_rad, (3, 3))
        np.fill_diagonal(mis, 0.0)
        self.M = np.diag(1.0 + sf) + mis  # scale factor + misalignment matrix
        self.temperature = 25.0
        self.ref_temperature = 25.0

    @property
    def total_bias(self) -> np.ndarray:
        thermal = self.p.thermal_coeff * (self.temperature - self.ref_temperature)
        return self.bias_rw + self.bias_gm + thermal

    def measure(self, omega_true: np.ndarray, dt: float) -> np.ndarray:
        p = self.p
        # Bias random walk (rate random walk)
        self.bias_rw += p.rrw * math.sqrt(dt) * self.rng.standard_normal(3)
        # First-order Gauss-Markov bias
        a = math.exp(-dt / p.gm_tau)
        self.bias_gm = a * self.bias_gm + p.gm_sigma * math.sqrt(1 - a * a) \
            * self.rng.standard_normal(3)
        # Angle random walk -> white rate noise of std arw/sqrt(dt)
        noise = p.arw / math.sqrt(dt) * self.rng.standard_normal(3)
        meas = self.M @ omega_true + self.total_bias + noise
        meas = np.clip(meas, -p.saturation, p.saturation)
        if p.quantization > 0:
            meas = np.round(meas / p.quantization) * p.quantization
        return meas


# ---------------------------------------------------------------------------
# Star tracker (optional absolute attitude reference)
# ---------------------------------------------------------------------------

@dataclass
class StarTrackerParams:
    enabled: bool = True
    rate_hz: float = 2.0
    noise_rad: float = 20e-6 * 5  # ~20 arcsec


class StarTracker:
    def __init__(self, p: StarTrackerParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng

    def measure(self, q_true: np.ndarray) -> np.ndarray:
        dq = q_from_rotvec(self.rng.normal(0, self.p.noise_rad, 3))
        return q_normalize(q_mul(q_true, dq))


# ---------------------------------------------------------------------------
# Attitude estimator: gyro propagation + complementary correction w/ bias est.
# ---------------------------------------------------------------------------

class AttitudeEstimator:
    def __init__(self, q0: np.ndarray, kp: float = 0.5, ki: float = 0.02):
        self.q = q_normalize(q0.copy())
        self.bias = np.zeros(3)
        self.kp = kp
        self.ki = ki

    def propagate(self, gyro_meas: np.ndarray, dt: float):
        w = gyro_meas - self.bias
        self.q = q_normalize(q_mul(self.q, q_from_rotvec(w * dt)))

    def correct(self, q_meas: np.ndarray, dt_since: float):
        dq = q_mul(q_conj(self.q), q_meas)
        if dq[0] < 0:
            dq = -dq
        err = 2.0 * dq[1:]  # small-angle error vector (body frame)
        self.q = q_normalize(q_mul(self.q, q_from_rotvec(self.kp * err)))
        self.bias -= self.ki * err / max(dt_since, 1e-6)

    @property
    def rate_estimate_fn(self):
        return lambda meas: meas - self.bias


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class QuaternionPD:
    def __init__(self, inertia: np.ndarray, wn: float = 0.15, zeta: float = 0.9,
                 max_rate: float = math.radians(2.0)):
        self.Kp = inertia * wn ** 