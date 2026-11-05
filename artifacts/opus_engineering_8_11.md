# 3-Axis Rigid-Body Attitude Simulator in Python

I don't know what "Cycle 8" refers to in your project, so this is a complete, self-contained simulator built from scratch. The modules are independent, so you can swap any of them out to fit your existing codebase.

## Features

- **Rigid-body dynamics:** full 3D Euler equations with a quaternion attitude state, integrated with RK4.
- **Reaction-wheel actuators:**
  - DC/BLDC electrical model (back-EMF, winding resistance, current limit, bus-voltage limit).
  - Tabulated torque-speed envelope (the motor torque curve).
  - Coulomb and viscous bearing friction.
  - Speed saturation.
- **Gyro error model:**
  - Angle random walk (ARW).
  - Rate random walk.
  - Gauss-Markov bias instability.
  - Turn-on bias and temperature-dependent bias drift.
  - Scale-factor error and axis misalignment.
  - Quantization and range saturation.
- **Disturbances:** gravity-gradient torque plus a constant/sinusoidal environmental torque.
- **Guidance, navigation and control:**
  - Gyro-integrated attitude estimator.
  - Optional star-tracker fusion using a Mahony-style complementary filter with bias estimation.
  - Quaternion PD controller with gyroscopic compensation.
  - Wheel torque allocation for any number of wheels (pseudo-inverse).
- **Output:** CSV logging, a summary report and optional matplotlib plots.

## Source: `attitude_sim.py`

```python
#!/usr/bin/env python3
"""
attitude_sim.py — 3-axis rigid-body attitude simulator
  * Rigid-body dynamics (Euler's equations, quaternion kinematics, RK4)
  * Reaction-wheel motors with electrical model + torque-speed curve + friction
  * Gyro error model (ARW, RRW, bias instability, temperature drift, SF, misalignment)
  * Estimator (gyro propagation + optional star-tracker complementary filter)
  * Quaternion PD attitude controller

Dependencies: numpy (required), matplotlib (optional, for --plot)
Usage:
  python attitude_sim.py --duration 300 --dt 0.01 --plot --csv out.csv
  python attitude_sim.py --no-star-tracker --plot      # watch gyro drift accumulate
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field

import numpy as np


# =============================================================================
# Quaternion utilities  (scalar-first [w, x, y, z], body -> inertial)
# =============================================================================
def quat_mul(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw * qw - px * qx - py * qy - pz * qz,
        pw * qx + px * qw + py * qz - pz * qy,
        pw * qy - px * qz + py * qw + pz * qx,
        pw * qz + px * qy - py * qx + pz * qw,
    ])


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def quat_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12 or abs(angle) < 1e-15:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), *(axis * s)])


def quat_from_rotvec(v: np.ndarray) -> np.ndarray:
    ang = np.linalg.norm(v)
    return quat_from_axis_angle(v, ang) if ang > 1e-15 else \
        quat_normalize(np.array([1.0, *(0.5 * v)]))


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """Rotation matrix R such that v_inertial = R @ v_body."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_error_vec(q_ref: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Small-angle rotation vector (rad, body frame) taking q_ref to q."""
    qe = quat_mul(quat_conj(q_ref), q)
    if qe[0] < 0:
        qe = -qe
    # exact log map for robustness at large angles
    vn = np.linalg.norm(qe[1:])
    if vn < 1e-12:
        return 2.0 * qe[1:]
    ang = 2.0 * math.atan2(vn, qe[0])
    return qe[1:] / vn * ang


def quat_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    return math.degrees(np.linalg.norm(quat_error_vec(q1, q2)))


def quat_to_euler_deg(q: np.ndarray) -> np.ndarray:
    """ZYX (yaw-pitch-roll) Euler angles in degrees, returned as [roll, pitch, yaw]."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees([roll, pitch, yaw])


# =============================================================================
# Configuration
# =============================================================================
@dataclass
class BodyConfig:
    inertia: np.ndarray = field(default_factory=lambda: np.array([
        [12.0, 0.15, -0.10],
        [0.15, 15.0, 0.20],
        [-0.10, 0.20, 9.0]]))                     # kg*m^2
    q0: np.ndarray = field(default_factory=lambda: quat_from_axis_angle([1, 2, 3], 0.05))
    w0: np.ndarray = field(default_factory=lambda: np.array([0.01, -0.008, 0.005]))  # rad/s


@dataclass
class MotorConfig:
    """Reaction wheel = rotor inertia + brushless DC motor."""
    J_rotor: float = 0.02          # kg*m^2
    Kt: float = 0.03               # N*m/A   torque constant
    Ke: float = 0.03               # V*s/rad back-EMF constant (= Kt in SI)
    R: float = 1.2                 # ohm     winding resistance
    V_bus: float = 28.0            # V       supply limit
    I_max: float = 3.0             # A       driver current limit
    coulomb: float = 2.0e-4        # N*m     bearing Coulomb friction
    viscous: float = 1.0e-6        # N*m*s/rad viscous friction
    omega_max: float = 6000 * 2 * math.pi / 60   # rad/s  hard speed limit
    # Torque-speed envelope: (speed rad/s, max |torque| N*m), linearly interpolated.
    # Models thermal/commutation derating on top of the electrical limit.
    curve_speed: np.ndarray = field(default_factory=lambda: np.array(
        [0, 200, 400, 550, 628.3]))
    curve_torque: np.ndarray = field(default_factory=lambda: np.array(
        [0.090, 0.088, 0.070, 0.040, 0.0]))


@dataclass
class GyroConfig:
    rate_hz: float = 100.0
    arw: float = math.radians(0.15) / 60.0         # rad/s/sqrt(Hz)  (0.15 deg/sqrt(hr))
    rrw: float = math.radians(0.5) / 3600 / 60.0   # rad/s^2/sqrt(Hz) rate random walk
    bias_instability: float = math.radians(0.5) / 3600   # rad/s (1-sigma GM)
    bias_tau: float = 300.0                        # s   Gauss-Markov correlation time
    turn_on_bias_sigma: float = math.radians(5.0) / 3600  # rad/s
    temp_coeff: float = math.radians(0.02) / 3600  # rad/s per degC
    scale_factor_sigma: float = 300e-6             # 300 ppm
    misalign_sigma: float = 1e-3                   # rad
    quantum: float = math.radians(0.0005)          # rad/s per LSB
    range_max: float = math.radians(300.0)         # rad/s
    temp_initial: float = 20.0                     # degC
    temp_final: float = 35.0                       # degC (warm-up)
    temp_tau: float = 600.0                        # s


@dataclass
class EnvConfig:
    orbit_rate: float = 2 * math.pi / 5640.0       # rad/s (LEO ~94 min)
    const_torque: np.ndarray = field(default_factory=lambda: np.array([2e-6, -1e-6, 1.5e-6]))
    sin_torque_amp: np.ndarray = field(default_factory=lambda: np.array([5e-6, 5e-6, 2e-6]))
    sin_torque_freq: float = 2 * math.pi / 5640.0


@dataclass
class ControlConfig: