# 3D Rigid-Body Physics Simulator with Reaction-Wheel Motors and Gyro Drift

This is a self-contained Python program. It models:

- **6-DOF rigid-body dynamics.** It uses quaternion attitude, the full Euler equations with gyroscopic coupling, wheel angular momentum, and RK4 integration.
- **Three reaction-wheel BLDC motors.** Each has an R-L electrical model, back-EMF, a voltage-limited torque-speed curve, a current limit, Coulomb and viscous friction, and cogging ripple.
- **A MEMS gyro error model.** It includes angle random walk, rate random walk, a bias-instability Gauss-Markov process, temperature-dependent bias, scale-factor error, axis misalignment, and quantization.
- **Navigation and control.** A gyro-integrated attitude estimator is optionally corrected by a star tracker with online bias estimation. A PD attitude controller drives the motor current loops.

> **About "Cycle 30":** I don't know what that refers to in your project. I treated it as a run label. It is exposed as `--cycle`, which tags the output and seeds the random generator, so Cycle 30 is reproducible. If it means something specific, such as 30 slew cycles or a test-plan ID, tell me and I'll adapt the scenario.

```python
#!/usr/bin/env python3
"""
rigidbody_sim.py — 3-axis rigid body simulator with reaction-wheel motor
torque curves and gyro drift modeling.

Usage:
    python rigidbody_sim.py --cycle 30 --duration 600 --plot
    python rigidbody_sim.py --cycle 30 --no-startracker --plot   # pure gyro drift
    python rigidbody_sim.py --cycle 30 --csv out.csv

Conventions:
    Quaternion q = [w, x, y, z], scalar first, rotates body -> inertial.
    Angular velocity omega is expressed in the body frame (rad/s).
    Reaction wheel i spins about body axis i (aligned triad).

Requires: numpy (matplotlib optional, for --plot)
"""

from __future__ import annotations
import argparse
import csv
import math
from dataclasses import dataclass, field

import numpy as np


# =============================================================================
# Quaternion / rotation utilities
# =============================================================================
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


def q_norm(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0, 0, 0])


def q_from_rotvec(v: np.ndarray) -> np.ndarray:
    ang = np.linalg.norm(v)
    if ang < 1e-12:
        return q_norm(np.array([1.0, 0.5 * v[0], 0.5 * v[1], 0.5 * v[2]]))
    axis = v / ang
    s = math.sin(ang / 2)
    return np.array([math.cos(ang / 2), *(axis * s)])


def q_to_rotvec(q: np.ndarray) -> np.ndarray:
    q = q if q[0] >= 0 else -q            # shortest path
    vn = np.linalg.norm(q[1:])
    if vn < 1e-12:
        return 2.0 * q[1:]
    ang = 2.0 * math.atan2(vn, q[0])
    return q[1:] / vn * ang


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def q_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX (yaw-pitch-roll) Euler angles in radians."""
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
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


# =============================================================================
# Motor model (BLDC / brushed-DC equivalent) with torque-speed curve
# =============================================================================
@dataclass
class MotorParams:
    Kt: float = 0.025          # N·m/A torque constant
    Ke: float = 0.025          # V·s/rad back-EMF constant (= Kt in SI)
    R: float = 1.2             # Ω winding resistance
    L: float = 0.8e-3          # H winding inductance
    V_bus: float = 12.0        # V supply
    I_max: float = 3.0         # A driver current limit
    J_rotor: float = 2.0e-3    # kg·m² wheel + rotor inertia
    tau_coulomb: float = 1.5e-4  # N·m Coulomb friction
    b_viscous: float = 2.0e-6    # N·m·s/rad viscous friction
    cog_amp: float = 2.0e-5    # N·m cogging torque amplitude
    cog_poles: int = 12        # cogging periods per revolution
    Kp_current: float = 8.0    # V/A current-loop proportional gain
    Ki_current: float = 4000.0 # V/(A·s) current-loop integral gain


class Motor:
    """Electrical + mechanical motor model. State lives in the global
    state vector; this class provides derivatives and the driver logic."""

    def __init__(self, p: MotorParams):
        self.p = p
        self.i_integ = 0.0  # current-loop integrator (discrete, driver side)

    # ---- static torque-speed curve (steady state, voltage + current limit)
    def max_torque_at_speed(self, omega: float, V: float | None = None) -> float:
        p = self.p
        V = p.V_bus if V is None else V
        i_volt = (V - p.Ke * abs(omega)) / p.R
        i = max(0.0, min(p.I_max, i_volt))
        return p.Kt * i

    def no_load_speed(self) -> float:
        p = self.p
        # Kt*(V - Ke*w)/R = tau_c + b*w  ->  solve for w
        return (p.Kt * p.V_bus / p.R - p.tau_coulomb) / (p.Kt * p.Ke / p.R + p.b_viscous)

    def torque_speed_curve(self, n: int = 200):
        w = np.linspace(0, self.no_load_speed() * 1.05, n)
        tau = np.array([self.max_torque_at_speed(x) for x in w])
        return w, tau

    # ---- friction and cogging
    def friction(self, omega: float) -> float:
        p = self.p
        # smooth Coulomb (tanh) avoids stiff chattering at zero crossing
        return p.tau_coulomb * math.tanh(omega / 0.5) + p.b_viscous * omega

    def cogging(self, theta: float) -> float:
        return self.p.cog_amp * math.sin(self.p.cog_poles * theta)

    # ---- driver: PI current loop -> saturated phase voltage
    def drive_voltage(self, i_cmd: float, i_meas: float, omega: float, dt: float) -> float:
        p = self.p
        i_cmd = max(-p.I_max, min(p.I_max, i_cmd))
        err = i_cmd - i_meas
        V_ff = p.R * i_cmd + p.Ke * omega          # feedforward incl. back-EMF
        V_unsat = V_ff + p.Kp_current * err + self.i_integ
        V = max(-p.V_bus, min(p.V_bus, V_unsat))
        if V == V_unsat:                            # anti-windup: clamp integrator
            self.i_integ += p.Ki_current * err * dt
        return V

    # ---- continuous-time derivatives
    def derivs(self, V: float, i: float, omega: float, theta: float):
        """Return (di/dt, dOmega/dt, dtheta/dt, tau_motor_on_wheel_net)."""
        p = self.p
        di = (V - p.R * i - p.Ke * omega) / p.L
        tau_em = p.Kt * i + self.cogging(theta)
        tau_net = tau_em - self.friction(omega)     # net torque on wheel
        return di, tau_net / p.J_rotor, omega, tau_net


# =============================================================================
# Gyro model with drift
# =============================================================================
@dataclass
class GyroParams:
    rate_hz: float = 200.0
    arw: float = math.radians(0.15) / 60.0         # rad/√s  (0.15 °/√hr)
    rrw: float = math.radians(0.5) / 3600.0 / 60.0 # rad/s/√s rate random walk
    bias_init_sigma: float = math.radians(10.0) / 3600.0  # rad/s turn-on bias
    bi_sigma: float = math.radians(2.0) / 3600.0   # bias instability (GM) σ
    bi_tau: float = 300.0                          # s Gauss-Markov correlation
    temp_coeff: float = math.radians(0.5) / 3600.0 # rad/s per °C
    scale_factor_sigma: float = 500e-6             # 500 ppm
    misalign_sigma: float = math.radians(0.05)     # rad
    quant: float = math.radians(0.01) / 3600.0 * 100  # LSB rad/s
    sat: float = math.radians(400.0)               # rad/s full scale


class Gyro:
    def __init__(self, p: GyroParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng
        self.bias_rw = rng.normal(0, p.bias_init_sigma, 3)  # turn-on + RRW
        self.bias_gm = np.zeros(3)                          # bias instability
        sf = rng.normal(0, p.scale_factor_sigma