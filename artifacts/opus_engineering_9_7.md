# 3-Axis Rigid Body Physics Simulator (Cycle 9)

This is a single-file Python 3D attitude dynamics simulator with these parts:

- **Rigid body mechanics.** It uses a full inertia tensor, Euler's equations with gyroscopic coupling, quaternion kinematics and RK4 integration.
- **Motor torque curves.** Three reaction wheels are driven by DC motors. Each motor has back-EMF, a current limit, a voltage limit, and viscous plus Coulomb friction. Together these give a realistic torque-speed curve.
- **Gyro drift model.** It includes bias, bias random walk, Gauss-Markov bias instability, angle random walk, temperature-dependent bias, scale factor error, axis misalignment and quantization.
- **Estimation and control.** Attitude is estimated by integrating the gyro, with optional star-tracker correction and bias estimation. A quaternion PD controller with feedforward commands the motor voltages.
- **Self-tests, logging and plotting.** It checks momentum and energy conservation, writes CSV logs, and draws plots if matplotlib is installed.

Requirements: Python 3.8+ and `numpy`. `matplotlib` is optional and only needed for plots.

```python
#!/usr/bin/env python3
"""
attitude_sim.py  --  3-Axis Rigid Body Physics Simulator  (Cycle 9)

Models:
  * Rigid-body rotational dynamics (Euler's equations, full inertia tensor,
    gyroscopic coupling, internal reaction-wheel momentum)
  * Quaternion attitude kinematics, RK4 integration
  * DC-motor-driven reaction wheels with torque-speed curves
    (back-EMF, current limit, voltage limit, viscous + Coulomb friction)
  * MEMS/FOG-style gyro error model with drift:
      constant bias, bias random walk, bias instability (Gauss-Markov),
      angle random walk, temperature-dependent bias, scale factor,
      misalignment, quantization
  * Attitude estimator (gyro integration + optional star tracker w/ bias est.)
  * Quaternion PD attitude controller

Usage:
  python attitude_sim.py                       # default run, 120 s
  python attitude_sim.py --time 600 --no-star  # see pure gyro drift
  python attitude_sim.py --selftest            # physics conservation checks
  python attitude_sim.py --plot --csv out.csv
  python attitude_sim.py --motor-curve         # plot motor torque curve
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass, field

import numpy as np

# ============================================================================
# Quaternion utilities  (scalar-first: q = [w, x, y, z], body -> inertial)
# ============================================================================

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
    return q / n if n > 0 else np.array([1.0, 0.0, 0.0, 0.0])


def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), *(axis * s)])


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
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """Rotation matrix body -> inertial."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def q_angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Smallest rotation angle (rad) between two attitudes."""
    d = abs(float(np.dot(q_norm(a), q_norm(b))))
    return 2.0 * math.acos(min(1.0, d))


def q_exp_rate(omega: np.ndarray, dt: float) -> np.ndarray:
    """Exact quaternion increment for constant body rate over dt."""
    theta = np.linalg.norm(omega) * dt
    if theta < 1e-12:
        return q_norm(np.array([1.0, *(0.5 * omega * dt)]))
    axis = omega / np.linalg.norm(omega)
    return np.array([math.cos(theta / 2), *(axis * math.sin(theta / 2))])


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


# ============================================================================
# DC motor + reaction wheel model
# ============================================================================

@dataclass
class DCMotorParams:
    kt: float = 0.025          # N*m/A torque constant
    ke: float = 0.025          # V*s/rad back-EMF constant (= kt in SI)
    resistance: float = 1.2    # Ohm winding resistance
    v_max: float = 24.0        # V supply limit
    i_max: float = 4.0         # A driver current limit
    viscous: float = 2.0e-6    # N*m*s/rad viscous friction
    coulomb: float = 1.0e-3    # N*m Coulomb friction
    coulomb_vel: float = 0.5   # rad/s smoothing width for friction sign
    wheel_inertia: float = 2.5e-3  # kg*m^2 rotor + flywheel axial inertia


class DCMotor:
    """Brushed/BLDC-equivalent DC motor driving a reaction wheel.

    Steady-state torque-speed curve at full voltage:
        i  = clamp((V - ke*Omega)/R, -i_max, i_max)
        T  = kt*i - b*Omega - Tc*tanh(Omega/w_c)
    This gives a flat current-limited region at low speed, followed by a
    linear back-EMF roll-off reaching zero torque near no-load speed.
    """

    def __init__(self, p: DCMotorParams):
        self.p = p

    @property
    def no_load_speed(self) -> float:
        return self.p.v_max / self.p.ke

    @property
    def stall_torque(self) -> float:
        return self.p.kt * min(self.p.v_max / self.p.resistance, self.p.i_max)

    def current(self, voltage: float, omega: float) -> float:
        v = float(np.clip(voltage, -self.p.v_max, self.p.v_max))
        i = (v - self.p.ke * omega) / self.p.resistance
        return float(np.clip(i, -self.p.i_max, self.p.i_max))

    def friction(self, omega: float) -> float:
        p = self.p
        return p.viscous * omega + p.coulomb * math.tanh(omega / p.coulomb_vel)

    def torque(self, voltage: float, omega: float) -> float:
        """Net torque applied to the wheel (N*m) by the motor shaft."""
        return self.p.kt * self.current(voltage, omega) - self.friction(omega)

    def voltage_for_torque(self, torque_cmd: float, omega: float) -> float:
        """Inverse model: voltage needed to produce the commanded wheel torque."""
        p = self.p
        t_total = torque_cmd + self.friction(omega)
        i_cmd = np.clip(t_total / p.kt, -p.i_max, p.i_max)
        v = i_cmd * p.resistance + p.ke * omega
        return float(np.clip(v, -p.v_max, p.v_max))

    def power(self, voltage: float, omega: float) -> float:
        return float(np.clip(voltage, -self.p.v_max, self.p.v_max)) * \
            self.current(voltage, omega)

    def torque_curve(self, n: int = 200):
        """Returns (speeds, torque at +Vmax, max-current torque) for plotting."""
        speeds = np.linspace(0, self.no_load_speed * 1.05, n)
        t_v = np.array([self.torque(self.p.v_max, w) for w in speeds])
        t_i = np.array([self.p.kt * self.p.i_max - self.friction(w) for w in speeds])
        return speeds, t_v, t_i


# ============================================================================
# Gyroscope error model
# ============================================================================

@dataclass
class GyroParams:
    rate_hz: float = 200.0
    bias0_dps: float = 0.05              # deg/s initial (turn-on) bias, per axis 1-sigma
    arw_dps_rthz: float = 0.005          # deg/s/sqrt(Hz)  (= deg/sqrt(s)) angle random walk
    rrw_dps_rts: float = 2e-5            # deg/s/sqrt(s) rate random walk (bias walk)
    bias_instab_dps: float = 0.003       # deg/s Gauss-Markov bias instability
    bias_corr_time_s: float = 100.0      # s correlation time
    temp_coeff_dps_per_c: float = 0.002  # deg/s/degC bias temperature sensitivity
    scale_factor_ppm: float = 500.0      # 1-sigma scale factor error
    misalign_mrad: float = 1.0           # 1-sigma axis misalignment
    quant_dps: float = 0.0005            # deg/s LSB
    range_dps: float = 500.0             # saturation
    t_ref_c: float = 25.0


class Gyro:
    def __init__(self, p: GyroParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng
        d2r = math.pi / 180.0
        self.bias_const = rng.normal(0, p.bias0_dps * d2r, 3)
        self.bias_walk = np.zeros(3)
        self.bias_gm = np.zeros(3)
        sf = rng.normal(0, p.scale_factor_ppm * 1e-6, 3