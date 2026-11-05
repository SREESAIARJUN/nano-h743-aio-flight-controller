# 3-Axis Rigid-Body Attitude Simulator

This is a single self-contained Python file. It needs NumPy, and optionally Matplotlib for plots. I treated "Cycle 23" as a version label because I don't have context for what it refers to. If it points to a specific spec, tell me what changed and I'll adapt the code.

**What it models:**
- **Rigid-body dynamics.** Full Euler equations with a non-diagonal inertia tensor, gyroscopic coupling, and quaternion kinematics, integrated with fixed-step RK4.
- **Reaction-wheel actuators.** Three wheels on configurable axes with momentum exchange between wheels and body.
- **Motor torque curves.** A DC/BLDC model with back-EMF, current limit, and voltage saturation, giving a torque-speed curve with a constant-torque plateau. Coulomb and viscous friction are included. An optional tabulated curve can override the model.
- **Gyro errors.** White noise (angle random walk), bias instability (random walk), a temperature-dependent bias, scale factor, axis misalignment, quantization, and range saturation.
- **Estimation and control.** The attitude estimator integrates the gyro, with optional periodic star-tracker fixes and online bias estimation. A PD controller with motor inversion computes the wheel commands.
- **Outputs.** CSV logging, plots, a torque-curve printout, and a momentum-conservation self-test.

```python
#!/usr/bin/env python3
"""
attitude_sim.py  --  3-axis rigid-body attitude simulator (Cycle 23)

Models:
  * Rigid-body rotational dynamics (Euler eqs. + quaternion kinematics, RK4)
  * 3 reaction wheels driven by DC/BLDC motors with realistic torque-speed
    curves (back-EMF, current limit, bus-voltage limit, friction)
  * MEMS/FOG-style gyro with bias instability, ARW, temperature drift,
    scale factor, misalignment, quantization and saturation
  * Gyro-integrating attitude estimator with optional star-tracker updates
    and bias estimation
  * PD attitude controller

Conventions:
  Quaternion q = [w, x, y, z], rotates body-frame vectors into inertial frame.
  Angular rates are body-frame, rad/s.

Usage:
  python attitude_sim.py                    # default run with plots
  python attitude_sim.py --no-tracker       # watch gyro drift unchecked
  python attitude_sim.py --selftest         # momentum conservation check
  python attitude_sim.py --curve            # print motor torque curve
  python attitude_sim.py --csv out.csv --no-plot
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

CYCLE = 23
__version__ = f"1.{CYCLE}.0"


# =============================================================================
# Quaternion utilities
# =============================================================================
def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a ⊗ b."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_norm(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0, 0, 0])


def q_from_rotvec(v: np.ndarray) -> np.ndarray:
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return q_norm(np.array([1.0, 0.5 * v[0], 0.5 * v[1], 0.5 * v[2]]))
    axis = v / angle
    s = math.sin(angle / 2)
    return np.array([math.cos(angle / 2), *(axis * s)])


def q_to_rotvec(q: np.ndarray) -> np.ndarray:
    q = q if q[0] >= 0 else -q            # shortest rotation
    v = q[1:]
    s = np.linalg.norm(v)
    if s < 1e-12:
        return 2.0 * v
    angle = 2.0 * math.atan2(s, q[0])
    return v / s * angle


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """Body -> inertial rotation matrix."""
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
# Motor model (torque-speed curve)
# =============================================================================
@dataclass
class MotorParams:
    kt: float = 0.025          # torque constant [N·m/A]
    ke: float = 0.025          # back-EMF constant [V·s/rad] (== kt in SI)
    resistance: float = 1.2    # winding resistance [ohm]
    v_bus: float = 12.0        # supply voltage [V]
    i_max: float = 2.0         # driver current limit [A]
    coulomb: float = 2.0e-4    # Coulomb friction [N·m]
    viscous: float = 1.0e-6    # viscous friction [N·m·s/rad]
    max_speed: float = 600.0   # hard overspeed limit [rad/s] (~5700 rpm)
    # Optional tabulated torque-speed envelope: (speed rad/s, max torque N·m).
    # If provided, motor torque magnitude is clipped to this envelope.
    table_speed: Optional[np.ndarray] = None
    table_torque: Optional[np.ndarray] = None


class DCMotor:
    """
    Brushed-DC / BLDC (trapezoidal, averaged) motor with a voltage-mode driver.

        i = (u*V_bus - ke*Ω) / R          (clipped to ±i_max)
        τ_em = kt * i
        τ_net = τ_em - τ_coulomb*sign(Ω) - b*Ω

    The resulting torque-speed curve has:
      - a constant-torque region (current limit) at low speed
      - a linearly falling region (back-EMF) up to the no-load speed
    """

    def __init__(self, p: MotorParams):
        self.p = p
        self.last_current = 0.0

    def electromagnetic_torque(self, u: float, omega_rel: float) -> float:
        p = self.p
        u = max(-1.0, min(1.0, u))
        v = u * p.v_bus
        i = (v - p.ke * omega_rel) / p.resistance
        i = max(-p.i_max, min(p.i_max, i))
        tau = p.kt * i
        if p.table_speed is not None and p.table_torque is not None:
            lim = float(np.interp(abs(omega_rel), p.table_speed, p.table_torque))
            tau = max(-lim, min(lim, tau))
        # overspeed protection: no further acceleration beyond max_speed
        if abs(omega_rel) >= p.max_speed and tau * omega_rel > 0:
            tau = 0.0
        self.last_current = tau / p.kt if p.kt else 0.0
        return tau

    def friction_torque(self, omega_rel: float) -> float:
        p = self.p
        # smooth sign to avoid chattering stiffness in RK4
        smooth_sign = math.tanh(omega_rel / 0.5)
        return p.coulomb * smooth_sign + p.viscous * omega_rel

    def net_torque(self, u: float, omega_rel: float) -> float:
        """Net torque on the rotor (wheel) about its spin axis."""
        return self.electromagnetic_torque(u, omega_rel) - self.friction_torque(omega_rel)

    def command_for_torque(self, tau_des: float, omega_rel: float) -> float:
        """Invert the motor model: required duty for desired EM torque."""
        p = self.p
        i_des = max(-p.i_max, min(p.i_max, tau_des / p.kt))
        v = i_des * p.resistance + p.ke * omega_rel
        return max(-1.0, min(1.0, v / p.v_bus))

    def no_load_speed(self) -> float:
        return self.p.v_bus / self.p.ke

    def curve(self, n: int = 21, u: float = 1.0):
        speeds = np.linspace(0, self.no_load_speed(), n)
        return [(s, self.electromagnetic_torque(u, s), self.net_torque(u, s)) for s in speeds]


# =============================================================================
# Gyro model
# =============================================================================
@dataclass
class GyroParams:
    arw: float = math.radians(0.15) / 60.0           # angle random walk [rad/√s]  (0.15 °/√h)
    bias_instability: float = math.radians(5.0) / 3600.0  # [rad/s] (5 °/h)
    bias_corr_time: float = 300.0                    # Gauss-Markov correlation time [s]
    initial_bias_sigma: float = math.radians(50.0) / 3600.0  # turn-on bias [rad/s]
    temp_coeff: float = math.radians(0.5) / 3600.0   # bias tempco [rad/s/°C]
    scale_factor_sigma: float = 300e-6               # 300 ppm
    misalignment_sigma: float = 1e-3                 # rad
    quantization: float = math.radians(0.001)        # LSB [rad/s]
    range_limit: float = math.radians(400.0)         # full-scale [rad/s]
    temp_nominal: float = 25.0                       # °C
    temp_amplitude: float = 10.0                     # °C thermal swing
    temp_period: float = 1200.0                      # s


class Gyro:
    """
    Measurement model:
        ω_m = Q( sat( (I + M) (I + S) 