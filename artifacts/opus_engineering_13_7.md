# 3-Axis Rigid Body Attitude Simulator

I don't have context for what "Cycle 13" refers to, so this is a complete, self-contained simulator built from your three requirements. If Cycle 13 has specific parameters or interfaces, they can be adapted into the configuration dataclasses near the top of the file.

The simulator models:

- **Rigid body dynamics**: full 3-axis Euler equations with gyroscopic coupling, quaternion kinematics and RK4 integration.
- **Motor torque curves**: reaction-wheel motors with a current-limited region, a back-EMF-limited region, regenerative braking, electrical lag, torque ripple and friction.
- **Gyro drift**: constant bias, angle random walk, rate random walk, Gauss-Markov bias instability, temperature-dependent bias, scale factor error, misalignment, quantization and saturation.

## Requirements

- **Required**: Python 3.9+ and `numpy`.
- **Optional**: `matplotlib`, for the plots.

## Usage

```bash
python attitude_sim.py                       # default slew + hold scenario with plots
python attitude_sim.py --star-tracker 0      # pure gyro propagation, so drift is unbounded
python attitude_sim.py --star-tracker 10     # attitude fix every 10 s
python attitude_sim.py --test                # run the physics verification tests
python attitude_sim.py --duration 600 --no-plot --seed 3
```

## Source: `attitude_sim.py`

```python
#!/usr/bin/env python3
"""
attitude_sim.py - 3D rigid-body attitude simulator with reaction wheels.

Models
------
* 3-axis rigid-body rotational dynamics (Euler equations + gyroscopic
  coupling with wheel momentum), quaternion kinematics, RK4 integration.
* Reaction-wheel motors with realistic torque-speed curves:
  current-limited region, back-EMF-limited region, regenerative braking,
  first-order electrical lag, torque ripple, Coulomb/viscous friction.
* MEMS/FOG-style rate gyro with drift: constant bias, angle random walk,
  rate random walk, Gauss-Markov bias instability, temperature-dependent
  bias, scale-factor error, axis misalignment, quantization, saturation.
* Gyro-propagated attitude estimator with optional periodic absolute
  (star-tracker-like) fixes, and a quaternion PD attitude controller.

Conventions
-----------
Quaternions are scalar-first [w, x, y, z] and rotate body -> inertial.
Angular velocity w is expressed in the body frame (rad/s).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np


# ============================================================================
# Quaternion utilities
# ============================================================================

def q_mult(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a ⊗ b."""
    a0, a1, a2, a3 = a
    b0, b1, b2, b3 = b
    return np.array([
        a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3,
        a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2,
        a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1,
        a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0,
    ])


def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-15:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), *(axis * s)])


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """Rotation matrix mapping body vectors to inertial vectors."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def q_angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    """Smallest rotation angle (rad) between two attitudes."""
    d = abs(float(np.dot(q_normalize(q1), q_normalize(q2))))
    return 2.0 * math.acos(min(1.0, d))


def q_kinematics(q: np.ndarray, w: np.ndarray) -> np.ndarray:
    """dq/dt = 0.5 * q ⊗ [0, w]."""
    return 0.5 * q_mult(q, np.array([0.0, *w]))


def q_propagate_exact(q: np.ndarray, w: np.ndarray, dt: float) -> np.ndarray:
    """Propagate q by a constant body rate over dt (exponential map)."""
    wn = np.linalg.norm(w)
    if wn * dt < 1e-12:
        dq = np.array([1.0, *(0.5 * w * dt)])
    else:
        dq = q_from_axis_angle(w / wn, wn * dt)
    return q_normalize(q_mult(q, dq))


# ============================================================================
# Motor model with torque-speed curve
# ============================================================================

@dataclass
class MotorParams:
    kt: float = 0.025             # torque constant [N*m/A]
    ke: float = 0.025             # back-EMF constant [V*s/rad] (= kt in SI)
    resistance: float = 1.2       # winding resistance [ohm]
    bus_voltage: float = 12.0     # supply voltage [V]
    current_limit: float = 3.0    # driver current limit [A]
    tau_electrical: float = 0.01  # current-loop time constant [s]
    coulomb_friction: float = 1.5e-4   # [N*m]
    viscous_friction: float = 2.0e-7   # [N*m/(rad/s)]
    stiction_band: float = 0.5    # speed below which friction is smoothed [rad/s]
    ripple_fraction: float = 0.02 # torque ripple amplitude (fraction of torque)
    ripple_poles: int = 12        # ripple cycles per revolution
    wheel_inertia: float = 2.0e-4 # rotor+flywheel axial inertia [kg*m^2]


class Motor:
    """
    Brushless reaction-wheel motor driven by a current-controlled driver.

    Torque-speed envelope (motoring quadrant):
        |tau| <= kt * I_max                                (current limit)
        |tau| <= kt * (V_bus - ke*|w|) / R                 (voltage limit)
    In the braking (regenerative) quadrant the back-EMF assists, so only
    the current limit applies (up to the electrical limit (V+ke|w|)/R).
    """

    def __init__(self, p: MotorParams):
        self.p = p
        self.torque_actual = 0.0   # electromagnetic torque after lag [N*m]
        self.angle = 0.0           # rotor angle, for ripple [rad]

    # ------------------------------------------------------------ curve
    def max_torque(self, speed: float, direction: float) -> float:
        """Available electromagnetic torque magnitude in `direction`
        (+1 or -1) at relative wheel speed `speed` [rad/s]."""
        p = self.p
        i_lim = p.current_limit
        motoring = direction * speed >= 0.0
        if motoring:
            i_volt = (p.bus_voltage - p.ke * abs(speed)) / p.resistance
        else:
            i_volt = (p.bus_voltage + p.ke * abs(speed)) / p.resistance
        return p.kt * max(0.0, min(i_lim, i_volt))

    def no_load_speed(self) -> float:
        return self.p.bus_voltage / self.p.ke

    def torque_curve(self, n: int = 200) -> Dict[str, np.ndarray]:
        """Sample the motoring-quadrant envelope (for plotting/inspection)."""
        speeds = np.linspace(0.0, self.no_load_speed() * 1.05, n)
        torques = np.array([self.max_torque(s, +1.0) for s in speeds])
        return {"speed": speeds, "torque": torques}

    # ------------------------------------------------------------ friction
    def friction(self, speed: float) -> float:
        """Friction torque opposing rotation (signed, acting on the wheel)."""
        p = self.p
        coul = p.coulomb_friction * math.tanh(speed / p.stiction_band)
        return -(coul + p.viscous_friction * speed)

    # ------------------------------------------------------------ update
    def step(self, torque_cmd: float, speed: float, dt: float) -> float:
        """
        Advance motor electrical state one step.

        Returns the net torque applied to the wheel rotor
        (electromagnetic + ripple + friction). The reaction on the body is
        the negative of this value.
        """
        direction = 1.0 if torque_cmd >= 0.0 else -1.0
        limit = self.max_torque(speed, direction)
        target = direction * min(abs(torque_cmd), limit)

        # First-order current-loop lag (exact discretization).
        alpha = 1.0 - math.exp(-dt / self.p.tau_electrical)
        self.torque_actual += alpha * (target - self.torque_actual)

        # Re-clip in case speed changed and the envelope shrank.
        d = 1.0 if self.torque_actual >= 0 else -1.0
        lim_now = self.max_torque(speed, d)
        self.torque_actual = d * min(abs(self.torque_actual), lim_now)

        self.angle = (self.angle + speed * dt) % (2.0 * math.pi)
        ripple = (self.p.ripple_fraction * self.torque_actual
                  * math.sin(self.p.ripple_poles * self.angle))

        return self.torque_actual + ripple + self.friction(speed)

    def current(self) -> float:
        return self.torque_actual / self.p.kt


# ============================================================================
# Gyro model with drift
# ============================================================================

@dataclass
class GyroParams:
    arw: float = math.radians(0.15) / 60.0          # angle random walk [rad/sqrt(s)]  (0.15 deg/sqrt(hr))
    rrw: float = math.radians(0.5) / 3600.0 / 60.0  # rate random walk [rad/s/sqrt(s)]
    bias_instability: float = math.radians(3.0) / 3600.0  # GM bias sigma [rad/s] (3 deg/hr)
    bias_corr_time: float = 300.0                   # GM correlation time [s]
    initial_bias_sigma: float = math.radians(10.0) / 3600.0  # turn-on bias [rad/s]
    temp_coeff: float = math.radians(0.5) / 3600.0  # bias tempco [rad/s per degC]
    scale_factor_sigma: float = 300e-6              # scale-factor error (300 ppm)
    misalignment_sigma: float = 1e-3                # cross-axis [rad]
    quantization: float = 1e-6                      # LSB [rad/s]
    saturation: float = math.radians(400.0)         # full scale [rad/s]
    ref_temperature: float = 25