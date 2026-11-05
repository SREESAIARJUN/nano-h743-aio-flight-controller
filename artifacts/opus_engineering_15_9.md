# 3-Axis Rigid Body Physics Simulator (Cycle 15)

This is a self-contained 3D simulator for a spacecraft that uses three reaction wheels for attitude control. It needs only `numpy`. Plotting with `matplotlib` is optional.

## Overview

| Subsystem | Model |
|---|---|
| **Rigid body** | Full 6-DOF. Euler's equations with the gyroscopic coupling term ω×(Iω + h_wheels). Quaternion kinematics. RK4 integration. |
| **Motors** | Per-axis BLDC: R-L electrical circuit, back-EMF, supply-voltage and current limits, Coulomb + viscous friction. The torque-speed curve emerges from this physics, and an analytic version is also provided. |
| **Gyro** | Angle random walk, Gauss-Markov bias instability, rate random walk, temperature-dependent bias, scale factor error, axis misalignment, quantization and saturation. |
| **Estimation** | Gyro integration plus an optional star tracker with Mahony-style attitude and bias correction. You can disable it to watch pure drift. |
| **Control** | Quaternion PD controller with gyroscopic feedforward. Includes slew maneuvers and an environmental disturbance torque. |

## Source: `rigid_body_sim.py`

```python
#!/usr/bin/env python3
"""
rigid_body_sim.py  --  Cycle 15
3D rigid-body simulator: 3-axis attitude dynamics, reaction-wheel motor
torque curves, and MEMS/FOG gyro drift modeling.

Conventions:
  * Quaternion q = [w, x, y, z], scalar first, rotates BODY -> INERTIAL.
  * Angular velocity omega is expressed in the BODY frame [rad/s].
  * SI units throughout unless noted.

Usage:
  python rigid_body_sim.py                  # default run, star tracker on
  python rigid_body_sim.py --no-startracker # observe pure gyro drift
  python rigid_body_sim.py --tend 600 --plot
"""

from __future__ import annotations
import argparse
import math
from dataclasses import dataclass, field
import numpy as np

# ---------------------------------------------------------------------------
# Quaternion utilities
# ---------------------------------------------------------------------------

def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])

def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])

def q_norm(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0, 0, 0])

def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle / 2)
    return np.array([math.cos(angle / 2), *(axis * s)])

def q_exp(rotvec: np.ndarray) -> np.ndarray:
    """Quaternion from rotation vector (axis * angle)."""
    ang = np.linalg.norm(rotvec)
    if ang < 1e-12:
        return q_norm(np.array([1.0, *(0.5 * rotvec)]))
    return q_from_axis_angle(rotvec / ang, ang)

def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """DCM rotating body vectors into inertial frame."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])

def q_to_euler(q: np.ndarray) -> np.ndarray:
    """ZYX (yaw-pitch-roll) Euler angles [roll, pitch, yaw] in rad."""
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])

def q_angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Smallest rotation angle between two attitudes [rad]."""
    d = abs(float(np.dot(q_norm(a), q_norm(b))))
    return 2.0 * math.acos(min(1.0, d))

# ---------------------------------------------------------------------------
# Motor model (BLDC reaction-wheel drive)
# ---------------------------------------------------------------------------

@dataclass
class MotorParams:
    Kt: float = 0.025          # torque constant [N*m/A]
    Ke: float = 0.025          # back-EMF constant [V*s/rad] (== Kt in SI)
    R: float = 1.2             # winding resistance [ohm]
    L: float = 0.8e-3          # winding inductance [H]
    V_supply: float = 28.0     # bus voltage [V]
    I_max: float = 3.0         # driver current limit [A]
    tau_coulomb: float = 2e-4  # Coulomb friction [N*m]
    b_viscous: float = 2e-7    # viscous friction [N*m/(rad/s)]
    J_wheel: float = 2.5e-3    # wheel spin inertia [kg*m^2]
    omega_max: float = 650.0   # mechanical overspeed limit [rad/s] (~6200 rpm)

class Motor:
    """
    Electrical:  L di/dt = V - R i - Ke * Omega
    Mechanical:  J dOmega/dt = Kt i - friction(Omega)
    Driver: current-mode controller. It converts a torque command into a
    voltage, clipped to +/- V_supply. This yields a realistic torque-speed
    curve: flat (current-limited) region, then a linear roll-off to the
    no-load speed V/Ke.
    """
    def __init__(self, p: MotorParams):
        self.p = p
        self.V_cmd = 0.0

    def friction(self, Omega: float) -> float:
        p = self.p
        # Smooth sign() avoids stiffness at zero crossing
        return p.tau_coulomb * math.tanh(Omega / 0.5) + p.b_viscous * Omega

    def set_torque_command(self, tau_cmd: float, Omega: float, i_now: float):
        """Driver: compute the voltage needed to track the current command."""
        p = self.p
        i_cmd = np.clip(tau_cmd / p.Kt, -p.I_max, p.I_max)
        # Overspeed protection: refuse to accelerate past omega_max
        if abs(Omega) > p.omega_max and np.sign(i_cmd) == np.sign(Omega):
            i_cmd = 0.0
        # PI-free feedforward + proportional current loop (fast electrical)
        V = p.R * i_cmd + p.Ke * Omega + 5.0 * (i_cmd - i_now)
        self.V_cmd = float(np.clip(V, -p.V_supply, p.V_supply))

    def derivatives(self, i: float, Omega: float):
        p = self.p
        di = (self.V_cmd - p.R * i - p.Ke * Omega) / p.L
        tau_em = p.Kt * i
        tau_net = tau_em - self.friction(Omega)
        dOmega = tau_net / p.J_wheel
        return di, dOmega, tau_net

    # ---- analytic steady-state torque-speed curve --------------------------
    def torque_curve(self, Omega: np.ndarray) -> np.ndarray:
        """Max available net shaft torque vs. wheel speed (motoring)."""
        p = self.p
        i_volt_lim = (p.V_supply - p.Ke * np.abs(Omega)) / p.R
        i_avail = np.clip(i_volt_lim, 0.0, p.I_max)
        tau = p.Kt * i_avail - (p.tau_coulomb + p.b_viscous * np.abs(Omega))
        return np.maximum(tau, 0.0)

    @property
    def no_load_speed(self) -> float:
        return self.p.V_supply / self.p.Ke

# ---------------------------------------------------------------------------
# Gyro model
# ---------------------------------------------------------------------------

@dataclass
class GyroParams:
    arw_deg_rthr: float = 0.15         # angle random walk [deg/sqrt(hr)]
    bias_instab_deg_hr: float = 3.0    # bias instability (Gauss-Markov sigma) [deg/hr]
    bias_tau_s: float = 300.0          # Gauss-Markov correlation time [s]
    rrw_deg_hr_rthr: float = 2.0       # rate random walk [deg/hr/sqrt(hr)]
    turn_on_bias_deg_hr: float = 10.0  # 1-sigma constant turn-on bias [deg/hr]
    temp_coeff_deg_hr_C: float = 0.5   # bias temperature sensitivity [deg/hr/degC]
    T_ref_C: float = 25.0
    scale_factor_ppm: float = 300.0    # 1-sigma scale-factor error
    misalign_mrad: float = 1.0         # 1-sigma axis misalignment
    quant_rad_s: float = 1e-6          # LSB [rad/s]
    range_rad_s: float = math.radians(400.0)

class Gyro:
    D2R = math.pi / 180.0

    def __init__(self, p: GyroParams, rng: np.random.Generator):
        self.p, self.rng = p, rng
        dh = self.D2R / 3600.0                          # deg/hr -> rad/s
        self.sig_arw = p.arw_deg_rthr * self.D2R / 60.0 # rad/sqrt(s)
        self.sig_gm = p.bias_instab_deg_hr * dh
        self.sig_rrw = p.rrw_deg_hr_rthr * dh / 60.0    # rad/s/sqrt(s)
        self.k_temp = p.temp_coeff_deg_hr_C * dh

        self.b_const = rng.normal(0, p.turn_on_bias_deg_hr * dh, 3)
        self.b_gm = np.zeros(3)
        self.b_rrw = np.zeros(3)
        sf = rng.normal(0, p.scale_factor_ppm * 1e-6, 3)
        mis = rng.normal(0, p.misalign_mrad * 1e-3, (3, 3))
        np.fill_diagonal(mis, 0.0)
        self.M = np.eye(3) + np.diag(sf) + mis   # scale + misalignment matrix

    def true_bias(self, temp_C: float) -> np.ndarray:
        return (self.b_const + self.b_gm + self.b_rrw
                + self.k_temp * (temp_C - self.p.T_ref_C))

    def measure(self, omega_true: np.ndarray, dt: float, temp_C: float) -> np.ndarray:
        r = self.rng
        a = math.exp(-dt / self.p.bias_tau_s)
        self.b_gm = a * self.b_gm + self.sig_gm * math.sqrt(1 - a*a) * r.standard_normal(3)
        self.b_rrw += self.