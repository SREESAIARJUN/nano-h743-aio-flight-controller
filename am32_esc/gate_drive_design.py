#!/usr/bin/env python3
"""
Nano H743 AIO - AM32 ESC gate-drive design: FD6288Q + TPHR8504PL (Revision 9.0)
==============================================================================
Derives, from datasheet data, the three numbers the AM32 target depends on and
checks that am32_esc/am32_target.h agrees with them:

  1. Gate-resistor network (slew-rate tuning).  Sweeps Rg_on, the turn-off
     topology (single resistor, or Rg_on || Schottky + R_off) and an optional
     external Cgs.  Each candidate is simulated by integrating the gate charge
     curve (charge-based, so the Miller plateau is exact), at the fast corner
     for dv/dt, shoot-through and overshoot and at the slow corner for dead
     time.  The lowest-loss candidate that meets every constraint is selected.
  2. Dead time.  The FD6288Q already inserts a fixed 100-300 ns interlock
     dead time.  The MCU dead time (TIM1 BDTR.DTG / AT32 TMR1_BRK.DTC) only
     has to cover what the internal dead time does not:
        DT_mcu >= t_out(off -> Vth,hot,min | slow) + toff_prop_max
                  - (toff_prop_min + DT_int_min) - t_in(0 -> Vth,hot,min | fast)
                  + MT_max + guard
     It is then quantised to AT32F421 TMR1 ticks (120 MHz -> 8.333 ns) with the
     DTG encoding and must land in the linear range (DTG < 128).
  3. Supporting checks: FD6288Q VCC/UVLO window over 2S-4S, gate-oxide
     (VGSS) margin, driver dissipation, bootstrap capacitor hold time, the
     AM32 driving-brake dead-time override (it is OR-ed into the register),
     and the PWM-frequency / temperature-limit settings AM32 actually reads.

Model calibration: the same integrator is run on Toshiba's switching test
circuit (VGS 0/10 V, RGG 4.7 ohm, ID 50 A, VDD 20 V) and must reproduce the
datasheet ton/toff within the documented band, otherwise the run fails.

Usage:   python3 am32_esc/gate_drive_design.py [--no-header-check]
Writes:  am32_esc/gate_drive_report.json   (consumed by analysis/thermal_power_analysis.py)
Exit 1 on any failed check, including header/report disagreement.
"""
import argparse
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
HEADER = os.path.join(HERE, "am32_target.h")
EEPROM = os.path.join(HERE, "eeprom_settings.json")
PARM = os.path.join(REPO, "ardupilot", "defaults.parm")
REPORT = os.path.join(HERE, "gate_drive_report.json")

# ----------------------------------------------------------------- driver data
# Fortior FD6288 datasheet V1.6 (2023-08-31), covers FD6288Q QFN24 (LCSC C328453).
# Table 3-2/3-3/3-4 at TA=25 C, VCC=VBS=15 V.
DRV = {
    "vcc_min": 5.0, "vcc_max_rec": 20.0, "vcc_abs_max": 25.0,          # Table 3-1/3-2
    "uvlo_on_max": 5.0, "uvlo_on_typ": 4.6, "uvlo_off_max": 4.7,      # VCCUV+/-, VBSUV+/- (same)
    "vih_min": 2.7, "vil_max": 0.8,                                   # 3.3 V logic OK
    "in_pulldown_kohm": (140, 200, 260),
    "ton_typ": 300e-9, "ton_max": 450e-9,                             # turn-on propagation
    "toff_typ": 100e-9, "toff_max": 160e-9,                           # turn-off propagation
    "mt_max": 30e-9,                                                  # HO/LO delay matching
    "dt_int": (100e-9, 200e-9, 300e-9),                               # internal dead time min/typ/max
    "io_src": (1.1, 1.5, 1.9), "io_snk": (1.3, 1.8, 2.3),             # A min/typ/max, VCC 15 V, PW <= 10 us
    "voh_typ": 0.6, "voh_max": 0.9, "vol_typ": 0.3, "vol_max": 0.45,  # at 100 mA
    "iqbs_max": 270e-6, "iqcc_max": 500e-6,
    "vs_neg_op": -4.0, "dvs_dt_max": 50e9,                            # V, V/s
    "rth_ja": 42.0, "tj_max": 150.0,
    "inputs_active_high": True,                                       # Table 1-2 + section 1.3 "output in phase with input"
}
# Output resistances are NOT specified; derived from VOH/VOL at 100 mA.
DRV["r_src"] = {"slow": DRV["voh_max"] / 0.1, "typ": DRV["voh_typ"] / 0.1}   # 9.0 / 6.0 ohm
DRV["r_snk"] = {"slow": DRV["vol_max"] / 0.1, "typ": DRV["vol_typ"] / 0.1}   # 4.5 / 3.0 ohm
ASSUME = {
    "r_drv_fast_factor": 0.6,        # fast-corner driver R = 0.6 x typ (no min in datasheet)
    "toff_prop_min": 50e-9,          # no min in datasheet: 0.5 x typ
    "dt_guard": 25e-9,               # added on top of the computed requirement
    "l_gate_nH": 10.0,               # driver -> gate -> source -> driver COM loop, <= 5 mm routing
    "l_loop_nH": 3.0,                # HS-LS-ceramic commutation loop, swept 1.5..5 nH below
    "dvdt_emi_max": 5e9,             # V/s design target (FD6288Q abs max is 50 V/ns)
    "est_frac_max": 0.10,            # shoot-through energy <= 10 % of HS turn-on energy
    "vds_derate": 0.80,              # peak VDS <= 80 % VDSS
    "schottky_vf": 0.30,             # gate turn-off Schottky (RB521S30 class) knee
    "qgd_scaling": "sqrt",           # Qgd(V) = Qgd(20 V) * sqrt(V/20), capped at Qgd(20 V)
    "c_bs_eff_uF": 1.10,             # 2.2 uF 25 V X5R 0603 at ~17 V DC bias (Rev 9 review: was 1 uF)
    "vf_bs": 0.45,                   # bootstrap Schottky at refresh current
    "i_bs_leak": 100e-6,             # bootstrap Schottky reverse leakage at VR 20 V, 105 C (part requirement)
    "r_pd_hs": 100e3,                # HS gate-source pull-down (loads C_BS while HS is on)
    "r_pd_ls": 10e3,                 # LS gate-source pull-down
    "i_plateau_A": (10.0, 20.0, 35.0),  # hard-switched currents evaluated for the outgoing plateau;
                                     # below ~10 A turn-off is Coss/load-current limited, not gate limited
    "tj_loss_C": 100.0,              # junction temperature for switching-energy export (thermal model)
    "pole_pairs": 7,                 # 14-pole whoop/toothpick motor (defaults.parm)
    "tj_board_C": 105.0,             # board limit from analysis/thermal_power_analysis.py
    "tj_fd6288_max_C": 125.0,
    "loss_band": 0.10,               # selection: candidates within +10 % of the minimum feasible loss
}

# ---------------------------------------------------------------- MOSFET data
# Toshiba TPHR8504PL datasheet Rev.5.0 (2019-10-24).
FET = {
    "vdss": 40.0, "vgss": 20.0,
    "vth_min": 1.4, "vth_max": 2.4,               # VDS 10 V, ID 1 mA
    "vth_typ_25": 1.95, "vth_tc": -8.5e-3,        # Fig. 8.8 (typical), read: 1.95 V @25 C -> ~1.0 V @140 C
    "ciss": 7370e-12, "crss": 58e-12, "coss": 1930e-12,   # VDS 20 V
    "qg_10v": 103e-9, "qg_4v5": 49e-9, "qgs1": 25e-9, "qgd": 12.4e-9, "qsw": 23e-9,
    "v_plateau": 3.3,                              # Fig. 8.10 @ 50 A: reference point of the Qg curve
    # Miller plateau vs drain current at 25 C, typical Vth (Fig. 8.1 transfer curve; 10 A extrapolated
    # with the square-law fit through the 20 A / 35 A points). Shifted by the Vth tempco and by
    # (vth_min - vth_typ) for the min corner; the mobility drop with temperature, which raises the
    # plateau, is ignored (conservative for turn-off).
    "v_plateau_vs_I": ((10.0, 3.07), (20.0, 3.20), (35.0, 3.35), (50.0, 3.40)),
    "rg_int": (0.6, 1.1),                          # typ / max
    "k_sat": 12.8,                                 # A/V^2, square-law fit of Fig. 8.1 (20 A @ 3.2 V, Vth 1.95)
    "test": {"vgs": 10.0, "rgg": 4.7, "id": 50.0, "vdd": 20.0, "ton": 26e-9, "toff": 63e-9},
}
TJ_HOT = 125.0


def v_plateau(i_load, tj=25.0, corner="typ"):
    tab = FET["v_plateau_vs_I"]
    if i_load <= tab[0][0]:
        v = tab[0][1]
    elif i_load >= tab[-1][0]:
        v = tab[-1][1]
    else:
        for (i0, v0), (i1, v1) in zip(tab, tab[1:]):
            if i0 <= i_load <= i1:
                v = v0 + (v1 - v0) * (i_load - i0) / (i1 - i0)
                break
    v += FET["vth_tc"] * (tj - 25.0)
    if corner == "min":
        v += FET["vth_min"] - FET["vth_typ_25"]
    return v
VTH_HOT_MIN = FET["vth_min"] + FET["vth_tc"] * (TJ_HOT - 25.0)          # 0.55 V
VTH_HOT_TYP = FET["vth_typ_25"] + FET["vth_tc"] * (TJ_HOT - 25.0)

# ------------------------------------------------------------- MCU / AM32 data
MCU = {
    "part": "AT32F421K8U7", "am32_mcu": "MCU_AT421", "groups": ["HARDWARE_GROUP_AT_B", "HARDWARE_GROUP_AT_045"],
    "tim_clk_hz": 120e6,          # AM32 targets.h MCU_AT421 CPU_FREQUENCY_MHZ 120, TMR1->div = 0, CKD = 1
    "nominal_pwm_hz": 24e3,       # AM32 TIM1_AUTORELOAD = CPU_MHz*1e6/24000 - 1
}
BUS = {"2S_min": 6.0, "2S_full": 8.4, "4S_full": 16.8, "4S_HV": 17.4}

E12 = [0.0, 1.0, 2.2, 3.3, 4.7, 6.8, 10.0, 15.0, 22.0]
ROFF = [None, 0.0, 1.0, 2.2]      # None = single resistor (no diode)
CGS_EXT = [0.0, 1.0e-9, 2.2e-9]


# ------------------------------------------------------------ gate-charge curve
def qgd_at(vbus):
    return min(FET["qgd"], FET["qgd"] * math.sqrt(max(vbus, 0.1) / 20.0))


class Gate:
    """Piecewise gate-charge curve with optional external Cgs (in parallel with Cgs).
    Hard-switched: C_low = Qgs1/3.3 V up to the plateau vpl (current/temperature/corner
    dependent), Qgd(Vbus) on the plateau, C_high above it.  Soft-switched (VDS ~ 0 throughout):
    no plateau and Ciss(VDS~0) ~ C_high everywhere."""

    def __init__(self, vbus, cgs_ext=0.0, switching=True, vpl=None):
        self.vpl = FET["v_plateau"] if vpl is None else vpl
        c_low = FET["qgs1"] / FET["v_plateau"]
        c_high = (FET["qg_10v"] - FET["qgs1"] - FET["qgd"]) / (10.0 - FET["v_plateau"])
        self.c1 = (c_low if switching else c_high) + cgs_ext
        self.qgd = qgd_at(vbus) if switching else 0.0
        self.c3 = c_high + cgs_ext
        self.q1 = self.c1 * self.vpl
        self.q2 = self.q1 + self.qgd

    def v(self, q):
        if q <= self.q1:
            return q / self.c1
        if q <= self.q2:
            return self.vpl
        return self.vpl + (q - self.q2) / self.c3

    def q(self, v):
        if v <= self.vpl:
            return v * self.c1
        return self.q2 + (v - self.vpl) * self.c3


def off_current(v, r_drv, rg_int, rg_on, r_off, i_cap=float("inf")):
    """Gate discharge current at gate voltage v through driver sink -> (Rg_on || D+R_off) -> rg_int.
    The Schottky branch is a knee at Vf followed by R_off (closed-form piecewise-linear solve).
    The driver sink saturates at i_cap (FD6288Q IOL, Rev 9 review)."""
    if v <= 0.0:
        return 0.0
    rs = r_drv + rg_int
    if r_off is None or rg_on <= 0.0:
        return min(i_cap, v / (rs + rg_on))
    ro, vf = max(r_off, 0.05), ASSUME["schottky_vf"]
    i = v / (rs + rg_on)                       # diode off
    if i * rg_on <= vf:
        return min(i_cap, i)
    vb = (v + rs * vf / ro) / (1.0 + rs * (1.0 / rg_on + 1.0 / ro))
    return min(i_cap, (v - vb) / rs)


def simulate_on(vd, vbus, r_src, rg_int, rg_on, cgs_ext, dt=0.05e-9, i_cap=float("inf"), vpl=None, switching=True):
    g = Gate(vbus, cgs_ext, switching, vpl)
    q, t = 0.0, 0.0
    ev = {}
    r = r_src + rg_int + rg_on
    vth_ref = FET["vth_typ_25"]
    while True:
        v = g.v(q)
        if "vth_hot" not in ev and v >= VTH_HOT_MIN:
            ev["vth_hot"] = t
        if "vth_hot_typ" not in ev and v >= VTH_HOT_TYP:
            ev["vth_hot_typ"] = t
        if "vth" not in ev and v >= vth_ref:
            ev["vth"] = t
        if "pl_start" not in ev and q >= g.q1:
            ev["pl_start"] = t
        if "pl_end" not in ev and q >= g.q2:
            ev["pl_end"] = t
        if v >= 0.9 * vd:
            ev["v90"] = t
            break
        q += min(i_cap, (vd - v) / r) * dt
        t += dt
        if t > 5e-6:
            raise RuntimeError("turn-on did not converge")
    ev.setdefault("vth", ev["pl_start"])
    ev["t_ri"] = ev["pl_start"] - ev["vth"]
    ev["t_vf"] = ev["pl_end"] - ev["pl_start"]
    return ev


def simulate_off(vd, vbus, r_snk, rg_int, rg_on, r_off, cgs_ext, v_stop, switching=True, dt=0.05e-9,
                 i_cap=float("inf"), vpl=None):
    g = Gate(vbus, cgs_ext, switching, vpl)
    q, t = g.q(vd), 0.0
    ev = {}
    while True:
        v = g.v(q)
        if "pl_start" not in ev and q <= g.q2:
            ev["pl_start"] = t
        if "pl_end" not in ev and q <= g.q1:
            ev["pl_end"] = t
        if v <= v_stop:
            ev["v_stop"] = t
            break
        q -= off_current(v, r_snk, rg_int, rg_on, r_off, i_cap) * dt
        t += dt
        if t > 5e-6:
            raise RuntimeError("turn-off did not converge")
    ev["t_vr"] = ev["pl_end"] - ev["pl_start"]
    return ev


def induced(vbus, t_vf, r_snk, rg_int, rg_on, r_off, cgs_ext, dt=0.05e-9, i_cap=float("inf")):
    """Cdv/dt charge injected into the held-off complementary gate during the
    switch-node transition; returns peak Vgs and shoot-through charge."""
    q_inj = qgd_at(vbus)
    c = FET["qgs1"] / FET["v_plateau"] + cgs_ext
    i_inj = q_inj / t_vf
    v, t, vpk, qst, ipk = 0.0, 0.0, 0.0, 0.0, 0.0
    while t < t_vf + 400e-9:
        inj = i_inj if t < t_vf else 0.0
        v += (inj - off_current(v, r_snk, rg_int, rg_on, r_off, i_cap)) / c * dt
        vpk = max(vpk, v)
        if v > VTH_HOT_MIN:
            ist = FET["k_sat"] * (v - VTH_HOT_MIN) ** 2
            qst += ist * dt
            ipk = max(ipk, ist)
        t += dt
    return vpk, qst, ipk


# ----------------------------------------------------------------- evaluation
def corner_r(corner):
    if corner == "fast":
        return (DRV["r_src"]["typ"] * ASSUME["r_drv_fast_factor"],
                DRV["r_snk"]["typ"] * ASSUME["r_drv_fast_factor"], FET["rg_int"][0])
    if corner == "typ":
        return DRV["r_src"]["typ"], DRV["r_snk"]["typ"], FET["rg_int"][0]
    return DRV["r_src"]["slow"], DRV["r_snk"]["slow"], FET["rg_int"][1]


def corner_caps(corner, vcc):
    """FD6288Q output current limits (IOH/IOL, specified at VCC 15 V).  The slow corner uses the
    minimum and scales it down with VCC below 15 V; fast uses the maximum unscaled."""
    k = {"slow": 0, "typ": 1, "fast": 2}[corner]
    scale = min(1.0, vcc / 15.0) if corner != "fast" else 1.0
    return DRV["io_src"][k] * scale, DRV["io_snk"][k] * scale


def qg_total(v, cgs=0.0):
    """Total gate charge at drive voltage v (linear above the plateau) incl. external Cgs."""
    return (FET["qg_10v"] + (v - 10.0) * (FET["qg_10v"] - FET["qgs1"] - FET["qgd"]) / (10.0 - FET["v_plateau"])
            + cgs * v)


def fd6288_tj(rg_on, r_off, cgs, f_pwm, vbus=None, t_board=None):
    """FD6288Q junction temperature. AM32 6-step with complementary PWM: in every 60-degree
    step only the PWM phase's HS and LS toggle at f_pwm (2 gates); the sink phase LS is static
    and commutation switching at the eRPM rate is negligible. The gate energy Qg*V splits between
    driver output resistance and the external path in proportion to resistance."""
    vbus = BUS["4S_HV"] if vbus is None else vbus
    t_board = ASSUME["tj_board_C"] if t_board is None else t_board
    p_gate = 2 * qg_total(vbus, cgs) * vbus * f_pwm
    r_ext_off = rg_on if r_off is None else min(rg_on, r_off + 0.5)
    share_on = DRV["r_src"]["slow"] / (DRV["r_src"]["slow"] + rg_on + FET["rg_int"][0])
    share_off = DRV["r_snk"]["slow"] / (DRV["r_snk"]["slow"] + r_ext_off + FET["rg_int"][0])
    p_ic = 0.5 * p_gate * (share_on + share_off) + vbus * (DRV["iqcc_max"] + 3 * DRV["iqbs_max"])
    return {"p_gate_total_W": p_gate, "p_fd6288_W": p_ic, "tj_fd6288_C": t_board + p_ic * DRV["rth_ja"]}


def switching_energy(rg_on, r_off, cgs, vbus, i_load, vd=None):
    """HS hard-switching energies for the loss model: slow driver (R max, IOH/IOL min), typical
    Vth at ASSUME['tj_loss_C'], plateau at the load current.  Shoot-through energy from the
    fast-corner Miller turn-on of the complementary FET (see stress())."""
    vd = vbus if vd is None else vd
    rs, rk, rgi = corner_r("slow")
    i_src, i_snk = corner_caps("slow", vd)
    vpl = v_plateau(i_load, ASSUME["tj_loss_C"], "typ")
    vth = FET["vth_typ_25"] + FET["vth_tc"] * (ASSUME["tj_loss_C"] - 25.0)
    on = simulate_on(vd, vbus, rs, rgi, rg_on, cgs, i_cap=i_src, vpl=vpl)
    off = simulate_off(vd, vbus, rk, rgi, rg_on, r_off, cgs, vth, i_cap=i_snk, vpl=vpl)
    # current rise: Vth -> plateau; voltage fall: plateau.  Turn-off mirrors it.
    t_on_ev = simulate_on(vd, vbus, rs, rgi, rg_on, cgs, i_cap=i_src, vpl=vpl)
    t_ri = t_on_ev["pl_start"] - (t_on_ev.get("vth_hot_typ") if ASSUME["tj_loss_C"] >= 100 else t_on_ev["vth"])
    e_on = 0.5 * vbus * i_load * (max(t_ri, 0.0) + on["t_vf"])
    t_fi = off["v_stop"] - off["pl_end"]
    e_off = 0.5 * vbus * i_load * (off["t_vr"] + t_fi)
    return e_on, e_off


def stress(rg_on, r_off, cgs, vbus, i_load):
    """Fast corner (driver R 0.6 x typ, IOH/IOL max, rg_int typ on the switching FET, hot Vth min)
    for dv/dt, Miller turn-on and di/dt overshoot.  The victim (held-off) FET is on the same
    FD6288Q die (fast sink) but is a separate MOSFET, so it gets the datasheet max rg_int."""
    vd = vbus
    rs_f, rk_f, rgi_f = corner_r("fast")
    i_src, i_snk = corner_caps("fast", vd)
    vpl = v_plateau(i_load, TJ_HOT, "min")
    on_f = simulate_on(vd, vbus, rs_f, rgi_f, rg_on, cgs, i_cap=i_src, vpl=vpl)
    off_f = simulate_off(vd, vbus, rk_f, rgi_f, rg_on, r_off, cgs, VTH_HOT_MIN, i_cap=i_snk, vpl=vpl)
    vpk, qst, ipk = induced(vbus, on_f["t_vf"], rk_f, FET["rg_int"][1], rg_on, r_off, cgs, i_cap=i_snk)
    t_fi_f = max(off_f["v_stop"] - off_f["pl_end"], 1e-12)
    return {"dvdt": vbus / on_f["t_vf"], "vpk": vpk, "e_st": vbus * qst, "ipk": ipk, "t_fi": t_fi_f,
            "v_os": ASSUME["l_loop_nH"] * 1e-9 * i_load / t_fi_f}


def evaluate(rg_on, r_off, cgs, vbus, i_load, fsw):
    e_on, e_off = switching_energy(rg_on, r_off, cgs, vbus, i_load)
    st = stress(rg_on, r_off, cgs, vbus, i_load)
    p_sw = (e_on + e_off + st["e_st"]) * fsw / 3.0     # 120-degree PWM window, as the thermal model
    return {
        "rg_on": rg_on, "r_off": r_off, "cgs_ext_nF": cgs * 1e9, "vbus": vbus, "i": i_load,
        "dvdt_V_per_ns": st["dvdt"] * 1e-9, "induced_vgs_pk_V": st["vpk"], "shoot_through_pk_A": st["ipk"],
        "e_on_uJ": e_on * 1e6, "e_off_uJ": e_off * 1e6, "e_st_uJ": st["e_st"] * 1e6,
        "est_frac": st["e_st"] / e_on if e_on > 0 else 0.0,
        "t_fall_fast_ns": st["t_fi"] * 1e9, "v_overshoot_V": st["v_os"], "vds_peak_V": vbus + st["v_os"],
        "p_sw_W": p_sw,
        "tj_fd6288_24k_C": fd6288_tj(rg_on, r_off, cgs, 24e3)["tj_fd6288_C"],
        "tj_fd6288_48k_C": fd6288_tj(rg_on, r_off, cgs, 48e3)["tj_fd6288_C"],
    }


def feasible(e):
    return (e["tj_fd6288_24k_C"] <= ASSUME["tj_fd6288_max_C"]
            and e["dvdt_V_per_ns"] <= ASSUME["dvdt_emi_max"] * 1e-9
            and e["est_frac"] <= ASSUME["est_frac_max"]
            and e["vds_peak_V"] <= ASSUME["vds_derate"] * FET["vdss"])


def dtg_encode(ticks):
    """Smallest DTG code whose dead time >= ticks*tDTS (STM32 BDTR / AT32 TMR1_BRK.DTC)."""
    for code in range(256):
        if dtg_decode(code) >= ticks:
            return code
    raise ValueError("dead time out of range")


def dtg_decode(code):
    if code < 0x80:
        return code
    if code < 0xC0:
        return (64 + (code & 0x3F)) * 2
    if code < 0xE0:
        return (32 + (code & 0x1F)) * 8
    return (32 + (code & 0x1F)) * 16


def parse_header(path):
    defs = {}
    blocks = []
    with open(path) as f:
        for line in f:
            s = line.split("//")[0].strip()
            m = re.match(r"#define\s+(\w+)(?:\s+(.*))?$", s)
            if m:
                defs[m.group(1)] = (m.group(2) or "").strip()
            m = re.match(r"#ifdef\s+(\w+)", s)
            if m:
                blocks.append(m.group(1))
    return defs, blocks


# ---------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-header-check", action="store_true")
    ap.add_argument("--out", default=REPORT)
    a = ap.parse_args()
    fails, checks = [], []

    def check(name, ok, detail):
        checks.append({"check": name, "pass": bool(ok), "detail": detail})
        if not ok:
            fails.append(name)

    # ---- C0: model calibration against Toshiba's switching test circuit
    t = FET["test"]
    cal_on = simulate_on(t["vgs"], t["vdd"], 0.0, FET["rg_int"][0], t["rgg"], 0.0)
    cal_off = simulate_off(t["vgs"], t["vdd"], 0.0, FET["rg_int"][0], t["rgg"], None, 0.0, FET["vth_typ_25"])
    ton_m = cal_on["pl_end"]
    toff_m = cal_off["v_stop"]
    r_on, r_off_ratio = ton_m / t["ton"], toff_m / t["toff"]
    check("C0_model_calibration", 0.7 <= r_on <= 2.0 and 1.0 <= r_off_ratio <= 2.0,
          f"model ton {ton_m*1e9:.1f} ns vs datasheet {t['ton']*1e9:.0f} ns (x{r_on:.2f}); "
          f"toff {toff_m*1e9:.1f} ns vs {t['toff']*1e9:.0f} ns (x{r_off_ratio:.2f}); turn-off must not be optimistic")

    # ---- slew-rate sweep at the stress point (4S HV, 35 A burst) + loss point (4S, 20 A, 24 kHz)
    fsw = MCU["nominal_pwm_hz"]
    sweep = []
    for cgs in CGS_EXT:
        for roff in ROFF:
            for rg in E12:
                if roff is None and rg == 0.0 and cgs == 0.0:
                    pass
                if roff is not None and rg == 0.0:
                    continue            # diode in parallel with 0 ohm is pointless
                st_pt = evaluate(rg, roff, cgs, BUS["4S_HV"], 35.0, fsw)
                loss = evaluate(rg, roff, cgs, BUS["4S_full"], 20.0, fsw)
                ok = feasible(st_pt)
                sweep.append({"rg_on": rg, "r_off": roff, "cgs_ext_nF": cgs * 1e9, "feasible": ok,
                              "stress_4S_HV_35A": st_pt,
                              "p_sw_4S_20A_24kHz_W": loss["p_sw_W"]})
    feas = [s for s in sweep if s["feasible"]]
    check("C1_feasible_gate_network_exists", bool(feas), f"{len(feas)}/{len(sweep)} candidates meet dv/dt, shoot-through and VDS limits")
    if not feas:
        json.dump({"status": "FAIL", "checks": checks, "sweep": sweep}, open(a.out, "w"), indent=1)
        print("FAIL: no feasible gate network"); return 1
    # Selection rule: the feasible loss landscape is flat, so among candidates within
    # LOSS_BAND of the minimum loss pick the fewest parts per FET, then the lowest dv/dt
    # (EMI next to the IMUs on an AIO board), then the lowest loss.
    def parts(s_):
        # every gate keeps a series-resistor footprint (fitted 0 ohm if Rg_on = 0) so slew rate can
        # be re-tuned at bring-up; the turn-off Schottky (+ R_off) and Cgs_ext add parts
        return 1 + (0 if s_["r_off"] is None else 1 + (s_["r_off"] > 0)) + (s_["cgs_ext_nF"] > 0)

    def cost(s_):
        return (parts(s_), round(s_["stress_4S_HV_35A"]["dvdt_V_per_ns"], 2), s_["p_sw_4S_20A_24kHz_W"])
    p_min = min(s_["p_sw_4S_20A_24kHz_W"] for s_ in feas)
    short = [s_ for s_ in feas if s_["p_sw_4S_20A_24kHz_W"] <= (1.0 + ASSUME["loss_band"]) * p_min]
    best = min(short, key=cost)
    rg_on, r_off, cgs = best["rg_on"], best["r_off"], best["cgs_ext_nF"] * 1e-9
    sel = evaluate(rg_on, r_off, cgs, BUS["4S_HV"], 35.0, fsw)
    # loss-optimal infeasible point, to show what the constraints cost
    unconstrained = min(sweep, key=lambda s: s["p_sw_4S_20A_24kHz_W"])

    # overshoot sensitivity to loop inductance
    os_sweep = {f"{L:.1f}nH": BUS["4S_HV"] + L * 1e-9 * 35.0 / (sel["t_fall_fast_ns"] * 1e-9) for L in (1.5, 3.0, 5.0)}
    check("C2_vds_peak_5nH", os_sweep["5.0nH"] <= ASSUME["vds_derate"] * FET["vdss"],
          f"VDS peak at 35 A turn-off, L_loop 1.5/3/5 nH: " + ", ".join(f"{k} {v:.1f} V" for k, v in os_sweep.items())
          + f" (limit {ASSUME['vds_derate']*FET['vdss']:.0f} V)")
    check("C3_dvdt_vs_FD6288_abs", sel["dvdt_V_per_ns"] * 1e9 <= DRV["dvs_dt_max"],
          f"{sel['dvdt_V_per_ns']:.2f} V/ns vs FD6288Q dVS/dt 50 V/ns")

    # ---- gate-loop damping: with a small or 0 ohm Rg the gate loop (driver -> trace -> Cgs) must
    # not ring: zeta = R/2 * sqrt(C/L) >= 1 on the fast source and fast sink paths.
    c_g = FET["qgs1"] / FET["v_plateau"] + cgs
    zeta = {}
    for path, r_drv in (("source", corner_r("fast")[0]), ("sink", corner_r("fast")[1])):
        r_path = r_drv + FET["rg_int"][0] + (rg_on if (path == "source" or r_off is None) else min(rg_on, r_off))
        zeta[path] = r_path / 2.0 * math.sqrt(c_g / (ASSUME["l_gate_nH"] * 1e-9))
    check("C9_gate_loop_damping", min(zeta.values()) >= 1.0,
          f"zeta source {zeta['source']:.2f}, sink {zeta['sink']:.2f} (L_gate {ASSUME['l_gate_nH']} nH, "
          f"C {c_g*1e9:.1f} nF, fast driver R + rg_int typ + external path)")

    # ---- dead time.  For each transition the OUTGOING device may be hard-switched (its own
    # current commutates: HS for positive phase current, LS for negative phase current during
    # deceleration / braking / ripple below zero) or soft.  All four combinations, both bus
    # corners and the gate-limited currents are evaluated; the worst sets DEAD_TIME.
    # Outgoing: slow driver (R max, IOL min scaled by VCC/15), rg_int max, plateau at Tj 125 C /
    # Vth min, declared off at Vth,min(125 C).  Incoming: fast driver, reaches Vth,min(125 C).
    dt_rows = []
    rs_f, rk_f, rgi_f = corner_r("fast")
    rs_s, rk_s, rgi_s = corner_r("slow")
    for name, vb in (("4S_HV", BUS["4S_HV"]), ("2S_min", BUS["2S_min"])):
        for trans in ("HS_off->LS_on", "LS_off->HS_on"):
            vd_out = vb if trans.startswith("LS") else vb - ASSUME["vf_bs"]
            vd_in = vb - ASSUME["vf_bs"] if trans.endswith("HS_on") else vb
            _, isnk_s = corner_caps("slow", vd_out)
            isrc_f, _ = corner_caps("fast", vd_in)
            inn = simulate_on(vd_in, vb, rs_f, rgi_f, rg_on, cgs, i_cap=isrc_f)
            t_in = ASSUME["toff_prop_min"] + DRV["dt_int"][0] + inn["vth_hot"]
            cases = [("soft", None)] + [("hard", i_) for i_ in ASSUME["i_plateau_A"]]
            for kind, i_l in cases:
                vpl = None if i_l is None else v_plateau(i_l, TJ_HOT, "min")
                out = simulate_off(vd_out, vb, rk_s, rgi_s, rg_on, r_off, cgs, VTH_HOT_MIN,
                                   switching=(kind == "hard"), i_cap=isnk_s, vpl=vpl)
                t_out = DRV["toff_max"] + out["v_stop"]
                need = t_out - t_in + DRV["mt_max"] + ASSUME["dt_guard"]
                dt_rows.append({"bus": name, "transition": trans, "outgoing": kind, "i_A": i_l,
                                "v_plateau_V": vpl, "t_outgoing_below_vth_ns": t_out * 1e9,
                                "t_incoming_at_vth_ns_without_mcu_dt": t_in * 1e9,
                                "dt_mcu_required_ns": max(0.0, need) * 1e9})
    worst = max(dt_rows, key=lambda r: r["dt_mcu_required_ns"])
    dt_need = worst["dt_mcu_required_ns"] * 1e-9
    tick = 1.0 / MCU["tim_clk_hz"]
    ticks = math.ceil(dt_need / tick - 1e-9)
    dtg = dtg_encode(ticks)
    dt_mcu = dtg_decode(dtg) * tick
    check("C4_dtg_linear_range", dtg < 0x80,
          f"DEAD_TIME = {dtg} (0x{dtg:02X}) -> {dt_mcu*1e9:.1f} ns, need {dt_need*1e9:.1f} ns "
          f"(worst: {worst['bus']} {worst['transition']} {worst['outgoing']} {worst['i_A'] or ''} A)")

    # Body-diode conduction per edge (thermal model input) = outgoing channel off -> incoming
    # channel on, both from the MCU edge.  typ: typical delays, R, caps, Vth(125 C) typ.
    # max: outgoing fastest (toff_prop_min, fast driver) vs incoming slowest (ton_max + MT,
    # slow driver, Vth typ at 25 C) -> the longest conduction.
    def diode_window(vb, corner):
        if corner == "typ":
            rs, rk, rgi = corner_r("typ")
            isrc, isnk = corner_caps("typ", vb)
            out = DRV["toff_typ"] + simulate_off(vb, vb, rk, rgi, rg_on, r_off, cgs, VTH_HOT_TYP, i_cap=isnk,
                                                 vpl=v_plateau(20.0, TJ_HOT, "typ"))["v_stop"]
            inn = dt_mcu + DRV["ton_typ"] + simulate_on(vb, vb, rs, rgi, rg_on, cgs, i_cap=isrc)["vth_hot_typ"]
        else:
            _, isnk = corner_caps("fast", vb)
            isrc, _ = corner_caps("slow", vb)
            out = ASSUME["toff_prop_min"] + simulate_off(vb, vb, rk_f, rgi_f, rg_on, r_off, cgs, VTH_HOT_TYP,
                                                         i_cap=isnk, vpl=v_plateau(35.0, TJ_HOT, "typ"))["v_stop"]
            inn = dt_mcu + DRV["ton_max"] + DRV["mt_max"] + simulate_on(vb, vb, rs_s, rgi_s, rg_on, cgs, i_cap=isrc)["vth"]
        return max(0.0, inn - out)
    dt_diode_typ = max(diode_window(v, "typ") for v in (BUS["4S_full"], BUS["2S_full"]))
    dt_diode_max = max(diode_window(v, "max") for v in (BUS["4S_full"], BUS["2S_full"]))
    arr = int(MCU["tim_clk_hz"] / MCU["nominal_pwm_hz"]) - 1

    # driving-brake override: dead_time_override = DEAD_TIME + (150 - 10*s), capped 200, then BDTR |= override
    brake = []
    for s in range(1, 10):
        ov = min(dtg + 150 - 10 * s, 200)
        reg = dtg | ov
        brake.append({"driving_brake_strength": s, "override": ov, "register": reg,
                      "dead_time_ns": dtg_decode(reg) * tick * 1e9})

    # ---- supply window, VGSS, driver power, bootstrap
    vcc_lo = BUS["2S_min"]
    vbs_lo = vcc_lo - ASSUME["vf_bs"]
    check("C5_vcc_uvlo_window", vbs_lo > DRV["uvlo_on_max"] and BUS["4S_HV"] <= DRV["vcc_max_rec"],
          f"VCC = VBAT 6.0..17.4 V; VBS at 2S/3.0 V-cell = {vbs_lo:.2f} V > UVLO+ max {DRV['uvlo_on_max']} V; "
          f"4S HV 17.4 V <= VCC rec. max {DRV['vcc_max_rec']} V")
    vgs_hs_worst = BUS["4S_HV"] - ASSUME["vf_bs"] + abs(DRV["vs_neg_op"])
    check("C6_vgss_margin", BUS["4S_HV"] <= FET["vgss"] - 2.0,
          f"LS VGS max 17.4 V vs VGSS 20 V; HS VBS may pump to {vgs_hs_worst:.1f} V on a -4 V VS undershoot "
          f"-> 2.2 ohm in series with each bootstrap diode + 18 V VCC Zener are required (see report 'requirements')")
    qg = lambda v: qg_total(v, cgs)
    drv_rows = {f"{int(f_k/1e3)}kHz": fd6288_tj(rg_on, r_off, cgs, f_k) for f_k in (24e3, 48e3)}
    check("C7_fd6288_tj", drv_rows["24kHz"]["tj_fd6288_C"] <= ASSUME["tj_fd6288_max_C"],
          f"FD6288Q Tj at board {ASSUME['tj_board_C']:.0f} C: 24 kHz {drv_rows['24kHz']['tj_fd6288_C']:.1f} C, "
          f"48 kHz {drv_rows['48kHz']['tj_fd6288_C']:.1f} C (limit {ASSUME['tj_fd6288_max_C']:.0f} C at 24 kHz; "
          f"48 kHz {'allowed' if drv_rows['48kHz']['tj_fd6288_C'] <= DRV['tj_max'] - 10 else 'NOT allowed'} vs 140 C)")
    # bootstrap: HS held on with no refresh for 1/3 electrical period at 100 % duty.  C_BS
    # supplies IQBS + the bootstrap Schottky leakage + the HS gate pull-down (Rev 9 review):
    # dV/dt = -(I_q + V/R_pd)/C  ->  t = R C ln((V0 + I_q R)/(V_min + I_q R)).
    c_bs = ASSUME["c_bs_eff_uF"] * 1e-6
    i_q = DRV["iqbs_max"] + ASSUME["i_bs_leak"]
    r_pd = ASSUME["r_pd_hs"]
    v_floor = max(DRV["uvlo_on_max"], 4.5 + 0.5)          # keep VGS >= 5 V (RDS(on) spec at 4.5 V + 0.5)
    bs = {}
    for name, vb in (("2S_min", BUS["2S_min"]), ("3S_nom", 11.1), ("4S_full", BUS["4S_full"])):
        dv_step = qg(vb) / c_bs
        v0 = vb - ASSUME["vf_bs"] - dv_step
        t_hold = r_pd * c_bs * math.log((v0 + i_q * r_pd) / (v_floor + i_q * r_pd)) if v0 > v_floor else 0.0
        erpm_min = (1.0 / (3.0 * t_hold)) * 60.0 if t_hold > 0 else float("inf")
        bs[name] = {"v_after_turn_on_V": v0, "v_floor_V": v_floor, "dv_charge_share_V": dv_step,
                    "i_static_uA": (i_q + v0 / r_pd) * 1e6, "max_hs_on_ms": t_hold * 1e3,
                    "min_erpm_at_100pct_duty": erpm_min,
                    "min_mech_rpm_at_100pct_duty": erpm_min / ASSUME["pole_pairs"]}
    check("C8_bootstrap_hold", bs["2S_min"]["max_hs_on_ms"] > 0.5 and bs["4S_full"]["max_hs_on_ms"] > 10,
          f"C_BS {ASSUME['c_bs_eff_uF']} uF eff, IQBS+leak {i_q*1e6:.0f} uA, HS pull-down {r_pd/1e3:.0f} k; "
          "max HS on-time without refresh: " + ", ".join(f"{k} {v['max_hs_on_ms']:.2f} ms" for k, v in bs.items()))

    # ---- header consistency
    hdr = {}
    if not a.no_header_check:
        defs, blocks = parse_header(HEADER)
        hdr = {"DEAD_TIME": defs.get("DEAD_TIME"), "blocks": blocks}
        check("H1_header_dead_time", defs.get("DEAD_TIME") == str(dtg),
              f"am32_target.h DEAD_TIME {defs.get('DEAD_TIME')} vs computed {dtg}")
        groups_ok = all(gname in defs for gname in MCU["groups"])
        check("H2_header_hw_group", groups_ok, f"requires {MCU['groups']}")
        inv = [m for m in ("USE_INVERTED_LOW", "USE_INVERTED_HIGH", "PWM_ENABLE_BRIDGE", "USE_OPEN_DRAIN_LOW",
                           "USE_OPEN_DRAIN_HIGH", "OPEN_DRAIN_PWM") if m in defs]
        check("H3_header_polarity", DRV["inputs_active_high"] and not inv,
              f"FD6288Q HIN/LIN active-high, so no inversion/bridge macros may be set (found: {inv or 'none'})")
        fn = defs.get("FIRMWARE_NAME", '""').strip('"')
        fl = defs.get("FILE_NAME", '""').strip('"')
        check("H4_header_names", len(fn) <= 12 and fl.endswith("_F421") and fl in blocks,
              f"FIRMWARE_NAME '{fn}' ({len(fn)}/12 chars, AM32 _Static_assert), FILE_NAME '{fl}' must end _F421 and gate its block")
        vd = defs.get("TARGET_VOLTAGE_DIVIDER")
        check("H5_header_voltage_divider", vd == "110",
              f"TARGET_VOLTAGE_DIVIDER {vd}: 100k/10k divider -> ratio 11 -> AM32 units x10 = 110; full scale 36.3 V")
        txt = open(HEADER).read()
        m7 = re.search(r"Body-diode conduction per edge: (\d+) ns typ / (\d+) ns max", txt)
        m8 = re.search(r"\b%d -> ([0-9.]+) ns MCU dead time" % dtg, txt)
        ok7 = (m7 is not None and int(m7.group(1)) == round(dt_diode_typ * 1e9) and int(m7.group(2)) == round(dt_diode_max * 1e9)
               and m8 is not None and abs(float(m8.group(1)) - dt_mcu * 1e9) < 0.05)
        check("H7_header_comments", ok7,
              f"header comment must quote '{dtg} -> {dt_mcu*1e9:.1f} ns MCU dead time' and body-diode "
              f"{round(dt_diode_typ*1e9)} ns typ / {round(dt_diode_max*1e9)} ns max")
        bad = [m for m in ("DEAD_TIME_NS", "PWM_FREQ_KHZ", "TEMPERATURE_CUTOFF_C", "BIDIRECTIONAL_DSHOT",
                           "SENSE_SHUNT_MILLIOHM", "OVER_CURRENT_LIMIT_AMPS", "DSHOT_TELEMETRY") if m in defs]
        check("H6_no_dead_macros", not bad, f"macros AM32 never reads: {bad or 'none'}")

    # ---- EEPROM settings the analysis depends on
    ee = {e["field"]: e["value"] for e in json.load(open(EEPROM))["settings"]}
    parm = {}
    for line in open(PARM):
        m = re.match(r"\s*([A-Z0-9_]+)\s+(-?[0-9.]+)", line)
        if m:
            parm[m.group(1)] = float(m.group(2))
    poles_parm = parm.get("SERVO_BLH_POLES")
    e_ok = (ee.get("comp_pwm") == 1 and ee.get("variable_pwm") == 0
            and ee.get("pwm_frequency") == int(MCU["nominal_pwm_hz"] / 1e3)
            and ee.get("driving_brake_strength") == 10
            and ee.get("low_voltage_cut_off") == 0
            and 70 <= ee.get("limits.temperature", 0) <= 140
            and poles_parm is not None and ee.get("motor_poles") == int(poles_parm) == 2 * ASSUME["pole_pairs"])
    check("E1_eeprom_settings", e_ok,
          f"comp_pwm {ee.get('comp_pwm')}, variable_pwm {ee.get('variable_pwm')}, pwm {ee.get('pwm_frequency')} kHz, "
          f"driving_brake {ee.get('driving_brake_strength')}, low_voltage_cut_off {ee.get('low_voltage_cut_off')} "
          f"(AM32 cutoff latches allOff() per ESC until power cycle -> must be 0), temp limit "
          f"{ee.get('limits.temperature')} C, poles {ee.get('motor_poles')} (defaults.parm {poles_parm})")
    # Minimum on-time: AM32 duty is 0..2000 of ARR (main.c:1517 adjusted = duty*arr/2000 + 1); the
    # floor is EEPROM byte 6 x 10 (main.c:645).  A commanded pulse shorter than DT_mcu + the FD6288Q
    # internal dead time (max) is swallowed whole, so the floor must exceed it.
    md = ee.get("minimum_duty_cycle", 0)
    t_min_on = (md * 10 * (arr + 1) / 2000 + 1) * tick
    t_swallow = dt_mcu + DRV["dt_int"][2] + DRV["mt_max"]
    md_need = math.ceil((t_swallow / tick - 1) * 2000 / (arr + 1) / 10)
    check("E2_min_on_time", t_min_on > t_swallow,
          f"minimum_duty_cycle {md} -> {md*10}/2000 -> {t_min_on*1e9:.0f} ns on-time vs DT_mcu + DT_int,max + MT "
          f"{t_swallow*1e9:.0f} ns (needs >= {md_need})")
    # FD6288Q UVLO floor enforced by the FC (the ESC-level cutoff is off): BATT_CRT_VOLT must keep
    # VBS = VBAT - Vf_bs above UVLO+ max with sag margin, and the critical failsafe must act.
    v_floor_bat = DRV["uvlo_on_max"] + ASSUME["vf_bs"] + 0.5
    crt = parm.get("BATT_CRT_VOLT", 0.0)
    check("E3_fc_uvlo_failsafe", crt >= v_floor_bat and parm.get("BATT_FS_CRT_ACT", 0) > 0,
          f"defaults.parm BATT_CRT_VOLT {crt} V >= {v_floor_bat:.2f} V (UVLO+ max {DRV['uvlo_on_max']} + Vf_bs "
          f"{ASSUME['vf_bs']} + 0.5 V sag), BATT_FS_CRT_ACT {parm.get('BATT_FS_CRT_ACT')} != 0")

    # ---- switching-energy table for analysis/thermal_power_analysis.py (selected network)
    e_table = {"i_A": [5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0], "rows": []}
    for vb in (BUS["2S_full"], BUS["4S_full"]):
        row = {"vbus": vb, "e_on_uJ": [], "e_off_uJ": [], "e_st_uJ": []}
        for i_ in e_table["i_A"]:
            e_on, e_off = switching_energy(rg_on, r_off, cgs, vb, i_)
            row["e_on_uJ"].append(e_on * 1e6)
            row["e_off_uJ"].append(e_off * 1e6)
            row["e_st_uJ"].append(stress(rg_on, r_off, cgs, vb, i_)["e_st"] * 1e6)
        e_table["rows"].append(row)

    report = {
        "revision": "9.0",
        "sources": {
            "FD6288Q": "Fortior FD6288 datasheet V1.6 (2023-08-31), Tables 3-1..3-4, LCSC C328453",
            "TPHR8504PL": "Toshiba TPHR8504PL datasheet Rev.5.0 (2019-10-24), sections 4-6, Figs 8.1/8.8/8.10",
            "AM32": "am32-firmware/AM32 3d8cd50 (v2.21): Inc/targets.h, Src/main.c:419,536,637-648,699-711, Mcu/f421/Src/peripherals.c:97-115",
        },
        "assumptions": ASSUME, "driver": DRV, "fet": {k: v for k, v in FET.items()},
        "vth_hot_min_V": VTH_HOT_MIN, "vth_hot_typ_V": VTH_HOT_TYP,
        "calibration": {"ton_model_ns": ton_m * 1e9, "toff_model_ns": toff_m * 1e9},
        "selected_gate_network": {
            "rg_on_ohm": rg_on, "turn_off": ("single resistor" if r_off is None else f"Schottky + {r_off} ohm in parallel with Rg_on"),
            "r_off_ohm": r_off, "cgs_ext_nF": cgs * 1e9, "stress_4S_HV_35A": {k: v for k, v in sel.items() if not k.startswith("_")},
            "p_sw_per_HS_4S_20A_24kHz_W": best["p_sw_4S_20A_24kHz_W"], "min_feasible_loss_W": p_min,
            "selection_rule": "within +10 % of min feasible loss -> fewest parts/FET -> lowest dv/dt -> lowest loss",
            "unconstrained_loss_optimum": {"rg_on": unconstrained["rg_on"], "r_off": unconstrained["r_off"],
                                           "cgs_ext_nF": unconstrained["cgs_ext_nF"], "feasible": unconstrained["feasible"],
                                           "p_sw_W": unconstrained["p_sw_4S_20A_24kHz_W"],
                                           "stress": unconstrained["stress_4S_HV_35A"]},
            "vds_peak_vs_loop_inductance_V": os_sweep,
        },
        "dead_time": {
            "rows": dt_rows, "dt_mcu_required_ns": dt_need * 1e9, "tick_ns": tick * 1e9,
            "DEAD_TIME": dtg, "dt_mcu_ns": dt_mcu * 1e9,
            "body_diode_per_edge_ns": {"typ": dt_diode_typ * 1e9, "max": dt_diode_max * 1e9},
            "arr_at_24kHz": arr, "min_on_time_ns": t_min_on * 1e9, "swallowed_pulse_below_ns": t_swallow * 1e9,
            "minimum_duty_cycle_eeprom_needed": md_need,
            "gate_drive_offset_note": f"DEAD_TIME also seeds gate_drive_offset = {dtg}/2000 of ARR "
                                      f"({dtg*(arr+1)/2000*tick*1e9:.0f} ns, sine start-up only); "
                                      "minimum_duty_cycle is overwritten from EEPROM byte 6 (main.c:645)",
            "driving_brake_override": brake,
        },
        "fd6288_dissipation": drv_rows, "bootstrap": bs, "switching_energy_table": e_table,
        "requirements": [
            "FD6288Q VCC = VBAT via 10 ohm + 10 uF/25 V X5R (datasheet C1), clamped by an 18 V Zener (BZT52C18) at VCC",
            f"Gate network per FET: Rg_on {rg_on} ohm 0402" + ("" if r_off is None else f", Schottky (RB521S30) + {r_off} ohm in parallel for turn-off")
            + ("" if cgs == 0 else f", Cgs_ext {cgs*1e9:.1f} nF C0G at the FET"),
            "Gate-source pull-down: 10 kohm on each LS, 100 kohm on each HS (HS value loads C_BS; holds off while VCC < UVLO)",
            "Bootstrap: 40 V Schottky with IR <= 100 uA at 20 V / 105 C + 2.2 ohm series + 2.2 uF/25 V X5R 0603 per phase, cap within 2 mm of VB/VS",
            f"HS-LS-ceramic commutation loop <= {ASSUME['l_loop_nH']:.0f} nH: >= 2 x 10 uF/25 V 0805 across each half-bridge on the FET layer",
            "FD6288Q HIN1..3 <- AT32F421 PA10/PA9/PA8 (TMR1_CH3/2/1), LIN1..3 <- PB1/PB0/PA7 (TMR1_CH3N/2N/1N); no inverters",
            "AM32 EEPROM per am32_esc/eeprom_settings.json; FC battery failsafe BATT_CRT_VOLT >= 6.4 V protects the FD6288Q UVLO",
        ],
        "sweep_feasible_by_loss": [
            {"rg_on": s_["rg_on"], "r_off": s_["r_off"], "cgs_ext_nF": s_["cgs_ext_nF"],
             "p_sw_W": round(s_["p_sw_4S_20A_24kHz_W"], 4),
             "dvdt": round(s_["stress_4S_HV_35A"]["dvdt_V_per_ns"], 2),
             "est_pct": round(100 * s_["stress_4S_HV_35A"]["est_frac"], 1),
             "tj_fd6288_24k": round(s_["stress_4S_HV_35A"]["tj_fd6288_24k_C"], 1),
             "parts_per_fet": parts(s_), "in_loss_band": s_ in short}
            for s_ in sorted(feas, key=lambda x: x["p_sw_4S_20A_24kHz_W"])],
        "sweep_rejected": [
            {"rg_on": s_["rg_on"], "r_off": s_["r_off"], "cgs_ext_nF": s_["cgs_ext_nF"],
             "p_sw_W": round(s_["p_sw_4S_20A_24kHz_W"], 4),
             "why": [w for w, bad in (
                 ("dv/dt", s_["stress_4S_HV_35A"]["dvdt_V_per_ns"] > ASSUME["dvdt_emi_max"] * 1e-9),
                 ("shoot-through", s_["stress_4S_HV_35A"]["est_frac"] > ASSUME["est_frac_max"]),
                 ("VDS", s_["stress_4S_HV_35A"]["vds_peak_V"] > ASSUME["vds_derate"] * FET["vdss"]),
                 ("FD6288Q Tj", s_["stress_4S_HV_35A"]["tj_fd6288_24k_C"] > ASSUME["tj_fd6288_max_C"])) if bad]}
            for s_ in sorted(sweep, key=lambda x: x["p_sw_4S_20A_24kHz_W"]) if not s_["feasible"]],
        "header": hdr, "checks": checks, "failures": fails,
        "status": "PASS" if not fails else "FAIL",
    }
    with open(a.out, "w") as f:
        json.dump(report, f, indent=1, default=lambda o: None)

    # ---- console summary
    print(f"Rev 9 gate-drive design  (Vth,min @ {TJ_HOT:.0f} C = {VTH_HOT_MIN:.2f} V)")
    print(f"  calibration: ton {ton_m*1e9:.1f} ns (ds {t['ton']*1e9:.0f}), toff {toff_m*1e9:.1f} ns (ds {t['toff']*1e9:.0f})")
    print(f"  gate network: Rg_on {rg_on} ohm, turn-off {report['selected_gate_network']['turn_off']}, Cgs_ext {cgs*1e9:.1f} nF")
    print(f"    4S-HV 35 A: dv/dt {sel['dvdt_V_per_ns']:.2f} V/ns, induced Vgs {sel['induced_vgs_pk_V']:.2f} V, "
          f"E_st/E_on {sel['est_frac']*100:.1f} %, VDS pk {sel['vds_peak_V']:.1f} V")
    print(f"    P_sw per HS @ 4S 20 A 24 kHz: {best['p_sw_4S_20A_24kHz_W']*1e3:.0f} mW "
          f"(unconstrained optimum {unconstrained['p_sw_4S_20A_24kHz_W']*1e3:.0f} mW, feasible={unconstrained['feasible']})")
    for r in dt_rows:
        print(f"  dead time {r['bus']:6s} {r['transition']:14s} need {r['dt_mcu_required_ns']:6.1f} ns")
    print(f"  DEAD_TIME = {dtg} -> {dt_mcu*1e9:.1f} ns MCU + 100..300 ns FD6288Q internal; "
          f"diode/edge typ {dt_diode_typ*1e9:.0f} ns max {dt_diode_max*1e9:.0f} ns")
    for c in checks:
        print(f"  [{'PASS' if c['pass'] else 'FAIL'}] {c['check']}: {c['detail']}")
    print("PASS" if not fails else f"FAIL: {fails}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
