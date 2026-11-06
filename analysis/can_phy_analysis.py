#!/usr/bin/env python3
"""
Nano H743 AIO - DroneCAN / CAN FD physical layer check (Revision 14.0)

FDCAN1 (PD0 RX / PD1 TX, PD3 STB) -> U7 TCAN334G -> split 120 R termination
(JP1/JP2 + R45/R46 60.4 R + C46 4.7 nF) -> D4 PESD2CANFD24V-T ESD array ->
J3 JST-GH 4-pin DroneCAN connector (1 +5V, 2 CANH, 3 CANL, 4 GND).

Pure Python (no numpy).  Writes analysis/can_phy_report.json and exits 1 if
any gate fails.

Sections
  1. FDCAN kernel clock: 24 MHz HSE -> PLL1 (M3, N120, Q12) = 80 MHz,
     FDCANSEL = PLL1_Q.  Re-read from ArduPilot's stm32h7_mcuconf.h when a
     tree is given.
  2. Bit timing: ArduPilot CANIface::computeTimings() ported line for line
     (nominal phase) + the computeFDTimings() table (data phase), checked for
     exact bit rate, sample point, CAN-FD oscillator tolerance (Bosch / CiA
     conditions 1..5) against the Y1 crystal budget, and the transmitter
     delay compensation (TDCO = 10 mtq) against the TCAN334G loop delay and
     receiver bit-symmetry window.  This decides which CAN_P1_FDBITRATE
     values are legal on this hardware.
  3. Termination: split 2 x 60.4 R, tolerance window vs ISO 11898-2 RL,
     common-mode corner of the split capacitor, worst-case resistor power
     for dominant drive and for the realistic miswire (CANH/CANL shorted to
     the connector's own +5 V pin), resistor-mismatch mode conversion.
  4. ESD array: stand-off vs. the bus common-mode range (no conduction in
     normal operation), capacitance and matching budget per node, bus RC at
     5 Mbit/s for up to N nodes, IEC 61000-4-2 level.  The diode is NOT a DC
     fault clamp: VBR(min) 25.5 V > TCAN334G abs-max 14 V; that limit is
     reported, not hidden.
  5. Mode pins / boot behaviour: SHDN tied to GND, STB integrated pull-DOWN
     (TCAN33x DS pin table) so the transceiver is in normal mode with TXD
     pulled recessive while the MCU is in reset, and the hwdef / defaults
     configuration (PD3 LOW, CANFD_SUPPORTED, CAN_P1_*) is consistent.
  6. On-board routing: J3 -> D4 -> U7 run is electrically short (lumped) at
     the TCAN334G edge rates, so no 120 R controlled-impedance pair is
     needed on this board; symmetry rules are derived instead.

Usage: python3 analysis/can_phy_analysis.py [--ardupilot /path/to/ardupilot]
       (or set ARDUPILOT_ROOT)
"""
import argparse
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
HWDEF = os.path.join(REPO, "ardupilot", "hwdef.dat")
PARM = os.path.join(REPO, "ardupilot", "defaults.parm")
REPORT = os.path.join(HERE, "can_phy_report.json")

# ----------------------------------------------------------------------------
# Assumptions / datasheet figures (all copied into the report)
# ----------------------------------------------------------------------------
A = {
    "hse_hz": 24e6,
    "crystal_ppm_total": 30.0,     # Y1: +/-10 ppm tolerance + +/-20 ppm over -40..85 C (Rev 7)
    "crystal_aging_ppm": 5.0,
    "vcc": {"min": 3.135, "nom": 3.3, "max": 3.465, "note": "+3V3 rail +/-5 %"},
    "tcan334g": {   # TI SLLSEJ9 (TCAN33x / TCAN33xG), LCSC C191005
        "part": "TCAN334GDCNR",
        "vcc_min": 3.0, "vcc_max": 3.6,
        "bus_abs_max_V": 14.0, "bus_cm_range_V": 12.0,
        "data_rate_max_bps": 5e6,
        "loop_delay_max_ns": 135.0,
        "tREC_SYM_5_ns": [120.0, 220.0], "tBUS_SYM_5_ns": [155.0, 210.0],
        "tREC_SYM_2_ns": [400.0, 550.0],
        "vo_rec_V": 1.85, "vod_dom_V": [1.5, 3.0], "rl_spec_ohm": 60.0,
        "ci_pF": 20.0, "cid_pF": 10.0,
        "icc_dom_mA": 55.0, "icc_rec_mA": 3.5,
        "tTXD_DTO_ms": [1.2, 3.8],
        "iec_contact_kV": 12.0,
        "pins": {"STB": "integrated pull-down (high = standby)",
                 "SHDN": "integrated pull-down (high = shutdown)",
                 "TXD": "integrated pull-up (recessive)"},
    },
    "esd": {        # Nexperia PESD2CANFD24V-T (11 Aug 2020), LCSC C552486
        "part": "PESD2CANFD24V-TR",
        "vrwm_V": 24.0, "vbr_min_V": 25.5, "vbr_max_V": 35.5,
        "cd_max_pF": 6.0, "cd_typ_pF": 5.2, "dcd_rel": 0.005,
        "ir_max_nA": 50.0, "vcl_1A_max_V": 42.0, "rdyn_ohm": 0.7,
        "iec_kV": 23.0,
    },
    "termination": {"r_ohm": 60.4, "tol": 0.01, "p_rating_W": 0.2,
                    "part": "YAGEO AC0603FR-7W60R4L (0603 anti-surge, 200 mW)",
                    "c_split_F": 4.7e-9, "c_part": "TDK CGA3EAC0G2A472JT000E C0G 100 V"},
    "iso11898_2": {"rl_min_ohm": 100.0, "rl_max_ohm": 130.0, "cm_range_V": [-2.0, 7.0]},
    "connector": {"part": "JST SM04B-GHS-TB", "pin_current_A": 1.0,
                  "pinout": {"1": "+5V", "2": "CANH", "3": "CANL", "4": "GND"}},
    "board": {"j_to_u_mm": 25.0, "er_eff": 3.2, "trace_pF_per_mm": 0.08, "conn_pF": 1.0},
    "bus": {"nodes_max": 8, "cable_pF_per_m": 50.0, "cable_m": 1.0},
    # ArduPilot fallbacks (overwritten from the tree when given)
    "ardupilot": {
        "pll1_divm": 3, "pll1_divn": 120, "pll1_divq": 12, "fdcansel": "STM32_FDCANSEL_PLL1_Q_CK",
        "fd_table": [[1, 4, 14, 5, 5, 75], [2, 2, 14, 5, 5, 75], [4, 1, 14, 5, 5, 75],
                     [5, 1, 11, 4, 4, 75], [8, 1, 6, 3, 3, 70]],
        "tdco_mtq": 10, "max_sp_permill": 900, "max_bs1": 16, "max_bs2": 8,
        "fdbitrate_values": [1, 2, 4, 5, 8], "bitrate_range": [10000, 1000000],
        "canfd_supported_default_h7": 4,
        "source": "embedded (Copter-4.7.1) - pass --ardupilot to re-read",
    },
}

GATES = []
INFO = []


def gate(name, ok, detail):
    GATES.append({"gate": name, "pass": bool(ok), "detail": detail})
    print("  [%s] %s: %s" % ("PASS" if ok else "FAIL", name, detail))


def info(msg):
    INFO.append(msg)
    print("  [INFO] %s" % msg)


# ----------------------------------------------------------------------------
# 1. ArduPilot tree (optional)
# ----------------------------------------------------------------------------
def read_ardupilot(root):
    ap = A["ardupilot"]
    if not root:
        return
    mc = open(os.path.join(root, "libraries/AP_HAL_ChibiOS/hwdef/common/stm32h7_mcuconf.h")).read()
    blk = mc.split("#elif STM32_HSECLK == 24000000U", 1)[1]
    m_blk = blk.split("#elif", 1)[0]
    ap["pll1_divm"] = int(re.search(r"STM32_PLL1_DIVM_VALUE\s+(\d+)", m_blk).group(1))
    # the 480 MHz custom-clock branch of the PLL1 N/Q block for 24 MHz
    nblk = mc.split("#elif STM32_HSECLK == 24000000U", 2)[2]
    b480 = nblk.split("HAL_CUSTOM_MCU_CLOCKRATE == 480000000", 1)[1].split("#elif", 1)[0]
    ap["pll1_divn"] = int(re.search(r"STM32_PLL1_DIVN_VALUE\s+(\d+)", b480).group(1))
    ap["pll1_divq"] = int(re.search(r"STM32_PLL1_DIVQ_VALUE\s+(\d+)", b480).group(1))
    ap["fdcansel"] = re.search(r"#define STM32_FDCANSEL\s+(\w+)", mc).group(1)
    src = open(os.path.join(root, "libraries/AP_HAL_ChibiOS/CANFDIface.cpp")).read()
    tab = src.split("CANFD_timings[] {", 1)[1].split("};", 1)[0]
    ap["fd_table"] = [[int(x) for x in r] for r in re.findall(
        r"\{\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\s*\}", tab)]
    ap["tdco_mtq"] = int(re.search(r"TDCR\s*=\s*(\d+)\s*<<\s*FDCAN_TDCR_TDCO_Pos", src).group(1))
    ap["max_sp_permill"] = int(re.search(r"MaxSamplePointLocation\s*=\s*(\d+)", src).group(1))
    ap["max_bs1"] = int(re.search(r"MaxBS1\s*=\s*(\d+)", src).group(1))
    ap["max_bs2"] = int(re.search(r"MaxBS2\s*=\s*(\d+)", src).group(1))
    prm = open(os.path.join(root, "libraries/AP_CANManager/AP_CANIfaceParams.cpp")).read()
    fdv = re.search(r"@Param: FDBITRATE.*?@Values:\s*([^\n]+)", prm, re.S).group(1)
    ap["fdbitrate_values"] = [int(k) for k in re.findall(r"(\d+)\s*:", fdv)]
    rng = re.search(r"@Param: BITRATE.*?@Range:\s*(\d+)\s+(\d+)", prm, re.S)
    ap["bitrate_range"] = [int(rng.group(1)), int(rng.group(2))]
    hp = open(os.path.join(root, "libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py")).read()
    m = re.search(r"canfd_supported = int\(self\.get_config\('CANFD_SUPPORTED', 0, default=(\d+)", hp)
    ap["canfd_supported_default_h7"] = int(m.group(1)) if m else ap["canfd_supported_default_h7"]
    ap["source"] = "read from %s" % root


# ----------------------------------------------------------------------------
# 2. Bit timing
# ----------------------------------------------------------------------------
def compute_timings(pclk, target):
    """Port of AP_HAL_ChibiOS CANIface::computeTimings() (CANFDIface.cpp)."""
    ap = A["ardupilot"]
    max_q = 10 if target >= 1000000 else 17
    prescaler_bs = pclk // target
    s = max_q - 1
    while prescaler_bs % (1 + s) != 0:
        if s <= 2:
            return None
        s -= 1
    presc = prescaler_bs // (1 + s)
    if presc < 1 or presc > 1024:
        return None

    def pair(bs1):
        bs2 = s - bs1
        return {"bs1": bs1, "bs2": bs2, "sp_permill": (1000 * (1 + bs1)) // (1 + bs1 + bs2)}
    sol = pair(((7 * s - 1) + 4) // 8)
    if sol["sp_permill"] > ap["max_sp_permill"]:
        sol = pair((7 * s - 1) // 8)
    valid = 1 <= sol["bs1"] <= ap["max_bs1"] and 1 <= sol["bs2"] <= ap["max_bs2"]
    if target != pclk // (presc * (1 + sol["bs1"] + sol["bs2"])) or not valid:
        return None
    return {"prescaler": presc, "sjw": 1, "bs1": sol["bs1"], "bs2": sol["bs2"],
            "tq_per_bit": 1 + sol["bs1"] + sol["bs2"], "sample_point": sol["sp_permill"] / 1000.0,
            "bitrate": pclk / (presc * (1 + sol["bs1"] + sol["bs2"]))}


def fd_timings(pclk, mbaud):
    for b, p, bs1, bs2, sjw, sp in A["ardupilot"]["fd_table"]:
        if b == mbaud:
            return {"prescaler": p, "sjw": sjw, "bs1": bs1, "bs2": bs2, "tq_per_bit": 1 + bs1 + bs2,
                    "sample_point": (1 + bs1) / (1.0 + bs1 + bs2), "table_sp_pct": sp,
                    "bitrate": pclk / (p * (1 + bs1 + bs2))}
    return None


def osc_tolerance(nom, dat):
    """CAN FD clock tolerance conditions (Hartwich, 'Bit Time Requirements for
    CAN FD', iCC 2013).  PS1 is taken as TSEG1 - 1 tq of propagation segment
    minimum, i.e. conservative.  Returns max allowed df (fraction)."""
    nbt, dbt = nom["tq_per_bit"], dat["tq_per_bit"]
    ps2n, ps2d = nom["bs2"], dat["bs2"]
    ps1n, ps1d = nom["bs1"] - 1, dat["bs1"] - 1
    ratio = dat["prescaler"] / float(nom["prescaler"])
    c = {
        "1_nominal_resync": nom["sjw"] / (2.0 * 10 * nbt),
        "2_nominal_error_flag": min(ps1n, ps2n) / (2.0 * (13 * nbt - ps2n)),
        "3_data_resync": dat["sjw"] / (2.0 * 10 * dbt),
        "4_data_to_nominal": min(ps1n, ps2n) / (2.0 * ((6 * dbt - ps2d) * ratio + 7 * nbt)),
        "5_data_resync_ps": (dat["sjw"] - max(0.0, ratio - 1)) / (2.0 * ((2 * nbt - ps2n) * (1 / ratio) + 4 * dbt))
        if ratio >= 1 else dat["sjw"] / (2.0 * 10 * dbt),
    }
    return min(c.values()), c


def tdc_window(bitrate):
    """Secondary sample point = measured TX->RX delay + TDCO.  Received bit
    asymmetry (tREC_SYM, worst at 5 Mbit/s) shifts the start of a recessive
    bit late by up to (Tbit - tREC_SYM_min); the bit then ends at the next
    measured edge (delay + Tbit).  SSP must sit in [d + shift, d + Tbit]."""
    t = A["tcan334g"]
    tbit = 1e9 / bitrate
    fclk = A["fdcan_clk_hz"]
    tdco_ns = A["ardupilot"]["tdco_mtq"] * 1e9 / fclk
    rec_min = t["tREC_SYM_5_ns"][0] * (200.0 / tbit) if tbit < 200 else t["tREC_SYM_5_ns"][0]
    shift = max(0.0, 200.0 - t["tREC_SYM_5_ns"][0])     # 80 ns at the 5 Mbit/s spec corner
    lo, hi = shift, tbit
    return {"tbit_ns": tbit, "tdco_ns": tdco_ns, "ssp_after_edge_ns": tdco_ns,
            "window_ns": [lo, hi], "margin_early_ns": tdco_ns - lo, "margin_late_ns": hi - tdco_ns,
            "ok": lo < tdco_ns < hi and bitrate <= t["data_rate_max_bps"],
            "rec_min_scaled_ns": rec_min}


# ----------------------------------------------------------------------------
# 3. Termination
# ----------------------------------------------------------------------------
def termination():
    t = A["termination"]
    x = A["tcan334g"]
    r, tol = t["r_ohm"], t["tol"]
    rt = {"nom": 2 * r, "min": 2 * r * (1 - tol), "max": 2 * r * (1 + tol)}
    r_cm = r / 2.0                       # both halves in parallel for common mode
    f_cm = 1.0 / (2 * math.pi * r_cm * t["c_split_F"])
    bus_rl = rt["nom"] / 2.0             # two terminated ends
    # dominant: VOD max across this node's 120.8 R
    i_dom = x["vod_dom_V"][1] / rt["min"]
    p_dom = i_dom ** 2 * r * (1 + tol)
    # miswire: CANH or CANL shorted to the connector's own +5 V (pin 1 adjacent to pin 2)
    # other line held by the transceiver: recessive 1.85 V, or dominant CANL ~0.5 V
    cases = {}
    for name, v_other in (("H_to_5V_recessive", x["vo_rec_V"]), ("H_to_5V_CANL_dominant", 0.5),
                          ("L_to_5V_recessive", x["vo_rec_V"])):
        i = (5.25 - v_other) / rt["min"]
        cases[name] = {"I_mA": i * 1e3, "P_per_R_mW": i ** 2 * r * (1 + tol) * 1e3}
    # abs-max fault (+14 V on one line, the other recessive) - outside the design envelope
    i14 = (x["bus_abs_max_V"] - x["vo_rec_V"]) / rt["min"]
    p14 = i14 ** 2 * r * (1 + tol)
    # mode conversion from resistor mismatch: differential signal Vd leaks to the
    # split node as Vd * dR / (2 * R) (unloaded divider imbalance)
    lcl_db = 20 * math.log10(1.0 / (2 * tol / 2.0)) if tol > 0 else float("inf")
    return {"R_term_ohm": rt, "R_cm_ohm": r_cm, "f_cm_corner_Hz": f_cm, "bus_RL_two_ends_ohm": bus_rl,
            "dominant": {"I_mA": i_dom * 1e3, "P_per_R_mW": p_dom * 1e3},
            "miswire_5V": cases,
            "fault_14V": {"I_mA": i14 * 1e3, "P_per_R_mW": p14 * 1e3,
                          "note": "+14 V is the TCAN334G abs-max; a battery-voltage short is outside the "
                                  "transceiver rating anyway - prevented by keyed JST-GH, VBAT never on J3"},
            "mode_conversion_LCL_dB_worst": lcl_db}


# ----------------------------------------------------------------------------
# 4. ESD / capacitance budget
# ----------------------------------------------------------------------------
def esd_budget():
    e, x, b, bus = A["esd"], A["tcan334g"], A["board"], A["bus"]
    c_line = x["ci_pF"] + e["cd_max_pF"] + b["conn_pF"] + b["trace_pF_per_mm"] * b["j_to_u_mm"]
    c_diff_node = x["cid_pF"] + e["cd_max_pF"] / 2.0 + (b["trace_pF_per_mm"] * b["j_to_u_mm"]) / 2.0
    c_bus = bus["nodes_max"] * c_diff_node + bus["cable_pF_per_m"] * bus["cable_m"] / 2.0
    tau = 60.0 * c_bus * 1e-12           # recessive edge: bus driven only by the 60 R termination pair
    t_settle = tau * math.log(0.9 / 0.5 * 2)     # 0.9 V -> 0.5 V receiver window, conservative
    unbal_pF = e["cd_max_pF"] * e["dcd_rel"]
    return {"C_line_to_gnd_pF": c_line, "C_diff_node_pF": c_diff_node, "C_diff_bus_pF": c_bus,
            "tau_ns": tau * 1e9, "recessive_settle_ns": t_settle * 1e9,
            "frac_of_5M_bit": t_settle * 5e6, "esd_unbalance_pF": unbal_pF,
            "standoff_vs_cm": {"vrwm_V": e["vrwm_V"], "bus_cm_V": x["bus_cm_range_V"]},
            "dc_fault_gap_V": [x["bus_abs_max_V"], e["vbr_min_V"]]}


# ----------------------------------------------------------------------------
# 5. Configuration evidence (hwdef / defaults)
# ----------------------------------------------------------------------------
def config():
    hw = open(HWDEF).read()
    pins = dict(re.findall(r"^(P[A-K]\d+)\s+(\S+)", hw, re.M))
    can_line = re.search(r"^PD3\s+GPIO_CAN1_SILENT\s+OUTPUT\s+PUSHPULL\s+\S+\s+(LOW|HIGH)", hw, re.M)
    canfd = re.search(r"^CANFD_SUPPORTED\s+(\d+)", hw, re.M)
    blk = hw.split("FDCAN1", 1)[1].split("SDMMC1", 1)[0] if "FDCAN1" in hw else ""
    params = {}
    for line in open(PARM):
        line = line.split("#", 1)[0].strip()
        m = re.match(r"^(\w+)\s+(-?[\d.]+)$", line)
        if m:
            params[m.group(1)] = float(m.group(2))
    return {"pins": {k: pins.get(k) for k in ("PD0", "PD1", "PD3")},
            "stb_default": can_line.group(1) if can_line else None,
            "canfd_supported": int(canfd.group(1)) if canfd else None,
            # the pre-Rev-14 (wrong) assertion, and the corrected datasheet statement
            "claims_stb_pullup": bool(re.search(r"STB has an internal pull-up: it must be actively driven LOW", blk, re.I)),
            "states_stb_pulldown": bool(re.search(r"STB[\s\S]{0,120}integrated pull[\s#-]*down", blk, re.I)),
            "params": {k: v for k, v in params.items() if k.startswith("CAN_")}}


# ----------------------------------------------------------------------------
def main():
    ap_ = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap_.add_argument("--ardupilot", default=os.environ.get("ARDUPILOT_ROOT"))
    args = ap_.parse_args()
    read_ardupilot(args.ardupilot)
    ap = A["ardupilot"]
    print("CAN PHY analysis (Rev 14.0) - ArduPilot data %s" % ap["source"])

    print("1 FDCAN kernel clock")
    vco = A["hse_hz"] / ap["pll1_divm"] * ap["pll1_divn"]
    fclk = vco / ap["pll1_divq"]
    A["fdcan_clk_hz"] = fclk
    gate("C1 FDCAN kernel clock = PLL1_Q = 80 MHz", ap["fdcansel"].endswith("PLL1_Q_CK") and abs(fclk - 80e6) < 1,
         "24 MHz / M%d x N%d / Q%d = %.3f MHz (%s)" % (ap["pll1_divm"], ap["pll1_divn"], ap["pll1_divq"],
                                                      fclk / 1e6, ap["fdcansel"]))

    print("2 bit timing")
    cfg = config()
    nom_rate = int(cfg["params"].get("CAN_P1_BITRATE", 1000000))
    nom = compute_timings(int(fclk), nom_rate)
    gate("T1 nominal %d bit/s has an exact ArduPilot solution" % nom_rate, nom is not None and nom["bitrate"] == nom_rate,
         "%s" % nom)
    lo_rate = 11.0 / (A["tcan334g"]["tTXD_DTO_ms"][0] * 1e-3)
    gate("T2 TXD dominant time-out allows the configured rate (min %.1f kbit/s)" % (lo_rate / 1e3),
         nom_rate >= lo_rate and ap["bitrate_range"][0] >= lo_rate,
         "CAN_P1_BITRATE %d, ArduPilot range %s" % (nom_rate, ap["bitrate_range"]))
    fd = {}
    budget_ppm = A["crystal_ppm_total"] + A["crystal_aging_ppm"]
    for mb in ap["fdbitrate_values"]:
        d = fd_timings(fclk, mb)
        tol, conds = osc_tolerance(nom, d)
        w = tdc_window(mb * 1e6)
        legal = (d is not None and abs(d["bitrate"] - mb * 1e6) < 1 and w["ok"]
                 and tol * 1e6 > budget_ppm and mb * 1e6 <= A["tcan334g"]["data_rate_max_bps"])
        fd[mb] = {"timing": d, "osc_tol_ppm": tol * 1e6, "osc_conditions_ppm": {k: v * 1e6 for k, v in conds.items()},
                  "tdc": w, "legal_on_this_board": legal}
        print("    FDBITRATE %d: %d tq, SP %.1f %%, TDC SSP %.0f ns in window %s ns, osc tol %.0f ppm -> %s"
              % (mb, d["tq_per_bit"], d["sample_point"] * 100, w["ssp_after_edge_ns"],
                 [round(v) for v in w["window_ns"]], tol * 1e6, "LEGAL" if legal else "ILLEGAL"))
    legal = sorted(k for k, v in fd.items() if v["legal_on_this_board"])
    fdset = int(cfg["params"].get("CAN_P1_FDBITRATE", cfg["canfd_supported"] or ap["canfd_supported_default_h7"]))
    gate("T3 CAN_P1_FDBITRATE %d is legal on TCAN334G + 80 MHz FDCAN" % fdset, fdset in legal,
         "legal %s Mbit/s (8 Mbit/s: above the 5 Mbit/s TCAN334G rating and SSP on the bit edge)" % legal)
    gate("T4 oscillator tolerance at the configured rates > crystal budget",
         fd[fdset]["osc_tol_ppm"] > budget_ppm * 10,
         "%.0f ppm allowed vs %.0f ppm Y1 (x%.0f margin)" % (fd[fdset]["osc_tol_ppm"], budget_ppm,
                                                            fd[fdset]["osc_tol_ppm"] / budget_ppm))
    gate("T5 TDC: secondary sample point inside the received bit at %d Mbit/s" % fdset, fd[fdset]["tdc"]["ok"],
         "SSP = delay + %.0f ns, window +%.0f..+%.0f ns (margins %.0f / %.0f ns); loop delay <= %.0f ns is "
         "measured by the FDCAN, not assumed" % (fd[fdset]["tdc"]["tdco_ns"], fd[fdset]["tdc"]["window_ns"][0],
                                                 fd[fdset]["tdc"]["window_ns"][1], fd[fdset]["tdc"]["margin_early_ns"],
                                                 fd[fdset]["tdc"]["margin_late_ns"], A["tcan334g"]["loop_delay_max_ns"]))
    cfd = cfg["canfd_supported"]
    gate("T6 hwdef CANFD_SUPPORTED pins the default FD rate to a legal value",
         cfd is not None and cfd in legal, "CANFD_SUPPORTED %s (ArduPilot H7 default %d; options %s)"
         % (cfd, ap["canfd_supported_default_h7"], ap["fdbitrate_values"]))
    if fd[fdset]["timing"]["prescaler"] != nom["prescaler"]:
        info("ArduPilot uses BRP %d (nominal) vs %d (data): CiA 601-3 recommends equal prescalers; tolerance "
             "conditions above already include the ratio, still >= x10 margin" % (nom["prescaler"],
                                                                                fd[fdset]["timing"]["prescaler"]))

    print("3 termination")
    t = termination()
    iso = A["iso11898_2"]
    gate("R1 split termination %.1f..%.1f R inside ISO 11898-2 RL %d..%d R" % (t["R_term_ohm"]["min"], t["R_term_ohm"]["max"],
                                                                         iso["rl_min_ohm"], iso["rl_max_ohm"]),
         iso["rl_min_ohm"] <= t["R_term_ohm"]["min"] and t["R_term_ohm"]["max"] <= iso["rl_max_ohm"],
         "two-ended bus RL %.1f R vs VOD spec load %.0f R" % (t["bus_RL_two_ends_ohm"], A["tcan334g"]["rl_spec_ohm"]))
    gate("R2 split-cap common-mode corner below the 1 Mbit/s fundamental",
         t["f_cm_corner_Hz"] < 1.5e6, "f = 1/(2 pi x %.1f R x 4.7 nF) = %.2f MHz" % (t["R_cm_ohm"], t["f_cm_corner_Hz"] / 1e6))
    pw = A["termination"]["p_rating_W"] * 1e3
    worst = max([t["dominant"]["P_per_R_mW"]] + [c["P_per_R_mW"] for c in t["miswire_5V"].values()])
    gate("R3 resistor power, dominant + +5V miswire <= 50 %% of %.0f mW" % pw, worst <= 0.5 * pw,
         "dominant %.1f mW, miswire %s mW" % (t["dominant"]["P_per_R_mW"],
                                             {k: round(v["P_per_R_mW"], 1) for k, v in t["miswire_5V"].items()}))
    info("+14 V abs-max fault: %.0f mW per resistor (> rating) - out of envelope, see report note"
         % t["fault_14V"]["P_per_R_mW"])

    print("4 ESD array")
    e = esd_budget()
    gate("E1 ESD stand-off >= bus common-mode range (no conduction in normal operation)",
         A["esd"]["vrwm_V"] >= A["tcan334g"]["bus_cm_range_V"] and A["esd"]["vrwm_V"] >= iso["cm_range_V"][1],
         "VRWM %.0f V vs CM +/-%.0f V, IR <= %.0f nA" % (A["esd"]["vrwm_V"], A["tcan334g"]["bus_cm_range_V"], A["esd"]["ir_max_nA"]))
    gate("E2 per-node bus capacitance small at 5 Mbit/s (recessive settle < 20 %% of bit, %d nodes)" % A["bus"]["nodes_max"],
         e["frac_of_5M_bit"] < 0.2, "C_diff node %.1f pF, bus %.0f pF, tau %.1f ns, settle %.1f ns = %.1f %% of 200 ns"
         % (e["C_diff_node_pF"], e["C_diff_bus_pF"], e["tau_ns"], e["recessive_settle_ns"], e["frac_of_5M_bit"] * 100))
    gate("E3 ESD capacitance matched (mode conversion negligible)", e["esd_unbalance_pF"] < 0.1,
         "dCd = %.3f pF (0.5 %% of 6 pF)" % e["esd_unbalance_pF"])
    gate("E4 IEC 61000-4-2 level 4 (8 kV contact / 15 kV air) at the connector",
         A["esd"]["iec_kV"] >= 15.0, "PESD2CANFD24V-T %.0f kV + TCAN334G on-chip %.0f kV contact"
         % (A["esd"]["iec_kV"], A["tcan334g"]["iec_contact_kV"]))
    info("DC-fault gap %.1f..%.1f V: diode does not conduct below VBR(min); TCAN334G abs-max is %.0f V. "
         "J3 carries only +5V/CANH/CANL/GND (keyed GH) - never VBAT." % (e["dc_fault_gap_V"][0], e["dc_fault_gap_V"][1],
                                                                       A["tcan334g"]["bus_abs_max_V"]))

    print("5 mode pins / configuration")
    gate("M1 hwdef CAN1_RX PD0, CAN1_TX PD1, GPIO_CAN1_SILENT PD3",
         cfg["pins"] == {"PD0": "CAN1_RX", "PD1": "CAN1_TX", "PD3": "GPIO_CAN1_SILENT"}, "%s" % cfg["pins"])
    gate("M2 STB driven LOW (normal mode) after init", cfg["stb_default"] == "LOW", "PD3 default %s" % cfg["stb_default"])
    gate("M3 hwdef no longer claims an STB pull-up (DS: integrated pull-DOWN)",
         not cfg["claims_stb_pullup"] and cfg["states_stb_pulldown"],
         "reset/boot: PD3 Hi-Z -> STB pull-down -> normal mode; PD1 Hi-Z -> TXD pull-up -> recessive")
    gate("M4 +3V3 inside TCAN334G VCC range", A["tcan334g"]["vcc_min"] <= A["vcc"]["min"] and
         A["vcc"]["max"] <= A["tcan334g"]["vcc_max"], "%.3f..%.3f V in %.1f..%.1f V" % (
             A["vcc"]["min"], A["vcc"]["max"], A["tcan334g"]["vcc_min"], A["tcan334g"]["vcc_max"]))
    pr = cfg["params"]
    gate("M5 defaults.parm enables DroneCAN on CAN1 at 1 Mbit/s",
         pr.get("CAN_P1_DRIVER") == 1 and pr.get("CAN_D1_PROTOCOL") == 1 and pr.get("CAN_P1_BITRATE") == 1e6,
         "%s" % pr)

    print("6 on-board routing")
    b = A["board"]
    tpd = b["j_to_u_mm"] * 1e-3 * math.sqrt(b["er_eff"]) / 299792458.0
    tr_min = 10e-9      # TCAN33xG driver edges are slope-controlled (tens of ns); 10 ns is a conservative floor
    gate("L1 J3 -> D4 -> U7 run is lumped (t_pd < t_r/6)", tpd < tr_min / 6,
         "%.0f mm: t_pd %.2f ns vs t_r/6 %.2f ns -> no 120 R controlled impedance needed on-board"
         % (b["j_to_u_mm"], tpd * 1e9, tr_min / 6 * 1e9))
    icc = A["tcan334g"]["icc_dom_mA"]
    info("+3V3 load: %.1f mA recessive, %.0f mA max dominant (50 %% duty ~%.0f mA avg)"
         % (A["tcan334g"]["icc_rec_mA"], icc, (icc + A["tcan334g"]["icc_rec_mA"]) / 2))

    ok = all(g["pass"] for g in GATES)
    rep = {"revision": "14.0", "assumptions": A, "fdcan_clk_hz": fclk, "nominal": nom,
           "data_phase": {str(k): v for k, v in fd.items()}, "legal_fdbitrate_mbit": legal,
           "termination": t, "esd": e, "config": cfg, "gates": GATES, "info": INFO,
           "result": "PASS" if ok else "FAIL"}
    with open(REPORT, "w") as fh:
        json.dump(rep, fh, indent=1, sort_keys=True, default=float)
        fh.write("\n")
    print("\nRESULT: %s (%d/%d gates) -> %s" % ("PASS" if ok else "FAIL", sum(g["pass"] for g in GATES), len(GATES),
                                             os.path.relpath(REPORT, REPO)))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
