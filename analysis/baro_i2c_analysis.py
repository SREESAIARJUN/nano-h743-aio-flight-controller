#!/usr/bin/env python3
"""
Nano H743 AIO - I2C1 (DPS310 baro + external compass) 400 kHz timing,
electrical and ESC-noise-immunity check (Revision 12.0)

Pure Python (no numpy).  Writes analysis/baro_i2c_report.json and exits 1 if
any gate fails.

Sections
  1. I2C kernel clock: 24 MHz HSE -> PLL3 (M, N, R) -> I2C123SEL = PLL3_R,
     re-read from ArduPilot's stm32h7_mcuconf.h when a tree is given.
  2. STM32H7 TIMINGR decode (RM0433 "I2C timings") and a worst-case corner
     model of the master clock: tSYNC = analog filter (50..260 ns) + DNF x
     tI2CCLK + 2..3 tI2CCLK, SCL/SDA RC edges, input thresholds anywhere in
     the guaranteed 0.3..0.7 VDD window.  Checked against NXP UM10204 Fast-mode
     and the RM0433 SCLDEL/SDADEL rules.  ArduPilot's defaults are evaluated,
     then a TIMINGR is searched that holds for every Fm-legal bus (tr <= 300 ns).
  3. Bus electricals: capacitance budget incl. the GPS/compass cable, Rp
     window, sink current with module pull-ups in parallel, 33 R series
     resistor effect, noise margins.
  4. ESC crosstalk: capacitive (switch node dV/dt), inductive (hot-loop
     di/dt) and ground bounce onto SCL/SDA over In1 GND, compared with the
     noise margins and the 50 ns spike filters; derives routing rules.
  5. ArduPilot config: hwdef.dat must carry HAL_I2C_MAX_CLOCK 400000, the
     chosen HAL_I2C_H7_400_TIMINGR and a Standard-mode 100 kHz TIMINGR in
     HAL_I2C_F7_100_TIMINGR (what the I2CDevice ctor loads on H7 when any
     driver on the bus asks for <= 100 kHz) and HAL_I2C_H7_100_TIMINGR; with
     --ardupilot the board is generated with ArduPilot's chibios_hwdef.py and
     the defines are checked in hwdef.h.

Usage: python3 analysis/baro_i2c_analysis.py [--ardupilot /path/to/ardupilot]
       (or set ARDUPILOT_ROOT)
"""
import argparse
import json
import math
import os
import re
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

MU0 = 4e-7 * math.pi
C0 = 299792458.0

# ----------------------------------------------------------------------------
# Assumptions (all copied into the report)
# ----------------------------------------------------------------------------
A = {
    "vdd": {"nom": 3.3, "min": 3.135, "max": 3.465, "note": "+3V3 rail +/-5 %"},
    "um10204_fm": {   # NXP UM10204 rev 7, Table 10, Fast-mode
        "fscl_max_hz": 400e3, "tLOW_min_ns": 1300, "tHIGH_min_ns": 600,
        "tSU_DAT_min_ns": 100, "tHD_DAT_min_ns": 0, "tVD_DAT_max_ns": 900,
        "tSU_STA_min_ns": 600, "tHD_STA_min_ns": 600, "tSU_STO_min_ns": 600,
        "tBUF_min_ns": 1300, "tr_max_ns": 300, "tf_max_ns": 300,
        "tf_min_ns_formula": "20 ns x VDD/5.5 V", "tSP_max_ns": 50,
        "Cb_max_pF": 400, "VIL_frac": 0.3, "VIH_frac": 0.7,
        "VOL_max_V_at_3mA": 0.4, "Vhys_min_frac": 0.05,
        "VnL_frac": 0.1, "VnH_frac": 0.2, "Ci_max_pF": 10,
    },
    "um10204_sm": {"tLOW_min_ns": 4700, "tHIGH_min_ns": 4000, "tSU_DAT_min_ns": 250,
                   "tHD_DAT_min_ns": 0, "tVD_DAT_max_ns": 3450, "tSU_STA_min_ns": 4700,
                   "tHD_STA_min_ns": 4000, "tSU_STO_min_ns": 4000, "tBUF_min_ns": 4700,
                   "tr_max_ns": 1000, "tf_max_ns": 300, "fscl_max_hz": 100e3},
    "stm32h7_i2c": {
        "tAF_ns": [50.0, 260.0],          # RM0433/DS12110 analog filter delay
        "sync_clk": [2, 3],               # SCL synchronisation 2..3 I2CCLK
        "DNF": 0,                         # see firmware_evidence: not settable from hwdef
        "ANFOFF": 0,                      # analog filter ON (CR1 = 0 in ArduPilot)
        "Cio_pF": 5.0,                    # DS12110 C_IO
        "Ron_sink_ohm": 50.0,             # VOL 0.4 V @ 8 mA (DS12110 CMOS) -> <= 50 ohm
        "Vth_frac_window": [0.3, 0.7],    # only VIL max / VIH min are guaranteed
        "Vth_frac_typ": 0.5,
    },
    "dps310": {
        "Cin_pF": 10.0,                   # not in datasheet -> UM10204 Fm Ci max
        "Ron_sink_ohm": 133.0,            # Fm minimum: 3 mA at 0.4 V
        "tSP_ns": 50.0,                   # Fm-compliant input spike suppression (assumed)
        "pinout": {1: "GND", 2: "CSB", 3: "SDI", 4: "SCK", 5: "SDO", 6: "VDDIO", 7: "GND", 8: "VDD"},
        "pinout_confidence": "high (DPS310 datasheet v1.1 pin table; CSB high selects I2C, "
                             "SDO low -> 0x76, SDO high -> 0x77)",
    },
    "baseline": {"Rp_ohm": 2200.0, "Rs_ohm": 33.0, "Rp_tol": 0.01},
    "pcb": {   # JLC061612-1080, tools/gen_power_stage_pcb.py STACKUP
        "h_L1_In1_mm": 0.1375, "er": 3.91, "w_mm": 0.10, "t_mm": 0.035,
        "len_mcu_to_baro_mm": 20.0, "len_mcu_to_gps_conn_mm": 25.0,
    },
    "external": {
        "connector_pF": 2.0,              # JST-GH pin + pad
        "cable_pF_per_m": [50.0, 120.0],  # loose 26-28 AWG JST-GH lead, conductor to rest
        "cable_len_m": [0.0, 0.30],       # nominal design 20-30 cm; 0 = nothing plugged
        "cable_len_abuse_m": 0.50,
        "module_devices": 2,              # compass + 1 more device (LED driver / 2nd compass)
        "module_pF_per_device": 10.0,
        "module_trace_pF": 5.0,
        "module_pullup_ohm": [4700.0, 10000.0, None],  # None = module without pull-ups
        "module_pullup_warn_ohm": 2200.0,
    },
    "esc": {
        "v_bus_max": 16.8, "overshoot_frac": 0.30,
        "dvdt_design_V_per_ns": 5.0,      # overwritten by gate_drive_report dvdt_emi_max if present
        "didt_design_A_per_ns": 3.5,      # 35 A commutated in 10 ns (conservative bound)
        "plane_pair_mm": 0.35,            # In1 GND - In2 VBAT core
        "hot_loop_half_width_mm": 2.0,
    },
    "noise_budget": {
        "total_frac_of_VnH": 0.5, "ind_frac_of_VnL": 0.5,
        "ground_bounce_max_V": 0.10,
        "note": "no credit taken for the 50 ns spike filters; they are extra margin",
    },
    "timing_margins": {"tLOW_ns": 65.0, "tHIGH_ns": 30.0, "tSU_DAT_ns": 20.0,
                       "tVD_DAT_ns": 50.0, "fscl_max_hz": 400e3},
    # Sm (100 kHz) design margins on top of the worst-case corner model; used for the
    # 100 kHz TIMINGR that ArduPilot loads when any I2C1 driver asks for <= 100 kHz
    "timing_margins_sm": {"tLOW_ns": 100.0, "tHIGH_ns": 100.0, "tSU_DAT_ns": 50.0,
                          "tVD_DAT_ns": 100.0, "fscl_max_hz": 100e3},
    "rise_time_design_max_sm_ns": 1000.0, # 100 kHz TIMINGR must hold for ANY Sm-legal bus
    "rise_time_design_max_ns": 300.0,     # chosen TIMINGR must hold for ANY Fm-legal bus
    "fall_time_design_max_ns": 120.0,
    "fall_time_design_min_ns": 2.0,
}

AP_DEFAULTS = {"HAL_I2C_H7_400_TIMINGR": 0x00300F38, "HAL_I2C_H7_100_TIMINGR": 0x00707CBB,
               "HAL_I2C_F7_100_TIMINGR": 0x30812E3E, "HAL_I2C_MAX_CLOCK": 100000}

GATES = []
HW_KEYS = ("HAL_I2C_MAX_CLOCK", "HAL_I2C_H7_400_TIMINGR", "HAL_I2C_F7_100_TIMINGR",
           "HAL_I2C_H7_100_TIMINGR")


def gate(name, ok, detail):
    GATES.append({"gate": name, "pass": bool(ok), "detail": detail})
    print("  [%s] %s: %s" % ("PASS" if ok else "FAIL", name, detail))
    return ok


# ----------------------------------------------------------------------------
# 1. Clock
# ----------------------------------------------------------------------------
def i2c_kernel_clock(ap_root):
    hse = 24e6
    m, n, r, sel = 6, 72, 9, "STM32_I2C123SEL_PLL3_R_CK"
    src = "built-in (stm32h7_mcuconf.h 24 MHz block)"
    if ap_root:
        p = os.path.join(ap_root, "libraries/AP_HAL_ChibiOS/hwdef/common/stm32h7_mcuconf.h")
        txt = open(p).read()
        blk = txt.split("#elif STM32_HSECLK == 24000000U")
        m = int(re.search(r"STM32_PLL3_DIVM_VALUE\s+(\d+)", blk[1]).group(1))
        n = int(re.search(r"STM32_PLL3_DIVN_VALUE\s+(\d+)", blk[2]).group(1))
        r = int(re.search(r"STM32_PLL3_DIVR_VALUE\s+(\d+)", blk[2]).group(1))
        sel = re.search(r"#define STM32_I2C123SEL\s+(\S+)", txt).group(1)
        src = p
    f = hse / m * n / r
    return {"hse_hz": hse, "pll3_m": m, "pll3_n": n, "pll3_r": r, "i2c123sel": sel,
            "f_i2cclk_hz": f, "source": src}


# ----------------------------------------------------------------------------
# 2. TIMINGR model
# ----------------------------------------------------------------------------
def decode(t):
    return {"PRESC": (t >> 28) & 0xF, "SCLDEL": (t >> 20) & 0xF, "SDADEL": (t >> 16) & 0xF,
            "SCLH": (t >> 8) & 0xFF, "SCLL": t & 0xFF}


def encode(presc, scldel, sdadel, sclh, scll):
    return (presc << 28) | (scldel << 20) | (sdadel << 16) | (sclh << 8) | scll


def rc_from_tr(tr):
    return tr / math.log(7.0 / 3.0)        # tr(30-70 %) = 0.8473 RC


def rise_to(frac, rc):
    return rc * math.log(1.0 / (1.0 - frac))


def fall_to(frac, tf):
    return (1.0 - frac) / 0.4 * tf          # linear current-sink fall, tf = 70->30 %


def corners(tr_list, tf_list, dnf, tclk, typ=False):
    st = A["stm32h7_i2c"]
    taf_l = [sum(st["tAF_ns"]) / 2] if typ else st["tAF_ns"]
    sc_l = [sum(st["sync_clk"]) / 2] if typ else st["sync_clk"]
    vth_l = [st["Vth_frac_typ"]] if typ else st["Vth_frac_window"]
    for tr in tr_list:
        for tf in tf_list:
            for taf in taf_l:
                for sc in sc_l:
                    for vf in vth_l:
                        for vr in vth_l:
                            yield tr, tf, taf + (dnf + sc) * tclk, vf, vr


def evaluate(timingr, fclk, tr_list, tf_list, dnf=0, spec="um10204_fm", typ=False, margins=None):
    """Worst case over all corners.  Times in ns."""
    if margins is None:
        margins = spec == "um10204_fm"
    d = decode(timingr)
    tclk = 1e9 / fclk
    tp = (d["PRESC"] + 1) * tclk
    tscll = (d["SCLL"] + 1) * tp
    tsclh = (d["SCLH"] + 1) * tp
    tscldel = (d["SCLDEL"] + 1) * tp
    tsdadel = d["SDADEL"] * tp + tclk       # RM0433: [SDADEL x (PRESC+1) + 1] x tI2CCLK
    U = A[spec]
    w = {"T_min": 1e9, "T_max": 0, "tLOW_min": 1e9, "tHIGH_min": 1e9, "tSU_DAT_tx_min": 1e9,
         "tHD_DAT_tx_min": 1e9, "tVD_DAT_tx_max": 0, "tSU_STA_min": 1e9, "tSU_STO_min": 1e9}
    for tr, tf, tsync, vf, vr in corners(tr_list, tf_list, dnf, tclk, typ):
        rc = rc_from_tr(tr)
        t_fd = fall_to(vf, tf) + tsync                    # SCL low detected
        t_rel = t_fd + max(tscll, tsdadel + tscldel)      # SCL released
        t_rd = t_rel + rise_to(vr, rc) + tsync            # SCL high detected
        T = t_rd + tsclh                                  # next SCL fall starts
        scl_fall_03 = fall_to(0.3, tf)
        scl_fall_07 = fall_to(0.7, tf)
        tlow = t_rel + rise_to(0.3, rc) - scl_fall_03
        thigh = T + scl_fall_07 - (t_rel + rise_to(0.7, rc))
        t_sda = t_fd + tsdadel                            # master changes SDA
        sda_valid = max(t_sda + rise_to(0.7, rc), t_sda + fall_to(0.3, tf))
        sda_leave = min(t_sda + rise_to(0.3, rc), t_sda + fall_to(0.7, tf))
        tsu = t_rel + rise_to(0.3, rc) - sda_valid
        thd = sda_leave - scl_fall_03
        tvd = sda_valid - scl_fall_03
        w["T_min"] = min(w["T_min"], T)
        w["T_max"] = max(w["T_max"], T)
        w["tLOW_min"] = min(w["tLOW_min"], tlow)
        w["tHIGH_min"] = min(w["tHIGH_min"], thigh)
        w["tSU_DAT_tx_min"] = min(w["tSU_DAT_tx_min"], tsu)
        w["tHD_DAT_tx_min"] = min(w["tHD_DAT_tx_min"], thd)
        w["tVD_DAT_tx_max"] = max(w["tVD_DAT_tx_max"], tvd)
        # repeated START / STOP setup: SCL high detected, SCLH counted, SDA moves
        w["tSU_STA_min"] = min(w["tSU_STA_min"], thigh)
        w["tSU_STO_min"] = min(w["tSU_STO_min"], thigh)
    trmax, tfmax = max(tr_list), max(tf_list)
    w["tHD_STA_min"] = tsclh - tfmax
    w["tBUF_min"] = tscll - tfmax
    # master receive: slave may take the full tVD;DAT(max) after SCL 0.3 VDD
    w["tSU_DAT_rx_min"] = w["tLOW_min"] - U["tVD_DAT_max_ns"]
    w["fSCL_max_hz"] = 1e9 / w["T_min"]
    w["fSCL_min_hz"] = 1e9 / w["T_max"]
    st = A["stm32h7_i2c"]
    # RM0433 I2C timing rules
    rm_scldel_need = trmax + U["tSU_DAT_min_ns"]
    rm_sdadel_lo = (tfmax + 0 - st["tAF_ns"][0] - (dnf + 3) * tclk) / tp
    # RM0433: SDADEL <= {tVD;DAT(max) - tr(max) - 260 ns - [(DNF+4) x tI2CCLK]} / tPRESC,
    # and the field is 4 bits (0..15)
    rm_sdadel_hi = (U["tVD_DAT_max_ns"] - trmax - st["tAF_ns"][1] - (dnf + 4) * tclk) / tp
    sd_min = max(0, math.ceil(rm_sdadel_lo))
    sd_max = min(15, math.floor(rm_sdadel_hi))
    w["rm"] = {"tSCLDEL_ns": tscldel, "tSCLDEL_need_ns": rm_scldel_need,
               "SDADEL_min": sd_min, "SDADEL_max": sd_max,
               "ok_scldel": tscldel >= rm_scldel_need,
               "ok_sdadel": sd_min <= d["SDADEL"] <= sd_max}
    m = {"um10204_fm": A["timing_margins"], "um10204_sm": A["timing_margins_sm"]}[spec] if margins \
        else {k: 0 for k in A["timing_margins"]}
    checks = {
        "fSCL<=max": w["fSCL_max_hz"] <= U["fscl_max_hz"],
        "tLOW": w["tLOW_min"] >= U["tLOW_min_ns"] + m["tLOW_ns"],
        "tHIGH": w["tHIGH_min"] >= U["tHIGH_min_ns"] + m["tHIGH_ns"],
        "tSU;DAT tx": w["tSU_DAT_tx_min"] >= U["tSU_DAT_min_ns"] + m["tSU_DAT_ns"],
        "tSU;DAT rx": w["tSU_DAT_rx_min"] >= U["tSU_DAT_min_ns"],
        "tHD;DAT>=0": w["tHD_DAT_tx_min"] >= 0,
        "tVD;DAT": w["tVD_DAT_tx_max"] <= U["tVD_DAT_max_ns"] - m["tVD_DAT_ns"],
        "RM SCLDEL": w["rm"]["ok_scldel"],
        "RM SDADEL": w["rm"]["ok_sdadel"],
    }
    checks["tHD;STA"] = w["tHD_STA_min"] >= U["tHD_STA_min_ns"]
    checks["tBUF"] = w["tBUF_min"] >= U["tBUF_min_ns"]
    checks["tSU;STA"] = w["tSU_STA_min"] >= U["tSU_STA_min_ns"]
    checks["tSU;STO"] = w["tSU_STO_min"] >= U["tSU_STO_min_ns"]
    w["checks"] = checks
    w["all_pass"] = all(checks.values())
    w["fields"] = d
    w["counters_ns"] = {"tPRESC": tp, "tSCLL": tscll, "tSCLH": tsclh, "tSCLDEL": tscldel,
                        "tSDADEL_total": tsdadel}
    w["timingr"] = "0x%08X" % timingr
    return w


def search_timingr(fclk, tr_list, tf_list, dnf):
    best = None
    tclk = 1e9 / fclk
    for presc in range(0, 4):
        tp = (presc + 1) * tclk
        U = A["um10204_fm"]
        scldel = math.ceil((max(tr_list) + U["tSU_DAT_min_ns"] + A["timing_margins"]["tSU_DAT_ns"]) / tp) - 1
        if scldel > 15:
            continue
        for sdadel in range(0, 16):
            for sclh in range(int(500 / tp), int(1500 / tp)):
                for scll in range(int(1200 / tp), int(2600 / tp)):
                    if scll > 255 or sclh > 255:
                        continue
                    t = encode(presc, scldel, sdadel, sclh, scll)
                    w = evaluate(t, fclk, tr_list, tf_list, dnf)
                    if not w["all_pass"]:
                        continue
                    key = (w["T_max"], -w["tHIGH_min"], presc)
                    if best is None or key < best[0]:
                        best = (key, t, w)
            # first passing SDADEL (smallest) is preferred: keeps tVD;DAT short
            if best is not None and decode(best[1])["PRESC"] == presc:
                break
    return best


def search_timingr_sm(fclk, tr_list, tf_list, dnf):
    """Standard-mode (<= 100 kHz) TIMINGR for every Sm-legal bus (tr <= 1000 ns),
    with A["timing_margins_sm"], RM0433 SCLDEL/SDADEL rules; minimise worst-case period."""
    best = None
    tclk = 1e9 / fclk
    U, M = A["um10204_sm"], A["timing_margins_sm"]
    for presc in range(0, 16):
        tp = (presc + 1) * tclk
        scldel = math.ceil((max(tr_list) + U["tSU_DAT_min_ns"] + M["tSU_DAT_ns"]) / tp) - 1
        if scldel > 15:
            continue
        lo_h, hi_h = int(3500 / tp), int(6500 / tp) + 1
        lo_l, hi_l = int(4000 / tp), int(7500 / tp) + 1
        if lo_h > 255 or lo_l > 255:
            continue
        found = None
        for sdadel in range(0, 16):
            for sclh in range(lo_h, min(hi_h, 256)):
                for scll in range(lo_l, min(hi_l, 256)):
                    t = encode(presc, scldel, sdadel, sclh, scll)
                    w = evaluate(t, fclk, tr_list, tf_list, dnf, "um10204_sm", margins=True)
                    if not w["all_pass"]:
                        continue
                    key = (round(w["T_max"], 3), -w["tHIGH_min"], presc)
                    if found is None or key < found[0]:
                        found = (key, t, w)
            if found is not None:
                break                       # smallest passing SDADEL for this PRESC
        if found is not None and (best is None or found[0] < best[0]):
            best = found
    return best


# ----------------------------------------------------------------------------
# 3. Bus electricals
# ----------------------------------------------------------------------------
def microstrip(w, h, t, er):
    """Hammerstad-Jensen microstrip with thickness correction (w, h, t in mm)."""
    we = w + t / math.pi * (1 + math.log(2 * h / t))
    u = we / h
    eeff = (er + 1) / 2 + (er - 1) / 2 * (1 / math.sqrt(1 + 12 / u) + (0.04 * (1 - u) ** 2 if u < 1 else 0))
    if u <= 1:
        z0 = 60 / math.sqrt(eeff) * math.log(8 / u + u / 4)
    else:
        z0 = 120 * math.pi / math.sqrt(eeff) / (u + 1.393 + 0.667 * math.log(u + 1.444))
    c_pf_mm = math.sqrt(eeff) / (C0 * z0) * 1e12 / 1e3
    l_nh_mm = z0 * math.sqrt(eeff) / C0 * 1e9 / 1e3
    return {"z0_ohm": z0, "eeff": eeff, "C_pF_per_mm": c_pf_mm, "L_nH_per_mm": l_nh_mm,
            "tpd_ps_per_mm": math.sqrt(eeff) / C0 * 1e12 / 1e3}


def bus_electricals():
    p, e, st, dps = A["pcb"], A["external"], A["stm32h7_i2c"], A["dps310"]
    ms = microstrip(p["w_mm"], p["h_L1_In1_mm"], p["t_mm"], p["er"])
    onboard = {
        "mcu_pin": st["Cio_pF"], "dps310_pin": dps["Cin_pF"], "rs_pad": 0.3,
        "pullup_pad": 0.3,
        "trace": ms["C_pF_per_mm"] * (p["len_mcu_to_baro_mm"] + p["len_mcu_to_gps_conn_mm"]),
        "connector": e["connector_pF"],
    }
    c_on = sum(onboard.values())

    def ext(len_m, cpm):
        if len_m == 0:
            return 0.0
        return len_m * cpm + e["module_trace_pF"] + e["module_devices"] * e["module_pF_per_device"]

    c_min = c_on
    c_typ = c_on + ext(0.20, e["cable_pF_per_m"][0])
    c_max = c_on + ext(e["cable_len_m"][1], e["cable_pF_per_m"][1])
    c_abuse = c_on + ext(e["cable_len_abuse_m"], e["cable_pF_per_m"][1])
    vdd = A["vdd"]
    rp = A["baseline"]["Rp_ohm"]
    rp_lo, rp_hi = rp * (1 - A["baseline"]["Rp_tol"]), rp * (1 + A["baseline"]["Rp_tol"])

    def par(a, b):
        return a if b is None else a * b / (a + b)

    rows = []
    for mod in e["module_pullup_ohm"] + [e["module_pullup_warn_ohm"]]:
        for name, c in (("min_no_gps", c_min), ("typ_20cm", c_typ), ("max_30cm", c_max), ("abuse_50cm", c_abuse)):
            if name == "min_no_gps" and mod is not None:
                continue
            r_hi = par(rp_hi, mod)
            r_lo = par(rp_lo, mod)
            tr = 0.8473 * r_hi * c * 1e-12 * 1e9
            i_sink = vdd["max"] / r_lo                          # worst: full VDD across Rp
            # weakest (3 mA-class) driver as a resistor: VOL at its pin
            vol_weak = vdd["max"] * dps["Ron_sink_ohm"] / (dps["Ron_sink_ohm"] + r_lo)
            rows.append({"module_pullup_ohm": mod, "case": name, "Cbus_pF": round(c, 1),
                         "Rp_eff_ohm": round(r_hi, 1), "tr_ns": round(tr, 1),
                         "I_sink_mA": round(i_sink * 1e3, 3), "VOL_weak_driver_V": round(vol_weak, 3),
                         "tau_ns": round(r_hi * c * 1e-3, 1)})
    # Rp window (no module pull-up, design Cbus max)
    rp_min = (vdd["max"] - A["um10204_fm"]["VOL_max_V_at_3mA"]) / 3e-3
    rp_max = A["um10204_fm"]["tr_max_ns"] * 1e-9 / (0.8473 * c_max * 1e-12)
    # with the lowest module pull-up still accepted (4.7 k) the local Rp must stay above:
    mod_min = min(m for m in e["module_pullup_ohm"] if m)
    rp_min_with_module = rp_min * mod_min / (mod_min - rp_min)
    # 33 R series resistor at U4
    rs = A["baseline"]["Rs_ohm"]
    i_max = vdd["max"] / par(rp_lo, mod_min)
    rs_drop = i_max * rs
    vil = A["um10204_fm"]["VIL_frac"] * vdd["min"]
    vol_mcu = vdd["max"] * st["Ron_sink_ohm"] / (st["Ron_sink_ohm"] + par(rp_lo, mod_min))
    vol_dps = vdd["max"] * dps["Ron_sink_ohm"] / (dps["Ron_sink_ohm"] + par(rp_lo, mod_min))
    margin_low_at_dps = vil - (vol_mcu + rs_drop)     # MCU drives, DPS310 receives through Rs
    margin_low_at_mcu = vil - (vol_dps + rs_drop)     # DPS310 drives through Rs
    margin_low_at_module = vil - vol_dps              # DPS310/compass driving, module receives
    rs_rc_ns = rs * dps["Cin_pF"] * 1e-3
    trace_len = p["len_mcu_to_baro_mm"]
    tpd_trace = trace_len * ms["tpd_ps_per_mm"] * 1e-3
    tf_fast_ns = A["fall_time_design_min_ns"]
    return {
        "microstrip_L1_over_In1": ms,
        "Cbus_onboard_pF": onboard, "Cbus_onboard_total_pF": c_on,
        "Cbus_pF": {"min_no_gps": c_min, "typ_20cm": c_typ, "max_30cm": c_max, "abuse_50cm": c_abuse},
        "cases": rows,
        "Rp_window": {"Rp_min_ohm_VOL_0V4_3mA": rp_min, "Rp_max_ohm_tr300_at_Cmax": rp_max,
                      "Rp_min_ohm_with_4k7_module": rp_min_with_module, "baseline_Rp_ohm": rp},
        "series_R": {
            "Rs_ohm": rs, "I_max_mA": i_max * 1e3, "Rs_drop_mV": rs_drop * 1e3,
            "VIL_V": vil, "VOL_mcu_V": vol_mcu, "VOL_dps310_V": vol_dps,
            "low_margin_at_dps310_V": margin_low_at_dps, "low_margin_at_mcu_V": margin_low_at_mcu,
            "low_margin_at_module_V": margin_low_at_module,
            "Rs_Cin_tau_ns": rs_rc_ns,
            "trace_tpd_ns": tpd_trace, "fastest_edge_ns": tf_fast_ns,
            "lumped": tf_fast_ns > 6 * tpd_trace,
            "ringing_note": "MCU-U4 trace is electrically short (tpd %.2f ns vs >= %.0f ns edges): "
                            "no transmission-line ringing on board, Rs vs Z0 matching is irrelevant. "
                            "33 R sets a 0.33 ns pole with the DPS310 pin: no noise filtering, "
                            "only fault/ESD current limiting and branch isolation." % (tpd_trace, tf_fast_ns),
        },
        "hysteresis": {"um10204_fm_min_V": A["um10204_fm"]["Vhys_min_frac"] * vdd["min"],
                       "note": "STM32H7 FT I/O and DPS310 are Schmitt inputs; only the Fm minimum "
                               "0.05 VDD is relied on"},
        "noise_margin": {"VnL_V": A["um10204_fm"]["VnL_frac"] * vdd["min"],
                         "VnH_V": A["um10204_fm"]["VnH_frac"] * vdd["min"]},
        "tf_min_fm_ns": 20 * vdd["min"] / 5.5,
    }


# ----------------------------------------------------------------------------
# 4. ESC crosstalk
# ----------------------------------------------------------------------------
def esc_inputs():
    e = dict(A["esc"])
    src = "assumptions"
    p = os.path.join(REPO, "am32_esc", "gate_drive_report.json")
    if os.path.exists(p):
        try:
            g = json.load(open(p))
            e["dvdt_design_V_per_ns"] = max(e["dvdt_design_V_per_ns"],
                                            g["assumptions"]["dvdt_emi_max"] * 1e-9)
            s = g["selected_gate_network"]["stress_4S_HV_35A"]
            e["selected_dvdt_V_per_ns"] = s["dvdt_V_per_ns"]
            e["selected_vds_peak_V"] = s.get("vds_peak_V")
            e["selected_didt_A_per_ns"] = s["i"] / s["t_fall_fast_ns"]
            src = p
        except (KeyError, ValueError, TypeError):
            pass
    e["source"] = src
    vstep = max(e["v_bus_max"] * (1 + e["overshoot_frac"]), e.get("selected_vds_peak_V") or 0)
    e["v_step_V"] = vstep
    return e


def crosstalk(bus, esc):
    p = A["pcb"]
    h = p["h_L1_In1_mm"]
    ms = bus["microstrip_L1_over_In1"]
    cbus_min = bus["Cbus_pF"]["min_no_gps"]
    rp = A["baseline"]["Rp_ohm"]
    tau = rp * cbus_min * 1e-3                        # ns, line HIGH, nothing plugged in
    dvdt = esc["dvdt_design_V_per_ns"]
    t_edge = esc["v_step_V"] / dvdt
    didt = esc["didt_design_A_per_ns"]
    vnh, vnl = bus["noise_margin"]["VnH_V"], bus["noise_margin"]["VnL_V"]
    nb = A["noise_budget"]

    def k_c(s):                                       # Johnson & Graham 1/(1+(D/h)^2)
        D = s + p["w_mm"] / 2
        return 1 / (1 + (D / h) ** 2)

    def v_cap(s, L):
        cm = k_c(s) * ms["C_pF_per_mm"] * L           # pF
        # ramp of duration t_edge into Rp || Cbus
        v = rp * cm * 1e-12 * dvdt * 1e9 * (1 - math.exp(-t_edge / tau))
        return min(v, cm / (cm + cbus_min) * esc["v_step_V"]), cm

    def v_ind(s, L):
        D = s + p["w_mm"] / 2
        m_per_mm = MU0 / (4 * math.pi) * math.log(1 + (2 * h / D) ** 2) * 1e-3  # H/mm
        return m_per_mm * L * didt * 1e9, m_per_mm * L * 1e9                     # V, nH

    def v_gb(D):
        a = A["esc"]["hot_loop_half_width_mm"]
        lsq = MU0 * A["esc"]["plane_pair_mm"] * 1e-3  # H per square
        return lsq * didt * 1e9 * (a / (a + D)) ** 2, lsq * 1e9

    # ground bounce keep-out
    d_gb = 0.0
    while v_gb(d_gb)[0] > nb["ground_bounce_max_V"]:
        d_gb += 0.1
    vgb_rule = v_gb(d_gb)[0]
    rules = []
    for L in (5.0, 10.0, 20.0, 40.0):
        s = 0.1
        while True:
            vc, cm = v_cap(s, L)
            vi, lm = v_ind(s, L)
            ok_h = vc + vi + vgb_rule <= nb["total_frac_of_VnH"] * vnh
            ok_l = vi + vgb_rule <= nb["ind_frac_of_VnL"] * vnl + 1e-12
            if ok_h and ok_l:
                break
            s = round(s + 0.1, 2)
        rules.append({"parallel_len_mm": L, "min_edge_gap_mm": s, "Cm_pF": cm, "Lm_nH": lm,
                      "V_cap_V": vc, "V_ind_V": vi, "V_gb_V": vgb_rule,
                      "V_total_high_V": vc + vi + vgb_rule})
    # glitch widths vs 50 ns filters at a reference violation (0.2 mm gap, 20 mm run)
    vc_ref, _ = v_cap(0.2, 20.0)
    vi_ref, _ = v_ind(0.2, 20.0)
    thr = 0.5 * vnh
    w_cap = tau * math.log(vc_ref / thr) if vc_ref > thr else 0.0
    t_filt = A["stm32h7_i2c"]["tAF_ns"][0]
    if vc_ref <= thr:
        cap_verdict = ("capacitive glitch %.3f V stays below half VnH (%.3f V): no filter needed"
                       % (vc_ref, thr))
    elif w_cap <= t_filt:
        cap_verdict = ("capacitive glitch %.3f V exceeds half VnH (%.3f V) for %.0f ns, shorter than the "
                       "%.0f ns STM32 analog filter / DPS310 tSP: filtered, but only by the filters "
                       "-> not allowed by the rules" % (vc_ref, thr, w_cap, t_filt))
    else:
        cap_verdict = ("capacitive glitch %.3f V exceeds half VnH (%.3f V) for %.0f ns, longer than the "
                       "%.0f ns STM32 analog filter / DPS310 tSP: not suppressed"
                       % (vc_ref, thr, w_cap, t_filt))
    return {
        "inputs": {"dvdt_V_per_ns": dvdt, "v_step_V": esc["v_step_V"], "edge_ns": t_edge,
                   "didt_A_per_ns": didt, "Cbus_victim_pF": cbus_min, "Rp_ohm": rp,
                   "tau_high_ns": tau, "VnH_V": vnh, "VnL_V": vnl, "source": esc["source"],
                   "selected_network_dvdt_V_per_ns": esc.get("selected_dvdt_V_per_ns"),
                   "selected_network_didt_A_per_ns": esc.get("selected_didt_A_per_ns")},
        "model": "Victim: 0.1 mm L1 microstrip over In1 GND (h 0.1375 mm). Coupling "
                 "k = 1/(1+(D/h)^2) to the switch-node/hot-loop copper edge, Cm = k C, "
                 "M = mu0/4pi ln(1+(2h/D)^2). Capacitive glitch decays with Rp*Cbus "
                 "(tau %.0f ns, comparable to the 50 ns filters, so how long it stays above "
                 "threshold depends on its amplitude); no filter credit is taken. Inductive glitch "
                 "lasts ~the current edge (%.0f ns) and sees the full EMF in the LOW state. "
                 "Ground bounce: plane-pair inductance %.2f nH/sq x di/dt, dipole decay "
                 "(a/(a+D))^2 outside the hot loop." % (tau, 35.0 / didt, v_gb(0)[1]),
        "ground_bounce": {"V_if_path_crosses_hot_loop_V": v_gb(0)[0],
                          "keepout_mm": d_gb, "V_at_keepout_V": vgb_rule},
        "spacing_rules": rules,
        "reference_violation_0p2mm_20mm": {
            "V_cap_V": vc_ref, "V_ind_V": vi_ref,
            "half_VnH_V": thr,
            "cap_width_above_half_VnH_ns": w_cap,
            "filter_verdict": cap_verdict,
            "ind_verdict": "inductive glitch lasts ~the %.0f ns current edge, inside the 50 ns analog filter / "
                           "DPS310 tSP, but its amplitude would rely on the filters (and on ESD diode "
                           "clamping) -> not allowed by the rules" % (35.0 / didt),
        },
        "filters": {"stm32_analog_ns": A["stm32h7_i2c"]["tAF_ns"][0], "stm32_DNF": 0,
                    "dps310_tSP_ns": A["dps310"]["tSP_ns"]},
    }


# ----------------------------------------------------------------------------
# 5. ArduPilot evidence / config
# ----------------------------------------------------------------------------
def firmware_evidence(ap_root):
    ev = {"checked": False}
    if not ap_root:
        return ev
    i2c = open(os.path.join(ap_root, "libraries/AP_HAL_ChibiOS/I2CDevice.cpp")).read()
    lld = open(os.path.join(ap_root, "modules/ChibiOS/os/hal/ports/STM32/LLD/I2Cv3/hal_i2c_lld.c")).read()
    ev["checked"] = True
    for k in AP_DEFAULTS:
        m = re.search(r"#ifndef %s\s*\n#define %s (\w+)" % (k, k), i2c)
        ev[k] = m.group(1) if m else None
    ev["defaults_overridable_ifndef"] = all(ev[k] is not None for k in AP_DEFAULTS)
    ev["max_clock_default_100k"] = ev["HAL_I2C_MAX_CLOCK"] == "100000"
    ev["cr1_only_smbhen"] = bool(re.search(r"i2ccfg\.cr1 \|= I2C_CR1_SMBHEN", i2c)) and \
        "I2C_CR1_DNF" not in i2c and "ANFOFF" not in i2c
    ev["lld_cr1_from_config"] = "dp->CR1 = i2cp->config->cr1" in lld
    ev["h7_low_clock_uses_F7_100_timingr"] = bool(re.search(
        r"bus_clock <= 100000\) \{\s*bus\.i2ccfg\.timingr = HAL_I2C_F7_100_TIMINGR", i2c))
    ev["h7_ctor_low_clock_body"] = bool(re.search(
        r"if \(bus_clock < bus\.busclock\) \{\s*#if defined\(STM32F7\) \|\| defined\(STM32H7\)[^\n]*\n"
        r"\s*if \(bus_clock <= 100000\) \{\s*bus\.i2ccfg\.timingr = HAL_I2C_F7_100_TIMINGR", i2c))
    drivers = {}
    for rel in ("libraries/AP_BattMonitor/AP_BattMonitor_INA2xx.cpp",
                "libraries/AP_BattMonitor/AP_BattMonitor_INA3221.cpp",
                "libraries/AP_BattMonitor/AP_BattMonitor_SMBus.cpp",
                "libraries/AP_BattMonitor/AP_BattMonitor_LTC2946.cpp",
                "libraries/AP_DAC/AP_DAC_MCP40D1x.cpp"):
        fp = os.path.join(ap_root, rel)
        if os.path.exists(fp):
            drivers[os.path.basename(rel)] = bool(re.search(
                r"get_device(?:_ptr)?\([^;]*?,\s*100000\s*,", open(fp).read(), re.S))
    ev["drivers_requesting_100kHz"] = drivers
    ev["clear_on_timeout"] = "HAL_I2C_CLEAR_ON_TIMEOUT 1" in i2c
    m = re.search(r"for\(uint8_t j = 0; j < (\d+); j\+\+\)[^}]*delay_microseconds\((\d+)\)", i2c, re.S)
    ev["clear_bus_toggles"], ev["clear_bus_half_period_us"] = (int(m.group(1)), int(m.group(2))) if m else (None, None)
    return ev


def hwdef_defines():
    txt = open(os.path.join(REPO, "ardupilot", "hwdef.dat")).read()
    out = {}
    for k in HW_KEYS:
        m = re.findall(r"^define %s (\S+)" % k, txt, re.M)
        out[k] = m
    return out


def generated_header_check(ap_root):
    sys.path.insert(0, os.path.join(REPO, "tools"))
    import validate_dma as vd                         # reuse its staging + generator wrapper
    tmp = tempfile.mkdtemp(prefix="nanoh743_i2c_")
    try:
        bdir = vd.stage(tmp)
        out = os.path.join(tmp, "fw")
        os.makedirs(out)
        rc, log = vd.run_generator(ap_root, os.path.join(bdir, "hwdef.dat"), out,
                                   params=os.path.join(bdir, "defaults.parm"))
        if rc != 0:
            return {"rc": rc, "log": log[-2000:]}
        hdr = open(os.path.join(out, "hwdef.h")).read()
        res = {"rc": 0}
        for k in HW_KEYS:
            res[k] = re.findall(r"#define %s (\S+)" % k, hdr)
        res["i2c_order"] = re.findall(r"#define HAL_I2C_DEVICE_LIST (.*)", hdr)
        res["baro_probe"] = re.findall(r"#define HAL_BARO_PROBE_LIST (.*)", hdr)
        return res
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ardupilot", default=os.environ.get("ARDUPILOT_ROOT"))
    args = ap.parse_args()
    ap_root = os.path.abspath(args.ardupilot) if args.ardupilot and os.path.isdir(args.ardupilot) else None

    print("== 1. I2C kernel clock ==")
    clk = i2c_kernel_clock(ap_root)
    fclk = clk["f_i2cclk_hz"]
    gate("C1 I2C123 kernel clock", clk["i2c123sel"].endswith("PLL3_R_CK") and abs(fclk - 32e6) < 1,
         "%s, PLL3 M=%d N=%d R=%d -> %.3f MHz (%s)" % (clk["i2c123sel"], clk["pll3_m"], clk["pll3_n"],
                                                     clk["pll3_r"], fclk / 1e6, clk["source"]))

    print("== 3. bus electricals ==")
    bus = bus_electricals()
    for r in bus["cases"]:
        print("    module Rp %-6s %-11s Cbus %6.1f pF  Rp_eff %6.0f  tr %5.1f ns  Isink %.2f mA  VOL(3mA drv) %.3f V"
              % (r["module_pullup_ohm"], r["case"], r["Cbus_pF"], r["Rp_eff_ohm"], r["tr_ns"],
                 r["I_sink_mA"], r["VOL_weak_driver_V"]))
    design = [r for r in bus["cases"] if r["case"] != "abuse_50cm" and
              r["module_pullup_ohm"] != A["external"]["module_pullup_warn_ohm"]]
    tr_max_bus = max(r["tr_ns"] for r in design)
    tr_min_bus = min(r["tr_ns"] for r in bus["cases"])
    gate("E1 tr <= 300 ns over design bus (no-GPS .. 30 cm cable, module Rp none/4k7/10k)",
         tr_max_bus <= A["um10204_fm"]["tr_max_ns"], "worst %.1f ns" % tr_max_bus)
    worst_vol = max(r["VOL_weak_driver_V"] for r in design)
    gate("E2 VOL <= 0.4 V for a 3 mA-class driver incl. module pull-ups", worst_vol <= 0.4,
         "worst %.3f V" % worst_vol)
    rw = bus["Rp_window"]
    gate("E3 Rp 2.2 k inside window", rw["Rp_min_ohm_with_4k7_module"] <= 2200 * 0.99 and
         2200 * 1.01 <= rw["Rp_max_ohm_tr300_at_Cmax"],
         "%.0f (with 4k7 module) .. %.0f ohm (tr 300 ns at %.0f pF)" %
         (rw["Rp_min_ohm_with_4k7_module"], rw["Rp_max_ohm_tr300_at_Cmax"], bus["Cbus_pF"]["max_30cm"]))
    sr = bus["series_R"]
    gate("E4 33 R series: low-level margin at every receiver >= VnL",
         min(sr["low_margin_at_dps310_V"], sr["low_margin_at_mcu_V"]) >= bus["noise_margin"]["VnL_V"],
         "VIL %.3f V, margin at U4 %.3f V, at MCU %.3f V (Rs drop %.0f mV)" %
         (sr["VIL_V"], sr["low_margin_at_dps310_V"], sr["low_margin_at_mcu_V"], sr["Rs_drop_mV"]))
    gate("E5 on-board I2C1 trace is lumped (no ringing)", sr["lumped"],
         "tpd %.3f ns vs edge >= %.0f ns" % (sr["trace_tpd_ns"], sr["fastest_edge_ns"]))
    gate("E6 Cbus <= 400 pF (Fm) incl. 50 cm abuse cable", bus["Cbus_pF"]["abuse_50cm"] <= 400,
         "%.1f pF" % bus["Cbus_pF"]["abuse_50cm"])
    warn_2k2 = [r for r in bus["cases"] if r["module_pullup_ohm"] == A["external"]["module_pullup_warn_ohm"]]
    abuse = [r for r in bus["cases"] if r["case"] == "abuse_50cm"]

    print("== 2. TIMINGR ==")
    ev = firmware_evidence(ap_root)
    if ev["checked"]:
        gate("F1 ArduPilot TIMINGR/MAX_CLOCK defaults match and are #ifndef-overridable",
             ev["defaults_overridable_ifndef"] and
             int(ev["HAL_I2C_H7_400_TIMINGR"], 16) == AP_DEFAULTS["HAL_I2C_H7_400_TIMINGR"],
             "H7_400 %s, H7_100 %s, MAX_CLOCK %s" % (ev["HAL_I2C_H7_400_TIMINGR"],
                                                    ev["HAL_I2C_H7_100_TIMINGR"], ev["HAL_I2C_MAX_CLOCK"]))
        gate("F2 CR1 DNF/ANFOFF not settable from hwdef (CR1 = config->cr1, ArduPilot sets SMBHEN only)",
             ev["cr1_only_smbhen"] and ev["lld_cr1_from_config"], "DNF = 0, analog filter ON")
        req100 = sorted(k for k, v in ev["drivers_requesting_100kHz"].items() if v)
        gate("F3 on H7 a device opened at <= 100 kHz lowers the bus to HAL_I2C_F7_100_TIMINGR, and "
             "external-bus drivers do request 100 kHz",
             ev["h7_low_clock_uses_F7_100_timingr"] and ev["h7_ctor_low_clock_body"] and len(req100) > 0,
             "I2CDevice ctor -> HAL_I2C_F7_100_TIMINGR (HAL_I2C_H7_100_TIMINGR only used if MAX_CLOCK <= "
             "100 kHz); 100 kHz requesters: %s; I2C1 is external (HAL_I2C_INTERNAL_MASK 0)" % req100)
    tf_list = [A["fall_time_design_min_ns"], A["fall_time_design_max_ns"]]
    tr_design = [max(5.0, tr_min_bus), A["rise_time_design_max_ns"]]
    sweep_tr = [tr_min_bus] + [float(x) for x in range(50, 451, 25)]
    dnf = A["stm32h7_i2c"]["DNF"]

    def summary(w):
        return {k: (round(v, 1) if isinstance(v, float) else v) for k, v in w.items()
                if k not in ("rm", "checks", "fields", "counters_ns")} | {
            "checks": w["checks"], "rm": w["rm"], "fields": w["fields"],
            "counters_ns": w["counters_ns"]}

    def tr_sweep(t, spec="um10204_fm"):
        rows = []
        for tr in sweep_tr:
            w = evaluate(t, fclk, [tr_min_bus, tr], tf_list, dnf, spec)
            rows.append({"tr_max_ns": tr, "fSCL_min_kHz": round(w["fSCL_min_hz"] / 1e3, 1),
                         "fSCL_max_kHz": round(w["fSCL_max_hz"] / 1e3, 1),
                         "tLOW_min_ns": round(w["tLOW_min"], 1), "tHIGH_min_ns": round(w["tHIGH_min"], 1),
                         "tSU_DAT_min_ns": round(w["tSU_DAT_tx_min"], 1),
                         "tVD_DAT_max_ns": round(w["tVD_DAT_tx_max"], 1),
                         "fail": [k for k, v in w["checks"].items() if not v]})
        return rows

    def max_ok_tr(rows):
        ok = [r["tr_max_ns"] for r in rows if not r["fail"]]
        return max(ok) if ok else None

    dflt = evaluate(AP_DEFAULTS["HAL_I2C_H7_400_TIMINGR"], fclk, tr_design, tf_list, dnf)
    dflt_typ = evaluate(AP_DEFAULTS["HAL_I2C_H7_400_TIMINGR"], fclk, [tr_min_bus, tr_max_bus], tf_list, dnf)
    dflt_sweep = tr_sweep(AP_DEFAULTS["HAL_I2C_H7_400_TIMINGR"])
    print("    default 0x00300F38: %s  tHIGH_min %.0f ns  tSCLDEL %.0f ns (need %.0f)  fails at tr<=300: %s"
          % (dflt["fields"], dflt["tHIGH_min"], dflt["rm"]["tSCLDEL_ns"], dflt["rm"]["tSCLDEL_need_ns"],
             [k for k, v in dflt["checks"].items() if not v]))
    sm = evaluate(AP_DEFAULTS["HAL_I2C_H7_100_TIMINGR"], fclk, [tr_min_bus, 1000.0], tf_list, dnf, "um10204_sm")
    f7 = evaluate(AP_DEFAULTS["HAL_I2C_F7_100_TIMINGR"], fclk, [tr_min_bus, 1000.0], tf_list, dnf, "um10204_sm")
    print("    ArduPilot 100 kHz path (MAX_CLOCK default): 0x00707CBB fSCL %.1f..%.1f kHz, fails %s"
          % (sm["fSCL_min_hz"] / 1e3, sm["fSCL_max_hz"] / 1e3, [k for k, v in sm["checks"].items() if not v]))
    print("    ArduPilot ctor lowering on H7 (F7_100): 0x30812E3E fSCL %.1f..%.1f kHz, fails %s"
          % (f7["fSCL_min_hz"] / 1e3, f7["fSCL_max_hz"] / 1e3, [k for k, v in f7["checks"].items() if not v]))
    tr_sm = [max(5.0, tr_min_bus), A["rise_time_design_max_sm_ns"]]
    best_sm = search_timingr_sm(fclk, tr_sm, tf_list, dnf)
    chosen_sm = best_sm[1] if best_sm else None
    if best_sm is None:
        gate("S1 100 kHz TIMINGR found meeting Sm for tr <= 1000 ns", False, "no solution")
    else:
        ws = best_sm[2]
        gate("S1 100 kHz TIMINGR meets UM10204 Sm with margin for every Sm-legal bus (tr <= 1000 ns)",
             ws["all_pass"],
             "0x%08X %s: fSCL %.1f..%.1f kHz, tLOW>=%.0f, tHIGH>=%.0f, tSU;STA>=%.0f, tBUF>=%.0f, "
             "tSU;DAT>=%.0f, tVD;DAT<=%.0f ns"
             % (chosen_sm, ws["fields"], ws["fSCL_min_hz"] / 1e3, ws["fSCL_max_hz"] / 1e3, ws["tLOW_min"],
                ws["tHIGH_min"], ws["tSU_STA_min"], ws["tBUF_min"], ws["tSU_DAT_tx_min"], ws["tVD_DAT_tx_max"]))
        gate("S2 100 kHz TIMINGR meets RM0433 SCLDEL/SDADEL rules (tr 1000 ns, tSU;DAT 250 ns)",
             ws["rm"]["ok_scldel"] and ws["rm"]["ok_sdadel"],
             "tSCLDEL %.1f >= %.1f ns, SDADEL %d in [%d, %d]" % (
                 ws["rm"]["tSCLDEL_ns"], ws["rm"]["tSCLDEL_need_ns"], ws["fields"]["SDADEL"],
                 ws["rm"]["SDADEL_min"], ws["rm"]["SDADEL_max"]))
        gate("S3 default F7_100 0x30812E3E (used on H7 by the ctor) is non-compliant -> override justified",
             not f7["all_pass"], "fails %s (tSCLDEL %.0f < %.0f ns)" % (
                 [k for k, v in f7["checks"].items() if not v], f7["rm"]["tSCLDEL_ns"],
                 f7["rm"]["tSCLDEL_need_ns"]))
    chosen_sm_sweep = []
    if chosen_sm:
        for trx in [tr_min_bus] + [float(x) for x in range(100, 1101, 100)]:
            wx = evaluate(chosen_sm, fclk, [tr_min_bus, trx], tf_list, dnf, "um10204_sm", margins=True)
            chosen_sm_sweep.append({"tr_max_ns": trx, "fSCL_min_kHz": round(wx["fSCL_min_hz"] / 1e3, 1),
                                    "fSCL_max_kHz": round(wx["fSCL_max_hz"] / 1e3, 1),
                                    "fail": [k for k, v in wx["checks"].items() if not v]})

    best = search_timingr(fclk, tr_design, tf_list, dnf)
    if best is None:
        gate("T1 TIMINGR found meeting Fm for tr <= 300 ns", False, "no solution")
        chosen = None
    else:
        chosen = best[1]
        w = best[2]
        gate("T1 TIMINGR found meeting Fm with margin for every Fm-legal bus (tr <= 300 ns)", w["all_pass"],
             "0x%08X %s: fSCL %.1f..%.1f kHz, tLOW>=%.0f, tHIGH>=%.0f, tSU;DAT>=%.0f, tVD;DAT<=%.0f ns"
             % (chosen, w["fields"], w["fSCL_min_hz"] / 1e3, w["fSCL_max_hz"] / 1e3, w["tLOW_min"],
                w["tHIGH_min"], w["tSU_DAT_tx_min"], w["tVD_DAT_tx_max"]))
        gate("T2 chosen TIMINGR meets RM0433 SCLDEL/SDADEL rules", w["rm"]["ok_scldel"] and w["rm"]["ok_sdadel"],
             "tSCLDEL %.1f >= %.1f ns, SDADEL %d in [%d, %d]" % (
                 w["rm"]["tSCLDEL_ns"], w["rm"]["tSCLDEL_need_ns"], w["fields"]["SDADEL"],
                 w["rm"]["SDADEL_min"], w["rm"]["SDADEL_max"]))
        gate("T3 default 0x00300F38 is non-compliant/marginal -> override justified", not dflt["all_pass"],
             "fails %s at tr 300 ns; on this bus (tr<=%.0f ns) fails %s"
             % ([k for k, v in dflt["checks"].items() if not v], tr_max_bus,
                [k for k, v in dflt_typ["checks"].items() if not v]))
    chosen_sweep = tr_sweep(chosen) if chosen else []
    typ_tr = [r["tr_ns"] for r in bus["cases"] if r["case"] == "typ_20cm" and r["module_pullup_ohm"] == 4700.0][0]
    typical = {}
    for name, t in (("default_0x00300F38", AP_DEFAULTS["HAL_I2C_H7_400_TIMINGR"]), ("chosen", chosen)):
        if t is None:
            continue
        wt = evaluate(t, fclk, [typ_tr], [10.0], dnf, typ=True)
        typical[name] = {"tr_ns": typ_tr, "fSCL_kHz": round(wt["fSCL_min_hz"] / 1e3, 1),
                         "tLOW_ns": round(wt["tLOW_min"], 1), "tHIGH_ns": round(wt["tHIGH_min"], 1)}
        print("    typical (tAF 155 ns, 2.5 clk sync, Vth 0.5 VDD, tr %.0f ns) %s: fSCL %.1f kHz tLOW %.0f tHIGH %.0f ns"
              % (typ_tr, name, typical[name]["fSCL_kHz"], typical[name]["tLOW_ns"], typical[name]["tHIGH_ns"]))
    chosen_bus = summary(evaluate(chosen, fclk, [tr_min_bus, tr_max_bus], tf_list, dnf)) if chosen else None

    print("== 4. ESC crosstalk ==")
    esc = esc_inputs()
    xt = crosstalk(bus, esc)
    for r in xt["spacing_rules"]:
        print("    parallel %4.0f mm -> min gap %.1f mm (Vcap %.3f V, Vind %.3f V, Vgb %.3f V)"
              % (r["parallel_len_mm"], r["min_edge_gap_mm"], r["V_cap_V"], r["V_ind_V"], r["V_gb_V"]))
    gate("N1 routing rules exist that keep ESC glitches < 50 % VnH / VnL without filter credit",
         all(r["min_edge_gap_mm"] <= 3.0 for r in xt["spacing_rules"] if r["parallel_len_mm"] <= 20),
         "gaps %s mm for runs 5/10/20/40 mm, ground keep-out %.1f mm" %
         ([r["min_edge_gap_mm"] for r in xt["spacing_rules"]], xt["ground_bounce"]["keepout_mm"]))
    rule_gap = max(r["min_edge_gap_mm"] for r in xt["spacing_rules"] if r["parallel_len_mm"] <= 20)
    routing = {
        "layer": "I2C1_SCL/SDA, BARO_SCL/SDA on L1 F.Cu over unbroken In1 GND only (L6 over In4 is the "
                 "other option); no inner signal layers exist (In1..In4 are planes). Never cross a plane split "
                 "or a via-antipad field.",
        "min_gap_to_switch_node_or_hot_loop_copper_mm": rule_gap,
        "max_parallel_run_alongside_ESC_copper_mm": 20.0,
        "gap_table": [{"parallel_len_mm": r["parallel_len_mm"], "min_gap_mm": r["min_edge_gap_mm"]}
                      for r in xt["spacing_rules"]],
        "hot_loop_keepout_mm": xt["ground_bounce"]["keepout_mm"],
        "u4_and_i2c_ground_path": "U4 GND pins and U1 VSS must connect through In1 without the "
                                  "straight path between them crossing any FET-pair + input-cap hot loop "
                                  "(%.2f V bounce if it does)." % xt["ground_bounce"]["V_if_path_crosses_hot_loop_V"],
        "max_onboard_length_mm": {"mcu_to_u4": A["pcb"]["len_mcu_to_baro_mm"],
                                  "mcu_to_gps_connector": A["pcb"]["len_mcu_to_gps_conn_mm"]},
        "guard": "GND pour between I2C pair and any ESC copper, stitched to In1 every <= 3 mm; SCL and SDA "
                 "may run together (same-bus crosstalk is harmless at 400 kHz).",
        "placement": "R23/R24 within 5 mm of U1 PB6/PB7; R25/R26 within 2 mm of U4 pins 4/3; "
                     "C25/C26 at U4 pins 8/6 with own GND vias.",
    }

    print("== 5. ArduPilot config ==")
    hw = hwdef_defines()
    gate("A1 hwdef.dat: define HAL_I2C_MAX_CLOCK 400000 (ArduPilot default is 100 kHz)",
         hw["HAL_I2C_MAX_CLOCK"] == ["400000"], "found %s" % hw["HAL_I2C_MAX_CLOCK"])
    gate("A2 hwdef.dat: define HAL_I2C_H7_400_TIMINGR = chosen value",
         chosen is not None and len(hw["HAL_I2C_H7_400_TIMINGR"]) == 1 and
         int(hw["HAL_I2C_H7_400_TIMINGR"][0], 16) == chosen,
         "found %s, chosen %s" % (hw["HAL_I2C_H7_400_TIMINGR"], "0x%08X" % chosen if chosen else None))
    def one_eq(vals, want):
        return want is not None and len(vals or []) == 1 and int(vals[0], 16) == want

    for tag, k in (("A4a", "HAL_I2C_F7_100_TIMINGR"), ("A4b", "HAL_I2C_H7_100_TIMINGR")):
        why = ("used by the I2CDevice ctor whenever a driver asks for <= 100 kHz" if k.startswith("HAL_I2C_F7")
               else "used only if HAL_I2C_MAX_CLOCK is ever <= 100 kHz (kept consistent)")
        gate("%s hwdef.dat: define %s = Sm TIMINGR (%s)" % (tag, k, why), one_eq(hw[k], chosen_sm),
             "found %s, chosen %s" % (hw[k], "0x%08X" % chosen_sm if chosen_sm else None))
    gen = None
    if ap_root:
        gen = generated_header_check(ap_root)
        ok = gen.get("rc") == 0 and gen.get("HAL_I2C_MAX_CLOCK") == ["400000"] and \
            one_eq(gen.get("HAL_I2C_H7_400_TIMINGR"), chosen) and \
            one_eq(gen.get("HAL_I2C_F7_100_TIMINGR"), chosen_sm) and \
            one_eq(gen.get("HAL_I2C_H7_100_TIMINGR"), chosen_sm)
        gate("A3 generated hwdef.h carries every I2C timing define once (I2CDevice.cpp #ifndef then uses them)",
             ok, "rc %s, %s" % (gen.get("rc"), {k: gen.get(k) for k in HW_KEYS}))
    else:
        print("  [SKIP] A3 generated hwdef.h check (no --ardupilot / ARDUPILOT_ROOT)")

    nfail = sum(1 for g in GATES if not g["pass"])
    report = {
        "revision": "12.0",
        "title": "I2C1 400 kHz timing, electricals and ESC noise immunity (DPS310 U4 + external compass)",
        "result": "PASS" if nfail == 0 else "FAIL",
        "gates": GATES,
        "assumptions": A,
        "clock": clk,
        "firmware_evidence": ev,
        "timingr": {
            "ardupilot_default_400": {"value": "0x00300F38",
                                      "worst_tr300": summary(dflt), "on_this_bus": summary(dflt_typ),
                                      "typical": typical.get("default_0x00300F38"),
                                      "tr_sweep": dflt_sweep, "max_tr_passing_ns": max_ok_tr(dflt_sweep)},
            "ardupilot_100k_path_used_without_MAX_CLOCK": summary(sm),
            "ardupilot_F7_100_used_by_H7_ctor_lowering": summary(f7),
            "chosen_100k": {"value": "0x%08X" % chosen_sm if chosen_sm else None,
                            "hwdef_macros": ["HAL_I2C_F7_100_TIMINGR", "HAL_I2C_H7_100_TIMINGR"],
                            "worst_tr1000": summary(best_sm[2]) if best_sm else None,
                            "tr_sweep": chosen_sm_sweep,
                            "search": "PRESC 0..15, SCLDEL = min for tr 1000 ns + tSU;DAT 250 ns + 50 ns, "
                                      "smallest passing SDADEL, (SCLH, SCLL) minimising worst-case period; "
                                      "UM10204 Sm + timing_margins_sm, tSU;STA/tSU;STO/tHD;STA/tBUF checked"},
            "chosen": {"value": "0x%08X" % chosen if chosen else None,
                       "worst_tr300": summary(best[2]) if best else None,
                       "on_this_bus": chosen_bus, "typical": typical.get("chosen"),
                       "tr_sweep": chosen_sweep, "max_tr_passing_ns": max_ok_tr(chosen_sweep),
                       "search": "PRESC 0..3, SCLDEL = min for tr 300 ns + tSU;DAT 100 ns + 20 ns, "
                                 "smallest passing SDADEL, (SCLH, SCLL) minimising worst-case period; "
                                 "DNF 0, analog filter on",
                       "DNF_note": "ArduPilot passes I2CConfig.cr1 (only SMBHEN ever set) to CR1, so DNF=0 "
                                   "and ANFOFF=0 are fixed; changing them needs an I2CDevice.cpp patch."},
            "model": "tLOW/tHIGH measured per UM10204 at 0.3/0.7 VDD; the H7 detects SCL edges anywhere in "
                     "0.3..0.7 VDD (only VIL/VIH are guaranteed), so tHIGH can lose a full tr and tLOW a full tf. "
                     "tSYNC = tAF(50..260) + DNF + 2..3 tI2CCLK; SCL low extended to SDADEL+SCLDEL if larger.",
        },
        "bus": bus,
        "bus_warnings": {
            "module_pullup_2k2": warn_2k2,
            "abuse_50cm_cable": abuse,
            "fall_time": "MCU open-drain at OSPEED MEDIUM gives tf of a few ns at low Cbus, below the Fm "
                         "tf(min) %.1f ns (an EMI 'should'); measure on hardware." % bus["tf_min_fm_ns"],
        },
        "crosstalk": xt,
        "routing_rules": routing,
        "hwdef": hw,
        "generated_hwdef_h": gen,
        "baseline_confirmation": {
            "U4": "DPS310 LGA-8: 1 GND, 2 CSB, 3 SDI, 4 SCK, 5 SDO, 6 VDDIO, 7 GND, 8 VDD (" +
                  A["dps310"]["pinout_confidence"] + ")",
            "Rp_2k2": "confirmed: window %.0f..%.0f ohm" % (rw["Rp_min_ohm_with_4k7_module"],
                                                         rw["Rp_max_ohm_tr300_at_Cmax"]),
            "Rs_33R": "confirmed (harmless: %.0f mV low-level cost, 0.33 ns pole; not a noise filter)"
                      % sr["Rs_drop_mV"],
            "fSCL": "400 kHz class confirmed only with both hwdef defines; worst-case fSCL %.0f kHz"
                    % (best[2]["fSCL_min_hz"] / 1e3 if best else 0),
        },
        "bus_clear": {
            "boot": "HAL_ChibiOS_Class.cpp calls I2CBus::clear_all() at start-up",
            "timeout": "I2CDevice::transfer: on MSG_TIMEOUT with SDA read low -> clear_bus()",
            "action": "SCL driven PUSH-PULL, %s toggles x %s us (= 10 clocks at ~50 kHz), no STOP generated"
                      % (ev.get("clear_bus_toggles"), ev.get("clear_bus_half_period_us")),
            "note": "Push-pull SCL can fight a clock-stretching external module for <=200 us; DPS310 does "
                    "not stretch. Default HAL_I2C_CLEAR_ON_TIMEOUT 1 kept.",
        },
        "upstream_note": "AP_HAL_ChibiOS I2CDevice ctor: on H7, a device asking <=100 kHz lowers the whole bus "
                         "to HAL_I2C_F7_100_TIMINGR (default 0x30812E3E, an F7/54 MHz value) instead of the H7 one. "
                         "At 32 MHz that default is %.1f..%.1f kHz and Sm-%s. I2C1 is the external bus "
                         "(HAL_I2C_INTERNAL_MASK 0), so external-bus probes/drivers that request 100 kHz "
                         "(%s) can trigger this path; hwdef.dat therefore overrides HAL_I2C_F7_100_TIMINGR (and "
                         "HAL_I2C_H7_100_TIMINGR for consistency) with %s, Sm-compliant for tr <= 1000 ns. "
                         "Once lowered, the bus stays in Standard mode (%.0f..%.0f kHz worst case) for all devices "
                         "including the DPS310."
                         % (f7["fSCL_min_hz"] / 1e3, f7["fSCL_max_hz"] / 1e3,
                            "compliant" if f7["all_pass"] else "NON-compliant (%s)" %
                            ", ".join(k for k, v in f7["checks"].items() if not v),
                            ", ".join(sorted(k for k, v in ev.get("drivers_requesting_100kHz", {}).items() if v))
                            or "INA2xx, INA3221, SMBus, LTC2946 battery monitors, MCP40D1x DAC",
                            "0x%08X" % chosen_sm if chosen_sm else "NONE",
                            best_sm[2]["fSCL_min_hz"] / 1e3 if best_sm else 0,
                            best_sm[2]["fSCL_max_hz"] / 1e3 if best_sm else 0),
    }
    out = os.path.join(HERE, "baro_i2c_report.json")
    with open(out, "w") as fh:
        json.dump(report, fh, indent=1, default=str)
    print("\nwrote %s" % out)
    print("RESULT: %s (%d gates, %d failed)" % (report["result"], len(GATES), nfail))
    return 1 if nfail else 0


if __name__ == "__main__":
    sys.exit(main())
