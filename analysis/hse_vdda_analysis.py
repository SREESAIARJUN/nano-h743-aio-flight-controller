#!/usr/bin/env python3
"""
Nano H743 AIO - HSE crystal and VDDA filter design check (Revision 7.0)

Pure-Python (no numpy) so it runs in CI.  Writes analysis/hse_vdda_report.json
and exits non-zero if any design rule fails.

Sections
  1. Clock tree: 24 MHz HSE -> PLL1/2/3 using ArduPilot's stm32h7_mcuconf.h
     480 MHz divider set (optionally re-read from an ArduPilot tree with
     --ardupilot to prove the numbers below are what the firmware will use).
  2. Crystal: effective load capacitance, frequency pulling, oscillator
     start-up margin (gm_crit vs Gm_critmax, AN2867), drive level.
  3. VDDA / VREF+ filter: +3V3 -> FB1 -> R2 -> (C3..C6), transfer function,
     resonance Q and peaking over component-tolerance corners, with and
     without the R2 damping resistor, plus DC drop.

Assumptions are stated in ASSUMPTIONS and copied into the report; any that
the datasheets in hand contradict must be updated here first.

Usage: python3 analysis/hse_vdda_analysis.py [--ardupilot /path/to/ardupilot]
"""
import argparse
import cmath
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

ASSUMPTIONS = {
    "crystal": {
        "part": "Yajingxin TXM24M0004322FBCDO00T (LCSC C362340), 3225-4P",
        "f_hz": 24e6, "CL_spec_pF": 12.0, "ESR_max_ohm": 30.0,
        "C0_max_pF": 5.0,              # datasheet item 7: shunt capacitance Co max 5 pF
        # datasheet item 8 lists C1 (motional) as N/A -> sweep a 24 MHz 3225 AT-cut range
        "C1_motional_fF_range": [3.0, 8.0],
        "DL_max_uW": 100.0,            # datasheet drive level 100 uW max
        "tol_ppm": 10, "stab_ppm": 20,
        "aging_ppm_per_yr": 5,         # datasheet item 14: +/-5 ppm/year
        "life_years": 10,
    },
    "load_caps": {"C1_pF": 16.0, "C2_pF": 16.0, "tol_pct": 1.0, "dielectric": "C0G",
                  "part": "Murata GJM1555C1H160FB01J (LCSC C3853959)"},
    "stray": {
        # OSC_IN/OSC_OUT pin capacitance ~5 pF each (DS12110 C_IO) in series
        # across the crystal (2.5 pF) + ~1.5 pF of 3 mm 0.1 mm traces over GND.
        "Cs_pF_nominal": 4.0, "Cs_pF_min": 2.5, "Cs_pF_max": 5.5,
    },
    "stm32h7_hse": {"Gm_critmax_mA_per_V": 1.5, "source": "DS12110 HSE oscillator characteristics",
                    "required_margin": 1.2},
    # Frequency-accuracy consumers on this board, tightest first.
    "freq_budget_ppm": {
        "limit": 1000,
        "basis": "FDCAN1 DroneCAN 1 Mbit/s, ISO 11898-1 bit timing with SJW>=2 tq tolerates "
                 ">=3000 ppm combined; 3x margin -> 1000 ppm. USB FS needs 2500 ppm; UARTs ~2%.",
    },
    "bead": {
        "part": "Murata BLM15AX601SN1D (LCSC C76886)", "rated_mA": 500,
        # datasheet: Rdc 0.34 R max initial, 0.39 R max after reliability test;
        # no minimum given -> 0.10 R is used as the low-damping corner.
        "Rdc_ohm_min": 0.10, "Rdc_ohm_max": 0.39,
        # Low-frequency inductance is not specified; sweep wide (0.47..4.7 uH).
        "L_uH_list": [0.47, 1.0, 2.2, 3.3, 4.7],
        "R_parallel_ohm": 600.0,       # lossy region, sets |Z| ~ 600R near 100 MHz
    },
    "vdda_caps": {
        "nominal_uF": [1.0, 0.1, 1.0, 0.1],
        "parts": "2x CL05A105KA5NQNC 1uF X5R 25V (C52923), 2x CL05B104KO5NNNC 100nF X7R 16V (C1525)",
        # 0402 X5R 25 V 1 uF at 3.3 V DC bias keeps ~55-100 % of nominal (Samsung curve)
        "dc_bias_factor_min": 0.55, "esr_ohm_each": 0.01,
        # 0402 body ESL ~0.5 nH + pad/via mounting ~0.5 nH
        "esl_nH_each": 1.0,
    },
    "R2_candidates_ohm": [1.0, 1.5, 2.2],
    "R2_damping_ohm": 2.2,             # design value; must equal the sweep's selection
    "R2_tol_pct": 1.0,
    "R2_part": "UNI-ROYAL 0402WGF220KTCE 2.2R 1% (LCSC C25101)",
    "peaking_limit_dB": 3.0,          # hard limit, checked at every corner
    "peaking_design_target_dB": 2.0,  # R2 selection target: 1 dB guard band for the unspecified bead L
    "dc_drop_limit_mV": 30.0,
    "I_vdda_max_mA": 10.0,  # ADC1/2 + VREF+ at full rate, DAC off, VREFBUF off
}

# ArduPilot 480 MHz set for STM32_HSECLK == 24000000U (stm32h7_mcuconf.h)
PLL = {
    "PLL1": {"M": 3, "N": 120, "P": 2, "Q": 12, "R": 2},
    "PLL2": {"M": 2, "N": 50, "P": 3, "Q": 6, "R": 3},
    "PLL3": {"M": 6, "N": 72, "Q": 6, "R": 9},
}

RESULTS = {"checks": []}


def check(name, ok, detail):
    RESULTS["checks"].append({"check": name, "pass": bool(ok), "detail": detail})
    print("[%s] %s: %s" % ("PASS" if ok else "FAIL", name, detail))


# ---------------------------------------------------------------------------
# 1. Clock tree
# ---------------------------------------------------------------------------
def read_ap_pll(ap_root):
    path = os.path.join(ap_root, "libraries/AP_HAL_ChibiOS/hwdef/common/stm32h7_mcuconf.h")
    src = open(path).read()
    blk = src.split("#elif STM32_HSECLK == 24000000U", 2)
    # second occurrence is the DIVN/DIVP/... block, first is the DIVM block
    if len(blk) < 3:
        raise SystemExit("24 MHz block not found in %s" % path)
    divm = blk[1].split("#elif", 1)[0]
    divn = blk[2].split("#elif STM32_HSECLK == 25000000U", 1)[0]
    m480 = divn.split("HAL_CUSTOM_MCU_CLOCKRATE == 480000000", 1)[1].split("#elif", 1)[0]
    common = divn.split("#endif", 1)[1] if "#endif" in divn else divn

    def g(txt, key):
        m = re.search(r"#define\s+STM32_%s_VALUE\s+(\d+)" % key, txt)
        return int(m.group(1)) if m else None
    got = {
        "PLL1": {"M": g(divm, "PLL1_DIVM"), "N": g(m480, "PLL1_DIVN"), "Q": g(m480, "PLL1_DIVQ"),
                 "P": g(common, "PLL1_DIVP")},
        "PLL2": {"M": g(divm, "PLL2_DIVM"), "N": g(common, "PLL2_DIVN"), "P": g(common, "PLL2_DIVP"),
                 "Q": g(common, "PLL2_DIVQ"), "R": g(common, "PLL2_DIVR")},
        "PLL3": {"M": g(divm, "PLL3_DIVM"), "N": g(common, "PLL3_DIVN"), "Q": g(common, "PLL3_DIVQ"),
                 "R": g(common, "PLL3_DIVR")},
    }
    return got, path


def clock_tree(fhse, ap_root=None):
    if ap_root:
        got, path = read_ap_pll(ap_root)
        mism = []
        for p, d in got.items():
            for k, v in d.items():
                if v is not None and PLL[p].get(k) != v:
                    mism.append("%s.%s ours=%s ap=%s" % (p, k, PLL[p].get(k), v))
        check("PLL dividers match ArduPilot stm32h7_mcuconf.h (24 MHz, 480 MHz)", not mism,
              "; ".join(mism) if mism else "M/N/P/Q/R identical (%s)" % os.path.relpath(path, ap_root))
    out = {}
    for name, d in PLL.items():
        ref = fhse / d["M"]
        vco = ref * d["N"]
        e = {"ref_MHz": ref / 1e6, "vco_MHz": vco / 1e6}
        for k in ("P", "Q", "R"):
            if k in d:
                e[k + "_MHz"] = vco / d[k] / 1e6
        out[name] = e
        check("%s reference input 2..16 MHz" % name, 2e6 <= ref <= 16e6, "%.3f MHz" % (ref / 1e6))
        check("%s VCO within wide range 192..960 MHz (rev V)" % name, 192e6 <= vco <= 960e6,
              "%.1f MHz" % (vco / 1e6))
    sysclk = out["PLL1"]["P_MHz"]
    check("SYSCLK = 480 MHz", abs(sysclk - 480) < 1e-6, "%.3f MHz" % sysclk)
    check("USB 48 MHz exact (PLL3_Q)", abs(out["PLL3"]["Q_MHz"] - 48) < 1e-9,
          "%.6f MHz" % out["PLL3"]["Q_MHz"])
    out["silicon_note"] = ("PLL1 VCO 960 MHz / 480 MHz SYSCLK is only specified for revision V "
                           "(DBGMCU REV_ID 0x2003); rev Y is limited to VCO 836 MHz / 400 MHz. "
                           "Enforced as a procurement field on U1 (tools/verify_mcu_core_sch.py S7).")
    # 25 MHz cannot reach 480 MHz with integer dividers inside the limits ArduPilot supports
    out["note_25MHz"] = ("stm32h7_mcuconf.h: '#elif STM32_HSECLK == 25000000U / #ifdef "
                         "HAL_CUSTOM_MCU_CLOCKRATE / #error Unable to configure custom clockrate'")
    return out


# ---------------------------------------------------------------------------
# 2. Crystal
# ---------------------------------------------------------------------------
def gm_crit(esr, f, c0, cl):
    return 4 * esr * (2 * math.pi * f) ** 2 * (c0 + cl) ** 2


def crystal():
    a = ASSUMPTIONS
    x, lc, st = a["crystal"], a["load_caps"], a["stray"]
    f = x["f_hz"]
    pF = 1e-12
    c0 = x["C0_max_pF"] * pF

    def cl_eff(c1, c2, cs):
        return c1 * c2 / (c1 + c2) + cs

    def pull_ppm(cl_actual, c1m_fF):
        cls = x["CL_spec_pF"] * pF
        return 1e6 * (c1m_fF * 1e-15 / 2) * (1 / (c0 + cl_actual * pF) - 1 / (c0 + cls))

    nom = cl_eff(lc["C1_pF"], lc["C2_pF"], st["Cs_pF_nominal"])
    t = lc["tol_pct"] / 100
    lo = cl_eff(lc["C1_pF"] * (1 - t), lc["C2_pF"] * (1 - t), st["Cs_pF_min"])
    hi = cl_eff(lc["C1_pF"] * (1 + t), lc["C2_pF"] * (1 + t), st["Cs_pF_max"])
    c1m_hi = x["C1_motional_fF_range"][1]
    pulls = [pull_ppm(v, c1m_hi) for v in (lo, nom, hi)]
    worst_pull = max(abs(p) for p in pulls)
    ideal_c = 2 * (x["CL_spec_pF"] - st["Cs_pF_nominal"])
    total_ppm = worst_pull + x["tol_ppm"] + x["stab_ppm"] + x["life_years"] * x["aging_ppm_per_yr"]
    check("Effective CL within +/-1 pF of crystal spec (nominal stray)", abs(nom - x["CL_spec_pF"]) <= 1.0,
          "CL_eff = %.2f pF (corners %.2f..%.2f pF), ideal C1=C2=%.1f pF" % (nom, lo, hi, ideal_c))
    check("Load caps centred: nominal pulling within +/-2 ppm", abs(pulls[1]) <= 2,
          "nominal pull %+.2f ppm" % pulls[1])
    check("Load-cap pulling within +/-30 ppm over stray, cap tolerance and C1m %g..%g fF"
          % tuple(x["C1_motional_fF_range"]), worst_pull <= 30,
          "pull = %+.1f / %+.1f / %+.1f ppm (lo/nom/hi) at C1m %g fF, C0 %g pF"
          % (pulls[0], pulls[1], pulls[2], c1m_hi, x["C0_max_pF"]))
    fb = a["freq_budget_ppm"]
    check("%d-year frequency error budget <= %d ppm (%s)" % (x["life_years"], fb["limit"], "FDCAN/USB/UART"),
          total_ppm <= fb["limit"],
          "%.1f ppm = pull %.1f + tol %d + stab %d + aging %dx%d"
          % (total_ppm, worst_pull, x["tol_ppm"], x["stab_ppm"], x["life_years"], x["aging_ppm_per_yr"]))

    hse = a["stm32h7_hse"]
    gm_lim = hse["Gm_critmax_mA_per_V"] * 1e-3
    gm_req = gm_lim / hse["required_margin"]
    gm = gm_crit(x["ESR_max_ohm"], f, c0, hi * pF)
    check("Oscillator start-up: gm_crit <= Gm_critmax/%.1f (AN2867)" % hse["required_margin"], gm <= gm_req,
          "gm_crit = %.3f mA/V (worst CL %.2f pF, ESR %g R, C0 %g pF) vs Gm_critmax %.1f mA/V -> margin x%.2f"
          % (gm * 1e3, hi, x["ESR_max_ohm"], x["C0_max_pF"], gm_lim * 1e3, gm_lim / gm))

    # Substitution rule: max ESR a replacement crystal may have, per CL, at the
    # worst stray corner and C0 5 pF, keeping the required margin.
    sub = {}
    for cl_alt in (8, 10, 12, 18, 20):
        cl_w = cl_alt + (hi - x["CL_spec_pF"])
        sub["CL%dpF" % cl_alt] = round(gm_req / gm_crit(1, f, c0, cl_w * pF), 1)
    check("Substitution rule computed: ESR_max(CL) at C0 5 pF, margin x%.1f" % hse["required_margin"],
          sub["CL12pF"] >= x["ESR_max_ohm"],
          "max ESR [R] per CL: %s; a CL 18-20 pF part must have ESR <= %.0f R"
          % (json.dumps(sub), sub["CL20pF"]))

    # Rext (R1): treated as added series resistance (conservative)
    esr_max_total = gm_req / gm_crit(1, f, c0, hi * pF)
    rext_max = esr_max_total - x["ESR_max_ohm"]
    r1_max_e12 = max([0] + [r for r in (1.0, 2.2, 3.3, 4.7, 6.8, 10, 15, 22, 33, 47) if r <= rext_max])

    # Drive level vs OSC_OUT amplitude: DL = ESR * (pi f Vpp Ctot)^2 / 2, Ctot = C2 + Cs/2
    ctot = (lc["C2_pF"] + st["Cs_pF_nominal"] / 2) * pF

    def dl(vpp):
        return x["ESR_max_ohm"] * (math.pi * f * vpp * ctot) ** 2 / 2

    vpp_lim = math.sqrt(2 * x["DL_max_uW"] * 1e-6 / x["ESR_max_ohm"]) / (math.pi * f * ctot)
    check("Drive level at 1.0 Vpp on OSC_OUT below crystal limit", dl(1.0) <= x["DL_max_uW"] * 1e-6,
          "DL(1.0 Vpp) = %.1f uW, DL(3.3 Vpp) = %.1f uW, limit %.0f uW -> bring-up gate Vpp(OSC_OUT) <= %.2f V; "
          "R1 may be raised to at most %g R (%.1f R keeps x%.1f start-up margin)"
          % (dl(1.0) * 1e6, dl(3.3) * 1e6, x["DL_max_uW"], vpp_lim, r1_max_e12, rext_max, hse["required_margin"]))
    return {
        "CL_eff_pF": {"min": round(lo, 3), "nom": round(nom, 3), "max": round(hi, 3)},
        "ideal_load_cap_pF": ideal_c,
        "pull_ppm_at_C1m_max": [round(p, 2) for p in pulls],
        "total_error_ppm": round(total_ppm, 1),
        "gm_crit_mA_per_V": round(gm * 1e3, 4),
        "gm_margin": round(gm_lim / gm, 2),
        "substitution_ESR_max_ohm": sub,
        "drive_level_uW": {"1.0Vpp": round(dl(1.0) * 1e6, 1), "2.0Vpp": round(dl(2.0) * 1e6, 1),
                           "3.3Vpp": round(dl(3.3) * 1e6, 1)},
        "Vpp_limit_for_DLmax_V": round(vpp_lim, 3),
        "R1_ext_max_ohm": round(rext_max, 1),
        "R1_ext_max_E12_ohm": r1_max_e12,
    }


# ---------------------------------------------------------------------------
# 3. VDDA filter
# ---------------------------------------------------------------------------
def h_filter(fr, L, Rdc, Rp, Rd, caps, esr, esl):
    w = 2 * math.pi * fr
    zl = 1j * w * L
    zbead = Rdc + (zl * Rp) / (zl + Rp)
    ysh = sum(1 / (esr + 1j * w * esl + 1 / (1j * w * c)) for c in caps)
    zsh = 1 / ysh
    return zsh / (zbead + Rd + zsh)


def logspace(a, b, n):
    la, lb = math.log10(a), math.log10(b)
    return [10 ** (la + (lb - la) * i / (n - 1)) for i in range(n)]


def vdda():
    a = ASSUMPTIONS
    b, c = a["bead"], a["vdda_caps"]
    esl = c["esl_nH_each"] * 1e-9
    freqs = logspace(1e3, 500e6, 1200)
    corners = [(L * 1e-6, k, rdc) for L in b["L_uH_list"]
               for k in (c["dc_bias_factor_min"], 1.0)
               for rdc in (b["Rdc_ohm_min"], b["Rdc_ohm_max"])]

    def sweep(rd):
        rows = []
        for L, k, rdc in corners:
            caps = [v * 1e-6 * k for v in c["nominal_uF"]]

            def H(fx):
                return h_filter(fx, L, rdc, b["R_parallel_ohm"], rd, caps, c["esr_ohm_each"], esl)
            mags = [abs(H(fr)) for fr in freqs]
            i = max(range(len(mags)), key=lambda j: mags[j])
            ctot = sum(caps)
            rows.append({
                "L_uH": round(L * 1e6, 2), "C_total_uF": round(ctot * 1e6, 3), "Rdc_ohm": rdc,
                "f0_kHz": round(1 / (2 * math.pi * math.sqrt(L * ctot)) / 1e3, 1),
                "Q": round(math.sqrt(L / ctot) / (rdc + rd + c["esr_ohm_each"] / 4), 2),
                "peak_dB": round(20 * math.log10(mags[i]), 2), "peak_at_kHz": round(freqs[i] / 1e3, 1),
                "att_dB_1MHz": round(20 * math.log10(abs(H(1e6))), 1),
                "att_dB_10MHz": round(20 * math.log10(abs(H(10e6))), 1),
                "att_dB_100MHz": round(20 * math.log10(abs(H(100e6))), 1),
            })
        return rows

    def drop_mV(r2):
        return a["I_vdda_max_mA"] * (b["Rdc_ohm_max"] + r2 * (1 + a["R2_tol_pct"] / 100))

    cand = {}
    for r2 in a["R2_candidates_ohm"]:
        rows = sweep(r2 * (1 - a["R2_tol_pct"] / 100))
        cand[str(r2)] = {"worst_peak_dB": max(r["peak_dB"] for r in rows), "dc_drop_mV": round(drop_mV(r2), 2)}
    ok = [float(k) for k, v in cand.items()
          if v["worst_peak_dB"] <= a["peaking_design_target_dB"] and v["dc_drop_mV"] <= a["dc_drop_limit_mV"]]
    pick = min(ok) if ok else None
    check("R2 = smallest candidate meeting peaking <= %.0f dB design target and DC drop <= %.0f mV"
          % (a["peaking_design_target_dB"], a["dc_drop_limit_mV"]), pick == a["R2_damping_ohm"],
          "selected %s R (design %s R); candidates %s" % (pick, a["R2_damping_ohm"], json.dumps(cand)))

    rd = a["R2_damping_ohm"] * (1 - a["R2_tol_pct"] / 100)
    undamped = sweep(0.0)
    damped = sweep(rd)
    worst_d = max(r["peak_dB"] for r in damped)
    worst_u = max(r["peak_dB"] for r in undamped)
    min_att10 = max(r["att_dB_10MHz"] for r in damped)
    min_att100 = max(r["att_dB_100MHz"] for r in damped)
    check("VDDA filter resonance peaking <= +%.0f dB at every corner (L 0.47..4.7 uH, Rdc 0.1..0.39 R, "
          "C -45%%, R2 -1%%)" % a["peaking_limit_dB"], worst_d <= a["peaking_limit_dB"],
          "worst %.2f dB with R2=%g R vs %.2f dB undamped" % (worst_d, a["R2_damping_ohm"], worst_u))
    check("VDDA filter attenuation >= 40 dB at 10 MHz incl. 1 nH cap ESL (all corners)", min_att10 <= -40,
          "worst %.1f dB" % min_att10)
    check("VDDA filter attenuation >= 40 dB at 100 MHz incl. 1 nH cap ESL (all corners)", min_att100 <= -40,
          "worst %.1f dB" % min_att100)
    drop = drop_mV(a["R2_damping_ohm"])
    check("VDDA DC drop <= %.0f mV at %g mA (bead Rdc 0.39 R after-test max + R2 +1%%)"
          % (a["dc_drop_limit_mV"], a["I_vdda_max_mA"]), drop <= a["dc_drop_limit_mV"], "%.1f mV" % drop)
    return {"R2_selection": cand, "damped_R2": damped, "undamped": undamped,
            "worst_peak_dB_damped": worst_d, "worst_peak_dB_undamped": worst_u, "dc_drop_mV": round(drop, 2),
            "note": "VREF+ shares the +3.3VA node, so VREF+ = VDDA by construction (VREFBUF must stay off)."}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ardupilot", default=os.environ.get("ARDUPILOT_ROOT"))
    ap.add_argument("-o", "--output", default=os.path.join(HERE, "hse_vdda_report.json"))
    args = ap.parse_args()
    RESULTS["revision"] = "7.0"
    RESULTS["assumptions"] = ASSUMPTIONS
    RESULTS["clock_tree"] = clock_tree(ASSUMPTIONS["crystal"]["f_hz"], args.ardupilot)
    RESULTS["crystal"] = crystal()
    RESULTS["vdda_filter"] = vdda()
    fails = [c for c in RESULTS["checks"] if not c["pass"]]
    RESULTS["summary"] = {"checks": len(RESULTS["checks"]), "failures": len(fails)}
    with open(args.output, "w") as fh:
        json.dump(RESULTS, fh, indent=2)
    print("%d checks, %d failures -> %s" % (len(RESULTS["checks"]), len(fails), os.path.relpath(args.output)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
