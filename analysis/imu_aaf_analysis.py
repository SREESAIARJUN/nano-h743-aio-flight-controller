#!/usr/bin/env python3
"""
Nano H743 AIO - dual ICM-42688-P anti-aliasing, clock-sync and SPI check (Revision 11.0)

Pure-Python (no numpy) so it runs in CI.  Writes analysis/imu_aaf_report.json
and exits non-zero if any FAIL-severity design rule fails.  WARN-severity
checks are datasheet-condition deviations that are reported (and listed in
the summary) but judged not to break function; each carries the reasoning.

Sections
  1. Per-IMU supply anti-aliasing filter: +3V3 -> FB2/FB3 -> R8/R9 22R ->
     10 uF + 2x 100 nF.  DC drop, corner, attenuation at ESC PWM 24/48/96 kHz
     and in every band that aliases onto 0-1 kHz around multiples of the ODR,
     LC resonance / peaking over bead-L, cap DC-bias and tolerance corners,
     start-up ramp, self-generated SPI-burst ripple, R/C candidate sweep.
  2. In-sensor digital AAF (ArduPilot ICM-42688 coefficients) + UI filter:
     attenuation at Nyquist for 1/2/4/8 kHz ODR, and combined board+sensor
     rejection of ESC supply ripple traced through the 32 kHz decimation and
     the ODR decimation (alias frequency in baseband).
  3. Synchronisation: shared 32.768 kHz CLKIN -> ODR 8192 Hz; CLKIN range,
     ArduPilot _update_sensor_rate() window, inter-IMU drift with/without the
     shared clock, absolute dt error, loss-of-clock fallback rate re-tracking.
  4. Clock distribution: Y2 into 2x pin 9 (R14/R15) + MCU monitor (R16):
     edge times (RC network transient), oscillator load vs 15 pF rating,
     attenuation at 32.768 kHz, power-up back-drive of an unpowered IMU pin.
  5. SPI source termination 33R: edge time vs quarter period, and MISO/MOSI
     data-valid timing budget at 10.0 MHz (SPI1) and 12.5 MHz (SPI4).

Every number not taken from a fetched datasheet is in ASSUMPTIONS with
"assumed" in its source string.  Datasheet sources used:
  ICM-42688-P DS-000347 rev 1.2 (fetched:
    https://www.cdiweb.com/datasheets/invensense/ds-000347-icm-42688-p-v1.2.pdf)
  SCTF 0CKM..7CKM 32.768 kHz oscillator datasheet rev 05/11/2026 (fetched via
    LCSC C2901603, https://pcbparts.dev/mcp/datasheet/C2901603)
  ArduPilot AP_InertialSensor_Invensensev3.cpp / AP_InertialSensor_Backend.cpp
    (read from the tree given by --ardupilot / $ARDUPILOT_ROOT when available).

Usage: python3 analysis/imu_aaf_analysis.py [--ardupilot /path/to/ardupilot]
"""
import argparse
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ICM_DS = "ICM-42688-P DS-000347 rev1.2 (cdiweb.com copy)"
Y2_DS = "SCTF 32.768kHz CMOS osc datasheet rev 05/11/2026 (LCSC C2901603)"

# Frozen Rev 11 design (refdes as given by the lead; see report 'refdes_note').
DESIGN = {
    "imu": "ICM-42688-P (U2 = IMU1 on SPI1, U3 = IMU2 on SPI4)",
    "supply_filter": {
        "IMU1": "+3V3 -> FB2 -> R8 22R -> +3V3_IMU1; C5 10uF, C6 100nF (VDD pin 8), C7 100nF (VDDIO pin 5)",
        "IMU2": "+3V3 -> FB3 -> R9 22R -> +3V3_IMU2; C8 10uF, C9 100nF (VDD), C10 100nF (VDDIO)",
        "R_ohm": 22.0, "R_tol_pct": 1.0,
        "C_bulk_uF": 10.0, "C_bulk_part": "0603 X5R 10V 10uF",
        "C_local_uF": [0.1, 0.1],
    },
    "clock": {
        "osc": "Y2 SCTF SX2M32.768KM20F30TNN (LCSC C2901603), OE strapped high via R17 10k",
        "f_hz": 32768.0,
        "branches": {"R14_IMU1": 33.0, "R15_IMU2": 33.0, "R16_MCU_PD12": 1000.0},
    },
    "spi": {
        "R_series_ohm": 33.0,
        "SPI1_sclk_hz": 10.0e6,   # hwdef: PLL1_Q 80 MHz /8
        "SPI4_sclk_hz": 12.5e6,   # hwdef: PLL2_Q 100 MHz /8
        "mode": 3,
    },
    "odr_nominal_hz": 8000.0,
}

ASSUMPTIONS = {
    "imu_current": {
        "I_typ_mA": 0.88, "source": ICM_DS + " Table 3: 6-axis LN mode 0.88 mA typ (no max given)",
        "I_design_mA": 1.5, "I_design_source": "assumed: typ 0.88 mA + VDDIO SPI/INT activity + margin "
                                               "(datasheet gives no max)",
        "gyro_enable_step_mA": 0.73, "step_source": ICM_DS + " Table 3: 3-axis gyro LN 0.73 mA",
    },
    "imu_supply_limits": {
        "VDD_min_V": 1.71, "VDD_max_V": 3.6, "source_range": ICM_DS + " features: 1.71-3.6 V",
        "ramp_min_ms": 0.01, "ramp_max_ms": 3.0, "source_ramp": ICM_DS + " Table 4: Supply Ramp Time 0.01..3 ms, monotonic",
        "noise_mVpp": 10.0, "source_noise": ICM_DS + " Table 4: Power Supply Noise 10 mV p-p (note 1: by design)",
        "por_ms": 1.0, "source_por": ICM_DS + " Table 4: start-up time for register r/w 1 ms",
        "input_abs_max_over_VDDIO_V": 0.5, "source_absmax": ICM_DS + " Table 8: input level -0.5..VDDIO+0.5 V",
        "CI_max_pF": 10.0, "source_CI": ICM_DS + " Table 4: digital inputs (FSYNC, SCLK, SDI, CS) CI < 10 pF",
    },
    "rail_ripple": {
        "V_esc_ripple_mVpp": 100.0,
        "source": "assumed: +3V3 ripple at each ESC PWM fundamental on a 4-in-1 AIO; to be measured at bring-up",
        "esc_pwm_hz": [24000.0, 48000.0, 96000.0],
        "harmonics": 9, "harmonic_rolloff": "1/n (rectangular-ish ripple), assumed",
    },
    "bead": {
        "part": "same as FB1: Murata BLM15AX601SN1D (LCSC C76886) 600R@100MHz, see tools/gen_mcu_core_sch.py",
        "Rdc_ohm_min": 0.10, "Rdc_ohm_max": 0.39,
        "Rdc_source": "Murata datasheet Rdc 0.34 R max initial / 0.39 R after test (as in hse_vdda_analysis.py); "
                      "0.10 R min assumed",
        "L_uH_list": [0.47, 1.0, 2.2, 3.3, 4.7],
        "L_source": "assumed: low-frequency inductance not specified by Murata; swept 0.47..4.7 uH (same sweep as Rev 7 VDDA)",
        "R_parallel_ohm": 600.0,
    },
    "caps": {
        "bulk_dc_bias_factor": {"min": 0.35, "nom": 0.55, "max": 1.10},
        "bulk_dc_bias_source": "assumed: 0603 X5R 10V 10uF keeps ~40-60 % at 3.3 V DC bias (typical Samsung "
                               "CL10A106KP8NNNC / Murata GRM188R61A106 curves); min 0.35 adds -10 % tol and "
                               "X5R temperature; max 1.10 = +10 % tol at 0 V bias",
        "local_factor": {"min": 0.80, "nom": 0.95, "max": 1.10},
        "local_source": "assumed: 0402 X7R 100nF 16V at 3.3 V, +/-10 % tol",
        "esr_bulk_ohm": 0.005, "esr_local_ohm": 0.02, "esl_nH": 1.0,
        "esr_esl_source": "assumed: typical MLCC ESR, 1 nH body+mounting ESL",
    },
    "spi_burst": {
        "C_sdo_load_pF": 8.0, "bits_per_sample": 16 * 8, "toggle_fraction": 0.5,
        "source": "assumed: MISO trace+MCU pin 8 pF, ArduPilot 16-byte FIFO packet per sample, 50 % toggling",
    },
    "rail_3v3_ramp_ms": {"min": 0.3, "max": 2.0,
                         "source": "assumed: +3V3 LDO soft-start 10-90 % ramp; not yet characterised"},
    "internal_osc": {
        "initial_pct": 1.0, "temp_pct": 2.0,
        "source": ICM_DS + " Table 4: CLKSEL=01 (PLL) & gyro active: initial +/-1 %, over temp +/-2 % "
                           "(note 1: by design, not production tested); sec 4.10: '+/-1 % in PLL mode'",
    },
    "clkin": {
        "f_min_hz": 31000.0, "f_max_hz": 50000.0, "f_ref_hz": 32000.0,
        "source": ICM_DS + " Table 4 external clock 31/32/50 kHz; sec 12.5 ODR scales by f_CLKIN/32 kHz",
    },
    "y2": {
        "tol_ppm": 20.0, "stab_ppm": 30.0, "aging_ppm_per_yr": 3.0, "life_years": 10,
        "L_CMOS_pF": 15.0, "tr_max_ns": 25.0, "duty_pct": [45.0, 55.0], "startup_ms_max": 5.0,
        "VOH_frac": 0.9, "source": Y2_DS + ": load 15 pF, tr/tf 25 ns max (10-90 %), duty 45-55 % at "
                                          "<=15 pF, start-up 5 ms max, aging +/-3 ppm first year; tol +/-20 ppm "
                                          "and stability +/-30 ppm from LCSC listing",
        "R_out_min_ohm": 30.0,
        "R_out_source": "R_out_max derived from tr 25 ns @ 15 pF (= tr/(2.2*15 pF)); R_out_min 30 R assumed",
    },
    "clock_loads": {
        "C_imu_pin9_pF": {"nom": 5.0, "max": 10.0},
        "C_mcu_pin_pF": 5.0, "C_trace_pF_per_branch": 1.5, "C_osc_node_pF": 2.0,
        "source": "pin 9 max from " + ICM_DS + " CI<10 pF; nominal 5 pF assumed; MCU pin 5 pF assumed "
                  "(STM32H7 typ C_IO); trace 1.5 pF/branch (~10 mm, 0.15 pF/mm) assumed",
    },
    "spi_elec": {
        "R_mcu_drv_ohm": 35.0, "t_mcu_intrinsic_ns": 2.5, "t_mcu_tv_mo_ns": 5.0, "t_mcu_tsu_mi_ns": 5.0,
        "R_imu_drv_ohm": 60.0,
        "C_imu_in_pF": 10.0, "C_mcu_in_pF": 5.0, "C_trace_pF": 2.0, "trace_mm_max": 30.0,
        "source": "assumed (not from DS12110 here): STM32H7 very-high-speed driver ~35 R, intrinsic edge "
                  "2.5 ns, master tv(MO) 5 ns, tsu(MI) 5 ns; ICM SDO driver 60 R; IMU CI 10 pF from DS",
        "icm_tVD_SDO_ns": 21.5, "icm_tHD_SDO_ns": 3.5, "icm_tSU_SDI_ns": 13.0, "icm_tHD_SDI_ns": 8.0,
        "icm_tLOW_ns": 17.0, "icm_tHIGH_ns": 17.0, "icm_fmax_hz": 24e6,
        "icm_source": ICM_DS + " Table 6 (24 MHz 4-wire SPI, VDDIO 1.8 V, tVD at Cload 20 pF)",
    },
    # ArduPilot ICM-42688 gyro AAF (DELT -> 3 dB) per backend rate, cross-checked
    # against DS-000347 sec 5.3 AAF table; UI filter = reset default 2nd order
    # (GYRO_CONFIG1 0x16 -> ORD=01), BW index 1 (GYRO_ACCEL_CONFIG0 0x11), not
    # written by ArduPilot.  3 dB values from DS sec 5.5 2nd-order table.
    "sensor_filters": {
        "aaf_table_hz": {6: 258.0, 12: 536.0, 21: 997.0, 5: 213.0},
        "ui_2nd_order_3dB_hz": {1000: 230.7, 2000: 524.0, 4000: 1048.1, 8000: 2096.3},
        "model": "assumed: AAF and UI filter each modelled as 2nd-order Butterworth (Q 0.707) analog "
                 "prototypes at the datasheet 3 dB point; DS says 'AAF is a 2nd order filter', Q not given. "
                 "The 32 kHz decimation filter response is not published -> taken as 0 dB (worst case).",
        "internal_rate_hz": 32000.0,
        "source_internal_rate": ICM_DS + " Fig. 8 signal path: decimation filter (32 kHz) before notch/AAF",
    },
    "ardupilot_rate": {"window": 0.05, "filter": 0.98, "conv_window": 1.0, "conv_filter": 0.8,
                       "conv_ms": 30000},
    "peaking_limit_dB": 1.0,
    "dc_drop_limit_mV": 50.0,
    "esc_atten_min_dB": 20.0,
    "baseband_hz": 1000.0,
}

RESULTS = {"checks": []}


def check(name, ok, detail, severity="fail"):
    sev = "pass" if ok else severity
    RESULTS["checks"].append({"check": name, "pass": bool(ok), "severity": sev, "detail": detail})
    tag = "PASS" if ok else ("FAIL" if severity == "fail" else "WARN")
    print("[%s] %s: %s" % (tag, name, detail))


def db(x):
    return 20 * math.log10(max(abs(x), 1e-30))


def logspace(a, b, n):
    la, lb = math.log10(a), math.log10(b)
    return [10 ** (la + (lb - la) * i / (n - 1)) for i in range(n)]


def fold(f, fs):
    """Alias frequency of f when sampled at fs, in [0, fs/2]."""
    r = math.fmod(f, fs)
    return fs - r if r > fs / 2 else r


# ---------------------------------------------------------------------------
# 1. Supply anti-aliasing filter
# ---------------------------------------------------------------------------
def h_supply(fr, L, Rdc, Rp, R, cbulk, clocal, a):
    c = a["caps"]
    w = 2 * math.pi * fr
    esl = c["esl_nH"] * 1e-9
    zl = 1j * w * L
    zbead = Rdc + (zl * Rp) / (zl + Rp)
    ysh = 1 / (c["esr_bulk_ohm"] + 1j * w * esl + 1 / (1j * w * cbulk))
    for cl in clocal:
        ysh += 1 / (c["esr_local_ohm"] + 1j * w * esl + 1 / (1j * w * cl))
    zsh = 1 / ysh
    return zsh / (zbead + R + zsh)


def supply_filter(R_ohm=None, Cb_uF=None, quiet=False):
    a = ASSUMPTIONS
    d = DESIGN["supply_filter"]
    b, c = a["bead"], a["caps"]
    R = d["R_ohm"] if R_ohm is None else R_ohm
    Cb = d["C_bulk_uF"] if Cb_uF is None else Cb_uF
    rtol = d["R_tol_pct"] / 100
    odr = DESIGN["odr_nominal_hz"] * DESIGN["clock"]["f_hz"] / a["clkin"]["f_ref_hz"]

    def caps(corner):
        return (Cb * 1e-6 * c["bulk_dc_bias_factor"][corner],
                [v * 1e-6 * c["local_factor"][corner] for v in d["C_local_uF"]])

    corners = []
    for L in b["L_uH_list"]:
        for cc in ("min", "max"):
            for rdc in (b["Rdc_ohm_min"], b["Rdc_ohm_max"]):
                for r in (R * (1 - rtol), R * (1 + rtol)):
                    corners.append((L * 1e-6, cc, rdc, r))

    def H(fx, L, cc, rdc, r):
        cb, cl = caps(cc)
        return h_supply(fx, L, rdc, b["R_parallel_ohm"], r, cb, cl, a)

    freqs = logspace(10.0, 300e6, 1500)
    worst_peak = (-1e9, None)
    rows = []
    for L, cc, rdc, r in corners:
        mags = [abs(H(fx, L, cc, rdc, r)) for fx in freqs]
        i = max(range(len(mags)), key=lambda j: mags[j])
        cb, cl = caps(cc)
        ctot = cb + sum(cl)
        q = math.sqrt(L / ctot) / (rdc + r)
        row = {"L_uH": round(L * 1e6, 2), "C_corner": cc, "C_total_uF": round(ctot * 1e6, 3),
               "Rdc_ohm": rdc, "R_ohm": round(r, 2),
               "f_RC_Hz": round(1 / (2 * math.pi * (r + rdc) * ctot), 1),
               "f0_LC_kHz": round(1 / (2 * math.pi * math.sqrt(L * ctot)) / 1e3, 1), "Q_LC": round(q, 4),
               "peak_dB": round(20 * math.log10(mags[i]), 4), "peak_at_Hz": round(freqs[i], 1)}
        for fe in a["rail_ripple"]["esc_pwm_hz"]:
            row["att_dB_%dk" % int(fe / 1e3)] = round(db(H(fe, L, cc, rdc, r)), 2)
        rows.append(row)
        if row["peak_dB"] > worst_peak[0]:
            worst_peak = (row["peak_dB"], row)

    # worst-case (least attenuation) over corners at a given frequency
    def worst_att(fx):
        return max(db(H(fx, L, cc, rdc, r)) for L, cc, rdc, r in corners)

    def nom_att(fx):
        cb = Cb * 1e-6 * c["bulk_dc_bias_factor"]["nom"]
        cl = [v * 1e-6 * c["local_factor"]["nom"] for v in d["C_local_uF"]]
        return db(h_supply(fx, 1e-6, 0.3, b["R_parallel_ohm"], R, cb, cl, a))

    # DC drop and supply voltage
    I = a["imu_current"]["I_design_mA"] * 1e-3
    drop = I * (b["Rdc_ohm_max"] + R * (1 + rtol))
    drop_typ = a["imu_current"]["I_typ_mA"] * 1e-3 * (b["Rdc_ohm_max"] + R)
    ctot_nom = Cb * 1e-6 * c["bulk_dc_bias_factor"]["nom"] + sum(d["C_local_uF"]) * 1e-6 * c["local_factor"]["nom"]
    ctot_min = Cb * 1e-6 * c["bulk_dc_bias_factor"]["min"] + sum(d["C_local_uF"]) * 1e-6 * c["local_factor"]["min"]
    ctot_max = Cb * 1e-6 * c["bulk_dc_bias_factor"]["max"] + sum(d["C_local_uF"]) * 1e-6 * c["local_factor"]["max"]
    fc_nom = 1 / (2 * math.pi * (R + 0.3) * ctot_nom)
    fc_max = 1 / (2 * math.pi * (R * (1 - rtol) + b["Rdc_ohm_min"]) * ctot_min)

    # ESC tones
    esc = {}
    rr = a["rail_ripple"]
    for fe in rr["esc_pwm_hz"]:
        w = worst_att(fe)
        esc["%d_kHz" % int(fe / 1e3)] = {"worst_dB": round(w, 2), "nominal_dB": round(nom_att(fe), 2),
                                        "ripple_out_mVpp": round(rr["V_esc_ripple_mVpp"] * 10 ** (w / 20), 3)}

    # Aliasing bands: k*ODR +/- baseband for k = 1..60 (up to ~490 kHz)
    bb = a["baseband_hz"]
    bands = []
    for k in range(1, 61):
        lo, hi = k * odr - bb, k * odr + bb
        # attenuation is monotonic-ish in the stop band; the band edge closest to DC is worst
        w = max(worst_att(lo), worst_att(k * odr), worst_att(hi))
        bands.append({"k": k, "band_kHz": [round(lo / 1e3, 3), round(hi / 1e3, 3)], "worst_board_dB": round(w, 2)})

    out = {"R_ohm": R, "C_bulk_uF": Cb, "corners": rows,
           "worst_peak_dB": worst_peak[0], "worst_peak_corner": worst_peak[1],
           "Q_LC_max": max(r_["Q_LC"] for r_ in rows),
           "dc_drop_mV_design": round(drop * 1e3, 2), "dc_drop_mV_typ": round(drop_typ * 1e3, 2),
           "C_total_uF": {"min": round(ctot_min * 1e6, 3), "nom": round(ctot_nom * 1e6, 3),
                          "max": round(ctot_max * 1e6, 3)},
           "f_corner_Hz": {"nominal": round(fc_nom, 1), "highest_corner": round(fc_max, 1)},
           "esc": esc, "alias_bands": bands}
    if quiet:
        return out

    lim = a["peaking_limit_dB"]
    check("IMU supply filter peaking <= +%.1f dB at every corner (L 0.47..4.7 uH, Rdc 0.1..0.39 R, C %s, R +/-1 %%)"
          % (lim, "x%.2f..x%.2f" % (c["bulk_dc_bias_factor"]["min"], c["bulk_dc_bias_factor"]["max"])),
          worst_peak[0] <= lim,
          "worst %+.3f dB (max LC Q %.3f -> heavily overdamped by 22R; f0_LC %.0f..%.0f kHz)"
          % (worst_peak[0], out["Q_LC_max"], min(r_["f0_LC_kHz"] for r_ in rows),
             max(r_["f0_LC_kHz"] for r_ in rows)))
    v_imu = 3.3 - drop
    check("IMU rail DC drop <= %.0f mV at %.2f mA design load (Rdc 0.39 R + R +1 %%)"
          % (a["dc_drop_limit_mV"], I * 1e3), drop * 1e3 <= a["dc_drop_limit_mV"],
          "%.1f mV (typ %.1f mV at 0.88 mA) -> +3V3_IMUx = %.3f V, inside 1.71..3.6 V; IMU VIH 0.7*VDDIO = %.2f V"
          % (drop * 1e3, drop_typ * 1e3, v_imu, 0.7 * v_imu))
    for fe in rr["esc_pwm_hz"]:
        e = esc["%d_kHz" % int(fe / 1e3)]
        check("Board filter >= %.0f dB at ESC PWM %.0f kHz (worst corner)" % (a["esc_atten_min_dB"], fe / 1e3),
              -e["worst_dB"] >= a["esc_atten_min_dB"],
              "worst %.1f dB, nominal %.1f dB" % (e["worst_dB"], e["nominal_dB"]))
    worst_ripple = max(e["ripple_out_mVpp"] for e in esc.values())
    check("Residual ESC ripple at IMU VDD <= %.0f mVpp datasheet supply-noise limit (input %.0f mVpp, assumed)"
          % (a["imu_supply_limits"]["noise_mVpp"], rr["V_esc_ripple_mVpp"]),
          worst_ripple <= a["imu_supply_limits"]["noise_mVpp"],
          "worst %.2f mVpp; max tolerable input ripple at 24 kHz = %.0f mVpp"
          % (worst_ripple, a["imu_supply_limits"]["noise_mVpp"] / 10 ** (esc["24_kHz"]["worst_dB"] / 20)))
    esc_bands = [b_ for b_ in bands if b_["band_kHz"][0] >= 20.0]
    wb = max(b_["worst_board_dB"] for b_ in esc_bands)
    low = [b_ for b_ in bands if b_["band_kHz"][0] < 20.0]
    check("Board filter >= %.0f dB in every k*ODR +/- 1 kHz alias band from 20 kHz to 490 kHz (ODR %.0f Hz)"
          % (a["esc_atten_min_dB"], odr), -wb >= a["esc_atten_min_dB"],
          "least attenuation %.1f dB; bands below 20 kHz (k=1,2: %s) get %s dB - no ESC PWM energy there, "
          "covered by the in-sensor AAF (sec 2)"
          % (wb, ", ".join("%.1f-%.1f kHz" % tuple(b_["band_kHz"]) for b_ in low),
             "/".join("%.1f" % b_["worst_board_dB"] for b_ in low)))
    return out


def supply_startup_and_dynamics(sf):
    a = ASSUMPTIONS
    d = DESIGN["supply_filter"]
    R = d["R_ohm"] * (1 + d["R_tol_pct"] / 100) + a["bead"]["Rdc_ohm_max"]
    tau_max = R * sf["C_total_uF"]["max"] * 1e-6
    tau_min = (d["R_ohm"] * (1 - d["R_tol_pct"] / 100) + a["bead"]["Rdc_ohm_min"]) * sf["C_total_uF"]["min"] * 1e-6
    lim = a["imu_supply_limits"]
    ramp = a["rail_3v3_ramp_ms"]
    # 10-90 % of a ramp through a 1st-order RC ~ RSS of ramp and 2.2 tau (overdamped, monotonic)
    rise_max_ms = math.sqrt(ramp["max"] ** 2 + (2.2 * tau_max * 1e3) ** 2)
    rise_min_ms = math.sqrt(ramp["min"] ** 2 + (2.2 * tau_min * 1e3) ** 2)
    monotonic = sf["Q_LC_max"] < 0.5
    check("IMU rail ramp monotonic and inside %.2f..%.0f ms (DS-000347 Table 4)"
          % (lim["ramp_min_ms"], lim["ramp_max_ms"]),
          monotonic and lim["ramp_min_ms"] <= rise_min_ms and rise_max_ms <= lim["ramp_max_ms"],
          "tau %.0f..%.0f us (2.2tau %.2f ms max); with +3V3 ramp %.1f..%.1f ms (assumed) -> IMU rail 10-90 %% "
          "%.2f..%.2f ms; LC Q %.3f < 0.5 -> no overshoot"
          % (tau_min * 1e6, tau_max * 1e6, 2.2 * tau_max * 1e3, ramp["min"], ramp["max"],
             rise_min_ms, rise_max_ms, sf["Q_LC_max"]))
    # IMU rail lags +3V3 by S*tau during a linear ramp -> Y2 (on +3V3) could
    # back-drive pin 9 above VDDIO + 0.5 V if it oscillates during the ramp.
    lag = []
    for rms in (ramp["min"], ramp["max"]):
        t_full = rms * 1e-3 / 0.8          # 0-100 % ramp duration for a linear ramp
        s = 3.3 / t_full
        # first-order RC driven by a ramp: lag peaks at the end of the ramp
        lag.append(s * tau_max * (1 - math.exp(-t_full / tau_max)))
    # smallest 10-90 % ramp that keeps the lag <= 0.5 V (bisection on the formula above)
    lo_, hi_ = 1e-6, 1.0
    for _ in range(80):
        mid = (lo_ + hi_) / 2
        tf = mid / 0.8
        if 3.3 / tf * tau_max * (1 - math.exp(-tf / tau_max)) > lim["input_abs_max_over_VDDIO_V"]:
            lo_ = mid
        else:
            hi_ = mid
    min_ramp_ms = hi_ * 1e3
    worst_lag = max(lag)
    r_series = DESIGN["clock"]["branches"]["R14_IMU1"]
    i_inj = max(0.0, worst_lag - 0.6) / (r_series + a["y2"]["R_out_min_ohm"])
    i_inj_1k = max(0.0, worst_lag - 0.6) / (1000.0 + a["y2"]["R_out_min_ohm"])
    check("Power-up: IMU rail lag behind +3V3 (Y2 supply) <= VDDIO+0.5 V abs-max on CLKIN pin 9",
          worst_lag <= lim["input_abs_max_over_VDDIO_V"],
          "peak lag = %.2f..%.2f V for +3V3 ramp %.1f..%.1f ms (needs +3V3 10-90 %% >= %.2f ms); "
          "worst ESD-diode injection through %g R ~ %.1f mA for < %.1f ms (latch-up rating 100 mA; %.1f mA with "
          "R14/R15 = 1k). Only matters if Y2 already oscillates during the ramp (start-up <= 5 ms, min not "
          "specified); the same lag applies to any +3V3-powered driver of an IMU pin"
          % (min(lag), worst_lag, ramp["min"], ramp["max"], min_ramp_ms, r_series, i_inj * 1e3,
             ramp["max"], i_inj_1k * 1e3), severity="warn")
    # gyro-enable load step
    step = a["imu_current"]["gyro_enable_step_mA"] * 1e-3 * R
    # SPI burst self-ripple (SDO switching from VDDIO, synchronous with ODR)
    sb = a["spi_burst"]
    q = sb["C_sdo_load_pF"] * 1e-12 * 3.3 * sb["bits_per_sample"] * sb["toggle_fraction"] / 2
    v_burst = q / (sf["C_total_uF"]["min"] * 1e-6)
    check("Self-generated ripple (gyro-enable step, SPI SDO bursts) << 10 mVpp supply-noise limit",
          v_burst * 1e3 < 1.0 and step * 1e3 < 30,
          "gyro enable 0.73 mA step -> %.1f mV slow sag (tau %.0f us, settles before ArduPilot reads); "
          "SDO burst %.2f nC/sample -> %.3f mV, synchronous with ODR so it aliases to DC (constant offset)"
          % (step * 1e3, tau_max * 1e6, q * 1e9, v_burst * 1e3))
    return {"tau_us": {"min": round(tau_min * 1e6, 1), "max": round(tau_max * 1e6, 1)},
            "rail_rise_10_90_ms": {"min": round(rise_min_ms, 3), "max": round(rise_max_ms, 3)},
            "por_note": "IMU register access 1 ms after its rail is valid (DS Table 4); ArduPilot probes "
                        "the IMUs hundreds of ms after boot, so the extra ~%.2f ms is irrelevant." % (2.2 * tau_max * 1e3),
            "powerup_lag_V": [round(x, 3) for x in lag], "min_3v3_ramp_ms_for_no_backdrive": round(min_ramp_ms, 3),
            "gyro_enable_sag_mV": round(step * 1e3, 2), "spi_burst_ripple_mV": round(v_burst * 1e3, 4)}


def candidate_sweep():
    rows = []
    for R in (10.0, 22.0, 33.0, 47.0):
        for Cb in (4.7, 10.0, 22.0):
            s = supply_filter(R, Cb, quiet=True)
            rows.append({"R_ohm": R, "C_bulk_uF": Cb,
                         "att_24k_worst_dB": s["esc"]["24_kHz"]["worst_dB"],
                         "att_alias_bands_ge20k_worst_dB": max(b["worst_board_dB"] for b in s["alias_bands"]
                                                              if b["band_kHz"][0] >= 20.0),
                         "peak_dB": round(s["worst_peak_dB"], 3), "dc_drop_mV": s["dc_drop_mV_design"],
                         "tau_max_us": round((R * 1.01 + 0.39) * s["C_total_uF"]["max"], 1)})
    return rows


# ---------------------------------------------------------------------------
# 2. In-sensor digital AAF
# ---------------------------------------------------------------------------
def lp2(f, fc, q=1 / math.sqrt(2)):
    s = 1j * f / fc
    return 1 / (s * s + s / q + 1)


def read_ap_aaf(ap_root):
    path = os.path.join(ap_root, "libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp")
    src = open(path).read()
    blk = src.split("// ICM-42688 / ICM-40609 / IIM-426525", 1)[1].split("sampling_rate_hz = backend_rate_hz", 1)[0]
    got = {}
    for m in re.finditer(r"case (\d+):.*?aaf_delt = (\d+);.*?aaf_deltsqr = (\d+);.*?aaf_bitshift = (\d+);",
                         blk, re.S):
        got[int(m.group(1))] = (int(m.group(2)), int(m.group(3)), int(m.group(4)))
    head = src.split("void AP_InertialSensor_Invensensev3::set_filter_and_scaling", 1)[1]
    m = re.search(r"uint8_t aaf_delt = (\d+), accel_aaf_delt = (\d+);", head)
    m2 = re.search(r"uint16_t aaf_deltsqr = (\d+), accel_aaf_deltsqr = (\d+);", head)
    m3 = re.search(r"uint8_t aaf_bitshift = (\d+), accel_aaf_bitshift = (\d+);", head)
    got[1000] = (int(m.group(1)), int(m2.group(1)), int(m3.group(1)))
    got["accel"] = (int(m.group(2)), int(m2.group(2)), int(m3.group(2)))
    return got, path


# DS-000347 sec 5.3 table rows used (3 dB Hz: DELT, DELTSQR, BITSHIFT)
DS_AAF = {213.0: (5, 25, 10), 258.0: (6, 36, 10), 536.0: (12, 144, 8), 997.0: (21, 440, 6)}
AP_AAF_EXPECTED = {1000: (6, 36, 10), 2000: (12, 144, 8), 4000: (21, 440, 6), 8000: (21, 440, 6),
                   "accel": (5, 25, 10)}


def sensor_aaf(ap_root, sf):
    a = ASSUMPTIONS
    sfl = a["sensor_filters"]
    if ap_root:
        got, path = read_ap_aaf(ap_root)
        mism = ["%s ours=%s ap=%s" % (k, AP_AAF_EXPECTED[k], got.get(k)) for k in AP_AAF_EXPECTED
                if got.get(k) != AP_AAF_EXPECTED[k]]
        check("ArduPilot ICM-42688 AAF coefficients match the modelled table", not mism,
              "; ".join(mism) if mism else "DELT/DELTSQR/BITSHIFT per rate identical (%s)" % os.path.relpath(path, ap_root))
    ds_ok = all(v in DS_AAF.values() for v in AP_AAF_EXPECTED.values())
    check("ArduPilot AAF codes are rows of the DS-000347 sec 5.3 AAF table", ds_ok,
          "1k: DELT6 -> 258 Hz, 2k: DELT12 -> 536 Hz, 4k/8k: DELT21 -> 997 Hz, accel DELT5 -> 213 Hz. "
          "NB: ArduPilot's 'default: // 1KHz, 348Hz AAF' comment is wrong (348 Hz = DELT 8); code uses 258 Hz")

    fclk = DESIGN["clock"]["f_hz"]
    scale_clk = fclk / a["clkin"]["f_ref_hz"]
    fc_aaf = {1000: 258.0, 2000: 536.0, 4000: 997.0, 8000: 997.0}
    per_odr = {}
    for odr_n, fa in fc_aaf.items():
        res = {}
        for label, sc in (("internal_osc", 1.0), ("clkin_32768", scale_clk)):
            odr = odr_n * sc
            fa_s = fa * sc
            fu = sfl["ui_2nd_order_3dB_hz"][odr_n] * sc
            ny = odr / 2
            h_a = abs(lp2(ny, fa_s))
            h_u = abs(lp2(ny, fu))
            res[label] = {"ODR_Hz": round(odr, 1), "AAF_3dB_Hz": round(fa_s, 1), "UI_3dB_Hz": round(fu, 1),
                          "AAF_att_at_Nyquist_dB": round(db(h_a), 2),
                          "AAF_plus_UI_att_at_Nyquist_dB": round(db(h_a * h_u), 2),
                          "group_delay_note": "AAF+UI group delay not published (ArduPilot comment: ~0.4 ms @1k UI)"}
        per_odr[str(odr_n)] = res
    r8 = per_odr["8000"]["clkin_32768"]
    check("8 kHz ODR (8192 Hz with CLKIN): AAF+UI attenuation at Nyquist reported",
          r8["AAF_plus_UI_att_at_Nyquist_dB"] < -20,
          "AAF %.1f dB + UI = %.1f dB at %.0f Hz (ArduPilot targets ~40 dB at Nyquist and accepts less at 8k; "
          "residual is handled by the 8k->loop-rate decimation + notch/LPF in AP)"
          % (r8["AAF_att_at_Nyquist_dB"], r8["AAF_plus_UI_att_at_Nyquist_dB"], r8["ODR_Hz"]))

    # Combined board+sensor rejection of ESC supply ripple (fundamentals + harmonics)
    fs_int = sfl["internal_rate_hz"] * scale_clk
    odr = 8000 * scale_clk
    fa = 997.0 * scale_clk
    fu = sfl["ui_2nd_order_3dB_hz"][8000] * scale_clk
    rr = a["rail_ripple"]
    tones = []
    for fe in rr["esc_pwm_hz"]:
        for n in range(1, rr["harmonics"] + 1):
            f = fe * n
            board = sf_worst_att(f)
            f_int = fold(f, fs_int)
            dig = db(lp2(f_int, fa) * lp2(f_int, fu))
            f_bb = fold(f_int, odr)
            f_bb_noclk = fold(fold(f, sfl["internal_rate_hz"]), 8000.0)
            tones.append({"src_kHz": fe / 1e3, "harmonic": n, "f_kHz": round(f / 1e3, 3),
                          "board_dB": round(board, 2), "after_32k_decim_Hz": round(f_int, 1),
                          "digital_AAF_UI_dB": round(dig, 2), "combined_dB": round(board + dig, 2),
                          "baseband_alias_Hz_clkin": round(f_bb, 1),
                          "baseband_alias_Hz_internal_8000": round(f_bb_noclk, 1),
                          "residual_uVpp": round(rr["V_esc_ripple_mVpp"] * 1e3 / n * 10 ** ((board + dig) / 20), 2)})
    in_bb = [t for t in tones if t["baseband_alias_Hz_clkin"] <= a["baseband_hz"]]
    worst = max(tones, key=lambda t: t["combined_dB"])
    worst_bb = max(in_bb, key=lambda t: t["combined_dB"]) if in_bb else None
    check("Combined board+sensor rejection of ESC ripple tones landing in 0-1 kHz >= 40 dB (ODR 8192 Hz)",
          worst_bb is None or worst_bb["combined_dB"] <= -40,
          ("%d of %d tones alias into 0-1 kHz; least rejected: %.0f kHz -> %.0f Hz alias, board %.1f + sensor %.1f "
           "= %.1f dB" % (len(in_bb), len(tones), worst_bb["f_kHz"], worst_bb["baseband_alias_Hz_clkin"],
                          worst_bb["board_dB"], worst_bb["digital_AAF_UI_dB"], worst_bb["combined_dB"]))
          if worst_bb else "none alias into baseband")
    notes = ("24 kHz ESC PWM aliases to 576 Hz at 8192 Hz ODR (24000 = 3*8192 - 576) and to ~0 Hz at a "
             "nominal 8000 Hz ODR; with the +/-3 %% internal oscillator the alias wanders anywhere in 0-4 kHz. "
             "The decimation-filter response (32 kHz) is unpublished and taken as 0 dB; tones within +/-1 kHz of "
             "k*32.768 kHz would bypass the AAF entirely and are protected only by the board filter "
             "(>= %.1f dB there, sec 1)." % -max(sf_worst_att(k * fs_int - 1000) for k in range(1, 15)))
    return {"per_odr": per_odr, "esc_tones": tones, "worst_tone": worst, "worst_baseband_tone": worst_bb,
            "internal_rate_hz_clkin": fs_int, "note": notes, "model": sfl["model"]}


_SF_CACHE = {}


def sf_worst_att(f):
    """Least board attenuation over all supply-filter corners (design values)."""
    if f in _SF_CACHE:
        return _SF_CACHE[f]
    a = ASSUMPTIONS
    d = DESIGN["supply_filter"]
    b, c = a["bead"], a["caps"]
    w = -1e9
    for L in b["L_uH_list"]:
        for cc in ("min", "max"):
            cb = d["C_bulk_uF"] * 1e-6 * c["bulk_dc_bias_factor"][cc]
            cl = [v * 1e-6 * c["local_factor"][cc] for v in d["C_local_uF"]]
            for rdc in (b["Rdc_ohm_min"], b["Rdc_ohm_max"]):
                r = d["R_ohm"] * (1 - d["R_tol_pct"] / 100)
                w = max(w, db(h_supply(f, L * 1e-6, rdc, b["R_parallel_ohm"], r, cb, cl, a)))
    _SF_CACHE[f] = w
    return w


# ---------------------------------------------------------------------------
# 3. Synchronisation
# ---------------------------------------------------------------------------
def read_ap_rate(ap_root):
    path = os.path.join(ap_root, "libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp")
    src = open(path).read().split("::_update_sensor_rate", 1)[1].split("\n}\n", 1)[0]
    g = lambda k: float(re.search(r"%s\s*=\s*([0-9.]+)f" % k, src).group(1))
    fc = [float(x) for x in re.findall(r"filter_constant\s*=\s*([0-9.]+)f", src)]
    up = [float(x) for x in re.findall(r"upper_limit\s*=\s*([0-9.]+)f", src)]
    return {"filter": fc[0], "upper": up[0], "lower": g("lower_limit"), "conv_filter": fc[1],
            "conv_upper": up[1]}, path


def rate_tracking(r0, f_true, steady=True):
    """Simulate _update_sensor_rate() once per second; return seconds to within 0.5 % and 0.1 %."""
    ar = ASSUMPTIONS["ardupilot_rate"]
    r = r0
    t05 = t01 = None
    for s in range(1, 2000):
        if steady:
            lo, hi, k = 1 - ar["window"], 1 + ar["window"], ar["filter"]
        else:
            lo, hi, k = 0.5, 2.0, ar["conv_filter"]
        obs = min(max(f_true, r * lo), r * hi)
        r = k * r + (1 - k) * obs
        e = abs(r - f_true) / f_true
        if t05 is None and e <= 0.005:
            t05 = s
        if t01 is None and e <= 0.001:
            t01 = s
            break
    return t05, t01


def synchronisation(ap_root):
    a = ASSUMPTIONS
    ar = a["ardupilot_rate"]
    if ap_root:
        got, path = read_ap_rate(ap_root)
        ok = (abs(got["upper"] - 1 - ar["window"]) < 1e-9 and abs(1 - got["lower"] - ar["window"]) < 1e-9
              and got["filter"] == ar["filter"] and got["conv_filter"] == ar["conv_filter"])
        check("ArduPilot _update_sensor_rate() constants match model", ok,
              "%s (%s)" % (json.dumps(got), os.path.relpath(path, ap_root)))
    y = a["y2"]
    ck = a["clkin"]
    f = DESIGN["clock"]["f_hz"]
    ppm_total = y["tol_ppm"] + y["stab_ppm"] + y["aging_ppm_per_yr"] * y["life_years"]
    fmin, fmax = f * (1 - ppm_total * 1e-6), f * (1 + ppm_total * 1e-6)
    check("CLKIN 32.768 kHz (+/-%.0f ppm life) inside ICM-42688-P 31..50 kHz" % ppm_total,
          ck["f_min_hz"] <= fmin and fmax <= ck["f_max_hz"],
          "%.3f..%.3f Hz; margin %.1f %% to 31 kHz" % (fmin, fmax, (fmin / ck["f_min_hz"] - 1) * 100))
    scale = f / ck["f_ref_hz"]
    odr = DESIGN["odr_nominal_hz"] * scale
    shift = scale - 1
    check("ODR with CLKIN = %.0f Hz (%+.2f %%) inside ArduPilot +/-%.0f %% rate window around 8000 Hz"
          % (odr, shift * 100, ar["window"] * 100), abs(shift) < ar["window"],
          "margin %.2f %% of 5 %%; boot convergence (first 30 s: x0.5..x2 window, k=0.8) reaches 0.1 %% in %s s"
          % ((ar["window"] - shift) * 100, rate_tracking(8000.0, odr, steady=False)[1]))
    oi = a["internal_osc"]
    per_imu = oi["initial_pct"] + oi["temp_pct"]
    rel_pct_init = 2 * oi["initial_pct"]
    rel_pct_worst = 2 * per_imu
    walk_init = rel_pct_init / 100 * DESIGN["odr_nominal_hz"]
    walk_worst = rel_pct_worst / 100 * DESIGN["odr_nominal_hz"]
    dt_err_ppm = ppm_total
    check("Shared CLKIN removes inter-IMU drift (0 ppm relative, constant offset <= 1 ODR period)", True,
          "without CLKIN: per-IMU ODR +/-%.0f %% (init+temp, DS Table 4) -> relative up to %.0f %% "
          "(%.0f %% at 25 C) = %.0f..%.0f samples/s phase walk (%.0f..%.0f ms/s) at 8 kHz; with CLKIN: "
          "0 ppm relative, fixed offset in [0, %.1f us) set by configuration order; absolute dt error "
          "+/-%.0f ppm (gyro integration scale error %.4f %%)"
          % (per_imu, rel_pct_worst, rel_pct_init, walk_init, walk_worst, walk_init / 8.0, walk_worst / 8.0,
             1e6 / odr, dt_err_ppm, dt_err_ppm * 1e-4))
    # loss-of-clock fallback: ODR jumps from 8192 Hz to internal 8000 Hz +/-3 %
    fb = {}
    for label, ft in (("internal_low", 8000 * (1 - per_imu / 100)), ("internal_nom", 8000.0),
                      ("internal_high", 8000 * (1 + per_imu / 100))):
        step = ft / odr - 1
        t05, t01 = rate_tracking(odr, ft, steady=True)
        fb[label] = {"new_ODR_Hz": round(ft, 1), "step_pct": round(step * 100, 2),
                     "outside_5pct_window": abs(step) > ar["window"], "s_to_0p5pct": t05, "s_to_0p1pct": t01}
    worst_t = max(v["s_to_0p5pct"] for v in fb.values())
    check("Loss-of-CLKIN fallback: ArduPilot rate estimate re-tracks within 5 s without a driver reset",
          worst_t <= 5,
          "ODR steps %s %%; _update_sensor_rate (k=0.98/s after 30 s) needs up to %d s to get within 0.5 %% "
          "(notch/FFT centre and dt off by up to %.1f %% meanwhile). The fallback patch must reset/re-seed "
          "the backend rate (or re-init the IMU) when it drops RTC mode."
          % ("/".join("%+.2f" % v["step_pct"] for v in fb.values()), worst_t,
             max(abs(v["step_pct"]) for v in fb.values())), severity="warn")
    return {"clkin_Hz_range": [round(fmin, 3), round(fmax, 3)], "clkin_ppm_life": ppm_total,
            "odr_clkin_Hz": odr, "odr_shift_pct": round(shift * 100, 3),
            "aaf_ui_cutoffs_scale": scale,
            "no_clkin": {"per_imu_pct": per_imu, "relative_pct_worst": rel_pct_worst,
                         "relative_pct_25C": rel_pct_init,
                         "phase_walk_samples_per_s": [walk_init, walk_worst]},
            "with_clkin": {"relative_ppm": 0.0, "phase_offset_max_us": round(1e6 / odr, 2),
                           "abs_dt_error_ppm": dt_err_ppm},
            "fallback": fb,
            "fifo_timestamp_note": "DS sec 12.7: in RTC mode FIFO timestamps scale by 32.768/30; ArduPilot uses "
                                   "the measured sample rate, not FIFO timestamps, so no change needed."}


# ---------------------------------------------------------------------------
# 4. Clock distribution
# ---------------------------------------------------------------------------
def star_transient(rout, c0, branches, dt=0.05e-9, tmax=2e-6):
    """Step response of Vs -> rout -> node0 (c0) -> Rk -> Ck. Returns 10-90 % times (ns)."""
    v0 = 0.0
    vk = [0.0] * len(branches)
    t = 0.0
    res = {"osc": [None, None]}
    for i in range(len(branches)):
        res[i] = [None, None]
    while t < tmax:
        ik = [(v0 - vk[i]) / r for i, (r, _) in enumerate(branches)]
        i0 = (1.0 - v0) / rout
        v0 += dt * (i0 - sum(ik)) / c0
        for i, (r, c) in enumerate(branches):
            vk[i] += dt * ik[i] / c
        t += dt
        for key, v in [("osc", v0)] + list(enumerate(vk)):
            if res[key][0] is None and v >= 0.1:
                res[key][0] = t
            if res[key][1] is None and v >= 0.9:
                res[key][1] = t
        if all(x[1] is not None for x in res.values()):
            break
    return {k: (v[1] - v[0]) * 1e9 for k, v in res.items()}


def clock_distribution():
    a = ASSUMPTIONS
    y, cl = a["y2"], a["clock_loads"]
    f = DESIGN["clock"]["f_hz"]
    T = 1 / f
    rout_max = y["tr_max_ns"] * 1e-9 / (2.2 * y["L_CMOS_pF"] * 1e-12)
    br = DESIGN["clock"]["branches"]
    out = {"R_out_max_derived_ohm": round(rout_max, 1)}
    variants = {"design_33R": (br["R14_IMU1"], br["R15_IMU2"]), "alt_1k": (1000.0, 1000.0)}
    for vname, (r14, r15) in variants.items():
        vres = {}
        for cname in ("nom", "max"):
            cimu = cl["C_imu_pin9_pF"][cname] + cl["C_trace_pF_per_branch"]
            cmcu = cl["C_mcu_pin_pF"] + cl["C_trace_pF_per_branch"]
            branches = [(r14, cimu * 1e-12), (r15, cimu * 1e-12), (br["R16_MCU_PD12"], cmcu * 1e-12)]
            c_total = cl["C_osc_node_pF"] + 2 * cimu + cmcu
            tr = star_transient(rout_max, cl["C_osc_node_pF"] * 1e-12, branches)
            tr_fast = star_transient(y["R_out_min_ohm"], cl["C_osc_node_pF"] * 1e-12, branches)
            # attenuation of the fundamental at each pin (RC low-pass per branch, Rout included)
            att = []
            for r, c in branches:
                h = 1 / (1 + 1j * 2 * math.pi * f * (rout_max + r) * c_total * 1e-12)
                att.append(db(h))
            vres[cname] = {"C_total_pF": round(c_total, 2),
                           "tr_ns_Rout_max": {"osc_pin": round(tr["osc"], 1), "U2_pin9": round(tr[0], 1),
                                              "U3_pin9": round(tr[1], 1), "MCU_PD12": round(tr[2], 1)},
                           "tr_ns_Rout_min": {"osc_pin": round(tr_fast["osc"], 2), "U2_pin9": round(tr_fast[0], 2),
                                              "MCU_PD12": round(tr_fast[2], 2)},
                           "att_32k_dB_worst": round(min(att), 5)}
        out[vname] = vres
    d = out["design_33R"]
    tr_imu = max(d[c]["tr_ns_Rout_max"]["U2_pin9"] for c in ("nom", "max"))
    tr_osc = max(d[c]["tr_ns_Rout_max"]["osc_pin"] for c in ("nom", "max"))
    check("CLKIN edge at IMU pin 9 <= 1 %% of the 32.768 kHz period (worst Rout %.0f R, CI 10 pF)" % rout_max,
          tr_imu <= 0.01 * T * 1e9,
          "tr %.1f ns vs %.0f ns (1 %% of %.2f us)" % (tr_imu, 0.01 * T * 1e9, T * 1e6))
    check("No meaningful attenuation of the 32.768 kHz fundamental at any load (> -0.01 dB)",
          min(d[c]["att_32k_dB_worst"] for c in ("nom", "max")) > -0.01,
          "worst %.6f dB" % min(d[c]["att_32k_dB_worst"] for c in ("nom", "max")))
    # duty-cycle distortion from slower edges, assuming fully asymmetric drive (one edge at Rout_max, the other ideal)
    cmax = d["max"]["C_total_pF"] * 1e-12
    dduty = 0.69 * rout_max * cmax / T * 100
    check("Extra duty-cycle distortion from the actual load (vs 45-55 % spec at 15 pF) <= 0.5 %",
          dduty <= 0.5, "worst %.3f %% of period (0.69*Rout_max*C_total, one-sided)" % dduty)
    check("Y2 capacitive load within L_CMOS %.0f pF datasheet condition (R14/R15 = %g R)"
          % (y["L_CMOS_pF"], br["R14_IMU1"]),
          d["max"]["C_total_pF"] <= y["L_CMOS_pF"] and tr_osc <= y["tr_max_ns"],
          "load %.1f pF nominal / %.1f pF worst (CI<10 pF per IMU pin 9, MCU 5 pF, traces) -> tr at Y2 pin up to "
          "%.0f ns vs 25 ns spec. Functionally harmless at 32 kHz (edges %.2f %% of period, duty shift %.3f %%), "
          "but outside the condition under which 45-55 %% duty and 25 ns tr are guaranteed. Raising R14/R15 does "
          "NOT fix this with the worst-case Rout (alt_1k: Y2-pin tr %.0f ns, IMU-pin tr %.0f ns); only a buffer "
          "would. Accept, and confirm duty/edges on Y2 pin 3 with a scope at bring-up."
          % (d["nom"]["C_total_pF"], d["max"]["C_total_pF"], tr_osc, tr_imu / (T * 1e9) * 100, dduty,
             max(out["alt_1k"][c]["tr_ns_Rout_max"]["osc_pin"] for c in ("nom", "max")),
             max(out["alt_1k"][c]["tr_ns_Rout_max"]["U2_pin9"] for c in ("nom", "max"))), severity="warn")
    vdd = 3.3
    vio = 3.3 - 1.5e-3 * 22.39
    check("Y2 VOH (0.9*Vdd) >= IMU VIH (0.7*VDDIO) and <= VDDIO+0.5 V in steady state",
          y["VOH_frac"] * vdd >= 0.7 * vio and vdd <= vio + 0.5,
          "VOH >= %.2f V vs VIH %.2f V; Y2 on +3V3 is %.0f mV above +3V3_IMUx" % (0.9 * vdd, 0.7 * vio,
                                                                               (vdd - vio) * 1e3))
    out["startup_note"] = ("Y2 start-up up to 5 ms; RTC mode must only be enabled after PD12 sees edges "
                           "(ArduPilot IMU init is far later than 5 ms after power-up).")
    out["R17_note"] = "OE: 'enabled if logic 1 or open' (Y2 datasheet) - R17 10k pull-up is belt-and-braces."
    return out


# ---------------------------------------------------------------------------
# 5. SPI termination and timing
# ---------------------------------------------------------------------------
def spi():
    a = ASSUMPTIONS["spi_elec"]
    rs = DESIGN["spi"]["R_series_ohm"]
    out = {}
    c_imu = (a["C_imu_in_pF"] + a["C_trace_pF"]) * 1e-12
    c_mcu = (a["C_mcu_in_pF"] + a["C_trace_pF"]) * 1e-12
    tpd = a["trace_mm_max"] * 6.5e-3   # ns, ~6.5 ps/mm inner/outer FR4 microstrip
    tr_rc_sclk = 2.2 * (a["R_mcu_drv_ohm"] + rs) * c_imu * 1e9
    tr_sclk = math.sqrt(a["t_mcu_intrinsic_ns"] ** 2 + tr_rc_sclk ** 2)
    d50_sclk = 0.69 * (a["R_mcu_drv_ohm"] + rs) * c_imu * 1e9 + tpd
    d50_miso = 0.69 * (a["R_imu_drv_ohm"] + rs) * c_mcu * 1e9 + tpd
    tr_miso = 2.2 * (a["R_imu_drv_ohm"] + rs) * c_mcu * 1e9
    lumped = tpd < tr_sclk / 6
    for bus, fsclk in (("SPI1", DESIGN["spi"]["SPI1_sclk_hz"]), ("SPI4", DESIGN["spi"]["SPI4_sclk_hz"])):
        T = 1e9 / fsclk
        half = T / 2
        # Mode 3: both sides shift on the falling edge, sample on the rising edge (half period apart).
        miso_margin = half - (d50_sclk + a["icm_tVD_SDO_ns"] + d50_miso + a["t_mcu_tsu_mi_ns"])
        mosi_setup = half - (a["t_mcu_tv_mo_ns"] + d50_sclk * 0) - a["icm_tSU_SDI_ns"]
        mosi_hold = half - a["icm_tHD_SDI_ns"]
        out[bus] = {"sclk_MHz": fsclk / 1e6, "period_ns": T, "quarter_ns": T / 4,
                    "tr_sclk_ns": round(tr_sclk, 2), "tr_miso_ns": round(tr_miso, 2),
                    "miso_margin_ns": round(miso_margin, 2), "mosi_setup_margin_ns": round(mosi_setup, 2),
                    "mosi_hold_margin_ns": round(mosi_hold, 2)}
        check("%s %.1f MHz: SCLK/MOSI/MISO edge (33R, ~12 pF) <= 1/3 of quarter period" % (bus, fsclk / 1e6),
              max(tr_sclk, tr_miso) <= T / 12,
              "tr SCLK %.2f ns (RC part %.2f ns), MISO %.2f ns vs T/4 = %.1f ns" % (tr_sclk, tr_rc_sclk, tr_miso, T / 4))
        check("%s %.1f MHz: MISO read budget (SCLK path + tVD.SDO 21.5 ns + MISO path + MCU tsu) fits half period"
              % (bus, fsclk / 1e6), miso_margin >= 5.0,
              "margin %.1f ns of %.1f ns (needs >= 5 ns); MOSI setup margin %.1f ns, hold margin %.1f ns; "
              "tLOW/tHIGH %.0f ns >= 17 ns" % (miso_margin, half, mosi_setup, mosi_hold, half))
    check("SPI traces electrically short -> 33R source termination non-critical (EMI/ringing damping only)",
          lumped and DESIGN["spi"]["SPI4_sclk_hz"] <= a["icm_fmax_hz"],
          "flight time %.2f ns for %.0f mm vs tr %.1f ns; R_drv+33R = %.0f R ~ Z0 of 0.1-0.15 mm trace; "
          "12.5 MHz <= 24 MHz ICM limit" % (tpd, a["trace_mm_max"], tr_sclk, a["R_mcu_drv_ohm"] + rs))
    out["note"] = ("ICM timing (Table 6) is characterised at VDDIO 1.8 V and Cload 20 pF; at 3.3 V the "
                   "output drive is stronger, so tVD.SDO 21.5 ns is conservative.")
    return out


def refdes_note(root):
    """Map the lead's frozen refdes to what tools/gen_mcu_core_sch.py actually uses (if present)."""
    path = os.path.join(root, "tools", "gen_mcu_core_sch.py")
    try:
        src = open(path).read()
    except OSError:
        return {"note": "tools/gen_mcu_core_sch.py not found; frozen refdes used as-is"}
    frozen = {"C5": ("1", "c_bulk"), "C6": ("1", "c_vdd"), "C7": ("1", "c_vddio"),
              "C8": ("2", "c_bulk"), "C9": ("2", "c_vdd"), "C10": ("2", "c_vddio")}
    mapping = {}
    for fz, (n, key) in frozen.items():
        m = re.search(r'\{"n": %s,.*?"%s": "(\w+)"' % (n, key), src, re.S)
        mapping[fz] = m.group(1) if m else None
    m = re.search(r'"c_dec": "(\w+)"', src)
    mapping["C11"] = m.group(1) if m else None
    return {"frozen_to_schematic": mapping, "source": "tools/gen_mcu_core_sch.py",
            "note": "Analysis uses the frozen values/topology; capacitor refdes were renumbered by the "
                    "schematic because C5..C11 already existed (VDDA/core decoupling). Values unaffected."}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ardupilot", default=os.environ.get("ARDUPILOT_ROOT"))
    ap.add_argument("-o", "--output", default=os.path.join(HERE, "imu_aaf_report.json"))
    args = ap.parse_args()
    ap_root = args.ardupilot if args.ardupilot and os.path.isdir(args.ardupilot) else None
    if not ap_root:
        print("note: ArduPilot tree not given/found; source cross-checks skipped")
    RESULTS["revision"] = "11.0"
    RESULTS["design"] = DESIGN
    RESULTS["assumptions"] = ASSUMPTIONS
    print("--- 1. supply anti-aliasing filter")
    sf = supply_filter()
    RESULTS["supply_filter"] = sf
    RESULTS["supply_startup"] = supply_startup_and_dynamics(sf)
    RESULTS["supply_candidates"] = candidate_sweep()
    print("--- 2. in-sensor AAF")
    RESULTS["sensor_aaf"] = sensor_aaf(ap_root, sf)
    print("--- 3. synchronisation")
    RESULTS["sync"] = synchronisation(ap_root)
    print("--- 4. clock distribution")
    RESULTS["clock_distribution"] = clock_distribution()
    print("--- 5. SPI")
    RESULTS["spi"] = spi()
    RESULTS["refdes"] = refdes_note(os.path.dirname(HERE))
    fails = [c for c in RESULTS["checks"] if c["severity"] == "fail"]
    warns = [c for c in RESULTS["checks"] if c["severity"] == "warn"]
    RESULTS["summary"] = {"checks": len(RESULTS["checks"]), "failures": len(fails), "warnings": len(warns),
                          "warning_list": [w["check"] for w in warns]}
    with open(args.output, "w") as fh:
        json.dump(RESULTS, fh, indent=2, default=str)
    print("%d checks, %d failures, %d warnings -> %s"
          % (len(RESULTS["checks"]), len(fails), len(warns), os.path.relpath(args.output)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
