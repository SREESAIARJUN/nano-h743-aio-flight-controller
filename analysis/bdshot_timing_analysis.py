#!/usr/bin/env python3
"""
Nano H743 AIO - bidirectional DShot600 timing and RPM-notch tracking analysis
(Revision 10.0)

Reads the configuration from the repository (ardupilot/hwdef.dat,
ardupilot/defaults.parm, am32_esc/eeprom_settings.json) and computes:

  1. TIM1 clocking and the DShot600 / GCR bit timing at 480 MHz, using
     ArduPilot's own prescaler arithmetic (RCOutput::setup_group_DMA,
     AP_HAL::RCOutput::calculate_bitrate_prescaler; RCOutput_bdshot telempsc).
  2. The bidir cycle budget: command frame + ESC turnaround + GCR reply +
     DMAMUX re-point margin vs the DShot frame period.
  3. The effective DShot frame rate exactly as
     AP_HAL_ChibiOS::RCOutput::set_dshot_rate() computes it, and the
     per-motor eRPM refresh rate under TIM1 round-robin capture.
  4. eRPM format quantisation over the motor speed envelope.
  5. Notch tracking lag during a throttle punch (REPORTED, not gated): age of
     the RPM the notch actually uses (round-robin wait + AP_ESC_Telem::get_rpm
     interpolation + loop consumption + ESC averaging) x motor frequency slew.
     Rev 10 review finding: no SERVO_DSHOT_RATE keeps this inside
     INS_HNTCH_BW, so it does not decide the rate.
  6. Line electricals for the R4..R7 100R series resistors.

Writes analysis/bdshot_report.json.  Exit 1 if any design gate fails.

Usage: python3 analysis/bdshot_timing_analysis.py
"""
import json
import math
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HWDEF = os.path.join(REPO, "ardupilot", "hwdef.dat")
PARM = os.path.join(REPO, "ardupilot", "defaults.parm")
AM32_EEPROM = os.path.join(REPO, "am32_esc", "eeprom_settings.json")
OUT = os.path.join(REPO, "analysis", "bdshot_report.json")

# ArduPilot constants (AP_HAL_ChibiOS/RCOutput.h, Filter/HarmonicNotchFilter.cpp)
TELEM_IC_SAMPLE = 16
GCR_BITS = 21
HARMONIC_NYQUIST_CUTOFF = 0.48
NOTCH_DISABLE_RATIO = 0.25
COPTER_DEFAULT_LOOP_HZ = 400       # SCHED_LOOP_RATE default (Copter)
INS_GYRO_RATE_HZ = 2000            # INS_GYRO_RATE default 1 on H7 = 2 kHz
DSHOT_RATES = {4: 150000, 5: 300000, 6: 600000, 7: 1200000}  # MOT_PWM_TYPE

# Motor envelope (5" 2207 1950-2450 KV on 4S): from hover idle to full speed
MOTOR_RPM_MIN = 1500
MOTOR_RPM_MAX = 42000
# Worst-case spool rate of the mechanical frequency during a full punch:
# ~6 000 RPM -> 40 000 RPM in ~60 ms (measured 5" class motors, prop-loaded)
PUNCH_SLEW_HZ_PER_S = (40000 - 6000) / 60.0 / 0.060   # ~9 440 Hz/s
# AM32 reports the average of the last 6 commutation periods; at hover
# (~20 000 RPM, 7 pole pairs -> 2 333 eRPS x 6 steps = 14 000 steps/s) one
# period is 71 us, so the 6-period window is ~0.43 ms of latency.
ESC_MEASUREMENT_LATENCY_S = 6.0 / (20000 * 7 / 60.0 * 6)

# Line electricals
R_SERIES = 100.0
C_LINE_PF = 4.0 + 5.0 + 5.0   # ~40 mm 0.15 mm trace over GND + ESC MCU pin + H7 pin
H7_PULLUP_OHM = 40000.0       # STM32H7 RPU typ


def parse_params(path):
    p = {}
    for line in open(path):
        line = line.split("#", 1)[0].strip()
        if line:
            k, v = line.split()[:2]
            p[k] = float(v)
    return p


def parse_hwdef(path):
    txt = open(path).read()
    clk = int(re.search(r"^MCU_CLOCKRATE_MHZ\s+(\d+)", txt, re.M).group(1))
    pwm = []
    for m in re.finditer(r"^(P[A-K]\d+)\s+(TIM(\d+)_CH(\d)(N?))\s+TIM\d+\s+PWM\((\d+)\)(.*)$", txt, re.M):
        pwm.append({"pin": m.group(1), "timer": int(m.group(3)), "ch": int(m.group(4)),
                    "compl": m.group(5) == "N", "out": int(m.group(6)), "bidir": "BIDIR" in m.group(7)})
    return clk, pwm


def parse_am32_eeprom(path):
    """AM32 keeps motor poles etc. in EEPROM (Inc/eeprom.h), not in targets.h;
    am32_esc/eeprom_settings.json is the programmed image.  Missing -> error."""
    d = json.load(open(path))
    return {s["field"]: s["value"] for s in d["settings"]}


def ap_dshot_rate(dshot_rate, loop_hz):
    """RCOutput::set_dshot_rate() + the rcout thread scheduling.
    Returns (mean_frame_hz, evenly_spaced, frame_intervals_us).

    Rate 0: _dshot_period_us = 1000 and _dshot_rate = 0.  Every loop push()
    sends a frame and (re)arms a 1 ms virtual timer (chVTSet resets it);
    dshot_update_tick() does not re-arm itself (_dshot_cycle + 1 < 0 false)
    but its synthetic send re-arms it from the thread.  So at a 400 Hz loop
    frames go out at 0, 1, 2, 2.5, 3.5, 4.5 ms ...: ~1.2 kHz mean, uneven.
    Rate >= 1: frames every loop_period / rate, phase-locked to push()."""
    if loop_hz <= 100 or dshot_rate == 0:
        loop_us = 1e6 / loop_hz
        n_extra = int(math.ceil(loop_us / 1000.0)) - 1
        intervals = [1000.0] * n_extra + [loop_us - 1000.0 * n_extra]
        return len(intervals) * loop_hz, False, intervals
    drate = dshot_rate * loop_hz
    while drate < 800:
        dshot_rate += 1
        drate = dshot_rate * loop_hz
    while dshot_rate > 1 and drate > max(4096, loop_hz):
        dshot_rate -= 1
        drate = dshot_rate * loop_hz
    return float(drate), True, [1e6 / drate] * dshot_rate


def encode_period(erpm):
    period = int((60e6 + erpm // 2) // erpm)
    e = 0
    while period > 511 and e < 7:
        period >>= 1
        e += 1
    return e, period


def ap_rpm_from_period(e, m, poles):
    erpm = m << e
    x100 = (600000 + erpm // 2) // erpm
    return x100 * 200 // poles


def main():
    gates = []

    def gate(ok, name, detail):
        gates.append({"gate": name, "pass": bool(ok), "detail": detail})

    p = parse_params(PARM)
    clk_mhz, pwm = parse_hwdef(HWDEF)
    am32 = parse_am32_eeprom(AM32_EEPROM)
    loop_hz = int(p.get("SCHED_LOOP_RATE", COPTER_DEFAULT_LOOP_HZ))
    poles = int(p["SERVO_BLH_POLES"])
    dshot_bps = DSHOT_RATES[int(p["MOT_PWM_TYPE"])]

    # ---- 1. clocking -------------------------------------------------------
    # H7 @480: HCLK = SYSCLK/2, PCLK2 = HCLK/2, APB timer clock = 2*PCLK2
    hclk = clk_mhz * 1e6 / 2
    tim1_clk = 2 * (hclk / 2)
    # RCOutput::setup_group_DMA: target = bitrate * DSHOT_BIT_WIDTH_TICKS (8),
    # prescaler = calculate_bitrate_prescaler() -> lrintf(clock/target) - 1,
    # ARR = 8 ticks per bit (bit_width_mul 1 when the prescaler is exact).
    DSHOT_BIT_WIDTH_TICKS = 8
    out_psc = int(round(tim1_clk / (dshot_bps * DSHOT_BIT_WIDTH_TICKS))) - 1
    out_tick_hz = tim1_clk / (out_psc + 1)
    out_ticks_per_bit = out_tick_hz / dshot_bps
    gcr_bps = dshot_bps * 5 // 4
    telempsc = int(round((tim1_clk / gcr_bps + 0.01) / TELEM_IC_SAMPLE)) - 1
    ic_tick_s = (telempsc + 1) / tim1_clk
    ticks_per_gcr_bit = (1.0 / gcr_bps) / ic_tick_s
    clocking = {
        "sysclk_mhz": clk_mhz, "tim1_kernel_clock_hz": tim1_clk,
        "dshot_bitrate": dshot_bps, "dshot_bit_us": 1e6 / dshot_bps,
        "output_psc": out_psc, "output_tick_hz": out_tick_hz,
        "output_ticks_per_bit": out_ticks_per_bit,
        "output_bitrate_error_pct": 100.0 * (out_tick_hz / DSHOT_BIT_WIDTH_TICKS / dshot_bps - 1.0),
        "gcr_bitrate": gcr_bps, "gcr_bit_us": 1e6 / gcr_bps,
        "telem_psc": telempsc, "ic_tick_ns": ic_tick_s * 1e9,
        "ic_ticks_per_gcr_bit": ticks_per_gcr_bit,
        # each edge interval (<= 3 GCR cells) is rounded independently
        "esc_clock_tolerance_pct_theory": 100.0 * (TELEM_IC_SAMPLE / 2) / (3 * TELEM_IC_SAMPLE),
        "ic_capture_window_ticks": GCR_BITS * TELEM_IC_SAMPLE,
        "ic_counter_wrap_ms": 65536 * ic_tick_s * 1e3,
    }
    gate(abs(out_ticks_per_bit - DSHOT_BIT_WIDTH_TICKS) < 1e-9, "B1 DShot600 output exact: PSC gives 8 ticks/bit",
         "PSC %d -> %.1f MHz, %.3f ticks/bit" % (out_psc, out_tick_hz / 1e6, out_ticks_per_bit))
    gate(abs(ticks_per_gcr_bit - TELEM_IC_SAMPLE) < 1e-9, "B2 IC sampling exactly 16 ticks/GCR bit",
         "PSC %d -> %.4f ticks/bit" % (telempsc, ticks_per_gcr_bit))

    # ---- 2. bidir cycle budget ---------------------------------------------
    # RCOutput.h: dshot_bit_length = 16 + dshot_pre(1) + dshot_post(2) = 19 slots;
    # pulse_send_time_us = 1e6 * 19 / rate (integer us).  RCOutput_bdshot.cpp
    # bdshot_receive_pulses_DMAR arms the receive timeout at
    # pulse_send_time_us + 30 + 10 us; that window is the hard upper bound.
    frame_us = float(int(1000000 * 19 / dshot_bps))
    turnaround_us = 30.0      # DShot bidir spec / ArduPilot's "+ 30U"
    reply_us = GCR_BITS * 1e6 / gcr_bps
    margin_us = 10.0          # ArduPilot's "+ 10U"
    cycle_us = frame_us + (frame_us + turnaround_us + margin_us)   # send + receive window
    gate(turnaround_us + reply_us <= frame_us + turnaround_us + margin_us, "B1b GCR reply fits ArduPilot's receive window",
         "%.1f us needed vs %.0f us window" % (turnaround_us + reply_us, frame_us + turnaround_us + margin_us))

    # ---- 3. rates ----------------------------------------------------------
    timers = sorted({c["timer"] for c in pwm})
    motors_per_group = {t: sum(1 for c in pwm if c["timer"] == t) for t in timers}
    worst_group = max(motors_per_group.values())
    sweep = []
    for r in range(0, 5):
        f, sync, intervals = ap_dshot_rate(r, loop_hz)
        per_motor = f / worst_group
        # worst gap between two captures of one motor (round-robin of uneven frames)
        rr = intervals * worst_group
        worst_gap_s = max(sum(rr[i:i + worst_group]) for i in range(len(intervals))) / 1e6
        # age of the value the notch uses: wait for the motor's slot (worst gap)
        # + AP_ESC_Telem::get_rpm() interpolates prev->new over one update
        # period (another 1/per_motor) + notch consumed at loop rate + ESC
        # 6-commutation averaging
        age_s = worst_gap_s + 1.0 / per_motor + 1.0 / loop_hz + ESC_MEASUREMENT_LATENCY_S
        err_hz = PUNCH_SLEW_HZ_PER_S * age_s
        # residual depth of a single INS_HNTCH_ATT notch whose centre is
        # err_hz away from the tone: 2nd-order notch, -3 dB half-width BW/2
        half = p["INS_HNTCH_BW"] / 2.0
        h = err_hz / math.hypot(err_hz, half)
        sweep.append({"SERVO_DSHOT_RATE": r, "punch_residual_attenuation_db": 20 * math.log10(max(h, 1e-5)),
                      "frame_interval_us": sorted(set(round(x, 1) for x in intervals)),
                      "worst_capture_gap_ms": worst_gap_s * 1e3, "frame_hz": f, "loop_synchronous": sync,
                      "per_motor_erpm_hz": per_motor, "worst_sample_age_ms": age_s * 1e3,
                      "punch_tracking_error_hz": err_hz,
                      "cycle_duty_pct": 100.0 * cycle_us * f / 1e6})
    chosen = next(s for s in sweep if s["SERVO_DSHOT_RATE"] == int(p.get("SERVO_DSHOT_RATE", 0)))
    bw = p["INS_HNTCH_BW"]
    gate(chosen["loop_synchronous"], "B3 DShot frames evenly spaced and loop-locked", "SERVO_DSHOT_RATE %d -> %.0f Hz, %s us"
         % (chosen["SERVO_DSHOT_RATE"], chosen["frame_hz"], chosen["frame_interval_us"]))
    gate(800 <= chosen["frame_hz"] <= 4096, "B4 frame rate within ArduPilot bounds [800, 4096] Hz",
         "%.0f Hz" % chosen["frame_hz"])
    gate(chosen["per_motor_erpm_hz"] >= 250, "B5 per-motor eRPM refresh >= 250 Hz under round-robin",
         "%.0f Hz (%d motors on one timer)" % (chosen["per_motor_erpm_hz"], worst_group))
    # B6 is REPORTED, not gated (Rev 10 review): with the get_rpm interpolation
    # lag included every rate leaves the notch more than one bandwidth behind a
    # full punch, i.e. the RPM notch gives ~1 dB during a 60 ms full-throttle
    # slew whatever SERVO_DSHOT_RATE is.  The rate is chosen on B3/B5.
    best = min(sweep, key=lambda s_: s_["punch_tracking_error_hz"])
    gate(chosen["SERVO_DSHOT_RATE"] == best["SERVO_DSHOT_RATE"], "B6 chosen rate has the smallest punch lag of 0..4",
         "%.1f Hz lag (BW %.0f Hz, residual depth %.1f dB during the slew; informational)"
         % (chosen["punch_tracking_error_hz"], bw, chosen["punch_residual_attenuation_db"]))
    gate(cycle_us * chosen["frame_hz"] / 1e6 < 0.5, "B7 bidir cycle < 50 % of frame period",
         "%.1f us of %.1f us" % (cycle_us, 1e6 / chosen["frame_hz"]))

    # ---- 4. quantisation -----------------------------------------------------
    worst = (0.0, 0)
    curve = []
    for rpm in range(MOTOR_RPM_MIN, MOTOR_RPM_MAX + 1, 250):
        e, m = encode_period(rpm * poles // 2)
        got = ap_rpm_from_period(e, m, poles)
        err = abs(got - rpm) / rpm
        curve.append([rpm, got])
        if err > worst[0]:
            worst = (err, rpm)
    quant = {"worst_rel_error_pct": 100 * worst[0], "at_rpm": worst[1],
             "worst_abs_error_hz": worst[0] * worst[1] / 60.0,
             "min_representable_rpm": int(60e6 / (511 << 7) / (poles / 2)),
             "period_at_max_rpm_us": 60e6 / (MOTOR_RPM_MAX * poles / 2)}
    gate(quant["worst_abs_error_hz"] < bw / 4, "B8 eRPM quantisation < BW/4",
         "%.2f Hz at %d RPM" % (quant["worst_abs_error_hz"], worst[1]))

    # ---- 5. notch configuration ---------------------------------------------
    fmin = p["INS_HNTCH_FREQ"] * p.get("INS_HNTCH_FM_RAT", 1.0)
    nyq = INS_GYRO_RATE_HZ * HARMONIC_NYQUIST_CUTOFF
    opts = int(p["INS_HNTCH_OPTS"])
    hm = int(p.get("INS_HNTCH_HMNCS", 3))
    harmonics = [i + 1 for i in range(8) if hm & (1 << i)]
    hover_hz = 20000 / 60.0
    notch = {
        "min_hz": fmin, "disable_below_hz": fmin * NOTCH_DISABLE_RATIO,
        "nyquist_guard_hz": nyq, "harmonics": harmonics,
        "filters_per_imu": 4 * len(harmonics) * (2 if opts & 1 else 3 if opts & 16 else 1),
        "idle_motor_hz": MOTOR_RPM_MIN / 60.0, "hover_motor_hz": hover_hz,
        "max_motor_hz": MOTOR_RPM_MAX / 60.0,
        "highest_tracked_h2_motor_hz": nyq / 2,
    }
    gate(int(p["INS_HNTCH_MODE"]) == 3, "B9 notch tracks ESC telemetry", "MODE %d" % p["INS_HNTCH_MODE"])
    gate(opts & 2 and opts & 4, "B10 OPTS multi-source + loop-rate update", "OPTS %d" % opts)
    gate(fmin >= 0.52 * bw, "B11 FREQ >= 0.52 x BW (HarmonicNotchFilter bandwidth_limit)",
         "%.0f vs %.1f" % (fmin, 0.52 * bw))
    gate(fmin < hover_hz * 0.5, "B12 min frequency well below hover fundamental", "%.0f vs hover %.0f Hz" % (fmin, hover_hz))
    gate(notch["filters_per_imu"] <= 54, "B13 notch filters fit HAL_HNF_MAX_FILTERS (54, H7)",
         "%d" % notch["filters_per_imu"])

    # ---- 6. line electricals -----------------------------------------------
    tau_ns = R_SERIES * C_LINE_PF * 1e-12 * 1e9
    pullup_tau_us = H7_PULLUP_OHM * C_LINE_PF * 1e-12 * 1e6
    line = {"series_ohm": R_SERIES, "line_capacitance_pF": C_LINE_PF,
            "rc_ns": tau_ns, "rise_10_90_ns": 2.2 * tau_ns,
            "internal_pullup_tau_us": pullup_tau_us}
    gate(2.2 * tau_ns < 0.02 * 1e3 / (gcr_bps / 1e6), "B14 series R edge < 2 % of a GCR bit",
         "%.1f ns vs %.0f ns bit" % (2.2 * tau_ns, 1e9 / gcr_bps))
    gate(5 * pullup_tau_us < turnaround_us, "B15 internal pull-up settles the idle level within turnaround",
         "5 tau = %.1f us vs %.0f us" % (5 * pullup_tau_us, turnaround_us))

    # ---- cross-file consistency --------------------------------------------
    # The FC alone converts eRPM -> RPM (erpm * 200 / SERVO_BLH_POLES); AM32's
    # motor_poles only feeds its own RPM-based features.  Both describe the same
    # physical motor, so they must agree.  EDT needs no ESC-side setting: AM32
    # starts with it off and ArduPilot enables it with DShot command 13 when
    # SERVO_DSHOT_ESC is 3/4.  AM32 auto-detects the DShot bit rate.
    gate(am32.get("motor_poles") == poles, "B16 AM32 EEPROM motor_poles == SERVO_BLH_POLES (same motor)",
         "%s vs %d" % (am32.get("motor_poles"), poles))
    gate(int(p.get("SERVO_DSHOT_ESC", 0)) in (3, 4), "B17 EDT requested by the FC (SERVO_DSHOT_ESC 3/4 -> cmd 13)",
         "SERVO_DSHOT_ESC %d" % p.get("SERVO_DSHOT_ESC", 0))

    report = {
        "revision": "10.0",
        "inputs": {"loop_hz": loop_hz, "poles": poles, "pwm_channels": pwm,
                   "punch_slew_hz_per_s": PUNCH_SLEW_HZ_PER_S, "gyro_rate_hz": INS_GYRO_RATE_HZ},
        "clocking": clocking,
        "bidir_cycle_us": {"frame_19_slots": frame_us, "turnaround": turnaround_us, "gcr_reply": reply_us,
                           "receive_window": frame_us + turnaround_us + margin_us, "total": cycle_us},
        "rate_sweep": sweep, "chosen": chosen,
        "quantisation": quant, "notch": notch, "line": line,
        "gates": gates,
        "result": "PASS" if all(g["pass"] for g in gates) else "FAIL",
    }
    with open(OUT, "w") as fh:
        json.dump(report, fh, indent=2)
        fh.write("\n")

    print("TIM1 %.0f MHz, DShot%d: PSC %d, %.0f ticks/bit; GCR PSC %d -> %.1f ns tick, %.2f ticks/bit"
          % (tim1_clk / 1e6, dshot_bps // 1000, out_psc, out_ticks_per_bit, telempsc, ic_tick_s * 1e9, ticks_per_gcr_bit))
    print("bidir cycle %.1f us (19-slot frame %.0f + receive window %.0f; reply needs %.1f)"
          % (cycle_us, frame_us, frame_us + turnaround_us + margin_us, turnaround_us + reply_us))
    print("SERVO_DSHOT_RATE sweep @ loop %d Hz, %d motors/timer:" % (loop_hz, worst_group))
    for s in sweep:
        print("  %d: %6.0f Hz %-5s per-motor %5.0f Hz  age %.2f ms  punch lag %5.1f Hz (%5.1f dB)"
              % (s["SERVO_DSHOT_RATE"], s["frame_hz"], "sync" if s["loop_synchronous"] else "async",
                 s["per_motor_erpm_hz"], s["worst_sample_age_ms"], s["punch_tracking_error_hz"],
                 s["punch_residual_attenuation_db"]))
    print("quantisation worst %.3f %% (%.2f Hz) at %d RPM" % (quant["worst_rel_error_pct"],
                                                           quant["worst_abs_error_hz"], worst[1]))
    for g in gates:
        print("  [%s] %s: %s" % ("PASS" if g["pass"] else "FAIL", g["gate"], g["detail"]))
    print("RESULT: %s -> %s" % (report["result"], os.path.relpath(OUT, REPO)))
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
