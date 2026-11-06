#!/usr/bin/env python3
"""
Nano H743 AIO - AT7456E analog OSD on SPI2, digital VTX connector and the
switchable 9 V / 12 V 2 A VTX BEC (Revision 13.0)

Pure Python (no numpy).  Writes analysis/osd_vtx_bec_report.json and exits 1 if
any gate fails.  Findings against upstream ArduPilot (behaviour this board has
to work around, not a defect of this design) are recorded under "findings" and
do not fail the run; "warnings" are design margins worth a bench check.

Sections
  1. SPI2 clock (gates C*): 24 MHz HSE -> PLL1 (M 3, N 120, Q 12 at
     MCU_CLOCKRATE_MHZ 480) -> SPI123SEL = PLL1_Q = 80 MHz, re-read from
     ArduPilot's stm32h7_mcuconf.h; the SCLK divisor search of
     SPIDevice.cpp derive_freq_flag_bus() reproduced for the hwdef
     "SPIDEV osd ... 8*MHZ 8*MHZ" (-> MBR 3, /16, 5.000 MHz) and for the stock
     10*MHZ (-> 10.000 MHz, zero tCP margin); AT7456E tCP/tCH/tCL/tDS and the
     MISO (SDOUT) setup at the H7 margins.
  2. OSD SPI bandwidth (gates B*): AP_OSD_MAX7456 transfer_frame() byte budget
     (512-byte buffer, stop at 480), worst flush, DMA/SCLK time per 100 ms OSD
     thread period, full-screen redraw, boot font check bursts, do_transfer()
     timeout.
  3. NVM read timing (gate N1): check_font_char() issues CMM = 0x50 and reads
     CMDO a few bytes later in the same burst; the AT7456E needs >= 22 us
     (30 us typ, DS p.36).  The upstream sequence is evaluated and recorded as
     a finding; the gate requires ardupilot/patches/0003-osd-at7456e-nvm-read-
     delay.patch to exist, to `git apply --check` against the unmodified
     upstream file of the given tree, and its patched burst layout to wait
     >= 40 us between CMM and the first CMDO (and after VM0[3] = 0).
  4. Video network (gates V*): VIN 75 R termination + 100 nF clamp coupling,
     input range, clamp DC level, VOUT gain / swing / headroom on +3V3_OSD,
     COUT = CSAG = 22 uF sag network vs DS Table 2, 75 R back-termination.
  5. TPS62933 BEC (gates R*, Q*, D*, L*, K*, T*): FB divider 9 / 12 V nominal
     and all corners (Vref 784..816 mV, 1 % resistors, FB leakage, Q1 Rds_on /
     IDSS), OVP on the 12 -> 9 V step vs UVP on 9 -> 12 V, 2N7002 gate drive,
     dropout table for 2S/3S/4S (500 kHz Dmax 0.93 / foldback 0.98), the
     12 V interlock thresholds, inductor ripple / peak / Isat vs current limit,
     effective Cout vs DS Table 10-2, soft start, VIN rating, IC junction
     temperature.
  6. Digital VTX connector (gates J*): per-contact current of J1 (JST SH 1 A)
     for DJI O3 / Vista / Walksnail / HDZero at 9 and 12 V; >1 A loads -> J2.
  7. BEC sense ADC (gates S*): R40/R41 = 11.0 vs BATT2_VOLT_MULT, ArduPilot
     3.3 V full scale, resolution, filter, soft-start vs supervisor delay.
  8. Boot state (gates P*): reset / bootloader / firmware early init /
     AP_Relay::init() levels of VTX_PWR (EN) and VTX_12V_SEL (Q1 gate).
  9. ArduPilot evidence (gates F*) and hwdef / defaults consistency (gates A*):
     regexes against SPIDevice.cpp, stm32h7_mcuconf.h, hal_lld_type1.h,
     AP_OSD_MAX7456.cpp/.h, AP_OSD.cpp, AP_Relay*.cpp, AnalogIn.cpp,
     STM32H743xx.py, Scheduler.cpp; with --ardupilot the board (and its
     bootloader) is generated with ArduPilot's chibios_hwdef.py (via
     tools/validate_dma.py stage()/run_generator()) and hwdef.h is checked for
     the SPIDEV osd speed, GPIO 81/82 + init levels, UART7 NODMA and ADC
     channel 8.

Usage: python3 analysis/osd_vtx_bec_analysis.py [--ardupilot /path/to/ardupilot]
       (or set ARDUPILOT_ROOT)
"""
import argparse
import itertools
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PATCH_REL = "ardupilot/patches/0003-osd-at7456e-nvm-read-delay.patch"
OSD_CPP = "libraries/AP_OSD/AP_OSD_MAX7456.cpp"

# ----------------------------------------------------------------------------
# Assumptions (all copied into the report).  [AT] = AT7456E-S402-V1.1
# (LCSC C82351 datasheet, /tmp/at7456dl), [TI] = TPS62933 SLUSEA4D Aug 2022,
# [CJ] = JSCJ 2N7002 datasheet (LCSC C8545, J Sep 2016), [AP] = ArduPilot tree.
# ----------------------------------------------------------------------------
A = {
    "rail_3v3": {"nom": 3.3, "tol_frac": 0.05,
                 "note": "+3V3 rail +/-5 % (repo convention, analysis/baro_i2c_analysis.py A['vdd'])"},
    "clock": {"hse_hz": 24e6, "mcu_clockrate_mhz": 480,
              "pll1_m": 3, "pll1_n": 120, "pll1_q": 12, "spi123sel": "STM32_SPI123SEL_PLL1_Q_CK",
              "source": "[AP] stm32h7_mcuconf.h 24 MHz block + HAL_CUSTOM_MCU_CLOCKRATE 480000000 "
                        "(chibios_hwdef.py emits it from MCU_CLOCKRATE_MHZ 480)"},
    "spi": {"hwdef_lowspeed_hz": 8e6, "hwdef_highspeed_hz": 8e6, "stock_osd_hz": 10e6,
            "mbr_max": 7,
            "h7_miso_setup_ns": 5.0,     # DS12110 SPI master tsu(MI) (few ns); 5 ns taken as bound
            "mcu_pin_pF": 5.0,           # DS12110 C_IO
            "trace_pF": 3.0,             # ~20 mm L1 microstrip, 0.15 pF/mm
            "r_series_ohm": 33.0,        # R30..R33
            "note": "SPI2 duty is exactly 50 % (even prescaler of the kernel clock, RM0433 SPI MBR)"},
    "at7456e": {   # [AT] p.2-4 electrical tables, p.15 serial interface, p.36 command timing
        "vdd_min": 3.15, "vdd_max": 5.25,                         # p.2
        "i_total_max_mA": 2.5 + 60.0 + 10.0,                      # p.2 IAVDD+IDVDD+IPVDD max (5 V cond.)
        "sclk_max_hz": 10e6,                                      # p.15
        "tCP_ns": 100.0, "tCH_ns": 40.0, "tCL_ns": 40.0,          # p.4
        "tCSS0_ns": 30.0, "tCSS1_ns": 30.0, "tCSW_ns": 100.0,     # p.4
        "tDS_ns": 30.0, "tDH_ns": 0.0, "tDO1_ns": 25.0,           # p.4 (tDO1 SDOUT valid, 20 pF;
                                                                  # taken as max delay after SCLK fall)
        "sclk_duty": [0.40, 0.60],                                # p.6 SCLK pin description
        "nvm_read_us": {"max7456_min": 0.5, "min": 22.0, "typ": 30.0},     # p.36 table, CMM = 0x50
        "osd_disable_us": {"max7456_min": 0.0, "min": 18.0, "typ": 30.0},  # p.36 table, VM0[3] = 0
        "sw_reset_us": {"min": 28.0, "typ": 40.0},                # p.36 table, VM0[1]
        "fixed_delay_rule": "p.36 note 3: when fixed delays are used, design to the typical value",
        "busy_ignore_rule": "p.29 CMM register: NVM read/write commands are ignored while VM0[3] = 1 or "
                            "STAT[5] = 1; p.34 read procedure waits 30 us after VM0[3] = 0",
        "nvm_endurance_stores": 100000,                           # p.2
        "vin_range_vpp": [0.5, 1.2], "vin_sync_detect_vpp": [0.5, 2.0],    # p.3
        "vin_clamp_sync_tip_V": 0.56,                             # p.10 input clamp
        "vin_coupling_uF_required": 0.1,                          # p.10 input clamp
        "clamp_settling_lines": 32,                               # p.3
        "gain": [1.89, 2.0, 2.11],                                # p.3
        "vout_swing_min_vpp": 2.4, "vout_sync_tip_V": 0.7,        # p.3
        "zout_ohm": 0.22, "ltd_max_pct": 0.5,                     # p.3
        "sag_table": [(470, None, 0.2), (100, None, 0.4), (100, 22, 0.3),
                      (47, 47, 0.3), (22, 22, 0.4), (10, 10, 0.6)],  # p.15 Table 2 (uF, uF, % typ)
    },
    "video": {"source_ohm": 75.0, "camera_vpp": 1.0, "load_ohm": 75.0,
              "r27_ohm": 75.0, "r28_ohm": 75.0, "c31_uF": 0.1,
              "cout_uF": 22.0, "csag_uF": 22.0, "cout_tol": 0.20,
              "cout_bias_retention": 0.75,
              "cout_bias_source": "assumed: 0805 10 V X5R 22 uF at ~1.3 V DC (VOUT black level) keeps "
                                  "~75 % (typical Samsung CL21A226M DC-bias curve); UNVERIFIED",
              "line_us": {"PAL": 64.0, "NTSC": 63.556}},
    "bec": {   # U6 TPS62933DRLR [TI]
        "vref_V": [0.784, 0.800, 0.816],                          # 8.5 VFB, TJ -40..150 C
        "ifb_max_A": 0.15e-6,                                     # 8.5 IFB
        "r_tol": 0.01,
        "rtop": 102e3, "rbot": 10.0e3, "rsw": 26.7e3,             # R34, R35, R36
        "r_gate_series": 1e3, "r_gate_pd": 100e3,                 # R37, R38
        "fsw_hz": [450e3, 500e3, 550e3],                          # 8.5, RT floating
        "toff_min_s": 140e-9, "ton_max_s": 7e-6,                  # 8.5 (typ)
        "dmax_fixed": None,                                       # computed: 1 - tOFF_MIN*fSW (Eq. 8)
        "dmax_foldback": 0.98,                                    # Features / 9.3.9
        "rds_hs_ohm": 0.076, "rds_ls_ohm": 0.032,                 # 8.5 at 25 C
        "rds_hot_factor": 1.4,                                    # Fig 8-3/8-4 trend to ~125 C (estimate)
        "ihs_limit_A": [4.2, 5.0, 5.8],                           # 8.5
        "ovp_frac": [1.12, 1.15, 1.18], "uvp_frac": 0.65,         # 8.5 / 9.3.11 / 9.3.12
        "iss_A": [4.5e-6, 5.5e-6, 6.5e-6], "css_F": 33e-9,        # 8.5 / Eq. 6, C38
        "css_min_F": 6.8e-9,                                      # Table 7-1 SS pin
        "en_rise_V": [1.21, 1.28], "en_fall_V": [1.1, 1.17],      # 8.5
        "en_ip_A": 0.7e-6, "en_abs_max_V": 6.0, "en_rec_max_V": 5.5,  # 8.5, 8.1, 8.3
        "r_en_series": 1e3,                                       # R39
        "vin_rec_max_V": 30.0, "vin_abs_max_V": 32.0,             # 8.3 / 8.1
        "iout_rated_A": 3.0, "iout_design_A": 2.0,
        "rth_ja_evm": 60.2, "psi_jb": 19.2, "tj_max_C": 150.0,    # 8.4 / 8.3
        "tj_margin_C": 20.0,
        "sw_edge_ns": 5.0,                                        # assumed tr = tf (not in DS)
        "iq_A": 12e-6,
        "table_10_2_12V": {"L_uH": 12, "cout_typ_uF": 15, "cout_min_uF": 10},   # Table 10-2
        "k_ripple_window": [0.2, 0.6], "ripple_min_frac_of_3A": 0.10,          # 10.2.2.7
        "inductor": {"part": "DMBJ PNLS6045-100M (LCSC C2849537)", "L_uH": 10.0, "L_tol": 0.20,
                     "isat_A": 4.5, "dcr_ohm": 0.057, "dcr_hot_ohm": 0.065,
                     "source": "LCSC C2849537 listing (Isat 4.5 A, DCR 57 mOhm), /tmp/rev13_bec_vtx.md"},
        "cout": {"n": 3, "uF": 22.0, "part": "Samsung CL31A226KAHNNNE 25 V X5R 1206 (C12891)",
                 "retention_at_12V": 0.40, "retention_at_9V": 0.50, "tol": 0.10,
                 "source": "assumed DC-bias retention (1206 25 V X5R): 40 % at 12 V, 50 % at 9 V; "
                           "UNVERIFIED, conservative vs Samsung typical curves"},
        "cin": {"n": 2, "uF": 10.0, "retention_at_16V8": 0.55,
                "part": "Samsung CL31A106KBHNNNE 50 V X5R 1206 (C13585)",
                "source": "assumed 55 % at 16.8 V on a 50 V part; UNVERIFIED"},
    },
    "q1": {"part": "2N7002 JSCJ (LCSC C8545)",                    # [CJ] electrical table
           "vth_V": [1.0, 1.6, 2.5], "idss_25C_A": 80e-9, "rds_5V_max_ohm": 7.0,
           "coss_pF": 25.0,
           "idss_hot_rule": "x2 per 10 C above 25 C (engineering rule, not in [CJ]); evaluated at 85 C",
           "t_hot_C": 85.0},
    "mcu_gpio": {"voh_drop_V": 0.0,                               # uA loads: VOH ~ VDD
                 "ilkg_A": 0.25e-6,
                 "ilkg_source": "STM32H743 DS12110 I/O static characteristics, FT input leakage "
                                "(sub-uA class); 0.25 uA taken as bound - UNVERIFIED",
                 "reset_mode": "RM0433 GPIOx_MODER reset value 0xFFFF FFFF for ports C..K = analog "
                               "(high-Z, no pull) until firmware board init"},
    "battery": {"cell_V": {"full": 4.2, "nominal": 3.7, "empty": 3.2},
                "packs": [2, 3, 4], "esc_overshoot_frac": 0.30,
                "overshoot_source": "analysis/baro_i2c_analysis.py A['esc'] overshoot_frac"},
    "interlock": {"allow_12V_V": 13.6, "allow_hold_s": 2.0, "drop_12V_V": 13.0, "drop_hold_s": 0.3,
                  "source": "/tmp/rev13_spec.md D.2 (ardupilot/scripts/vtx_bec.lua policy)"},
    "vtx": {   # /tmp/rev13_bec_vtx.md section 3
        "DJI_O3": {"P_W": 16.0, "vin": [7.4, 26.4]},
        "Caddx_Vista": {"P_W": 9.0, "vin": [7.4, 26.4]},
        "Walksnail_Avatar_HD_Pro": {"P_W": 4.5, "vin": [6.0, 25.2]},
        "HDZero_Race_V3": {"P_W": 5.8, "vin": [4.0, 12.0]},
    },
    "connector": {"J1": "JST SM06B-SRSS-TB (C160405)", "contact_A": 1.0,
                  "pinout": {1: "+VTX", 2: "GND", 3: "VTX_TXD", 4: "VTX_RXD", 5: "GND", 6: "VTX_SBUS"},
                  "J2": "2-pad +VTX/GND solder pair for > 1 A loads", "J2_A": 3.0,
                  "source": "JST SH series page (1 A AC/DC, AWG 28); J2 = bare pads + AWG 22..24 lead"},
    "sense": {"r40": 10e3, "r41": 1e3, "c43_F": 100e-9, "tol": 0.01, "adc_bits": 12,
              "ap_fullscale_V": 3.3, "fw_window_frac": 0.06, "fw_settle_ms": 10.0,
              "fw_source": "/tmp/rev13_spec.md D.3 (+/-6 % window after 10 ms soft start)"},
    "osd_driver": {"thread_period_ms": 100.0, "spi_buffer": 512, "stop_at": 480,
                   "pal_rows": 16, "ntsc_rows": 13, "cols": 30, "nvm_chars": 256, "nvm_bytes": 54,
                   "nvm_write_poll_ms": 15.0, "required_delay_us": 40.0},
}

GATES = []
WARNS = []
FINDINGS = []


def gate(name, ok, detail):
    GATES.append({"gate": name, "pass": bool(ok), "detail": detail})
    print("  [%s] %s: %s" % ("PASS" if ok else "FAIL", name, detail))
    return ok


def warn(name, detail):
    WARNS.append({"warn": name, "detail": detail})
    print("  [WARN] %s: %s" % (name, detail))


def finding(name, detail):
    FINDINGS.append({"finding": name, "detail": detail})
    print("  [FINDING] %s: %s" % (name, detail))


def par(*rs):
    return 1.0 / sum(1.0 / r for r in rs)


def read(p):
    with open(p) as fh:
        return fh.read()


def line_of(txt, pos):
    return txt.count("\n", 0, pos) + 1


def cite(ap_root, rel, txt, pattern, flags=0):
    """Return 'rel:line' of the first regex match, or None."""
    m = re.search(pattern, txt, flags)
    return ("%s:%d" % (rel, line_of(txt, m.start()))) if m else None


# ----------------------------------------------------------------------------
# 1. SPI2 clock
# ----------------------------------------------------------------------------
def spi_kernel_clock(ap_root):
    c = A["clock"]
    m, n, q, sel = c["pll1_m"], c["pll1_n"], c["pll1_q"], c["spi123sel"]
    src = "built-in (" + c["source"] + ")"
    cites = {}
    if ap_root:
        rel = "libraries/AP_HAL_ChibiOS/hwdef/common/stm32h7_mcuconf.h"
        txt = read(os.path.join(ap_root, rel))
        # PLL1_DIVM in the first "#elif STM32_HSECLK == 24000000U" block,
        # DIVN/DIVQ in the second one under HAL_CUSTOM_MCU_CLOCKRATE == 480000000
        blk = txt.split("#elif STM32_HSECLK == 24000000U")
        m = int(re.search(r"STM32_PLL1_DIVM_VALUE\s+(\d+)", blk[1]).group(1))
        b2 = blk[2].split("#elif STM32_HSECLK")[0]
        mm = re.search(r"HAL_CUSTOM_MCU_CLOCKRATE == 480000000\s*\n#define STM32_PLL1_DIVN_VALUE\s+(\d+)\s*\n"
                       r"#define STM32_PLL1_DIVQ_VALUE\s+(\d+)", b2)
        n, q = int(mm.group(1)), int(mm.group(2))
        sel = re.search(r"#define STM32_SPI123SEL\s+(\S+)", txt).group(1)
        cites["SPI123SEL"] = cite(ap_root, rel, txt, r"#define STM32_SPI123SEL\s")
        sep = "#elif STM32_HSECLK == 24000000U"
        off2 = len(blk[0]) + len(sep) + len(blk[1]) + len(sep)
        cites["PLL1_480"] = "%s:%d" % (rel, line_of(txt, off2 + blk[2].index(mm.group(0))) + 1)
        rel2 = "modules/ChibiOS/os/hal/ports/STM32/STM32H7xx/hal_lld_type1.h"
        p2 = os.path.join(ap_root, rel2)
        if os.path.exists(p2):
            t2 = read(p2)
            cites["SPI2CLK=PLL1_Q"] = cite(ap_root, rel2, t2, r"#define STM32_SPI2CLK\s+STM32_PLL1_Q_CK")
        src = rel
    f = c["hse_hz"] / m * n / q
    return {"hse_hz": c["hse_hz"], "pll1_m": m, "pll1_n": n, "pll1_q": q, "spi123sel": sel,
            "f_spi123_hz": f, "source": src, "cites": cites}


def derive_freq_flag_bus(kernel_hz, request_hz, mbr_max=7):
    """Reproduces AP_HAL_ChibiOS SPIDevice::derive_freq_flag_bus(): start at
    bus_clocks[busid-1] / 2 and halve while above the request, at most 7 times;
    H7 CFG1.MBR = i (divisor 2^(i+1))."""
    f = int(kernel_hz) // 2
    i = 0
    while f > request_hz and i < mbr_max:
        f >>= 1
        i += 1
    return {"request_hz": request_hz, "mbr": i, "divisor": 2 ** (i + 1), "sclk_hz": float(f)}


def spi_timing(sclk_hz):
    at, s = A["at7456e"], A["spi"]
    tcp = 1e9 / sclk_hz
    th = tl = tcp / 2.0                                    # even prescaler -> 50 % duty
    rc = s["r_series_ohm"] * (s["mcu_pin_pF"] + s["trace_pF"]) * 1e-3   # ns
    # mode 0: AT7456E shifts SDOUT on SCLK fall (<= tDO1), H7 samples on the next rise
    miso_setup = tl - at["tDO1_ns"] - 2.2 * rc - s["h7_miso_setup_ns"]
    # MCU MOSI changes on fall, AT7456E samples on rise: available setup ~ tCL - RC
    mosi_setup = tl - 2.2 * rc
    return {"sclk_hz": sclk_hz, "tCP_ns": tcp, "tCH_ns": th, "tCL_ns": tl,
            "margin_tCP_ns": tcp - at["tCP_ns"], "margin_tCH_ns": th - at["tCH_ns"],
            "margin_tCL_ns": tl - at["tCL_ns"],
            "mosi_setup_ns": mosi_setup, "margin_tDS_ns": mosi_setup - at["tDS_ns"],
            "miso_setup_margin_ns": miso_setup, "series_rc_ns": rc,
            "within_10MHz": sclk_hz <= at["sclk_max_hz"]}


# ----------------------------------------------------------------------------
# 2. OSD bandwidth
# ----------------------------------------------------------------------------
def frame_bytes(dirty, rows, cols, buf=512):
    """Byte-exact replay of AP_OSD_MAX7456::transfer_frame() for a dirty map.
    Returns (bytes, chars_sent)."""
    off, prev, auto, sent = 0, 65534, False, 0

    def isd(x, y):
        return 0 <= y < rows and 0 <= x < cols and dirty[y][x]
    for y in range(rows):
        for x in range(cols):
            if not dirty[y][x]:
                continue
            if off >= buf - 32:
                break
            pos = y * cols + x
            if prev + 1 != pos and auto:
                off += 4
                auto = False
            if not auto:
                off += 4
            if not auto and isd(x + 1, y):
                off += 2
                auto = True
            off += 2
            sent += 1
            prev = pos
    if auto:
        off += 4
    return off, sent


def osd_bandwidth(sclk_hz):
    d = A["osd_driver"]
    rows, cols = d["pal_rows"], d["cols"]
    full = [[True] * cols for _ in range(rows)]
    checker = [[(x + y) % 2 == 0 for x in range(cols)] for y in range(rows)]
    single = [[x == 0 and y == 0 for x in range(cols)] for y in range(rows)]
    cases = {}
    worst = 0
    for name, dm in (("full_screen", full), ("checkerboard", checker), ("one_char", single)):
        remaining = [row[:] for row in dm]
        flushes, first = 0, None
        while any(any(r) for r in remaining):
            b, sent = frame_bytes(remaining, rows, cols, d["spi_buffer"])
            first = first or (b, sent)
            worst = max(worst, b)
            # mark the first `sent` dirty cells clean (row-major, as the driver does)
            k = sent
            for y in range(rows):
                for x in range(cols):
                    if remaining[y][x] and k > 0:
                        remaining[y][x] = False
                        k -= 1
            flushes += 1
        cases[name] = {"dirty_chars": sum(sum(1 for v in r if v) for r in dm),
                       "first_flush_bytes": first[0], "first_flush_chars": first[1],
                       "flushes_to_complete": flushes,
                       "time_to_complete_ms": flushes * d["thread_period_ms"]}
    # exhaustive bound: a character adds at most 4 (leave auto) + 4 (DMAH/L) + 2 (DMM) + 2 (DMDI)
    bound = (d["spi_buffer"] - 32 - 1) + 12 + 4
    t_flush_us = worst * 8 / sclk_hz * 1e6
    font_burst = 3 * 2 + d["nvm_bytes"] * 4
    write_burst = 2 * 2 + d["nvm_bytes"] * 4 + 2
    return {"cases": cases, "worst_flush_bytes_replay": worst, "worst_flush_bytes_bound": bound,
            "buffer_bytes": d["spi_buffer"],
            "flush_sclk_time_us": t_flush_us,
            "duty_per_period_pct": 100 * t_flush_us / (d["thread_period_ms"] * 1e3),
            "do_transfer_timeout_us": 20000 + bound * 32,
            "font_check_burst_bytes": font_burst,
            "font_check_burst_us": font_burst * 8 / sclk_hz * 1e6,
            "font_write_burst_bytes": write_burst,
            "boot_font_check_ms_no_rewrite": d["nvm_chars"] * font_burst * 8 / sclk_hz * 1e3,
            "boot_font_rewrite_all_s_min": d["nvm_chars"] * d["nvm_write_poll_ms"] / 1e3}


# ----------------------------------------------------------------------------
# 3. NVM read timing
# ----------------------------------------------------------------------------
def upstream_nvm_timing(sclk_hz):
    """check_font_char() single burst: [VM0,0][CMAH,c][CMM,0x50] then 54 x [CMAL,x][CMDO,FF].
    CMM latches at the end of byte 5.  Read x is requested when its CMDO address
    byte (index 8+4x) ends and clocked out during byte 9+4x.  Zero inter-byte gap
    (H7 SPI + DMA) is the worst case."""
    at = A["at7456e"]
    tb = 8 / sclk_hz * 1e6
    rows = []
    for x in range(A["osd_driver"]["nvm_bytes"]):
        t_req = (8 + 4 * x - 5) * tb
        rows.append({"x": x, "t_request_us": round(t_req, 2), "t_data_end_us": round(t_req + tb, 2),
                     "before_min": t_req < at["nvm_read_us"]["min"],
                     "before_typ": t_req < at["nvm_read_us"]["typ"]})
    return {"sclk_hz": sclk_hz, "byte_us": tb,
            "first_cmdo_request_us": rows[0]["t_request_us"],
            "first_cmdo_data_end_us": rows[0]["t_data_end_us"],
            "reads_before_22us_min": sum(r["before_min"] for r in rows),
            "reads_before_30us_typ": sum(r["before_typ"] for r in rows),
            "vm0_to_cmm_us": 4 * tb,
            "ok_max7456": rows[0]["t_request_us"] >= at["nvm_read_us"]["max7456_min"],
            "ok_at7456e": rows[0]["t_request_us"] >= at["nvm_read_us"]["typ"],
            "rows_first_12": rows[:12]}


def _func_body(txt, name):
    m = re.search(r"\n[^\n]*::%s\s*\([^)]*\)\s*\{" % re.escape(name), txt)
    if not m:
        return None
    i, depth = m.end() - 1, 0
    for j in range(i, len(txt)):
        if txt[j] == "{":
            depth += 1
        elif txt[j] == "}":
            depth -= 1
            if depth == 0:
                return txt[i:j + 1]
    return None


def _strip_comments(s):
    s = re.sub(r"/\*.*?\*/", "", s, flags=re.S)
    return re.sub(r"//[^\n]*", "", s)


def burst_layout(body, macros):
    """Byte layout of check_font_char() from its buffer_add_cmd() sequence,
    the transfer slices, the waits between them and the rx parse offsets."""
    b = _strip_comments(body)
    pre = b.split("for")[0]
    regs = re.findall(r"buffer_add_cmd\(\s*(\w+)\s*,", pre)
    loop = re.search(r"for\s*\([^)]*NVM_RAM_SIZE[^)]*\)\s*\{(.*?)\}", b, re.S)
    loop_regs = re.findall(r"buffer_add_cmd\(\s*(\w+)\s*,", loop.group(1)) if loop else []
    layout = {}
    for k, r in enumerate(regs):
        layout[r] = 2 * k
    base = 2 * len(regs)
    if "MAX7456ADD_CMDO" in loop_regs:
        layout["first_CMDO_addr"] = base + 2 * loop_regs.index("MAX7456ADD_CMDO")
        layout["first_CMDO_data"] = layout["first_CMDO_addr"] + 1
        layout["stride"] = 2 * len(loop_regs)
    # sequence of transfers and waits after the loop
    seq = []
    for m in re.finditer(r"_dev->transfer_fullduplex\(\s*(&buffer\[\s*(\w+)\s*\]|buffer)\s*,\s*([^)]+?)\s*\)"
                         r"|hal\.scheduler->delay_microseconds\(\s*(\w+)\s*\)"
                         r"|hal\.scheduler->delay\(\s*(\w+)\s*\)", b):
        if m.group(1):
            start = 0 if m.group(1) == "buffer" else int(macros.get(m.group(2), m.group(2)))
            ln = m.group(3)
            seq.append({"op": "xfer", "start": start, "len": ln})
        else:
            val = m.group(4) or m.group(5)
            us = float(macros.get(val, val)) * (1000.0 if m.group(5) else 1.0)
            seq.append({"op": "wait", "us": us, "expr": val})
    parse = re.search(r"buffer_offset\s*=\s*(\d+)\s*;(?!.*buffer_offset\s*=\s*\d+\s*;)", b, re.S)
    stride = re.search(r"buffer_offset\s*\+=\s*(\d+)", b)
    return {"regs_before_loop": regs, "loop_regs": loop_regs, "layout": layout, "sequence": seq,
            "rx_parse_start": int(parse.group(1)) if parse else None,
            "rx_parse_stride": int(stride.group(1)) if stride else None}


def evaluate_layout(lay, sclk_hz):
    """Worst-case time from the CMM latch (and from VM0 write) to the first CMDO read
    request for a transfer/wait sequence.  Waits are taken as N-1 us (one 1 MHz
    ChibiOS tick of quantisation), transfer setup/CS overhead as zero."""
    L = lay["layout"]
    tb = 8 / sclk_hz * 1e6
    if "MAX7456ADD_CMM" not in L or "first_CMDO_addr" not in L:
        return {"ok": False, "why": "layout not recognised"}
    cmm_end = L["MAX7456ADD_CMM"] + 2          # byte index after which CMM is latched
    vm0_end = L.get("MAX7456ADD_VM0", -2) + 2
    cmdo_end = L["first_CMDO_addr"] + 1
    # map byte index -> time, walking the sequence
    t, pos, tmap, gaps = 0.0, 0, {}, []
    slices = []
    total_len = None
    for s in lay["sequence"]:
        if s["op"] == "wait":
            t += max(0.0, s["us"] - 1.0)
            gaps.append((pos, s["us"]))
            continue
        start = s["start"]
        ln = s["len"]
        if re.fullmatch(r"\d+", ln):
            n = int(ln)
        elif re.fullmatch(r"buffer_offset\s*-\s*(\d+)", ln):
            n = None
        elif ln.strip() == "buffer_offset":
            n = None
        else:
            return {"ok": False, "why": "transfer length %r not understood" % ln}
        if start != pos:
            return {"ok": False, "why": "transfer slices not contiguous (%d != %d)" % (start, pos)}
        if n is None:
            total_len = "to buffer_offset"
            n = 10 ** 6
        slices.append((start, n))
        for k in range(start, min(start + n, cmdo_end + 1)):
            tmap[k + 1] = t + (k - start + 1) * tb   # time at the END of byte k
        t += n * tb if n < 10 ** 6 else 0
        pos = start + n
    if cmm_end not in tmap or cmdo_end not in tmap:
        return {"ok": False, "why": "CMM or CMDO bytes not covered by transfers"}
    d_cmm = tmap[cmdo_end] - tmap[cmm_end]
    d_vm0 = (tmap[L["MAX7456ADD_CMAH"] + 2] - tmap[vm0_end]) if vm0_end in tmap and \
        "MAX7456ADD_CMAH" in L else None
    waits_cmm = [g for g in gaps if cmm_end <= g[0] < cmdo_end]
    waits_vm0 = [g for g in gaps if vm0_end <= g[0] <= L.get("MAX7456ADD_CMAH", -1)]
    return {"ok": True, "slices": slices, "last_slice": total_len,
            "cmm_to_first_cmdo_request_us": d_cmm, "vm0_to_cmah_us": d_vm0,
            "waits_between_cmm_and_cmdo_us": [g[1] for g in waits_cmm],
            "waits_between_vm0_and_cmah_us": [g[1] for g in waits_vm0],
            "rx_index_ok": lay["rx_parse_start"] == L["first_CMDO_data"] and
                           lay["rx_parse_stride"] == L.get("stride"),
            "rx_parse": (lay["rx_parse_start"], lay["rx_parse_stride"]),
            "rx_expected": (L["first_CMDO_data"], L.get("stride"))}


def patch_check(ap_root, sclk_hz):
    p = os.path.join(REPO, PATCH_REL)
    res = {"path": PATCH_REL, "present": os.path.exists(p)}
    if not res["present"]:
        return res
    txt = read(p)
    files = re.findall(r"^\+\+\+ b/(\S+)", txt, re.M)
    res["files"] = files
    res["touches_only_osd"] = files == [OSD_CPP]
    res["added_delay_macros"] = dict(re.findall(r"^\+#define (MAX7456_\w+_US) (\d+)", txt, re.M))
    if not ap_root:
        res["applies"] = None
        res["why"] = "no --ardupilot tree: git apply --check skipped"
        return res
    up = os.path.join(ap_root, OSD_CPP)
    top = subprocess.run(["git", "-C", ap_root, "rev-parse", "--show-toplevel"], capture_output=True, text=True)
    pre = subprocess.run(["git", "-C", ap_root, "rev-parse", "--show-prefix"], capture_output=True, text=True)
    if top.returncode == 0:
        st = subprocess.run(["git", "-C", ap_root, "status", "--porcelain", "--", OSD_CPP],
                            capture_output=True, text=True)
        res["upstream_file_unmodified"] = st.returncode == 0 and st.stdout.strip() == ""
        res["tree_head"] = subprocess.run(["git", "-C", ap_root, "rev-parse", "HEAD"],
                                          capture_output=True, text=True).stdout.strip()
        # run from the tree root: git apply prefixes paths with the cwd's
        # repository prefix (the reference tree may be a sub-directory of a worktree)
        chk = subprocess.run(["git", "-C", ap_root, "apply", "--check", p], capture_output=True, text=True)
        res["tree_repo_prefix"] = pre.stdout.strip()
        res["git_apply_check_tree"] = {"rc": chk.returncode, "out": (chk.stdout + chk.stderr).strip()[-400:]}
    else:
        res["upstream_file_unmodified"] = None
    tmp = tempfile.mkdtemp(prefix="nanoh743_osd_")
    try:
        dst = os.path.join(tmp, os.path.dirname(OSD_CPP))
        os.makedirs(dst)
        shutil.copy(up, dst)
        r = subprocess.run(["git", "apply", "--check", p], cwd=tmp, capture_output=True, text=True)
        res["git_apply_check_copy"] = {"rc": r.returncode, "out": (r.stdout + r.stderr).strip()[-400:]}
        r2 = subprocess.run(["git", "apply", p], cwd=tmp, capture_output=True, text=True)
        res["applies"] = r.returncode == 0 and r2.returncode == 0 and \
            res.get("git_apply_check_tree", {"rc": 0})["rc"] == 0
        if res["applies"]:
            patched = read(os.path.join(tmp, OSD_CPP))
            macros = dict(re.findall(r"^#define (\w+) (\d+)\s*$", patched, re.M))
            body = _func_body(patched, "check_font_char")
            lay = burst_layout(body, macros)
            res["patched_layout"] = lay
            res["patched_timing"] = evaluate_layout(lay, sclk_hz)
            res["patched_timing_10MHz"] = evaluate_layout(lay, A["spi"]["stock_osd_hz"])
            # the rest of the file must be unchanged except check_font_char and the new macros
            up_txt = read(up)
            res["other_functions_unchanged"] = all(
                _func_body(up_txt, f) == _func_body(patched, f)
                for f in ("update_font_char", "transfer_frame", "init", "update_font", "check_reinit"))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return res


# ----------------------------------------------------------------------------
# 4. Video network
# ----------------------------------------------------------------------------
def ltd_interp(c_eff_uF):
    """Line-time distortion from DS Table 2 for COUT = CSAG rows (10/22/47 uF),
    log-interpolated (DS p.10: LTD scales inversely with capacitance)."""
    rows = sorted([(c, l) for c, s, l in A["at7456e"]["sag_table"] if s == c])
    if c_eff_uF <= rows[0][0]:
        return rows[0][1] * rows[0][0] / c_eff_uF
    for (c1, l1), (c2, l2) in zip(rows, rows[1:]):
        if c1 <= c_eff_uF <= c2:
            f = math.log(c_eff_uF / c1) / math.log(c2 / c1)
            return l1 + (l2 - l1) * f
    return rows[-1][1]


def video_network():
    at, v = A["at7456e"], A["video"]
    rail = A["rail_3v3"]
    i_osd = at["i_total_max_mA"] * 1e-3
    fb_rdc = 0.39                                    # BLM15AX601SN1D Rdc max (hse_vdda_analysis.py A['bead'])
    v_osd_nom = rail["nom"] - i_osd * fb_rdc
    v_osd_min = rail["nom"] * (1 - rail["tol_frac"]) - i_osd * fb_rdc
    rail_tol_needed = 1 - (at["vdd_min"] + i_osd * fb_rdc) / rail["nom"]
    vin_at_pin = v["camera_vpp"] * v["r27_ohm"] / (v["source_ohm"] + v["r27_ohm"]) * 2  # camera EMF 2 Vpp
    vin_peak = at["vin_clamp_sync_tip_V"] + at["vin_range_vpp"][1]
    vout_pp = vin_at_pin * at["gain"][1]
    vout_pp_max_in = at["vin_range_vpp"][1] * at["gain"][2]
    vout_top = at["vout_sync_tip_V"] + vout_pp
    load_pp = vout_pp * v["load_ohm"] / (v["load_ohm"] + v["r28_ohm"] + at["zout_ohm"])
    gamma = (v["r28_ohm"] + at["zout_ohm"] - v["load_ohm"]) / (v["r28_ohm"] + at["zout_ohm"] + v["load_ohm"])
    c_eff = v["cout_uF"] * v["cout_bias_retention"]
    c_eff_low = c_eff * (1 - v["cout_tol"])
    rows = [{"COUT_uF": c, "CSAG_uF": s, "LTD_typ_pct": l} for c, s, l in at["sag_table"]]
    tau_in_ms = (v["source_ohm"] * v["r27_ohm"] / (v["source_ohm"] + v["r27_ohm"])) * v["c31_uF"] * 1e-3
    return {
        "osd_rail": {"I_max_mA": at["i_total_max_mA"], "FB5_Rdc_max_ohm": fb_rdc,
                     "V_osd_nom_V": v_osd_nom, "V_osd_min_at_rail_tol_V": v_osd_min,
                     "AT7456E_min_V": at["vdd_min"], "rail_tol_needed_frac": rail_tol_needed},
        "input": {"termination_ohm": v["r27_ohm"], "coupling_uF": v["c31_uF"],
                  "vin_pp_V": vin_at_pin, "range_vpp": at["vin_range_vpp"],
                  "clamp_sync_tip_V": at["vin_clamp_sync_tip_V"], "vin_peak_V_at_1V2pp": vin_peak,
                  "source_tau_ms": tau_in_ms,
                  "clamp_settle_ms_PAL": at["clamp_settling_lines"] * v["line_us"]["PAL"] / 1e3,
                  "note": "100 nF is the clamp hold capacitor required by DS p.10 (settling scales with C, "
                          "LTD inversely); the DC restore happens every line, so there is no field-rate HPF tilt"},
        "output": {"gain": at["gain"], "vout_pp_V": vout_pp, "vout_pp_at_1V2_in_max_gain_V": vout_pp_max_in,
                   "swing_min_vpp": at["vout_swing_min_vpp"], "sync_tip_V": at["vout_sync_tip_V"],
                   "vout_top_V": vout_top, "headroom_to_PVDD_min_V": v_osd_min - vout_top,
                   "load_vpp_V": load_pp, "back_termination_gamma": gamma,
                   "sag_table": rows, "COUT_eff_uF": c_eff, "COUT_eff_low_uF": c_eff_low,
                   "LTD_nominal_pct": ltd_interp(v["cout_uF"]),
                   "LTD_dc_bias_pct": ltd_interp(c_eff), "LTD_dc_bias_tol_pct": ltd_interp(c_eff_low),
                   "LTD_limit_pct": at["ltd_max_pct"]},
    }


# ----------------------------------------------------------------------------
# 5. BEC
# ----------------------------------------------------------------------------
def fb_corners(mode, leak_A=0.0, rds_ohm=0.0):
    b = A["bec"]
    t = b["r_tol"]
    out = []
    for vref, kt, kb, ks, ifb in itertools.product(b["vref_V"][::2], (1 - t, 1 + t), (1 - t, 1 + t),
                                                   (1 - t, 1 + t), (-b["ifb_max_A"], b["ifb_max_A"])):
        rtop, rbot = b["rtop"] * kt, b["rbot"] * kb
        if mode == 12:
            rb = par(rbot, b["rsw"] * ks + rds_ohm)
            extra = 0.0
        else:
            rb = rbot
            extra = leak_A
        out.append(vref * (1 + rtop / rb) + (ifb + extra) * rtop)
    return min(out), max(out)


def fb_nominal(mode):
    b = A["bec"]
    rb = par(b["rbot"], b["rsw"]) if mode == 12 else b["rbot"]
    return b["vref_V"][1] * (1 + b["rtop"] / rb)


def q1_analysis():
    b, q, r = A["bec"], A["q1"], A["rail_3v3"]
    vgs_min = r["nom"] * (1 - r["tol_frac"]) * b["r_gate_pd"] / (b["r_gate_pd"] + b["r_gate_series"])
    overdrive = vgs_min - q["vth_V"][2]
    # Rds bound: datasheet 7 R max at VGS 5 V, scaled 1/(VGS - Vth) (square-law triode)
    rds = q["rds_5V_max_ohm"] * (5.0 - q["vth_V"][2]) / overdrive if overdrive > 0 else float("inf")
    i_d = b["vref_V"][2] / b["rsw"]
    idss_hot = q["idss_25C_A"] * 2 ** ((q["t_hot_C"] - 25) / 10.0)
    # off with gate floating: R38 ties gate to GND; GPIO high-Z leakage x R38
    vg_off = A["mcu_gpio"]["ilkg_A"] * b["r_gate_pd"]
    return {"vgs_on_min_V": vgs_min, "vth_max_V": q["vth_V"][2], "overdrive_V": overdrive,
            "rds_on_bound_ohm": rds, "id_on_A": i_d, "vds_on_V": i_d * rds,
            "err_12V_from_rds_V": fb_nominal(12) - A["bec"]["vref_V"][1] * (
                1 + b["rtop"] / par(b["rbot"], b["rsw"] + rds)),
            "idss_25C_A": q["idss_25C_A"], "idss_hot_A": idss_hot,
            "err_9V_leak_25C_V": q["idss_25C_A"] * b["rtop"], "err_9V_leak_hot_V": idss_hot * b["rtop"],
            "vg_off_highZ_V": vg_off, "vth_min_V": q["vth_V"][0],
            "coss_pole_ns": b["rsw"] * q["coss_pF"] * 1e-3}


def transitions():
    b = A["bec"]
    v9_min = fb_corners(9)[0]
    t = b["r_tol"]
    # 12 -> 9: Q1 off, output still at V12; FB / Vref (lowest ratio corner)
    fb_12to9 = min(fb_corners(12)[0] * b["rbot"] * kb / (b["rtop"] * kt + b["rbot"] * kb) / vr
                   for kb in (1 - t, 1 + t) for kt in (1 - t, 1 + t) for vr in b["vref_V"][::2])
    # 9 -> 12: Q1 on, output still at V9; FB / Vref (lowest ratio corner)
    fb_9to12 = min(v9_min * par(b["rbot"] * kb, b["rsw"] * ks) / (b["rtop"] * kt + par(b["rbot"] * kb, b["rsw"] * ks))
                   / vr for kb in (1 - t, 1 + t) for kt in (1 - t, 1 + t) for ks in (1 - t, 1 + t)
                   for vr in b["vref_V"][::2])
    fb_9to12_max = max(fb_corners(9)[1] * par(b["rbot"] * kb, b["rsw"] * ks) /
                       (b["rtop"] * kt + par(b["rbot"] * kb, b["rsw"] * ks)) / vr
                       for kb in (1 - t, 1 + t) for kt in (1 - t, 1 + t) for ks in (1 - t, 1 + t)
                       for vr in b["vref_V"][::2])
    cout_eff = A["bec"]["cout"]["n"] * A["bec"]["cout"]["uF"] * A["bec"]["cout"]["retention_at_12V"] * 1e-6
    v_ovp_release = fb_nominal(9) * b["ovp_frac"][1]
    dv = fb_nominal(12) - v_ovp_release
    loads = {k: v["P_W"] / fb_nominal(12) for k, v in A["vtx"].items()}
    return {"12to9_FB_over_Vref_min": fb_12to9, "OVP_max": b["ovp_frac"][2],
            "12to9_OVP_trips": fb_12to9 > b["ovp_frac"][2],
            "9to12_FB_over_Vref_min": fb_9to12, "9to12_FB_over_Vref_max": fb_9to12_max,
            "UVP": b["uvp_frac"],
            "9to12_above_UVP": fb_9to12 > b["uvp_frac"],
            "ovp_release_V": v_ovp_release,
            "12to9_decay_ms_per_load": {k: 1e3 * cout_eff * dv / i for k, i in loads.items()},
            "12to9_decay_s_unloaded_divider": cout_eff * (b["rtop"] + b["rbot"]) * math.log(fb_nominal(12) /
                                                                                          v_ovp_release),
            "note": "DS 9.3.11: OVP turns the high-side FET off, non-latching; DS 9.3.12: hiccup needs FB < 65 % "
                    "AND current limit for > 256 us"}


def dropout_model(vout, i, hot, d):
    b = A["bec"]
    k = b["rds_hot_factor"] if hot else 1.0
    rhs, rls = b["rds_hs_ohm"] * k, b["rds_ls_ohm"] * k
    dcr = b["inductor"]["dcr_hot_ohm"] if hot else b["inductor"]["dcr_ohm"]
    vin_min = (vout + i * dcr + (1 - d) * i * rls) / d + i * rhs
    return vin_min, rhs, rls, dcr


def dropout_table():
    b = A["bec"]
    i = b["iout_design_A"]
    d_fixed = 1 - b["toff_min_s"] * b["fsw_hz"][2]          # worst (highest) fsw -> lowest Dmax
    b["dmax_fixed"] = d_fixed
    vmin = {}
    for mode in (9, 12):
        v = fb_nominal(mode)
        for hot in (False, True):
            for name, d in (("fixed", d_fixed), ("foldback", b["dmax_foldback"])):
                vmin[(mode, hot, name)] = dropout_model(v, i, hot, d)[0]
    rows = []
    for n in A["battery"]["packs"]:
        for st, vc in A["battery"]["cell_V"].items():
            vin = n * vc
            for mode in (9, 12):
                vset = fb_nominal(mode)
                if vin >= vmin[(mode, True, "fixed")]:
                    state, vout = "regulated (fixed 500 kHz)", vset
                elif vin >= vmin[(mode, True, "foldback")]:
                    state, vout = "regulated (frequency foldback)", vset
                else:
                    _, rhs, rls, dcr = dropout_model(vset, i, True, b["dmax_foldback"])
                    d = b["dmax_foldback"]
                    vout = d * (vin - i * rhs) - (1 - d) * i * rls - i * dcr
                    state = "dropout (pass-through)"
                rows.append({"pack": "%dS" % n, "state": st, "VBAT_V": round(vin, 2), "mode_V": mode,
                             "Vout_V": round(vout, 2), "regime": state,
                             "headroom_V": round(vin - vout, 2)})
    return {"Dmax_fixed": d_fixed, "Dmax_foldback": b["dmax_foldback"], "I_A": i,
            "Vin_min_V": {"%dV_%s_%s" % (m, "hot" if h else "25C", nm): round(v, 3)
                          for (m, h, nm), v in vmin.items()},
            "Dmax_1200kHz": 1 - b["toff_min_s"] * 1.35e6,
            "Vin_min_12V_1200kHz_hot_V": dropout_model(fb_nominal(12), i, True, 1 - b["toff_min_s"] * 1.35e6)[0],
            "table": rows,
            "dropout_headroom_2A_hot_V": vmin[(12, True, "foldback")] - fb_nominal(12),
            "model": "Vin_min = (Vout + I DCR + (1-D) I Rls)/D + I Rhs (TI Eq. 8/10 with conduction drops); "
                     "fixed-frequency Dmax at fSW max 550 kHz; hot = Rds x1.4, DCR 65 mOhm; battery sag, "
                     "traces and shunt not included"}


def inductor_caps():
    b = A["bec"]
    ind, co, ci = b["inductor"], b["cout"], b["cin"]
    i = b["iout_design_A"]
    vin_max = 4 * A["battery"]["cell_V"]["full"]
    rows = []
    for mode in (9, 12):
        v = fb_nominal(mode)
        for vin in (vin_max, 4 * A["battery"]["cell_V"]["nominal"], 3 * A["battery"]["cell_V"]["nominal"]):
            if vin <= v + 0.5:
                continue
            for lk in (1 - ind["L_tol"], 1.0):
                dil = v * (vin - v) / (vin * ind["L_uH"] * lk * 1e-6 * b["fsw_hz"][0])
                rows.append({"mode_V": mode, "Vin_V": vin, "L_uH": ind["L_uH"] * lk,
                             "dIL_App": dil, "IL_peak_A": i + dil / 2,
                             "IL_rms_A": math.sqrt(i ** 2 + dil ** 2 / 12), "K": dil / b["iout_rated_A"]})
    nominal = [r for r in rows if r["L_uH"] == ind["L_uH"]]
    # DS 10.2.2.7: K (ripple / IOUT_MAX 3 A) is a selection rule evaluated at VIN_MAX
    at_vmax = [r for r in nominal if r["Vin_V"] == vin_max]
    peak = max(r["IL_peak_A"] for r in rows)
    # current-limit peak (short / overload): IHS_LIMIT max
    cout_eff = {m: co["n"] * co["uF"] * (1 - co["tol"]) * co["retention_at_%dV" % m] for m in (9, 12)}
    rip = {}
    for m in (9, 12):
        dil = max(r["dIL_App"] for r in rows if r["mode_V"] == m)
        rip[m] = dil / (8 * b["fsw_hz"][0] * cout_eff[m] * 1e-6)
    tss = [b["css_F"] * b["vref_V"][1] / iss * 1e3 for iss in b["iss_A"][::-1]]
    inrush = {m: cout_eff[m] * 1e-6 * fb_nominal(m) / (min(tss) * 1e-3) for m in (9, 12)}
    # input ripple current (Eq. 22: I sqrt(D(1-D))), worst at D = 0.5
    d_list = [fb_nominal(m) / v for m in (9, 12) for v in (vin_max, 14.8, 11.1) if v > fb_nominal(m)]
    icin = max(i * math.sqrt(d * (1 - d)) for d in d_list)
    cin_eff = ci["n"] * ci["uF"] * ci["retention_at_16V8"]
    dvin = max(i * d * (1 - d) / (b["fsw_hz"][0] * cin_eff * 1e-6) for d in d_list)
    return {"ripple_rows": rows, "IL_peak_max_A": peak, "Isat_A": ind["isat_A"],
            "IHS_LIMIT_max_A": b["ihs_limit_A"][2],
            "K_nominal_range": [min(r["K"] for r in at_vmax), max(r["K"] for r in at_vmax)],
            "K_all_rows_range": [min(r["K"] for r in nominal), max(r["K"] for r in nominal)],
            "ripple_min_A_nominal": min(r["dIL_App"] for r in nominal),
            "ripple_min_required_A": b["ripple_min_frac_of_3A"] * b["iout_rated_A"],
            "Cout_eff_uF": cout_eff, "Cout_DS_min_uF": b["table_10_2_12V"]["cout_min_uF"],
            "Cout_DS_typ_uF": b["table_10_2_12V"]["cout_typ_uF"], "Vout_ripple_mVpp": {m: rip[m] * 1e3 for m in rip},
            "tSS_ms": tss, "inrush_A": inrush, "Icin_rms_A": icin, "Cin_eff_uF": cin_eff,
            "Vin_ripple_mVpp": dvin * 1e3,
            "TI_12V_reference": b["table_10_2_12V"],
            "L_vs_TI_note": "10 uH vs TI 12 uH (Table 10-2, 12 V / 500 kHz): ripple +20 %, K inside 20..60 %"}


def thermal():
    b = A["bec"]
    i = b["iout_design_A"]
    rows = []
    for mode in (9, 12):
        v = fb_nominal(mode)
        for vin in (16.8, 14.8, 11.1, 12.6):
            if vin < v + 0.6:
                continue
            d = v / vin
            for hot in (False, True):
                k = b["rds_hot_factor"] if hot else 1.0
                pc = i ** 2 * (b["rds_hs_ohm"] * k * d + b["rds_ls_ohm"] * k * (1 - d))
                psw = 0.5 * vin * i * 2 * b["sw_edge_ns"] * 1e-9 * b["fsw_hz"][1]
                pq = vin * b["iq_A"] + 5.0 * 5e-9 * b["fsw_hz"][1]   # gate charge ~5 nC at 5 V (estimate)
                p = pc + psw + pq
                pl = (i ** 2) * (b["inductor"]["dcr_hot_ohm"] if hot else b["inductor"]["dcr_ohm"])
                rows.append({"mode_V": mode, "Vin_V": vin, "hot_rds": hot, "P_ic_W": p, "P_L_W": pl,
                             "eff_est": v * i / (v * i + p + pl)})
    pmax = max(r["P_ic_W"] for r in rows if r["hot_rds"])
    t_amb = 35.0          # analysis/thermal_power_analysis.py ASSUME t_amb
    t_board = 105.0       # analysis/thermal_report.json limits_C.t_board_cont
    return {"rows": rows, "P_ic_max_W": pmax,
            "Tj_ambient_case_C": t_amb + pmax * b["rth_ja_evm"],
            "Tj_hot_board_case_C": t_board + pmax * b["psi_jb"],
            "Tj_limit_C": b["tj_max_C"] - b["tj_margin_C"],
            "basis": "a) 35 C ambient x RthJA_EVM 60.2 K/W; b) AIO board at its 105 C continuous limit "
                     "x PsiJB 19.2 K/W (DS 8.4); switching edges 5 ns assumed (not in DS)"}


def connector():
    c, rows = A["connector"], []
    for name, v in A["vtx"].items():
        for mode in (9, 12):
            vout = fb_nominal(mode)
            i = v["P_W"] / vout
            in_range = v["vin"][0] <= fb_corners(mode)[0] and fb_corners(mode)[1] <= v["vin"][1]
            rows.append({"vtx": name, "mode_V": mode, "I_A": round(i, 3),
                         "I_per_GND_contact_A": round(i / 2, 3),
                         "J1_ok": i <= c["contact_A"], "needs_J2": i > c["contact_A"],
                         "J2_ok": i <= c["J2_A"], "bec_ok": i <= A["bec"]["iout_design_A"],
                         "vin_range_ok": in_range})
    return rows


def sense():
    s = A["sense"]
    ratio = (s["r40"] + s["r41"]) / s["r41"]
    t = s["tol"]
    r_hi = (s["r40"] * (1 + t) + s["r41"] * (1 - t)) / (s["r41"] * (1 - t))
    r_lo = (s["r40"] * (1 - t) + s["r41"] * (1 + t)) / (s["r41"] * (1 + t))
    ratio_err = max(r_hi / ratio - 1, 1 - r_lo / ratio)
    v12 = fb_nominal(12)
    lsb = s["ap_fullscale_V"] / (2 ** s["adc_bits"])
    tau = par(s["r40"], s["r41"]) * s["c43_F"]
    rail_err_9 = max(abs(fb_corners(9)[0] / fb_nominal(9) - 1), abs(fb_corners(9)[1] / fb_nominal(9) - 1))
    rail_err_12 = max(abs(fb_corners(12)[0] / v12 - 1), abs(fb_corners(12)[1] / v12 - 1))
    vdda = A["rail_3v3"]["tol_frac"]
    worst = max(rail_err_9, rail_err_12) + ratio_err + vdda
    rss = math.sqrt(max(rail_err_9, rail_err_12) ** 2 + ratio_err ** 2 + vdda ** 2)
    return {"ratio": ratio, "ratio_err_frac": ratio_err, "V_adc_at_12V": v12 / ratio,
            "V_adc_at_9V": fb_nominal(9) / ratio, "fullscale_input_V": s["ap_fullscale_V"] * ratio,
            "VBAT_max_V": 4 * A["battery"]["cell_V"]["full"],
            "lsb_input_mV": lsb * ratio * 1e3, "tau_ms": tau * 1e3, "settle_0p1pct_ms": tau * 1e3 * math.log(1000),
            "rail_err_frac": {"9V": rail_err_9, "12V": rail_err_12}, "vdda_err_frac": vdda,
            "total_err_worst_frac": worst, "total_err_rss_frac": rss, "fw_window_frac": s["fw_window_frac"],
            "recommended_window_frac": math.ceil(worst * 100 + 1) / 100.0,
            "c43_charge_share_err": 4e-12 / s["c43_F"],
            "ap_scaling_note": "hwdef.h HAL_ANALOG_PINS uses 3.30/4096; AnalogIn.cpp VOLTAGE_SCALING 3.3/4095 "
                               "(0.02 % apart): BATT2_VOLT_MULT = divider ratio 11.0"}


# ----------------------------------------------------------------------------
# 9. ArduPilot evidence / hwdef consistency
# ----------------------------------------------------------------------------
def firmware_evidence(ap_root):
    ev = {"checked": False}
    if not ap_root:
        return ev
    ev["checked"] = True
    cites = {}

    def load(rel):
        return read(os.path.join(ap_root, rel))
    rel = "libraries/AP_HAL_ChibiOS/SPIDevice.cpp"
    t = load(rel)
    ev["spi_bus_clock_half"] = bool(re.search(r"spi_clock_freq = bus_clocks\[busid-1\] / 2;", t))
    ev["spi_halving_loop"] = bool(re.search(r"while \(spi_clock_freq > _frequency && i<7\) \{\s*"
                                            r"spi_clock_freq >>= 1;\s*i\+\+;", t))
    ev["spi_h7_mbr"] = "(i * SPI_CFG1_MBR_0) | SPI_CFG1_DSIZE_VALUE(7)" in t
    ev["spi2_clock_is_STM32_SPI2CLK"] = bool(re.search(r"#define SPI2_CLOCK\s+STM32_SPI2CLK", t))
    ev["spi_ctor_speed_low"] = "set_speed(AP_HAL::Device::SPEED_LOW);" in t
    ev["spi_timeout_20ms_32us"] = "const uint32_t timeout_us = 20000U + len * 32U;" in t
    ev["fullduplex_inplace_do_transfer"] = bool(re.search(
        r"transfer_fullduplex\(uint8_t \*send_recv, uint32_t len\)\s*\{.{0,200}?do_transfer\(send_recv, send_recv, len\)",
        t, re.S))
    cites["derive_freq_flag_bus"] = cite(ap_root, rel, t, r"uint32_t SPIDevice::derive_freq_flag_bus")
    cites["halving_loop"] = cite(ap_root, rel, t, r"while \(spi_clock_freq > _frequency")
    cites["timeout"] = cite(ap_root, rel, t, r"timeout_us = 20000U \+ len \* 32U")
    rel = OSD_CPP
    t = load(rel)
    body = _func_body(t, "check_font_char") or ""
    ev["osd_upstream_layout"] = burst_layout(body, {})
    ev["osd_single_burst"] = bool(re.search(r"transfer_fullduplex\(buffer, buffer_offset\)", body)) and \
        "delay" not in _strip_comments(body)
    ev["osd_rx_offset_9_stride_4"] = (ev["osd_upstream_layout"]["rx_parse_start"],
                                      ev["osd_upstream_layout"]["rx_parse_stride"]) == (9, 4)
    ev["osd_no_set_speed"] = "set_speed" not in t
    ev["osd_frame_stop"] = "if (buffer_offset >= spi_buffer_size - 32)" in t
    ev["osd_reset_wait_1ms"] = bool(re.search(r"write_register\(MAX7456ADD_VM0, MAX7456_RESET\);\s*"
                                              r"hal\.scheduler->delay\(1\);", t))
    ev["osd_nvm_poll_15ms"] = "hal.scheduler->delay(15);" in t
    ev["osd_ntsc_alt_quirk"] = "#define VIN_IS_NTSC_ALT(val)  (!STAT_IS_LOS(val) && !STAT_IS_PAL(val))" in t
    cites["check_font_char"] = cite(ap_root, rel, t, r"bool AP_OSD_MAX7456::check_font_char")
    cites["transfer_frame_stop"] = cite(ap_root, rel, t, r"buffer_offset >= spi_buffer_size - 32")
    h = load("libraries/AP_OSD/AP_OSD_MAX7456.h")
    ev["osd_spi_buffer_512"] = "static const uint16_t spi_buffer_size = 512;" in h
    t = load("libraries/AP_OSD/AP_OSD.cpp")
    ev["osd_thread_100ms"] = bool(re.search(r"hal\.scheduler->delay\(100\);\s*\n[^\n]*\n?[^\n]*update_osd\(\)", t)) \
        or "hal.scheduler->delay(100);" in t
    cites["osd_thread_delay"] = cite(ap_root, "libraries/AP_OSD/AP_OSD.cpp", t, r"hal\.scheduler->delay\(100\);")
    rel = "libraries/AP_HAL_ChibiOS/Scheduler.cpp"
    t = load(rel)
    ev["delay_us_chThdSleep"] = bool(re.search(r"chThdSleep\(MAX\(ticks,CH_CFG_ST_TIMEDELTA\)\)", t))
    cites["delay_microseconds"] = cite(ap_root, rel, t, r"void Scheduler::delay_microseconds\(uint16_t usec\)")
    t = load("libraries/AP_HAL_ChibiOS/hwdef/common/chconf.h")
    m = re.search(r"#define CH_CFG_ST_FREQUENCY\s+(\d+)", t)
    ev["st_frequency_hz"] = int(m.group(1)) if m else None
    rel = "libraries/AP_Relay/AP_Relay.cpp"
    t = load(rel)
    ev["relay_pin_default_ifndef"] = all(("#ifndef RELAY%d_PIN_DEFAULT" % k) in t for k in (2, 3))
    ev["relay_init_drives_default"] = bool(re.search(
        r"if \(use_default_param\) \{.*?set_instance_state\(instance, \(bool\)default_state\);", t, re.S))
    ev["relay_convert_sets_RELAY_when_pin_valid"] = bool(re.search(
        r"if \(_params\[i\]\.pin < 0\) \{.*?continue;\s*\}\s*new_fun = AP_Relay_Params::FUNCTION::RELAY;", t, re.S))
    ev["relay_gpio_output_write"] = "hal.gpio->pinMode(pin, HAL_GPIO_OUTPUT);\n    hal.gpio->write(pin, value);" in t
    cites["relay_init"] = cite(ap_root, rel, t, r"void AP_Relay::init\(\)")
    cites["relay_set_defaults"] = cite(ap_root, rel, t, r"void AP_Relay::set_defaults\(\)")
    rel = "libraries/AP_Relay/AP_Relay_Params.cpp"
    t = load(rel)
    ev["relay_DEFAULT_param_default_off"] = bool(re.search(
        r'AP_GROUPINFO\("DEFAULT", 3, AP_Relay_Params, default_state, \(float\)DefaultState::OFF\)', t))
    cites["relay_DEFAULT_param"] = cite(ap_root, rel, t, r'AP_GROUPINFO\("DEFAULT"')
    rel = "libraries/AP_HAL_ChibiOS/AnalogIn.cpp"
    t = load(rel)
    ev["adc_voltage_scaling_3v3"] = "#define VOLTAGE_SCALING (3.3f / ((1 << 12) - 1))" in t
    cites["VOLTAGE_SCALING"] = cite(ap_root, rel, t, r"#define VOLTAGE_SCALING")
    rel = "libraries/AP_HAL_ChibiOS/hwdef/scripts/STM32H743xx.py"
    t = load(rel)
    a1 = t.split("ADC1_map = {")[1].split("}")[0]
    m = re.search(r'"PC5"\s*:\s*(\d+)', a1)
    ev["adc1_PC5_channel"] = int(m.group(1)) if m else None
    cites["ADC1_map_PC5"] = cite(ap_root, rel, t, r'ADC1_map = \{')
    rel = "libraries/AP_BattMonitor/AP_BattMonitor_Analog.cpp"
    t = load(rel)
    ev["batt_volt_pin_mult_params"] = 'AP_GROUPINFO("VOLT_PIN"' in t and 'AP_GROUPINFO("VOLT_MULT"' in t
    rel = "libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py"
    t = load(rel)
    ev["hwdef_ODR_default_HIGH"] = bool(re.search(r"def get_ODR_value\(self\):.*?v = 'HIGH'", t, re.S))
    ev["hwdef_unlisted_pin_INPUT"] = bool(re.search(r'else:\s*v = "INPUT"\s*return v', t))
    cites["get_ODR_value"] = cite(ap_root, rel, t, r"def get_ODR_value\(self\)")
    ev["cites"] = cites
    return ev


def hwdef_check():
    hw = read(os.path.join(REPO, "ardupilot", "hwdef.dat"))
    bl = read(os.path.join(REPO, "ardupilot", "hwdef-bl.dat"))
    dp = read(os.path.join(REPO, "ardupilot", "defaults.parm"))
    hw_nc = "\n".join(l.split("#")[0].rstrip() for l in hw.splitlines())
    bl_nc = "\n".join(l.split("#")[0].rstrip() for l in bl.splitlines())
    params = dict(re.findall(r"^([A-Z0-9_]+)\s+(\S+)", "\n".join(l.split("#")[0] for l in dp.splitlines()), re.M))
    r = {}
    m = re.findall(r"^SPIDEV\s+osd\s+SPI2\s+DEVID\d+\s+OSD1_CS\s+(MODE\d)\s+(\S+)\s+(\S+)\s*$", hw_nc, re.M)
    r["spidev_osd"] = m

    def mhz(s):
        mm = re.fullmatch(r"(\d+(?:\.\d+)?)\*MHZ", s)
        return float(mm.group(1)) * 1e6 if mm else None
    r["spidev_osd_hz"] = [(md, mhz(a), mhz(b)) for md, a, b in m]

    def gpio(port, label):
        mm = re.findall(r"^%s\s+%s\s+OUTPUT\b(.*)$" % (port, label), hw_nc, re.M)
        if len(mm) != 1:
            return None
        toks = mm[0].split()
        g = re.search(r"GPIO\((\d+)\)", mm[0])
        return {"level": "LOW" if "LOW" in toks else ("HIGH" if "HIGH" in toks else "HIGH(default)"),
                "gpio": int(g.group(1)) if g else None, "tokens": toks}
    r["VTX_PWR"] = gpio("PD10", "VTX_PWR")
    r["VTX_12V_SEL"] = gpio("PD11", "VTX_12V_SEL")
    r["RELAY2_PIN_DEFAULT"] = re.findall(r"^define RELAY2_PIN_DEFAULT (\S+)", hw_nc, re.M)
    r["RELAY3_PIN_DEFAULT"] = re.findall(r"^define RELAY3_PIN_DEFAULT (\S+)", hw_nc, re.M)
    r["VTX_VSENSE"] = re.findall(r"^PC5\s+VTX_VSENSE\s+(ADC\d)\s+SCALE\((\S+)\)", hw_nc, re.M)
    r["UART7_RX"] = re.findall(r"^PE7\s+UART7_RX\s+UART7(.*)$", hw_nc, re.M)
    r["UART7_TX_present"] = bool(re.search(r"^PE8\s+UART7_TX", hw_nc, re.M))
    so = re.findall(r"^SERIAL_ORDER\s+(.*)$", hw_nc, re.M)
    r["SERIAL_ORDER"] = so[0].split() if len(so) == 1 else so
    r["OSD_ENABLED"] = re.findall(r"^define OSD_ENABLED (\S+)", hw_nc, re.M)
    r["HAL_OSD_TYPE_DEFAULT"] = re.findall(r"^define HAL_OSD_TYPE_DEFAULT (\S+)", hw_nc, re.M)
    r["OSD1_CS"] = bool(re.search(r"^PB12\s+OSD1_CS\s+CS\b", hw_nc, re.M))
    r["spi2_pins"] = re.findall(r"^(PB1[345])\s+(SPI2_\w+)\s+SPI2\b", hw_nc, re.M)
    r["bl_has_vtx_pins"] = bool(re.search(r"^(PD10|PD11|PC5)\s", bl_nc, re.M))
    r["bl_osd_cs"] = bool(re.search(r"^PB12\s+OSD1_CS\s+CS\b", bl_nc, re.M))
    r["params"] = {k: params.get(k) for k in ("RELAY2_DEFAULT", "RELAY3_DEFAULT", "BATT2_MONITOR",
                                               "BATT2_VOLT_PIN", "BATT2_VOLT_MULT", "OSD_TYPE",
                                               "RELAY2_FUNCTION", "RELAY3_FUNCTION", "RELAY2_PIN", "RELAY3_PIN")}
    return r


def generated_header_check(ap_root):
    sys.path.insert(0, os.path.join(REPO, "tools"))
    import validate_dma as vd                         # reuse its staging + generator wrapper
    tmp = tempfile.mkdtemp(prefix="nanoh743_osd_")
    try:
        bdir = vd.stage(tmp)
        out = os.path.join(tmp, "fw")
        os.makedirs(out)
        rc, log = vd.run_generator(ap_root, os.path.join(bdir, "hwdef.dat"), out,
                                   params=os.path.join(bdir, "defaults.parm"))
        if rc != 0:
            return {"rc": rc, "log": log[-2000:]}
        hdr = read(os.path.join(out, "hwdef.h"))
        res = {"rc": 0}
        res["spidev_osd"] = re.findall(r'SPIDesc\("osd"\s*,\s*(\d+),\s*(\d+),\s*(PAL_LINE\([^)]*\)),\s*'
                                       r'(SPIDEV_MODE\d),\s*(\S+),\s*(\S+)\)', hdr)
        res["gpio81"] = re.findall(r"\{\s*81,\s*(\w+),\s*(\d+),\s*(PAL_LINE\([^)]*\))\}", hdr)
        res["gpio82"] = re.findall(r"\{\s*82,\s*(\w+),\s*(\d+),\s*(PAL_LINE\([^)]*\))\}", hdr)
        res["relay_pin_defaults"] = re.findall(r"#define (RELAY[23]_PIN_DEFAULT) (\d+)", hdr)

        def blk(reg, port):
            m = re.search(r"#define VAL_GPIO%s_%s\s+\((.*?)\)\n" % (port, reg), hdr, re.S)
            return m.group(1) if m else ""
        res["PD10"] = {r: (re.findall(r"PIN_\w+\(10U\)", blk(r, "D")) or [None])[0] for r in ("MODER", "ODR")}
        res["PD11"] = {r: (re.findall(r"PIN_\w+\(11U\)", blk(r, "D")) or [None])[0] for r in ("MODER", "ODR")}
        res["PC5_MODER"] = (re.findall(r"PIN_\w+\(5U\)", blk("MODER", "C")) or [None])[0]
        res["uart7_dma"] = re.findall(r"#define STM32_UART7_(RX|TX)_DMA_CONFIG (.*)", hdr)
        res["uart7_rx_stream"] = re.findall(r"STM32_UART_UART7_RX_DMA_STREAM", hdr)
        res["serial_list"] = re.findall(r"HAL_(\w+)_CONFIG", hdr.split("#define HAL_SERIAL_DEVICE_LIST")[1]
                                        .split("#define HAL_UART_NUM_SERIAL_PORTS")[0]) \
            if "#define HAL_SERIAL_DEVICE_LIST" in hdr else []
        res["analog_pins"] = re.findall(r"\{\s*(\d+),\s*(\d+),\s*([\d.]+/\d+)\s*\},\s*/\* (\w+) (\w+) \*/", hdr)
        res["spi2_dma"] = re.findall(r"#define STM32_SPI_SPI2_(RX|TX)_DMA_STREAM\s+(.*)", hdr)
        res["osd"] = re.findall(r"#define (HAL_WITH_SPI_OSD|OSD_ENABLED|HAL_OSD_TYPE_DEFAULT) (\S+)", hdr)
        # bootloader: what PD10/PD11/PC5 do before the firmware runs
        outb = os.path.join(tmp, "bl")
        os.makedirs(outb)
        rcb, logb = vd.run_generator(ap_root, os.path.join(bdir, "hwdef-bl.dat"), outb, bootloader=True)
        res["bl_rc"] = rcb
        if rcb == 0:
            hb = read(os.path.join(outb, "hwdef.h"))

            def blkb(reg, port):
                m = re.search(r"#define VAL_GPIO%s_%s\s+\((.*?)\)\n" % (port, reg), hb, re.S)
                return m.group(1) if m else ""
            res["bl"] = {
                "PD10": {r: (re.findall(r"PIN_\w+\(10U\)", blkb(r, "D")) or [None])[0] for r in ("MODER", "PUPDR")},
                "PD11": {r: (re.findall(r"PIN_\w+\(11U\)", blkb(r, "D")) or [None])[0] for r in ("MODER", "PUPDR")},
                "PB12": {r: (re.findall(r"PIN_\w+\(12U\)", blkb(r, "B")) or [None])[0] for r in ("MODER", "ODR")},
            }
        else:
            res["bl_log"] = logb[-1000:]
        return res
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def lua_crosscheck():
    p = os.path.join(REPO, "ardupilot", "scripts", "vtx_bec.lua")
    if not os.path.exists(p):
        return {"present": False}
    t = read(p)
    nums = [float(x) for x in re.findall(r"(?<![\w.])(\d+\.\d+)(?![\w.])", t)]
    return {"present": True, "has_13p6": 13.6 in nums, "has_13p0": 13.0 in nums,
            "uses_relay": "relay:" in t, "uses_arming": "arming:is_armed" in t}


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ardupilot", default=os.environ.get("ARDUPILOT_ROOT"))
    args = ap.parse_args()
    ap_root = os.path.abspath(args.ardupilot) if args.ardupilot and os.path.isdir(args.ardupilot) else None
    hw = hwdef_check()
    at = A["at7456e"]

    print("== 1. SPI2 clock ==")
    clk = spi_kernel_clock(ap_root)
    fk = clk["f_spi123_hz"]
    gate("C1 SPI2 kernel clock = PLL1_Q = 80 MHz", clk["spi123sel"].endswith("PLL1_Q_CK") and abs(fk - 80e6) < 1,
         "%s, PLL1 M=%d N=%d Q=%d -> %.3f MHz (%s)" % (clk["spi123sel"], clk["pll1_m"], clk["pll1_n"],
                                                       clk["pll1_q"], fk / 1e6, clk["source"]))
    hz = hw["spidev_osd_hz"]
    low = hz[0][1] if len(hz) == 1 else None
    high = hz[0][2] if len(hz) == 1 else None
    div = derive_freq_flag_bus(fk, low or A["spi"]["hwdef_lowspeed_hz"], A["spi"]["mbr_max"])
    div_hi = derive_freq_flag_bus(fk, high or A["spi"]["hwdef_highspeed_hz"], A["spi"]["mbr_max"])
    stock = derive_freq_flag_bus(fk, A["spi"]["stock_osd_hz"], A["spi"]["mbr_max"])
    table = [derive_freq_flag_bus(fk, f * 1e6) for f in (40, 20, 12, 10, 9.99, 8, 6, 5, 4.99, 2.5)]
    sclk = div["sclk_hz"]
    gate("C2 derive_freq_flag_bus(80 MHz, %s) -> MBR %d (/%d) = %.3f MHz (SPEED_LOW, used by AP_OSD_MAX7456)"
         % ("%.0f MHz" % (low / 1e6) if low else "?", div["mbr"], div["divisor"], sclk / 1e6),
         low is not None and div["mbr"] == 3 and abs(sclk - 5e6) < 1 and div_hi["sclk_hz"] == sclk,
         "stock 10*MHZ -> MBR %d = %.3f MHz; highspeed -> %.3f MHz" % (stock["mbr"], stock["sclk_hz"] / 1e6,
                                                                      div_hi["sclk_hz"] / 1e6))
    tim = spi_timing(sclk)
    tim10 = spi_timing(stock["sclk_hz"])
    gate("C3 AT7456E SCLK timing at %.1f MHz: tCP/tCH/tCL/tDS margins >= 25 %% of spec" % (sclk / 1e6),
         tim["within_10MHz"] and tim["margin_tCP_ns"] >= 0.25 * at["tCP_ns"] and
         tim["margin_tCH_ns"] >= 0.25 * at["tCH_ns"] and tim["margin_tCL_ns"] >= 0.25 * at["tCL_ns"] and
         tim["margin_tDS_ns"] >= 0.25 * at["tDS_ns"],
         "tCP %.0f (min 100) / tCH %.0f, tCL %.0f (min 40) / setup %.1f (tDS 30) ns; at 10 MHz tCP margin %.0f ns, "
         "tCH margin %.0f ns" % (tim["tCP_ns"], tim["tCH_ns"], tim["tCL_ns"], tim["mosi_setup_ns"],
                                 tim10["margin_tCP_ns"], tim10["margin_tCH_ns"]))
    gate("C4 SDOUT -> PB14 setup at the H7 sampling edge (tDO1 25 ns, 33 R x 8 pF, H7 tsu 5 ns)",
         tim["miso_setup_margin_ns"] > 20.0,
         "%.1f ns spare at %.0f MHz (%.1f ns at 10 MHz)" % (tim["miso_setup_margin_ns"], sclk / 1e6,
                                                           tim10["miso_setup_margin_ns"]))

    print("== 2. OSD SPI bandwidth ==")
    bw = osd_bandwidth(sclk)
    for k, c in bw["cases"].items():
        print("    %-12s %3d dirty: first flush %3d B (%3d chars), %d flush(es) = %.0f ms"
              % (k, c["dirty_chars"], c["first_flush_bytes"], c["first_flush_chars"], c["flushes_to_complete"],
                 c["time_to_complete_ms"]))
    gate("B1 transfer_frame() burst fits the 512-byte buffer (replay %d B, bound %d B)"
         % (bw["worst_flush_bytes_replay"], bw["worst_flush_bytes_bound"]),
         bw["worst_flush_bytes_bound"] <= bw["buffer_bytes"],
         "stop at 480 + <= 12 B/char + 4 B auto-increment exit")
    gate("B2 OSD SCLK time per 100 ms thread period < 2 %",
         bw["duty_per_period_pct"] < 2.0,
         "%.0f us per flush = %.2f %%; full PAL redraw in %d flushes (%.0f ms); do_transfer timeout %.1f ms"
         % (bw["flush_sclk_time_us"], bw["duty_per_period_pct"], bw["cases"]["full_screen"]["flushes_to_complete"],
            bw["cases"]["full_screen"]["time_to_complete_ms"], bw["do_transfer_timeout_us"] / 1e3))

    print("== 3. NVM read timing (check_font_char) ==")
    up5 = upstream_nvm_timing(sclk)
    up10 = upstream_nvm_timing(stock["sclk_hz"])
    finding("NVM-UP upstream AP_OSD_MAX7456::check_font_char single burst vs AT7456E NVM read (would FAIL)",
            "first CMDO read requested %.1f us (data clocked out by %.1f us) after CMM = 0x50 at 10 MHz, "
            "%.1f / %.1f us at this board's 5 MHz; AT7456E needs >= %.0f us min, %.0f us typ (DS p.36; "
            "MAX7456 0.5 us).  At 5 MHz the first %d reads precede 22 us and %d precede 30 us (at 10 MHz: %d / %d) "
            "-> stale shadow RAM, check fails, update_font() rewrites all 256 chars (>= %.1f s, NVM endurance %d)"
            % (up10["first_cmdo_request_us"], up10["first_cmdo_data_end_us"], up5["first_cmdo_request_us"],
               up5["first_cmdo_data_end_us"], at["nvm_read_us"]["min"], at["nvm_read_us"]["typ"],
               up5["reads_before_22us_min"], up5["reads_before_30us_typ"], up10["reads_before_22us_min"],
               up10["reads_before_30us_typ"], bw["boot_font_rewrite_all_s_min"], at["nvm_endurance_stores"]))
    finding("NVM-VM0 upstream VM0 = 0 -> CMAH/CMM gap vs AT7456E display-off time",
            "%.1f us at 5 MHz (%.1f at 10 MHz) vs up to %.0f us typ (DS p.36); CMAH/CMM are ignored while VM0[3] "
            "is still set (DS p.29).  Harmless at boot (init() soft-reset leaves VM0 = 0) but the first character "
            "of a runtime font change (flush() -> update_font(), display on) is re-written needlessly"
            % (up5["vm0_to_cmm_us"], up10["vm0_to_cmm_us"], at["osd_disable_us"]["typ"]))
    pc = patch_check(ap_root, sclk)
    req = A["osd_driver"]["required_delay_us"]
    if not pc["present"]:
        gate("N1 AT7456E NVM read delay patch %s" % PATCH_REL, False,
             "ABSENT: upstream single-burst sequence is the only one available -> font check fails on AT7456E")
    elif pc.get("applies") is None:
        gate("N1 AT7456E NVM read delay patch present (content-only, no tree)",
             pc["touches_only_osd"] and all(int(v) >= req for v in pc["added_delay_macros"].values()) and
             len(pc["added_delay_macros"]) > 0,
             "macros %s; %s" % (pc["added_delay_macros"], pc.get("why")))
    else:
        pt = pc.get("patched_timing", {"ok": False, "why": "patch does not apply"})
        ok = (pc["applies"] and pc["touches_only_osd"] and pc.get("upstream_file_unmodified") is not False and
              pt.get("ok") and pc.get("other_functions_unchanged") and
              all(w >= req for w in pt.get("waits_between_cmm_and_cmdo_us", [])) and
              len(pt.get("waits_between_cmm_and_cmdo_us", [])) >= 1 and
              pt["cmm_to_first_cmdo_request_us"] >= at["nvm_read_us"]["typ"] and pt["rx_index_ok"])
        gate("N1 patch 0003 applies to the unmodified upstream %s and waits >= %.0f us between CMM = 0x50 and "
             "the first CMDO" % (OSD_CPP, req), ok,
             "git apply --check rc %s (tree %s, file unmodified %s); slices %s; CMM->CMDO %.1f us worst "
             "(DS typ %.0f); VM0->CMAH %s us (DS typ %.0f); rx parse %s == expected %s"
             % (pc.get("git_apply_check_tree", {}).get("rc"), pc.get("tree_head", "?")[:10],
                pc.get("upstream_file_unmodified"), pt.get("slices"), pt.get("cmm_to_first_cmdo_request_us", -1),
                at["nvm_read_us"]["typ"],
                "%.1f" % pt["vm0_to_cmah_us"] if pt.get("vm0_to_cmah_us") is not None else None,
                at["osd_disable_us"]["typ"], pt.get("rx_parse"), pt.get("rx_expected")))
        if pt.get("ok") and (pt.get("vm0_to_cmah_us") or 0) < at["osd_disable_us"]["typ"]:
            warn("N1b patch does not wait after VM0 = 0", "runtime font change: first char re-written")
    ev = firmware_evidence(ap_root)
    boot_check_ms = bw["boot_font_check_ms_no_rewrite"] + A["osd_driver"]["nvm_chars"] * 2 * req / 1e3

    print("== 4. video network ==")
    vid = video_network()
    vi, vo, vr = vid["input"], vid["output"], vid["osd_rail"]
    gate("V1 VIN: 75 R termination + 100 nF clamp coupling (DS p.10), level inside 0.5..1.2 Vpp, clamp peak "
         "< AVDD", A["video"]["r27_ohm"] == 75.0 and abs(vi["coupling_uF"] - at["vin_coupling_uF_required"]) < 1e-9
         and vi["range_vpp"][0] <= vi["vin_pp_V"] <= vi["range_vpp"][1] and vi["vin_peak_V_at_1V2pp"] < vr["V_osd_nom_V"],
         "%.2f Vpp at VIN, sync tip %.2f V, peak %.2f V at 1.2 Vpp; clamp settles in %.1f ms (32 lines)"
         % (vi["vin_pp_V"], vi["clamp_sync_tip_V"], vi["vin_peak_V_at_1V2pp"], vi["clamp_settle_ms_PAL"]))
    gate("V2 VOUT: 2x gain into 75 R back-termination + 75 R load = 1 Vpp; swing <= 2.4 Vpp min",
         vo["vout_pp_V"] <= vo["swing_min_vpp"] and abs(vo["load_vpp_V"] - 1.0) < 0.02 and
         abs(vo["back_termination_gamma"]) < 0.01,
         "VOUT %.2f Vpp (1.2 Vpp in at max gain: %.2f Vpp), load %.3f Vpp, |Gamma| %.4f"
         % (vo["vout_pp_V"], vo["vout_pp_at_1V2_in_max_gain_V"], vo["load_vpp_V"], vo["back_termination_gamma"]))
    gate("V3 COUT = CSAG = 22 uF -> LTD <= 0.5 % (DS Table 2: 0.4 %; with DC-bias derating)",
         vo["LTD_dc_bias_pct"] <= vo["LTD_limit_pct"],
         "nominal %.2f %%, derated %.1f uF -> %.2f %%, -20 %% tol %.1f uF -> %.2f %%"
         % (vo["LTD_nominal_pct"], vo["COUT_eff_uF"], vo["LTD_dc_bias_pct"], vo["COUT_eff_low_uF"],
            vo["LTD_dc_bias_tol_pct"]))
    if vo["LTD_dc_bias_tol_pct"] > vo["LTD_limit_pct"]:
        warn("V3b LTD at -20 % capacitance tolerance", "%.2f %% > %.1f %% (cosmetic line tilt); 47 uF parts give 0.3 %%"
             % (vo["LTD_dc_bias_tol_pct"], vo["LTD_limit_pct"]))
    gate("V4 +3V3_OSD >= 3.15 V (AT7456E min) at nominal +3V3 incl. FB5 drop; video top < PVDD",
         vr["V_osd_nom_V"] >= at["vdd_min"] and vo["headroom_to_PVDD_min_V"] > 0,
         "%.3f V at 72.5 mA x 0.39 R; video top %.2f V; +3V3 must hold within -%.1f %% (headroom at that "
         "corner %.2f V)" % (vr["V_osd_nom_V"], vo["vout_top_V"], vr["rail_tol_needed_frac"] * 100,
                             vo["headroom_to_PVDD_min_V"]))
    if vr["V_osd_min_at_rail_tol_V"] < at["vdd_min"]:
        warn("V4b +3V3 tolerance vs AT7456E 3.15 V minimum",
             "at the repo's generic -5 %% rail corner +3V3_OSD = %.3f V < 3.15 V; the +3V3 regulator must be "
             "<= -%.1f %% (typ. LDO/buck +/-2..3 %% is fine)" % (vr["V_osd_min_at_rail_tol_V"],
                                                                 vr["rail_tol_needed_frac"] * 100))

    print("== 5. BEC (TPS62933) ==")
    v9n, v12n = fb_nominal(9), fb_nominal(12)
    q1 = q1_analysis()
    c9 = fb_corners(9, leak_A=A["q1"]["idss_25C_A"])
    c12 = fb_corners(12, rds_ohm=q1["rds_on_bound_ohm"])
    gate("R1 FB divider nominal 8.96 V / 12.02 V (Vout = 0.8 (1 + RTOP/RBOT), DS Eq. 1)",
         abs(v9n - 8.96) < 0.01 and abs(v12n - 12.02) < 0.01, "%.3f V / %.3f V" % (v9n, v12n))
    vmin_vtx = max(v["vin"][0] for v in A["vtx"].values())
    gate("R2 9 V corners (Vref 784..816 mV, 1 % R, IFB, Q1 IDSS) inside every VTX input range",
         c9[0] >= vmin_vtx and c9[1] <= min(v["vin"][1] for v in A["vtx"].values()),
         "%.3f .. %.3f V (VTX min %.1f V, HDZero max 12 V)" % (c9[0], c9[1], vmin_vtx))
    nonhd = [k for k in A["vtx"] if A["vtx"][k]["vin"][1] > 12.0]
    gate("R3 12 V corners inside the DJI / Walksnail input ranges",
         all(A["vtx"][k]["vin"][0] <= c12[0] and c12[1] <= A["vtx"][k]["vin"][1] for k in nonhd),
         "%.3f .. %.3f V" % (c12[0], c12[1]))
    if c12[1] > A["vtx"]["HDZero_Race_V3"]["vin"][1]:
        warn("R3b HDZero Race V3 (4..12 V max) must not use the 12 V setting",
             "12 V mode reaches %.2f V worst case (%.2f V nominal) > 12 V; keep RELAY3 = 0 (9 V) for HDZero"
             % (c12[1], v12n))
    tr = transitions()
    gate("R4 12->9 V step trips OVP (non-latching, DS 9.3.11) and 9->12 V step stays above 65 % UVP",
         tr["12to9_OVP_trips"] and tr["9to12_above_UVP"] and tr["9to12_FB_over_Vref_max"] < tr["OVP_max"],
         "12->9 FB/Vref >= %.0f %% (> OVP %.0f %%): HS off until Vout < %.2f V (%.2f ms at Walksnail load, "
         "%.1f s unloaded); 9->12 FB/Vref %.1f..%.1f %% (> UVP %.0f %%, < OVP)"
         % (tr["12to9_FB_over_Vref_min"] * 100, tr["OVP_max"] * 100, tr["ovp_release_V"],
            tr["12to9_decay_ms_per_load"]["Walksnail_Avatar_HD_Pro"], tr["12to9_decay_s_unloaded_divider"],
            tr["9to12_FB_over_Vref_min"] * 100, tr["9to12_FB_over_Vref_max"] * 100, tr["UVP"] * 100))
    gate("Q1 2N7002 on: VGS >= Vth(max) + 0.5 V through R37/R38 at +3V3 -5 %; Rds error < 0.5 %",
         q1["overdrive_V"] >= 0.5 and abs(q1["err_12V_from_rds_V"]) / v12n < 0.005,
         "VGS %.2f V vs Vth max %.1f V, Rds <= %.0f R at %.0f uA -> %.1f mV on 12 V"
         % (q1["vgs_on_min_V"], q1["vth_max_V"], q1["rds_on_bound_ohm"], q1["id_on_A"] * 1e6,
            q1["err_12V_from_rds_V"] * 1e3))
    gate("Q2 2N7002 off: IDSS x RTOP error on 9 V < 1 % (25 C, 80 nA DS max); gate held < Vth(min)",
         q1["err_9V_leak_25C_V"] / v9n < 0.01 and q1["vg_off_highZ_V"] < q1["vth_min_V"] / 4,
         "%.1f mV at 25 C; hot-estimate (%s) %.0f nA -> %.0f mV; gate %.0f mV with GPIO high-Z"
         % (q1["err_9V_leak_25C_V"] * 1e3, A["q1"]["idss_hot_rule"], q1["idss_hot_A"] * 1e9,
            q1["err_9V_leak_hot_V"] * 1e3, q1["vg_off_highZ_V"] * 1e3))
    dt = dropout_table()
    print("    %-4s %-8s %6s %5s %7s  %s" % ("pack", "state", "VBAT", "mode", "Vout", "regime"))
    for r in dt["table"]:
        print("    %-4s %-8s %6.2f %5d %7.2f  %s" % (r["pack"], r["state"], r["VBAT_V"], r["mode_V"], r["Vout_V"],
                                                 r["regime"]))

    def row(p, s, m):
        return [r for r in dt["table"] if r["pack"] == p and r["state"] == s and r["mode_V"] == m][0]
    gate("D1 12 V regulated on 4S down to empty (3.2 V/cell) at 2 A, hot",
         all(row("4S", s, 12)["regime"].startswith("regulated") for s in ("full", "nominal", "empty")),
         "Vin_min 12 V/2 A hot: fixed %.2f V, foldback %.2f V; 1.2 MHz would need %.2f V (Dmax %.3f)"
         % (dt["Vin_min_V"]["12V_hot_fixed"], dt["Vin_min_V"]["12V_hot_foldback"],
            dt["Vin_min_12V_1200kHz_hot_V"], dt["Dmax_1200kHz"]))
    gate("D2 9 V regulated on 3S and 4S down to empty at 2 A, hot; 2S passes through",
         all(row(p, s, 9)["regime"].startswith("regulated") for p in ("3S", "4S")
             for s in ("full", "nominal", "empty")) and row("2S", "nominal", 9)["regime"].startswith("dropout"),
         "Vin_min 9 V/2 A hot: fixed %.2f V, foldback %.2f V; 2S nominal -> %.2f V"
         % (dt["Vin_min_V"]["9V_hot_fixed"], dt["Vin_min_V"]["9V_hot_foldback"], row("2S", "nominal", 9)["Vout_V"]))
    il = A["interlock"]
    gate("D3 12 V interlock: allow >= %.1f V covers fixed-frequency Vin_min, drop at %.1f V >= foldback Vin_min"
         % (il["allow_12V_V"], il["drop_12V_V"]),
         il["allow_12V_V"] >= dt["Vin_min_V"]["12V_hot_fixed"] and il["drop_12V_V"] >= dt["Vin_min_V"]["12V_hot_foldback"]
         and il["drop_12V_V"] < il["allow_12V_V"],
         "fixed %.2f V, foldback %.2f V (hot, 2 A); hysteresis %.1f V"
         % (dt["Vin_min_V"]["12V_hot_fixed"], dt["Vin_min_V"]["12V_hot_foldback"],
            il["allow_12V_V"] - il["drop_12V_V"]))
    lc = inductor_caps()
    gate("L1 inductor peak at 2 A (L -20 %%, 4S full) < Isat %.1f A; K at VIN_MAX inside 20..60 %% (DS 10.2.2.7)"
         % lc["Isat_A"], lc["IL_peak_max_A"] < lc["Isat_A"] and
         lc["K_nominal_range"][0] >= A["bec"]["k_ripple_window"][0] - 1e-9 and
         lc["K_nominal_range"][1] <= A["bec"]["k_ripple_window"][1],
         "peak %.2f A; K %.2f..%.2f of 3 A; min ripple %.2f A >= %.2f A (subharmonic rule)"
         % (lc["IL_peak_max_A"], lc["K_nominal_range"][0], lc["K_nominal_range"][1], lc["ripple_min_A_nominal"],
            lc["ripple_min_required_A"]))
    if lc["Isat_A"] < lc["IHS_LIMIT_max_A"]:
        warn("L1b Isat < IHS_LIMIT max", "PNLS6045-100M Isat %.1f A < %.1f A high-side limit: an output short can "
             "saturate L1 before hiccup (DS 10.2.2.7 'ideally Isat >= IHS_LIMIT'); valley limit + hiccup still "
             "protect the IC. Higher-Isat 6x6 10 uH part if short-circuit robustness is required"
             % (lc["Isat_A"], lc["IHS_LIMIT_max_A"]))
    gate("K1 effective Cout (3 x 22 uF, -10 %% tol, DC bias) >= TI min %d uF at 12 V and 9 V (Table 10-2)"
         % lc["Cout_DS_min_uF"], min(lc["Cout_eff_uF"].values()) >= lc["Cout_DS_min_uF"],
         "12 V %.1f uF, 9 V %.1f uF (TI typ %d uF); ripple %.1f / %.1f mVpp; tSS %.1f..%.1f ms, inrush %.0f mA"
         % (lc["Cout_eff_uF"][12], lc["Cout_eff_uF"][9], lc["Cout_DS_typ_uF"], lc["Vout_ripple_mVpp"][12],
            lc["Vout_ripple_mVpp"][9], lc["tSS_ms"][0], lc["tSS_ms"][-1], lc["inrush_A"][12] * 1e3))
    vin_spike = 4 * A["battery"]["cell_V"]["full"] * (1 + A["battery"]["esc_overshoot_frac"])
    gate("K2 VIN: 4S + 30 % ESC overshoot <= 30 V recommended; CSS >= 6.8 nF",
         vin_spike <= A["bec"]["vin_rec_max_V"] and A["bec"]["css_F"] >= A["bec"]["css_min_F"],
         "%.1f V (abs max 32 V); Cin_eff %.1f uF, Icin %.2f A rms, Vin ripple %.0f mVpp"
         % (vin_spike, lc["Cin_eff_uF"], lc["Icin_rms_A"], lc["Vin_ripple_mVpp"]))
    th = thermal()
    gate("T1 U6 junction temperature at 2 A (hot Rds) <= %.0f C" % th["Tj_limit_C"],
         max(th["Tj_ambient_case_C"], th["Tj_hot_board_case_C"]) <= th["Tj_limit_C"],
         "P_ic %.2f W: 35 C amb x 60.2 K/W -> %.0f C; 105 C board x PsiJB 19.2 -> %.0f C"
         % (th["P_ic_max_W"], th["Tj_ambient_case_C"], th["Tj_hot_board_case_C"]))

    print("== 6. digital VTX connector ==")
    cn = connector()
    for r in cn:
        print("    %-24s %2d V  %.2f A  J1 %s%s" % (r["vtx"], r["mode_V"], r["I_A"], "ok" if r["J1_ok"] else "OVER",
                                                  "" if r["J1_ok"] else " -> J2 pads"))
    gate("J1 every VTX: BEC 2 A covers it; > 1 A/contact loads (DJI O3) are routed to J2 pads",
         all(r["bec_ok"] and (r["J1_ok"] or r["J2_ok"]) for r in cn),
         "J1 over 1 A: %s" % sorted({"%s@%dV %.2fA" % (r["vtx"], r["mode_V"], r["I_A"]) for r in cn if r["needs_J2"]}))

    print("== 7. BEC sense ADC ==")
    sn = sense()
    mult = hw["params"].get("BATT2_VOLT_MULT")
    gate("S1 R40/R41 ratio %.1f == BATT2_VOLT_MULT; 12 V -> %.3f V; full scale %.1f V > 4S %.1f V"
         % (sn["ratio"], sn["V_adc_at_12V"], sn["fullscale_input_V"], sn["VBAT_max_V"]),
         mult is not None and abs(float(mult) - sn["ratio"]) < 1e-6 and sn["fullscale_input_V"] > sn["VBAT_max_V"],
         "LSB %.1f mV at +VTX, ratio tol +/-%.2f %%, tau %.3f ms" % (sn["lsb_input_mV"], sn["ratio_err_frac"] * 100,
                                                                     sn["tau_ms"]))
    gate("S2 supervisor ADC check after %.0f ms >= soft start %.1f ms + 0.1 %% settle %.2f ms"
         % (A["sense"]["fw_settle_ms"], lc["tSS_ms"][-1], sn["settle_0p1pct_ms"]),
         A["sense"]["fw_settle_ms"] >= lc["tSS_ms"][-1] + sn["settle_0p1pct_ms"], "CSS 33 nF, ISS 4.5 uA min")
    if sn["total_err_worst_frac"] > sn["fw_window_frac"]:
        warn("S3 +/-6 % rail window vs worst-case error",
             "rail %.1f %% + divider %.1f %% + VDDA %.1f %% = %.1f %% worst (RSS %.1f %%) > %.0f %%: use >= +/-%.0f %% "
             "or a tighter +3V3 spec" % (max(sn["rail_err_frac"].values()) * 100, sn["ratio_err_frac"] * 100,
                                         sn["vdda_err_frac"] * 100, sn["total_err_worst_frac"] * 100,
                                         sn["total_err_rss_frac"] * 100, sn["fw_window_frac"] * 100,
                                         sn["recommended_window_frac"] * 100))

    print("== 8. boot state ==")
    gen = generated_header_check(ap_root) if ap_root else None
    b = A["bec"]
    en_low = A["mcu_gpio"]["voh_drop_V"] + b["en_ip_A"] * b["r_en_series"]
    en_high = A["rail_3v3"]["nom"] * (1 - A["rail_3v3"]["tol_frac"]) - b["en_ip_A"] * 0
    p = hw["params"]
    boot = [
        {"phase": "power-on / reset", "VTX_PWR (PD10)": "analog high-Z (RM0433 GPIOx_MODER reset)",
         "EN": "internal pull-up %.1f uA vs pin leakage <= %.2f uA -> ON" % (b["en_ip_A"] * 1e6,
                                                                            A["mcu_gpio"]["ilkg_A"] * 1e6),
         "VTX_12V_SEL (PD11)": "analog high-Z", "Q1": "R38 100k pull-down -> OFF -> 9 V"},
        {"phase": "bootloader", "VTX_PWR (PD10)": (gen or {}).get("bl", {}).get("PD10") or
         ("not in hwdef-bl.dat -> INPUT FLOATING" if not hw["bl_has_vtx_pins"] else "configured"),
         "EN": "ON", "VTX_12V_SEL (PD11)": (gen or {}).get("bl", {}).get("PD11") or "INPUT FLOATING",
         "Q1": "OFF -> 9 V"},
        {"phase": "firmware early init (hwdef.h VAL_GPIOD_*)", "VTX_PWR (PD10)": "OUTPUT %s" % (hw["VTX_PWR"] or {}).get("level"),
         "EN": "ON (3.3 V)", "VTX_12V_SEL (PD11)": "OUTPUT %s" % (hw["VTX_12V_SEL"] or {}).get("level"),
         "Q1": "OFF -> 9 V"},
        {"phase": "AP_Relay::init() (RELAYn_DEFAULT)", "VTX_PWR (PD10)": "RELAY2_DEFAULT %s" % p.get("RELAY2_DEFAULT"),
         "EN": "ON" if p.get("RELAY2_DEFAULT") == "1" else "OFF (!)",
         "VTX_12V_SEL (PD11)": "RELAY3_DEFAULT %s" % p.get("RELAY3_DEFAULT"),
         "Q1": "OFF -> 9 V" if p.get("RELAY3_DEFAULT") == "0" else "ON -> 12 V (!)"},
    ]
    for r in boot:
        print("    %-42s PD10 %-40s PD11 %s" % (r["phase"], r["VTX_PWR (PD10)"], r["VTX_12V_SEL (PD11)"]))
    gate("P1 VTX powered and at 9 V in every boot phase (reset, bootloader, early init, AP_Relay::init)",
         all(r["EN"].startswith("ON") or "ON" in r["EN"].split()[-1] for r in boot) and
         all("9 V" in r["Q1"] for r in boot) and not hw["bl_has_vtx_pins"] and
         A["mcu_gpio"]["ilkg_A"] < b["en_ip_A"] and en_low < b["en_fall_V"][0] and
         b["en_rise_V"][1] < en_high <= b["en_rec_max_V"],
         "EN low %.1f mV < %.2f V, EN high %.2f V in %.2f..%.1f V; R38 holds Q1 gate at %.0f mV"
         % (en_low * 1e3, b["en_fall_V"][0], en_high, b["en_rise_V"][1], b["en_rec_max_V"],
            q1["vg_off_highZ_V"] * 1e3))

    print("== 9. ArduPilot evidence / hwdef ==")
    if ev["checked"]:
        gate("F1 SPIDevice.cpp derive_freq_flag_bus: bus_clocks[busid-1]/2, halve while > request (i<7), "
             "MBR = i; SPI2_CLOCK = STM32_SPI2CLK; ctor SPEED_LOW",
             ev["spi_bus_clock_half"] and ev["spi_halving_loop"] and ev["spi_h7_mbr"] and
             ev["spi2_clock_is_STM32_SPI2CLK"] and ev["spi_ctor_speed_low"],
             "%s, %s" % (ev["cites"]["derive_freq_flag_bus"], ev["cites"]["halving_loop"]))
        gate("F2 stm32h7_mcuconf.h SPI123SEL = PLL1_Q_CK, PLL1 (24 MHz, 480 MHz) M3 N120 Q12; "
             "hal_lld_type1.h SPI2CLK = PLL1_Q",
             clk["spi123sel"] == "STM32_SPI123SEL_PLL1_Q_CK" and (clk["pll1_m"], clk["pll1_n"], clk["pll1_q"]) ==
             (3, 120, 12) and clk["cites"].get("SPI2CLK=PLL1_Q") is not None,
             "%s, %s, %s" % (clk["cites"].get("SPI123SEL"), clk["cites"].get("PLL1_480"),
                             clk["cites"].get("SPI2CLK=PLL1_Q")))
        ul = ev["osd_upstream_layout"]["layout"]
        gate("F3 AP_OSD_MAX7456 check_font_char: VM0, CMAH, CMM=READ_NVR, 54 x (CMAL, CMDO) in ONE "
             "transfer_fullduplex, rx from index 9 step 4; no set_speed(); 512 B buffer; 100 ms thread",
             ev["osd_single_burst"] and ev["osd_rx_offset_9_stride_4"] and ev["osd_no_set_speed"] and
             ev["osd_spi_buffer_512"] and ev["osd_frame_stop"] and ev["osd_thread_100ms"] and
             ul.get("MAX7456ADD_CMM") == 4 and ul.get("first_CMDO_addr") == 8 and
             ev["fullduplex_inplace_do_transfer"],
             "%s; layout %s; %s" % (ev["cites"]["check_font_char"], ul, ev["cites"]["osd_thread_delay"]))
        gate("F4 AP_Relay: RELAYn_PIN_DEFAULT #ifndef; convert_params -> FUNCTION Relay when pin >= 0; init() "
             "drives RELAYn_DEFAULT (param default OFF) -> RELAY2_DEFAULT 1 required",
             ev["relay_pin_default_ifndef"] and ev["relay_convert_sets_RELAY_when_pin_valid"] and
             ev["relay_init_drives_default"] and ev["relay_DEFAULT_param_default_off"] and
             ev["relay_gpio_output_write"],
             "%s, %s" % (ev["cites"]["relay_init"], ev["cites"]["relay_DEFAULT_param"]))
        gate("F5 ADC: STM32H743xx.py ADC1 PC5 -> channel 8; AnalogIn 3.3 V full scale; BattMonitor VOLT_PIN/MULT; "
             "delay_microseconds = chThdSleep at 1 MHz tick",
             ev["adc1_PC5_channel"] == 8 and ev["adc_voltage_scaling_3v3"] and ev["batt_volt_pin_mult_params"] and
             ev["delay_us_chThdSleep"] and ev["st_frequency_hz"] == 1000000,
             "%s, %s, %s" % (ev["cites"]["ADC1_map_PC5"], ev["cites"]["VOLTAGE_SCALING"],
                             ev["cites"]["delay_microseconds"]))
    else:
        print("  [SKIP] F1..F5 ArduPilot source evidence (no --ardupilot / ARDUPILOT_ROOT)")
    sp = hw["spidev_osd"]
    gate("A1 hwdef SPIDEV osd SPI2 OSD1_CS MODE0 8*MHZ 8*MHZ; PB12 OSD1_CS; SPI2 on PB13/14/15; OSD type 1",
         len(sp) == 1 and sp[0] == ("MODE0", "8*MHZ", "8*MHZ") and hw["OSD1_CS"] and len(hw["spi2_pins"]) == 3 and
         hw["OSD_ENABLED"] == ["1"] and hw["HAL_OSD_TYPE_DEFAULT"] == ["1"] and hw["bl_osd_cs"],
         "%s, SPI2 %s" % (sp, hw["spi2_pins"]))
    vp, vs = hw["VTX_PWR"] or {}, hw["VTX_12V_SEL"] or {}
    gate("A2 hwdef PD10 VTX_PWR OUTPUT HIGH GPIO(81), PD11 VTX_12V_SEL OUTPUT LOW GPIO(82), RELAY2/3_PIN_DEFAULT "
         "81/82; defaults RELAY2_DEFAULT 1, RELAY3_DEFAULT 0",
         vp.get("level") == "HIGH" and vp.get("gpio") == 81 and vs.get("level") == "LOW" and vs.get("gpio") == 82 and
         hw["RELAY2_PIN_DEFAULT"] == ["81"] and hw["RELAY3_PIN_DEFAULT"] == ["82"] and
         p.get("RELAY2_DEFAULT") == "1" and p.get("RELAY3_DEFAULT") == "0" and
         p.get("RELAY2_PIN") is None and p.get("RELAY3_PIN") is None,
         "PD10 %s, PD11 %s, RELAY_PIN_DEFAULT %s/%s, RELAY_DEFAULT %s/%s"
         % (vp.get("tokens"), vs.get("tokens"), hw["RELAY2_PIN_DEFAULT"], hw["RELAY3_PIN_DEFAULT"],
            p.get("RELAY2_DEFAULT"), p.get("RELAY3_DEFAULT")))
    ch = ev.get("adc1_PC5_channel", 8)
    gate("A4 hwdef PC5 VTX_VSENSE ADC1 SCALE(1); defaults BATT2_MONITOR 1, BATT2_VOLT_PIN %d, BATT2_VOLT_MULT %.1f"
         % (ch, sn["ratio"]),
         hw["VTX_VSENSE"] == [("ADC1", "1")] and p.get("BATT2_MONITOR") == "1" and
         p.get("BATT2_VOLT_PIN") == str(ch) and mult is not None and abs(float(mult) - sn["ratio"]) < 1e-9,
         "%s, params %s" % (hw["VTX_VSENSE"], {k: p[k] for k in ("BATT2_MONITOR", "BATT2_VOLT_PIN", "BATT2_VOLT_MULT")}))
    gate("A5 hwdef PE7 UART7_RX UART7 NODMA (no TX), SERIAL_ORDER ends with UART7 (SERIAL4)",
         len(hw["UART7_RX"]) == 1 and "NODMA" in hw["UART7_RX"][0].split() and not hw["UART7_TX_present"] and
         hw["SERIAL_ORDER"][-1:] == ["UART7"] and hw["SERIAL_ORDER"].index("UART7") == 4,
         "PE7 %s, SERIAL_ORDER %s" % (hw["UART7_RX"], hw["SERIAL_ORDER"]))
    if ap_root:
        g = gen
        ok = (g.get("rc") == 0 and len(g.get("spidev_osd", [])) == 1 and
              g["spidev_osd"][0][2:] == ("PAL_LINE(GPIOB,12U)", "SPIDEV_MODE0", "8*MHZ", "8*MHZ") and
              g["gpio81"] == [("true", "0", "PAL_LINE(GPIOD,10U)")] and
              g["gpio82"] == [("true", "0", "PAL_LINE(GPIOD,11U)")] and
              sorted(g["relay_pin_defaults"]) == [("RELAY2_PIN_DEFAULT", "81"), ("RELAY3_PIN_DEFAULT", "82")] and
              g["PD10"] == {"MODER": "PIN_MODE_OUTPUT(10U)", "ODR": "PIN_ODR_HIGH(10U)"} and
              g["PD11"] == {"MODER": "PIN_MODE_OUTPUT(11U)", "ODR": "PIN_ODR_LOW(11U)"} and
              g["PC5_MODER"] == "PIN_MODE_ANALOG(5U)" and
              sorted(g["uart7_dma"]) == [("RX", "false, 0, 0"), ("TX", "false, 0, 0")] and
              not g["uart7_rx_stream"] and g["serial_list"][-1:] == ["UART7"] and
              ("8", "8", "3.30/4096", "PC5", "VTX_VSENSE") in g["analog_pins"] and len(g["spi2_dma"]) == 2 and
              g.get("bl_rc") == 0 and g.get("bl", {}).get("PD10", {}).get("MODER") == "PIN_MODE_INPUT(10U)" and
              g.get("bl", {}).get("PD11", {}).get("MODER") == "PIN_MODE_INPUT(11U)" and
              g.get("bl", {}).get("PB12", {}).get("ODR") == "PIN_ODR_HIGH(12U)")
        gate("A3 generated hwdef.h: SPIDesc osd 8*MHZ MODE0, GPIO 81/82 on PD10/PD11 (OUTPUT HIGH/LOW), UART7 NODMA, "
             "ADC channel 8 = PC5; bootloader leaves PD10/PD11 INPUT, PB12 high",
             ok, "rc %s/%s, osd %s, gpio81 %s, gpio82 %s, uart7 %s, adc %s, bl %s"
             % (g.get("rc"), g.get("bl_rc"), g.get("spidev_osd"), g.get("gpio81"), g.get("gpio82"),
                g.get("uart7_dma"), [a for a in g.get("analog_pins", []) if a[0] == "8"], g.get("bl")))
    else:
        print("  [SKIP] A3 generated hwdef.h check (no --ardupilot / ARDUPILOT_ROOT)")
    lua = lua_crosscheck()
    if lua["present"] and not (lua["has_13p6"] and lua["has_13p0"]):
        warn("LUA vtx_bec.lua thresholds", "13.6 / 13.0 V literals not found; check against interlock %s" % il)

    nfail = sum(1 for g_ in GATES if not g_["pass"])
    report = {
        "revision": "13.0",
        "title": "AT7456E OSD on SPI2, 6-pin digital VTX connector and switchable 9 V / 12 V 2 A VTX BEC (U5, U6, J1)",
        "result": "PASS" if nfail == 0 else "FAIL",
        "gates": GATES,
        "findings": FINDINGS,
        "warnings": WARNS,
        "assumptions": A,
        "spi_clock": {"kernel": clk, "divisor": div, "divisor_highspeed": div_hi, "stock_10MHz": stock,
                      "rate_table": table, "timing": tim, "timing_10MHz": tim10},
        "osd_bandwidth": bw,
        "nvm_read": {"upstream_5MHz": up5, "upstream_10MHz": up10, "patch": pc,
                     "boot_font_check_ms_patched_no_rewrite": boot_check_ms,
                     "verdict_upstream": "FAIL (finding, recorded - not a gate)",
                     "verdict_patched": "PASS" if any(g_["gate"].startswith("N1") and g_["pass"] for g_ in GATES)
                     else "FAIL"},
        "video": vid,
        "bec": {"Vout_nominal_V": {"9": v9n, "12": v12n},
                "Vout_corners_V": {"9": c9, "12": c12}, "q1": q1, "transitions": tr,
                "dropout": dt, "inductor_caps": lc, "thermal": th},
        "connector": cn,
        "sense": sn,
        "boot_state": boot,
        "firmware_evidence": ev,
        "hwdef": hw,
        "generated_hwdef_h": gen,
        "lua_crosscheck": lua,
        "schematic_values": {   # for tools/gen_mcu_core_sch.py sheet notes (fallback when absent)
            "osd_sclk_MHz": sclk / 1e6, "vtx_9V": round(v9n, 2), "vtx_12V": round(v12n, 2),
            "vtx_9V_range": [round(c9[0], 2), round(c9[1], 2)], "vtx_12V_range": [round(c12[0], 2), round(c12[1], 2)],
            "vin_min_12V_V": round(dt["Vin_min_V"]["12V_hot_fixed"], 2), "sense_ratio": sn["ratio"],
            "tss_ms": round(lc["tSS_ms"][1], 1), "J1_contact_A": A["connector"]["contact_A"]},
    }
    out = os.path.join(HERE, "osd_vtx_bec_report.json")
    with open(out, "w") as fh:
        json.dump(report, fh, indent=1, default=str)
    print("\nwrote %s" % out)
    print("RESULT: %s (%d gates, %d failed, %d findings, %d warnings)"
          % (report["result"], len(GATES), nfail, len(FINDINGS), len(WARNS)))
    return 1 if nfail else 0


if __name__ == "__main__":
    sys.exit(main())
