#!/usr/bin/env python3
"""
Nano H743 AIO - AT7456E OSD / digital VTX BEC firmware validator (Revision 13.0)

Checks the Rev 13.0 ArduPilot integration against a real ArduPilot tree
instead of against comments:

  V1  chibios_hwdef.py generates hwdef.h for ardupilot/hwdef.dat and
      hwdef-bl.dat.  In the bootloader PB12 OSD1_CS is a driven-high output
      and PD10 VTX_PWR / PD11 VTX_12V_SEL are left INPUT + floating (U6 EN
      internal pull-up -> BEC on; Q1 gate pull-down -> 9 V).
  V2  The OSD SPIDEV is on SPI2 / OSD1_CS, SPIDEV_MODE0, 8 MHz low and high
      speed.  The SPI2 kernel clock is taken from the real stm32h7_mcuconf.h
      (preprocessed with the generated hwdef.h) and the SCLK is resolved
      with ArduPilot's SPIDevice::derive_freq_flag_bus() divisor search
      (statements checked verbatim): 5.000 MHz, tCP/tCH/tCL vs AT7456E DS p.4.
  V3  GPIO 81 = PD10 (output, ODR HIGH) and GPIO 82 = PD11 (output, ODR LOW)
      in HAL_GPIO_PINS; RELAY2/3_PIN_DEFAULT 81/82 reach hwdef.h and are
      honoured by AP_Relay.cpp; RELAY2/3_DEFAULT match the reset levels.
  V4  PC5 is ADC1 channel 8 in HAL_ANALOG_PINS (and in STM32H743xx.py
      ADC1_map); UART7 is SERIAL4 with RX-only and no DMA; the serial port
      count equals SERIAL_ORDER.
  V5  Every Rev 13 defaults.parm name (RELAY2/3_DEFAULT, BATT2_MONITOR,
      BATT2_VOLT_PIN, BATT2_VOLT_MULT) exists in the tree (@Group prefix +
      @Param) and its value is valid; BATT2_VOLT_PIN is an ADC channel of
      this board and BATT2_VOLT_MULT equals the R40/R41 divider.
  V6  ardupilot/patches/0003-*.patch applies with git apply --check to a
      temporary copy of AP_OSD_MAX7456.cpp (the tree is never modified),
      the patched file is structurally sound, and the UPSTREAM and PATCHED
      check_font_char() / buffer_add_cmd() are extracted verbatim, compiled
      against an AT7456E SPI timing simulator (NVM read 22/30 us, display-
      off 30 us, DS p.36) and run: upstream must fail, patched must pass,
      and both must pass with MAX7456 timing.
  V7  ardupilot/scripts/vtx_bec.lua only calls bindings that exist in
      AP_Scripting/docs/docs.lua, compiles with luac (system luac or one
      built from the tree's own Lua 5.3 sources into a temp dir) and its
      interlock behaviour passes a closed-loop simulation run in that Lua
      (mocked relay/battery/arming/param/gcs/rc bindings and a BEC plant).
  V8  tools/validate_dma.py still passes with UART7 added.

Usage:
  python3 tools/validate_osd_vtx.py --ardupilot /path/to/ardupilot
  (or set ARDUPILOT_ROOT)

Exit status 0 = all checks pass.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "tools"))
import validate_dma as vdma  # noqa: E402

HWDEF = os.path.join(REPO, "ardupilot", "hwdef.dat")
HWDEF_BL = os.path.join(REPO, "ardupilot", "hwdef-bl.dat")
PARM = os.path.join(REPO, "ardupilot", "defaults.parm")
PATCH = os.path.join(REPO, "ardupilot", "patches", "0003-osd-at7456e-nvm-read-delay.patch")
LUA_SCRIPT = os.path.join(REPO, "ardupilot", "scripts", "vtx_bec.lua")

OSD_SRC = "libraries/AP_OSD/AP_OSD_MAX7456.cpp"

# AT7456E-S402-V1.1 limits
AT7456E = {
    "f_sclk_max_hz": 10e6,     # p.15 "up to 10 MHz"
    "tCP_min_ns": 100.0,       # p.4 SCLK period
    "tCH_min_ns": 40.0,        # p.4 SCLK high
    "tCL_min_ns": 40.0,        # p.4 SCLK low
    "nvm_read_min_us": 22.0,   # p.36 table, CMM = 0x50 -> STAT[5] = 0, min
    "nvm_read_typ_us": 30.0,   # p.36 table, typ (note 3: design fixed delays to typ)
    "osd_off_typ_us": 30.0,    # p.36 table, VM0[3] = 0 -> STAT[5] = 0, typ
}
# Rev 13 BEC sense divider (frozen spec B: R40 10k / R41 1k)
R40, R41 = 10e3, 1e3
REV13_PARAMS = ["RELAY2_DEFAULT", "RELAY3_DEFAULT", "BATT2_MONITOR",
                "BATT2_VOLT_PIN", "BATT2_VOLT_MULT"]

# defaults.parm prefix -> (top-level object prefix, group file, subgroup file)
# Top-level names: ArduCopter/Parameters.cpp GOBJECT(relay, "RELAY") and
# GOBJECT(battery, "BATT"); instance subgroups "2_", "3_" from
# AP_Relay.cpp / AP_BattMonitor.cpp @Group lines.
PARAM_GROUPS = [
    ("RELAY", "libraries/AP_Relay/AP_Relay.cpp",
     ["libraries/AP_Relay/AP_Relay_Params.cpp"]),
    ("BATT", "libraries/AP_BattMonitor/AP_BattMonitor.cpp",
     ["libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp",
      "libraries/AP_BattMonitor/AP_BattMonitor_Analog.cpp"]),
]

FAILS = []


def check(ok, msg):
    print("  [%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        FAILS.append(msg)


def skip(msg):
    print("  [SKIP] %s" % msg)


def read(path):
    with open(path) as fh:
        return fh.read()


def strip_hash_comments(text):
    return [ln.split("#", 1)[0].strip() for ln in text.splitlines()]


def load_params():
    params = {}
    for line in strip_hash_comments(read(PARM)):
        if line:
            k, v = line.split()[:2]
            params[k] = float(v)
    return params


def gpio_reg_bits(hdr, port, reg, pin):
    """Return the macro name (e.g. PIN_ODR_HIGH) used for <pin> of
    VAL_GPIO<port>_<reg> in the generated hwdef.h."""
    m = re.search(r"#define VAL_GPIO%s_%s\s+\((.*?)\)\s*\n\s*\n|#define VAL_GPIO%s_%s\s+\((.*?)\)\n"
                  % (port, reg, port, reg), hdr, re.S)
    if not m:
        return None
    body = m.group(1) or m.group(2)
    mm = re.search(r"(PIN_\w+)\(%dU\)" % pin, body)
    return mm.group(1) if mm else None


def extract_function(src, signature):
    i = src.index(signature)
    j = src.index("{", i)
    depth = 0
    for k in range(j, len(src)):
        if src[k] == "{":
            depth += 1
        elif src[k] == "}":
            depth -= 1
            if depth == 0:
                return src[i:k + 1]
    raise ValueError("unbalanced braces in %s" % signature)


def strip_c(src):
    """Remove C/C++ comments and string/char literals (for balance checks)."""
    out, i, n = [], 0, len(src)
    while i < n:
        c = src[i]
        if src.startswith("//", i):
            i = src.find("\n", i)
            i = n if i < 0 else i
        elif src.startswith("/*", i):
            j = src.find("*/", i + 2)
            i = n if j < 0 else j + 2
        elif c in "\"'":
            j = i + 1
            while j < n and src[j] != c:
                j += 2 if src[j] == "\\" else 1
            i = j + 1
        else:
            out.append(c)
            i += 1
    return "".join(out)


# ---------------------------------------------------------------------------
# V1 .. V4: generated hwdef.h
# ---------------------------------------------------------------------------
def generate(ap_root, tmp):
    bdir = vdma.stage(tmp)
    out_fw, out_bl = os.path.join(tmp, "fw"), os.path.join(tmp, "bl")
    os.makedirs(out_fw)
    os.makedirs(out_bl)
    rc, log = vdma.run_generator(ap_root, os.path.join(bdir, "hwdef.dat"), out_fw,
                                 params=os.path.join(bdir, "defaults.parm"))
    check(rc == 0, "V1 hwdef.dat generates (rc=%d)" % rc)
    if rc:
        print(log[-2000:])
        return None, None
    rc, log = vdma.run_generator(ap_root, os.path.join(bdir, "hwdef-bl.dat"), out_bl,
                                 bootloader=True)
    check(rc == 0, "V1 hwdef-bl.dat generates (rc=%d)" % rc)
    if rc:
        print(log[-2000:])
        return read(os.path.join(out_fw, "hwdef.h")), None
    return read(os.path.join(out_fw, "hwdef.h")), read(os.path.join(out_bl, "hwdef.h"))


def v1_bootloader(bl):
    print("V1 bootloader pin states")
    for port, pin, name in (("D", 10, "PD10 VTX_PWR"), ("D", 11, "PD11 VTX_12V_SEL")):
        mode = gpio_reg_bits(bl, port, "MODER", pin)
        pupd = gpio_reg_bits(bl, port, "PUPDR", pin)
        check(mode == "PIN_MODE_INPUT" and pupd == "PIN_PUPDR_FLOATING",
              "V1 bootloader %s high-Z (%s, %s)" % (name, mode, pupd))
    bl_src = strip_hash_comments(read(HWDEF_BL))
    check(not any(re.match(r"^P(D10|D11)\s", ln) for ln in bl_src),
          "V1 hwdef-bl.dat does not assign PD10/PD11")
    mode = gpio_reg_bits(bl, "B", "MODER", 12)
    odr = gpio_reg_bits(bl, "B", "ODR", 12)
    pupd = gpio_reg_bits(bl, "B", "PUPDR", 12)
    check(mode == "PIN_MODE_OUTPUT" and odr == "PIN_ODR_HIGH",
          "V1 bootloader PB12 OSD1_CS held high (%s, %s, %s)" % (mode, odr, pupd))


def mcuconf_defines(ap_root, tmp, fw_hdr):
    """Preprocess the real mcuconf.h with the generated hwdef.h."""
    common = os.path.join(ap_root, "libraries", "AP_HAL_ChibiOS", "hwdef", "common")
    d = os.path.join(tmp, "mcuconf")
    os.makedirs(d)
    with open(os.path.join(d, "hwdef.h"), "w") as fh:
        fh.write(fw_hdr)
    src = os.path.join(d, "m.c")
    with open(src, "w") as fh:
        fh.write('#include "hwdef.h"\n#include "mcuconf.h"\n')
    r = subprocess.run(["gcc", "-E", "-dM", "-I", d, "-I", common, src],
                       capture_output=True, text=True)
    if r.returncode:
        return None, r.stderr[-500:]
    defs = {}
    for m in re.finditer(r"^#define (\w+) (.*)$", r.stdout, re.M):
        defs[m.group(1)] = m.group(2).strip()
    return defs, ""


def c_int(s):
    return int(re.sub(r"[uUlL]+$", "", s.strip("() ")))


def v2_spi(ap_root, tmp, hdr):
    print("V2 OSD SPI device and resolved SCLK")
    m = re.search(r'SPIDesc\("osd"\s*,\s*(\d+),\s*(\d+),\s*PAL_LINE\(GPIO(\w),(\d+)U\)\s*,\s*'
                  r'(\w+),\s*(\d+)\*MHZ,\s*(\d+)\*MHZ\)', hdr)
    check(m is not None, "V2 hwdef.h declares SPIDesc(\"osd\")")
    if not m:
        return
    bus_idx, devid, cs_port, cs_pin, mode, lo, hi = m.groups()
    buses = re.search(r"#define HAL_SPI_BUS_LIST (.+)", hdr).group(1).split(",")
    bus_name = buses[int(bus_idx)].strip()
    bus_cfg = re.search(r"#define %s \{ &SPID(\d+), (\d+)," % bus_name, hdr)
    spi_n, busid = int(bus_cfg.group(1)), int(bus_cfg.group(2))
    check(spi_n == 2 and (cs_port, cs_pin) == ("B", "12"),
          "V2 osd on SPI%d (busid %d), CS P%s%s = OSD1_CS" % (spi_n, busid, cs_port, cs_pin))
    check(mode == "SPIDEV_MODE0", "V2 %s (AT7456E samples SDIN on rising SCLK, DS p.15)" % mode)
    check(int(lo) == 8 and int(hi) == 8, "V2 requested speed low %s / high %s MHz == 8 / 8" % (lo, hi))
    check(int(devid) == 4, "V2 DEVID4 as in hwdef.dat (devid %s)" % devid)

    osd_src = read(os.path.join(ap_root, OSD_SRC))
    check("set_speed" not in osd_src,
          "V2 AP_OSD_MAX7456 never calls set_speed(): LOW speed entry is the one used")

    spi_cpp = read(os.path.join(ap_root, "libraries", "AP_HAL_ChibiOS", "SPIDevice.cpp"))
    stmts = ["spi_clock_freq = bus_clocks[busid-1] / 2;",
             "while (spi_clock_freq > _frequency && i<7) {",
             "spi_clock_freq >>= 1;",
             "#define SPI2_CLOCK  STM32_SPI2CLK"]
    missing = [s for s in stmts if s not in spi_cpp]
    check(not missing, "V2 SPIDevice.cpp derive_freq_flag_bus() matches the modelled algorithm %s"
          % missing)

    defs, err = mcuconf_defines(ap_root, tmp, hdr)
    check(defs is not None, "V2 stm32h7_mcuconf.h preprocesses with the generated hwdef.h %s" % err)
    if not defs:
        return
    hse = c_int(defs["STM32_HSECLK"])
    pm, pn, pq = (c_int(defs["STM32_PLL1_%s_VALUE" % k]) for k in ("DIVM", "DIVN", "DIVQ"))
    sel = defs.get("STM32_SPI123SEL", "")
    check(sel == "STM32_SPI123SEL_PLL1_Q_CK", "V2 SPI1/2/3 kernel clock source %s" % sel)
    kernel = hse // pm * pn // pq
    check(kernel == 80000000, "V2 SPI2 kernel = %d / M%d x N%d / Q%d = %.3f MHz"
          % (hse, pm, pn, pq, kernel / 1e6))
    want = int(lo) * 1000000
    f = kernel // 2
    i = 0
    steps = [f]
    while f > want and i < 7:
        f >>= 1
        i += 1
        steps.append(f)
    t_cp = 1e9 / f
    print("      divisor search %s -> MBR %d, SCLK %.3f MHz" %
          (" -> ".join("%g" % (s / 1e6) for s in steps), i, f / 1e6))
    check(f == 5000000, "V2 resolved OSD SCLK == 5.000 MHz (got %.3f MHz)" % (f / 1e6))
    check(t_cp >= AT7456E["tCP_min_ns"] and f <= AT7456E["f_sclk_max_hz"],
          "V2 tCP %.0f ns >= %.0f ns (x%.1f margin)" % (t_cp, AT7456E["tCP_min_ns"],
                                                       t_cp / AT7456E["tCP_min_ns"]))
    check(t_cp / 2 >= AT7456E["tCH_min_ns"],
          "V2 tCH = tCL = %.0f ns >= %.0f ns (50 %% duty SPI master)" % (t_cp / 2, AT7456E["tCH_min_ns"]))
    # same request at 10 MHz (upstream default) for the record
    f10, j = kernel // 2, 0
    while f10 > 10000000 and j < 7:
        f10 >>= 1
        j += 1
    print("      (10*MHZ request would give %.3f MHz, tCP %.0f ns = zero margin)" % (f10 / 1e6, 1e9 / f10))


def v3_gpio(ap_root, hdr, params):
    print("V3 VTX GPIOs and relays")
    pins = {}
    for m in re.finditer(r"\{\s*(\d+),\s*(true|false),\s*(\d+),\s*PAL_LINE\(GPIO(\w),(\d+)U\)\}", hdr):
        pins.setdefault(int(m.group(1)), []).append((m.group(4), int(m.group(5))))
    for gpio, port, pin, odr_want, name in ((81, "D", 10, "PIN_ODR_HIGH", "VTX_PWR"),
                                            (82, "D", 11, "PIN_ODR_LOW", "VTX_12V_SEL")):
        check(pins.get(gpio) == [(port, pin)],
              "V3 GPIO(%d) is P%s%d %s, unique (%s)" % (gpio, port, pin, name, pins.get(gpio)))
        mode = gpio_reg_bits(hdr, port, "MODER", pin)
        odr = gpio_reg_bits(hdr, port, "ODR", pin)
        check(mode == "PIN_MODE_OUTPUT" and odr == odr_want,
              "V3 P%s%d reset state %s %s" % (port, pin, mode, odr))
    relay_cpp = read(os.path.join(ap_root, "libraries", "AP_Relay", "AP_Relay.cpp"))
    for n, gpio in ((2, 81), (3, 82)):
        check(re.search(r"#define RELAY%d_PIN_DEFAULT %d\b" % (n, gpio), hdr) is not None,
              "V3 hwdef.h RELAY%d_PIN_DEFAULT %d" % (n, gpio))
        check("#ifndef RELAY%d_PIN_DEFAULT" % n in relay_cpp and
              re.search(r"RELAY%d_PIN_DEFAULT,?\s" % n, relay_cpp.split("set_defaults()", 1)[-1]) is not None,
              "V3 AP_Relay.cpp set_defaults() consumes RELAY%d_PIN_DEFAULT" % n)
    check(params.get("RELAY2_DEFAULT") == 1.0,
          "V3 RELAY2_DEFAULT %s == 1 matches PD10 OUTPUT HIGH (AP_Relay::init() drives "
          "the pin to RELAYn_DEFAULT)" % params.get("RELAY2_DEFAULT"))
    check(params.get("RELAY3_DEFAULT") == 0.0,
          "V3 RELAY3_DEFAULT %s == 0 matches PD11 OUTPUT LOW (9 V)" % params.get("RELAY3_DEFAULT"))
    init = extract_function(relay_cpp, "void AP_Relay::init()")
    check("convert_params();" in init and "set_defaults();" in init,
          "V3 AP_Relay::init() applies pin defaults and converts RELAYn_FUNCTION")


def v4_adc_uart(ap_root, hdr):
    print("V4 ADC channel and UART7")
    m = re.search(r"#define HAL_ANALOG_PINS \\\n(.*?)\n\n", hdr, re.S)
    analog = re.findall(r"\{\s*(\d+),\s*(\d+),\s*[\d./]+\s*\},\s*/\* (\w+) (\w+) \*/", m.group(1)) if m else []
    chans = {a[2]: (int(a[0]), int(a[1]), a[3]) for a in analog}
    check(chans.get("PC5", (None,))[0] == 8 and chans["PC5"][2] == "VTX_VSENSE",
          "V4 HAL_ANALOG_PINS PC5 VTX_VSENSE -> pin %s" % (chans.get("PC5"),))
    mcu_py = read(os.path.join(ap_root, "libraries", "AP_HAL_ChibiOS", "hwdef", "scripts", "STM32H743xx.py"))
    adc1 = mcu_py.split("ADC1_map", 1)[1].split("}", 1)[0]
    mm = re.search(r'"PC5"\s*:\s*(\d+)', adc1)
    check(mm is not None and int(mm.group(1)) == 8, "V4 STM32H743xx.py ADC1_map PC5 = %s" %
          (mm.group(1) if mm else None))
    check(re.search(r"#define STM32_UART7_RX_DMA_CONFIG false", hdr) is not None and
          re.search(r"#define STM32_UART7_TX_DMA_CONFIG false", hdr) is not None,
          "V4 UART7 RX/TX DMA disabled (NODMA)")
    check(not re.search(r"STM32_UART_UART7_\w+_DMA_STREAM", hdr), "V4 UART7 consumes no DMA stream")
    cfg = re.search(r"#define HAL_UART7_CONFIG \{(.*)\}", hdr)
    if cfg:
        f = [x.strip() for x in cfg.group(1).split(",")]
        # { stream, n, usb, rxdma, txdma, tx_line, rx_line, ... } (chibios_hwdef.py)
        check("PAL_LINE(GPIOE" in cfg.group(1) and "7U)" in cfg.group(1),
              "V4 UART7 RX line PE7 (%s)" % [x for x in f if "PAL_LINE" in x])
    else:
        check(False, "V4 HAL_UART7_CONFIG present")
    order = []
    for ln in strip_hash_comments(read(HWDEF)):
        if ln.startswith("SERIAL_ORDER"):
            order = ln.split()[1:]
    n = re.search(r"#define HAL_NUM_SERIAL_PORTS (\d+)", hdr)
    check(n is not None and int(n.group(1)) == len(order),
          "V4 HAL_NUM_SERIAL_PORTS %s == len(SERIAL_ORDER %s)" % (n.group(1) if n else None, order))
    dl = re.search(r"#define HAL_SERIAL_DEVICE_LIST \\\n(.*?)\n\n", hdr, re.S)
    devs = re.findall(r"HAL_(\w+)_CONFIG", dl.group(1)) if dl else []
    check(devs == order and order[4:5] == ["UART7"],
          "V4 SERIAL4 = UART7 (device list %s)" % devs)
    check(re.search(r"PE7 UART7_RX UART7 AF7", hdr) is not None, "V4 PE7 UART7_RX on AF7")


# ---------------------------------------------------------------------------
# V5 parameters
# ---------------------------------------------------------------------------
def param_doc(ap_root, name):
    for top, group_file, sub_files in PARAM_GROUPS:
        if not name.startswith(top):
            continue
        rest = name[len(top):]
        m = re.match(r"(\d+_)(\w+)$", rest)
        if not m:
            return None, "no instance prefix"
        inst, leaf = m.groups()
        gtxt = read(os.path.join(ap_root, group_file))
        paths = re.findall(r"@Group: %s\s*\n\s*//\s*@Path: (\S+)" % re.escape(inst), gtxt)
        for sf in sub_files:
            if os.path.basename(sf) not in paths:
                continue
            txt = read(os.path.join(ap_root, sf))
            for b in re.split(r"//\s*@Param(?:\{[^}]*\})?:\s*", txt)[1:]:
                if b.split()[0] == leaf:
                    return sf, b.split("AP_GROUP", 1)[0]
        return None, "@Param %s not under @Group %s (paths %s)" % (leaf, inst, paths)
    return None, "no source mapping"


def value_ok(doc, val):
    vals = set()
    for line in re.findall(r"@Values(?:\{[^}]*\})?:\s*(.+)", doc):
        vals |= {float(k) for k in re.findall(r"(-?[\d.]+)\s*:", line)}
    rng = re.search(r"@Range(?:\{[^}]*\})?:\s*(-?[\d.]+)\s+(-?[\d.]+)", doc)
    if vals and val in vals:
        return True, "in @Values"
    if rng:
        lo, hi = float(rng.group(1)), float(rng.group(2))
        return lo <= val <= hi, "@Range %g..%g%s" % (lo, hi, " (@Values list is informative)" if vals else "")
    if vals:
        return False, "@Values %s" % sorted(vals)
    return True, "no @Values/@Range (free value)"


def v5_params(ap_root, hdr, params):
    print("V5 Rev 13 defaults.parm entries vs ArduPilot @Param docs")
    for name in REV13_PARAMS:
        if name not in params:
            check(False, "%s present in defaults.parm" % name)
            continue
        src, doc = param_doc(ap_root, name)
        if src is None:
            check(False, "%s exists in the tree (%s)" % (name, doc))
            continue
        ok, why = value_ok(doc, params[name])
        check(ok, "%s = %g valid (%s; %s)" % (name, params[name], os.path.basename(src), why))
    # semantic checks the doc ranges cannot catch
    bp = os.path.join(ap_root, "libraries", "AP_BattMonitor", "AP_BattMonitor_Params.h")
    enum = dict((v, int(n)) for v, n in re.findall(r"(\w+)\s*=\s*(\d+),", read(bp)))
    want = enum.get("ANALOG_VOLTAGE_ONLY")
    check(params.get("BATT2_MONITOR") == want,
          "BATT2_MONITOR %s == Type::ANALOG_VOLTAGE_ONLY (%s, AP_BattMonitor_Params.h) - "
          "any other value creates no Analog backend, so BATT2_VOLT_* do not exist"
          % (params.get("BATT2_MONITOR"), want))
    m = re.search(r"#define HAL_ANALOG_PINS \\\n(.*?)\n\n", hdr, re.S)
    adc_pins = [int(x) for x in re.findall(r"\{\s*(\d+),", m.group(1))] if m else []
    check(int(params.get("BATT2_VOLT_PIN", -1)) in adc_pins,
          "BATT2_VOLT_PIN %s is an analog pin of this board %s" % (params.get("BATT2_VOLT_PIN"), adc_pins))
    ratio = (R40 + R41) / R41
    check(abs(params.get("BATT2_VOLT_MULT", 0) - ratio) < 1e-6,
          "BATT2_VOLT_MULT %s == (R40 + R41) / R41 = %.3f" % (params.get("BATT2_VOLT_MULT"), ratio))
    v12 = 0.8 * (1 + 102e3 / (10e3 * 26.7e3 / (10e3 + 26.7e3)))
    vadc = v12 / ratio
    check(vadc < 3.3 * 0.9, "BEC sense at 12.02 V nominal = %.3f V at PC5 (< 90 %% of VREF 3.3 V)" % vadc)


# ---------------------------------------------------------------------------
# V6 patch 0003
# ---------------------------------------------------------------------------
SIM = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static uint64_t now_ns = 0;
struct SimCfg { double sclk_hz; double t_read_us; double t_off_us; };
static SimCfg cfg;
struct AT7456E_Sim {
    uint8_t nvm[256][64];
    uint8_t shadow[64];
    uint8_t vm0, cmah, cmal;
    uint64_t busy_until_ns;     // STAT[5]
    uint64_t read_done_ns;      // pending NVM -> shadow copy
    int read_chr;
    int ignored_cmm;
    void settle() {
        if (read_chr >= 0 && now_ns >= read_done_ns) {
            memcpy(shadow, nvm[read_chr], 64);
            read_chr = -1;
        }
    }
    void reg_write(uint8_t a, uint8_t d) {
        settle();
        switch (a) {
        case 0x00:
            if ((vm0 & 0x08) && !(d & 0x08)) busy_until_ns = now_ns + (uint64_t)(cfg.t_off_us * 1000.0);
            vm0 = d; break;
        case 0x09: cmah = d; break;
        case 0x0a: cmal = d; break;
        case 0x08:
            if ((vm0 & 0x08) || now_ns < busy_until_ns) { ignored_cmm++; break; }
            if ((d & 0xF0) == 0x50) {
                read_chr = cmah;
                read_done_ns = now_ns + (uint64_t)(cfg.t_read_us * 1000.0);
                busy_until_ns = read_done_ns;
            }
            break;
        default: break;
        }
    }
    uint8_t reg_read(uint8_t a) {
        settle();
        if (a == 0xC0) return shadow[cmal & 63];
        if (a == 0x80) return vm0;
        if (a == 0xA0) return (now_ns < busy_until_ns) ? 0x20 : 0x00;
        return 0;
    }
};
static AT7456E_Sim sim;
static void byte_time() { now_ns += (uint64_t)(8e9 / cfg.sclk_hz); }
struct Sem { void take_blocking() {} void give() {} };
struct Dev {
    Sem s;
    Sem* get_semaphore() { return &s; }
    // one CS-framed transfer: 16-bit commands, read data in the 2nd byte
    bool transfer_fullduplex(uint8_t* b, uint32_t n) {
        for (uint32_t i = 0; i + 1 < n; i += 2) {
            const uint8_t a = b[i];
            byte_time(); b[i] = 0;
            byte_time();
            if (a & 0x80) { b[i + 1] = sim.reg_read(a); }
            else { sim.reg_write(a, b[i + 1]); b[i + 1] = 0; }
        }
        return true;
    }
};
struct Sched { void delay_microseconds(uint16_t us) { now_ns += (uint64_t)us * 1000u; } };
struct HAL { Sched* scheduler; };
static Sched sched_obj;
static const HAL hal = { &sched_obj };
%(defines)s
struct AP_OSD_MAX7456 {
    static const uint16_t spi_buffer_size = 512;
    uint8_t buffer[spi_buffer_size];
    uint16_t buffer_offset;
    Dev dev_obj;
    Dev* _dev = &dev_obj;
    void buffer_add_cmd(uint8_t reg, uint8_t arg);
    bool check_font_char(uint8_t chr, const uint8_t* font_data);
};
/* ---- extracted verbatim from %(which)s AP_OSD_MAX7456.cpp ---- */
%(add_cmd)s
%(check_fn)s
/* ------------------------------------------------------------- */
static uint32_t g = 0x0743A7u;
static uint8_t rnd() { g = g * 1664525u + 1013904223u; return (uint8_t)(g >> 24); }
int main(int argc, char** argv) {
    if (argc != 5) return 2;
    cfg.sclk_hz = atof(argv[1]); cfg.t_read_us = atof(argv[2]); cfg.t_off_us = atof(argv[3]);
    const int display_on = atoi(argv[4]);
    static uint8_t font[256][54];
    for (int c = 0; c < 256; c++) for (int x = 0; x < 54; x++) { font[c][x] = rnd(); sim.nvm[c][x] = font[c][x]; }
    memset(sim.shadow, 0x55, sizeof sim.shadow);
    sim.read_chr = -1; sim.vm0 = display_on ? 0x48 : 0x00;
    AP_OSD_MAX7456 osd;
    int ok = 0;
    for (int c = 0; c < 256; c++) {
        if (display_on) sim.vm0 = 0x48;       // runtime font change: OSD running
        now_ns += 1000000;                      // >= 1 ms between characters
        ok += osd.check_font_char((uint8_t)c, font[c]) ? 1 : 0;
    }
    printf("%%d %%d\n", ok, sim.ignored_cmm);
    return 0;
}
'''


def build_sim(src_text, which, tmp):
    defs_start = src_text.index("#define VIDEO_COLUMNS")
    defs_end = src_text.index("extern const AP_HAL::HAL &hal;")
    defines = src_text[defs_start:defs_end]
    add_cmd = extract_function(src_text, "void AP_OSD_MAX7456::buffer_add_cmd(uint8_t reg, uint8_t arg)")
    check_fn = extract_function(src_text, "bool AP_OSD_MAX7456::check_font_char(uint8_t chr, const uint8_t* font_data)")
    code = SIM.replace("%(defines)s", defines).replace("%(which)s", which) \
              .replace("%(add_cmd)s", add_cmd).replace("%(check_fn)s", check_fn).replace("%%", "%")
    cpp = os.path.join(tmp, "sim_%s.cpp" % which)
    exe = os.path.join(tmp, "sim_%s" % which)
    with open(cpp, "w") as fh:
        fh.write("#include <stdlib.h>\n" + code)
    r = subprocess.run(["g++", "-std=c++17", "-O1", "-Wall", "-Werror", "-Wno-unused-macros",
                        "-o", exe, cpp], capture_output=True, text=True)
    check(r.returncode == 0, "V6 %s check_font_char() compiles verbatim in the AT7456E simulator %s"
          % (which, r.stderr[-400:]))
    return exe if r.returncode == 0 else None


def run_sim(exe, sclk, t_read, t_off, display_on):
    r = subprocess.run([exe, str(sclk), str(t_read), str(t_off), str(display_on)],
                       capture_output=True, text=True)
    ok, ignored = (int(x) for x in r.stdout.split())
    return ok, ignored


def v6_patch(ap_root, tmp):
    print("V6 patch 0003 (AT7456E NVM read delay)")
    if not os.path.exists(PATCH):
        check(False, "V6 %s exists" % os.path.relpath(PATCH, REPO))
        return
    ptxt = read(PATCH)
    targets = sorted(set(re.findall(r"^\+\+\+ b/(\S+)", ptxt, re.M)))
    check(targets == [OSD_SRC], "V6 patch touches only %s (%s)" % (OSD_SRC, targets))
    if ap_root and os.path.isdir(os.path.join(ap_root, ".git")) or \
            subprocess.run(["git", "-C", ap_root, "rev-parse"], capture_output=True).returncode == 0:
        r = subprocess.run(["git", "-C", ap_root, "diff", "--quiet", "HEAD", "--", OSD_SRC],
                           capture_output=True)
        check(r.returncode == 0, "V6 tree %s is unmodified vs HEAD (patch base is upstream)" % OSD_SRC)
    work = os.path.join(tmp, "patch0003")
    os.makedirs(os.path.join(work, os.path.dirname(OSD_SRC)))
    shutil.copy(os.path.join(ap_root, OSD_SRC), os.path.join(work, OSD_SRC))
    upstream = read(os.path.join(work, OSD_SRC))
    env = dict(os.environ, GIT_DIR=os.path.join(work, ".nogit"))   # never touch a repo
    r = subprocess.run(["git", "apply", "--check", "-v", PATCH], cwd=work, capture_output=True,
                       text=True, env=env)
    check(r.returncode == 0, "V6 git apply --check on a temp copy %s" % (r.stderr.strip()[-300:]))
    if r.returncode:
        return
    r = subprocess.run(["git", "apply", PATCH], cwd=work, capture_output=True, text=True, env=env)
    check(r.returncode == 0, "V6 git apply on the temp copy")
    patched = read(os.path.join(work, OSD_SRC))
    r = subprocess.run(["patch", "-p1", "--dry-run", "-s", "-i", PATCH],
                       cwd=ap_root, capture_output=True, text=True)
    check(r.returncode == 0, "V6 patch -p1 --dry-run against the tree (validate_bdshot V7 method)")

    # structural checks on the patched file
    s = strip_c(patched)
    check(s.count("{") == s.count("}") and s.count("(") == s.count(")"),
          "V6 patched file braces/parens balanced ({%d}%d (%d)%d)" %
          (s.count("{"), s.count("}"), s.count("("), s.count(")")))
    pp = [ln.strip().split()[0] for ln in patched.splitlines() if ln.strip().startswith("#")]
    opens = sum(1 for d in pp if d in ("#if", "#ifdef", "#ifndef"))
    check(opens == pp.count("#endif"), "V6 preprocessor #if/#endif balanced (%d/%d)" % (opens, pp.count("#endif")))
    fn = extract_function(patched, "bool AP_OSD_MAX7456::check_font_char(uint8_t chr, const uint8_t* font_data)")
    calls = re.findall(r"(transfer_fullduplex\(([^;]*)\)|delay_microseconds\((\w+)\))", fn)
    seq = [("X", c[1]) if c[0].startswith("transfer") else ("D", c[2]) for c in calls]
    check([k for k, _ in seq] == ["X", "D", "X", "D", "X"],
          "V6 check_font_char: transfer / delay / transfer / delay / transfer (%s)" % [k for k, _ in seq])
    delays = {}
    for name, val in re.findall(r"#define (MAX7456_\w+_DELAY_US) (\d+)", patched):
        delays[name] = int(val)
    for k, arg in seq:
        if k == "D":
            v = delays.get(arg, int(arg) if arg.isdigit() else None)
            check(v is not None and v >= AT7456E["nvm_read_typ_us"],
                  "V6 %s = %s us >= %g us (AT7456E typ, DS p.36 note 3)" % (arg, v, AT7456E["nvm_read_typ_us"]))
    # byte layout: the first two transfers must end exactly on VM0 and on CMM=READ_NVR
    pre_loop = fn.split("for (", 1)[0]
    cmds = re.findall(r"buffer_add_cmd\((\w+),\s*(\w+)\)", pre_loop)
    check(cmds == [("MAX7456ADD_VM0", "0"), ("MAX7456ADD_CMAH", "chr"), ("MAX7456ADD_CMM", "READ_NVR")],
          "V6 pre-loop commands VM0, CMAH, CMM=READ_NVR unchanged (%s)" % cmds)
    xs = [a for k, a in seq if k == "X"]
    check(xs == ["buffer, 2", "&buffer[2], 4", "&buffer[6], buffer_offset - 6"],
          "V6 transfer slices [0,2) VM0 | [2,6) CMAH+CMM | [6,end) CMAL/CMDO pairs (%s)" % xs)
    check("buffer_offset = 9;" in fn, "V6 response parse offset 9 (first CMDO data byte) unchanged")
    check(upstream.count("transfer_fullduplex(buffer, buffer_offset);") == 1,
          "V6 upstream check_font_char is the single-burst version")

    # behavioural: extracted upstream vs patched code against the simulator
    if shutil.which("g++") is None:
        skip("V6 g++ not available - simulator not run")
        return
    exe_up = build_sim(upstream, "upstream", tmp)
    exe_p = build_sim(patched, "patched", tmp)
    if not (exe_up and exe_p):
        return
    cases = [
        # (label, sclk, t_read, t_off, display_on, expect_upstream_all_ok, expect_patched_all_ok)
        ("AT7456E min 22 us, 10 MHz, boot", 10e6, 22.0, 30.0, 0, False, True),
        ("AT7456E typ 30 us, 5 MHz, boot", 5e6, 30.0, 30.0, 0, False, True),
        ("AT7456E typ, 5 MHz, runtime font change (display on)", 5e6, 30.0, 30.0, 1, False, True),
        ("MAX7456 0.5 us, 10 MHz, boot", 10e6, 0.5, 0.0, 0, True, True),
        ("MAX7456 0.5 us, 10 MHz, display on", 10e6, 0.5, 0.0, 1, True, True),
    ]
    for label, sclk, tr, to, disp, exp_up, exp_p in cases:
        ok_u, ign_u = run_sim(exe_up, sclk, tr, to, disp)
        ok_p, ign_p = run_sim(exe_p, sclk, tr, to, disp)
        check((ok_u == 256) == exp_up and (ok_p == 256) == exp_p,
              "V6 %s: upstream %d/256 chars verified (%d CMM ignored), patched %d/256 (%d ignored)"
              % (label, ok_u, ign_u, ok_p, ign_p))


# ---------------------------------------------------------------------------
# V7 Lua script
# ---------------------------------------------------------------------------
LUA_BUILTINS = {"assert", "error", "pairs", "ipairs", "tostring", "tonumber", "type",
                "select", "pcall", "setmetatable", "getmetatable", "rawget", "rawset",
                "next", "require"}


def lua_strip(src):
    src = re.sub(r"--\[(=*)\[.*?\]\1\]", "", src, flags=re.S)
    src = re.sub(r"--[^\n]*", "", src)
    src = re.sub(r'"(?:\\.|[^"\\])*"', '""', src)
    src = re.sub(r"'(?:\\.|[^'\\])*'", "''", src)
    return src


def v7_bindings(ap_root, src):
    docs = read(os.path.join(ap_root, "libraries", "AP_Scripting", "docs", "docs.lua"))
    methods = set(re.findall(r"^function (\w+:\w+)\(", docs, re.M))
    globals_ = set(re.findall(r"^function (\w+)\(", docs, re.M))
    singletons = {m.split(":")[0] for m in methods}
    code = lua_strip(src)
    param_vars = set(re.findall(r"local (\w+) = bind_add_param\(", code))
    local_fns = set(re.findall(r"local function (\w+)\(", code)) | set(re.findall(r"^function (\w+)\(", code, re.M))
    used = set()
    bad = []
    for m in re.finditer(r"(\w+|\))\s*:\s*(\w+)\s*\(", code):
        obj, meth = m.group(1), m.group(2)
        if obj == ")":
            key = "uint32_t_ud:%s" % meth          # (millis() - t):toint()
        elif obj in param_vars:
            key = "Parameter_ud:%s" % meth
        elif obj in singletons:
            key = "%s:%s" % (obj, meth)
        else:
            bad.append("%s:%s (unresolved object)" % (obj, meth))
            continue
        used.add(key)
        if key not in methods:
            bad.append(key)
    for m in re.finditer(r"(?<![\w.:])(\w+)\s*\(", code):
        name = m.group(1)
        if name in ("function", "if", "and", "or", "not", "return", "elseif", "until", "while"):
            continue
        if name in local_fns or name in LUA_BUILTINS:
            continue
        used.add(name)
        if name not in globals_:
            bad.append("%s() (global)" % name)
    for m in re.finditer(r"(?<![\w.:])(math|string|table)\.(\w+)", code):
        used.add("%s.%s" % m.groups())
    check(not bad, "V7 every binding exists in AP_Scripting/docs/docs.lua (%d used) %s"
          % (len(used), sorted(set(bad))))
    print("      bindings: %s" % ", ".join(sorted(u for u in used if ":" in u or u in globals_)))
    # binding index semantics (0-based) straight from bindings.desc
    desc = read(os.path.join(ap_root, "libraries", "AP_Scripting", "generator", "description", "bindings.desc"))
    rel = re.findall(r"singleton AP_Relay method (on|off|enabled|get) \w+ uint8_t 0 AP_RELAY_NUM_RELAYS", desc)
    bat = re.findall(r"singleton AP_BattMonitor method (voltage|healthy) \w+ uint8_t 0 ud->num_instances\(\)", desc)
    check(sorted(rel) == ["enabled", "get", "off", "on"] and sorted(bat) == ["healthy", "voltage"],
          "V7 relay/battery bindings take a raw 0-based instance (bindings.desc)")
    relay_h = read(os.path.join(ap_root, "libraries", "AP_Relay", "AP_Relay.h"))
    check("void        on(uint8_t instance) { set(instance, true); }" in relay_h,
          "V7 AP_Relay::on(instance) indexes _params[instance]: relay:on(1) = RELAY2")
    ri = dict(re.findall(r"local (RELAY_VTX_\w+) = (\d+)", code))
    check(ri == {"RELAY_VTX_PWR": "1", "RELAY_VTX_12V": "2"},
          "V7 script uses relay 1 = RELAY2 (VTX_PWR), 2 = RELAY3 (VTX_12V_SEL) %s" % ri)
    key = re.search(r"PARAM_TABLE_KEY = (\d+)", code)
    used_keys = set()
    for d in ("applets", "examples", "drivers"):
        p = os.path.join(ap_root, "libraries", "AP_Scripting", d)
        for root, _, files in os.walk(p):
            for f in files:
                if f.endswith(".lua"):
                    used_keys |= {int(k) for k in re.findall(r"PARAM_TABLE_KEY\s*=\s*(\d+)", read(os.path.join(root, f)))}
    check(key is not None and int(key.group(1)) not in used_keys and 0 < int(key.group(1)) < 200,
          "V7 PARAM_TABLE_KEY %s not used by any upstream applet/example" % (key.group(1) if key else None))
    names = re.findall(r'bind_add_param\("(\w+)"', src)
    pref = re.search(r'PARAM_TABLE_PREFIX = "(\w+)"', src).group(1)
    ntab = int(re.search(r"param:add_table\(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, (\d+)\)", src).group(1))
    check(len(names) <= ntab and all(len(pref + n) <= 16 for n in names),
          "V7 %d params <= table size %d, names <= 16 chars (AP_MAX_NAME_SIZE)" % (len(names), ntab))


LUA_STUB_C = r'''
#include <stdlib.h>
#include <stdio.h>
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
int lua_get_current_env_ref(void) { return 0; }
const char* lua_get_modules_path(void) { return ""; }
void lua_abort(void) { abort(); }
static void *l_alloc(void *ud, void *ptr, size_t osize, size_t nsize) {
  (void)ud; (void)osize;
  if (nsize == 0) { free(ptr); return NULL; }
  return realloc(ptr, nsize);
}
lua_State *luaL_newstate(void) { return lua_newstate(l_alloc, NULL); }
#ifdef NANO_LUA_RUNNER
static int l_print(lua_State *L) {
  int n = lua_gettop(L);
  for (int i = 1; i <= n; i++) { if (i > 1) fputs("\t", stdout); fputs(luaL_tolstring(L, i, NULL), stdout); lua_pop(L, 1); }
  fputs("\n", stdout); return 0;
}
int main(int argc, char **argv) {
  if (argc < 2) return 2;
  lua_State *L = luaL_newstate();
  luaL_requiref(L, "_G", luaopen_base, 1); lua_pop(L, 1);
  luaL_requiref(L, "math", luaopen_math, 1); lua_pop(L, 1);
  luaL_requiref(L, "string", luaopen_string, 1); lua_pop(L, 1);
  luaL_requiref(L, "table", luaopen_table, 1); lua_pop(L, 1);
  lua_register(L, "print", l_print);
  if (luaL_dofile(L, argv[1]) != LUA_OK) { fprintf(stderr, "%s\n", lua_tostring(L, -1)); return 1; }
  return 0;
}
#endif
'''

LUA_CORE = ["lapi", "lcode", "lctype", "ldebug", "ldo", "ldump", "lfunc", "lgc", "llex", "lmem",
            "lobject", "lopcodes", "lparser", "lstate", "lstring", "ltable", "ltm", "lundump",
            "lvm", "lzio", "lauxlib"]
LUA_LIBS = ["lbaselib", "lmathlib", "lstrlib", "ltablib"]


def build_tree_lua(ap_root, tmp):
    """Build luac and a minimal Lua runner from the tree's own (ArduPilot-
    modified Lua 5.3) sources.  The ArduPilot-only headers are stubbed in the
    temp dir; nothing in the tree is written."""
    src = os.path.join(ap_root, "libraries", "AP_Scripting", "lua", "src")
    d = os.path.join(tmp, "lua")
    for sub in ("AP_Filesystem", "AP_HAL", "AP_Scripting"):
        os.makedirs(os.path.join(d, "stub", sub))
    with open(os.path.join(d, "stub", "AP_Filesystem", "posix_compat.h"), "w") as fh:
        fh.write("#pragma once\n")
    with open(os.path.join(d, "stub", "AP_HAL", "ap_setjmp.h"), "w") as fh:
        fh.write("#pragma once\n#include <setjmp.h>\n#define ap_setjmp setjmp\n"
                 "#define ap_longjmp longjmp\n#define ap_jmp_buf jmp_buf\n")
    with open(os.path.join(d, "stub", "AP_Scripting", "lua_common_defs.h"), "w") as fh:
        fh.write('#pragma once\n#define SCRIPTING_DIRECTORY "./scripts"\n'
                 "int lua_get_current_env_ref(void);\nconst char* lua_get_modules_path(void);\n"
                 "void lua_abort(void) __attribute__((noreturn));\n")
    stub_c = os.path.join(d, "stub.c")
    with open(stub_c, "w") as fh:
        fh.write(LUA_STUB_C)
    base = ["gcc", "-std=gnu99", "-O1", "-w", "-I", os.path.join(d, "stub"), "-I", src, "-DLUA_USE_POSIX"]
    core = [os.path.join(src, f + ".c") for f in LUA_CORE]
    luac = os.path.join(d, "luac")
    r1 = subprocess.run(base + ["-o", luac, stub_c, os.path.join(src, "luac.c")] + core + ["-lm"],
                        capture_output=True, text=True)
    runner = os.path.join(d, "lua_runner")
    r2 = subprocess.run(base + ["-DNANO_LUA_RUNNER", "-o", runner, stub_c] + core +
                        [os.path.join(src, f + ".c") for f in LUA_LIBS] + ["-lm"],
                        capture_output=True, text=True)
    return (luac if r1.returncode == 0 else None,
            runner if r2.returncode == 0 else None,
            (r1.stderr + r2.stderr)[-400:])


LUA_SIM = r'''
-- closed-loop test of ardupilot/scripts/vtx_bec.lua (mocked bindings)
local T = 4294967296 - 3000      -- start 3 s before the uint32 millis() wrap
local U = {}
U.__index = U
local function u32(v) return setmetatable({v = v %% 4294967296}, U) end
U.__sub = function(a, b)
  local av = type(a) == "table" and a.v or a
  local bv = type(b) == "table" and b.v or b
  return u32(av - bv)
end
function U:toint() local v = self.v if v >= 2147483648 then v = v - 4294967296 end return math.floor(v) end
function millis() return u32(T) end

local W = {relay = {[0] = 0, [1] = 1, [2] = 0}, relay_en = {[0] = false, [1] = true, [2] = true},
           pins = {RELAY2_PIN = 81, RELAY3_PIN = 82},
           vbat = 16.0, nbatt = 2, healthy = {[0] = true, [1] = true}, armed = false,
           aux = {}, msgs = {}, plant_fault = false, bec_dead = false}
local function inst(i, n) assert(math.type(i) == "integer" and i >= 0 and i < n, "bad instance " .. tostring(i)) end
relay = {}
function relay:on(i) inst(i, 6) W.relay[i] = 1 end
function relay:off(i) inst(i, 6) W.relay[i] = 0 end
function relay:get(i) inst(i, 6) return W.relay[i] or 0 end
function relay:enabled(i) inst(i, 6) return W.relay_en[i] == true end
battery = {}
function battery:num_instances() return W.nbatt end
local function vrail()
  if W.relay[1] ~= 1 or W.bec_dead then return 0 end
  local target = (W.relay[2] == 1 and not W.plant_fault) and 12.02 or 8.96
  return math.min(target, W.vbat - 0.35)
end
function battery:voltage(i) inst(i, W.nbatt) if i == 0 then return W.vbat end return vrail() end
function battery:healthy(i) return i < W.nbatt and W.healthy[i] end
arming = {}
function arming:is_armed() return W.armed end
gcs = {}
function gcs:send_text(sev, txt) assert(sev >= 0 and sev <= 7) W.msgs[#W.msgs + 1] = txt end
rc = {}
function rc:get_aux_cached(fn) return W.aux[fn] end
local P = {}
local tables = {}
param = {}
function param:add_table(key, prefix, n) tables[key] = {prefix = prefix, n = n} return true end
function param:add_param(key, idx, name, default)
  local t = tables[key] assert(t and idx >= 1 and idx <= t.n) P[t.prefix .. name] = default return true
end
function param:get(name) if W.pins[name] then return W.pins[name] end return P[name] end
local PU = {}
PU.__index = PU
function PU:get() return P[self.n] end
function PU:set(v) P[self.n] = v return true end
function Parameter(name) assert(P[name] ~= nil, "no param " .. name) return setmetatable({n = name}, PU) end

local script = function()
%(script)s
end
local fn, period = script()
local fails, checks = 0, 0
local function CHECK(id, ok, what)
  checks = checks + 1
  if not ok then fails = fails + 1 end
  print(string.format("      [%%s] %%s %%s", ok and "ok" or "FAIL", id, what))
end
-- run the scheduler for ms milliseconds
local function run(ms)
  local stop = T + ms
  while T < stop do
    T = T + period
    local f, p = fn()
    fn, period = f, p
  end
end
local function last_msg() return W.msgs[#W.msgs] or "" end

CHECK("L1", period == 1000 and P.VTXB_ENABLE == 1 and P.VTXB_12V_REQ == 0, "params registered, first run after 1 s")
run(1000)
CHECK("L2", W.relay[1] == 1 and W.relay[2] == 0, "boot: VTX on, 9 V")
-- 3S never gets 12 V
W.vbat = 12.6 P.VTXB_12V_REQ = 1
run(10000)
CHECK("L3", W.relay[2] == 0, "3S 12.6 V + request: stays 9 V for 10 s")
-- 4S: qualification 2 s (crosses the millis() wrap)
W.vbat = 16.0
run(1800)
CHECK("L4", W.relay[2] == 0, "4S: still 9 V after 1.8 s of qualification")
run(400)
CHECK("L5", W.relay[2] == 1, "4S: 12 V granted after 2 s")
-- sag below 13.0 V
W.vbat = 12.8 run(200) W.vbat = 15.5
run(500)
CHECK("L6", W.relay[2] == 1, "VBAT dip < 13.0 V for 0.2 s ignored")
W.vbat = 12.8 run(400)
CHECK("L7", W.relay[2] == 0, "VBAT < 13.0 V for 0.4 s -> 9 V")
W.vbat = 13.3 run(5000)
CHECK("L8", W.relay[2] == 0, "VBAT 13.3 V (hysteresis band) does not re-qualify")
W.vbat = 16.0 run(2200)
CHECK("L9", W.relay[2] == 1, "recovered VBAT re-qualifies after 2 s")
-- rail fault (Q1 open: rail stays at 9 V while RELAY3 is high)
W.plant_fault = true
run(900)
CHECK("L10", W.relay[2] == 0 and string.find(last_msg(), "rail fault") ~= nil, "rail fault -> 9 V: " .. last_msg())
W.plant_fault = false
run(5000)
CHECK("L11", W.relay[2] == 0, "fault latched while request held")
P.VTXB_12V_REQ = 0 run(300) P.VTXB_12V_REQ = 1 run(2300)
CHECK("L12", W.relay[2] == 1, "request cycle clears the latch, 12 V again")
P.VTXB_12V_REQ = 0 run(200)
CHECK("L13", W.relay[2] == 0, "request removed -> 9 V")
-- external RELAY3 on is reverted
W.relay[2] = 1 run(100)
CHECK("L14", W.relay[2] == 0, "external RELAY3 on reverted in 9 V mode")
-- RC aux request
P.VTXB_RC_FUNC = 300 W.aux[300] = 2 run(2300)
CHECK("L15", W.relay[2] == 1, "RC Scripting1 high requests 12 V")
W.aux[300] = 0 run(200)
CHECK("L16", W.relay[2] == 0, "RC Scripting1 low -> 9 V")
-- VTX power never off while armed
P.VTXB_PWR = 0 W.armed = true run(1000)
CHECK("L17", W.relay[1] == 1, "VTXB_PWR 0 while armed: VTX stays on")
W.relay[1] = 0 run(100)
CHECK("L18", W.relay[1] == 1, "external RELAY2 off while armed forced back on")
W.armed = false run(200)
CHECK("L19", W.relay[1] == 0, "disarmed + VTXB_PWR 0: VTX off")
P.VTXB_12V_REQ = 1 run(3000)
CHECK("L20", W.relay[2] == 0, "no 12 V while VTX is off")
W.armed = true run(100)
CHECK("L21", W.relay[1] == 1, "arming switches the VTX back on")
P.VTXB_PWR = 1 W.armed = false P.VTXB_12V_REQ = 0 run(500)
-- BEC dead (sense disagrees) -> never 12 V
W.bec_dead = true P.VTXB_12V_REQ = 1 run(4000)
CHECK("L22", W.relay[2] == 0, "+VTX sense 0 V with VTX on: 12 V refused")
W.bec_dead = false run(2300)
CHECK("L23", W.relay[2] == 1, "sense back in the 9 V window: 12 V granted")
-- BATT2 missing: no binding range error, 9 V
W.nbatt = 1 run(500)
CHECK("L24", W.relay[2] == 0, "BATT2 instance gone -> 9 V, no Lua error")
W.nbatt = 2 W.healthy[0] = false run(3000)
CHECK("L25", W.relay[2] == 0, "VBAT monitor unhealthy: 12 V refused")
W.healthy[0] = true run(2300)
CHECK("L26", W.relay[2] == 1 and W.relay[1] == 1, "healthy again: 12 V, VTX on")
-- disabled script does nothing
P.VTXB_ENABLE = 0 W.relay[2] = 0 P.VTXB_12V_REQ = 1 run(5000)
CHECK("L27", W.relay[2] == 0 and period == 1000, "VTXB_ENABLE 0: script idle")
-- misconfigured relay: reports, does not crash
W.relay_en[2] = false
local fn2, p2 = script()
fn, period = fn2, p2
P.VTXB_ENABLE = 1
run(1000)
CHECK("L28", string.find(last_msg(), "RELAY2/RELAY3") ~= nil and period == 5000, "RELAY3 not configured: error + 5 s retry: " .. last_msg())
print(string.format("LUA_SIM %%d checks, %%d failed", checks, fails))
'''


def v7_lua(ap_root, tmp):
    print("V7 Lua script %s" % os.path.relpath(LUA_SCRIPT, REPO))
    if not os.path.exists(LUA_SCRIPT):
        check(False, "V7 script exists")
        return
    src = read(LUA_SCRIPT)
    v7_bindings(ap_root, src)
    sys_luac = next((p for p in ("luac5.3", "luac", "luac5.4") if shutil.which(p)), None)
    luac, runner, err = (None, None, "")
    if shutil.which("gcc"):
        luac, runner, err = build_tree_lua(ap_root, tmp)
    if sys_luac:
        r = subprocess.run([sys_luac, "-p", LUA_SCRIPT], capture_output=True, text=True)
        check(r.returncode == 0, "V7 %s -p %s" % (sys_luac, r.stderr.strip()))
    if luac:
        r = subprocess.run([luac, "-v"], capture_output=True, text=True)
        ver = (r.stdout + r.stderr).strip().split("  ")[0]
        r = subprocess.run([luac, "-p", LUA_SCRIPT], capture_output=True, text=True)
        check(r.returncode == 0, "V7 luac -p with the tree's own %s %s" % (ver, r.stderr.strip()))
    elif not sys_luac:
        skip("V7 no luac and the tree Lua could not be built: %s" % err)
    if not runner:
        skip("V7 closed-loop simulation needs the tree Lua runner: %s" % err)
        return
    body = re.sub(r"^", "  ", src, flags=re.M)
    test = os.path.join(tmp, "lua", "vtx_bec_sim.lua")
    with open(test, "w") as fh:
        fh.write(LUA_SIM.replace("%(script)s", body).replace("%%", "%"))
    r = subprocess.run([runner, test], capture_output=True, text=True, timeout=120)
    out = r.stdout.rstrip()
    if out:
        print(out)
    m = re.search(r"LUA_SIM (\d+) checks, (\d+) failed", out)
    check(r.returncode == 0 and m is not None and m.group(2) == "0",
          "V7 interlock simulation %s %s" % (m.group(0) if m else "", r.stderr.strip()[-300:]))


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ardupilot", default=os.environ.get("ARDUPILOT_ROOT"))
    ap.add_argument("--keep", action="store_true", help="keep temp output")
    args = ap.parse_args()
    if not args.ardupilot or not os.path.isdir(args.ardupilot):
        print("ERROR: pass --ardupilot <ArduPilot checkout> or set ARDUPILOT_ROOT")
        return 2
    ap_root = os.path.abspath(args.ardupilot)
    tmp = tempfile.mkdtemp(prefix="nanoh743_osdvtx_")
    try:
        params = load_params()
        print("V1 generated hwdef.h (firmware + bootloader)")
        hdr, bl = generate(ap_root, tmp)
        if hdr is None:
            return 1
        if bl is not None:
            v1_bootloader(bl)
        v2_spi(ap_root, tmp, hdr)
        v3_gpio(ap_root, hdr, params)
        v4_adc_uart(ap_root, hdr)
        v5_params(ap_root, hdr, params)
        v6_patch(ap_root, tmp)
        v7_lua(ap_root, tmp)

        print("V8 DMA budget with UART7 (tools/validate_dma.py)")
        r = subprocess.run([sys.executable, os.path.join(REPO, "tools", "validate_dma.py"),
                            "--ardupilot", ap_root], capture_output=True, text=True)
        res = [ln for ln in r.stdout.splitlines() if "RESULT" in ln or "streams used" in ln]
        check(r.returncode == 0, "V8 validate_dma.py: %s" % "; ".join(s.strip() for s in res))
    finally:
        if args.keep:
            print("\noutput kept in %s" % tmp)
        else:
            shutil.rmtree(tmp, ignore_errors=True)

    print("\nRESULT: %s (%d failure%s)" % ("PASS" if not FAILS else "FAIL", len(FAILS),
                                         "" if len(FAILS) == 1 else "s"))
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
