#!/usr/bin/env python3
"""
Nano H743 AIO - MCU core schematic verifier (Revision 7.0, Rev 11.0 IMU block)

Gates (exit 0 only if every check passes):
  S1  Committed .kicad_sch / .kicad_sym are byte-identical to a fresh
      regeneration (into a temp dir) from the current hwdef.dat.  The
      committed files are never modified by this script.
  S2  kicad-cli ERC: 0 errors; the only warnings are global_label_dangling on
      hwdef interface labels whose peripherals live on future sheets.
  S3  U1 pin numbers/names equal KiCad's official STM32H743VITx symbol
      (independent reference table, NOT the generator's own table).
  S4  Exact netlist: HSE_IN/HSE_OUT/HSE_OUT_MCU, VDDA_FB, +3.3VA, +3V3, GND,
      VCAP, NRST, BOOT0 compared as exact node sets (both ends of every
      passive), every hwdef.dat pin on its official LQFP-100 pin number (S6),
      no unexpected nets, every other GPIO a single no-connect pin, and the
      only other no-connect pins are the ICM-42688-P RESV pins 2/3/10/11.
  S5  Exact values for every BOM part (the expected-value table, the
      netlist components and the generator's placed-reference list must be
      the same set; the count is computed, not hard-coded), key MPNs, no
      DNP, only #PWR/#FLG symbols out of BOM, U1 rev V silicon field.
  S8  (Rev 10.0) DShot series resistors R4..R7: U1 TIM1_CHn -> 100R ->
      ESCn_DSHOT, in hwdef PWM(n) order.
  S7  Clock consistency: OSCILLATOR_HZ in hwdef.dat and hwdef-bl.dat equals
      the Y1 frequency, and hwdef.dat sets MCU_CLOCKRATE_MHZ 480.
  S9  (Rev 11.0) dual ICM-42688-P block, exact netlist from an independent
      table in this file (NOT the generator's IMU_BLOCKS): every U2/U3 pin on
      its net, CLKIN star Y2 -> R14/R15/R16, per-IMU supply filter
      +3V3 -> FB -> IMUn_FB -> 22R -> +3V3_IMUn with its caps, 33R SPI series
      resistors between the hwdef (MCU) net and the IMU-pin net, DRDY nets to
      U1 PC4 / PE1 with distinct EXTI lines, exact values, and hwdef.dat
      INVENSENSEV3_CLKIN_BITMASK with bits 0 (imu1) and 1 (imu2) set.
  S10 (Rev 12.0) DPS310 barometer U4, independent table in this file: U4
      pin names vs the LCSC C130156 / datasheet pinout, every U4 pin net,
      +3V3 -> FB4 -> +3V3_BARO with C24/C25 (VDD) and C26 (VDDIO) to GND,
      R23/R24 2.2k pull-ups I2C1_SCL/SDA -> +3V3, R25/R26 33R series
      I2C1_x -> BARO_x, CSB on the VDDIO net (I2C mode), SDO on GND => 0x76
      == hwdef 'BARO DPS310 I2C:0:0x76', I2C1 nets on U1 PB6 (92) / PB7 (93)
      per hwdef, and the U4 footprint land pattern matches the DPS310 pad
      map (8 pads, 0.65 mm pitch, 1.6 mm rows, clockwise numbering).  The
      land-pattern check runs offline against the pad table vendored from
      the KiCad 8.0.9 .kicad_mod (tools/footprint_ref/<name>.json); when an
      installed KiCad library or the network is available, the live stock
      footprint is additionally cross-checked pad-for-pad against the
      vendored copy (set NANO_FP_NO_NETWORK=1 to skip the download).

Usage: python3 tools/verify_mcu_core_sch.py
"""
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KDIR = os.path.join(REPO, "kicad")
SCH = os.path.join(KDIR, "nano-h743-aio.kicad_sch")
SYM = os.path.join(KDIR, "nano-h743-aio.kicad_sym")
sys.path.insert(0, os.path.join(REPO, "tools"))
import gen_mcu_core_sch as gen  # noqa: E402

FAIL = []

# Rev 12.0 (S10): DPS310 pinout, LCSC C130156 EasyEDA symbol / Infineon DS.
DPS310_REF_PINS = {"1": "GND", "2": "CSB", "3": "SDI", "4": "SCK", "5": "SDO",
                   "6": "VDDIO", "7": "GND", "8": "VDD"}
# DPS310 land pattern, LCSC C130156 EasyEDA footprint "LGA-8_DPS310XTSA1"
# (pad centre mm, top view, y down).  Independent of the KiCad footprint.
DPS310_PADS = {"1": (0.8, -0.98), "2": (0.8, -0.33), "3": (0.8, 0.33), "4": (0.8, 0.98),
               "5": (-0.8, 0.98), "6": (-0.8, 0.33), "7": (-0.8, -0.33), "8": (-0.8, -0.98)}
KICAD_FP_TAG = "8.0.9"
FP_REF_DIR = os.path.join(REPO, "tools", "footprint_ref")
KICAD_FP_DIRS = [os.environ.get("KICAD8_FOOTPRINT_DIR", ""), "/usr/share/kicad/footprints",
                 "/usr/local/share/kicad/footprints"]


def kicad_footprint_text(lib_id):
    """Return (source, text) of a stock KiCad 8 footprint: installed library
    first, else the official kicad-footprints repo at tag KICAD_FP_TAG."""
    lib, name = lib_id.split(":", 1)
    for d in KICAD_FP_DIRS:
        p_ = os.path.join(d, lib + ".pretty", name + ".kicad_mod") if d else ""
        if p_ and os.path.exists(p_):
            return p_, open(p_).read()
    if os.environ.get("NANO_FP_NO_NETWORK") == "1":
        return "network disabled (NANO_FP_NO_NETWORK=1)", None
    import urllib.request
    url = ("https://gitlab.com/kicad/libraries/kicad-footprints/-/raw/%s/%s.pretty/%s.kicad_mod"
           % (KICAD_FP_TAG, lib, name))
    try:
        with urllib.request.urlopen(url, timeout=10) as r:
            return url, r.read().decode()
    except Exception as e:  # noqa: BLE001
        return "unreachable (%s)" % e, None


def parse_pads(txt):
    """Pad table from .kicad_mod text: {num: {type, shape, at[x,y,rot], size, layers}}."""
    out = {}
    for m in re.finditer(r'\(pad "(\w+)" (\w+) (\w+)\s*\(at (\S+) (\S+?)(?: (\S+))?\)\s*'
                         r'\(size (\S+) (\S+)\)\s*\(layers ([^)]*)\)', txt):
        n, typ, shape, x, y, rot, sx, sy, lay = m.groups()
        out[n] = {"type": typ, "shape": shape, "at": [float(x), float(y), float(rot or 0)],
                  "size": [float(sx), float(sy)], "layers": re.findall(r'"([^"]+)"', lay)}
    return out


def vendored_footprint(lib_id):
    """Vendored pad table (tools/footprint_ref/<name>.json) -> (path, meta, pads) or (path, None, {})."""
    p_ = os.path.join(FP_REF_DIR, lib_id.split(":", 1)[-1] + ".json")
    if not os.path.exists(p_):
        return p_, None, {}
    meta = json.load(open(p_))
    return p_, meta, {d["number"]: {k: d[k] for k in ("type", "shape", "at", "size", "layers")}
                      for d in meta["pads"]}

# Independent pin reference: KiCad official library MCU_ST_STM32H7.kicad_sym,
# symbol STM32H743VITx (LQFP-100), as packaged by skidl.  Deliberately NOT
# taken from gen_mcu_core_sch.MCU_PINS so a wrong generator table is caught.
KICAD_REF_PINS = {
    1: "PE2", 2: "PE3", 3: "PE4", 4: "PE5", 5: "PE6", 6: "VBAT",
    7: "PC13", 8: "PC14", 9: "PC15", 10: "VSS", 11: "VDD", 12: "PH0",
    13: "PH1", 14: "NRST", 15: "PC0", 16: "PC1", 17: "PC2_C", 18: "PC3_C",
    19: "VSSA", 20: "VREF+", 21: "VDDA", 22: "PA0", 23: "PA1", 24: "PA2",
    25: "PA3", 26: "VSS", 27: "VDD", 28: "PA4", 29: "PA5", 30: "PA6",
    31: "PA7", 32: "PC4", 33: "PC5", 34: "PB0", 35: "PB1", 36: "PB2",
    37: "PE7", 38: "PE8", 39: "PE9", 40: "PE10", 41: "PE11", 42: "PE12",
    43: "PE13", 44: "PE14", 45: "PE15", 46: "PB10", 47: "PB11", 48: "VCAP",
    49: "VSS", 50: "VDD", 51: "PB12", 52: "PB13", 53: "PB14", 54: "PB15",
    55: "PD8", 56: "PD9", 57: "PD10", 58: "PD11", 59: "PD12", 60: "PD13",
    61: "PD14", 62: "PD15", 63: "PC6", 64: "PC7", 65: "PC8", 66: "PC9",
    67: "PA8", 68: "PA9", 69: "PA10", 70: "PA11", 71: "PA12", 72: "PA13",
    73: "VCAP", 74: "VSS", 75: "VDD", 76: "PA14", 77: "PA15", 78: "PC10",
    79: "PC11", 80: "PC12", 81: "PD0", 82: "PD1", 83: "PD2", 84: "PD3",
    85: "PD4", 86: "PD5", 87: "PD6", 88: "PD7", 89: "PB3", 90: "PB4",
    91: "PB5", 92: "PB6", 93: "PB7", 94: "BOOT0", 95: "PB8", 96: "PB9",
    97: "PE0", 98: "PE1", 99: "VSS", 100: "VDD",
}


def check(ok, msg):
    print("  [%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        FAIL.append(msg)


def sexpr(text):
    tok = re.findall(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()]+', text)
    stack, cur = [], []
    for t in tok:
        if t == "(":
            stack.append(cur)
            cur = []
        elif t == ")":
            done = cur
            cur = stack.pop()
            cur.append(done)
        else:
            cur.append(t[1:-1].replace('\\"', '"') if t.startswith('"') else t)
    return cur[0]


def find(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def main():
    print("S1 generator determinism")
    tmp = tempfile.mkdtemp(prefix="nanoh743_sch_")
    regen = os.path.join(tmp, "regen")
    subprocess.run([sys.executable, os.path.join(REPO, "tools", "gen_mcu_core_sch.py"), "--out", regen],
                   check=True, stdout=subprocess.DEVNULL)
    same = (sha(SCH), sha(SYM)) == (sha(os.path.join(regen, os.path.basename(SCH))),
                                    sha(os.path.join(regen, os.path.basename(SYM))))
    check(same, "committed schematic + library match a fresh regeneration from hwdef.dat "
                "(if not: run tools/gen_mcu_core_sch.py)")
    erc = os.path.join(tmp, "erc.json")
    net = os.path.join(tmp, "net.net")
    subprocess.run(["kicad-cli", "sch", "erc", "--format", "json", "--severity-all", "-o", erc, SCH],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["kicad-cli", "sch", "export", "netlist", "--format", "kicadsexpr", "-o", net, SCH],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    hw = gen.load_hwdef_pins(gen.HWDEF)
    hw_labels = {gen.net_for_hwdef(v) for v in hw.values()}

    print("S2 ERC")
    v = [x for s in json.load(open(erc))["sheets"] for x in s["violations"]]
    errs = [x for x in v if x["severity"] == "error"]
    check(not errs, "ERC errors: %d %s" % (len(errs), [e["description"] for e in errs][:5]))
    other = [x for x in v if x["severity"] != "error" and x["type"] != "global_label_dangling"]
    check(not other, "unexpected ERC warnings: %d %s" % (len(other), [(o["type"], o["description"]) for o in other][:5]))
    dang = set()
    for x in v:
        if x["type"] == "global_label_dangling":
            m = re.search(r"'([^']+)'", x["items"][0]["description"])
            dang.add(m.group(1) if m else "?")
    esc_labels = {esc for _, _, esc in gen.DSHOT_SERIES}
    imu_closed = {"SPI1_SCK", "SPI1_MOSI", "SPI1_MISO", "IMU1_CS", "IMU1_DRDY",
                  "SPI4_SCK", "SPI4_MOSI", "SPI4_MISO", "IMU2_CS", "IMU2_DRDY", "IMU_CLKIN_MON"}
    check(not (dang & imu_closed), "S9 IMU interface labels are closed on this sheet (not dangling) %s"
          % sorted(dang & imu_closed))
    check(dang <= hw_labels | esc_labels,
          "dangling global labels are all hwdef / ESC interface signals (%d) %s"
          % (len(dang), sorted(dang - hw_labels - esc_labels)[:5]))

    nl = sexpr(open(net).read())
    nets = {}
    for n in find(find(nl, "nets")[0], "net"):
        name = find(n, "name")[0][1].lstrip("/")
        nets[name] = {(find(nd, "ref")[0][1], find(nd, "pin")[0][1]) for nd in find(n, "node")}
    comps = {}
    for c in find(find(nl, "components")[0], "comp"):
        ref = find(c, "ref")[0][1]
        flds = {}
        for fl in (find(find(c, "fields")[0], "field") if find(c, "fields") else []):
            flds[fl[1][1]] = fl[2] if len(fl) > 2 else ""
        comps[ref] = {"value": find(c, "value")[0][1], "fields": flds}
    u1_lib_pins = {}
    for lp in find(find(nl, "libparts")[0], "libpart"):
        if find(lp, "part")[0][1] == "STM32H743VITx":
            for pn in find(find(lp, "pins")[0], "pin"):
                u1_lib_pins[int(find(pn, "num")[0][1])] = find(pn, "name")[0][1]

    def net_of(ref, pin):
        for k, s_ in nets.items():
            if (ref, pin) in s_:
                return k
        return None

    def U(*nums):
        return {("U1", str(n)) for n in nums}

    def P(refs, pin):
        return {(r, pin) for r in refs}

    print("S3 U1 pin table vs KiCad official STM32H743VITx (MCU_ST_STM32H7)")
    check(len(u1_lib_pins) == 100, "U1 symbol has 100 pins (%d)" % len(u1_lib_pins))
    pin_bad = [(n, u1_lib_pins.get(n), KICAD_REF_PINS[n]) for n in KICAD_REF_PINS
               if (u1_lib_pins.get(n) or "").split("-")[0] != KICAD_REF_PINS[n]]
    check(not pin_bad, "every U1 pin number/name matches the official KiCad symbol %s" % pin_bad[:5])
    sk = "/usr/local/lib/python3.12/dist-packages/skidl/tools/skidl/libs/MCU_ST_STM32H7_sklib.py"
    if os.path.exists(sk):
        txt = open(sk).read()
        blk = txt.split("'name':'STM32H743VITx'", 1)[1].split("Part(**", 1)[0]
        live = {int(n): nm for n, nm in re.findall(r"Pin\(num='(\d+)',name='([^']+)'", blk)}
        check(live == KICAD_REF_PINS, "embedded reference table == installed KiCad library copy (%s)" % os.path.basename(sk))

    print("S4 exact netlist (all non-interface nets, both ends of every part)")
    caps_3v3 = ["C9", "C10", "C11", "C12", "C13", "C14", "C15", "C23"]
    all_caps = ["C%d" % i for i in range(1, 27)]
    expected = {
        "HSE_IN": U(12) | {("Y1", "1"), ("C1", "1")},
        "HSE_OUT": {("Y1", "3"), ("C2", "1"), ("R1", "2")},
        "HSE_OUT_MCU": U(13) | {("R1", "1")},
        "VDDA_FB": {("FB1", "2"), ("R2", "1")},
        "+3.3VA": U(20, 21) | {("R2", "2")} | P(["C3", "C4", "C5", "C6"], "1"),
        "+3V3": U(6, 11, 27, 50, 75, 100) | P(["FB1", "FB2", "FB3", "R17", "FB4", "R23", "R24"], "1")
        | {("Y2", "4")}
        | P(caps_3v3, "1"),
        "GND": U(10, 26, 49, 74, 99, 19) | {("Y1", "2"), ("Y1", "4"), ("R3", "2"), ("Y2", "2")}
        | P(["U2", "U3"], "6") | P(["U2", "U3"], "7") | P(all_caps, "2")
        | {("U4", "1"), ("U4", "7"), ("U4", "5")},
        "VCAP": U(48, 73) | P(["C7", "C8"], "1"),
        "NRST": U(14) | {("C16", "1")},
        "BOOT0": U(94) | {("R3", "1")},
    }
    port_to_num = {nm: n for n, nm in KICAD_REF_PINS.items() if re.match(r"^P[A-K]\d+", nm)}
    port_to_num.update({nm.split("_")[0]: n for nm, n in list(port_to_num.items()) if nm.endswith("_C")})
    for port, func in hw.items():
        expected[gen.net_for_hwdef(func)] = U(port_to_num[port])
    # Rev 10.0: DShot lines run U1 -> 100R -> ESCn_DSHOT
    for ref, mcu_net, esc_net in gen.DSHOT_SERIES:
        expected[mcu_net] = expected[mcu_net] | {(ref, "1")}
        expected[esc_net] = {(ref, "2")}
    pwm_nets = {}
    for line in open(gen.HWDEF):
        m = re.match(r"^\s*P[A-K]\d+\s+(TIM\d+_CH\d)\s+TIM\d+\s+PWM\((\d+)\)", line)
        if m:
            pwm_nets[int(m.group(2))] = m.group(1)
    check(sorted(pwm_nets) == [1, 2, 3, 4] and all(
          gen.DSHOT_SERIES[n - 1][1] == pwm_nets[n] and gen.DSHOT_SERIES[n - 1][2] == "ESC%d_DSHOT" % n
          for n in pwm_nets), "S8 R4..R7 map hwdef PWM(1..4) %s to ESC1..4_DSHOT" % pwm_nets)
    # Rev 11.0 IMU block (S9): independent expected table.  Series resistor
    # pin 1 = MCU/hwdef net, pin 2 = IMU-pin net.
    IMU_SPI = {  # ref: (MCU net, IMU net, IMU ref, IMU pin)
        "R10": ("SPI1_SCK", "IMU1_SCLK", "U2", "13"), "R11": ("SPI1_MOSI", "IMU1_SDI", "U2", "14"),
        "R12": ("IMU1_CS", "IMU1_CS_N", "U2", "12"), "R18": ("SPI1_MISO", "IMU1_SDO", "U2", "1"),
        "R19": ("SPI4_SCK", "IMU2_SCLK", "U3", "13"), "R20": ("SPI4_MOSI", "IMU2_SDI", "U3", "14"),
        "R21": ("IMU2_CS", "IMU2_CS_N", "U3", "12"), "R22": ("SPI4_MISO", "IMU2_SDO", "U3", "1"),
    }
    imu_exp = {
        "IMU1_FB": {("FB2", "2"), ("R8", "1")},
        "IMU2_FB": {("FB3", "2"), ("R9", "1")},
        "+3V3_IMU1": {("R8", "2"), ("C17", "1"), ("C18", "1"), ("C19", "1"), ("U2", "8"), ("U2", "5")},
        "+3V3_IMU2": {("R9", "2"), ("C20", "1"), ("C21", "1"), ("C22", "1"), ("U3", "8"), ("U3", "5")},
        "IMU_CLKIN": {("Y2", "3"), ("R14", "1"), ("R15", "1"), ("R16", "1")},
        "IMU1_CLKIN": {("R14", "2"), ("U2", "9")},
        "IMU2_CLKIN": {("R15", "2"), ("U3", "9")},
        "Y2_OE": {("Y2", "1"), ("R17", "2")},
    }
    for ref, (mcu_net, inet, u, upin) in IMU_SPI.items():
        imu_exp[inet] = {(ref, "2"), (u, upin)}
        expected[mcu_net] = expected[mcu_net] | {(ref, "1")}
    expected["IMU1_DRDY"] = expected["IMU1_DRDY"] | {("U2", "4")}
    expected["IMU2_DRDY"] = expected["IMU2_DRDY"] | {("U3", "4")}
    expected["IMU_CLKIN_MON"] = expected["IMU_CLKIN_MON"] | {("R16", "2")}
    expected.update(imu_exp)
    # Rev 12.0 DPS310 (S10): independent expected table.
    baro_exp = {
        "+3V3_BARO": {("FB4", "2"), ("C24", "1"), ("C25", "1"), ("C26", "1"),
                      ("U4", "8"), ("U4", "6"), ("U4", "2")},
        "BARO_SCL": {("R25", "2"), ("U4", "4")},
        "BARO_SDA": {("R26", "2"), ("U4", "3")},
    }
    expected["I2C1_SCL"] = expected["I2C1_SCL"] | {("R23", "2"), ("R25", "1")}
    expected["I2C1_SDA"] = expected["I2C1_SDA"] | {("R24", "2"), ("R26", "1")}
    expected.update(baro_exp)
    imu_exp_s9 = dict(imu_exp)
    imu_exp.update(baro_exp)    # excluded from the plain hwdef-pin check below
    for k, v in sorted(expected.items()):
        if k in ("HSE_IN", "HSE_OUT", "HSE_OUT_MCU", "VDDA_FB", "+3.3VA", "+3V3", "GND", "VCAP", "NRST", "BOOT0"):
            got = nets.get(k, set())
            check(got == v, "%s exact: missing %s extra %s" % (k, sorted(v - got), sorted(got - v)))
    hw_bad = [k for k in expected if k not in ("HSE_IN", "HSE_OUT", "HSE_OUT_MCU", "VDDA_FB", "+3.3VA", "+3V3",
                                               "GND", "VCAP", "NRST", "BOOT0") and k not in imu_exp
              and nets.get(k) != expected[k]]
    check(not hw_bad, "S6 %d hwdef.dat pins on the official LQFP-100 pin numbers %s" % (len(hw), hw_bad[:5]))
    extra = [k for k in nets if k not in expected and not k.startswith("unconnected-")]
    check(not extra, "no unexpected nets %s" % extra[:5])
    stray = [k for k in nets if k.startswith("unconnected-") and len(nets[k]) != 1]
    check(not stray, "every unconnected net is a single no-connect pin")
    nc_got = {nd for k in nets if k.startswith("unconnected-") for nd in nets[k]}
    gpio_nc = {("U1", str(port_to_num[pt])) for pt in port_to_num
               if re.match(r"^P[A-K]\d+$", pt) and pt not in hw and pt not in ("PH0", "PH1")}  # PH0/PH1 = HSE
    imu_nc = {(u, p_) for u in ("U2", "U3") for p_ in ("2", "3", "10", "11")}
    check(nc_got == gpio_nc | imu_nc,
          "no-connects are exactly the %d unassigned GPIOs + ICM-42688-P RESV 2/3/10/11 (missing %s extra %s)"
          % (len(gpio_nc), sorted((gpio_nc | imu_nc) - nc_got)[:5], sorted(nc_got - gpio_nc - imu_nc)[:5]))

    print("S5 values, MPNs, fitted/BOM state")
    values = {"U1": "STM32H743VITx", "Y1": "24MHz CL=12pF", "C1": "16pF", "C2": "16pF", "R1": "0R",
              "FB1": "600R@100MHz", "R2": "2R2", "C3": "1uF", "C4": "100nF", "C5": "1uF", "C6": "100nF",
              "C7": "2.2uF", "C8": "2.2uF", "C9": "100nF", "C10": "100nF", "C11": "100nF", "C12": "100nF",
              "C13": "100nF", "C14": "4.7uF", "C15": "100nF", "C16": "100nF", "R3": "10k",
              "R4": "100R", "R5": "100R", "R6": "100R", "R7": "100R",
              "U4": "DPS310", "FB4": "600R@100MHz", "C24": "1uF", "C25": "100nF", "C26": "100nF",
              "R23": "2.2k", "R24": "2.2k", "R25": "33R", "R26": "33R"}
    imu_values = {"U2": "ICM-42688-P", "U3": "ICM-42688-P", "Y2": "32.768kHz CMOS",
                  "FB2": "600R@100MHz", "FB3": "600R@100MHz", "R8": "22R", "R9": "22R",
                  "C17": "10uF", "C20": "10uF", "C18": "100nF", "C19": "100nF", "C21": "100nF",
                  "C22": "100nF", "C23": "100nF", "R14": "33R", "R15": "33R", "R16": "1k", "R17": "10k"}
    imu_values.update({r: "33R" for r in IMU_SPI})
    values.update(imu_values)
    vbad = [(r, comps.get(r, {}).get("value"), v) for r, v in values.items() if comps.get(r, {}).get("value") != v]
    check(not vbad, "all %d part values exact %s" % (len(values), vbad))
    gen_sheet, _ = gen.build()
    gen_refs = set(gen_sheet.refs)
    check(len(gen_refs) == len(gen_sheet.refs), "generator places every reference once")
    check(set(comps) == set(values) == gen_refs,
          "BOM refs: netlist %d == value table %d == generator part list %d (netlist-only %s, table-only %s, "
          "generator-only %s)" % (len(comps), len(values), len(gen_refs), sorted(set(comps) - set(values)),
                                  sorted(set(values) - set(comps)), sorted(gen_refs - set(comps))))
    mpns = {"U1": "STM32H743VIT6", "Y1": "TXM24M0004322FBCDO00T", "C1": "GJM1555C1H160FB01J",
            "C2": "GJM1555C1H160FB01J", "FB1": "BLM15AX601SN1D", "R2": "0402WGF220KTCE",
            "C7": "CL10B225KP8NNNC", "C8": "CL10B225KP8NNNC",
            "R4": "0402WGF1000TCE", "R5": "0402WGF1000TCE", "R6": "0402WGF1000TCE", "R7": "0402WGF1000TCE",
            "U2": "ICM-42688-P", "U3": "ICM-42688-P", "Y2": "SX2M32.768KM20F30TNN",
            "FB2": "BLM15AX601SN1D", "FB3": "BLM15AX601SN1D", "R8": "0402WGF220JTCE", "R9": "0402WGF220JTCE",
            "C17": "CL10A106KP8NNNC", "C20": "CL10A106KP8NNNC", "R16": "0402WGF1001TCE"}
    mpns.update({r: "0402WGF330JTCE" for r in list(IMU_SPI) + ["R14", "R15", "R25", "R26"]})
    mpns.update({"U4": "DPS310XTSA1", "FB4": "BLM15AX601SN1D", "C24": "CL05A105KA5NQNC",
                 "R23": "0402WGF2201TCE", "R24": "0402WGF2201TCE"})
    mbad = [(r, comps.get(r, {}).get("fields", {}).get("MPN"), m) for r, m in mpns.items()
            if comps.get(r, {}).get("fields", {}).get("MPN") != m]
    check(not mbad, "key MPNs (Y1, C1/C2, FB1, R2, VCAP, U1, R4-R7, U2/U3, Y2, FB2/3, R8/9, C17/20, 33R, R16) %s"
          % mbad)
    lcsc = {"U2": "C1850418", "U3": "C1850418", "Y2": "C2901603", "U4": "C130156",
            "R23": "C25879", "R24": "C25879"}
    lbad = [(r, comps.get(r, {}).get("fields", {}).get("LCSC"), c) for r, c in lcsc.items()
            if comps.get(r, {}).get("fields", {}).get("LCSC") != c]
    check(not lbad, "S9/S10 LCSC U2/U3 C1850418, Y2 C2901603, U4 C130156, R23/R24 C25879 %s" % lbad)
    sch_txt = open(SCH).read()
    check("(dnp yes)" not in sch_txt, "no part marked DNP")
    nobom = re.findall(r'\(in_bom no\).*?\(property "Reference" "([^"]+)"', sch_txt, re.S)
    check(all(r.startswith("#") for r in nobom), "only power/flag symbols excluded from BOM")
    check("V required" in comps["U1"]["fields"].get("Silicon_Rev", ""),
          "U1 procurement field requires rev V silicon (480 MHz)")

    print("S7 clock consistency")
    hz = {}
    for fn in ("hwdef.dat", "hwdef-bl.dat"):
        m = re.search(r"^OSCILLATOR_HZ\s+(\d+)", open(os.path.join(REPO, "ardupilot", fn)).read(), re.M)
        hz[fn] = int(m.group(1)) if m else None
    yf = int(re.match(r"(\d+)MHz", comps["Y1"]["value"]).group(1)) * 1000000
    check(hz["hwdef.dat"] == yf and hz["hwdef-bl.dat"] == yf,
          "OSCILLATOR_HZ fw=%s bl=%s == Y1 %d Hz" % (hz["hwdef.dat"], hz["hwdef-bl.dat"], yf))
    check(re.search(r"^MCU_CLOCKRATE_MHZ\s+480\b", open(os.path.join(REPO, "ardupilot", "hwdef.dat")).read(), re.M)
          is not None, "hwdef.dat MCU_CLOCKRATE_MHZ 480")

    print("S9 dual ICM-42688-P block (Rev 11.0)")
    for k in sorted(imu_exp_s9):
        got = nets.get(k, set())
        check(got == imu_exp[k], "%s exact: missing %s extra %s" % (k, sorted(imu_exp[k] - got), sorted(got - imu_exp[k])))
    for k in ("SPI1_SCK", "SPI1_MOSI", "SPI1_MISO", "IMU1_CS", "SPI4_SCK", "SPI4_MOSI", "SPI4_MISO", "IMU2_CS",
              "IMU1_DRDY", "IMU2_DRDY", "IMU_CLKIN_MON"):
        got = nets.get(k, set())
        check(got == expected[k], "%s exact (U1 pin + IMU-side part): missing %s extra %s"
              % (k, sorted(expected[k] - got), sorted(got - expected[k])))
    # every U2/U3 pin accounted for (14 each): on an expected net or a RESV no-connect
    for u in ("U2", "U3"):
        pins_on = {p_ for k in nets for (r, p_) in nets[k] if r == u}
        check(pins_on == {str(i) for i in range(1, 15)}, "%s all 14 pins present in netlist %s" % (u, sorted(pins_on)))
    # supply filter chain order: +3V3 -> FB -> IMUn_FB -> R -> +3V3_IMUn
    for fb, r, mid, rail, u in (("FB2", "R8", "IMU1_FB", "+3V3_IMU1", "U2"), ("FB3", "R9", "IMU2_FB", "+3V3_IMU2", "U3")):
        ok = (net_of(fb, "1") == "+3V3" and net_of(fb, "2") == mid and net_of(r, "1") == mid
              and net_of(r, "2") == rail and net_of(u, "8") == rail and net_of(u, "5") == rail
              and net_of(u, "6") == "GND" and net_of(u, "7") == "GND")
        check(ok, "%s supply: +3V3 -> %s -> %s -> %s 22R -> %s (VDD+VDDIO), GND+RESV7 -> GND" % (u, fb, mid, r, rail))
    # CLKIN star: Y2 OUT is the only driver; each IMU pin 9 reached through exactly one 33R
    star = (net_of("Y2", "3") == "IMU_CLKIN" and net_of("U2", "9") != net_of("U3", "9")
            and net_of("U1", "59") == "IMU_CLKIN_MON" and net_of("Y2", "4") == "+3V3"
            and net_of("Y2", "2") == "GND" and net_of("R17", "1") == "+3V3")
    check(star, "CLKIN star: Y2.3 -> R14 -> U2.9, R15 -> U3.9, R16 1k -> U1 PD12 (pin 59); Y2 OE pulled to +3V3")
    # DRDY EXTI uniqueness
    drdy = {v: k for k, v in hw.items() if v in ("IMU1_DRDY", "IMU2_DRDY")}
    exti = {k: int(re.match(r"P[A-K](\d+)", drdy[k]).group(1)) for k in drdy}
    check(drdy.get("IMU1_DRDY") == "PC4" and drdy.get("IMU2_DRDY") == "PE1",
          "DRDY ports IMU1=PC4 IMU2=PE1 (%s)" % drdy)
    check(len(exti) == 2 and exti["IMU1_DRDY"] != exti["IMU2_DRDY"],
          "IMU1_DRDY / IMU2_DRDY on distinct EXTI lines %s" % exti)
    check(net_of("U1", "32") == "IMU1_DRDY" and net_of("U1", "98") == "IMU2_DRDY",
          "U1 pin 32 (PC4) = IMU1_DRDY, pin 98 (PE1) = IMU2_DRDY")
    hwtxt = open(gen.HWDEF).read()
    m = re.search(r"^define\s+INVENSENSEV3_CLKIN_BITMASK\s+(\S+)", hwtxt, re.M)
    bm = int(m.group(1), 0) if m else 0
    check(m is not None and (bm & 0b11) == 0b11,
          "hwdef.dat INVENSENSEV3_CLKIN_BITMASK %s has bits 0 and 1 set" % (m.group(1) if m else None))
    imu_order = re.findall(r"^IMU\s+Invensensev3\s+SPI:(\w+)", hwtxt, re.M)
    spidev = dict(re.findall(r"^SPIDEV\s+(\w+)\s+(SPI\d)\b", hwtxt, re.M))
    check(imu_order[:2] == ["imu1", "imu2"] and spidev.get("imu1") == "SPI1" and spidev.get("imu2") == "SPI4",
          "bitmask bit0 = imu1 on SPI1 (U2), bit1 = imu2 on SPI4 (U3) %s %s" % (imu_order, spidev))

    print("S10 DPS310 barometer on I2C1 (Rev 12.0)")
    dps_lib = {}
    for lp in find(find(nl, "libparts")[0], "libpart"):
        if find(lp, "part")[0][1] == "DPS310":
            dps_lib = {find(pn, "num")[0][1]: find(pn, "name")[0][1] for pn in find(find(lp, "pins")[0], "pin")}
    check(dps_lib == DPS310_REF_PINS, "U4 symbol pins == DPS310 pinout (LCSC C130156) %s" % dps_lib)
    for k in sorted(baro_exp) + ["I2C1_SCL", "I2C1_SDA"]:
        got = nets.get(k, set())
        check(got == expected[k], "%s exact: missing %s extra %s" % (k, sorted(expected[k] - got), sorted(got - expected[k])))
    pins_on = {p_ for k in nets for (r, p_) in nets[k] if r == "U4"}
    check(pins_on == set(DPS310_REF_PINS), "U4 all 8 pins present in netlist %s" % sorted(pins_on))
    check(net_of("FB4", "1") == "+3V3" and net_of("FB4", "2") == "+3V3_BARO"
          and net_of("U4", "8") == "+3V3_BARO" and net_of("U4", "6") == "+3V3_BARO",
          "supply: +3V3 -> FB4 -> +3V3_BARO on U4 VDD (8) and VDDIO (6)")
    check(all(net_of(c, "1") == "+3V3_BARO" and net_of(c, "2") == "GND" for c in ("C24", "C25", "C26")),
          "C24 1uF / C25 100nF (VDD) / C26 100nF (VDDIO) between +3V3_BARO and GND")
    check(net_of("U4", "1") == "GND" and net_of("U4", "7") == "GND", "U4 GND pins 1/7 on GND")
    for r, mn in (("R23", "I2C1_SCL"), ("R24", "I2C1_SDA")):
        check(net_of(r, "1") == "+3V3" and net_of(r, "2") == mn and comps[r]["value"] == "2.2k",
              "%s 2.2k pull-up %s -> +3V3" % (r, mn))
    for r, mn, bn, up in (("R25", "I2C1_SCL", "BARO_SCL", "4"), ("R26", "I2C1_SDA", "BARO_SDA", "3")):
        check(net_of(r, "1") == mn and net_of(r, "2") == bn and net_of("U4", up) == bn
              and comps[r]["value"] == "33R",
              "%s 33R series %s -> %s -> U4 pin %s (%s)" % (r, mn, bn, up, DPS310_REF_PINS[up]))
    csb_i2c = net_of("U4", "2") == net_of("U4", "6") == "+3V3_BARO"
    check(csb_i2c, "U4 CSB (2) tied to VDDIO (6) net -> I2C interface selected")
    sdo = net_of("U4", "5")
    addr = {"GND": 0x76, "+3V3_BARO": 0x77, "+3V3": 0x77}.get(sdo)
    m = re.search(r"^BARO\s+DPS310\s+I2C:(\d+):(0x[0-9A-Fa-f]+)\b", open(gen.HWDEF).read(), re.M)
    hw_addr = int(m.group(2), 16) if m else None
    check(addr is not None and addr == hw_addr and m.group(1) == "0",
          "U4 SDO on %s => 7-bit addr %s == hwdef 'BARO DPS310 I2C:%s:%s'"
          % (sdo, hex(addr) if addr else None, m.group(1) if m else "?", m.group(2) if m else "?"))
    i2c_order = re.findall(r"^I2C_ORDER\s+(.+)$", open(gen.HWDEF).read(), re.M)
    check(bool(i2c_order) and i2c_order[0].split()[0] == "I2C1", "hwdef I2C_ORDER bus 0 = I2C1 %s" % i2c_order)
    check(hw.get("PB6") == "I2C1_SCL" and hw.get("PB7") == "I2C1_SDA"
          and net_of("U1", str(port_to_num["PB6"])) == "I2C1_SCL" and net_of("U1", str(port_to_num["PB7"])) == "I2C1_SDA"
          and port_to_num["PB6"] == 92 and port_to_num["PB7"] == 93,
          "I2C1_SCL/SDA on U1 PB6 (pin 92) / PB7 (pin 93) per hwdef")
    check(not (dang & {"I2C1_SCL", "I2C1_SDA"}), "I2C1 labels closed on this sheet (not dangling)")
    u4fp = re.search(r'\(property "Reference" "U4".*?\(property "Footprint" "([^"]*)"', sch_txt, re.S)
    u4fp = u4fp.group(1) if u4fp else ""
    check(u4fp == gen.FP_DPS310, "U4 footprint %s" % u4fp)
    vpath, vmeta, pads = vendored_footprint(u4fp) if u4fp else ("none", None, {})
    check(vmeta is not None and vmeta.get("lib_id") == u4fp and vmeta.get("kicad_tag") == KICAD_FP_TAG,
          "U4 footprint pad table vendored from KiCad %s (%s)" % (KICAD_FP_TAG, os.path.relpath(vpath, REPO)))
    # KiCad pad (x, y) rotated +90 deg (x, y) -> (-y, x); rotation keeps chirality
    rot = {n: (-d["at"][1], d["at"][0]) for n, d in pads.items()}
    dev = max((abs(rot[n][0] - DPS310_PADS[n][0]) + abs(rot[n][1] - DPS310_PADS[n][1])
               for n in DPS310_PADS if n in rot), default=99)
    check(set(pads) == set(DPS310_PADS) and dev <= 0.03,
          "U4 land pattern (vendored) == DPS310 pad map (8 pads, 0.65 pitch, 1.6 mm rows, "
          "clockwise numbering), max deviation %.3f mm" % dev)
    check(bool(pads) and all(d["type"] == "smd" and "F.Cu" in d["layers"] and "F.Mask" in d["layers"]
                             and sorted(d["size"]) == [0.35, 0.5] for d in pads.values()),
          "U4 pads all SMD F.Cu/F.Mask, 0.35 x 0.5 mm (< 0.65 pitch, no pad bridging)")
    src, txt = kicad_footprint_text(u4fp) if u4fp else ("none", None)
    if txt is None:
        print("  [INFO] live KiCad %s footprint cross-check not run: %s (vendored table used)" % (KICAD_FP_TAG, src))
    else:
        live = parse_pads(txt)
        sha_ok = (hashlib.sha256(txt.encode()).hexdigest() == vmeta.get("source_sha256")) if vmeta else False
        check(live == pads, "cross-check: live stock footprint pads == vendored table (%s%s)"
              % (src, ", byte-identical" if sha_ok else ", file differs outside pads" if live == pads else ""))
    print("\nRESULT: %s (%d failures)" % ("PASS" if not FAIL else "FAIL", len(FAIL)))
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
