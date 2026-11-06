#!/usr/bin/env python3
"""
Nano H743 AIO - MCU core schematic verifier (Revision 7.0)

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
      no unexpected nets, every other GPIO a single no-connect pin.
  S5  Exact values for all 22 parts, key MPNs, no DNP, only #PWR/#FLG
      symbols out of BOM, U1 rev V silicon procurement field.
  S8  (Rev 10.0) DShot series resistors R4..R7: U1 TIM1_CHn -> 100R ->
      ESCn_DSHOT, in hwdef PWM(n) order.
  S7  Clock consistency: OSCILLATOR_HZ in hwdef.dat and hwdef-bl.dat equals
      the Y1 frequency, and hwdef.dat sets MCU_CLOCKRATE_MHZ 480.

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
    caps_3v3 = ["C9", "C10", "C11", "C12", "C13", "C14", "C15"]
    all_caps = ["C%d" % i for i in range(1, 17)]
    expected = {
        "HSE_IN": U(12) | {("Y1", "1"), ("C1", "1")},
        "HSE_OUT": {("Y1", "3"), ("C2", "1"), ("R1", "2")},
        "HSE_OUT_MCU": U(13) | {("R1", "1")},
        "VDDA_FB": {("FB1", "2"), ("R2", "1")},
        "+3.3VA": U(20, 21) | {("R2", "2")} | P(["C3", "C4", "C5", "C6"], "1"),
        "+3V3": U(6, 11, 27, 50, 75, 100) | {("FB1", "1")} | P(caps_3v3, "1"),
        "GND": U(10, 26, 49, 74, 99, 19) | {("Y1", "2"), ("Y1", "4"), ("R3", "2")} | P(all_caps, "2"),
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
    for k, v in sorted(expected.items()):
        if k in ("HSE_IN", "HSE_OUT", "HSE_OUT_MCU", "VDDA_FB", "+3.3VA", "+3V3", "GND", "VCAP", "NRST", "BOOT0"):
            got = nets.get(k, set())
            check(got == v, "%s exact: missing %s extra %s" % (k, sorted(v - got), sorted(got - v)))
    hw_bad = [k for k in expected if k not in ("HSE_IN", "HSE_OUT", "HSE_OUT_MCU", "VDDA_FB", "+3.3VA", "+3V3",
                                               "GND", "VCAP", "NRST", "BOOT0") and nets.get(k) != expected[k]]
    check(not hw_bad, "S6 %d hwdef.dat pins on the official LQFP-100 pin numbers %s" % (len(hw), hw_bad[:5]))
    extra = [k for k in nets if k not in expected and not k.startswith("unconnected-")]
    check(not extra, "no unexpected nets %s" % extra[:5])
    stray = [k for k in nets if k.startswith("unconnected-") and len(nets[k]) != 1]
    check(not stray, "every unconnected net is a single no-connect U1 pin")

    print("S5 values, MPNs, fitted/BOM state")
    values = {"U1": "STM32H743VITx", "Y1": "24MHz CL=12pF", "C1": "16pF", "C2": "16pF", "R1": "0R",
              "FB1": "600R@100MHz", "R2": "2R2", "C3": "1uF", "C4": "100nF", "C5": "1uF", "C6": "100nF",
              "C7": "2.2uF", "C8": "2.2uF", "C9": "100nF", "C10": "100nF", "C11": "100nF", "C12": "100nF",
              "C13": "100nF", "C14": "4.7uF", "C15": "100nF", "C16": "100nF", "R3": "10k",
              "R4": "100R", "R5": "100R", "R6": "100R", "R7": "100R"}
    vbad = [(r, comps.get(r, {}).get("value"), v) for r, v in values.items() if comps.get(r, {}).get("value") != v]
    check(not vbad, "all %d part values exact %s" % (len(values), vbad))
    check(set(comps) == set(values), "BOM refs exactly %s (extra %s)" % (len(values), sorted(set(comps) - set(values))))
    mpns = {"U1": "STM32H743VIT6", "Y1": "TXM24M0004322FBCDO00T", "C1": "GJM1555C1H160FB01J",
            "C2": "GJM1555C1H160FB01J", "FB1": "BLM15AX601SN1D", "R2": "0402WGF220KTCE",
            "C7": "CL10B225KP8NNNC", "C8": "CL10B225KP8NNNC",
            "R4": "0402WGF1000TCE", "R5": "0402WGF1000TCE", "R6": "0402WGF1000TCE", "R7": "0402WGF1000TCE"}
    mbad = [(r, comps.get(r, {}).get("fields", {}).get("MPN"), m) for r, m in mpns.items()
            if comps.get(r, {}).get("fields", {}).get("MPN") != m]
    check(not mbad, "key MPNs (Y1, C1/C2, FB1, R2, VCAP, U1, R4-R7) %s" % mbad)
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

    print("\nRESULT: %s (%d failures)" % ("PASS" if not FAIL else "FAIL", len(FAIL)))
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
