#!/usr/bin/env python3
"""
Nano H743 AIO - bidirectional DShot600 / ESC telemetry validator (Revision 10.0)

Checks the board's bidir-DShot configuration against a real ArduPilot tree
instead of against comments:

  V1  hwdef.dat generates; hwdef.h defines HAL_WITH_BIDIR_DSHOT, a TIM1_UP
      DShot DMA stream and an input-capture DMA stream for every BIDIR channel.
  V2  Every motor output can be input-captured: it is BIDIR itself, or its
      CH1/CH2 / CH3/CH4 partner on the same timer is BIDIR and has IC DMA
      (RCOutput::bdshot_setup_group_ic_DMA pairing rule).  No CHxN outputs.
      SERVO_BLH_BDMASK covers exactly the motor outputs.
  V3  Every parameter in defaults.parm exists in the ArduPilot source
      (@Param docs) and its value is inside the documented @Values /
      @Bitmask / @Range.
  V4  Bit-exact decoder cross-check: ArduPilot's own
      RCOutput::bdshot_decode_telemetry_packet() is extracted from the tree,
      compiled next to firmware/bdshot_telem and both are run over a corpus
      (all 4096 payloads x timing errors, jitter, dropped/extra edges and
      200 000 random edge trains).  0 mismatches required.  The eRPM
      arithmetic statements are checked verbatim against the source.
  V5  firmware/bdshot_telem unit tests (ASan + UBSan) pass.
  V6  analysis/bdshot_timing_analysis.py gates pass.
  V7  ardupilot/patches/*.patch apply cleanly to the tree (dry run).

Usage:
  python3 tools/validate_bdshot.py --ardupilot /path/to/ardupilot
  (or set ARDUPILOT_ROOT)
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

FW = os.path.join(REPO, "firmware", "bdshot_telem")
PARM = os.path.join(REPO, "ardupilot", "defaults.parm")
HWDEF = os.path.join(REPO, "ardupilot", "hwdef.dat")

# parameter prefix -> (source file, group prefix inside @Param names)
PARAM_SOURCES = [
    ("SERVO_BLH_", "libraries/AP_BLHeli/AP_BLHeli.cpp"),
    ("SERVO_", "libraries/SRV_Channel/SRV_Channels.cpp"),
    ("INS_HNTCH_", "libraries/Filter/HarmonicNotchFilter.cpp"),
    ("INS_", "libraries/AP_InertialSensor/AP_InertialSensor.cpp"),
    ("MOT_", "libraries/AP_Motors/AP_MotorsMulticopter.cpp"),
    ("LOG_", "libraries/AP_Logger/AP_Logger.cpp"),
    ("SCHED_", "libraries/AP_Scheduler/AP_Scheduler.cpp"),
]

FAILS = []


def check(ok, msg):
    print("  [%s] %s" % ("PASS" if ok else "FAIL", msg))
    if not ok:
        FAILS.append(msg)


# ---------------------------------------------------------------------------
def param_doc(ap_root, name):
    for prefix, rel in PARAM_SOURCES:
        if not name.startswith(prefix):
            continue
        suffix = name[len(prefix):]
        txt = open(os.path.join(ap_root, rel)).read()
        blocks = re.split(r"//\s*@Param(?:\{[^}]*\})?:\s*", txt)[1:]
        for b in blocks:
            pname = b.split()[0]
            if pname in (suffix, "_" + suffix):
                return rel, b.split("AP_GROUP", 1)[0]
        return rel, None
    return None, None


def value_ok(doc, val):
    m = re.search(r"@Values(?:\{[^}]*\})?:\s*(.+)", doc)
    if m:
        keys = {float(k) for k in re.findall(r"(-?[\d.]+)\s*:", m.group(1))}
        return val in keys, "values %s" % sorted(keys)
    bits = re.findall(r"@Bitmask(?:\{[^}]*\})?:\s*(.+)", doc)
    if bits:
        allowed = {int(k) for line in bits for k in re.findall(r"(\d+)\s*:", line)}
        v = int(val)
        bad = [i for i in range(32) if v & (1 << i) and i not in allowed]
        return not bad and v == val, "bitmask bits %s" % sorted(allowed)
    m = re.search(r"@Range(?:\{[^}]*\})?:\s*(-?[\d.]+)\s+(-?[\d.]+)", doc)
    if m:
        lo, hi = float(m.group(1)), float(m.group(2))
        return lo <= val <= hi, "range %g..%g" % (lo, hi)
    return True, "undocumented range"


# ---------------------------------------------------------------------------
HARNESS = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bdshot_telem.h"
typedef uint32_t dmar_uint_t;   // STM32H7: RCOutput.h non-F1 branch
typedef int32_t  dmar_int_t;
static const uint16_t TELEM_IC_SAMPLE = 16;
static const uint32_t INVALID_ERPM = 0xffffU;
static inline uint32_t lsl_m7(uint32_t x, uint32_t n) { n &= 0xFFU; return n >= 32U ? 0U : (x << n); }
/* ---- extracted verbatim from ArduPilot, shifts mapped to Cortex-M7 LSL ---- */
%s
/* ------------------------------------------------------------------------- */
static uint32_t g = 0x5EED743u;
static uint32_t rnd(uint32_t n) { g = g * 1664525u + 1013904223u; return (g >> 8) %% n; }
static long n_cases = 0, n_mismatch = 0, n_valid = 0;
static void cmp(const uint16_t* e, uint32_t n) {
    dmar_uint_t b[23] = {0};
    for (uint32_t i = 0; i < n; i++) b[i] = e[i];
    const uint32_t a = ap_decode(b, n);
    const uint32_t m = nano_bdshot::decode_gcr_edges(e, n);
    n_cases++;
    if (a != INVALID_ERPM) n_valid++;
    if (a != m) { if (n_mismatch++ < 5) printf("mismatch n=%%u ap=%%x ours=%%x\n", n, a, m); }
}
int main() {
    uint16_t e[23];
    const uint32_t tpb[] = {16*256, 15*256, 17*256, 14*256, 18*256, 13*256, 19*256, 3840, 4352};
    for (uint32_t p = 0; p < 4096; p++) {
        for (uint32_t t : tpb) {
            uint32_t n = nano_bdshot::encode_gcr_edges(p, 300, t, e, 22);
            cmp(e, n);
            for (int j = 0; j < 3; j++) {          // jitter
                uint16_t f[23]; memcpy(f, e, sizeof f);
                for (uint32_t i = 1; i < n; i++) f[i] = uint16_t(f[i] + rnd(2 * (4 + 3 * j) + 1) - (4 + 3 * j));
                cmp(f, n);
            }
            if (n > 2) {                            // dropped edge
                uint16_t f[23]; memcpy(f, e, sizeof f);
                uint32_t k = 1 + rnd(n - 1);
                memmove(&f[k], &f[k + 1], (n - k - 1) * sizeof f[0]);
                cmp(f, n - 1);
            }
            if (n < 22) {                           // extra glitch edge
                uint16_t f[23]; memcpy(f, e, sizeof f);
                uint32_t k = 1 + rnd(n - 1);
                memmove(&f[k + 1], &f[k], (n - k) * sizeof f[0]);
                f[k] = uint16_t(f[k - 1] + 1 + rnd(uint32_t(f[k + 1] - f[k - 1])));
                cmp(f, n + 1);
            }
        }
    }
    for (int r = 0; r < 200000; r++) {             // random edge trains
        uint32_t n = 1 + rnd(22);
        uint16_t t = uint16_t(rnd(1000));
        for (uint32_t i = 0; i < n; i++) { e[i] = t; t = uint16_t(t + 1 + rnd(60)); }
        cmp(e, n);
    }
    printf("cases %%ld valid %%ld mismatches %%ld\n", n_cases, n_valid, n_mismatch);
    return n_mismatch ? 1 : 0;
}
'''

ERPM_STATEMENTS = [
    r"uint8_t expo = ((encodederpm & 0xfffffe00U) >> 9U) & 0xffU;",
    r"uint16_t value = (encodederpm & 0x000001ffU);",
    r"if (!(value & 0x100U) && (_dshot_esc_type == DSHOT_ESC_BLHELI_EDT || _dshot_esc_type == DSHOT_ESC_BLHELI_EDT_S)) {",
    r"uint16_t erpm = value << expo;",
    r"erpm = (1000000U * 60U / 100U + erpm / 2U) / erpm;",
    r"if (encodederpm == ZERO_ERPM) {",
    r"update_rpm(normalized_chan, erpm * 200U / _bdshot.motor_poles, get_erpm_error_rate(chan));",
    r"static const uint16_t TELEM_IC_SAMPLE = 16;",
    r"static const uint16_t ZERO_ERPM = 0x0fffU;",
    r"telempsc = (uint16_t)(lrintf(((float)group.pwm_drv->clock / bdshot_get_output_rate_hz(group.current_mode) + 0.01f)/TELEM_IC_SAMPLE) - 1);",
    r"return 600000U * 5 / 4;",
]


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
    raise ValueError("unbalanced")


def bit_exact(ap_root, tmp):
    bd = os.path.join(ap_root, "libraries", "AP_HAL_ChibiOS", "RCOutput_bdshot.cpp")
    hdr = os.path.join(ap_root, "libraries", "AP_HAL_ChibiOS", "RCOutput.h")
    src = open(bd).read()
    both = src + open(hdr).read()
    missing = [s for s in ERPM_STATEMENTS if s not in both]
    check(not missing, "V4 ArduPilot eRPM/IC arithmetic matches the modelled statements %s" % missing[:2])

    fn = extract_function(src, "uint32_t RCOutput::bdshot_decode_telemetry_packet(dmar_uint_t* buffer, uint32_t count)")
    fn = fn.replace("uint32_t RCOutput::bdshot_decode_telemetry_packet", "static uint32_t ap_decode", 1)
    subs = [("value <<= len;", "value = lsl_m7(value, len);"),
            ("value |= 1U << (len - 1U);", "value |= lsl_m7(1U, len - 1U);")]
    for a, b in subs:
        check(fn.count(a) == 1, "V4 upstream decoder contains '%s' exactly once" % a)
        fn = fn.replace(a, b)
    hfile = os.path.join(tmp, "harness.cpp")
    with open(hfile, "w") as fh:
        fh.write(HARNESS % fn)
    exe = os.path.join(tmp, "harness")
    r = subprocess.run(["g++", "-std=c++17", "-O2", "-I", FW, "-o", exe, hfile,
                        os.path.join(FW, "bdshot_telem.cpp")], capture_output=True, text=True)
    check(r.returncode == 0, "V4 harness compiles against upstream source %s" % r.stderr[-300:])
    if r.returncode:
        return
    r = subprocess.run([exe], capture_output=True, text=True)
    print("      " + r.stdout.strip().replace("\n", "\n      "))
    check(r.returncode == 0, "V4 firmware/bdshot_telem decoder is bit-exact with ArduPilot")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ardupilot", default=os.environ.get("ARDUPILOT_ROOT"))
    args = ap.parse_args()
    if not args.ardupilot or not os.path.isdir(args.ardupilot):
        print("ERROR: pass --ardupilot <ArduPilot checkout> or set ARDUPILOT_ROOT")
        return 2
    ap_root = os.path.abspath(args.ardupilot)
    tmp = tempfile.mkdtemp(prefix="nanoh743_bdshot_")
    try:
        params = {}
        for line in open(PARM):
            line = line.split("#", 1)[0].strip()
            if line:
                k, v = line.split()[:2]
                params[k] = float(v)

        print("V1 generated hwdef.h")
        bdir = vdma.stage(tmp)
        out = os.path.join(tmp, "fw")
        os.makedirs(out)
        rc, log = vdma.run_generator(ap_root, os.path.join(bdir, "hwdef.dat"), out,
                                     params=os.path.join(bdir, "defaults.parm"))
        check(rc == 0, "hwdef.dat generates (rc=%d)" % rc)
        if rc:
            print(log[-2000:])
            return 1
        hdr = open(os.path.join(out, "hwdef.h")).read()
        check("#define HAL_WITH_BIDIR_DSHOT" in hdr, "HAL_WITH_BIDIR_DSHOT defined")
        streams = {m.group(1): (int(m.group(2)), int(m.group(3))) for m in vdma.STREAM_RE.finditer(hdr)}

        pwm = []
        for line in open(HWDEF):
            m = re.match(r"^(P[A-K]\d+)\s+TIM(\d+)_CH(\d)(N?)\s+TIM\d+\s+PWM\((\d+)\)(.*)$", line.split("#")[0].strip())
            if m:
                pwm.append({"pin": m.group(1), "tim": int(m.group(2)), "ch": int(m.group(3)),
                            "n": m.group(4) == "N", "out": int(m.group(5)), "bidir": "BIDIR" in m.group(6)})
        for t in sorted({c["tim"] for c in pwm}):
            check("TIM%d_UP" % t in streams, "TIM%d_UP DShot stream %s" % (t, streams.get("TIM%d_UP" % t)))
        for c in pwm:
            if c["bidir"]:
                k = "TIM%d_CH%d" % (c["tim"], c["ch"])
                check(k in streams, "%s IC DMA stream %s" % (k, streams.get(k)))
                if k in streams and streams[k] == streams.get("TIM%d_UP" % c["tim"]):
                    print("      %s shares the TIM%d_UP stream (DMAMUX re-point per receive window)" % (k, c["tim"]))

        print("V2 capture coverage / BDMASK")
        for c in pwm:
            partner = c["ch"] + 1 if c["ch"] % 2 else c["ch"] - 1
            ok = c["bidir"] or any(o["tim"] == c["tim"] and o["ch"] == partner and o["bidir"] and
                                   "TIM%d_CH%d" % (o["tim"], o["ch"]) in streams for o in pwm)
            check(ok and not c["n"], "PWM(%d) %s TIM%d_CH%d%s capturable (%s)" % (
                c["out"], c["pin"], c["tim"], c["ch"], "N" if c["n"] else "",
                "own IC DMA" if c["bidir"] else "via CH%d IC DMA" % partner))
        outs_mask = sum(1 << (c["out"] - 1) for c in pwm)
        check(int(params.get("SERVO_BLH_BDMASK", 0)) == outs_mask,
              "SERVO_BLH_BDMASK %d == motor outputs mask %d" % (params.get("SERVO_BLH_BDMASK", 0), outs_mask))
        per_timer = {}
        for c in pwm:
            per_timer[c["tim"]] = per_timer.get(c["tim"], 0) + 1
        print("      round-robin: %s motors per timer group -> per-motor eRPM = f_dshot / %d"
              % (per_timer, max(per_timer.values())))

        print("V3 defaults.parm against ArduPilot @Param docs")
        for name, val in params.items():
            rel, doc = param_doc(ap_root, name)
            if doc is None:
                check(False, "%s not found in %s" % (name, rel))
                continue
            ok, why = value_ok(doc, val)
            check(ok, "%s = %g valid (%s; %s)" % (name, val, os.path.basename(rel), why))

        print("V4 decoder bit-exactness vs upstream source")
        bit_exact(ap_root, tmp)

        print("V5 unit tests")
        r = subprocess.run(["make", "-s", "-C", FW, "test"], capture_output=True, text=True)
        tail = [ln for ln in r.stdout.splitlines() if "RESULT" in ln or "FAIL" in ln]
        check(r.returncode == 0, "firmware/bdshot_telem tests: %s" % (tail[-1] if tail else r.stderr[-300:]))

        print("V6 timing analysis gates")
        r = subprocess.run([sys.executable, os.path.join(REPO, "analysis", "bdshot_timing_analysis.py")],
                           capture_output=True, text=True)
        check(r.returncode == 0, "analysis/bdshot_timing_analysis.py: %s" % r.stdout.strip().splitlines()[-1])

        print("V7 upstream patches")
        pdir = os.path.join(REPO, "ardupilot", "patches")
        for pf in sorted(os.listdir(pdir)) if os.path.isdir(pdir) else []:
            if pf.endswith(".patch"):
                r = subprocess.run(["patch", "-p1", "--dry-run", "-s", "-i", os.path.join(pdir, pf)],
                                   cwd=ap_root, capture_output=True, text=True)
                check(r.returncode == 0, "%s applies cleanly %s" % (pf, r.stdout.strip()[:200]))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print("\nRESULT: %s (%d failure%s)" % ("PASS" if not FAILS else "FAIL", len(FAILS), "" if len(FAILS) == 1 else "s"))
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
