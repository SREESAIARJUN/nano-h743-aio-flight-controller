#!/usr/bin/env python3
"""
Nano H743 AIO - ArduPilot hwdef DMA allocation validator (Revision 6.0)

Runs ArduPilot's own chibios_hwdef.py / dma_resolver.py over
ardupilot/hwdef.dat (and hwdef-bl.dat), then parses the generated hwdef.h and
enforces the Rev 6.0 DMA policy:

  R1  SPI1, SPI4 and USART1 RX+TX, and TIM1_UP each have a DMA stream.
  R2  No stream is shared between different peripherals.  The single
      permitted multi-occupancy is TIMn_UP + TIMn_CHx of the SAME timer
      (ArduPilot's bidir-DShot DMAMUX time-multiplexing on H7).
  R3  SHARED_DMA_MASK == 0 (no runtime lock-sharing of any stream).
  R4  SDMMC1 is enabled (HAL_USE_SDC, STM32_SDC_USE_SDMMC1, FATFS) and
      consumes no DMA1/DMA2 stream (internal IDMA).
  R5  Stream usage <= 16 (DMA1 + DMA2); spare streams reported.
  R6  The bootloader hwdef also generates cleanly.
  R7  DMA_PRIORITY / DMA_NOSHARE are each declared exactly once.

Usage:
  python3 tools/validate_dma.py --ardupilot /path/to/ardupilot
  (or set ARDUPILOT_ROOT)

Exit status 0 = all rules pass.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HWDEF_DIR = os.path.join(REPO, "ardupilot")
BOARD = "NanoH743AIO"

CRITICAL = ["SPI1_RX", "SPI1_TX", "SPI4_RX", "SPI4_TX",
            "USART1_RX", "USART1_TX", "TIM1_UP"]

STREAM_RE = re.compile(
    r"#define\s+STM32_(?:SPI|UART|TIM|ADC|I2C|SDC)_(\w+?)_DMA_STREAM\s+"
    r"STM32_DMA_STREAM_ID\((\d+),\s*(\d+)\)")


def run_generator(ap_root, hwdef, outdir, bootloader=False, params=None):
    script = os.path.join(ap_root, "libraries", "AP_HAL_ChibiOS", "hwdef",
                          "scripts", "chibios_hwdef.py")
    cmd = [sys.executable, script, "-D", outdir, "--quiet"]
    if bootloader:
        cmd.append("--bootloader")
    cmd += ["--params", params or os.devnull, hwdef]
    r = subprocess.run(cmd, cwd=ap_root, capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr


def stage(tmp):
    """Copy the board into <tmp>/<BOARD>/ so the board name is meaningful.
    AP_BOOTLOADER_FLASHING_ENABLED is disabled in the staged copy only: the
    generator otherwise aborts mid-header when no compiled *_bl.bin exists.
    That flag only controls bootloader embedding and has no effect on DMA
    resolution."""
    bdir = os.path.join(tmp, BOARD)
    os.makedirs(bdir)
    for f in ("hwdef.dat", "hwdef-bl.dat", "defaults.parm"):
        src = os.path.join(HWDEF_DIR, f)
        if os.path.exists(src):
            shutil.copy(src, bdir)
    with open(os.path.join(bdir, "hwdef.dat"), "a") as fh:
        fh.write("\n# validate_dma.py staging only\n"
                 "define AP_BOOTLOADER_FLASHING_ENABLED 0\n")
    return bdir


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ardupilot", default=os.environ.get("ARDUPILOT_ROOT"))
    ap.add_argument("--keep", action="store_true", help="keep temp output")
    args = ap.parse_args()
    if not args.ardupilot or not os.path.isdir(args.ardupilot):
        print("ERROR: pass --ardupilot <ArduPilot checkout> or set ARDUPILOT_ROOT")
        return 2
    ap_root = os.path.abspath(args.ardupilot)

    tmp = tempfile.mkdtemp(prefix="nanoh743_dma_")
    fails = []

    def check(ok, msg):
        print("  [%s] %s" % ("PASS" if ok else "FAIL", msg))
        if not ok:
            fails.append(msg)

    try:
        bdir = stage(tmp)
        out_fw = os.path.join(tmp, "fw")
        out_bl = os.path.join(tmp, "bl")
        os.makedirs(out_fw)
        os.makedirs(out_bl)

        print("== chibios_hwdef.py: firmware ==")
        rc, log = run_generator(ap_root, os.path.join(bdir, "hwdef.dat"), out_fw,
                                params=os.path.join(bdir, "defaults.parm"))
        check(rc == 0, "hwdef.dat generates (rc=%d)" % rc)
        if rc != 0:
            print(log)
            return 1

        print("== chibios_hwdef.py: bootloader ==")
        rc, log = run_generator(ap_root, os.path.join(bdir, "hwdef-bl.dat"), out_bl,
                                bootloader=True)
        check(rc == 0, "R6 hwdef-bl.dat generates (rc=%d)" % rc)
        if rc != 0:
            print(log)

        # R7: DMA policy directives must appear exactly once. chibios_hwdef.py
        # silently lets a later duplicate overwrite an earlier one.
        src_lines = open(os.path.join(HWDEF_DIR, "hwdef.dat")).read().splitlines()
        for kw in ("DMA_PRIORITY", "DMA_NOSHARE"):
            n = sum(1 for ln in src_lines if ln.split()[:1] == [kw])
            check(n == 1, "R7 %s declared exactly once in hwdef.dat (found %d)" % (kw, n))

        hdr = open(os.path.join(out_fw, "hwdef.h")).read()

        streams = {}   # request -> (ctrl, stream)
        for m in STREAM_RE.finditer(hdr):
            streams[m.group(1)] = (int(m.group(2)), int(m.group(3)))

        print("\n== resolved DMA map ==")
        by_stream = {}
        for req, st in sorted(streams.items(), key=lambda kv: kv[1]):
            by_stream.setdefault(st, []).append(req)
        for st in sorted(by_stream):
            print("  DMA%d-%d : %s" % (st[0], st[1], ", ".join(by_stream[st])))

        print("\n== policy ==")
        for req in CRITICAL:
            check(req in streams, "R1 %s has a dedicated DMA stream" % req)

        for st, reqs in sorted(by_stream.items()):
            if len(reqs) == 1:
                continue
            timers = {r.split("_")[0] for r in reqs}
            only_timer = (len(timers) == 1 and all(r.startswith("TIM") for r in reqs)
                          and sum(r.endswith("_UP") for r in reqs) == 1)
            check(only_timer, "R2 DMA%d-%d multi-occupancy %s is same-timer "
                  "UP/CH bidir multiplexing" % (st[0], st[1], reqs))
        for req in CRITICAL:
            if req in streams:
                others = [r for r in by_stream[streams[req]]
                          if r.split("_")[0] != req.split("_")[0]]
                check(not others, "R2 %s not shared with another peripheral" % req)

        m = re.search(r"#define SHARED_DMA_MASK\s+(.+)", hdr)
        check(m is not None and m.group(1).strip() == "0",
              "R3 SHARED_DMA_MASK == 0 (got %s)" % (m.group(1).strip() if m else None))

        check("#define HAL_USE_SDC TRUE" in hdr, "R4 HAL_USE_SDC enabled")
        check("#define STM32_SDC_USE_SDMMC1 TRUE" in hdr, "R4 SDMMC1 selected")
        check("#define HAL_OS_FATFS_IO 1" in hdr, "R4 FATFS IO enabled")
        check(not any(r.startswith("SDMMC") or r.startswith("SDIO") for r in streams),
              "R4 SDMMC1 uses IDMA (no DMA1/DMA2 stream)")

        used = {st for st in by_stream if st[0] in (1, 2)}
        check(len(used) <= 16, "R5 %d/16 DMA1+DMA2 streams used, %d spare"
              % (len(used), 16 - len(used)))

        # UART DMA config sanity: critical UART must have RX and TX DMA enabled
        for d in ("RX", "TX"):
            check(re.search(r"#define STM32_USART1_%s_DMA_CONFIG true" % d, hdr) is not None,
                  "R1 USART1 %s DMA enabled in UART driver config" % d)

    finally:
        if args.keep:
            print("\noutput kept in %s" % tmp)
        else:
            shutil.rmtree(tmp, ignore_errors=True)

    print("\nRESULT: %s (%d failure%s)" % ("PASS" if not fails else "FAIL",
                                         len(fails), "" if len(fails) == 1 else "s"))
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
