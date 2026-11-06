#!/usr/bin/env bash
# Nano H743 AIO - build and verify the AM32 ESC target (Revision 9.0)
#
#   tools/am32_build_target.sh <AM32 checkout> [work dir]
#
# 1. Copies the AM32 tree to a scratch dir (the checkout is never modified).
# 2. Splices am32_esc/am32_target.h into Inc/targets.h just before AM32's
#    "Missing defines for target" guard, plus a SITL twin with the same DEAD_TIME.
# 3. Installs AM32's pinned toolchain (make arm_sdk_install) if absent and
#    builds NANO_H743_AIO_F421 with -Werror on our block's warnings.
# 4. Preprocesses targets.h for the target with the ARM compiler and asserts
#    the resolved configuration (MCU, clocks, PWM/LIN pins, comparator, input,
#    DEAD_TIME, divider, no inversion macros).
# 5. Runs the SITL twin and checks that the DEAD_TIME code reaches the
#    simulated TIM1 BDTR unchanged.
# Exit status is non-zero on any failure.
set -euo pipefail

AM32_SRC=${1:?usage: $0 <AM32 checkout> [work dir]}
WORK=${2:-$(mktemp -d /tmp/am32_nano.XXXXXX)}
REPO=$(cd "$(dirname "$0")/.." && pwd)
BLOCK="$REPO/am32_esc/am32_target.h"
TARGET=NANO_H743_AIO_F421
SITL=NANO_H743_AIO_SITL

fail() { echo "FAIL: $*" >&2; exit 1; }

echo "== AM32 source: $(git -C "$AM32_SRC" rev-parse --short HEAD 2>/dev/null || echo unknown) -> $WORK"
rm -rf "$WORK/am32"
mkdir -p "$WORK"
cp -a "$AM32_SRC" "$WORK/am32"
cd "$WORK/am32"

DT=$(sed -n 's/^#define DEAD_TIME \([0-9]*\).*/\1/p' "$BLOCK")
[ -n "$DT" ] || fail "DEAD_TIME not found in $BLOCK"

# --- splice the target block (and a SITL twin) into Inc/targets.h
python3 - "$BLOCK" "$DT" <<'EOF'
import sys
block, dt = open(sys.argv[1]).read(), sys.argv[2]
sitl = f'''
#ifdef NANO_H743_AIO_SITL
#define FIRMWARE_NAME "NanoH743SITL"
#define FILE_NAME "NANO_H743_AIO_SITL"
#define DEAD_TIME {dt}
#define HARDWARE_GROUP_SITL_A
#endif
'''
p = "Inc/targets.h"
s = open(p).read()
anchor = "#ifndef FIRMWARE_NAME\n/* if you get this"
if anchor not in s:
    sys.exit("anchor not found in Inc/targets.h - AM32 layout changed")
if "NANO_H743_AIO_F421" in s:
    sys.exit("target already present in Inc/targets.h")
s = s.replace(anchor, "// ---- Nano H743 AIO (spliced by tools/am32_build_target.sh)\n" + block + sitl + "\n" + anchor, 1)
open(p, "w").write(s)
EOF
grep -q "HARDWARE_GROUP_SITL_A" Inc/targets.h || fail "SITL hardware group missing in this AM32 version"

# --- toolchain
CC=tools/linux/xpack-arm-none-eabi-gcc-10.3.1-2.3/bin/arm-none-eabi-gcc
if [ ! -x "$CC" ]; then
  if [ -x "$AM32_SRC/$CC" ]; then
    mkdir -p tools/linux && cp -a "$AM32_SRC/tools/linux/." tools/linux/
  else
    make arm_sdk_install >/dev/null
  fi
fi
[ -x "$CC" ] || fail "toolchain not installed"

make targets 2>/dev/null | tr ' ' '\n' | grep -qx "$TARGET" || fail "$TARGET not discovered by make targets"

# --- build
echo "== make $TARGET"
make "$TARGET" > "$WORK/build_f421.log" 2>&1 || { tail -40 "$WORK/build_f421.log"; fail "build failed"; }
BIN=$(ls obj/AM32_${TARGET}_*.bin | head -1)
[ -s "$BIN" ] || fail "no binary"
grep -E "(FLASH|RAM):" "$WORK/build_f421.log" | sed 's/^/   /' || true
if grep -n "warning" "$WORK/build_f421.log" | grep -v "overriding recipe\|ignoring old recipe" | grep -q "targets.h"; then
  grep -n "warning" "$WORK/build_f421.log" | grep "targets.h"; fail "warnings in targets.h"
fi
strings "$BIN" | grep -qx "$TARGET" || fail "FILE_NAME not embedded in binary (.file_name section)"
echo "   $BIN ($(stat -c %s "$BIN") bytes)"

# --- resolved configuration
echo "== resolved configuration (arm-none-eabi-gcc -E -dM)"
INC=$(sed -n 's/^MCU_INC.*:= *//p' f421makefile.mk 2>/dev/null | head -1)
"$CC" -E -dM -DUSE_MAKE -D$TARGET -DAT32F421K8U7 -IInc $(find Mcu/f421 -type d -name Inc -printf '-I%p ') \
      $(find Mcu/f421 -type d -path '*inc*' -printf '-I%p ') -include Inc/targets.h -x c /dev/null 2>/dev/null > "$WORK/macros.txt" \
  || "$CC" -E -dM -DUSE_MAKE -D$TARGET -IInc -include Inc/targets.h -x c /dev/null > "$WORK/macros.txt"
python3 - "$WORK/macros.txt" "$DT" <<'EOF'
import re, sys
m = {}
for line in open(sys.argv[1]):
    r = re.match(r"#define (\w+) ?(.*)", line)
    if r:
        m[r.group(1)] = r.group(2).strip()
dt = sys.argv[2]
want = {
    "MCU_AT421": "", "ARTERY": "", "CPU_FREQUENCY_MHZ": "120", "DEAD_TIME": dt,
    "TARGET_VOLTAGE_DIVIDER": "110", "CURRENT_OFFSET": "0", "MILLIVOLT_PER_AMP": "20",
    "PHASE_A_GPIO_HIGH": "GPIO_PINS_10", "PHASE_A_GPIO_PORT_HIGH": "GPIOA",
    "PHASE_A_GPIO_LOW": "GPIO_PINS_1", "PHASE_A_GPIO_PORT_LOW": "GPIOB",
    "PHASE_B_GPIO_HIGH": "GPIO_PINS_9", "PHASE_B_GPIO_PORT_HIGH": "GPIOA",
    "PHASE_B_GPIO_LOW": "GPIO_PINS_0", "PHASE_B_GPIO_PORT_LOW": "GPIOB",
    "PHASE_C_GPIO_HIGH": "GPIO_PINS_8", "PHASE_C_GPIO_PORT_HIGH": "GPIOA",
    "PHASE_C_GPIO_LOW": "GPIO_PINS_7", "PHASE_C_GPIO_PORT_LOW": "GPIOA",
    "INPUT_PIN": "GPIO_PINS_4", "INPUT_PIN_PORT": "GPIOB", "IC_TIMER_REGISTER": "TMR3",
    "PHASE_A_COMP": "0x400000E5", "PHASE_B_COMP": "0x400000C5", "PHASE_C_COMP": "0x400000D5",
    "VOLTAGE_ADC_PIN": "GPIO_PINS_3", "CURRENT_ADC_PIN": "GPIO_PINS_6",
}
bad = []
for k, v in want.items():
    got = m.get(k)
    ok = got is not None and (got.split("//")[0].strip() == v or (v and got.startswith(v)))
    print(f"   {'ok ' if ok else 'BAD'} {k:24s} {got!s:16s} (want {v or 'defined'})")
    if not ok:
        bad.append(k)
for k in ("USE_INVERTED_LOW", "USE_INVERTED_HIGH", "PWM_ENABLE_BRIDGE", "USE_OPEN_DRAIN_LOW",
          "USE_OPEN_DRAIN_HIGH", "OPEN_DRAIN_PWM", "USE_SERIAL_TELEMETRY"):
    if k in m:
        print(f"   BAD {k} must not be defined"); bad.append(k)
    else:
        print(f"   ok  {k:24s} undefined")
arr = int(m.get("CPU_FREQUENCY_MHZ", "0")) * 1000000 // 24000 - 1
print(f"   TIM1_AUTORELOAD @24 kHz = {arr}; DEAD_TIME {dt} = {int(dt)/120*1000:.1f} ns")
sys.exit(1 if bad else 0)
EOF

# --- SITL twin: DEAD_TIME code must reach BDTR unchanged
echo "== make $SITL"
if make "$SITL" > "$WORK/build_sitl.log" 2>&1; then
  ELF=$(ls obj/AM32_${SITL}_*.elf | head -1)
  OUT=$(cd "$WORK" && timeout -s INT 4 "$WORK/am32/$ELF" 2>&1 || true)
  LINE=$(echo "$OUT" | grep -m1 "dead time" || true)
  echo "   $LINE"
  HEX=$(printf "0x%x" "$DT")
  echo "$LINE" | grep -q "BDTR $HEX" || fail "SITL BDTR does not carry DEAD_TIME $DT ($HEX)"
else
  tail -20 "$WORK/build_sitl.log"; fail "SITL build failed"
fi
echo "PASS: $TARGET built and verified (work dir $WORK)"
