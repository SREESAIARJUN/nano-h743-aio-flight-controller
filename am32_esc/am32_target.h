// AM32 target block - Nano H743 AIO integrated 4-in-1 ESC (Revision 9.0)
// =======================================================================
// Drop-in block for am32-firmware/AM32 Inc/targets.h (verified against v2.21,
// commit 3d8cd50). tools/am32_build_target.sh splices it in, builds it with
// AM32's own toolchain and checks the preprocessed configuration.
//
// Hardware per ESC channel (x4, identical):
//   MCU          AT32F421K8U7 (QFN-32, 120 MHz)  -> AM32 MCU_AT421
//   Gate driver  Fortior FD6288Q (QFN-24), VCC = VBAT via 10 R / 10 uF + 18 V Zener
//   Power stage  6x Toshiba TPHR8504PL (40 V, 0.85 mOhm @ 10 V)
//   Gate network 15 R (0402) per gate, Schottky (RB521S30) across it for fast
//                turn-off, 10 k gate-source pull-down
//
// PWM / gate-driver interface (HARDWARE_GROUP_AT_B):
//   FD6288Q HIN1 (A) <- PA10 TMR1_CH3     LIN1 (A) <- PB1 TMR1_CH3N
//   FD6288Q HIN2 (B) <- PA9  TMR1_CH2     LIN2 (B) <- PB0 TMR1_CH2N
//   FD6288Q HIN3 (C) <- PA8  TMR1_CH1     LIN3 (C) <- PA7 TMR1_CH1N
//   FD6288Q HIN and LIN are both ACTIVE-HIGH ("output in phase with input",
//   datasheet V1.6 section 1.3 / Table 1-2), so USE_INVERTED_LOW/HIGH,
//   USE_OPEN_DRAIN_* and PWM_ENABLE_BRIDGE must NOT be defined. The 200 k
//   input pull-downs hold all six FETs off while the AT32 is in reset.
//   Signal input: PB4 TMR3_CH1 <- H743 TIM1_CHx (bidirectional DShot, see
//   ardupilot/hwdef.dat). Back-EMF comparator (HARDWARE_GROUP_AT_045):
//   A = PA0, B = PA4, C = PA5, virtual neutral on PA1.
//
// Dead time
//   DEAD_TIME is the raw TMR1_BRK.DTC (STM32 BDTR.DTG) code, NOT nanoseconds.
//   tDTS = 1/120 MHz = 8.333 ns; codes < 128 are linear.
//   34 -> 283.3 ns MCU dead time, in series with the FD6288Q fixed internal
//   interlock dead time of 100/200/300 ns (min/typ/max). Derived in
//   am32_esc/gate_drive_design.py: the slowest outgoing FET (4S HV, 125 C,
//   Vth,min 0.55 V, max driver Rsink, max toff propagation) must be below
//   threshold before the fastest incoming gate reaches it, plus the 30 ns
//   HO/LO delay matching and a 25 ns guard -> 283.0 ns required.
//   Body-diode conduction per edge: 483 ns typ / 613 ns max.
//   DEAD_TIME also seeds AM32's minimum_duty_cycle and gate_drive_offset
//   (Src/main.c:419,536): 34 of 5000 ARR counts = 0.68 % at 24 kHz.
//
// Settings AM32 reads from EEPROM, not from this file (programmed by
// am32_esc/eeprom_settings.json via the AM32 configurator / passthrough):
//   pwm_frequency 24 kHz, driving_brake_strength 10 (dead-time override
//   off; 1..9 OR into DTC and give 0.88-2.03 us), limits.temperature 115 C
//   (MCU die sensor), limits.current 0 (no per-channel current sense).

#ifdef NANO_H743_AIO_F421
#define FIRMWARE_NAME "NanoH743AIO"
#define FILE_NAME "NANO_H743_AIO_F421"
#define DEAD_TIME 34
#define HARDWARE_GROUP_AT_B
#define HARDWARE_GROUP_AT_045
#define TARGET_VOLTAGE_DIVIDER 110
#define CURRENT_OFFSET 0
#define MILLIVOLT_PER_AMP 20
#endif
