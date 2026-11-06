// AM32 target block - Nano H743 AIO integrated 4-in-1 ESC (Revision 9.0)
// =======================================================================
// Drop-in block for am32-firmware/AM32 Inc/targets.h (verified against v2.21,
// commit 3d8cd50). tools/am32_build_target.sh splices it in, builds it with
// AM32's own toolchain and checks the preprocessed configuration.
// Every number below is derived and cross-checked by
// am32_esc/gate_drive_design.py (checks H1-H7); do not edit by hand.
//
// Hardware per ESC channel (x4, identical):
//   MCU          AT32F421K8U7 (QFN-32, 120 MHz)  -> AM32 MCU_AT421
//   Gate driver  Fortior FD6288Q (QFN-24), VCC = VBAT via 10 R / 10 uF + 18 V Zener
//   Power stage  6x Toshiba TPHR8504PL (40 V, 0.85 mOhm @ 10 V)
//   Gate network 0 R series footprint (slew re-tune at bring-up) + 2.2 nF C0G
//                Cgs at each FET; 10 k (LS) / 100 k (HS) gate-source pull-down.
//                Slew is set by the FD6288Q output current (1.1-1.9 A source,
//                1.3-2.3 A sink): 2.85 V/ns worst case at 4S HV / 35 A.
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
//   43 -> 358.3 ns MCU dead time, in series with the FD6288Q fixed internal
//   interlock dead time of 100/200/300 ns (min/typ/max). Required: 353.9 ns,
//   set by the LS hard turn-off (negative phase current) at 4S HV / 10 A:
//   slow driver (IOL 1.3 A min, scaled by VCC), rg_int max, Miller plateau at
//   125 C / Vth min, outgoing declared off at Vth,min(125 C) = 0.55 V, against
//   the fastest incoming gate, plus 30 ns HO/LO matching and a 25 ns guard.
//   Body-diode conduction per edge: 457 ns typ / 755 ns max.
//   DEAD_TIME also seeds gate_drive_offset (43/2000 of ARR, sine start-up
//   only). minimum_duty_cycle comes from EEPROM byte 6 (pinned to 4 -> 842 ns
//   so no commanded pulse is shorter than the 688 ns worst-case dead time).
//
// Settings AM32 reads from EEPROM, not from this file (programmed per
// am32_esc/eeprom_settings.json via BLHeli passthrough):
//   pwm_frequency 24 kHz, comp_pwm 1, variable_pwm 0, minimum_duty_cycle 4,
//   driving_brake_strength 10 (dead-time override off; 1..9 OR into DTC and
//   give 0.93-2.12 us), low_voltage_cut_off 0 (AM32 cutoff latches a motor
//   off; the FC failsafe BATT_CRT_VOLT 6.4 guards the FD6288Q UVLO instead),
//   limits.temperature 115 C (uncalibrated MCU die sensor, last-resort step
//   derate), limits.current 0 (no per-channel current sense).

#ifdef NANO_H743_AIO_F421
#define FIRMWARE_NAME "NanoH743AIO"
#define FILE_NAME "NANO_H743_AIO_F421"
#define DEAD_TIME 43
#define HARDWARE_GROUP_AT_B
#define HARDWARE_GROUP_AT_045
#define TARGET_VOLTAGE_DIVIDER 110
#define CURRENT_OFFSET 0
#define MILLIVOLT_PER_AMP 20
#endif
