#ifndef AM32_TARGET_NANO_H743_AIO_H
#define AM32_TARGET_NANO_H743_AIO_H

// AM32 Multi-Rotor ESC Firmware Definition for Nano H743 AIO (Revision 8.0)
// Gate Driver: FD6288Q (Integrated 3-Phase Half-Bridge Driver)
// Power MOSFETs: 24x Toshiba TPHR8504PL, 40 V, SOP Advance (2-5Q1S)
//   RDS(on) max 0.85 mOhm @ VGS=10 V, 1.40 mOhm @ VGS=4.5 V (ID=50 A, 25 C)
//   Qg 103 nC @ 10 V, Qsw 23 nC, Rth(ch-c) 0.88 K/W max
//   (Rev <=7 quoted "1.4 mOhm" -- that is the 4.5 V figure, not the operating value
//    at 3S-4S gate drive.)

#define DEAD_TIME_NS 60

// Rev 8.0: 48 -> 24 kHz.  analysis/thermal_power_analysis.py (geometry read back
// from the 6-layer board, 4S, 20 A per motor, h = 80 W/m2K): 48 kHz adds +2.6 W to
// the board (+1.5 W HS switching -- t_off ~75 ns for Qsw 23 nC through the FD6288Q
// sink + 4.7 ohm Rg -- plus Coss/Qrr, gate drive and the RDS(on) rise).  The board
// runs 94.6 C instead of 86.7 C, and the rated 28 A / 10 s burst ends at 118.9 C,
// past TEMPERATURE_CUTOFF_C.  At 24 kHz the same burst ends at 109.2 C (Tj 114 C).
#define PWM_FREQ_KHZ 24

#define SENSE_SHUNT_MILLIOHM 0.5f
// Rev 8.0 thermal ratings per motor (all four loaded): 20 A continuous / 28 A for
// 10 s in typical prop wash; 17 A / 22 A in the pessimistic case.  35 A is allowed
// for only ~4 s from 20 A.  The 38 A limit stays as a fast short-circuit/stall trip
// and is NOT a thermal rating.
#define OVER_CURRENT_LIMIT_AMPS 38.0f
#define TEMPERATURE_CUTOFF_C 115.0f

#define BIDIRECTIONAL_DSHOT 1
#define DSHOT_TELEMETRY 1

#endif
