#ifndef AM32_TARGET_NANO_H743_AIO_H
#define AM32_TARGET_NANO_H743_AIO_H

// AM32 Multi-Rotor ESC Firmware Definition for Nano H743 AIO
// Gate Driver: FD6288Q (Integrated 3-Phase Half-Bridge Driver)
// Power MOSFETs: 24x TPHR8504PL 40V, RDS(on) = 1.4 mOhm

#define DEAD_TIME_NS 60
#define PWM_FREQ_KHZ 48
#define SENSE_SHUNT_MILLIOHM 0.5f
#define OVER_CURRENT_LIMIT_AMPS 38.0f
#define TEMPERATURE_CUTOFF_C 115.0f

#define BIDIRECTIONAL_DSHOT 1
#define DSHOT_TELEMETRY 1

#endif
