# 3-Axis Gimbal on ESP32-S3: FOC Mathematical Foundations (Cycle 1)

**Document:** GMB-ARCH-001, Rev A
**Scope:** Derivation of the Clarke and Park transforms, the PMSM model in the dq frame, SVPWM duty-cycle equations, and field weakening (FW). Each result is mapped to an ESP32-S3 implementation.

---

## 0. System Context and Conventions

### 0.1 Reference hardware (Cycle 1 baseline)

| Item | Choice | Rationale |
|---|---|---|
| MCU | ESP32-S3, dual Xtensa LX7 @ 240 MHz, single-precision FPU | |
| Motors | 3× gimbal PMSM (e.g., GBM2804-class), 7 pole pairs | Low Kv, high R, smooth cogging |
| Driver | 3× DRV8313 (3-PWM + EN, internal dead time) | 9 PWM lines total, which fits the MCPWM budget (§4.8) |
| Encoders | 3× 14-bit magnetic (AS5048A / MT6701), SPI/SSI | Absolute angle, no alignment search after first calibration |
| Bus | 3S Li-ion, $V_{dc}$ = 10.5–12.6 V | |
| Current sensing | None in Cycle 1 (voltage-mode FOC); hooks reserved | The S3 SAR ADC is marginal, and R-dominated motors permit model-based current estimation |

### 0.2 Representative motor parameters (used in all numeric examples)

| Symbol | Meaning | Value |
|---|---|---|
| $R_s$ | phase resistance | 10 Ω |
| $L_s$ | phase synchronous inductance ($L_d