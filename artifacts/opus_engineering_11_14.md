# 3-Axis Gimbal on ESP32-S3, Cycle 11: FOC Mathematical Foundations

**Document:** GMB-S3-ARCH-011 · **Scope:** Clarke/Park transforms, SVPWM, field weakening, and how they map onto ESP32-S3 hardware

---

## 0. System Context and Notation

| Symbol | Meaning | Gimbal reference value (2804-class, 100T) |
|---|---|---|
| $p$ | pole pairs | 7 (12N14P) |
| $R_s$ | phase resistance | ≈ 10 Ω |
| $L_d, L_q$ | d/q inductance | ≈ 2.5 mH (SPM, so $L_d \approx L_q = L$) |
| $\psi_m$ | PM flux linkage (peak, per phase) | ≈ 3.8 mWb |
| $V_{dc}$ | bus voltage | 12 V (3S Li-ion nominal ≈