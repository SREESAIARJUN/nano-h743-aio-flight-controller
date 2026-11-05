# 3-Axis Gimbal on ESP32-S3: FOC Mathematical Foundations (Design Cycle 2)

**Scope:** Clarke/Park transforms, the dq machine model, SVPWM duty-cycle synthesis, current-loop design, and field weakening. Each section ends with the implementation consequence for an ESP32-S3 driving three gimbal PMSMs.

---

## 0. Reference Plant and Notation

Gimbal motors are low-speed, high-pole-count, high-resistance surface-mount PMSMs (SPMSM). The numbers below are a representative 2804/4108-class motor. They anchor the worked examples and should be replaced with measured values from the Cycle 2 characterization bench.

| Symbol | Meaning | Value |
|---|---|---|
| $p$ | pole pairs | 7 |
| $R_s$ | phase resistance | 5.0