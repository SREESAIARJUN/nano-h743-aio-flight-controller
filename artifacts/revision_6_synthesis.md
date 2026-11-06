# Rev 6.0/150 Engineering Brief: Job #0

**Platform:** STM32H743 FC + 4-in-1 ESC AIO, Radxa CM4 carrier
**Scope:** (1) ArduPilot DMA allocation, (2) bidirectional DShot and RPM harmonic notch configuration, (3) 6-layer stackup and high-current/controlled-impedance routing

> **Assumptions to confirm before release.** The baseline is 6S, 4 × 45 A continuous ESC (≈60 A burst/phase), 5–7" class motors (14-pole, ~1750 KV), ICM-42688-P dual IMU, and a CM4-compatible SoM using USB2, GbE, MIPI-CSI and optional PCIe. Flag any deviation, because several numbers below scale with it.

---

## 1. ArduPilot DMA Allocation (STM32H743)

### 1.1 Resource constraints

| Controller | Streams | Notes |
|---|---|---|
| DMA1 | 8 | Any request on any stream via DMAMUX1 |
| DMA2 | 8 | Any request on any stream via DMAMUX1 |
| BDMA | 8 | D3 domain only (SPI6, I2C4, LPUART1, ADC3). ArduPilot support is limited, so keep critical paths off it. |
| SDMMC1 | IDMA | Has its own internal DMA, so it consumes no stream. Buffers must live in AXI SRAM, which ArduPilot handles. |
| MDMA | — | Not used by ArduPilot. |

On H7, `dma_resolver.py` assigns DMAMUX streams automatically. You cannot hard-pin a stream number in hwdef. You can only control **priority** and **sharing**, then verify the generated `hwdef.h`.

### 1.2 Peripheral map and intended allocation (16 streams)

| # | Request | Function | Policy |
|---|---|---|---|
| 1 | TIM1_UP | DShot burst output (M1–M4) | NOSHARE, top priority |
| 2 | TIM1_CH1 | BDShot capture, pair M1/M2 | Priority |
| 3 | TIM1_CH3 | BDShot capture, pair M3/M4 | Priority |
| 4–5 | SPI1_RX/TX | IMU0 ICM-42688-P (8 kHz FIFO) | NOSHARE |
| 6–7 | SPI4_RX/TX | IMU1 ICM-42688-P | NOSHARE |
| 8–9 | USART1_RX/TX | CM4 MAVLink link (1.5–2 Mbaud) | NOSHARE |
| 10 | USART3_RX | CRSF/ELRS RC (420k) | Dedicated |
| 11 | USART2_RX | GPS (230400) | Dedicated |
| 12 | ADC1 | VBAT / current shunt / RSSI | Dedicated |
| 13–14 | SPI2_RX/TX | MAX7456 OSD (or DisplayPort on UART, see below) | Shareable |
| 15–16 | SPI3_RX/TX | Baro DPS310 + dataflash, if no SD | Shareable |
| — | USART2/3_TX, UART4/7/8 | Low-rate | No DMA (`NODMA`) |

### 1.3 Allocation rules

- **CM4 link.** Run USART1 at ≥1.5 Mbaud with RTS/CTS (`BRD_SER1_RTSCTS=1`). Better still, use USB-CDC to the CM4, which frees two streams entirely.
- **Avoid MAX7456 if possible.** Using an HD DisplayPort OSD on a UART frees the two SPI2 streams.
- **Use SD for logging.** SDMMC logging on IDMA is preferable to SPI flash, since it removes SPI3 DMA pressure.

### 1.4 hwdef.dat excerpt

```
# Motors – 4-in-1 on TIM1, bidir pairs (BIDIR on lower channel of each pair)
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50) BIDIR
PE11 TIM1_CH2 TIM1 PWM(2) GPIO(51)
PE13 TIM1_CH3 TIM1 PWM(3) GPIO(52) BIDIR
PE14 TIM1_CH4 TIM1 PWM(4) GPIO(53)

# DMA policy
DMA_PRIORITY TIM1* SPI1* SPI4* USART1*
DMA_NOSHARE  TIM1_UP SPI1* SPI4* USART1*

# Low-rate UARTs without DMA
PA2  USART2_TX USART2 NODMA
PD8  USART3_TX USART3 NODMA
```

### 1.5 Verification

1. Check the build output `hwdef.h` for `STM32_DMA_STREAM` assignments and confirm no `SHARED` flag on TIM1_UP, SPI1, SPI4 or USART1.
2. Read `@SYS/dma.txt` over MAVFTP in flight. Target contention below 1% on IMU streams.
3. Read `@SYS/timers.txt` to confirm the bidir capture is active.

---

## 2. Bidirectional DShot and RPM Harmonic Notch

### 2.1 DShot1200 bidir feasibility

| Parameter | DShot600 | DShot1200 |
|---|---|---|
| Bit time | 1.67 µs | 0.83 µs |
| Command frame (16 bit) | 26.7 µs | 13.3 µs |
| eRPM reply (21-bit GCR, 5/4 rate) | ~28 µs | ~14 µs |
| Turnaround guard | 30 µs | 30 µs (fixed) |
| ESC FW support (AM32 / Bluejay / BLHeli32) | Yes | Limited / unreliable |

**Recommendation:** ship `MOT_PWM_TYPE=6` (DShot600) as the production default. Qualify `MOT_PWM_TYPE=7` (DShot1200) only if the ESC firmware explicitly supports bidir at 1200.

DShot1200 also leaves very little margin for signal integrity. Motor lines must be under 30 mm, have a 33 Ω series termination at the MCU, and have no capacitive load above ~20 pF. The faster rate gains essentially nothing at a 1–2 kHz loop.

### 2.2 Core parameters

```
MOT_PWM_TYPE      6        # 7 = DShot1200 if qualified
SERVO_BLH_BDMASK  15       # bidir on outputs 1–4
SERVO_BLH_POLES   14
SERVO_DSHOT_RATE  0        # 1x loop-synced (use 2 at 400 Hz loops)
SCHED_LOOP_RATE   1000     # H743 handles 1 kHz comfortably; 2000 possible with FSTRATE
INS_GYRO_RATE     1        # 2 kHz backend; 2 = 4 kHz if CPU allows
INS_FAST_SAMPLE   3        # both IMUs
```

### 2.3 Harmonic frequency plan

Mechanical rotor frequency is derived from eRPM:

**f₁ = eRPM / (pole_pairs × 60)**, with pole_pairs = 7.

For 1750 KV on 6S (25.2 V full, ~85% loaded efficiency):

| Condition | RPM | f₁ (H