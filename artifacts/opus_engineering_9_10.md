# ArduPilot hwdef for an STM32H743VIT6 Nano AIO Flight Controller

**Before flashing, note two points:**

* **The pinout is a reference design, not a real board.** "STM32H743VIT6 nano AIO" doesn't identify a specific commercial board, and I don't know what "Cycle 9" refers to (I've used it only as a revision label). The pins below are electrically valid for the LQFP100 package, with alternate functions checked against the H743 datasheet and no pin conflicts. You must still verify every line against your board's schematic. A wrong CS, timer or oscillator line can leave the board unbootable until it is recovered via DFU.
* **ArduPilot doesn't use hand-written DMA streams on H7.** The H7 has a DMAMUX, and `Tools/ardupilotwaf/chibios_hwdef.py` runs a DMA resolver that allocates streams automatically. Hard-coding stream numbers, as on F4 boards, is ignored or harmful. You control DMA through `DMA_PRIORITY`, `DMA_NOSHARE` and `NODMA`, which this file uses. The resolved map is written to `build/<board>/hwdef.h` (`STM32_*_DMA_STREAM` defines) for review.

---

## Directory layout

```
libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
├── hwdef.dat
├── hwdef-bl.dat
└── defaults.parm
```

---

## `hwdef.dat`

```text
# ===================================================================
#  NanoH743AIO  -  STM32H743VIT6 nano all-in-one flight controller
#  Revision: Cycle 9
#  Reference pinout - VERIFY AGAINST SCHEMATIC
# ===================================================================

# ---------------- MCU / clocks / board ID ----------------
MCU STM32H7xx STM32H743xx

# Must be registered in Tools/AP_Bootloader/board_types.txt
# (placeholder ID - request a real one before distribution)
APJ_BOARD_ID 5999

# Crystal frequency (8 MHz typical; some boards use 16 or 24 MHz)
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048

# Bootloader occupies the first 128k sector
FLASH_RESERVE_START_KB 128

# Parameter storage in internal flash
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 16384

# 4 sectors per boot; H7 sectors are 128k
define HAL_FLASH_PROTECTION 0

# ---------------- USB ----------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
USB_STRING_MANUFACTURER "ArduPilot"
USB_STRING_PRODUCT "NanoH743AIO"

# ---------------- SWD ----------------
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------- Serial ports ----------------
# SERIAL0=USB, then SERIAL1..SERIAL7
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 UART7 UART8 USART6

# USART1 - Telem / MAVLink (SERIAL1)
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - GPS (SERIAL2)
PD5  USART2_TX USART2
PD6  USART2_RX USART2

# USART3 - RC input (SERIAL3), CRSF/ELRS
PD8  USART3_TX USART3
PD9  USART3_RX USART3

# UART4 - DJI / MSP DisplayPort (SERIAL4)
PA0  UART4_TX UART4
PA1  UART4_RX UART4

# UART7 - ESC telemetry (SERIAL5), RX only meaningful
PE8  UART7_TX UART7
PE7  UART7_RX UART7

# UART8 - spare / VTX SmartAudio (SERIAL6)
PE1  UART8_TX UART8 NODMA
PE0  UART8_RX UART8 NODMA

# USART6 - spare (SERIAL7)
PC6  USART6_TX USART6 NODMA
PC7  USART6_RX USART6 NODMA

# ---------------- SPI buses ----------------
# SPI1 - primary IMU
PA5  SPI1_SCK  SPI1
PA6  SPI1_MISO SPI1
PA7  SPI1_MOSI SPI1

# SPI4 - secondary IMU
PE2  SPI4_SCK  SPI4
PE5  SPI4_MISO SPI4
PE6  SPI4_MOSI SPI4

# SPI2 - MAX7456 analog OSD
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - dataflash
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# Chip selects (all idle high)
PA4  IMU1_CS   CS
PE11 IMU2_CS   CS
PB12 OSD1_CS   CS
PA15 FLASH1_CS CS

# IMU data-ready interrupts (optional, used by some drivers)
PB2  IMU1_DRDY INPUT
PE15 IMU2_DRDY INPUT

# ---------------- SPI device table ----------------
#        name       bus  devid  cs         mode   lowspd   highspd
SPIDEV imu1       SPI1 DEVID1 IMU1_CS   MODE3  2*MHZ    16*MHZ
SPIDEV imu2       SPI4 DEVID1 IMU2_CS   MODE3  2*MHZ    16*MHZ
SPIDEV osd        SPI2 DEVID4 OSD1_CS   MODE0  10*MHZ   10*MHZ
SPIDEV dataflash  SPI3 DEVID1 FLASH1_CS MODE3  104*MHZ  104*MHZ

# ---------------- IMUs ----------------
# Probe-list: whichever chip is populated will be detected
IMU Invensensev3 SPI:imu1 ROTATION_YAW_180
IMU Invensense   SPI:imu1 ROTATION_YAW_180
IMU BMI270       SPI:imu1 ROTATION_YAW_180
IMU Invensensev3 SPI:imu2 ROTATION_YAW_270
IMU BMI270       SPI:imu2 ROTATION_YAW_270

define HAL_DEFAULT_INS_FAST_SAMPLE 3

# ---------------- I2C ----------------
I2C_ORDER I2C1 I2C2

# I2C1 - internal barometer
PB8  I2C1_SCL I2C1
PB9  I2C1_SDA I2C1

# I2C2 - external connector (compass, airspeed, etc.)
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

# ---------------- Barometer ----------------
BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76
BARO SPL06  I2C:0:0x76

# ---------------- Compass ----------------
# No onboard compass; probe external buses
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 1
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ---------------- Motor / servo outputs ----------------
# TIM3 group (outputs 1-4)
PB0  TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1  TIM3_CH4 TIM3 PWM(2) GPIO(51)
PB4  TIM3_CH1 TIM3 PWM(3) GPIO(52) BIDIR
PB5  TIM3_CH2 TIM3 PWM(4) GPIO(53)

# TIM4 group (outputs 5-8)
PD12 TIM4_CH1 TIM4 PWM(5) GPIO(54) BIDIR
PD13 TIM4_CH2 TIM4 PWM(6) GPIO(55)
PD14 TIM4_CH3 TIM4 PWM(7) GPIO(56) BIDIR
PD15 TIM4_CH4 TIM4 PWM(8) GPIO(57)

# LED strip / NeoPixel (output 9)
PE9  TIM1_CH1 TIM1 PWM(9) GPIO(58)

# ---------------- ADC ----------------
PC0  BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1  BATT_CURRENT_SENS ADC1 SCALE(1)
PC5  RSSI_IN           ADC1

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 8

# ---------------- DroneCAN (FDCAN1) ----------------
PD0  CAN1_RX CAN1
PD1  CAN1_TX CAN1
# Transceiver standby/silent pin (LOW = active)
PD3  GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

define HAL_NUM_CAN_IFACES 1
define HAL_CAN_POOL_SIZE 6000

# ---------------- LEDs / buzzer / misc GPIO ----------------
PC13 LED0 OUTPUT LOW GPIO(90)
PC14 LED1 OUTPUT LOW GPIO(91)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN