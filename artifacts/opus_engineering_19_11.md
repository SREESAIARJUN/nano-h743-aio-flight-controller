# ArduPilot hwdef.dat for a Generic STM32H743VIT6 Nano AIO Flight Controller

**Important:** This is a **reference template** with a self-consistent, conflict-checked pin map. It is not the definition for any specific commercial board. Every pin, sensor, and rotation must be checked against your actual schematic before flashing. Wrong pins can drive outputs into shorts, and wrong IMU rotation will cause a crash on takeoff. I don't know what "Cycle 19" refers to (possibly your internal revision counter), so it appears only as a comment.

## Notes on DMA

On STM32H7, ArduPilot's `chibios_hwdef.py` allocates DMA streams automatically through the **DMAMUX**. Unlike F4 boards, you do not hand-assign stream/channel numbers. You control allocation with:

- `DMA_PRIORITY`: which peripherals get first pick of streams.
- `DMA_NOSHARE`: peripherals that must never share a stream. Use it for the IMU SPI and DShot timers.
- `NODMA` on a pin line: excludes that peripheral from DMA.

After building, the resolved allocation appears in `build/<board>/hwdef.h` as `STM32_*_DMA_STREAM` defines. Check it there.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```
# hw definition file for generic STM32H743VIT6 Nano AIO flight controller
# Revision: Cycle 19 (template - verify against schematic)
# MCU: STM32H743VIT6 (LQFP100, 2MB flash, 1MB RAM)

# ---------------------------------------------------------------------------
# MCU / clock / board identity
# ---------------------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# 8MHz external crystal
OSCILLATOR_HZ 8000000

# Board ID: request an official ID before distributing firmware.
# Placeholder value - must match hwdef-bl.dat
APJ_BOARD_ID 5999

FLASH_SIZE_KB 2048

# Bootloader occupies first 128k sector on H7
FLASH_RESERVE_START_KB 128

# Use flash for parameter storage (no FRAM on nano AIO)
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 32768

# ---------------------------------------------------------------------------
# Serial port order
# SERIAL0=USB, 1..7 = UARTs below
# ---------------------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USB
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9  VBUS INPUT OPENDRAIN

# USART1 - SERIAL1 (Telem1 / MAVLink)
PB14 USART1_TX USART1
PB15 USART1_RX USART1

# USART2 - SERIAL2 (Telem2 / VTX control)
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - SERIAL3 (GPS1)
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - SERIAL4 (GPS2 / spare)
PB9 UART4_TX UART4
PB8 UART4_RX UART4

# USART6 - SERIAL5 (RC input, CRSF/ELRS default)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 - SERIAL6 (ESC telemetry, RX only)
PE7 UART7_RX UART7
PE8 UART7_TX UART7 NODMA

# UART8 - SERIAL7 (DJI/HD VTX / spare)
PE0 UART8_RX UART8
PE1 UART8_TX UART8

# Default protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_None

# ---------------------------------------------------------------------------
# SWD debug
# ---------------------------------------------------------------------------
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------------------------------------------------------------------
# SPI buses
#   SPI1 -> IMU (dedicated, high-rate)
#   SPI2 -> AT7456E analog OSD
#   SPI3 -> Dataflash (W25N01G / W25Q128)
# ---------------------------------------------------------------------------
# SPI1 - IMU
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1
PA4 IMU1_CS CS
PC4 IMU1_DRDY INPUT

# SPI2 - OSD
PB13 SPI2_SCK  SPI2
PC2  SPI2_MISO SPI2
PC3  SPI2_MOSI SPI2
PB12 OSD_CS CS

# SPI3 - Flash
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3
PD3  FLASH_CS CS

# SPI device table
#        name       bus devid cs        mode       lowspd  highspd
SPIDEV  icm42688   SPI1 DEVID1 IMU1_CS  MODE3       2*MHZ  16*MHZ
SPIDEV  bmi270     SPI1 DEVID1 IMU1_CS  MODE3       1*MHZ  10*MHZ
SPIDEV  osd        SPI2 DEVID4 OSD_CS   MODE0      10*MHZ  10*MHZ
SPIDEV  dataflash  SPI3 DEVID1 FLASH_CS MODE3     104*MHZ 104*MHZ

# ---------------------------------------------------------------------------
# IMU - probe ICM42688 first, BMI270 as alternate population
# Rotation MUST be verified on the physical board
# ---------------------------------------------------------------------------
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
IMU BMI270       SPI:bmi270   ROTATION_YAW_180
define HAL_DEFAULT_INS_FAST_SAMPLE 1

# ---------------------------------------------------------------------------
# I2C - baro onboard, external compass on same bus
# ---------------------------------------------------------------------------
PB6 I2C1_SCL I2C1
PB7 I2C1_SDA I2C1
I2C_ORDER I2C1

# Barometer (DPS310 / SPL06 / BMP280 alternates)
BARO DPS280 I2C:0:0x76
BARO SPL06  I2C:0:0x76
BARO BMP280 I2C:0:0x76

# No onboard compass - probe external on I2C
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_COMPASS_AUTO_ROT_DEFAULT 2
define HAL_I2C_INTERNAL_MASK 0

# ---------------------------------------------------------------------------
# OSD
# ---------------------------------------------------------------------------
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ---------------------------------------------------------------------------
# Logging - SPI NAND/NOR flash
# ---------------------------------------------------------------------------
define HAL_LOGGING_DATAFLASH_ENABLED 1
define HAL_OS_FATFS_IO 0

# ---------------------------------------------------------------------------
# DroneCAN (FDCAN1)
# ---------------------------------------------------------------------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
# Optional transceiver silent/standby pin (pull low = active)
PD2 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW GPIO(70)

define HAL_NUM_CAN_IFACES 1
define HAL_CAN_DEFAULT_NODE_ID 0
define AP_CAN_SLCAN_ENABLED 1

# Enable DroneCAN on CAN1 by default
define HAL_CAN_DRIVER_DEFAULT 1

# ---------------------------------------------------------------------------
# Motor / PWM outputs (DShot-capable, bidir on TIM4/TIM1 groups)
# Timer groups: TIM4 (M1-M4), TIM3 (M5-M6), TIM1 (M7-M8)
# Note: outputs on the same timer must share protocol/rate
# ---------------------------------------------------------------------------
PD12 TIM4_CH1 TIM4 PWM(1) GPIO(50) BIDIR
PD13 TIM4_CH2 TIM4 PWM(2) GPIO(51)
PD14 TIM4_CH3 TIM4 PWM(3) GPIO(52) BIDIR
PD15 TIM4_CH4 TIM4 PWM(4) GPIO(53)
PB0  TIM3_CH3 TIM3 PWM(5) GPIO(54)
PB1  TIM3_CH4 TIM3 PWM(6) GPIO(55)
PE9  TIM1_CH1 TIM1 PWM(7) GPIO(56)
PE11 TIM1_CH2 TIM1 PWM(8) GPIO(57)

# WS2812 LED strip - own timer so it doesn't constrain motor groups
PA15 TIM2_CH1 TIM2 PWM(9) GPIO(58)

define HAL_WITH_DSHOT 1
define DEFAULT_MOT_PWM_TYPE 6

# ---------------------------------------------------------------------------
# Analog inputs
# ---------------------------------------------------------------------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC5 RSSI_ADC          ADC1

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 8

# ---------------------------------------------------------------------------
# LEDs / buzzer / misc GPIO
# ---------------------------------------------------------------------------
PE3 LED0 OUTPUT LOW GPIO(90)
PE4 LED1 OUTPUT LOW GPIO(91)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

PA8 BUZZER OUTPUT GPIO(80) LOW
define HAL_BUZZER_PIN 80

# VTX / camera power switch
PE13 PINIO1 OUTPUT GPIO(81) LOW
define RELAY1_PIN_DEFAULT 81

# ---------------------------------------------------------------------------
# DMA control (H7 DMAMUX - streams auto-assigned by hwdef script)
# ---------------------------------------------------------------------------
# Highest priority: IMU SPI and DShot motor timers
DMA_PRIORITY SPI1* TIM4* TIM3* TIM1* USART6* SPI3*
# Never share streams for IMU or primary motor timer
DMA_NOSHARE  SPI1* TIM4* TIM3*

# ---------------------------------------------------------------------------
# Features trimmed for small-board build
# ---------------------------------------------------------------------------
define HAL_MOUNT_ENABLED 1
define AP_OPTICALFLOW_ENABLED 1
define HAL_SOARING_ENABLED 0
```

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef-bl.dat`

```
# bootloader hwdef for generic STM32H743VIT6 Nano AIO

MCU STM32H7xx STM32H743xx
OSCILLATOR_HZ 8000000
APJ_BOARD_ID 5999

FLASH_SIZE_K