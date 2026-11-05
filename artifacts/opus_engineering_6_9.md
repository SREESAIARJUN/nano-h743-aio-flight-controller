# ArduPilot hwdef for an STM32H743VIT6 Nano AIO Flight Controller

This is a complete, buildable hardware definition for a generic H743 nano AIO board. It includes:

- an ICM42688 and BMI270 on separate SPI buses
- a DPS310 barometer
- a MAX7456 OSD
- SPI NOR flash for logging
- 6 UARTs
- 8 DShot motor outputs
- an LED strip output
- FDCAN1 for DroneCAN

> ⚠️ **Verify every pin against your board's schematic before flashing.** These assignments are electrically valid for the H743 alternate-function table and conflict-free. They are not taken from a specific commercial board. A wrong pin can damage hardware or produce a board that won't boot.

---

## How DMA Is Handled in ArduPilot

ArduPilot's `chibios_hwdef.py` **allocates DMA streams automatically** at build time. On the H7, the DMAMUX makes any request routable to any stream. You don't write stream or channel numbers by hand. Instead you control allocation with these directives:

- `DMA_PRIORITY`: which peripherals get first pick
- `DMA_NOSHARE`: which peripherals get exclusive streams
- `NODMA`: per-UART opt-out

The resolved map is written to `build/<board>/hwdef.h`. A representative result is shown in section 4.

---

## 1. `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```
# hw definition file for NanoH743AIO
# STM32H743VIT6, 8MHz HSE, LQFP100

# ---------------------------------------------------------------
# MCU / board identity
# ---------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Board ID - replace with an ID registered in Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID TARGET_HW_NANOH743AIO

OSCILLATOR_HZ 8000000
MCU_CLOCKRATE_MHZ 480

FLASH_SIZE_KB 2048
# 128kB reserved for bootloader (sector 0)
FLASH_RESERVE_START_KB 128

# Default stack sizes are fine for H7
define HAL_STORAGE_SIZE 32768
STORAGE_FLASH_PAGE 1

# ---------------------------------------------------------------
# USB (OTG1 FS)
# ---------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# SWD debug
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

USB_STRING_MANUFACTURER "ArduPilot"
USB_STRING_PRODUCT "NanoH743AIO"

# ---------------------------------------------------------------
# Serial port order
#  SERIAL0 = USB
#  SERIAL1 = USART1  (Telem/VTX)
#  SERIAL2 = USART2  (GPS)
#  SERIAL3 = USART3  (ESC telemetry)
#  SERIAL4 = UART7   (spare / DJI)
#  SERIAL5 = UART8   (spare)
#  SERIAL6 = USART6  (RC input)
# ---------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART7 UART8 USART6

# USART1
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - GPS
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - ESC telemetry (RX only needed; TX left for flexibility)
PD8 USART3_TX USART3
PD9 USART3_RX USART3 NODMA

# UART7
PE8 UART7_TX UART7
PE7 UART7_RX UART7

# UART8
PE1 UART8_TX UART8 NODMA
PE0 UART8_RX UART8 NODMA

# USART6 - RC input (CRSF/ELRS/SBUS etc. via RC protocol detection)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# ---------------------------------------------------------------
# Default serial protocols
# ---------------------------------------------------------------
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL3_BAUD 115
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_None
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_None
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_RCIN

# ---------------------------------------------------------------
# SPI buses
#  SPI1 - IMU1 (ICM42688)
#  SPI2 - OSD  (MAX7456)
#  SPI3 - Dataflash (W25Q128)
#  SPI4 - IMU2 (BMI270)
# ---------------------------------------------------------------
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PD7 SPI1_MOSI SPI1

PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

PE12 SPI4_SCK  SPI4
PE13 SPI4_MISO SPI4
PE14 SPI4_MOSI SPI4

# Chip selects
PC15 IMU1_CS  CS
PE11 IMU2_CS  CS
PB12 MAX7456_CS CS
PA15 FLASH1_CS CS

# IMU data-ready / interrupt lines (optional, not used by default drivers)
PB2 IMU1_DRDY INPUT
PE15 IMU2_DRDY INPUT

# ---------------------------------------------------------------
# SPI device table
# SPIDEV name bus devid cs mode lowspeed highspeed
# ---------------------------------------------------------------
SPIDEV imu1     SPI1 DEVID1 IMU1_CS    MODE3  2*MHZ  16*MHZ
SPIDEV imu2     SPI4 DEVID1 IMU2_CS    MODE3  1*MHZ  10*MHZ
SPIDEV osd      SPI2 DEVID4 MAX7456_CS MODE0 10*MHZ  10*MHZ
SPIDEV dataflash SPI3 DEVID1 FLASH1_CS MODE3 32*MHZ  32*MHZ

# ---------------------------------------------------------------
# IMUs
# ---------------------------------------------------------------
IMU Invensensev3 SPI:imu1 ROTATION_YAW_180
IMU BMI270       SPI:imu2 ROTATION_YAW_270

define HAL_DEFAULT_INS_FAST_SAMPLE 3

# ---------------------------------------------------------------
# I2C buses
#  I2C1 - onboard baro, external compass
#  I2C2 - external
# ---------------------------------------------------------------
I2C_ORDER I2C1 I2C2

PB8  I2C1_SCL I2C1
PB9  I2C1_SDA I2C1

PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

# Barometer
BARO DPS310 I2C:0:0x76

# Compass: probe external only (no onboard mag on AIO)
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 0
define AP_COMPASS_ENABLE_DEFAULT 1

# ---------------------------------------------------------------
# OSD
# ---------------------------------------------------------------
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ---------------------------------------------------------------
# Logging to SPI dataflash
# ---------------------------------------------------------------
define HAL_LOGGING_DATAFLASH_ENABLED 1
define HAL_OS_FATFS_IO 0

# ---------------------------------------------------------------
# ADC
# ---------------------------------------------------------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 RSSI_ADC_PIN      ADC1 SCALE(1)

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 4

# ---------------------------------------------------------------
# Motor / servo outputs (8x DShot, bidirectional capable)
# ---------------------------------------------------------------
PB0  TIM3_CH3 TIM3 PWM(1) GPIO(50)
PB1  TIM3_CH4 TIM3 PWM(2) GPIO(51)
PA0  TIM5_CH1 TIM5 PWM(3) GPIO(52) BIDIR
PA1  TIM5_CH2 TIM5 PWM(4) GPIO(53)
PD12 TIM4_CH1 TIM4 PWM(5) GPIO(54) BIDIR
PD13 TIM4_CH2 TIM4 PWM(6) GPIO(55)
PD14 TIM4_CH3 TIM4 PWM(7) GPIO(56) BIDIR
PD15 TIM4_CH4 TIM4 PWM(8) GPIO(57)

# LED strip (WS2812 / NeoPixel)
PA8  TIM1_CH1 TIM1 PWM(9) GPIO(58)

define HAL_WITH_DSHOT 1

# ---------------------------------------------------------------
# FDCAN1 - DroneCAN
# ---------------------------------------------------------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1

# Optional transceiver silent/standby pin (pull low = active)
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW GPIO(70)

define HAL_NUM_CAN_IFACES 1
define HAL_CAN_DEFAULT_NODE_ID 0
define HAL_ENABLE_DRONECAN_DRIVERS 1

# ---------------------------------------------------------------
# LEDs, buzzer, misc GPIO
# ---------------------------------------------------------------
PE3 LED0 OU