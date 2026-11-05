# ArduPilot hwdef.dat: STM32H743VIT6 Nano AIO Flight Controller

> **Important:** This is a complete, internally consistent hwdef for a *representative* H743VIT6 (LQFP100) nano AIO design. Every pin and alternate function below is valid for the H743 in the LQFP100 package. However, **your board's schematic is the source of truth.** Verify each line against the actual netlist before flashing. A wrong CS, EXTI, or motor pin can produce a non-booting board or uncommanded motor output. The `APJ_BOARD_ID` is a placeholder and must be registered with ArduPilot (`Tools/AP_Bootloader/board_types.txt`) before release.

Assumed hardware:

| Function | Device | Bus |
|---|---|---|
| IMU | ICM-42688-P | SPI1 |
| OSD | AT7456E (MAX7456 compatible) | SPI2 |
| Logging | W25N01G 1 Gbit NAND | SPI3 |
| Barometer | DPS310 / BMP280 (probed) | I2C2 (internal) |
| Compass | External, via GPS port | I2C1 |
| CAN | FDCAN1 + transceiver (DroneCAN) | PD0/PD1 |
| Outputs | 8 × DShot/bidir-DShot + 1 × WS2812 LED | TIM3/TIM1/TIM4/TIM15 |
| Battery | Voltage and current sensing | ADC1 |

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```text
# =====================================================================
#  NanoH743AIO - STM32H743VIT6 nano AIO flight controller
#  hwdef.dat for ArduPilot (ChibiOS HAL)
#  Revision: Cycle 28
# =====================================================================

# ---------------------------------------------------------------------
# MCU / Clock / Board ID
# ---------------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Placeholder ID - MUST be registered in Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID AP_HW_NanoH743AIO

# 8 MHz HSE crystal
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048

# 128 KB bootloader at the start of flash
FLASH_RESERVE_START_KB 128

# Parameter storage in internal flash (sectors 14/15 on bank 2)
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 32768

# 1 kHz scheduler capable on H7
define HAL_CHIBIOS_ARCH_FMUV3 0

# ---------------------------------------------------------------------
# Status LEDs
# ---------------------------------------------------------------------
PE3 LED_BLUE  OUTPUT LOW GPIO(90)
PE4 LED_GREEN OUTPUT LOW GPIO(91)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

# ---------------------------------------------------------------------
# USB (OTG1 FS)
# ---------------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9  VBUS INPUT OPENDRAIN

# ---------------------------------------------------------------------
# Serial port order
#   SERIAL0 = USB
#   SERIAL1 = USART1 (Telem1 / MAVLink)
#   SERIAL2 = USART2 (Telem2 / VTX control)
#   SERIAL3 = USART3 (GPS)
#   SERIAL4 = UART4  (ESC telemetry)
#   SERIAL5 = USART6 (RC input - CRSF/ELRS/SBUS)
#   SERIAL6 = UART7  (Spare / DJI O3)
#   SERIAL7 = UART8  (Spare)
# ---------------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USART1 (TX on PB6 since PA9 is used for VBUS sense)
PB6  USART1_TX USART1
PA10 USART1_RX USART1

# USART2
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - GPS
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - ESC telemetry (RX only used)
PA0 UART4_TX UART4
PA1 UART4_RX UART4

# USART6 - RC input
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7
PE8 UART7_TX UART7
PE7 UART7_RX UART7

# UART8
PE1 UART8_TX UART8
PE0 UART8_RX UART8

# Default serial protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_MSP
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_None
define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_None
define DEFAULT_SERIAL4_BAUD 115

# No dedicated RCIN timer pin - RC comes in on SERIAL5
define HAL_SERIAL5_PROTOCOL SerialProtocol_RCIN

# ---------------------------------------------------------------------
# SPI buses
# ---------------------------------------------------------------------
# SPI1 - IMU (dedicated bus, no sharing)
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1

# SPI2 - OSD
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - Dataflash
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# Chip selects
PA4  IMU1_CS   CS
PB12 OSD1_CS   CS
PA15 FLASH1_CS CS

# IMU data-ready interrupt
PC4 IMU1_DRDY INPUT GPIO(92)

# ---------------------------------------------------------------------
# SPI device table
#   SPIDEV name bus devid cs mode lowspeed highspeed
# ---------------------------------------------------------------------
SPIDEV icm42688  SPI1 DEVID1 IMU1_CS   MODE3  2*MHZ  16*MHZ
SPIDEV osd       SPI2 DEVID1 OSD1_CS   MODE0 10*MHZ  10*MHZ
SPIDEV dataflash SPI3 DEVID1 FLASH1_CS MODE3 32*MHZ  32*MHZ

# ---------------------------------------------------------------------
# IMU
# ---------------------------------------------------------------------
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
define HAL_DEFAULT_INS_FAST_SAMPLE 1

# ---------------------------------------------------------------------
# I2C buses
#   I2C1 = external (GPS/compass connector)
#   I2C2 = internal (barometer)
# ---------------------------------------------------------------------
I2C_ORDER I2C2 I2C1

PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1

PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

# Barometer (probe both common nano-AIO parts)
BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76

# Compass - external only
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 1
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ---------------------------------------------------------------------
# OSD (AT7456E)
# ---------------------------------------------------------------------
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ---------------------------------------------------------------------
# Dataflash logging (W25N01G)
# ---------------------------------------------------------------------
define HAL_LOGGING_DATAFLASH_ENABLED 1
define HAL_LOGGING_MAVLINK_ENABLED 0

# ---------------------------------------------------------------------
# CAN / DroneCAN (FDCAN1)
# ---------------------------------------------------------------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1

# Transceiver silent/standby control (LOW = active)
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW GPIO(70)

define HAL_NUM_CAN_IFACES 1
define HAL_CAN_POOL_SIZE 6000
define HAL_ENABLE_DRONECAN_DRIVERS 1

# ---------------------------------------------------------------------
# Motor / servo outputs
#   Group 1: TIM3  -> M1, M2
#   Group 2: TIM1  -> M3..M6
#   Group 3: TIM4  -> M7, M8
#   Group 4: TIM15 -> serial LED (isolated so LED timing never
#            constrains motor protocol)
# ---------------------------------------------------------------------
PB0  TIM3_CH3  TIM3  PWM(1) GPIO(50) BIDIR
PB1  TIM3_CH4  TIM3  PWM(2) GPIO(51)
PE9  TIM1_CH1  TIM1  PWM(3) GPIO(52) BIDIR
PE11 TIM1_CH2  TIM1  PWM(4) GPIO(53)
P