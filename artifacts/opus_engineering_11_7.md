# ArduPilot `hwdef.dat` for a Generic STM32H743VIT6 Nano AIO

The **pin assignments below are a representative layout** based on common H743 AIO designs (Matek/SpeedyBee-style), not a specific commercial board. Verify every pin against your actual schematic before flashing. A wrong USB, SPI CS, or motor pin can make the board unbootable or misbehave. I don't know what "Cycle 11" refers to; if it's a board revision or internal iteration, change the pinout to match it.

**How ArduPilot handles DMA:** you don't hard-code DMA streams in `hwdef.dat`. The build script (`chibios_hwdef.py`) allocates DMA streams and DMAMUX requests automatically. You control the allocation with `DMA_PRIORITY`, `DMA_NOSHARE`, and `NODMA`. The resolved stream map appears in the generated `hwdef.h` and in `build/<board>/hwdef.h` comments after `waf configure`.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```text
# =====================================================================
# hw definition file for a generic STM32H743VIT6 Nano AIO flight controller
# MCU: STM32H743VIT6 (LQFP100, ports A-E only)
# =====================================================================

# ---------------------------------------------------------------------
# MCU / board identity
# ---------------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Board ID - MUST be unique. Request an official ID via
# Tools/AP_Bootloader/board_types.txt pull request before distribution.
APJ_BOARD_ID TARGET_HW_NANO_H743_AIO
# (or a numeric placeholder for local testing, e.g. APJ_BOARD_ID 5999)

# Crystal frequency
OSCILLATOR_HZ 8000000

# ---------------------------------------------------------------------
# Flash layout (2MB H743)
#   Sector 0       : bootloader (128KB)
#   Sectors 1-2    : parameter storage (2 x 128KB)
#   Sector 3+      : firmware
# ---------------------------------------------------------------------
FLASH_SIZE_KB 2048
FLASH_RESERVE_START_KB 384
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 16384

# Use the default H7 memory map
env OPTIMIZE -O2

# ---------------------------------------------------------------------
# USB
# ---------------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# USB VBUS sense not used (PA9 reserved for USART1)
define HAL_USB_PRODUCT_ID 0x1058

# ---------------------------------------------------------------------
# Serial ports
# SERIAL0 = USB, then the UART order as mapped below
# ---------------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USART1 - SERIAL1 (Telem1 / MAVLink)
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - SERIAL2 (Telem2 / DJI / MSP)
PD5  USART2_TX USART2
PD6  USART2_RX USART2

# USART3 - SERIAL3 (GPS1)
PD8  USART3_TX USART3
PD9  USART3_RX USART3

# UART4 - SERIAL4 (GPS2 / spare)
PA0  UART4_TX UART4
PA1  UART4_RX UART4

# USART6 - SERIAL5 (RC input - CRSF/ELRS/SBUS)
PC6  USART6_TX USART6
PC7  USART6_RX USART6

# UART7 - SERIAL6 (ESC telemetry, RX only used)
PE8  UART7_TX UART7
PE7  UART7_RX UART7

# UART8 - SERIAL7 (spare / VTX SmartAudio)
PE1  UART8_TX UART8
PE0  UART8_RX UART8

# Low-bandwidth ports don't need DMA, which frees streams for SPI/DShot
NODMA UART7 UART8

# Default serial protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_None

# ---------------------------------------------------------------------
# SPI buses
# ---------------------------------------------------------------------
# SPI1 - IMU
PA5  SPI1_SCK  SPI1
PA6  SPI1_MISO SPI1
PA7  SPI1_MOSI SPI1

# SPI2 - OSD (AT7456E / MAX7456)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - Dataflash
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# Chip selects (all start high = deselected)
PA4  IMU1_CS   CS
PB12 OSD1_CS   CS
PA15 FLASH1_CS CS

# IMU data-ready (optional, used for fast sampling)
PB2  IMU1_DRDY INPUT

# ---------------------------------------------------------------------
# SPI device table
#   SPIDEV name bus devid cs_pin mode lowspeed highspeed
# ---------------------------------------------------------------------
SPIDEV imu1      SPI1 DEVID1 IMU1_CS   MODE3   2*MHZ  16*MHZ
SPIDEV osd       SPI2 DEVID2 OSD1_CS   MODE0  10*MHZ  10*MHZ
SPIDEV dataflash SPI3 DEVID3 FLASH1_CS MODE3 104*MHZ 104*MHZ

# ---------------------------------------------------------------------
# I2C (baro + external compass)
# ---------------------------------------------------------------------
I2C_ORDER I2C1
PB6  I2C1_SCL I2C1
PB7  I2C1_SDA I2C1

# ---------------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------------
# IMU - ICM-42688-P (Invensensev3 driver). Probing BMI270 as an
# alternate allows a second hardware variant on the same CS line.
IMU Invensensev3 SPI:imu1 ROTATION_YAW_180
IMU BMI270       SPI:imu1 ROTATION_YAW_180
define HAL_DEFAULT_INS_FAST_SAMPLE 1

# Barometer (on-board, I2C1)
BARO DPS310 I2C:0:0x76
BARO SPL06  I2C:0:0x76
BARO BMP280 I2C:0:0x76

# No on-board compass - probe external I2C compasses
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ---------------------------------------------------------------------
# OSD
# ---------------------------------------------------------------------
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ---------------------------------------------------------------------
# Logging - SPI NOR dataflash
# ---------------------------------------------------------------------
define HAL_LOGGING_DATAFLASH_ENABLED 1

# ---------------------------------------------------------------------
# Motor / servo outputs  (DShot + Bi-directional DShot)
# BIDIR is applied per timer on the *first* channel used for input
# capture; keep channels paired on the same timer.
# ---------------------------------------------------------------------
PB0  TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1  TIM3_CH4 TIM3 PWM(2) GPIO(51)
PB4  TIM3_CH1 TIM3 PWM(3) GPIO(52) BIDIR
PB5  TIM3_CH2 TIM3 PWM(4) GPIO(53)

PE9  TIM1_CH1 TIM1 PWM(5) GPIO(54) BIDIR
PE11 TIM1_CH2 TIM1 PWM(6) GPIO(55)
PE13 TIM1_CH3 TIM1 PWM(7) GPIO(56) BIDIR
PE14 TIM1_CH4 TIM1 PWM(8) GPIO(57)

# Addressable LED strip (WS2812) on its own timer
PD12 TIM4_CH1 TIM4 PWM(9) GPIO(58)

# Default to 4 motor outputs on quads
define HAL_FRAME_TYPE_DEFAULT 1

# ---------------------------------------------------------------------
# DMA control
# The script auto-allocates DMA1/DMA2 streams via DMAMUX. These hints
# make sure the latency-critical peripherals get dedicated streams.
# ---------------------------------------------------------------------
DMA_PRIORITY SPI1* TIM3* TIM1* USART6* SPI3*
DMA_NOSHARE  SPI1* TIM3_UP* TIM1_UP*

# ---------------------------------------------------------------------
# DroneCAN (FDCAN1)
# ---------------------------------------------------------------------
PD0  CAN1_RX CAN1
PD1  CAN1_TX CAN1
# Transceiver silent/standby pin (LOW = normal operation)
PD3  GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

define HAL_NUM_CAN_IFACES 1
define HAL_CAN_DEFAULT_NODE_ID 0

# ---------------------------------------------------------------------
# Analog inputs
# ---------------------------------------------------------------------
PC0  BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1  BATT_CURRENT_SENS ADC1 SCALE(1)
PC4  RSSI_IN           ADC1 SCALE(1)
PC5  BATT2_VOLTAGE_SENS ADC1 SCALE(1)

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
