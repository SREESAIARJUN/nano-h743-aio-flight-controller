# ArduPilot hwdef for a Generic STM32H743VIT6 Nano AIO Flight Controller

**Important:** "STM32H743VIT6 nano AIO" describes a class of boards, not one specific board. I don't have a schematic or earlier "cycles" of this work, so this is a self-consistent reference design. It uses only pins that exist on the LQFP100 (V) package, with no alternate-function conflicts. **Before flashing, verify every pin against your actual schematic.** A wrong pin can short outputs or leave the board unbootable.

Directory layout: `libraries/AP_HAL_ChibiOS/hwdef/NanoAIO-H743/`

---

## 1. `hwdef.dat`

```
# hw definition file for NanoAIO-H743 (STM32H743VIT6, LQFP100)
# Single IMU, I2C baro, SPI OSD, SPI NAND dataflash, FDCAN1 for DroneCAN

# ---------------------------------------------------------------- MCU / clocks
MCU STM32H7xx STM32H743xx

# Board ID: placeholder. Request an official ID via a PR to
# Tools/AP_Bootloader/board_types.txt before distributing firmware.
APJ_BOARD_ID 5999

OSCILLATOR_HZ 8000000

# 2 MB flash; first 128 KB sector reserved for bootloader
FLASH_SIZE_KB 2048
FLASH_RESERVE_START_KB 128

# Use TIM12 for the ChibiOS system tick (keeps TIM2/3/4/5/8 free for outputs)
STM32_ST_USE_TIMER 12
define CH_CFG_ST_RESOLUTION 16

# Parameter storage in the last two 128 KB flash sectors
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 32768

# ---------------------------------------------------------------- USB
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9  VBUS INPUT OPENDRAIN

USB_STRING_MANUFACTURER "ArduPilot"

# ---------------------------------------------------------------- SWD
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------------------------------------------------------- Serial ports
# SERIAL0=USB, 1=USART1, 2=USART2, 3=USART3, 4=UART7, 5=UART8, 6=USART6
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART7 UART8 USART6

# USART1 - Telemetry / MAVLink
PB6 USART1_TX USART1
PB7 USART1_RX USART1

# USART2 - GPS
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - spare / DisplayPort (HD VTX)
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART7 - ESC telemetry (RX only needed); no DMA to save streams
PE8 UART7_TX UART7 NODMA
PE7 UART7_RX UART7 NODMA

# UART8 - spare / VTX SmartAudio; no DMA
PE1 UART8_TX UART8 NODMA
PE0 UART8_RX UART8 NODMA

# USART6 - RC input (CRSF/ELRS/SBUS via SERIAL6_PROTOCOL=23)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# ---------------------------------------------------------------- SPI buses
# SPI1 - IMU
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1

# SPI2 - OSD (AT7456E)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - NAND dataflash (W25N01G)
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# Chip selects
PA4  IMU1_CS  CS
PB12 OSD_CS   CS
PA15 FLASH_CS CS

# SPI device table
#      name       bus  devid  cs        mode  lowspeed  highspeed
SPIDEV imu1       SPI1 DEVID1 IMU1_CS   MODE3 2*MHZ     16*MHZ
SPIDEV osd        SPI2 DEVID4 OSD_CS    MODE0 10*MHZ    10*MHZ
SPIDEV dataflash  SPI3 DEVID1 FLASH_CS  MODE3 104*MHZ   104*MHZ

# ---------------------------------------------------------------- I2C
# I2C1 - onboard baro + external compass on the GPS connector
PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1

I2C_ORDER I2C1
define HAL_I2C_INTERNAL_MASK 0

# ---------------------------------------------------------------- CAN (DroneCAN)
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
# Transceiver silent/standby pin (drive LOW = normal operation)
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

# ---------------------------------------------------------------- Sensors
# IMU
IMU Invensensev3 SPI:imu1 ROTATION_YAW_180
define HAL_DEFAULT_INS_FAST_SAMPLE 1

# Barometer (probe both common parts)
BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76

# No onboard compass; probe external I2C and DroneCAN compasses
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define ALLOW_ARM_NO_COMPASS

# ---------------------------------------------------------------- Dataflash logging
define HAL_LOGGING_DATAFLASH_ENABLED 1

# ---------------------------------------------------------------- OSD
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ---------------------------------------------------------------- ADC
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 RSSI_IN           ADC1 SCALE(1)

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 4

# ---------------------------------------------------------------- Motor / servo outputs
# 8 outputs, all DShot capable; BIDIR on TIM3 and TIM5 groups
PB0  TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1  TIM3_CH4 TIM3 PWM(2) GPIO(51)
PA0  TIM5_CH1 TIM5 PWM(3) GPIO(52) BIDIR
PA1  TIM5_CH2 TIM5 PWM(4) GPIO(53)
PA2  TIM5_CH3 TIM5 PWM(5) GPIO(54