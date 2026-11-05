# ArduPilot `hwdef.dat` for a Generic STM32H743VIT6 Nano AIO Flight Controller

## Before You Use This

- **This is a reference template, not a drop-in file.** The pin assignments follow common H743 AIO conventions and are valid alternate functions for the LQFP100 (VIT6) package. Your board's schematic is the only authority. Check every line against it before flashing.
- **The board ID is a placeholder.** `APJ_BOARD_ID` must be a unique ID. Request one by submitting a PR to `Tools/AP_Bootloader/board_types.txt`.
- **DMA channels are not listed by hand.** On STM32H7, ArduPilot's `chibios_hwdef.py` assigns DMA streams automatically through the DMAMUX. You can only influence the allocator with `DMA_PRIORITY` and `DMA_NOSHARE`. Hard-coded stream or channel tables are not valid hwdef syntax for H7 targets.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```
# hw definition file for generic STM32H743VIT6 Nano AIO flight controller
# MCU: STM32H743VIT6 (LQFP100, 2MB flash, 1MB RAM)

# ---------------------------------------------------------------------
# MCU / board identity
# ---------------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# PLACEHOLDER - obtain a real ID from ArduPilot board_types.txt
APJ_BOARD_ID TARGET_HW_NANOH743AIO

# crystal frequency
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048

# bootloader occupies the first 128k sector
FLASH_RESERVE_START_KB 128

# store parameters in flash (no FRAM on nano AIO)
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 16384

# use last 2 pages of flash for storage, ensure fw does not overlap
FLASH_BOOTLOADER_LOAD_KB 128

env OPTIMIZE -Os

USB_STRING_MANUFACTURER "ArduPilot"
USB_STRING_PRODUCT "NanoH743AIO"

# ---------------------------------------------------------------------
# Serial port order
# SERIAL0 = USB, SERIAL1..7 = UARTs, SERIAL8 = USB (SLCAN/MAVLink 2nd)
# ---------------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8 OTG2

# ---------------------------------------------------------------------
# USB
# ---------------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# SWD debug
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------------------------------------------------------------
# UARTs
# ---------------------------------------------------------------------
# USART1 - SERIAL1 (Telem1 / MAVLink)
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - SERIAL2 (Telem2 / VTX MSP)
PD5  USART2_TX USART2
PD6  USART2_RX USART2

# USART3 - SERIAL3 (GPS)
PD8  USART3_TX USART3
PD9  USART3_RX USART3

# UART4 - SERIAL4 (GPS2 / spare)
PB9  UART4_TX UART4
PB8  UART4_RX UART4

# USART6 - SERIAL5 (RC input, CRSF/ELRS)
PC6  USART6_TX USART6
PC7  USART6_RX USART6

# UART7 - SERIAL6 (ESC telemetry, RX only used)
PE8  UART7_TX UART7
PE7  UART7_RX UART7

# UART8 - SERIAL7 (spare / DJI)
PE1  UART8_TX UART8
PE0  UART8_RX UART8

# default protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_MSP_DisplayPort
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
PD7  SPI1_MOSI SPI1

# SPI2 - AT7456E analog OSD
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - dataflash
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# chip selects
PC15 IMU1_CS   CS
PB12 OSD1_CS   CS
PA4  FLASH1_CS CS

# IMU data-ready (optional, used for logging/fast sampling diagnostics)
PB2  IMU1_DRDY INPUT

# ---------------------------------------------------------------------
# SPI device table
# name       bus  devid   cs         mode   lowspeed  highspeed
# ---------------------------------------------------------------------
SPIDEV imu1      SPI1 DEVID1 IMU1_CS   MODE3  2*MHZ   16*MHZ
SPIDEV osd       SPI2 DEVID4 OSD1_CS   MODE0 10*MHZ   10*MHZ
SPIDEV dataflash SPI3 DEVID1 FLASH1_CS MODE3 104*MHZ 104*MHZ

# ---------------------------------------------------------------------
# I2C buses
# ---------------------------------------------------------------------
# I2C1 - onboard baro
PB6  I2C1_SCL I2C1
PB7  I2C1_SDA I2C1

# I2C2 - external (GPS/compass connector)
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

I2C_ORDER I2C2 I2C1

# all buses treated as external for compass probing
define HAL_I2C_INTERNAL_MASK 0

# ---------------------------------------------------------------------
# DroneCAN (FDCAN1)
# ---------------------------------------------------------------------
PD0  CAN1_RX CAN1
PD1  CAN1_TX CAN1
# transceiver silent/standby pin (drive LOW for normal operation)
PD3  GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

define HAL_CAN_POOL_SIZE 6000

# ---------------------------------------------------------------------
# Motor / servo outputs (all DShot + bidirectional DShot capable)
# ---------------------------------------------------------------------
PB0  TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1  TIM3_CH4 TIM3 PWM(2) GPIO(51)
PD12 TIM4_CH1 TIM4 PWM(3) GPIO(52) BIDIR
PD13 TIM4_CH2 TIM4 PWM(4) GPIO(53)
PD14 TIM4_CH3 TIM4 PWM(5) GPIO(54) BIDIR
PD15 TIM4_CH4 TIM4 PWM(6) GPIO(55)
PC8  TIM8_CH3 TIM8 PWM(7) GPIO(56) BIDIR
PC9  TIM8_CH4 TIM8 PWM(8) GPIO(57)

# WS2812 LED strip
PA8  TIM1_CH1 TIM1 PWM(9) GPIO(58)

define HAL_WITH_DSHOT 1

# ---------------------------------------------------------------------
# Buzzer
# ---------------------------------------------------------------------
PA15 TIM2_CH1 TIM2 GPIO(32) ALARM

# ---------------------------------------------------------------------
# Analog inputs (ADC1)
# ---------------------------------------------------------------------
PC0  BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1  BATT_CURRENT_SENS ADC1 SCALE(1)
PC4  RSSI_ADC_PIN      ADC1 SCALE(1)
PC5  PRESSURE_SENS     ADC1 SCALE(2)

define HAL_BATT_MONITOR_DEFAULT 4
define