# ArduPilot hwdef for an STM32H743VIT6 Nano AIO Flight Controller

**Before you use this:** I don't have the schematic for your board, so this is a reference design, not your board's actual definition.

- The pins are valid for the STM32H743VIT6 (LQFP100) and follow common AIO layouts, such as the Matek, MicoAir and SpeedyBee H743 boards.
- Every pin, chip-select and IMU rotation must be checked against your schematic before flashing. A wrong pin can short a driven output or brick the bootloader.
- `APJ_BOARD_ID` uses an unregistered placeholder. Request an official ID through ArduPilot's `Tools/AP_Bootloader/board_types.txt` process before you distribute firmware.

The files go in `libraries/AP_HAL_ChibiOS/hwdef/NanoAIO-H743/`:
- `hwdef.dat` (main firmware)
- `hwdef-bl.dat` (bootloader)
- `defaults.parm` (default parameters)

---

## 1. `hwdef.dat`

```
# hw definition file for NanoAIO-H743 (STM32H743VIT6, LQFP100)
# Integrated 4in1 ESC-ready AIO: ICM42688P, DPS310, AT7456E OSD,
# W25Q128 dataflash, FDCAN1 DroneCAN, 8 PWM, 7 UARTs

# ---------------------------------------------------------------
# MCU / board identity
# ---------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Placeholder - must be replaced with an officially allocated ID
APJ_BOARD_ID 5900

# 8 MHz HSE crystal
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048
# Bootloader occupies first 128 KB sector
FLASH_RESERVE_START_KB 128

# Use 2nd and 3rd flash pages for parameter storage
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 32768

# Enable the larger feature set allowed by 2 MB flash
env OPTIMIZE -O2

# ---------------------------------------------------------------
# Serial ports (ArduPilot SERIALn order)
# SERIAL0 = USB, SERIAL1..7 = UARTs, SERIAL8 = USB SLCAN/2nd CDC
# ---------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8 OTG2

# USB (OTG1 FS)
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9  VBUS INPUT OPENDRAIN

# USART1 - SERIAL1 - Telem1 (MAVLink2)
PB14 USART1_TX USART1
PB15 USART1_RX USART1

# USART2 - SERIAL2 - Telem2 / VTX (DisplayPort / MSP)
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - SERIAL3 - GPS1
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - SERIAL4 - GPS2 / user
PB9 UART4_TX UART4
PB8 UART4_RX UART4

# USART6 - SERIAL5 - RC input (CRSF/ELRS/SBUS)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 - SERIAL6 - ESC telemetry (RX only on most AIOs)
PE8 UART7_TX UART7
PE7 UART7_RX UART7

# UART8 - SERIAL7 - user / RunCam / Lidar
PE1 UART8_TX UART8
PE0 UART8_RX UART8

# SWD
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------------------------------------------------------
# SPI buses
# ---------------------------------------------------------------
# SPI1 - IMU (ICM42688P)
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1

# SPI2 - OSD (AT7456E)
PB13 SPI2_SCK  SPI2
PC2  SPI2_MISO SPI2
PC3  SPI2_MOSI SPI2

# SPI3 - Dataflash (W25Q128JV)
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# SPI4 - spare / second IMU footprint
PE12 SPI4_SCK  SPI4
PE13 SPI4_MISO SPI4
PE14 SPI4_MOSI SPI4

# Chip selects (idle high)
PC15 IMU1_CS  CS
PB12 OSD_CS   CS
PA15 FLASH_CS CS
PE11 IMU2_CS  CS

# IMU data-ready interrupt
PB2 DRDY1_ICM42688 INPUT

# ---------------------------------------------------------------
# SPI device table
# name       bus  dev cs        mode       lowspeed  highspeed
# ---------------------------------------------------------------
SPIDEV icm42688   SPI1 DEVID1 IMU1_CS  MODE3   2*MHZ  16*MHZ
SPIDEV osd        SPI2 DEVID1 OSD_CS   MODE0  10*MHZ  10*MHZ
SPIDEV dataflash  SPI3 DEVID1 FLASH_CS MODE3  32*MHZ  32*MHZ
SPIDEV icm42688_2 SPI4 DEVID1 IMU2_CS  MODE3   2*MHZ  16*MHZ

# ---------------------------------------------------------------
# I2C buses
# ---------------------------------------------------------------
I2C_ORDER I2C1 I2C2

# I2C1 - external (GPS compass, airspeed)
PB6 I2C1_SCL I2C1
PB7 I2C1_SDA I2C1

# I2C2 - internal barometer
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

# ---------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------
# Primary IMU (rotation must match PCB orientation)
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
# Optional second IMU - probed, harmless if not populated
IMU Invensensev3 SPI:icm42688_2 ROTATION_YAW_270

define HAL_DEFAULT_INS_FAST_SAMPLE 1

# Barometers (probe list - only fitted part is detected)
BARO DPS310 I2C:1:0x76
BARO BMP280 I2C:1:0x76
BARO SPL06  I2C:1:0x76

# No onboard compass - probe external compasses on I2C
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 2
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ---------------------------------------------------------------
# Dataflash logging
# ---------------------------------------------------------------
DATAFLASH littlefs:w25nxx
define HAL_LOGGING_DATAFLASH_ENABLED 1
define HAL_OS_FATFS_IO 0

# ---------------------------------------------------------------
# OSD
# ---------------------------------------------------------------
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ---------------------------------------------------------------
# DroneCAN (FDCAN1) with transceiver silent pin
# ---------------------------------------------------------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

define HAL_CAN_POOL_SIZE 6000
define HAL_ENABLE_DRONECAN_DRIVERS 1

# ---------------------------------------------------------------
# PWM outputs (motors) - grouped by timer for DShot/BDShot
# ---------------------------------------------------------------
# TIM3 -> M1/M2 (bidirectional DShot capable)
PB0 TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1 TIM3_CH4 TIM3 PWM(2) GPIO(51)

# TIM5 -> M3..M6
PA0 TIM5_CH1 TIM5 PWM(3) GPIO(52) BIDIR
PA1 TIM5_CH2 TIM5 PWM(4) GPIO(53)
PA2 TIM5_CH3 TIM5 PWM(5) GPIO(54) BIDIR
PA3 TIM5_CH4 TIM5 PWM(6) GPIO(55)

# TIM4 -> M7/M8
PD12 TIM4_CH1 TIM4 PWM(7) GPIO(56) BIDIR
PD13 TIM4_CH2 TIM4 PWM(8) GPIO(57)

# TIM1 -> addressable LED strip
PE9 TIM1_CH1 TIM1 PWM(9) GPIO(58)

# ---------------------------------------------------------------
# DMA configuration
# On H7 the DMAMUX lets any stream serve any request; hwdef's
# resolver assigns streams automatically. These directives
# control priority and prevent sharing on latency-critical paths.
# ---------------------------------------------------------------
DMA_PRIORITY SPI1* TIM3* TIM5* TIM4* USART6* SPI3*
DMA_NOSHARE  SPI1* TIM3_UP TIM5_UP TIM4_UP USART6*

# Disable DMA on low-rate UART TX to save streams
NODMA UART8_TX UART7_TX

# ---------------------------------------------------------------
# ADC - power monitoring
# ---------------------------------------------------------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 RSSI_ADC          ADC1
PC5 PRESSURE_SENS     ADC1 SCALE(2)

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 4

# ---------------------------------------------------------------
# LEDs, buzzer, switchable power
# ---------------------------------------------------------------
PE3 LED0 OUTPUT LOW GPIO(90)
PE4 LED1 OUTPUT LOW GPIO(91)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_ON 1

PE5 BUZZER OUTPUT GPIO(80) LOW
define HAL_BUZZER_PIN 80
define HAL_BUZZER_ON 1
define HAL_BUZZER_OFF 0

# 10V VTX/camera switch, controlled by RELAYn
PD10 PIN