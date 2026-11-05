# ArduPilot hwdef for an STM32H743VIT6 Nano AIO Flight Controller (Cycle 1)

This is a reference hwdef for a generic H743 "nano AIO" board. It uses the LQFP100 package, so only ports A–E plus PH0/PH1 are available. It has been checked for pin, alternate-function and timer conflicts at the pin-mux level.

**The pinout is a proposal, not a real board.** Before flashing anything, verify every pin against your actual schematic. In particular:

- The board ID is a placeholder. Request a real one through `Tools/AP_Bootloader/board_types.txt`.
- The resistor-divider and current-sensor scale values are placeholders.

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

```
# hw definition file for NanoH743AIO (STM32H743VIT6, LQFP100)
# Cycle 1 - reference pinout, verify against schematic

# ---------------------------------------------------------------
# MCU / clock / flash
# ---------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# TODO: request official ID; placeholder must not collide
APJ_BOARD_ID TARGET_HW_NANOH743AIO

# 8MHz external crystal
OSC_FREQUENCY 8000000

FLASH_SIZE_KB 2048
# first 128k sector reserved for bootloader
FLASH_RESERVE_START_KB 128
FLASH_BOOTLOADER_LOAD_KB 128

# use flash storage for parameters (no FRAM on nano AIO)
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 16384

# ---------------------------------------------------------------
# Serial port order
# SERIAL0=USB, then physical UARTs
# ---------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USB
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# SWD
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# USART1 - SERIAL1 (Telem1 / MAVLink)
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - SERIAL2 (Telem2 / VTX)
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - SERIAL3 (GPS)
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - SERIAL4 (GPS2 / ESC telem)
PA0 UART4_TX UART4
PA1 UART4_RX UART4

# USART6 - SERIAL5 (RC input, CRSF/ELRS/SBUS via serial)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 - SERIAL6 (spare / DJI)
PE8 UART7_TX UART7
PE7 UART7_RX UART7

# UART8 - SERIAL7 (ESC telemetry, RX only, no DMA needed)
PE1 UART8_TX UART8 NODMA
PE0 UART8_RX UART8 NODMA

# ---------------------------------------------------------------
# SPI buses
# ---------------------------------------------------------------
# SPI1 - IMU (dedicated, high rate)
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PD7 SPI1_MOSI SPI1

# SPI2 - OSD (MAX7456 / AT7456E)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - dataflash
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# Chip selects
PA4  IMU1_CS    CS
PB12 OSD_CS     CS
PA15 FLASH_CS   CS

# IMU data-ready interrupt (not used by driver, held as input)
PE4 IMU1_DRDY INPUT

# ---------------------------------------------------------------
# I2C
# ---------------------------------------------------------------
I2C_ORDER I2C1

PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1

# ---------------------------------------------------------------
# DroneCAN (FDCAN1)
# ---------------------------------------------------------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
# transceiver silent/standby control (LOW = active)
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

# ---------------------------------------------------------------
# Motor / servo outputs
# ---------------------------------------------------------------
# TIM1 - M1..M4 (DShot / bidir capable)
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50) BIDIR
PE11 TIM1_CH2 TIM1 PWM(2) GPIO(51)
PE13 TIM1_CH3 TIM1 PWM(3) GPIO(52) BIDIR
PE14 TIM1_CH4 TIM1 PWM(4) GPIO(53)

# TIM4 - M5..M8
PD12 TIM4_CH1 TIM4 PWM(5) GPIO(54) BIDIR
PD13 TIM4_CH2 TIM4 PWM(6) GPIO(55)
PD14 TIM4_CH3 TIM4 PWM(7) GPIO(56) BIDIR
PD15 TIM4_CH4 TIM4 PWM(8) GPIO(57)

# TIM15 - LED strip (NeoPixel), own timer so DShot rates are unaffected
PE5 TIM15_CH1 TIM15 PWM(9) GPIO(58)

# Buzzer on TIM8 (standalone timer)
PC8 TIM8_CH3 TIM8 GPIO(32) ALARM

# ---------------------------------------------------------------
# ADC
# ---------------------------------------------------------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 RSSI_ADC_PIN      ADC1 SCALE(1)
PC5 PRESSURE_SENS     ADC1 SCALE(2)

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
# TODO: match to divider / current sensor on schematic
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 4
define HAL_DEFAULT_AIRSPEED_PIN 8

# ---------------------------------------------------------------
# LEDs / misc GPIO
# ---------------------------------------------------------------
PE3 LED0 OUTPUT LOW GPIO(90)   # blue
PE2 LED1 OUTPUT LOW GPIO(91)   # green
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_ON 1

# switchable VTX power
PC13 PINIO1 OUTPUT GPIO(81) LOW
define RELAY1_PIN_DEFAULT 81

# ---------------------------------------------------------------
# DMA configuration
# ---------------------------------------------------------------
# On H7 the DMAMUX lets any peripheral request map to any stream on
# DMA1/DMA2, so ArduPilot allocates streams automatically at build
# time. Control comes from priority and no-share lists, not from
# hard-coded stream numbers.
DMA_PRIORITY TIM1* TIM4* SPI1* SPI3* USART6* USART3*
DMA_NOSHARE  SPI1* TIM1* TIM4* USART6*

# ---------------------------------------------------------------
# SPI device table
# name, bus, devid, cs, mode, lowspeed, highspeed
# ---------------------------------------------------------------
SPIDEV icm42688  SPI1 DEVID1 IMU1_CS   MODE3  2*MHZ  16*MHZ
SPIDEV osd       SPI2 DEVID4 OSD_CS    MODE0 10*MHZ  10*MHZ
SPIDEV dataflash SPI3 DEVID1 FLASH_CS  MODE3 32*MHZ  32*MHZ

# ---------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
define HAL_DEFAULT_INS_FAST_SAMPLE 1

BARO DPS310 I2C:0:0x76
# alternative population:
# BARO BMP280 I2C:0:0x76

# external compass only on nano AIO
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 0
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ---------------------------------------------------------------
# Logging
# ---------------------------------------------------------------
define HAL_LOGGING_DATAFLASH_ENABLED 1

# ---------------------------------------------------------------
# OSD
# ---------------------------------------------------------------
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ---------------------------------------------------------------
# Features
# ---------------------------------