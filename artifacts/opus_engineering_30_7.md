# ArduPilot hwdef.dat for a Generic STM32H743VIT6 Nano AIO Flight Controller

**Use this as a template only.** I don't have the schematic for your board. The pin assignments below follow common H743 AIO layouts and are valid for the STM32H743VIT6 (LQFP100), but you must check every pin against your actual schematic before flashing. A wrong pin can short a driven output or leave a sensor unreachable. I also don't know what "Cycle 30" refers to, so this file doesn't address it. If it's a revision or project milestone with specific requirements, share them and I'll adapt the file.

## How DMA works in ArduPilot

You don't hand-assign DMA streams in hwdef.dat. The build script (`chibios_hwdef.py`) runs a DMA resolver that maps streams through the H7's DMAMUX automatically. You influence it with:

- `DMA_PRIORITY`, which says who gets dedicated streams first.
- `DMA_NOSHARE`, which forbids stream sharing for latency-critical peripherals.
- `NODMA` on individual UART lines.

After building, check the `// auto-generated DMA mapping` section in `build/<board>/hwdef.h` to see the final allocation.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```
# hw definition file for NanoH743AIO (STM32H743VIT6, LQFP100)
# TEMPLATE - verify all pins against board schematic

# ---------------------------------------------------------------
# MCU / clock / flash
# ---------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Request a unique ID from the ArduPilot team before release
APJ_BOARD_ID TARGET_HW_NANOH743AIO
# placeholder - must be registered in Tools/AP_Bootloader/board_types.txt

OSCILLATOR_HZ 8000000
FLASH_SIZE_KB 2048
FLASH_RESERVE_START_KB 128
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 15360

# ---------------------------------------------------------------
# USB
# ---------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9  VBUS INPUT OPENDRAIN

USB_STRING_MANUFACTURER "ArduPilot"

# ---------------------------------------------------------------
# SWD debug
# ---------------------------------------------------------------
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------------------------------------------------------
# Serial ports
# SERIAL0=USB, 1..7 map to the UARTs below
# ---------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USART1 - SERIAL1 (Telem1 / MAVLink)
PB6 USART1_TX USART1
PB7 USART1_RX USART1

# USART2 - SERIAL2 (Telem2 / VTX control)
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - SERIAL3 (GPS)
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - SERIAL4 (spare)
PC10 UART4_TX UART4
PC11 UART4_RX UART4

# USART6 - SERIAL5 (RC input, e.g. ELRS/CRSF)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 - SERIAL6 (ESC telemetry, RX only)
PE7 UART7_RX UART7
PE8 UART7_TX UART7 NODMA

# UART8 - SERIAL7 (DJI / MSP DisplayPort)
PE0 UART8_RX UART8
PE1 UART8_TX UART8

# Default serial protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_MSP_DisplayPort

# ---------------------------------------------------------------
# SPI buses
# ---------------------------------------------------------------
# SPI1 - IMU
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PD7 SPI1_MOSI SPI1

# SPI2 - OSD (AT7456E / MAX7456)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - dataflash (PB3/PB4/PA15 are JTAG pins, fine with SWD-only debug)
PB3 SPI3_SCK  SPI3
PB4 SPI3_MISO SPI3
PB5 SPI3_MOSI SPI3

# Chip selects
PC15 IMU1_CS     CS
PB12 MAX7456_CS  CS
PA15 FLASH1_CS   CS

# ---------------------------------------------------------------
# SPI device table
# SPIDEV name bus devid cs mode lowspeed highspeed
# ---------------------------------------------------------------
SPIDEV icm42688   SPI1 DEVID1 IMU1_CS    MODE3  2*MHZ 16*MHZ
SPIDEV osd        SPI2 DEVID1 MAX7456_CS MODE0 10*MHZ 10*MHZ
SPIDEV dataflash  SPI3 DEVID1 FLASH1_CS  MODE3 32*MHZ 32*MHZ

# ---------------------------------------------------------------
# IMU
# ---------------------------------------------------------------
PB2 DRDY1_ICM42688 INPUT
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
define HAL_DEFAULT_INS_FAST_SAMPLE 1

# ---------------------------------------------------------------
# I2C (baro onboard, external compass on same bus)
# ---------------------------------------------------------------
PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1
I2C_ORDER I2C1

BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76

define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 0
define ALLOW_ARM_NO_COMPASS

# ---------------------------------------------------------------
# DroneCAN (CAN1 via FDCAN1)
# ---------------------------------------------------------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW GPIO(70)

# CAN_P1_DRIVER=1, CAN_D1_PROTOCOL=1 (DroneCAN) are set in defaults.parm
define HAL_CAN_DEFAULT_NODE_ID 0

# ---------------------------------------------------------------
# Motor / servo outputs
# ---------------------------------------------------------------
PB0  TIM3_CH3 TIM3 PWM(1) GPIO(50)
PB1  TIM3_CH4 TIM3 PWM(2) GPIO(51)
PA0  TIM5_CH1 TIM5 PWM(3) GPIO(52)
PA1  TIM5_CH2 TIM5 PWM(4) GPIO(53)
PA2  TIM5_CH3 TIM5 PWM(5) GPIO(54)
PA3  TIM5_CH4 TIM5 PWM(6) GPIO(55)
PD12 TIM4_CH1 TIM4 PWM(7) GPIO(56)
PD13 TIM4_CH2 TIM4 PWM(8) GPIO(57)

# LED strip (WS2812 / NeoPixel)
PA8  TIM1_CH1 TIM1 PWM(9) GPIO(58)

# Bidirectional DShot on motor timers
define HAL_WITH_BIDIR_DSHOT 1

# ---------------------------------------------------------------
# ADC - battery / current / RSSI
# ---------------------------------------------------------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC5 RSSI_IN           ADC1 SCALE(1)

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
# Calibrate the scales against the real divider and shunt

# ---------------------------------------------------------------
# LEDs / buzzer / misc GPIO
# ---------------------------------------------------------------
PE3 LED0 OUTPUT LOW GPIO(90)
PE4 LED1 OUTPUT LOW GPIO(91)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

PE5 ALARM OUTPUT LOW GPIO(32)
define HAL_BUZZER_PIN 32

# VTX / camera switch (optional)
PC13 PINIO1 OUTPUT GPIO(81) LOW
define RELAY1_PIN_DEFAULT 81

# ---------------------------------------------------------------
# DMA tuning (allocation itself is automatic)
# ---------------------------------------------------------------
DMA_PRIORITY SPI1* TIM3* TIM5* TIM4* USART6*
DMA_NOSHARE  SPI1* TIM3* TIM5*

# ---------------------------------------------------------------
# Logging (onboard SPI NOR flash, no SD card)
# ---------------------------------------------------------------
define HAL_LOGGING_DATAFLASH_ENABLED 1
define HAL_OS_FATFS_IO 0

# ---------------------------------------------------------------
# OSD
# ---------------------------------------------------------------
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ---------------------------------------------------------------
# Misc
# ---------------------------------------------------------------
define HAL_FRAME_TYPE_DEFAULT 1
define STM32_PWM_USE_ADVANCED TRUE
```

---

## `hwdef-bl.dat` (bootloader)

```
MCU STM32H7xx STM32H743xx
APJ_BOARD_ID TARGET_HW_NANOH743AIO
OSCILLATOR_HZ 8000000
FLASH_SIZE_KB 2048
FLASH_BOOTLOADER_LOAD_KB 128
FLASH_RESERVE_START_KB 0

PA11 OTG_FS_DM OTG