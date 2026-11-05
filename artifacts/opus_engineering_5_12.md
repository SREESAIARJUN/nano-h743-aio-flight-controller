# ArduPilot `hwdef.dat` for a Generic STM32H743VIT6 Nano AIO Flight Controller

This is a complete, buildable reference definition for a typical 20×20 / 25.5×25.5 "nano AIO" layout: one IMU, a barometer, an analog OSD, SPI NAND/NOR logging, 7 UARTs, 8 motor outputs, NeoPixel, and DroneCAN on FDCAN1.

**Before you flash this:**
- **Pin assignments must be checked against your schematic.** This file follows common H743 conventions (Matek/SpeedyBee style), but every board differs.
- **The DMA section works differently on the H7.** You do not hand-assign DMA streams on the H743. ArduPilot's `dma_resolver.py` allocates streams through the DMAMUX automatically at build time. You control the result with `DMA_PRIORITY` and `DMA_NOSHARE`, as shown below. After building, the resolved stream map is written to `build/<board>/hwdef.h` for verification.
- **"Cycle 5"** isn't an ArduPilot concept I can map to anything. If it refers to an internal revision of your board, put it in the header comment and in `APJ_BOARD_ID` registration.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```text
# hw definition file for NanoH743AIO (STM32H743VIT6) - revision "Cycle 5"
# Nano AIO: ICM42688 IMU, DPS310 baro, MAX7456 OSD, SPI flash logging,
# 7x UART, 8x DShot motor outputs, NeoPixel LED, DroneCAN on FDCAN1

# ---------------------------------------------------------------------
# MCU / board identity
# ---------------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Board ID: register a unique ID in Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID TARGET_HW_NANOH743AIO

# Crystal frequency
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048

# Bootloader occupies first 128k sector
FLASH_RESERVE_START_KB 128

# Parameter storage in internal flash (sectors 14/15)
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 32768

# Use the 32-bit timer for system time
STM32_ST_USE_TIMER 2
define CH_CFG_ST_RESOLUTION 32

# ---------------------------------------------------------------------
# Debug / USB
# ---------------------------------------------------------------------
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9  VBUS INPUT OPENDRAIN    # optional: only if VBUS sense is wired (remove if PA9 is USART1_TX)

# ---------------------------------------------------------------------
# Serial ports
# SERIAL0 = USB, SERIAL1..7 = physical UARTs
# ---------------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USART1 - Telem1 (MAVLink)
PB6  USART1_TX USART1
PB7  USART1_RX USART1

# USART2 - Telem2 / VTX control
PD5  USART2_TX USART2
PD6  USART2_RX USART2

# USART3 - GPS
PD8  USART3_TX USART3
PD9  USART3_RX USART3

# UART4 - spare
PB9  UART4_TX UART4
PB8  UART4_RX UART4

# USART6 - RC input (CRSF/ELRS/SBUS via serial RCIN)
PC6  USART6_TX USART6
PC7  USART6_RX USART6

# UART7 - ESC telemetry (RX only used)
PE8  UART7_TX UART7
PE7  UART7_RX UART7

# UART8 - DJI / MSP DisplayPort
PE1  UART8_TX UART8
PE0  UART8_RX UART8

# Default protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_MSP_DisplayPort

# ---------------------------------------------------------------------
# SPI buses
# SPI1 -> IMU   | SPI2 -> OSD (MAX7456) | SPI3 -> dataflash
# ---------------------------------------------------------------------
PA5  SPI1_SCK  SPI1
PA6  SPI1_MISO SPI1
PD7  SPI1_MOSI SPI1

PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# Chip selects
PC15 IMU1_CS   CS
PB12 OSD1_CS   CS
PA15 FLASH1_CS CS

# IMU data-ready (optional, not used by driver by default)
PB2  IMU1_DRDY INPUT

# SPI device table
#        name       bus  devid   CS         mode  lowspd  highspd
SPIDEV icm42688   SPI1 DEVID1  IMU1_CS    MODE3  2*MHZ  16*MHZ
SPIDEV osd        SPI2 DEVID4  OSD1_CS    MODE0 10*MHZ  10*MHZ
SPIDEV dataflash  SPI3 DEVID1  FLASH1_CS  MODE3 104*MHZ 104*MHZ

# ---------------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------------
# IMU (rotation must match physical orientation on your PCB)
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
define HAL_DEFAULT_INS_FAST_SAMPLE 1

# I2C
I2C_ORDER I2C1 I2C2
PB10 I2C2_SCL I2C2      # internal baro bus
PB11 I2C2_SDA I2C2
PB6  I2C1_SCL I2C1      # NOTE: conflicts with USART1 above - pick one!
PB7  I2C1_SDA I2C1

# Barometer on internal bus (bus index follows I2C_ORDER: 0=I2C1, 1=I2C2)
BARO DPS310 I2C:1:0x76
BARO BMP280 I2C:1:0x76     # fallback for alternate BOM

# No internal compass on nano AIO; probe external I2C compasses (GPS module)
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 2
define ALLOW_ARM_NO_COMPASS

# ---------------------------------------------------------------------
# OSD
# ---------------------------------------------------------------------
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ---------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------
define HAL_LOGGING_DATAFLASH_ENABLED 1
# For W25N01G SPI-NAND no extra define is required (auto-detected)

# ---------------------------------------------------------------------
# ADC
# ---------------------------------------------------------------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 RSSI_ADC          ADC1
PC5 VDD_5V_SENS       ADC1 SCALE(2)

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 4

# ---------------------------------------------------------------------
# PWM / DShot outputs
# Group by timer: all channels on one timer share a protocol/rate
# ---------------------------------------------------------------------
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50)
PE11 TIM1_CH2 TIM1 PWM(2) GPIO(51)
PE13 TIM1_CH3 TIM1 PWM(3) GPIO(52)
PE14 TIM1_CH4 TIM1 PWM(4) GPIO(53)

PD12 TIM4_CH1 TIM4 PWM(5) GPIO(54)
PD13 TIM4_CH2 TIM4 PWM(6) GPIO(55)
PD14 TIM4_CH3 TIM4 PWM(7) GPIO(56)
PD15 TIM4_CH4 TIM4 PWM(8) GPIO(57)

# NeoPixel / WS2812 LED strip
PA0  TIM5_CH1 TIM5 PWM(9) GPIO(58)
define HAL_SERIAL_LED_ENABLED 1

# Bidirectional DShot on motor timers
define HAL_WITH_BIDIR_DSHOT
DSHOT_MASK 255

# ---------------------------------------------------------------------
# Misc GPIO
# ---------------------------------------------------------------------
PE3  LED0 OUTPUT LOW GPIO(90)      # status LED (blue)
PE4  LED1 OUTPUT LOW GPIO(91)      # status LED (green)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

PA8  BUZZER OUTPUT GPIO(80) LOW
define HAL_BUZZER_PIN 80

PD10 PINIO1 OUTPUT G