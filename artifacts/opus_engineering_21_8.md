# ArduPilot hwdef for an STM32H743VIT6 Nano AIO Flight Controller

This is a reference hardware definition for a typical 20×20 / 25.5×25.5 mm "nano AIO" H743 board. It includes:

- A single ICM-42688-P IMU
- A DPS310 barometer
- A MAX7456 OSD
- W25N01G NAND flash
- Seven UARTs
- Eight DShot outputs
- An LED strip output
- FDCAN1 for DroneCAN

> **Important:** The pin mapping is a coherent, conflict-checked reference design for the LQFP100 (VIT6) package, which has only ports A–E plus PH0/PH1. It is **not** the pinout of any specific commercial board. Verify every pin against your actual schematic before flashing. A wrong pin can short outputs or brick peripherals.

---

## A note on DMA for STM32H7

On the H7, DMA streams are routed through the **DMAMUX**. ArduPilot's `chibios_hwdef.py` allocates streams automatically. You don't hard-code stream or channel numbers as you would on F4/F7. Instead you control allocation with:

- `DMA_PRIORITY`: the order in which peripherals get dedicated streams.
- `DMA_NOSHARE`: peripherals that must never share a stream.

The generated `hwdef.h` (in `build/<board>/`) shows the final allocation. Inspect it after building. The expected allocation is listed later in this document.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```text
# hw definition file for NanoH743AIO
# STM32H743VIT6 (LQFP100, 2MB flash, 1MB RAM)
# ICM42688 / DPS310 / MAX7456 / W25N01G / FDCAN1 DroneCAN

# ---------------------------------------------------------------
# MCU / clock / board ID
# ---------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Board ID: request an official ID via a PR to Tools/AP_Bootloader/board_types.txt.
# 5600 is a placeholder; do NOT distribute firmware with it.
APJ_BOARD_ID 5600

# 8MHz crystal
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048

# Bootloader occupies the first 128k sector
FLASH_RESERVE_START_KB 128

# Parameter storage in internal flash (no FRAM on nano AIO)
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 32768

# Use the board-specific default params file
env DEFAULT_PARAMETERS 'libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/defaults.parm'

# ---------------------------------------------------------------
# Serial port order
# SERIAL0=USB, 1=USART1, 2=USART2, 3=USART3, 4=UART4,
# 5=USART6 (RCIN), 6=UART7, 7=UART8
# ---------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USB
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9  VBUS INPUT OPENDRAIN

# USART1 - MAVLink telem1
PB6 USART1_TX USART1
PB7 USART1_RX USART1

# USART2 - GPS
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - telem2 / ESC telemetry
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - VTX (MSP DisplayPort / SmartAudio)
PA0 UART4_TX UART4
PA1 UART4_RX UART4

# USART6 - RC input (full duplex, supports CRSF/ELRS/SBUS)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 - spare
PE8 UART7_TX UART7
PE7 UART7_RX UART7

# UART8 - spare
PE1 UART8_TX UART8
PE0 UART8_RX UART8

# RC input defaults to USART6
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_MSP_DisplayPort
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_ESCTelemetry

# No dedicated timer RCIN; RC is UART based
define HAL_SERIAL_RCIN 1

# ---------------------------------------------------------------
# SPI buses
# ---------------------------------------------------------------
# SPI1 - IMU (ICM42688)
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1

# SPI2 - OSD (MAX7456)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - dataflash (W25N01G)
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# Chip selects
PA4  IMU1_CS   CS
PB12 MAX7456_CS CS
PD10 FLASH1_CS CS

# IMU data-ready (optional, used for fast sampling)
PC13 IMU1_DRDY INPUT

# SPI device table
#        name       bus  devid  cs          mode   lowspeed   highspeed
SPIDEV   icm42688   SPI1 DEVID1 IMU1_CS     MODE3  2*MHZ      16*MHZ
SPIDEV   osd        SPI2 DEVID4 MAX7456_CS  MODE0  10*MHZ     10*MHZ
SPIDEV   dataflash  SPI3 DEVID1 FLASH1_CS   MODE3  32*MHZ     32*MHZ

# ---------------------------------------------------------------
# I2C
# ---------------------------------------------------------------
# I2C1 - internal baro + external compass on GPS connector
PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1

# I2C2 - external (spare pads)
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

I2C_ORDER I2C1 I2C2

# ---------------------------------------------------------------
# FDCAN1 - DroneCAN
# ---------------------------------------------------------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1

# Transceiver silent/standby pin (LOW = active)
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

# ---------------------------------------------------------------
# PWM / DShot outputs
# Group1: TIM1 (M1-M4), Group2: TIM3 (M5-M6), Group3: TIM4 (M7-M8)
# Keep motor groups on separate timers so DShot rates can differ per group
# ---------------------------------------------------------------
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50)
PE11 TIM1_CH2 TIM1 PWM(2) GPIO(51)
PE13 TIM1_CH3 TIM1 PWM(3) GPIO(52)
PE14 TIM1_CH4 TIM1 PWM(4) GPIO(53)

PB0  TIM3_CH3 TIM3 PWM(5) GPIO(54)
PB1  TIM3_CH4 TIM3 PWM(6) GPIO(55)

PD12 TIM4_CH1 TIM4 PWM(7) GPIO(56)
PD13 TIM4_CH2 TIM4 PWM(8) GPIO(57)

# WS2812 LED strip (serial LED / NeoPixel)
PE5  TIM15_CH1 TIM15 PWM(9) GPIO(58)

# Bidirectional DShot (RPM telemetry) on motor groups
DSHOT_MASK 0xFF

# Buzzer
PA2 TIM2_CH3 TIM2 GPIO(32) ALARM

# ---------------------------------------------------------------
# ADC
# ---------------------------------------------------------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC5 RSSI_IN           ADC1 SCALE(1)
PC4 PRESSURE_SENS     ADC1 SCALE(2)

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 8

# ---------------------------------------------------------------
# GPIO / LEDs / PINIO
# ---------------------------------------------------------------
PE3 LED0 OUTPUT LOW GPIO(90)
PE4 LED1 OUTPUT LOW GPIO(91)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

# VTX power switch (RELAY / PINIO)
PD11 PINIO1 OUTPUT GPIO(81) LOW
# Camera switch
PD14 PINIO2 OUTPUT GPIO(82) LOW
define RELAY1_PIN_DEFAULT 81
define RELAY2_PIN_DEFAULT 82

# ---------------------------------------------------------------
#