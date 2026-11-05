# ArduPilot hwdef for a Generic STM32H743VIT6 Nano AIO Flight Controller

**This is a reference design, not a drop-in file for any specific commercial board.** A hwdef must match the real schematic pin-for-pin. Wrong pin assignments can produce a board that won't boot, a gyro that isn't detected, or motors that spin unexpectedly. Check every pin against your schematic before flashing, and bench-test with props off. I treated "Cycle 31" as a revision tag and put it in the header comments.

## Directory layout
```
libraries/AP_HAL_ChibiOS/hwdef/NanoAIO-H743/
├── hwdef.dat
├── hwdef-bl.dat
├── defaults.parm
└── README.md
```

---

## hwdef.dat

```
# hw definition file for NanoAIO-H743 (STM32H743VIT6, LQFP100)
# Revision: Cycle 31
# Reference design - verify every pin against the schematic

# ------------------------------------------------------------------
# MCU / clocks / board ID
# ------------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Board ID: request a unique ID via a PR to Tools/AP_Bootloader/board_types.txt
# Placeholder below - DO NOT ship with this value
APJ_BOARD_ID TARGET_HW_NANOAIO_H743

# 8 MHz HSE crystal (change to 16000000/24000000 if your crystal differs)
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048
# bootloader occupies the first 128k sector
FLASH_RESERVE_START_KB 128

# TIM2 reserved for the ChibiOS system tick
STM32_ST_USE_TIMER 2

# parameter storage in on-chip flash (no FRAM on board)
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 32768

# ------------------------------------------------------------------
# USB / SWD
# ------------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9  VBUS INPUT OPENDRAIN    # remove if USART1 uses PA9; see SERIAL map

PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ------------------------------------------------------------------
# Serial port order
#   SERIAL0 = USB (MAVLink)
#   SERIAL1 = USART1  (Telem1 / MAVLink)
#   SERIAL2 = USART2  (Telem2 / VTX control)
#   SERIAL3 = USART3  (GPS)
#   SERIAL4 = UART4   (spare / ESC telemetry)
#   SERIAL5 = USART6  (RC input - CRSF/ELRS/SBUS)
#   SERIAL6 = UART7   (spare / DJI)
#   SERIAL7 = UART8   (spare)
#   SERIAL8 = OTG2    (second USB CDC - SLCAN/MAVLink)
# ------------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8 OTG2

# USART1 (PA9 also used as VBUS sense above - pick ONE per your schematic)
PB14 USART1_TX USART1 NODMA    # alt mapping AF4 on H743 - verify; else PA9/PA10
PB15 USART1_RX USART1 NODMA

# USART2
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - GPS
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4
PB9 UART4_TX UART4 NODMA
PB8 UART4_RX UART4 NODMA

# USART6 - RC input (DMA recommended for CRSF)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7
PE8 UART7_TX UART7 NODMA
PE7 UART7_RX UART7 NODMA

# UART8
PE1 UART8_TX UART8 NODMA
PE0 UART8_RX UART8 NODMA

# ------------------------------------------------------------------
# SPI buses
#   SPI1 - IMU  (ICM-42688-P)
#   SPI2 - OSD  (AT7456E / MAX7456)
#   SPI3 - Dataflash (W25N01G / W25Q128)
# ------------------------------------------------------------------
# SPI1 - IMU
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1
PC4 IMU1_CS CS
PC5 IMU1_DRDY INPUT    # optional, not used by default driver

# SPI2 - OSD
PB13 SPI2_SCK  SPI2
PC2  SPI2_MISO SPI2
PC3  SPI2_MOSI SPI2
PB12 OSD1_CS CS

# SPI3 - Flash
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3
PA15 FLASH1_CS CS

# SPI device table
#        name      bus  devid  CS         mode  lowspeed  highspeed
SPIDEV imu1       SPI1 DEVID1 IMU1_CS    MODE3  2*MHZ   16*MHZ
SPIDEV osd        SPI2 DEVID4 OSD1_CS    MODE0 10*MHZ   10*MHZ
SPIDEV dataflash  SPI3 DEVID1 FLASH1_CS  MODE3 104*MHZ 104*MHZ

# ------------------------------------------------------------------
# I2C
#   I2C1 - internal baro
#   I2C2 - external GPS/compass connector
# ------------------------------------------------------------------
PB6  I2C1_SCL I2C1
PB7  I2C1_SDA I2C1
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

# external bus first so compass probing sees it as bus 0
I2C_ORDER I2C2 I2C1

# ------------------------------------------------------------------
# FDCAN1 - DroneCAN
# ------------------------------------------------------------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
# transceiver silent/standby pin (LOW = active)
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

# ------------------------------------------------------------------
# Motor / servo outputs (8 x DShot, bidirectional capable)
# Groups: TIM3 (1-4), TIM5 (5-6), TIM4 (7-8)
# ------------------------------------------------------------------
PB0  TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1  TIM3_CH4 TIM3 PWM(2) GPIO(51)
PB4  TIM3_CH1 TIM3 PWM(3) GPIO(52) BIDIR
PB5  TIM3_CH2 TIM3 PWM(4) GPIO(53)

PA0  TIM5_CH1 TIM5 PWM(5) GPIO(54) BIDIR
PA1  TIM5_CH2 TIM5 PWM(6) GPIO(55)

PD12 TIM4_CH1 TIM4 PWM(7) GPIO(56) BIDIR
PD13 TIM4_CH2 TIM4 PWM(8) GPIO(57)

# Serial LED (WS2812) on its own timer
PE14 TIM1_CH4 TIM1 PWM(9) GPIO(58)

# ------------------------------------------------------------------
# Buzzer (TIM15 so it doesn't steal a motor timer)
# ------------------------------------------------------------------
PE5 TIM15_CH1 TIM15 GPIO(77) ALARM

# ------------------------------------------------------------------
# Status LEDs
# ------------------------------------------------------------------
PE3 LED0 OUTPUT LOW GPIO(90)   # blue
PE4 LED1 OUTPUT LOW GPIO(91)   # green
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

# ------------------------------------------------------------------
# ADC - battery / current / RSSI
# ------------------------------------------------------------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC5 RSSI_ADC ADC1      # remove if PC5 is used as IMU DRDY

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10          # PC0 = ADC1_INP10
define HAL_BATT_CURR_PIN 11          # PC1 = ADC1_INP11
define HAL_BATT_VOLT_SCALE 11.0      # 10k:1k divider - calibrate!
define HAL_BATT_CURR_SCALE 40.0      # depends on ESC/shunt - calibrate!
define BOARD_RSSI_ANA_PIN 8          # PC5 = ADC1_INP8

# ------------------------------------------------------------------
# DMA configuration
# ChibiOS DMA is assigned automatically by the hwdef DMA resolver
# (Tools/scripts/... dma_resolver.py). Here we steer it:
# ------------------------------------------------------------------
# Highest priority: motor timers (DShot) and IMU SPI
DMA_PRIORITY TIM3* TIM5* TIM4* SPI1* USART6*

# Never share a stream with the IMU bus or RC input
DMA_NOSHARE SPI1* USART6* TIM3*

# ------------------------------------------------------------------
# Sensors
# ------------------------------------------------------------------
# IMU - ICM-42688-P; rotation must match physical orientation
IMU Invensensev3 SPI:imu1 ROTATION_YAW_180
define HAL_DEFAULT_