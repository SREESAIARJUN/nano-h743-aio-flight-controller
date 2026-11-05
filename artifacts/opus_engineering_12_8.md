# ArduPilot hwdef.dat for an STM32H743VIT6 Nano AIO Flight Controller

**Important:** This is a reference design, not a drop-in file for a specific commercial board. The STM32H743**VIT6** is the LQFP-100 package, so only ports A–E are used, and every pin/alternate-function pairing below is valid for that package. However, you must verify each pin against your actual schematic before flashing.

I don't have context for "Cycle 12," so I've treated it as a revision label in the header.

---

## Directory Layout

```
libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
├── hwdef.dat
├── hwdef-bl.dat
└── defaults.parm
```

---

## hwdef.dat

```
# =====================================================================
#  hw definition file for NanoH743AIO (STM32H743VIT6, LQFP100)
#  Revision: Cycle 12
#  Features: dual IMU (ICM42688 + BMI270), DPS310 baro, AT7456E OSD,
#            W25N01G dataflash, 8 motor outputs, LED strip, DroneCAN
# =====================================================================

# ---------------------------------------------------------------------
# MCU
# ---------------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Board ID - placeholder; request a unique ID via ArduPilot
# (Tools/AP_Bootloader/board_types.txt) before distributing firmware
APJ_BOARD_ID 5600

# crystal frequency
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048
# first 128k sector reserved for bootloader
FLASH_RESERVE_START_KB 128

# use TIM2 as the system timer (32-bit)
STM32_ST_USE_TIMER 2

# ---------------------------------------------------------------------
# Parameter storage in internal flash (no FRAM on nano AIO)
# ---------------------------------------------------------------------
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 16384

# ---------------------------------------------------------------------
# USB
# ---------------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

USB_STRING_MANUFACTURER "ArduPilot"
USB_STRING_PRODUCT      "NanoH743AIO"

# ---------------------------------------------------------------------
# SWD debug
# ---------------------------------------------------------------------
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------------------------------------------------------------
# Serial ports
# SERIAL0=USB, SERIAL1..7 map to the UARTs below
# ---------------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USART1 - SERIAL1 (telemetry / MAVLink)
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - SERIAL2 (GPS)
PD5  USART2_TX USART2
PD6  USART2_RX USART2

# USART3 - SERIAL3 (VTX / MSP DisplayPort)
PD8  USART3_TX USART3
PD9  USART3_RX USART3

# UART4 - SERIAL4 (spare)
PD0  UART4_RX UART4
PD1  UART4_TX UART4

# USART6 - SERIAL5 (RC input: CRSF/ELRS/SBUS)
PC6  USART6_TX USART6
PC7  USART6_RX USART6

# UART7 - SERIAL6 (ESC telemetry, RX only used)
PE7  UART7_RX UART7
PE8  UART7_TX UART7

# UART8 - SERIAL7 (spare)
PE0  UART8_RX UART8
PE1  UART8_TX UART8

# No IOMCU on this board
define HAL_WITH_IO_MCU 0

# ---------------------------------------------------------------------
# SPI buses
# ---------------------------------------------------------------------
# SPI1 - IMU1 (ICM42688)
PA5  SPI1_SCK  SPI1
PA6  SPI1_MISO SPI1
PA7  SPI1_MOSI SPI1

# SPI2 - OSD (AT7456E)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - Dataflash (W25N01G)
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# SPI4 - IMU2 (BMI270)
PE12 SPI4_SCK  SPI4
PE13 SPI4_MISO SPI4
PE14 SPI4_MOSI SPI4

# Chip selects
PC15 IMU1_CS  CS
PE11 IMU2_CS  CS
PB12 OSD_CS   CS
PA15 FLASH_CS CS

# IMU data-ready interrupts (optional, used for fast sampling)
PB2  IMU1_DRDY INPUT
PE15 IMU2_DRDY INPUT

# ---------------------------------------------------------------------
# SPI device table
#        name       bus  devid  cs        mode  lowspeed  highspeed
# ---------------------------------------------------------------------
SPIDEV icm42688   SPI1 DEVID1 IMU1_CS  MODE3   2*MHZ   16*MHZ
SPIDEV bmi270     SPI4 DEVID1 IMU2_CS  MODE3   1*MHZ   10*MHZ
SPIDEV osd        SPI2 DEVID1 OSD_CS   MODE0  10*MHZ   10*MHZ
SPIDEV dataflash  SPI3 DEVID1 FLASH_CS MODE3 104*MHZ  104*MHZ

# ---------------------------------------------------------------------
# IMUs (rotations must be verified against board orientation)
# ---------------------------------------------------------------------
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
IMU BMI270       SPI:bmi270   ROTATION_PITCH_180_YAW_90

define HAL_DEFAULT_INS_FAST_SAMPLE 3

# ---------------------------------------------------------------------
# I2C
# ---------------------------------------------------------------------
I2C_ORDER I2C2

PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

# Barometer
BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76

# Compass: none onboard, probe external I2C compasses
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define ALLOW_ARM_NO_COMPASS
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ---------------------------------------------------------------------
# DroneCAN (FDCAN1)
# ---------------------------------------------------------------------
PB8  CAN1_RX CAN1
PB9  CAN1_TX CAN1

# CAN transceiver silent/standby pin (drive low = active)
PD3  GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW GPIO(70)

# Enable CAN driver 1 with DroneCAN protocol by default
define HAL_CAN_DRIVER_DEFAULT 1
define HAL_ENABLE_DRONECAN_DRIVERS 1

# ---------------------------------------------------------------------
# Motor / servo outputs (bidirectional DShot capable)
# Keep motors in groups sharing a timer: TIM3 | TIM5 | TIM4
# ---------------------------------------------------------------------
PB0  TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1  TIM3_CH4 TIM3 PWM(2) GPIO(51)
PA0  TIM5_CH1 TIM5 PWM(3) GPIO(52) BIDIR
PA1  TIM5_CH2 TIM5 PWM(4) GPIO(53)
PA2  TIM5_CH3 TIM5 PWM(5) GPIO(54) BIDIR
PA3  TIM5_CH4 TIM5 PWM(6) GPIO(55)
PD12 TIM4_CH1 TIM4 PWM(7) GPIO(56) BIDIR
PD13 TIM4_CH2 TIM4 PWM(8) GPIO(57)

# LED strip (NeoPixel) on its own timer
PA8  TIM1_CH1 TIM1 PWM(9) GPIO(58)

# Buzzer
PE5  TIM15_CH1 TIM15 GPIO(32) ALARM

# ---------------------------------------------------------------------
# ADC
# ---------------------------------------------------------------------
PC0  BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1  BATT_CURRENT_SENS ADC1 SCALE(1)
PC4  RSSI_ADC_PIN      ADC1 SCALE(1)

define HAL_BATT_MON