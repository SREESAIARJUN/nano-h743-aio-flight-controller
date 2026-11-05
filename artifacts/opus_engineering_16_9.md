# ArduPilot `hwdef.dat` for a Generic STM32H743VIT6 Nano AIO

**Before you flash, read this.** No single standard "nano AIO" board exists. The pinout below is a coherent, internally consistent reference design. Every pin uses a valid alternate function for the STM32H743VIT6 (LQFP100, ports A–E). You must still check every line against your board's schematic. A wrong CS, timer, or ADC pin will not brick the MCU, but sensors, motors, or battery monitoring will fail. I have treated "Cycle 16" as a revision label, since it has no meaning in ArduPilot's hwdef format.

Two clarifications about your request:

- **DMA channels:** On H7 parts, ArduPilot allocates DMA streams automatically through the DMAMUX. You don't hard-code channel numbers. You steer allocation with `DMA_PRIORITY`, `DMA_NOSHARE`, and `NODMA`. The resolved map is written to `build/<board>/hwdef.h` at build time.
- **DroneCAN:** This is configured through the FDCAN pins plus `defaults.parm`, shown below.

Place these files in `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/`.

---

## 1. `hwdef.dat`

```
# hw definition file for NanoH743AIO (STM32H743VIT6, LQFP100)
# Rev: Cycle 16

# ------------------------------------------------------------------
# MCU / clock / flash
# ------------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Placeholder ID - obtain a unique ID from ArduPilot (board_types.txt)
APJ_BOARD_ID 9999

OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048
FLASH_RESERVE_START_KB 128

# Parameter storage in internal flash (sectors 14/15)
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 32768

# System timer
STM32_ST_USE_TIMER 2

# ------------------------------------------------------------------
# USB
# ------------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
USB_STRING_MANUFACTURER "ArduPilot"
USB_STRING_PRODUCT "NanoH743AIO"

# ------------------------------------------------------------------
# Debug (SWD)
# ------------------------------------------------------------------
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ------------------------------------------------------------------
# SPI buses
# ------------------------------------------------------------------
# SPI1 - IMU1 (ICM-42688-P)
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1

# SPI2 - OSD (AT7456E)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - Dataflash (W25N01G / W25Q128)
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PB5  SPI3_MOSI SPI3

# SPI4 - IMU2 (BMI270)
PE2 SPI4_SCK  SPI4
PE5 SPI4_MISO SPI4
PE6 SPI4_MOSI SPI4

# Chip selects
PA4  GYRO1_CS     CS
PE4  GYRO2_CS     CS
PB12 OSD1_CS      CS
PA15 FLASH1_CS    CS

# SPI device table
#        name       bus   devid  CS          mode   lowspeed  highspeed
SPIDEV imu1      SPI1  DEVID1  GYRO1_CS    MODE3  2*MHZ    16*MHZ
SPIDEV imu2      SPI4  DEVID1  GYRO2_CS    MODE3  1*MHZ    10*MHZ
SPIDEV osd       SPI2  DEVID4  OSD1_CS     MODE0  10*MHZ   10*MHZ
SPIDEV dataflash SPI3  DEVID1  FLASH1_CS   MODE3  104*MHZ  104*MHZ

# ------------------------------------------------------------------
# IMUs (adjust ROTATION_* to match physical orientation)
# ------------------------------------------------------------------
IMU Invensensev3 SPI:imu1 ROTATION_NONE
IMU BMI270       SPI:imu2 ROTATION_NONE
define HAL_DEFAULT_INS_FAST_SAMPLE 3

# IMU data-ready interrupt (optional)
PE15 GYRO1_DRDY INPUT

# ------------------------------------------------------------------
# I2C - baro + external compass
# ------------------------------------------------------------------
PB6 I2C1_SCL I2C1
PB7 I2C1_SDA I2C1
I2C_ORDER I2C1

BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76

define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 0
define ALLOW_ARM_NO_COMPASS
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ------------------------------------------------------------------
# UARTs
# ------------------------------------------------------------------
#             SERIAL0 SERIAL1 SERIAL2 SERIAL3 SERIAL4 SERIAL5 SERIAL6 SERIAL7
SERIAL_ORDER OTG1    USART1  USART2  USART3  UART4   USART6  UART7   UART8

# USART1 - SERIAL1 (Telem / MAVLink)
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - SERIAL2 (VTX / MSP DisplayPort)
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - SERIAL3 (GPS)
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - SERIAL4 (ESC telemetry - RX only meaningful)
PB9 UART4_TX UART4
PB8 UART4_RX UART4

# USART6 - SERIAL5 (RC input: CRSF/ELRS)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 - SERIAL6 (spare)
PE8 UART7_TX UART7 NODMA
PE7 UART7_RX UART7 NODMA

# UART8 - SERIAL7 (spare)
PE1 UART8_TX UART8 NODMA
PE0 UART8_RX UART8 NODMA

# ------------------------------------------------------------------
# RC input (timer capture - alternative to UART RC)
# ------------------------------------------------------------------
PB4 TIM3_CH1 TIM3 RCININT PULLDOWN LOW

# ------------------------------------------------------------------
# DroneCAN (FDCAN1)
# ------------------------------------------------------------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

# ------------------------------------------------------------------
# Motor / servo outputs (DShot + bidirectional DShot)
# ------------------------------------------------------------------
# Group 1 - TIM1
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50) BIDIR
PE11 TIM1_CH2 TIM1 PWM(2) GPIO(51)
PE13 TIM1_CH3 TIM1 