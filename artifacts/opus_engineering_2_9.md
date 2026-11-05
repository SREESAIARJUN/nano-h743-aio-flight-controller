# ArduPilot hwdef.dat for an STM32H743VIT6 Nano AIO Flight Controller

## Assumptions

There is no single standard "nano AIO H743" board, so this file uses a **reference pinout** built from valid STM32H743 alternate functions. Before flashing:

- **Check every pin against your schematic.** This is especially important for the chip-selects, motor timer channels, and ADC pins.
- **Replace `APJ_BOARD_ID`** with an ID registered in `Tools/AP_Bootloader/board_types.txt`.
- **Interpreting "Cycle 2":** I treated it as hardware revision 2 of the board. It is reflected in the comments and in a `BOARD_REV` define.

Place the files in `libraries/AP_HAL_ChibiOS/hwdef/NanoAIO-H743/`.

---

## `hwdef.dat`

```
# =====================================================================
#  NanoAIO-H743  (Cycle 2 / Rev 2)
#  MCU: STM32H743VIT6 (LQFP100), 2MB flash, 1MB RAM
#  20x20 / 25.5x25.5 AIO: ICM42688P, SPL06/DPS310, AT7456E OSD,
#  W25N01G NAND log flash, 4-in-1 ESC header, DroneCAN on FDCAN1
# =====================================================================

# ---------------------------------------------------------------------
# MCU / clocks / board identity
# ---------------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# !! Replace with your registered board ID !!
APJ_BOARD_ID 5999

define BOARD_REV 2

# 8 MHz HSE crystal
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048

# 128 KB bootloader + 2x128 KB sectors used for parameter storage
FLASH_RESERVE_START_KB 384
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 32768

# Use TIM12 as the ChibiOS system tick timer (keeps TIM1-5/8 free for PWM)
STM32_ST_USE_TIMER 12
define CH_CFG_ST_RESOLUTION 16

# ---------------------------------------------------------------------
# USB
# ---------------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9  VBUS INPUT OPENDRAIN

USB_STRING_MANUFACTURER "ArduPilot"
USB_STRING_PRODUCT      "NanoAIO-H743"

# ---------------------------------------------------------------------
# SWD debug
# ---------------------------------------------------------------------
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------------------------------------------------------------
# Status LEDs / beeper
# ---------------------------------------------------------------------
PE3 LED0 OUTPUT LOW GPIO(90)    # blue
PE4 LED1 OUTPUT LOW GPIO(91)    # green
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

PD15 BUZZER OUTPUT GPIO(80) LOW
define HAL_BUZZER_PIN 80
define HAL_BUZZER_ON 1
define HAL_BUZZER_OFF 0

# ---------------------------------------------------------------------
# SPI buses
#   SPI1 : IMU (ICM42688P)         - dedicated, high priority DMA
#   SPI2 : OSD (AT7456E)
#   SPI3 : NAND log flash (W25N01G)
# ---------------------------------------------------------------------
# SPI1 - IMU
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1

# SPI2 - OSD
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - Flash
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# Chip selects (all idle HIGH)
PC15 IMU1_CS   CS
PB12 OSD1_CS   CS
PA15 FLASH1_CS CS

# IMU data-ready (optional, used for timing diagnostics)
PB2 IMU1_DRDY INPUT

# SPI device table
#        name       bus  devid  cs          mode   low     high
SPIDEV icm42688   SPI1 DEVID1 IMU1_CS   MODE3  2*MHZ  16*MHZ
SPIDEV bmi270     SPI1 DEVID1 IMU1_CS   MODE3  1*MHZ  10*MHZ
SPIDEV osd        SPI2 DEVID4 OSD1_CS   MODE0 10*MHZ  10*MHZ
SPIDEV dataflash  SPI3 DEVID1 FLASH1_CS MODE3 104*MHZ 104*MHZ

# ---------------------------------------------------------------------
# IMU (probe ICM42688 first, BMI270 as alternate population)
# ---------------------------------------------------------------------
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
IMU BMI270       SPI:bmi270   ROTATION_YAW_180

define HAL_DEFAULT_INS_FAST_SAMPLE 1

# ---------------------------------------------------------------------
# I2C (baro + external compass)
# ---------------------------------------------------------------------
PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1
I2C_ORDER I2C1

BARO SPL06  I2C:0:0x76
BARO DPS310 I2C:0:0x76

# No onboard compass; probe external compasses on the I2C bus
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 0
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ---------------------------------------------------------------------
# UARTs
# ---------------------------------------------------------------------
#             SERIAL0 SERIAL1 SERIAL2 SERIAL3 SERIAL4 SERIAL5 SERIAL6 SERIAL7
SERIAL_ORDER  OTG1    USART1  USART2  USART3  UART4   USART6  UART7   UART8

# USART1 - Telem1 / MAVLink
PB6 USART1_TX USART1
PB7 USART1_RX USART1

# USART2 - GPS
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - VTX (MSP DisplayPort / SmartAudio)
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - spare
PA0 UART4_TX UART4
PA1 UART4_RX UART4

# USART6 - RC input (CRSF/ELRS/SBUS)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 - ESC telemetry (RX only used)
PE8 UART7_TX UART7
PE7 UART7_RX UART7

# UART8 - spare / external receiver
PE1 UART8_TX UART8
PE0 UART8_RX UART8

# Disable DMA on low-rate ports to conserve streams
NODMA UART4 UART7 UART8

# ---------------------------------------------------------------------
# RC input: serial RC on USART6 (SERIAL5)
# ---------------------------------------------------------------------
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN

# ---------------------------------------------------------------------
# Motor / servo outputs
#   M1-M4 : TIM1 (4-in-1 ESC connector, bi-dir DShot capable)
#   M5-M8 : TIM3
#   LED   : TIM4_CH1 (WS2812 / NeoPixel)
# ---------------------------------------------------------------------
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50) BIDIR
PE11 TIM1_CH2 TIM1 PWM(2) GPIO(51)
PE13 TIM1_CH3 TIM1 PWM(3) GPIO(52) BIDIR
PE14 TIM1_CH4 TIM1 PWM(4) GPIO(53)

PB4 TIM3_CH1 TIM3 PWM(5) GPIO(54) BIDIR
PB5 TIM3_CH2 TIM3 PWM(6) GPIO(55)
PB0 TIM3_CH3 TIM3 PWM(7) GPIO(56) BIDIR
PB1 TIM3_CH4 TIM3 PWM(8) GPIO(57)

PD12 TIM4_CH1 TIM4 PWM