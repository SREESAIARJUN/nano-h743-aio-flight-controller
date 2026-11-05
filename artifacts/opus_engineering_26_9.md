# ArduPilot `hwdef.dat` for an STM32H743VIT6 Nano AIO

This is a generic reference design. No specific board was named, so the pin assignments below are a self-consistent example. Check every pin against your actual schematic before flashing. I couldn't tell what "Cycle 26" refers to, so it isn't reflected here.

Two ArduPilot notes before the files:

- **DMA streams are not assigned by hand.** ChibiOS/ArduPilot's `dma_resolver.py` allocates them at build time. You control the result with `DMA_PRIORITY` and `DMA_NOSHARE`. To see the final stream map, check `build/<board>/hwdef.h` after `./waf configure --board <board>`.
- **The VIT6 is an LQFP100 package.** Only ports A–E (plus PH0/PH1) exist, so all pins below are limited to those ports.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```text
# ============================================================
# NanoH743AIO - STM32H743VIT6 nano AIO flight controller
# ============================================================

# ---------------- MCU / clock / memory ----------------
MCU STM32H7xx STM32H743xx

# Placeholder ID: register a unique one in Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID AP_HW_NanoH743AIO

OSCILLATOR_HZ 8000000
FLASH_SIZE_KB 2048

# 128KB bootloader sector
FLASH_RESERVE_START_KB 128

# Parameter storage in internal flash (sectors 14/15)
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 32768

# Required for STM32H7 + DShot/FDCAN performance
env OPTIMIZE -O2

# ---------------- USB ----------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# ---------------- SWD ----------------
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------- Serial ports ----------------
#            SERIAL0 SERIAL1 SERIAL2 SERIAL3 SERIAL4 SERIAL5 SERIAL6 SERIAL7 SERIAL8
SERIAL_ORDER OTG1    USART1  USART2  USART3  UART4   USART6  UART7   UART8   OTG2

# USART1 - SERIAL1 (MAVLink telemetry)
PB6 USART1_TX USART1
PB7 USART1_RX USART1

# USART2 - SERIAL2 (GPS)
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - SERIAL3 (VTX / MSP DisplayPort)
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - SERIAL4 (spare / ESC telemetry)
PA0 UART4_TX UART4
PA1 UART4_RX UART4

# USART6 - SERIAL5 (RC input: CRSF/ELRS/SBUS)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 - SERIAL6 (spare)
PE8 UART7_TX UART7
PE7 UART7_RX UART7

# UART8 - SERIAL7 (spare)
PE1 UART8_TX UART8
PE0 UART8_RX UART8

# Serial protocol defaults
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_MSP_DisplayPort
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN

# Serial ports that don't need DMA (saves streams for SPI/TIM)
NODMA UART7 UART8

# ---------------- SPI buses ----------------
# SPI1 - IMU (ICM-42688-P)
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1
PA4 IMU1_CS   CS

# SPI2 - OSD (AT7456E / MAX7456)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2
PB12 OSD1_CS   CS

# SPI3 - Dataflash (W25Q128)
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3
PA15 FLASH1_CS CS

# SPI device table
#      name      bus  devid  cs         mode   lowspeed  highspeed
SPIDEV imu1      SPI1 DEVID1 IMU1_CS    MODE3  2*MHZ     16*MHZ
SPIDEV osd       SPI2 DEVID4 OSD1_CS    MODE0  10*MHZ    10*MHZ
SPIDEV dataflash SPI3 DEVID1 FLASH1_CS  MODE3  104*MHZ   104*MHZ

# IMU interrupt line (optional, for data-ready)
PC15 IMU1_INT INPUT

# ---------------- I2C ----------------
I2C_ORDER I2C1
PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1

# ---------------- Sensors ----------------
# IMU - adjust rotation to match board orientation
IMU Invensensev3 SPI:imu1 ROTATION_YAW_180
define HAL_DEFAULT_INS_FAST_SAMPLE 1

# Barometer (probe both common AIO parts)
BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76

# No onboard compass; probe external I2C compasses
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define ALLOW_ARM_NO_COMPASS
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ---------------- Motors (bidirectional DShot capable) ----------------
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50) BIDIR
PE11 TIM1_CH2 TIM1 PWM(2) GPIO(51)
PE13 TIM1_CH3 TIM1 PWM(3) GPIO(52) BIDIR
PE14 TIM1_CH4 TIM1 PWM(4) GPIO(53)

# Extra outputs (octo / servos)
PB0  TIM3_CH3 TIM3 PWM(5) GPIO(54) BIDIR
PB1  TIM3_CH4 TIM3 PWM(6) GPIO(55)
PB4  TIM3_CH1 TIM3 PWM(7) GPIO(56) BIDIR
PB5  TIM3_CH2 TIM3 PWM(8) GPIO(57)

# Addressable LED strip (WS2812) on its own timer
PD12 TIM4_CH1 TIM4 PWM(9) GPIO(58)

# ---------------- ADC ----------------
# PC0 = ADC123_INP10, PC1 = ADC123_INP11, PC4 = ADC12_INP4
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 RSSI_ADC          ADC1 SCALE(1)

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 4

# ---------------- LEDs / buzzer / misc GPIO ----------------
PE3  LED0 OUTPUT LOW GPIO(90)
PE4  LED1 OUTPUT LOW GPIO(91)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

PD15 BUZZER O