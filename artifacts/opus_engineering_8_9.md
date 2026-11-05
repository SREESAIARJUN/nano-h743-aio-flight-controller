# ArduPilot Hardware Definition: STM32H743VIT6 Nano AIO Flight Controller

The "Nano AIO" here is a generic reference board, not a specific commercial product. This is a coherent, conflict-free reference design that follows current ArduPilot `libraries/AP_HAL_ChibiOS/hwdef` conventions. Before flashing to real hardware, check every pin against your schematic.

**How DMA works on the H7:** the STM32H7 uses a DMAMUX. ArduPilot does **not** take hand-written stream/channel numbers for H7 boards. `chibios_hwdef.py` resolves DMA automatically at build time. You control it with `DMA_PRIORITY`, `DMA_NOSHARE`, and per-pin `NODMA`. The resolved map is written to `build/<board>/hwdef.h`. An expected allocation table is included in section 4.

Files go in `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/`.

---

## 1. `hwdef.dat`

```
# hw definition file for NanoH743AIO (STM32H743VIT6, LQFP100)
# Cycle 8 revision

# ---------------------------------------------------------------------
# MCU / board identity
# ---------------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Board ID: register in Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID AP_HW_NanoH743AIO

# 8 MHz crystal
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048

# bootloader occupies first 128k sector
FLASH_RESERVE_START_KB 128

# parameter storage in flash (sectors 1-2 of bank 1)
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 15360

# ---------------------------------------------------------------------
# USB
# ---------------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

USB_STRING_MANUFACTURER "ArduPilot"
USB_STRING_PRODUCT "NanoH743AIO"

# ---------------------------------------------------------------------
# SWD debug
# ---------------------------------------------------------------------
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------------------------------------------------------------
# Serial ports
# SERIAL0 = USB, SERIAL1..7 = UARTs, SERIAL8 = USB2 (SLCAN/MAVLink)
# ---------------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8 OTG2

# USART1 - SERIAL1 - MAVLink telem
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - SERIAL2 - GPS
PD5  USART2_TX USART2
PD6  USART2_RX USART2

# USART3 - SERIAL3 - MSP DisplayPort / VTX
PD8  USART3_TX USART3
PD9  USART3_RX USART3

# UART4 - SERIAL4 - ESC telemetry (RX only needed)
PA0  UART4_TX UART4 NODMA
PA1  UART4_RX UART4 NODMA

# USART6 - SERIAL5 - RC input (CRSF/ELRS/SBUS)
PC6  USART6_TX USART6
PC7  USART6_RX USART6

# UART7 - SERIAL6 - user
PE8  UART7_TX UART7 NODMA
PE7  UART7_RX UART7 NODMA

# UART8 - SERIAL7 - user / SmartAudio
PE1  UART8_TX UART8 NODMA
PE0  UART8_RX UART8 NODMA

# default protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_MSP_DisplayPort
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_None
define DEFAULT_SERIAL4_BAUD 115

# ---------------------------------------------------------------------
# SPI buses
# ---------------------------------------------------------------------
# SPI1 - IMU
PA5  SPI1_SCK  SPI1
PA6  SPI1_MISO SPI1
PA7  SPI1_MOSI SPI1
PA4  IMU1_CS   CS
PC4  IMU1_DRDY INPUT

# SPI2 - OSD (AT7456E)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2
PB12 OSD1_CS   CS

# SPI3 - Flash (W25N01G NAND, 128MB)
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3
PA15 FLASH1_CS CS

# ---------------------------------------------------------------------
# SPI device table
# SPIDEV name   bus  devid  cs        mode  lowspeed  highspeed
# ---------------------------------------------------------------------
SPIDEV icm42688  SPI1 DEVID1 IMU1_CS   MODE3  2*MHZ  16*MHZ
SPIDEV osd       SPI2 DEVID4 OSD1_CS   MODE0 10*MHZ  10*MHZ
SPIDEV dataflash SPI3 DEVID1 FLASH1_CS MODE3 32*MHZ  32*MHZ

# ---------------------------------------------------------------------
# I2C
# ---------------------------------------------------------------------
PB8  I2C1_SCL I2C1
PB9  I2C1_SDA I2C1

I2C_ORDER I2C1

# ---------------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------------
# IMU - ICM-42688-P
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
define HAL_DEFAULT_INS_FAST_SAMPLE 1

# Baro - DPS310 (alt: BMP280 at 0x76)
BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76

# no onboard compass; probe external on I2C/GPS
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 0
define ALLOW_ARM_NO_COMPASS

# ---------------------------------------------------------------------
# Motor / PWM outputs
# group 1: TIM1 (M1-M4, bidirectional DShot)
# group 2: TIM4 (M5-M6)
# group 3: TIM2 (WS2812 LED)
# ---------------------------------------------------------------------
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50) BIDIR
PE11 TIM1_CH2 TIM1 PWM(2) GPIO(51)
PE13 TIM1_CH3 TIM1 PWM(3) GPIO(52) BIDIR
PE14 TIM1_CH4 TIM1 PWM(4) GPIO(53)

PD12 TIM4_CH1 TIM4 PWM(5) GPIO(54) BIDIR
PD13 TIM4_CH2 TIM4 PWM(6) GPIO(55)

# serial LED strip
PB3  TIM2_CH2 TIM2 PWM(7) GPIO(56)

define HAL_WITH_DSP 1
define DEFAULT_SERVO_PROTOCOL 0

# ---------------------------------------------------------------------
# Buzzer
# ---------------------------------------------------------------------
PE5  TIM15_CH1 TIM15 GPIO(32) ALARM

# ---------------------------------------------------------------------
# ADC
# ---------------------------------------------------------------------
PC0  BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1  BATT_CURRENT_SENS ADC1 SCALE(1)
PC2  RSSI_IN           ADC1 SCALE(1)

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 12

# ---------------------------------------------------------------------
# LEDs / GPIO
# ---------------------------------------------------------------------
PE3  LED0 OUTPUT LOW GPIO(90)   # blue
PE4  LED1 OUTPUT LOW GPIO(91)   # green
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

# VTX / camera power switch (PINIO)
PD10 PINIO1 OUTPUT GPIO(81) LOW
define RELAY1_PIN