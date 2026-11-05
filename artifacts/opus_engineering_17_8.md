# ArduPilot hwdef for a Generic STM32H743VIT6 Nano AIO Board

I don't know of a specific board called "STM32H743VIT6 nano AIO," and "Cycle 17" doesn't match any ArduPilot convention I know of. Below is a complete, syntactically valid hwdef set for a plausible AIO design. Every pin assignment is a reasonable choice for the LQFP100 package, but it must be checked against your actual schematic before you flash it.

### How DMA works on the H7

You don't assign DMA streams by hand in ArduPilot on the H7. The STM32H7 has a DMAMUX, and `chibios_hwdef.py` allocates streams automatically at build time. You control allocation with three tools:

- **`DMA_PRIORITY`** chooses which peripherals get streams first.
- **`DMA_NOSHARE`** stops a peripheral's stream from being shared.
- **`NODMA`** on a pin line excludes that peripheral from DMA.

After a build, the actual stream map is written to `build/<board>/hwdef.h`. Check it there; hard-coding streams in the hwdef is not supported.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```
# hw definition file for generic STM32H743VIT6 nano AIO flight controller
# MCU: STM32H743VIT6 (LQFP100, 2MB flash, 1MB RAM)

MCU STM32H7xx STM32H743xx

# Board ID: TODO - request a unique ID via ArduPilot board_types.txt PR
APJ_BOARD_ID TARGET_HW_NANO_H743_AIO

# crystal frequency
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048
# bootloader occupies first 128KB sector
FLASH_RESERVE_START_KB 128

# parameter storage in flash sectors (no FRAM on AIO)
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 32768

# 1MHz system timer resolution
STM32_ST_USE_TIMER 2
define CH_CFG_ST_RESOLUTION 32

# ------------------- USB -------------------
USB_STRING_MANUFACTURER "ArduPilot"
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# ------------------- Serial ports -------------------
# SERIAL0 = USB, SERIAL1..7 below
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USART1 - telemetry 1 (MAVLink)
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - GPS
PD5  USART2_TX USART2
PD6  USART2_RX USART2

# USART3 - spare / VTX control
PD8  USART3_TX USART3 NODMA
PD9  USART3_RX USART3 NODMA

# UART4 - spare
PA0  UART4_TX UART4 NODMA
PA1  UART4_RX UART4 NODMA

# USART6 - RC input (serial RC: CRSF/ELRS/SBUS)
PC6  USART6_TX USART6
PC7  USART6_RX USART6

# UART7 - ESC telemetry (RX only used)
PE8  UART7_TX UART7 NODMA
PE7  UART7_RX UART7 NODMA

# UART8 - spare
PE1  UART8_TX UART8 NODMA
PE0  UART8_RX UART8 NODMA

# default protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL6_BAUD 115

# ------------------- SPI buses -------------------
# SPI1 - IMU
PA5  SPI1_SCK  SPI1
PA6  SPI1_MISO SPI1
PD7  SPI1_MOSI SPI1

# SPI2 - OSD (MAX7456 / AT7456E)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - dataflash
PB3  SPI3_SCK  SPI3
PB4  SPI3_MISO SPI3
PB5  SPI3_MOSI SPI3

# chip selects
PC15 GYRO1_CS  CS
PB12 OSD1_CS   CS
PD3  FLASH1_CS CS

# IMU data-ready (optional)
PB2  GYRO1_DRDY INPUT

# SPI device table
#       name       bus  devid  cs         mode  lowspeed  highspeed
SPIDEV icm42688   SPI1 DEVID1 GYRO1_CS   MODE3 2*MHZ     16*MHZ
SPIDEV osd        SPI2 DEVID4 OSD1_CS    MODE0 10*MHZ    10*MHZ
SPIDEV dataflash  SPI3 DEVID1 FLASH1_CS  MODE3 104*MHZ   104*MHZ

# ------------------- I2C -------------------
I2C_ORDER I2C1
PB8  I2C1_SCL I2C1
PB9  I2C1_SDA I2C1

# ------------------- Sensors -------------------
# IMU - set rotation to match PCB orientation
IMU Invensensev3 SPI:icm42688 ROTATION_NONE
define HAL_DEFAULT_INS_FAST_SAMPLE 1

# Barometer (on-board)
BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76

# Compass: external only (via GPS module)
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 0
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ------------------- Dataflash logging -------------------
define HAL_LOGGING_DATAFLASH_ENABLED 1

# ------------------- OSD -------------------
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ------------------- Motor / servo outputs -------------------
# TIM1: outputs 1-4 (bidirectional DShot capable)
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50) BIDIR
PE11 TIM1_CH2 TIM1 PWM(2) GPIO(51)
PE13 TIM1_CH3 TIM1 PWM(3) GPIO(52) BIDIR
PE14 TIM1_CH4 TIM1 PWM(4) GPIO(53)

# TIM4: outputs 5-8
PD12 TIM4_CH1 TIM4 PWM(5) GPIO(54) BIDIR
PD13 TIM4_CH2 TIM4 PWM(6) GPIO(55)
PD14 TIM4_CH3 TIM4 PWM(7) GPIO(56) BIDIR
PD15 TIM4_CH4 TIM4 PWM(8) GPIO(57)

# TIM2: WS2812 LED strip / NeoPixel (output 9)
PA15 TIM2_CH1 TIM2 PWM(9) GPIO(58)

# ------------------- DMA allocation control -------------------
# H7 DMAMUX streams are auto-allocated; this sets priority order
DMA_PRIORITY SPI1* TIM1* TIM4* USART6* USART2* SPI3*
DMA_NOSHARE  SPI1* TIM1* TIM4*

# ------------------- ADC -------------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 RSSI_IN           ADC1 SCALE(1)

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 4

# ------------------- LEDs / beeper -------------------
PE4 LED0 OUTPUT LOW GPIO(90)
PE5 LED1 OUTPUT LOW GPIO(91)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_ON  0

PE3 ALARM OUTPUT GPIO(32) LOW
define HAL_BUZZER_PIN 32
define HAL_BUZZ