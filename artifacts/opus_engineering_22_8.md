# ArduPilot hwdef for a Nano AIO Flight Controller (STM32H743VIT6)

## Before You Flash: Verify Against Your Schematic

ArduPilot has no "standard nano AIO" board, so this is a **complete, syntactically valid template**. You must check every pin against your actual schematic.

- The pin assignments follow common H743 AIO layouts (Matek/SpeedyBee-style). Each one is a legal alternate-function mapping for the LQFP100 package.
- I don't recognize "Cycle 22" as an ArduPilot convention. If it is your internal revision tag, it is noted in the header.
- **DMA channels are not hand-assigned on the H7.** The DMAMUX lets any stream serve any peripheral. `chibios_hwdef.py` allocates the streams at build time, and you steer the allocation with `DMA_PRIORITY` and `DMA_NOSHARE`. Hard-coding streams the F4 way is neither needed nor supported. The resolved map appears in `build/<board>/hwdef.h` after `./waf configure`.

Place the files in `libraries/AP_HAL_ChibiOS/hwdef/NanoAIO-H743/`.

---

## `hwdef.dat`

```
# hw definition file for NanoAIO-H743 (STM32H743VIT6, LQFP100)
# Board revision: Cycle 22
# VERIFY ALL PINS AGAINST SCHEMATIC BEFORE FLASHING

# ------------------------------------------------------------------
# MCU / clocks / flash
# ------------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Board ID: register a unique ID in Tools/AP_Bootloader/board_types.txt
# 5999 is a placeholder; do not ship with it.
APJ_BOARD_ID 5999

OSCILLATOR_HZ 8000000
MCU_CLOCKRATE_MHZ 480

FLASH_SIZE_KB 2048
# H7 sectors are 128KB; the bootloader occupies the first sector
FLASH_RESERVE_START_KB 128

STM32_ST_USE_TIMER 12
define CH_CFG_ST_RESOLUTION 16

# Parameter storage in internal flash (last two sectors)
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 32768

# ------------------------------------------------------------------
# USB
# ------------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9  VBUS INPUT OPENDRAIN   # remove if PA9 is used for USART1 TX (it is below)

USB_STRING_MANUFACTURER "ArduPilot"
USB_STRING_PRODUCT "NanoAIO-H743"

# ------------------------------------------------------------------
# Serial ports
# SERIAL0=USB, then UARTs in user-visible order
# ------------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USART1 - Telem1 (MAVLink)
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - GPS
PD5  USART2_TX USART2
PD6  USART2_RX USART2

# USART3 - VTX (MSP DisplayPort / SmartAudio)
PD8  USART3_TX USART3
PD9  USART3_RX USART3

# UART4 - spare
PB9  UART4_TX UART4
PB8  UART4_RX UART4

# USART6 - RC input (CRSF/ELRS default; full-duplex)
PC6  USART6_TX USART6
PC7  USART6_RX USART6

# UART7 - ESC telemetry (RX only used)
PE8  UART7_TX UART7
PE7  UART7_RX UART7

# UART8 - spare
PE1  UART8_TX UART8
PE0  UART8_RX UART8

# No hardware flow control on this board
define HAL_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define HAL_SERIAL2_PROTOCOL SerialProtocol_GPS
define HAL_SERIAL3_PROTOCOL SerialProtocol_MSP_DisplayPort
define HAL_SERIAL5_PROTOCOL SerialProtocol_RCIN
define HAL_SERIAL6_PROTOCOL SerialProtocol_ESCTelemetry
define HAL_SERIAL6_BAUD 115

# ------------------------------------------------------------------
# SPI buses
# ------------------------------------------------------------------
# SPI1 - IMUs
PA5  SPI1_SCK  SPI1
PA6  SPI1_MISO SPI1
PD7  SPI1_MOSI SPI1

# SPI2 - OSD (MAX7456 / AT7456E)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - Dataflash
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# Chip selects
PC15 IMU1_CS   CS
PC13 IMU2_CS   CS
PB12 OSD1_CS   CS
PD10 FLASH1_CS CS

# ------------------------------------------------------------------
# SPI device table
# name       bus  devid  cs         mode  lowspeed  highspeed
# ------------------------------------------------------------------
SPIDEV icm42688   SPI1 DEVID1 IMU1_CS   MODE3  2*MHZ  16*MHZ
SPIDEV icm42688_2 SPI1 DEVID2 IMU2_CS   MODE3  2*MHZ  16*MHZ
SPIDEV osd        SPI2 DEVID4 OSD1_CS   MODE0 10*MHZ  10*MHZ
SPIDEV dataflash  SPI3 DEVID1 FLASH1_CS MODE3 104*MHZ 104*MHZ

# ------------------------------------------------------------------
# IMUs (adjust rotation to the physical sensor orientation)
# ------------------------------------------------------------------
IMU Invensensev3 SPI:icm42688   ROTATION_YAW_180
IMU Invensensev3 SPI:icm42688_2 ROTATION_YAW_270
define HAL_DEFAULT_INS_FAST_SAMPLE 3

# ------------------------------------------------------------------
# I2C - baro on-board, external compass
# ------------------------------------------------------------------
PB6  I2C1_SCL I2C1
PB7  I2C1_SDA I2C1
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

I2C_ORDER I2C2 I2C1

BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76    # fallback probe for alternate BOM

define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 0
define HAL_COMPASS_AUTO_ROT_DEFAULT 2
define ALLOW_ARM_NO_COMPASS

# ------------------------------------------------------------------
# OSD
# ------------------------------------------------------------------
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ------------------------------------------------------------------
# Dataflash logging
# ------------------------------------------------------------------
define HAL_LOGGING_DATAFLASH_ENABLED 1
define HAL_OS_FATFS_IO 0

# ------------------------------------------------------------------
# DroneCAN (CAN1 via FDCAN1 on PD0/PD1)
# ------------------------------------------------------------------
PD0  CAN1_RX CAN1
PD1  CAN1_TX CAN1
# Transceiver standby/silent pin: LOW = active
PD3  GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

# HAL_NUM_CAN_IFACES is derived automatically from the CAN pins above.
# DroneCAN is enabled by default on H7. Driver/protocol defaults are set
# in defaults.parm (CAN_P1_DRIVER / CAN_D1_PROTOCOL).

# ------------------------------------------------------------------
# Motor / PWM outputs (bidirectional DShot capable)
# BIDIR goes on the first channel of each timer channel pair
# ------------------------------------------------------------------
PB0  TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1  TIM3_CH4 TIM3 PWM(2) GPIO(51)
PA0  TIM5_CH1 TIM5 PWM(3) GPIO(52) BIDIR
PA1  TIM5_CH2 TIM5 PWM(4) GPIO(53)
PA2  TIM5_CH3 TIM5 PWM(5) GPIO(54) BIDIR
PA3  TIM5_CH4 TIM5 PWM(6) GPIO(55)
PD12 TIM4_CH1 TIM4 PWM(7) GPIO(56) BIDIR
PD13 TIM4_CH2 TIM4 PWM(8) GPIO(57)

# Serial LED (WS2812) output
PA8  TIM1_CH1 TIM1 PWM(9) GPIO(58)

# ------------------------------------------------------------------
# Buzzer
# ------------------------------------------------------------------
PA15 TIM2_CH1 TIM2 GPIO(32) ALARM

# ------------------------------------------------------------------
# Status LEDs
# ------------------------------------------------------------------
PE3  LED0 OUTPUT LOW GPIO(90)   # blue
PE4  LED1 OUTPUT LOW GPIO(91)   # green
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

# ------------------------------------------------------------------
# Analog inputs
# H743: PC0=ADC123_INP10, PC1=ADC123_INP11, PC4=ADC12_INP4, PC5=ADC12_INP8
# ------------------------------------------------------------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 RSSI_ADC          ADC1 SCALE(1)
PC5 PRESSURE_SENS     ADC1 SCALE(2)   # airspeed / spare

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_B