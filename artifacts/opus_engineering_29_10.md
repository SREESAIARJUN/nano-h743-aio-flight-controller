# ArduPilot `hwdef.dat` for a Generic STM32H743VIT6 Nano AIO

This is a reference design, not the definition for any specific commercial board. I can't identify a real board from "nano AIO" and "Cycle 29", so every pin is a placeholder. **Verify every pin, sensor, and rotation against your schematic before flashing.** Wrong pin assignments can cause motor misfires or no-boot conditions.

The pin map is chosen to be internally consistent for the LQFP100 (`V`) package: no pin conflicts, and valid alternate functions on the H743.

---

## `hwdef.dat`

```text
# hw definition file for a generic STM32H743VIT6 "Nano AIO" flight controller
# Reference design: verify all pins against the board schematic

# ---------------------------------------------------------------
# MCU / board identity
# ---------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Board ID: must be registered in Tools/AP_Bootloader/board_types.txt
# 9999 is a PLACEHOLDER and must be replaced with an allocated ID
APJ_BOARD_ID 9999

# Crystal
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048

# First 128k for bootloader, next 256k reserved for flash storage
FLASH_RESERVE_START_KB 384
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 15360

# Use the internal RTC/backup domain for watchdog info
define HAL_WITH_DSP 1

# ---------------------------------------------------------------
# USB
# ---------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
USB_STRING_MANUFACTURER "ArduPilot"
USB_STRING_PRODUCT "NanoAIO-H743"

# ---------------------------------------------------------------
# SWD debug
# ---------------------------------------------------------------
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------------------------------------------------------
# Serial port order
# SERIAL0 = USB, SERIAL1..7 = UARTs below
# ---------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USART1 - Telemetry / MAVLink
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - GPS
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - VTX (MSP / SmartAudio)
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - ESC telemetry (RX only used)
PB9 UART4_TX UART4
PB8 UART4_RX UART4

# USART6 - RC input (CRSF / ELRS / SBUS)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 - spare / DJI
PE8 UART7_TX UART7
PE7 UART7_RX UART7

# UART8 - spare
PE1 UART8_TX UART8
PE0 UART8_RX UART8

# Low-bandwidth ports: free up DMA streams
NODMA UART7 UART8

# Default protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_MSP_DisplayPort
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_None
define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_None

# ---------------------------------------------------------------
# SPI buses
# ---------------------------------------------------------------
# SPI1 - IMU1
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PD7 SPI1_MOSI SPI1

# SPI2 - OSD (MAX7456 / AT7456E)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - Dataflash
PB3 SPI3_SCK  SPI3
PB4 SPI3_MISO SPI3
PB5 SPI3_MOSI SPI3

# SPI4 - IMU2
PE12 SPI4_SCK  SPI4
PE13 SPI4_MISO SPI4
PE14 SPI4_MOSI SPI4

# Chip selects
PC15 IMU1_CS   CS
PE11 IMU2_CS   CS
PB12 OSD_CS    CS
PA15 FLASH_CS  CS

# IMU data-ready interrupts (optional, used for FIFO timing)
PB2  IMU1_DRDY INPUT
PE15 IMU2_DRDY INPUT

# SPI device table
#       name       bus dev cs        mode       lowspeed  highspeed
SPIDEV  icm42688   SPI1 DEVID1 IMU1_CS  MODE3   2*MHZ    16*MHZ
SPIDEV  bmi270     SPI4 DEVID1 IMU2_CS  MODE3   1*MHZ    10*MHZ
SPIDEV  osd        SPI2 DEVID1 OSD_CS   MODE0  10*MHZ    10*MHZ
SPIDEV  dataflash  SPI3 DEVID1 FLASH_CS MODE3 104*MHZ   104*MHZ

# ---------------------------------------------------------------
# IMUs (rotations are PLACEHOLDERS - set from board layout)
# ---------------------------------------------------------------
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
IMU BMI270       SPI:bmi270   ROTATION_YAW_90

define HAL_DEFAULT_INS_FAST_SAMPLE 3

# ---------------------------------------------------------------
# I2C buses
# ---------------------------------------------------------------
I2C_ORDER I2C1 I2C2

# I2C1 - internal baro
PB6 I2C1_SCL I2C1
PB7 I2C1_SDA I2C1

# I2C2 - external compass / peripherals (GPS connector)
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

# Barometer
BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76

# No internal compass; probe external I2C buses
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 1
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ---------------------------------------------------------------
# Motor / servo outputs (bidirectional DShot capable)
# ---------------------------------------------------------------
PB0  TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1  TIM3_CH4 TIM3 PWM(2) GPIO(51)
PA0  TIM5_CH1 TIM5 PWM(3) GPIO(52) BIDIR
PA1  TIM5_CH2 TIM5 PWM(4) GPIO(53)
PA2  TIM5_CH3 TIM5 PWM(5) GPIO(54) BIDIR
PA3  TIM5_CH4 TIM5 PWM(6) GPIO(55)
PD12 TIM4_CH1 TIM4 PWM(7) GPIO(56) BIDIR
PD13 TIM4_CH2 TIM4 PWM(8) GPIO(57)

# LED strip (WS2812 / NeoPixel) on its own timer
PA8  TIM1_CH1 TIM1 PWM(9) GPIO(58)

# Buzzer (timer-driven alarm)
PD15 TIM4_CH4 TIM4 GPIO(32) ALARM

# ---------------------------------------------------------------
# DroneCAN (FDCAN1)
# ---------------------------------------------------------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1

# Transceiver standby/silent pin (active-low enable on most xcvrs)
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW GPIO(70)

# ---------------------------------------------------------------
# ADC
# ---------------------------------------------------------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 RSSI_ADC          ADC1
PC5 SPARE_ADC         ADC1

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
# Divider/shunt scales are PLACEHOLDERS - calibrate per board
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0

define BOARD_RSSI_ANA_PIN 4

# ---------------------------------------------------------------
# LEDs and GPIO
# ---------------------------------------------------------------
PE3 LED0 OUTPUT LOW GPIO(90)
PE4 LED1 OUTPUT LOW GPIO(91)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

# PINIO: VTX power and camera switch
PE9  PINIO1 OUTPUT GPIO(81) LOW
PE10 PINIO2 OUTPUT GPIO(82) LOW
define RELAY1_PIN_DEFAULT 81
define RELAY2_PIN_DEFAULT 82

# ---------------------------------------------------------------
# DMA
# H7 uses DMAMUX: streams are allocated automatically by the
# build scripts. Only priorities/sharing rules are specified.
# ---------------------------------------------------------------
DMA_PRIORITY TIM3* TIM5* TIM4* SPI1* SPI4* USART6*
DMA_NOSHARE  SPI1* SPI4* TIM3* TIM5