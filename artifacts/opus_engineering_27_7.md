# ArduPilot hwdef for a Generic STM32H743VIT6 Nano AIO Flight Controller

You didn't name a commercial board, so this is a **reference design** with a coherent, conflict-free pin map. All pins are valid for the LQFP100 package (ports A to E plus PH0/PH1). **You must check every pin against your actual schematic** before flashing, because wrong pin assignments can damage ESCs or peripherals.

I don't recognize "Cycle 27" as an ArduPilot term, so it appears below only as a revision label in the header.

## About DMA on the H7

On the STM32H7, ArduPilot does **not** hard-code DMA stream and channel numbers in hwdef.dat. The H7 has a DMAMUX, and `chibios_hwdef.py` allocates streams automatically at build time. You control allocation only through:

- `DMA_PRIORITY`: which peripherals get streams first.
- `DMA_NOSHARE`: which peripherals must not share a stream.

The resulting stream map is written to `build/<board>/hwdef.h` (look for the `STM32_..._DMA_STREAM` defines). Writing manual stream numbers into hwdef.dat would be ignored or would break the build.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```
# hw definition file for generic NanoH743AIO (STM32H743VIT6, LQFP100)
# Revision label: Cycle 27
# VERIFY ALL PINS AGAINST YOUR SCHEMATIC

# ---------------- MCU ----------------
MCU STM32H7xx STM32H743xx

# Board ID: placeholder. Register a unique ID in Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID 9999

# Crystal
OSCILLATOR_HZ 8000000
PH0 RCC_OSC_IN OSC
PH1 RCC_OSC_OUT OSC

FLASH_SIZE_KB 2048
# Bootloader occupies the first 128k sector
FLASH_RESERVE_START_KB 128

# Parameter storage in flash sectors
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 32768

env OPTIMIZE -Os

# ---------------- USB ----------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
USB_STRING_MANUFACTURER "ArduPilot"

# ---------------- Serial ----------------
# SERIAL0=USB, 1=USART1, 2=USART2, 3=USART3, 4=(CAN-shared, empty),
# 5=USART6 (RCIN), 6=UART7, 7=UART8
SERIAL_ORDER OTG1 USART1 USART2 USART3 EMPTY USART6 UART7 UART8

PA9  USART1_TX USART1
PA10 USART1_RX USART1

PD5  USART2_TX USART2
PD6  USART2_RX USART2

PD8  USART3_TX USART3
PD9  USART3_RX USART3

PC6  USART6_TX USART6
PC7  USART6_RX USART6

PE8  UART7_TX UART7
PE7  UART7_RX UART7

PE1  UART8_TX UART8
PE0  UART8_RX UART8

# Default serial protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_MSP_DisplayPort
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_None

# ---------------- SPI buses ----------------
# SPI1: IMU1 (ICM-42688-P)
PA5  SPI1_SCK  SPI1
PA6  SPI1_MISO SPI1
PA7  SPI1_MOSI SPI1

# SPI2: OSD (AT7456E / MAX7456)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3: Dataflash (W25N01GV)
PB3  SPI3_SCK  SPI3
PB4  SPI3_MISO SPI3
PB5  SPI3_MOSI SPI3

# SPI4: IMU2 (BMI270)
PE2  SPI4_SCK  SPI4
PE5  SPI4_MISO SPI4
PE6  SPI4_MOSI SPI4

# Chip selects
PA4  IMU1_CS   CS
PE4  IMU2_CS   CS
PB12 OSD1_CS   CS
PD10 FLASH1_CS CS

# ---------------- SPI device table ----------------
#       name       bus  devid  CS pin     mode  lowspd    highspd
SPIDEV icm42688   SPI1 DEVID1 IMU1_CS    MODE3  2*MHZ    16*MHZ
SPIDEV bmi270     SPI4 DEVID1 IMU2_CS    MODE3  1*MHZ    10*MHZ
SPIDEV osd        SPI2 DEVID1 OSD1_CS    MODE0 10*MHZ    10*MHZ
SPIDEV dataflash  SPI3 DEVID1 FLASH1_CS  MODE3 104*MHZ  104*MHZ

# ---------------- IMUs ----------------
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
IMU BMI270       SPI:bmi270   ROTATION_YAW_90
define HAL_DEFAULT_INS_FAST_SAMPLE 3

# ---------------- I2C ----------------
I2C_ORDER I2C1 I2C2

# I2C1: onboard baro
PB8  I2C1_SCL I2C1
PB9  I2C1_SDA I2C1

# I2C2: external compass / GPS module
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

# ---------------- Baro ----------------
BARO DPS310 I2C:0:0x76
BARO SPL06  I2C:0:0x76

# ---------------- Compass ----------------
# No onboard mag; probe external I2C
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 1
define ALLOW_ARM_NO_COMPASS

# ---------------- DroneCAN / FDCAN ----------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
# Optional transceiver silent/standby pin (remove if not fitted)
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

# ---------------- Motor / PWM outputs ----------------
# Group 1: TIM1 (M1-M4) - DShot/BDShot capable
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50)
PE11 TIM1_CH2 TIM1 PWM(2) GPIO(51)
PE13 TIM1_CH3 TIM1 PWM(3) GPIO(52)
PE14 TIM1_CH4 TIM1 PWM(4) GPIO(53)

# Group 2: TIM4 (M5-M8)
PD12 TIM4_CH1 TIM4 PWM(5) GPIO(54)
PD13 TIM4_CH2 TIM4 PWM(6) GPIO(55)
PD14 TIM4_CH3 TIM4 PWM(7) GPIO(56)
PD15 TIM4_CH4 TIM4 PWM(8) GPIO(57)

# Group 3: WS2812 LED strip (separate timer)
PA15 TIM2_CH1 TIM2 PWM(9) GPIO(58)

define HAL_WITH_DSP 1
define STM32_PWM_USE_ADVANCED TRUE

# ---------------- ADC ----------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 RSSI_IN           ADC1 SCALE(1)

define HAL_BATT