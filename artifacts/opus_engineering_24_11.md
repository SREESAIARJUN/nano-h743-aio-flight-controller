# ArduPilot hwdef for a Generic STM32H743VIT6 Nano AIO (Rev "Cycle 24")

This is a reference hwdef for a hypothetical board. Before you flash it:

- **Check every pin against your schematic.** Pin choices here are valid for the LQFP100 package (ports A–E only). Your board's routing will differ.
- **DMA streams are allocated automatically.** ArduPilot does not take hand-written DMA stream numbers. You control allocation with `DMA_PRIORITY`, `DMA_NOSHARE` and `NODMA`, and `chibios_hwdef.py` resolves the streams at build time.
- **Replace the board ID.** `APJ_BOARD_ID` must be a registered ID from `Tools/AP_Bootloader/board_types.txt`. The value used here is a placeholder.

---

## 1. `libraries/AP_HAL_ChibiOS/hwdef/NanoAIO-H743/hwdef.dat`

```
# hw definition file for NanoAIO-H743 (STM32H743VIT6, LQFP100), Rev Cycle24
# 4-in-1 ESC AIO, ICM42688 IMU, DPS310 baro, MAX7456 OSD, SPI dataflash, DroneCAN

# ---------------------------------------------------------------------------
# MCU / board identity
# ---------------------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# PLACEHOLDER - obtain/register a unique ID in Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID TARGET_HW_NANOAIO_H743

# crystal frequency
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048
# bootloader occupies first 128k sector
FLASH_RESERVE_START_KB 128

# parameter storage in last two 128k flash sectors
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 32768

# system timer
STM32_ST_USE_TIMER 12
define CH_CFG_ST_RESOLUTION 16

# ---------------------------------------------------------------------------
# USB
# ---------------------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
USB_STRING_MANUFACTURER "ArduPilot"

# SWD debug
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------------------------------------------------------------------
# Serial ports
# SERIAL0=USB, 1=USART1 (telem), 2=USART2 (RCIN), 3=USART3 (GPS),
# 4=UART4 (VTX/MSP), 5=UART7 (ESC telem RX), 6=UART8 (spare)
# ---------------------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 UART7 UART8

# USART1 - telem1
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - RC input (CRSF/ELRS/SBUS)
PA2  USART2_TX USART2
PA3  USART2_RX USART2

# USART3 - GPS
PB10 USART3_TX USART3
PB11 USART3_RX USART3

# UART4 - VTX / DisplayPort
PA0  UART4_TX UART4
PA1  UART4_RX UART4

# UART7 - ESC telemetry (RX only used)
PE8  UART7_TX UART7
PE7  UART7_RX UART7 NODMA

# UART8 - spare, low rate
PE1  UART8_TX UART8 NODMA
PE0  UART8_RX UART8 NODMA

# RC input default on SERIAL2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL2_BAUD 115
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_DJI_FPV
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL5_BAUD 115

# ---------------------------------------------------------------------------
# SPI buses
# SPI1 = IMU, SPI2 = OSD, SPI3 = dataflash
# ---------------------------------------------------------------------------
PA5  SPI1_SCK  SPI1
PA6  SPI1_MISO SPI1
PA7  SPI1_MOSI SPI1

PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# chip selects
PA4  IMU1_CS   CS
PB12 OSD1_CS   CS
PA15 FLASH1_CS CS

# IMU data-ready
PC4  IMU1_DRDY INPUT

# SPI device table
#      name       bus  devid  CS        mode  low     high
SPIDEV icm42688   SPI1 DEVID1 IMU1_CS   MODE3 2*MHZ   16*MHZ
SPIDEV osd        SPI2 DEVID4 OSD1_CS   MODE0 10*MHZ  10*MHZ
SPIDEV dataflash  SPI3 DEVID1 FLASH1_CS MODE3 104*MHZ 104*MHZ

# ---------------------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------------------
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
define HAL_DEFAULT_INS_FAST_SAMPLE 1

# I2C1 - baro onboard, external compass on same bus via connector
PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1
I2C_ORDER I2C1
define HAL_I2C_INTERNAL_MASK 0

BARO DPS310 I2C:0:0x76

# no onboard compass; probe externals
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
define HAL_LOGGING_DATAFLASH_ENABLED 1
define HAL_OS_FATFS_IO 0

# ---------------------------------------------------------------------------
# OSD
# ---------------------------------------------------------------------------
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ---------------------------------------------------------------------------
# ADC - battery / current / RSSI
# ---------------------------------------------------------------------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC5 RSSI_IN ADC1

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 8

# ---------------------------------------------------------------------------
# Motors / PWM (TIM8 for 4-in-1 ESC, bidirectional DShot on CH1/CH3)
# ---------------------------------------------------------------------------
PC6 TIM8_CH1 TIM8 PWM(1) GPIO(50) BIDIR
PC7 TIM8_CH2 TIM8 PWM(2) GPIO(51)
PC8 TIM8_CH3 TIM8 PWM(3) GPIO(52) BIDIR
PC9 TIM8_CH4 TIM8 PWM(4) GPIO(53)

# auxiliary outputs (servo/extra motors)
PB0 TIM3_CH3 TIM3 PWM(5) GPIO(54)
PB1 TIM3_CH4 TIM3 PWM(6) GPIO(55)

# WS2812 LED strip
PA8 TIM1_CH1 TIM1 PWM(7) GPIO(56)

define HAL_DEFAULT_MOT_PWM_TYPE 6        # DShot600 (verify enum for your branch)

# ---------------------------------------------------------------------------
# GPIO: LEDs, buzzer, camera switch
# ---------------------------------------------------------------------------
PE3 LED0 OUTPUT LOW GPIO(90)
PE4 LED1 OUTPUT LOW GPIO(91)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

PD15 TIM4_CH4 TIM4 GPIO(77) ALARM

PE9 PINIO1 OUTPUT LOW GPIO(81)          # VTX power / cam switch
define RELAY1_PIN_DEFAULT 81

# ---------------------------------------------------------------------------
# DroneCAN (FDCAN1)
# ---------------------------------------------------------------------------
PD0 CAN1_RX CAN1