# ArduPilot hwdef for a Generic STM32H743VIT6 Nano AIO

This is a **reference template**, not a drop-in file for a specific board. Every pin assignment must be checked against your board's schematic. Wrong CS, DRDY or timer pins will at best stop sensors from probing and at worst drive outputs incorrectly.

**How ArduPilot handles DMA:** you do not assign DMA streams manually. `chibios_hwdef.py` and `dma_resolver.py` allocate them automatically at build time. You only influence the allocation with `DMA_PRIORITY`, `DMA_NOSHARE` and `NODMA`. The resolved map is written to `build/<board>/hwdef.h`. Check it there, and look for "shared" streams.

**Package constraint:** the STM32H743**V**IT6 is LQFP100, so only ports A–E are available. All pins below exist on that package.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoAIO-H743/hwdef.dat`

```
# hw definition file for generic NanoAIO-H743 (STM32H743VIT6, LQFP100)
# VERIFY ALL PINS AGAINST SCHEMATIC BEFORE FLASHING

# ---------------- MCU / board ----------------
MCU STM32H7xx STM32H743xx

# Use a registered board ID from Tools/AP_Bootloader/board_types.txt
# Placeholder - request an official ID before distribution
APJ_BOARD_ID 1999

OSCILLATOR_HZ 8000000

# 2MB flash, 128k bootloader reserve
FLASH_SIZE_KB 2048
FLASH_RESERVE_START_KB 128

STM32_ST_USE_TIMER 2
define CH_CFG_ST_RESOLUTION 32

# ---------------- USB / debug ----------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

USB_STRING_MANUFACTURER "ArduPilot"

# ---------------- Serial ports ----------------
# order: SERIAL0..SERIAL8
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8 OTG2

# USART1 - telem1
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - telem2 / VTX
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - GPS1
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - GPS2 / spare
PB9 UART4_TX UART4
PB8 UART4_RX UART4

# USART6 - RC input (serial protocols: CRSF/ELRS/SBUS)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 - ESC telemetry (RX only typically)
PE8 UART7_TX UART7
PE7 UART7_RX UART7

# UART8 - spare / DJI
PE1 UART8_TX UART8
PE0 UART8_RX UART8

# Default protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_None

# Don't waste DMA on low-rate ESC telemetry TX
NODMA UART7_TX

# ---------------- RC input ----------------
# Timer-based RCIN (PPM/SBUS inverted via timer) - optional, alt to USART6
PB7 TIM4_CH2 TIM4 RCININT PULLDOWN LOW

# ---------------- SPI buses ----------------
# SPI1 - IMU1
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PD7 SPI1_MOSI SPI1

# SPI2 - OSD (MAX7456 / AT7456E)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - dataflash
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# SPI4 - IMU2
PE12 SPI4_SCK  SPI4
PE13 SPI4_MISO SPI4
PE14 SPI4_MOSI SPI4

# Chip selects
PC15 IMU1_CS   CS
PB12 OSD1_CS   CS
PA15 FLASH1_CS CS
PE11 IMU2_CS   CS

# Data-ready interrupts
PB2  DRDY1_ICM42688 INPUT
PE15 DRDY2_BMI270   INPUT

# ---------------- SPI device table ----------------
# NAME     BUS  DEV  CS        MODE        LOWSPEED   HIGHSPEED
SPIDEV imu1     SPI1 DEVID1 IMU1_CS   MODE3  2*MHZ  16*MHZ
SPIDEV osd      SPI2 DEVID1 OSD1_CS   MODE0 10*MHZ  10*MHZ
SPIDEV dataflash SPI3 DEVID1 FLASH1_CS MODE3 32*MHZ  32*MHZ
SPIDEV imu2     SPI4 DEVID1 IMU2_CS   MODE3  1*MHZ  10*MHZ

# ---------------- IMUs ----------------
IMU Invensensev3 SPI:imu1 ROTATION_YAW_180
IMU BMI270       SPI:imu2 ROTATION_YAW_270

define HAL_DEFAULT_INS_FAST_SAMPLE 3

# ---------------- I2C ----------------
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

I2C_ORDER I2C2

# Baro (onboard)
BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76

# Compass - external only on AIO
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 0
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ---------------- OSD ----------------
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ---------------- Dataflash logging ----------------
define HAL_LOGGING_DATAFLASH_ENABLED 1

# ---------------- ADC ----------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 RSSI_ADC          ADC1

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 4

# ---------------- Motor / PWM outputs ----------------
# Keep motors on whole timers so DShot rate groups are clean
PB0 TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1 TIM3_CH4 TIM3 PWM(2) GPIO(51)
PB4 TIM3_CH1 TIM3 PWM(3) GPIO(52) BIDIR
PB5 TIM3_CH2 TIM3 PWM(4) GPIO(53)

PA0 TIM5_CH1 TIM5 PWM(5) GPIO(54) BIDIR
PA1 TIM5_CH2 TIM5 PWM(6) GPIO(55)
PA2 TIM5_CH3 TIM5 PWM(7) GPIO(56) BIDIR
PA3 TIM5_CH4 TIM5 PWM(8) GPIO(57)

# Serial LED (NeoPixel)
PA8 TIM1_CH1 TIM1 PWM(9) GPIO(58)

define HAL_WITH_DSHOT 1

# ---------------- Buzzer / LEDs ----------------
PE5 TIM15_CH1 TIM15 GPIO(32) ALARM

PE3 LED0 OUTPUT LOW GPIO(90)
PE4 LED1 OUTPUT LOW GPIO(91)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

# ---------------- DroneCAN ----------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
# Transceiver silent/standby control (active high = silent on most PHYs)
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW GPIO(70)

define HAL_NUM_CAN_IFACES 1
define HAL_CAN_POOL_SIZE 6000
define CAN_P1_DRIVER_DEFAULT 1
define CAN_D1_PROTOCOL_DEFAULT 1    # 1 = DroneCAN

# ---------------- Power / VDD ----------------
define HAL_HAVE_SAFETY_SWITCH 0
define BOARD_SAFETY_ENABLE_DEFAULT 0
define HAL_STORAGE_SIZE 32768
STORAGE_FLASH_PAGE 14

# ---------------- DMA hints (allocation is automatic) ----------------
# Prioritise IMU and DShot; never share IMU streams
DMA_PRIORITY SPI1* SPI4* TIM3* TIM5* SPI3*
DMA_NOSHARE  SPI1* SPI4* TIM3_UP TIM5_UP
```

---

## `hwdef-bl.dat` (bootloader)

```
MCU STM32H7xx STM32H743xx
APJ_BOARD_ID 1999
OSCILLATOR_HZ 8000000
FLASH_SIZE_KB 2048
FLASH_BOOTLOADER_LOAD_KB 128
FLASH_RESERVE_START_KB 0

STM32_ST_USE_TIMER 2
define CH_CFG_ST_RESOLUTION 32

SERIAL_ORDER OTG1 USART1

PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

PA9  USART1_TX USART1
PA10 USART1_RX USART1

PE3 LED_BOOTLOADER OUTPUT LOW
PE4 LED_ACTIVITY   OUTPUT LOW
define HAL_LED_ON 0

# Hold CS lines high so SPI devices don't drive buses during boot
PC15 IMU1_CS   CS
PB12 OSD1_CS   CS
PA15 FLASH1_CS CS
PE11 IMU2_CS   CS

# CAN bootloader support (optional DroneCAN firmware update)
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW
```

---

## Build and Verify

```bash
./waf configure --