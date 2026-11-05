# ArduPilot hwdef for a Generic STM32H743VIT6 Nano AIO Flight Controller

## Before you start

- **This is a template, not a drop-in file for a specific board.** No standard board is called "STM32H743VIT6 nano AIO." Every pin below must be checked against your schematic. If one is wrong, a sensor won't be detected or a peripheral will stay silent.
- **ArduPilot assigns DMA channels for you.** On the H7, any stream can serve any peripheral through the DMAMUX. Hand-written stream numbers are ignored or cause conflicts. You control DMA only with `DMA_PRIORITY` and `DMA_NOSHARE`, as shown below. You can check the result in `build/<board>/hwdef.h` after building.
- **The VIT6 is a 100-pin package.** It only has ports PA–PE. Any PF/PG/PH pins copied from other boards will not exist.
- **DroneCAN needs a physical CAN transceiver**, for example a TJA1051 or SN65HVD230 on FDCAN1.
- "Cycle 13" doesn't match any ArduPilot convention, so I've ignored it.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```text
# hw definition file for generic STM32H743VIT6 Nano AIO flight controller
# Verify ALL pins against your schematic before flashing.

# ---------------- MCU ----------------
MCU STM32H7xx STM32H743xx

# Board ID: request a unique ID via PR to Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID TARGET_HW_NANOH743AIO

# crystal frequency (check your board: 8MHz or 16MHz typical)
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048
# bootloader occupies first 128k
FLASH_RESERVE_START_KB 128

# use flash for parameter storage (no FRAM on AIO boards)
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 15360

# ---------------- Serial ports ----------------
#           SERIAL0 SERIAL1 SERIAL2 SERIAL3 SERIAL4 SERIAL5 SERIAL6 SERIAL7 SERIAL8
SERIAL_ORDER OTG1   USART1  USART2  USART3  UART4   USART6  UART7   UART8   OTG2

# USB
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# SWD debug
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# USART1 - SERIAL1 (telemetry / MAVLink)
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - SERIAL2 (RC input, e.g. ELRS/CRSF)
PD5  USART2_TX USART2
PD6  USART2_RX USART2

# USART3 - SERIAL3 (GPS)
PD8  USART3_TX USART3
PD9  USART3_RX USART3

# UART4 - SERIAL4
PB9  UART4_TX UART4
PB8  UART4_RX UART4

# USART6 - SERIAL5 (VTX / DJI)
PC6  USART6_TX USART6
PC7  USART6_RX USART6

# UART7 - SERIAL6 (ESC telemetry, RX only)
PE7  UART7_RX UART7
PE8  UART7_TX UART7

# UART8 - SERIAL7
PE0  UART8_RX UART8
PE1  UART8_TX UART8

# defaults for serial protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_DJI_FPV
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL6_BAUD 115

# ---------------- SPI buses ----------------
# SPI1 - IMU (ICM42688-P)
PA5  SPI1_SCK  SPI1
PA6  SPI1_MISO SPI1
PA7  SPI1_MOSI SPI1
PA4  IMU1_CS   CS
PC4  IMU1_DRDY INPUT

# SPI2 - OSD (AT7456E / MAX7456)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2
PB12 OSD1_CS   CS

# SPI3 - dataflash (W25N01G / W25Q128)
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3
PA15 FLASH1_CS CS

# SPI device table
#       name       bus dev mode        lowspeed highspeed
SPIDEV  icm42688   SPI1 DEVID1 IMU1_CS   MODE3 2*MHZ 16*MHZ
SPIDEV  osd        SPI2 DEVID1 OSD1_CS   MODE0 10*MHZ 10*MHZ
SPIDEV  dataflash  SPI3 DEVID1 FLASH1_CS MODE3 32*MHZ 32*MHZ

# IMU (rotation must match physical mounting - verify!)
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
define HAL_DEFAULT_INS_FAST_SAMPLE 1

# OSD
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# Dataflash logging
define HAL_LOGGING_DATAFLASH_ENABLED 1

# ---------------- I2C ----------------
PB6  I2C1_SCL I2C1
PB7  I2C1_SDA I2C1
I2C_ORDER I2C1

# Barometer (on-board) + probe external compass
BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 0
define ALLOW_ARM_NO_COMPASS

# ---------------- DroneCAN (FDCAN1) ----------------
PD0  CAN1_RX CAN1
PD1  CAN1_TX CAN1
PD3  GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

# ---------------- Motors / PWM ----------------
# TIM1 - motors 1-4 (bidirectional DShot capable)
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50) BIDIR
PE11 TIM1_CH2 TIM1 PWM(2) GPIO(51)
PE13 TIM1_CH3 TIM1 PWM(3) GPIO(52) BIDIR
PE14 TIM1_CH4 TIM1 PWM(4) GPIO(53)

# TIM4 - motors 5-8
PD12 TIM4_CH1 TIM4 PWM(5) GPIO(54) BIDIR
PD13 TIM4_CH2 TIM4 PWM(6) GPIO(55)
PD14 TIM4_CH3 TIM4 PWM(7) GPIO(56) BIDIR
PD15 TIM4_CH4 TIM4 PWM(8) GPIO(57)

# TIM15 - WS2812 LED strip
PE5  TIM15_CH1 TIM15 PWM(9) GPIO(58)

# ---------------- ADC ----------------
PC0  BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1  BATT_CURRENT_SENS ADC1 SCALE(1)
PC5  RSSI_ADC          ADC1

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 8

# ---------------- LEDs / buzzer ----------------
PE3  LED0 OUTPUT LOW GPIO(90)
PE4  LED1 OUTPUT LOW GPIO(91)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

PC13 BUZZER OUTPUT GPIO(80) LOW
define HAL_BUZZER_PIN 80

# ---------------- DMA policy ----------------
# Streams are auto-allocated via DMAMUX; only priority/sharing is configured.
DMA_PRIORITY SPI1* TIM1* TIM4* USART2*
DMA_NOSHARE  SPI1* TIM1UP* TIM4UP*

# ---------------- Misc ----------------
define HAL_OS_FATFS_IO 0
define HAL_WITH_DSP 1
define HAL_FRAME_TYPE_DEFAULT 1
```

### Notes on the analog pin numbers

The values for `HAL_BATT_VOLT_PIN`, `HAL_BATT_CURR_PIN` and `BOARD_RSSI_ANA_PIN` are the H7 ADC channel numbers for PC0, PC1 and PC5:

| Pin | Use | ADC channel |
|-----|-----|-------------|
| PC0 | Battery voltage | 10 |
| PC1 | Battery current | 11 |
| PC5 | RSSI | 8 |

Re-check these numbers in the RM0433 reference manual if you move any of these pins.

### Notes on CAN

- Including the `CAN1` pins is enough to enable the FDCAN driver.
- DroneCAN itself is switched on through the parameter defaults below.

---

## `hwdef-bl.dat` (bootloader)

```text
MCU STM32H7xx STM32H743xx
APJ_BOARD_ID TARGET_HW_NANOH743AIO
OSCILLATOR_HZ 8000000
FLASH_SIZE_KB 2048
FLASH_BOOTLOADER_LOAD_KB 128
FLASH_RESERVE_START_KB 0

SERIAL_ORDER OTG1 USART1
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD
PA9  USART1_TX USART1
PA10 USART1_RX USART1

PE3  LED_BOOTLOADER OUTPUT LOW
PE4  LED_ACTIVITY   OUTPUT LOW
define HAL_LED_ON 0

# allow DroneCAN firmware update from bootloader
PD0  CAN1_RX CAN1
PD1  CAN1_TX CAN1
PD3  GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

# keep flash CS deselected during boot
PA15 FLASH1_CS CS
PA4  IMU1_CS   CS
PB12 OSD1_CS   CS
```

---

## `defaults.parm` (DroneCAN en