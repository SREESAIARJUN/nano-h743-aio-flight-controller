# ArduPilot hwdef.dat: STM32H743VIT6 Nano AIO (Revision 10)

**This is a reference template. Pin assignments must be checked against your schematic before you build or flash.** The pinout is a valid, conflict-free allocation for the STM32H743VIT6 (LQFP100, ports A to E only). It follows common H743 AIO layouts:

- **Sensors:** dual ICM-42688-P, DPS310 barometer, MAX7456 OSD, 128 Mbit SPI NOR flash
- **Outputs:** 8 bidirectional-DShot outputs plus 1 NeoPixel output
- **Ports:** 7 UARTs, 2 I²C buses, 1 FDCAN bus for DroneCAN

## Note on DMA in ArduPilot

ArduPilot does **not** use hand-written DMA stream tables. The H7 has a DMAMUX, and `Tools/ardupilotwaf/chibios_hwdef.py` (via `dma_resolver.py`) assigns streams automatically at build time.

You control DMA through three mechanisms:

- `DMA_PRIORITY`: which peripherals get first claim on streams
- `DMA_NOSHARE`: which peripherals must never share a stream
- the `NODMA` flag on individual pins

After building, the resolved allocation appears in `build/<board>/hwdef.h` (look for `STM32_*_DMA_STREAM`). A reference table is included at the end of this answer.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```
# hw definition file for NanoH743AIO
# MCU: STM32H743VIT6 (LQFP100, 2MB flash, 1MB RAM)
# Revision: 10

# ---------------------------------------------------------------
# MCU / board identity
# ---------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Board ID: MUST be allocated in Tools/AP_Bootloader/board_types.txt
# before release. Placeholder value below.
APJ_BOARD_ID TARGET_HW_NANO_H743_AIO

# 8 MHz HSE crystal
OSCILLATOR_HZ 8000000

# System timer
STM32_ST_USE_TIMER 12

# ---------------------------------------------------------------
# Flash layout
# Sector 0 (128k) = bootloader
# Sectors 1-2 (256k) = parameter storage
# Firmware starts at 384k
# ---------------------------------------------------------------
FLASH_SIZE_KB 2048
FLASH_RESERVE_START_KB 384
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 16384

# ---------------------------------------------------------------
# USB (OTG_FS)
# ---------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9  VBUS INPUT OPENDRAIN

USB_STRING_MANUFACTURER "ArduPilot"
USB_STRING_PRODUCT "NanoH743AIO"

# ---------------------------------------------------------------
# SWD debug
# ---------------------------------------------------------------
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------------------------------------------------------
# Serial port ordering
# SERIAL0 = USB
# SERIAL1 = USART1  (Telem1 / MAVLink)
# SERIAL2 = USART2  (Telem2 / MSP DisplayPort)
# SERIAL3 = USART3  (GPS1)
# SERIAL4 = UART4   (GPS2 / spare)
# SERIAL5 = USART6  (RC input)
# SERIAL6 = UART7   (ESC telemetry, RX only)
# SERIAL7 = UART8   (spare / VTX control)
# SERIAL8 = OTG2    (virtual SLCAN / MAVLink2 USB)
# ---------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8 OTG2

# USART1
PB6 USART1_TX USART1
PB7 USART1_RX USART1

# USART2
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 (GPS)
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4
PA0 UART4_TX UART4
PA1 UART4_RX UART4

# USART6 (RC input, full-duplex capable for CRSF/ELRS)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 (ESC telemetry, RX only, no DMA needed at 115200)
PE7 UART7_RX UART7 NODMA

# UART8 (spare, low priority, no DMA)
PE1 UART8_TX UART8 NODMA
PE0 UART8_RX UART8 NODMA

# Serial defaults
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_MSP_DisplayPort
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL6_BAUD 115
define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_None

# ---------------------------------------------------------------
# I2C buses
# I2C2 = external (GPS/compass connector)
# I2C1 = internal (barometer)
# ---------------------------------------------------------------
I2C_ORDER I2C2 I2C1

PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1

# ---------------------------------------------------------------
# SPI buses
# ---------------------------------------------------------------
# SPI1: IMU1 (ICM-42688-P)
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1

# SPI2: OSD (MAX7456 / AT7456E)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3: dataflash (W25N01G / W25Q128)
PC10 SPI3_SCK  SPI3
PC11 S