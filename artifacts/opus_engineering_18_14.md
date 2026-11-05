# ArduPilot hwdef for a Generic STM32H743VIT6 Nano AIO Board

You didn't provide a schematic, so this file uses a **reference pinout** I designed for this MCU. It is internally consistent:

- Every pin exists on the LQFP100 (VIT6) package.
- Every alternate function and timer mapping is valid for the H743.
- No pin, DMA, or timer conflicts exist.

**Before you flash anything, compare every pin against your actual schematic.** A wrong pin assignment can damage ESCs or the MCU. I wasn't sure what "Cycle 18" refers to. If it's a revision label for your project, put it in the header comment.

---

## Important note on DMA

On STM32H7, ArduPilot **does not take hand-written DMA stream/channel tables**. The H7 has a DMAMUX, and the hwdef build script (`chibios_hwdef.py` / `dma_resolver.py`) assigns DMA1/DMA2 streams automatically. You control DMA with:

- `DMA_PRIORITY`: which peripherals get first pick of streams.
- `DMA_NOSHARE`: which peripherals must never share a stream.

The resolved table is written to `build/<board>/hwdef.h`. Check it there after building. Hand-coding stream numbers would be ignored or would break the build.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```text
# hw definition file for NanoH743AIO - STM32H743VIT6 nano AIO flight controller
# Revision: Cycle 18
# 30.5/25.5mm AIO: ICM42688P IMU, DPS310 baro, MAX7456 OSD,
# 16MB SPI dataflash, 8 motor outputs, DroneCAN on FDCAN1

# ---------------------------------------------------------------
# MCU
# ---------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Board ID - placeholder. You MUST request an official ID
# (PR to Tools/AP_Bootloader/board_types.txt) before distribution.
APJ_BOARD_ID TARGET_HW_NANOH743AIO

# 8 MHz crystal on PH0/PH1
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048

# 128 KB bootloader at start of flash
FLASH_RESERVE_START_KB 128

# Parameter storage in internal flash (sectors 14/15)
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 32768

# ---------------------------------------------------------------
# Clocks / debug
# ---------------------------------------------------------------
PH0 RCC_OSC_IN OUTPUT
PH1 RCC_OSC_OUT OUTPUT

PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------------------------------------------------------
# USB
# ---------------------------------------------------------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# ---------------------------------------------------------------
# Serial ports
# SERIAL0=USB, 1=USART1, 2=USART2(RX in), 3=USART3(GPS),
# 4=UART4, 5=USART6, 6=UART7(ESC telem), 7=UART8
# ---------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USART1 - telem1
PB6 USART1_TX USART1
PB7 USART1_RX USART1

# USART2 - RC input (CRSF/ELRS/SBUS)
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - GPS
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - user / VTX
PA0 UART4_TX UART4
PA1 UART4_RX UART4

# USART6 - user / DJI / MSP
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 - ESC telemetry (RX only used)
PE8 UART7_TX UART7
PE7 UART7_RX UART7

# UART8 - user
PE1 UART8_TX UART8
PE0 UART8_RX UART8

# Default protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_None
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_MSP_DisplayPort
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_None

# No separate RCIN timer pin - RC is via serial
define HAL_NO_RCIN_THREAD 0

# ---------------------------------------------------------------
# SPI buses
# SPI1 = IMU, SPI2 = OSD, SPI3 = dataflash
# ---------------------------------------------------------------
# SPI1 - ICM42688P
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1
PA4 IMU1_CS   CS
PC4 IMU1_DRDY INPUT

# SPI2 - MAX7456 OSD
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2
PB12 OSD1_CS   CS

# SPI3 - W25N/W25Q dataflash
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3
PA15 FLASH1_CS CS

# SPI device table
#        name       bus  devid    CS        mode  lowspeed  highspeed
SPIDEV icm42688   SPI1 DEVID1 IMU1_CS   MODE3  2*MHZ   16*MHZ
SPIDEV osd        SPI2 DEVID1 OSD1_CS   MODE0 10*MHZ   10*MHZ
SPIDEV dataflash  SPI3 DEVID1 FLASH1_CS MODE3 104*MHZ 104*MHZ

# ---------------------------------------------------------------
# I2C
# I2C1 = onboard baro + external compass on GPS connector
# ---------------------------------------------------------------
I2C_ORDER I2C1
PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1

# ---------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
define HAL_DEFAULT_INS_FAST_SAMPLE 1

BARO DPS310 I2C:0:0x76

# No onboard compass - probe external I2C compasses
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 0
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ---------------------------------------------------------------
# DMA control (DMAMUX - streams auto-allocated)
# ---------------------------------------------------------------
DMA_PRIORITY SPI1* TIM1* TIM3* SPI3* USART2*
DMA_NOSHARE  SPI1* TIM1* TIM3*

# ---------------------------------------------------------------
# Motor outputs (DShot / BiDir DShot capable)
# TIM1 -> M1-M4, TIM3 -> M5-M8
# ---------------------------------------------------------------
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50) BIDIR
PE11 TIM1_CH2 TIM1 PWM(2) GPIO(51)
PE13 TIM1_CH3 TIM1 PWM(3) GPIO(52) BIDIR
PE14 TIM1_CH4 TIM1 PWM(4) GPIO(53)

PB0  TIM3_CH3 TIM3 PWM(5) GPIO(54) BIDIR
PB1  TIM3_CH4 TIM3 PWM(6) GPIO(55)
PB4  TIM3_CH1 TIM3 PWM(7) GPIO(56) BIDIR
PB5  TIM3_CH2 TIM3 PWM(8) GPIO(57)

# WS2