# ArduPilot `hwdef.dat` for a Nano AIO STM32H743VIT6 Flight Controller

This is a complete, buildable reference `hwdef.dat` for a generic "Nano AIO" board on the STM32H743VIT6 (LQFP100, ports A–E only). Every pin and alternate function below is valid for the H743 in this package. Before using it on real hardware, check it against your own schematic.

> **Important note on DMA:** ArduPilot hwdefs do **not** assign DMA streams by hand. On the H7, the DMAMUX lets any peripheral use any stream. `dma_resolver.py` allocates streams automatically at build time. You control allocation only with `DMA_PRIORITY`, `DMA_NOSHARE`, and `NODMA`. The resolved map is written to `build/<board>/hwdef.h`, and a typical result is shown at the end.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoAIO-H743/hwdef.dat`

```
# hw definition file for NanoAIO-H743 (STM32H743VIT6, LQFP100)
# Reference design: dual IMU, DPS310 baro, 1Gbit NAND logging,
# analog OSD (AT7456E), 8 bidir-DShot outputs, 1x DroneCAN (FDCAN1)

# ------------------------------------------------------------------
# MCU / clock / board identity
# ------------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Placeholder ID - request an official one via ArduPilot board_types.txt PR
APJ_BOARD_ID 5600
USB_STRING_MANUFACTURER "ArduPilot"
USB_STRING_PRODUCT "NanoAIO-H743"

# 8MHz HSE crystal
OSCILLATOR_HZ 8000000

# 2MB flash, first 128k reserved for bootloader
FLASH_SIZE_KB 2048
FLASH_RESERVE_START_KB 128

# Parameter storage in internal flash (no FRAM on AIO)
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 32768

# ------------------------------------------------------------------
# Debug / USB
# ------------------------------------------------------------------
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9  VBUS INPUT OPENDRAIN

# ------------------------------------------------------------------
# Serial port order
# SERIAL0 = USB, SERIAL1..7 = UARTs
# ------------------------------------------------------------------
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USART1 - SERIAL1 (Telem1 / MAVLink)
PB6 USART1_TX USART1
PB7 USART1_RX USART1

# USART2 - SERIAL2 (GPS)
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - SERIAL3 (DJI / MSP DisplayPort)
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - SERIAL4 (ESC telemetry, RX only used)
PA0 UART4_TX UART4
PA1 UART4_RX UART4

# USART6 - SERIAL5 (RC input: CRSF/ELRS/SBUS)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 - SERIAL6 (VTX control)
PE8 UART7_TX UART7
PE7 UART7_RX UART7

# UART8 - SERIAL7 (spare)
PE1 UART8_TX UART8
PE0 UART8_RX UART8

define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_DJI_FPV
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_Tramp
define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_None

# Serial RC only, no dedicated timer RCIN pin
define HAL_SERIAL5_BAUD 115

# ------------------------------------------------------------------
# SPI buses
# ------------------------------------------------------------------
# SPI1 - IMU1
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1

# SPI2 - OSD (AT7456E)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - NAND dataflash
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# SPI4 - IMU2
PE2 SPI4_SCK  SPI4
PE5 SPI4_MISO SPI4
PE6 SPI4_MOSI SPI4

# Chip selects
PA4  GYRO1_CS   CS
PE4  GYRO2_CS   CS
PB12 OSD1_CS    CS
PA15 FLASH1_CS  CS

# IMU data-ready (optional, wired for future use)
PB2  GYRO1_DRDY INPUT
PE3  GYRO2_DRDY INPUT

# ------------------------------------------------------------------
# SPI device table
# SPIDEV name bus devid cs mode lowspeed highspeed
# ------------------------------------------------------------------
SPIDEV imu1      SPI1 DEVID1 GYRO1_CS  MODE3  2*MHZ 16*MHZ
SPIDEV imu2      SPI4 DEVID1 GYRO2_CS  MODE3  1*MHZ  4*MHZ
SPIDEV osd       SPI2 DEVID4 OSD1_CS   MODE0 10*MHZ 10*MHZ
SPIDEV dataflash SPI3 DEVID1 FLASH1_CS MODE3 32*MHZ 32*MHZ

# ------------------------------------------------------------------
# IMUs
# IMU1 ICM-42688-P, IMU2 BMI270 (adjust rotations to PCB layout)
# ------------------------------------------------------------------
IMU Invensensev3 SPI:imu1 ROTATION_YAW_180
IMU BMI270       SPI:imu2 ROTATION_YAW_270

define HAL_DEFAULT_INS_FAST_SAMPLE 3

# ------------------------------------------------------------------
# I2C buses
# ------------------------------------------------------------------
# I2C1 - external (GPS/compass module)
PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1

# I2C2 - internal baro
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

I2C_ORDER I2C2 I2C1

# Baro
BARO DPS310 I2C:0:0x76

# No onboard compass - probe external buses
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 1
define HAL_COMPASS_AUTO_ROT_DEFAULT 2

# ------------------------------------------------------------------
# DroneCAN (FDCAN1 -> onboard transceiver, e.g. TJA1051T/3)
# ------------------------------------------------------------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

# CAN interface count is derived from CAN pins; set default driver/protocol
define HAL_CAN_DEFAULT_NODE_ID 0
define HAL_ENABLE_DRONECAN_DRIVERS 1

# ------------------------------------------------------------------
# Motor / servo outputs (bidirectional DShot capable)
# TIM1 and TIM4 each own an UP DMA request for DShot
# ------------------------------------------------------------------
PE9  TIM1_CH1 TIM1 PWM(1) GPIO(50) BIDIR
PE11 TIM1_CH2 TIM