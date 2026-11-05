# ArduPilot hwdef.dat: STM32H743VIT6 Nano AIO (Rev C23)

This is a reference hardware definition for a generic 20×20 / 25.5×25.5 nano AIO flight controller with these components:

- **MCU:** STM32H743VIT6 (LQFP100)
- **IMU:** ICM-42688-P
- **Baro:** DPS310
- **OSD:** AT7456E
- **Flash:** 16 MB NOR dataflash
- **CAN:** 1× DroneCAN port

> **⚠️ You must verify every pin against your actual schematic before flashing.** Pin assignments vary between boards. The file below is internally consistent and valid for the LQFP100 package (ports A–E only), but it is not tied to a specific commercial board. Also, `APJ_BOARD_ID` must be a registered ID from `Tools/AP_Bootloader/board_types.txt`.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoAIO-H743/hwdef.dat`

```
# hw definition file for NanoAIO-H743 (STM32H743VIT6, LQFP100)
# Hardware revision: C23

# ---------------------------------------------------------------
# MCU
# ---------------------------------------------------------------
MCU STM32H7xx STM32H743xx

# Board ID - MUST be registered in Tools/AP_Bootloader/board_types.txt
# Replace with your allocated ID
APJ_BOARD_ID TARGET_HW_NANOAIO_H743

# 8MHz crystal
OSCILLATOR_HZ 8000000

FLASH_SIZE_KB 2048

# bootloader occupies first 128k sector
FLASH_RESERVE_START_KB 128

# use last 2 sectors for parameter storage (flash storage)
STORAGE_FLASH_PAGE 14
define HAL_STORAGE_SIZE 16384

# ---------------------------------------------------------------
# USB / Serial order
# ---------------------------------------------------------------
# SERIAL0 = USB, then SERIAL1..7
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USB
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# SWD
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# ---------------------------------------------------------------
# UARTs
# ---------------------------------------------------------------
# USART1 - SERIAL1 (GPS)
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - SERIAL2 (Telem / MAVLink)
PD5  USART2_TX USART2
PD6  USART2_RX USART2

# USART3 - SERIAL3 (VTX / MSP DisplayPort)
PD8  USART3_TX USART3
PD9  USART3_RX USART3

# UART4 - SERIAL4 (ESC telemetry, RX only used)
PA0  UART4_TX UART4
PA1  UART4_RX UART4

# USART6 - SERIAL5 (RC input - CRSF/ELRS/SBUS)
PC6  USART6_TX USART6
PC7  USART6_RX USART6

# UART7 - SERIAL6 (spare)
PE8  UART7_TX UART7
PE7  UART7_RX UART7

# UART8 - SERIAL7 (spare)
PE1  UART8_TX UART8
PE0  UART8_RX UART8

# no hardware flow control on any port
define HAL_SERIAL_NO_FLOW_CONTROL

# ---------------------------------------------------------------
# SPI buses
# ---------------------------------------------------------------
# SPI1 - IMU
PA5  SPI1_SCK  SPI1
PA6  SPI1_MISO SPI1
PA7  SPI1_MOSI SPI1

# SPI2 - OSD (AT7456E)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - Dataflash
PC10 SPI3_SCK  SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3

# Chip selects
PA4  IMU1_CS   CS
PB12 OSD_CS    CS
PA15 FLASH_CS  CS

# IMU data-ready (interrupt), optional
PB2  IMU1_DRDY INPUT

# ---------------------------------------------------------------
# SPI device table
# SPIDEV name bus devid cs_pin mode lowspeed highspeed
# ---------------------------------------------------------------
SPIDEV imu1      SPI1 DEVID1 IMU1_CS  MODE3  2*MHZ 16*MHZ
SPIDEV osd       SPI2 DEVID1 OSD_CS   MODE0 10*MHZ 10*MHZ
SPIDEV dataflash SPI3 DEVID1 FLASH_CS MODE3 32*MHZ 32*MHZ

# ---------------------------------------------------------------
# I2C
# ---------------------------------------------------------------
# I2C1 - onboard baro + external compass (shared on pads)
PB8  I2C1_SCL I2C1
PB9  I2C1_SDA I2C1

# I2C2 - external only
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

I2C_ORDER I2C1 I2C2

# ---------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------
# IMU: ICM-42688-P, board rotated - adjust ROTATION to your layout
IMU Invensensev3 SPI:imu1 ROTATION_YAW_180
define HAL_DEFAULT_INS_FAST_SAMPLE 1

# Barometer: DPS310 on I2C1 (onboard)
BARO DPS310 I2C:0:0x76

# Compass: probe external I2C only
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 1
define ALLOW_ARM_NO_COMPASS

# OSD
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# Dataflash logging
define HAL_LOGGING_DATAFLASH_ENABLED 1

# ---------------------------------------------------------------
# PWM / Motor outputs (bidirectional DShot capable)
# ---------------------------------------------------------------
# TIM3 - M1/M2
PB0  TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1  TIM3_CH4 TIM3 PWM(2) GPIO(51)
# TIM4 - M3/M4
PD12 TIM4_CH1 TIM4 PWM(3) GPIO(52) BIDIR
PD13 TIM4_CH2 TIM4 PWM(4) GPIO(53)
# TIM1 - M5..M8 (octo / servo outputs)
PE9  TIM1_CH1 TIM1 PWM(5) GPIO(54) BIDIR
PE11 TIM1_CH2 TIM1 PWM(6) GPIO(55)
PE13 TIM1_CH3 TIM1 PWM(7) GPIO(56) BIDIR
PE14 TIM1_CH4 TIM1 PWM(8) GPIO(57)

# LED strip (WS2812) - SERVO9
PE5  TIM15_CH1 TIM15 PWM(9) GPIO(58)

# ---------------------------------------------------------------
# RC input
# ---------------------------------------------------------------
# RC via USART6 (SERIAL5_PROTOCOL=23). Set in defaults.parm.
# Optional timer RCIN for PPM/SBUS (uncomment if a pad exists):
# PC8 TIM8_CH3 TIM8 RCININT PULLDOWN LOW

# ---------------------------------------------------------------
# DroneCAN (FDCAN1)
# ---------------------------------------------------------------
PD0  CAN1_RX CAN1
PD1  CAN1_TX CAN1

# Transceiver silent/standby pin (LOW = active)
PD3  GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

define HAL_NUM_CAN_IFACES 1
define HAL_CAN_POOL_SIZE 6000

# ---------------------------------------------------------------
# ADC
# ---------------------------------------------------------------
PC0  BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1  BATT_CURRENT_SENS ADC1 SCALE(1)
PC5  RSSI_IN           ADC1 SCALE(1)

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define BOARD_RSSI_ANA_PIN 8

# ---------------------------------------------------------------
# LEDs, buzzer, power switches
# ---------------------------------------------------------------
PE3  LED0 OUTPUT LOW GPIO(90)
PE4  LED1 OUTPUT LOW GPIO(91)
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

# Buzzer (active, driven by GPIO)
PD15 ALARM OUTPUT GPIO(80) LOW
define HAL_BUZZER_PIN 80
define HAL_BUZZER_ON 1
define HAL_BUZZER_OFF 0

# VTX 9V regulator enable (RELAY2)
PC13 PINIO1 OUTPUT GPIO(81) HIGH
# Camera switch (RELAY3)
PC14 PINIO2 OUTPUT GPIO(82) LOW
define RELAY2_PIN_DEFAULT 81
define RELAY3_PIN_DEFAULT 82

# ---------------------------------------------------------------
# DMA
# ---------------------------------------------------------------
# ArduPilot allocates DMA streams automatically via the DMAMUX on H7.
# You set priority and sharing rules here, NOT raw stream numbers.
DMA_PRIORITY TIM3* TIM4* TIM1* SPI1* SPI3*
DMA_NOSHARE  SPI1* TIM3* TIM4*

# ---------------------------------------------------------------
# Feature trimming / defaults
# ---------------------------------------------------------------
define HAL_OS_FATFS_IO 0
define HAL_WITH_DSP 1
define AP_OPTICALFLOW_ENABLED 1
define HAL_FRAME_TYPE_DEFAULT 1   # quad

# default parameters
env DEFAULT_PARAMETERS libraries/AP_HAL_ChibiOS/hwdef/NanoAIO-H743/defaults.parm
```

---

## `hwdef-bl.dat` (bootloader)

```
MCU STM32H7xx STM32H743xx
APJ_BOARD_ID TARGET_HW_NANOAIO_H743
OSCILLATOR_HZ 8000000
FLASH_SIZE_KB 2048
FLASH_RESERVE_START_KB 0
FLASH_BOOTLOADER_LOAD_KB 128

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

# keep CS lines high during boot
PA4  IMU1_CS  CS
PB12 OSD_CS   CS
PA15 FLASH_CS CS
```

---

## `defaults.parm`

```
SERIAL1_PROTOCOL 5      # GPS
SERIAL2_PROTOCOL 2      # MAVLink2
SERIAL3_PROTOCOL 42     # DisplayPort
SERIAL4_PROTOCOL 16     # ESC telemetry
SERIAL5_PROTOCOL 