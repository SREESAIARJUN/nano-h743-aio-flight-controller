# ArduPilot hwdef for an STM32H743VIT6 Nano AIO Flight Controller

## Before you use this

- **This is a reference design, not a drop-in file.** "STM32H743VIT6 nano AIO" doesn't identify one specific board. The pinout below is a complete, internally consistent design for the H743 LQFP100 package. Every pin must still be checked against your schematic before flashing.
- **"Cycle 25" is used only as a revision label.** It doesn't correspond to any ArduPilot convention I know of, so it appears only in the header comment.
- **DMA streams are not hand-assigned in ArduPilot hwdef files.** The H7 has a DMAMUX, and `chibios_hwdef.py` allocates streams automatically. You control allocation with `DMA_PRIORITY` and `DMA_NOSHARE`. After a build, the actual stream map is in `build/<board>/hwdef.h`.
- **`APJ_BOARD_ID` must be unique.** Request an official ID before distributing firmware.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```
# hw definition file for NanoH743AIO (STM32H743VIT6, LQFP100)
# Revision label: Cycle 25

# ---------------- MCU ----------------
MCU STM32H7xx STM32H743xx

# Placeholder ID - obtain an official one
APJ_BOARD_ID 5925

# Crystal
OSCILLATOR_HZ 8000000

# Flash: 2MB, first 128k reserved for the bootloader
FLASH_SIZE_KB 2048
FLASH_RESERVE_START_KB 128

# Parameter storage in flash
STORAGE_FLASH_PAGE 1
define HAL_STORAGE_SIZE 32768

# System timer (TIM12 is not used for anything else)
STM32_ST_USE_TIMER 12
define CH_CFG_ST_RESOLUTION 16

# ---------------- USB ----------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
USB_STRING_MANUFACTURER "ArduPilot"
USB_STRING_PRODUCT "NanoH743AIO"

# ---------------- Serial ports ----------------
# SERIAL0=USB, 1=USART1, 2=USART2, 3=USART3, 4=UART4,
# 5=USART6, 6=UART7, 7=UART8
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8

# USART1 - Telem1 / MAVLink
PA9  USART1_TX USART1
PA10 USART1_RX USART1

# USART2 - GPS
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# USART3 - VTX / MSP DisplayPort
PD8 USART3_TX USART3
PD9 USART3_RX USART3

# UART4 - spare / ESC telemetry
PB9 UART4_TX UART4
PB8 UART4_RX UART4

# USART6 - RC input (CRSF/ELRS/SBUS via serial)
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# UART7 - spare (RX only typically used for ESC telem)
PE8 UART7_TX UART7
PE7 UART7_RX UART7

# UART8 - spare
PE1 UART8_TX UART8
PE0 UART8_RX UART8

# RC input is on a UART (no timer RCIN)
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN

# ---------------- SPI buses ----------------
# SPI1 - IMU1 (ICM-42688-P)
PA5 SPI1_SCK  SPI1
PA6 SPI1_MISO SPI1
PD7 SPI1_MOSI SPI1

# SPI2 - OSD (AT7456E / MAX7456)
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - Dataflash (W25N01G / W25Q128)
PB3 SPI3_SCK  SPI3
PB4 SPI3_MISO SPI3
PB5 SPI3_MOSI SPI3

# SPI4 - IMU2 (BMI270)
PE12 SPI4_SCK  SPI4
PE13 SPI4_MISO SPI4
PE14 SPI4_MOSI SPI4

# Chip selects
PC15 IMU1_CS   CS
PE11 IMU2_CS   CS
PB12 OSD1_CS   CS
PA4  FLASH1_CS CS

# ---------------- SPI device table ----------------
#       name      bus  devid  cs         mode  lowspeed  highspeed
SPIDEV imu1      SPI1 DEVID1 IMU1_CS    MODE3 2*MHZ     16*MHZ
SPIDEV imu2      SPI4 DEVID1 IMU2_CS    MODE3 1*MHZ     4*MHZ
SPIDEV osd       SPI2 DEVID4 OSD1_CS    MODE0 10*MHZ    10*MHZ
SPIDEV dataflash SPI3 DEVID1 FLASH1_CS  MODE3 32*MHZ    32*MHZ

# ---------------- IMUs ----------------
IMU Invensensev3 SPI:imu1 ROTATION_YAW_180
IMU BMI270       SPI:imu2 ROTATION_YAW_270
define HAL_DEFAULT_INS_FAST_SAMPLE 3

# ---------------- I2C ----------------
# I2C1 - internal baro, I2C2 - external mag/airspeed
PB6  I2C1_SCL I2C1
PB7  I2C1_SDA I2C1
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

I2C_ORDER I2C2 I2C1
define HAL_I2C_INTERNAL_MASK 2

# Baro (either part may be fitted)
BARO DPS310 I2C:1:0x76
BARO BMP280 I2C:1:0x76

# Compass: none onboard, probe external
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_COMPASS_AUTO_ROT_DEFAULT 2
define ALLOW_ARM_NO_COMPASS

# ---------------- DroneCAN (FDCAN1) ----------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
# Optional transceiver silent/standby pin (pull LOW = active)
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW GPIO(70)

# ---------------- PWM outputs ----------------
# Group 1 - TIM3
PB0 TIM3_CH3 TIM3 PWM(1) GPIO(50)
PB1 TIM3_CH4 TIM3 PWM(2) GPIO(51)
# Group 2 - TIM5
PA0 TIM5_CH1 TIM5 PWM(3) GPIO(52)
PA1 TIM5_CH2 TIM5 PWM(4) GPIO(53)
PA2 TIM5_CH3 TIM5 PWM(5) GPIO(54)
PA3 TIM5_CH4 TIM5 PWM(6) GPIO(55)
# Group 3 - TIM4
PD12 TIM4_CH1 TIM4 PWM(7) GPIO(56)
PD13 TIM4_CH2 TIM4 PWM(8) GPIO(57)
# Group 4 - TIM1 - LED strip
PA8 TIM1_CH1 TIM1 PWM(9) GPIO(58)

# Bidirectional DShot on the motor groups
define HAL_WITH_BIDIR_DSHOT

# ---------------- ADC ----------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 RSSI_IN           ADC1 SCALE(1)
PC5 PRESSURE_SENS     ADC1 SCALE(2)

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCAL