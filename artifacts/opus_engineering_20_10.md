# ArduPilot hwdef for a Generic STM32H743VIT6 Nano AIO Flight Controller

**Important caveats before use:**

1. **This is not a real board's file.** You did not name a specific board, so this is a plausible, internally consistent definition. Every pin must be checked against your actual schematic before flashing. A wrong pin can short a GPIO or leave the board unbootable. I don't know what "Cycle 20" refers to. If it is a hardware revision, compare it against that revision's schematic.
2. **Don't hand-write DMA channels.** ArduPilot's `chibios_hwdef.py` script assigns DMA streams automatically from the peripherals you declare. You control allocation only through `DMA_PRIORITY` and `DMA_NOSHARE`. Hard-coded stream numbers are ignored or cause conflicts. Below, the hwdef shows the correct DMA directives, and a later section explains how to inspect the generated allocation.
3. **Pick a unique board ID.** `APJ_BOARD_ID` must be unique. Register one in `Tools/AP_Bootloader/board_types.txt` before distributing firmware.

---

## `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`

```text
# hw definition file for generic NanoH743AIO (STM32H743VIT6, LQFP100)
# AIO: ICM42688 IMU, DPS310 baro, AT7456E OSD, W25Q128 flash,
#      8x DShot (bidir), FDCAN1 DroneCAN, NeoPixel LED

# ---------------- MCU ----------------
MCU STM32H7xx STM32H743xx
FLASH_SIZE_KB 2048
FLASH_RESERVE_START_KB 128      # bootloader size
STORAGE_FLASH_PAGE 14           # params in flash (no FRAM)
define HAL_STORAGE_SIZE 32768

APJ_BOARD_ID 0xFFFF             # PLACEHOLDER - register a real ID!
OSCILLATOR_HZ 8000000           # verify crystal (8 or 16 MHz typical)

define STM32_ST_USE_TIMER 12
define CH_CFG_ST_RESOLUTION 16

# ---------------- USB ----------------
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
USB_STRING_MANUFACTURER "ArduPilot"
USB_STRING_PRODUCT "NanoH743AIO"

# ---------------- Serial ----------------
# SERIAL0=USB, 1..7 = UARTs
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7 UART8 OTG2

PA9  USART1_TX USART1
PA10 USART1_RX USART1

PD5  USART2_TX USART2
PD6  USART2_RX USART2

PD8  USART3_TX USART3
PD9  USART3_RX USART3

PA0  UART4_TX UART4
PA1  UART4_RX UART4

PC6  USART6_TX USART6           # RC input (CRSF/ELRS/SBUS via inverter)
PC7  USART6_RX USART6

PE8  UART7_TX UART7
PE7  UART7_RX UART7

PE1  UART8_TX UART8             # ESC telemetry
PE0  UART8_RX UART8

# Default serial protocols
define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_MAVLink2
define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_GPS
define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_None
define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_RCIN
define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_None
define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_ESCTelemetry
define DEFAULT_SERIAL7_BAUD 115

# ---------------- SPI buses ----------------
# SPI1 - IMU
PA5  SPI1_SCK  SPI1
PA6  SPI1_MISO SPI1
PA7  SPI1_MOSI SPI1
PA4  GYRO1_CS CS
PB2  GYRO1_DRDY INPUT           # optional, verify routing

# SPI2 - OSD
PB13 SPI2_SCK  SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2
PB12 OSD1_CS CS

# SPI4 - dataflash
PE12 SPI4_SCK  SPI4
PE13 SPI4_MISO SPI4
PE14 SPI4_MOSI SPI4
PE11 FLASH1_CS CS

# ---------------- SPI device table ----------------
SPIDEV icm42688   SPI1 DEVID1 GYRO1_CS  MODE3  2*MHZ  16*MHZ
SPIDEV osd        SPI2 DEVID1 OSD1_CS   MODE0 10*MHZ  10*MHZ
SPIDEV dataflash  SPI4 DEVID1 FLASH1_CS MODE3 104*MHZ 104*MHZ

# ---------------- IMU ----------------
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180   # verify orientation
define HAL_DEFAULT_INS_FAST_SAMPLE 1

# ---------------- I2C ----------------
# I2C1 external (compass / airspeed), I2C2 internal baro
PB8  I2C1_SCL I2C1
PB9  I2C1_SDA I2C1
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2
I2C_ORDER I2C2 I2C1

# ---------------- Baro / compass ----------------
BARO DPS310 I2C:0:0x76
define HAL_PROBE_EXTERNAL_I2C_COMPASSES
define HAL_I2C_INTERNAL_MASK 1
define ALLOW_ARM_NO_COMPASS

# ---------------- OSD ----------------
define OSD_ENABLED 1
define HAL_OSD_TYPE_DEFAULT 1
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin

# ---------------- Dataflash logging ----------------
define HAL_LOGGING_DATAFLASH_ENABLED 1
define HAL_OS_FATFS_IO 0

# ---------------- ADC ----------------
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 RSSI_IN ADC1
define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT_VOLT_SCALE 11.0       # verify divider
define HAL_BATT_CURR_SCALE 40.0       # verify shunt/sensor
define BOARD_RSSI_ANA_PIN 4

# ---------------- Motors / PWM ----------------
PB0  TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1  TIM3_CH4 TIM3 PWM(2) GPIO(51)
PA2  TIM5_CH3 TIM5 PWM(3) GPIO(52) BIDIR
PA3  TIM5_CH4 TIM5 PWM(4) GPIO(53)
PD12 TIM4_CH1 TIM4 PWM(5) GPIO(54) BIDIR
PD13 TIM4_CH2 TIM4 PWM(6) GPIO(55)
PD14 TIM4_CH3 TIM4 PWM(7) GPIO(56) BIDIR
PD15 TIM4_CH4 TIM4 PWM(8) GPIO(57)

# NeoPixel / serial LED
PA8  TIM1_CH1 TIM1 PWM(9) GPIO(58)

define HAL_WITH_DSP 1
define DEFAULT_SERIAL_LED_TYPE 1

# ---------------- DMA policy ----------------
# Streams are auto-assigned; these directives steer allocation.
DMA_PRIORITY SPI1* TIM3* TIM5* TIM4* SPI4*
DMA_NOSHARE  SPI1* TIM3_UP TIM5_UP TIM4_UP

# ---------------- DroneCAN (FDCAN1) ----------------
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW   # transceiver STB/S pin

define HAL_NUM_CAN_IFACES 1
define HAL_CAN_POOL_SIZE 6000

# ---------------- LEDs / buzzer / GPIO ----------------
PE3 LED0 OUTPUT LOW GPIO(90)     # blue
PE4 LED1 OUTPUT LOW GPIO(91)     # green
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_LED_OFF 1

PA15 TIM2_CH1 TIM2 GPIO(32) ALARM

PC13 PINIO1 OUTPUT GPIO(81) LOW  # VTX power switch
PC14 PINIO2 OUTPUT GPIO(82) LOW  # camera switch
define RELAY1_PIN_DEFAULT 81
define RELAY2_PIN_DEFAULT 82

# ---------------- Features ----------------
define HAL_FRAME_TYPE_DEFAULT 1
define AP_OPTICALFLOW_ENABLED 1
```

---

## `hw