/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "CBCF"
#define USBD_PRODUCT_STRING     "CrazyBee F3 CRSF"

#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_ON

#define LED0_PIN                PB3
#define USE_BEEPER 
#define BEEPER_PIN              PC15
#define BEEPER_INVERTED

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC13
#define USE_MPU_DATA_READY_SIGNAL
#define GYRO_1_SPI_INSTANCE     SPI1
#define GYRO_1_CS_PIN           PA4
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_1_ALIGN            CW90_DEG
#define USE_ACC
#define USE_ACC_SPI_MPU6000

#define USE_VCP
#define USE_UART3
#define SERIAL_PORT_COUNT       3
#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#undef USE_RX_SPI
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_CRSF
#define SERIALRX_UART           SERIAL_PORT_USART3

#define DEFAULT_FEATURES        (FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_SOFTSERIAL)

#define USE_SOFTSERIAL1

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI1
#define MAX7456_SPI_CS_PIN      PB1

#define USE_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define VBAT_ADC_PIN            PA0
#define CURRENT_METER_ADC_PIN   PA1
#define ADC_INSTANCE            ADC1
#define CURRENT_METER_SCALE_DEFAULT 2350

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))
#define USABLE_TIMER_CHANNEL_COUNT 6
#define USED_TIMERS             (TIM_N(2) |TIM_N(3) |TIM_N(4) | TIM_N(8))

#undef USE_BLACKBOX
#undef USE_SDCARD
#undef USE_SDCARD_SPI
#undef USE_GPS
#define USE_VIRTUAL_CURRENT_METER
#define USE_CRSF_CMS_TELEMETRY
#define USE_MSP_DISPLAYPORT
#define USE_OSD_OVER_MSP_DISPLAYPORT
#define USE_VTX_TRAMP
#define USE_EXTENDED_CMS_MENUS
#define USE_RCDEVICE
#define USE_ESC_SENSOR_INFO
#define USE_BOARD_INFO
#define USE_OSD_PROFILES

#undef PID_PROFILE_COUNT
#undef CONTROL_RATE_PROFILE_COUNT
#define PID_PROFILE_COUNT 2
#define CONTROL_RATE_PROFILE_COUNT  3

#define USE_SERIALRX_CRSF
#define USE_TELEMETRY_CRSF
#define USE_CRSF_LINK_STATISTICS
#define USE_RX_RSSI_DBM
#define USE_RX_LINK_QUALITY_INFO
#define USE_BATTERY_VOLTAGE_SAG_COMPENSATION
#define USE_SERIAL_PASSTHROUGH
#define USE_PWM_OUTPUT
#define USE_CMS
#define USE_OSD
#define USE_CANVAS
#define USE_OSD_ADJUSTMENTS
#define USE_VTX_COMMON
#define USE_VTX_CONTROL
#define USE_VTX_SMARTAUDIO
#define USE_ESC_SENSOR
#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#define USE_GYRO_LPF2
#define USE_DYN_LPF
#define USE_THROTTLE_BOOST
#define USE_INTEGRATED_YAW_CONTROL
#define USE_ITERM_RELAX
#define USE_RC_SMOOTHING_FILTER
#define USE_THRUST_LINEARIZATION
#define USE_TPA_MODE
#define USE_HUFFMAN
#define USE_AIRMODE_LPF
#define USE_VTX_TABLE

#undef USE_CAMERA_CONTROL
#undef USE_BARO
#undef USE_PWM
#undef USE_PPM
#undef USE_SERVOS
#undef USE_LED_STRIP
#undef USE_LAUNCH_CONTROL
#undef USE_D_MIN
#undef USE_PIN_PULL_UP_DOWN
#undef USE_RTC_TIME
#undef USE_DASHBOARD
#undef USE_FRSKYOSD
#undef USE_MULTI_GYRO
#undef USE_SENSOR_NAMES
#undef USE_ESC_SENSOR_TELEMETRY
#undef USE_MSP_OVER_TELEMETRY
#undef USE_OSD_STICK_OVERLAY
#undef USE_ESCSERIAL_SIMONK
#undef USE_SERIAL_4WAY_SK_BOOTLOADER
#undef USE_TELEMETRY_SENSORS_DISABLED_DETAILS
#undef USE_PINIO
#undef USE_PINIOBOX
#undef USE_TRANSPONDER

#undef USE_RX_MSP
#undef USE_SERIALRX_SUMD
#undef USE_SERIALRX_SUMH
#undef USE_SERIALRX_XBUS
#undef USE_SERIALRX_FPORT
#undef USE_SERIALRX_SPEKTRUM
#undef USE_SERIALRX_SBUS
#undef USE_SERIALRX_IBUS
#undef USE_SERIALRX_JETIEXBUS
#undef USE_SERIALRX_SRXL2
#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_LTM
#undef USE_TELEMETRY_MAVLINK
#undef USE_TELEMETRY_FRSKY_HUB
#undef USE_TELEMETRY_SMARTPORT
#undef USE_TELEMETRY_SRXL
#undef USE_TELEMETRY_IBUS
#undef USE_TELEMETRY_IBUS_EXTENDED
#undef USE_TELEMETRY_JETIEXBUS