/****************************************************************************
 * bsp/board/spresense/include/board.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __BSP_BOARD_SPRESENSE_INCLUDE_BOARD_H
#define __BSP_BOARD_SPRESENSE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <sys/boardctl.h>

#if defined(CONFIG_BOARDCTL_IOCTL) && defined(CONFIG_USBDEV)
#  include <arch/chip/usbdev.h>
#endif

#include <arch/board/common/cxd56_gpioif.h>
#include <arch/board/common/cxd56_power.h>
#include <arch/board/common/cxd56_clock.h>
#include <arch/board/common/cxd56_audio.h>
#include <arch/board/common/cxd56_flash.h>
#include <arch/board/common/cxd56_sdcard.h>
#include <arch/board/common/cxd56_spisd.h>
#include <arch/board/common/cxd56_emmcdev.h>
#include <arch/board/common/cxd56_pwm.h>
#include <arch/board/common/cxd56_sensors.h>
#include <arch/board/common/cxd56_isx012.h>
#include <arch/board/common/cxd56_gauge.h>
#include <arch/board/common/cxd56_charger.h>
#include <arch/board/common/cxd56_bcm20706.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking ****************************************************************/

#ifdef CONFIG_CXD56_80MHz
#  define BOARD_FCLKOUT_FREQUENCY   (80000000)
#else
#  define BOARD_FCLKOUT_FREQUENCY   (100000000)
#endif

/* UART clocking ***********************************************************/
/* Configure all UARTs to use the XTAL input frequency */

#define BOARD_UART0_BASEFREQ        CONFIG_CXD56_XOSC_CLOCK
#define BOARD_UART1_BASEFREQ        BOARD_FCLKOUT_FREQUENCY
#define BOARD_UART2_BASEFREQ        CONFIG_CXD56_XOSC_CLOCK

/* LCD Display clocking ****************************************************/

#define ILI9340_SPI_MAXFREQUENCY    40000000

/* LED definitions *********************************************************/

#define BOARD_LED1          (0)
#define BOARD_LED2          (1)
#define BOARD_NLEDS         (2)

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT      (1 << BOARD_LED1)
#define BOARD_LED2_BIT      (1 << BOARD_LED2)

/* LED pattern for use with board_autoled_on() and board_autoled_off()
 *           ON            OFF
 *       LED1   LED2   LED1   LED2
 * PTN0: OFF    OFF     -      -
 * PTN1: ON     OFF     -      -
 * PTN2: -      ON      -      OFF
 *
 */

#define LED_AUTOLED_PTN0    (0)
#define LED_AUTOLED_PTN1    (1)
#define LED_AUTOLED_PTN2    (2)

#define LED_STARTED         (LED_AUTOLED_PTN0)
#define LED_HEAPALLOCATE    (LED_AUTOLED_PTN1)
#define LED_IRQSENABLED     (LED_AUTOLED_PTN1)
#define LED_STACKCREATED    (LED_AUTOLED_PTN1)
#define LED_INIRQ           (LED_AUTOLED_PTN2)
#define LED_SIGNAL          (LED_AUTOLED_PTN2)
#define LED_ASSERTION       (LED_AUTOLED_PTN2)
#define LED_PANIC           (LED_AUTOLED_PTN2)

/* Buttons definitions *****************************************************/

#define BOARD_NUM_BUTTONS   (2)

/* Power Control definitions ***********************************************/

/*
 *   For SPRESENSE board:
 *
 *     Switch    Device
 *     --------- -------------------------------
 *     LDO_EMMC  GNSS A-ANT
 *     DDC_ANA   N/A
 *     LDO_PERI  N/A
 *     LSW2      CXD5247 Audio Digital VDD
 *     LSW3      SPI-Flash
 *     LSW4      TCXO & GNSS LNA
 *     GPO0
 *     GPO1      CXD5247 Audio Analog VDD
 *     GPO2
 *     GPO3
 *     GPO4      Camera
 *     GPO5      Camera
 *     GPO6      Audio External Amp.
 *     GPO7      Camera
 *
 */

enum board_power_device {

  /* DDC/LDO */

  POWER_DDC_IO          = PMIC_DDCLDO(0),
  POWER_LDO_EMMC        = PMIC_DDCLDO(1),
  POWER_DDC_ANA         = PMIC_DDCLDO(2),
  POWER_LDO_ANA         = PMIC_DDCLDO(3),
  POWER_DDC_CORE        = PMIC_DDCLDO(4),
  POWER_LDO_PERI        = PMIC_DDCLDO(5),

  /* Load Switch */

  POWER_AUDIO_DVDD      = PMIC_LSW(2),
  POWER_FLASH           = PMIC_LSW(3),
  POWER_TCXO            = PMIC_LSW(4),
  POWER_LNA             = PMIC_LSW(4),

  /* GPO */

  POWER_AUDIO_AVDD      = PMIC_GPO(1),
  POWER_AUDIO_MUTE      = PMIC_GPO(6),
  POWER_IMAGE_SENSOR    = PMIC_GPO(4) | PMIC_GPO(5) | PMIC_GPO(7),

  POWER_BTBLE           = PMIC_NONE,
  POWER_SENSOR          = PMIC_NONE,
  POWER_EMMC            = PMIC_NONE,
  POWER_LTE             = PMIC_GPO(2),
};

/* Power Off Level definitions *********************************************/

#define BOARD_POWEROFF_DEEP (0)
#define BOARD_POWEROFF_COLD (1)

/* CXD5247 audio control definitions ***************************************/

#define CXD5247_XRST  PIN_SPI3_CS2_X
#define CXD5247_AVDD  (0x01)
#define CXD5247_DVDD  (0x02)

/* Sensor device bus definitions *******************************************/

#define SENSOR_I2C      0
#define SENSOR_SPI      3

/* Display device pin definitions ******************************************/

#if defined(CONFIG_LCD_ON_MAIN_BOARD) /* Display connected to main board. */

#define DISPLAY_RST     PIN_I2S0_BCK
#define DISPLAY_DC      PIN_I2S0_LRCK

#define DISPLAY_SPI     5

#else /* Display is connected through extension board. */

#define DISPLAY_RST     PIN_SPI2_MISO
#define DISPLAY_DC      PIN_PWM2

#define DISPLAY_SPI     4

#endif

/* Imager device pin definitions *******************************************/

#define IMAGER_RST      PIN_SDIO_DIR1_3
#define IMAGER_SLEEP    PIN_SDIO_DIR0

#define IMAGER_I2C      2

/*
 * Set signal id for notify USB device connection status and supply current value.
 * signal returns "usbdev_notify_s" struct pointer in sival_ptr.
 *
 * Arg: Value of sinal number
 */

#define BOARDIOC_USBDEV_SETNOTIFYSIG      (BOARDIOC_USER+0x0001)

#endif  /* __BSP_BOARD_SPRESENSE_INCLUDE_BOARD_H */
