/****************************************************************************
 * system/gpio/gpio_status.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <arch/board/board.h>
#include <arch/chip/pin.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pinlist_s
{
  uint32_t pin;
  const char *name;
}
g_pinlist[] =
{
#define _P(p) {(p), (#p)}
  //_P(PIN_RTC_CLK_IN),
  _P(PIN_I2C4_BCK),
  _P(PIN_I2C4_BDT),
  _P(PIN_PMIC_INT),
  _P(PIN_RTC_IRQ_OUT),
  _P(PIN_AP_CLK),
  _P(PIN_GNSS_1PPS_OUT),
  _P(PIN_SPI0_CS_X),
  _P(PIN_SPI0_SCK),
  _P(PIN_SPI0_MOSI),
  _P(PIN_SPI0_MISO),
  _P(PIN_SPI1_CS_X),
  _P(PIN_SPI1_SCK),
  _P(PIN_SPI1_IO0),
  _P(PIN_SPI1_IO1),
  _P(PIN_SPI1_IO2),
  _P(PIN_SPI1_IO3),
  _P(PIN_SPI2_CS_X),
  _P(PIN_SPI2_SCK),
  _P(PIN_SPI2_MOSI),
  _P(PIN_SPI2_MISO),
  _P(PIN_HIF_IRQ_OUT),
  _P(PIN_HIF_GPIO0),
  _P(PIN_SEN_IRQ_IN),
  _P(PIN_SPI3_CS0_X),
  _P(PIN_SPI3_CS1_X),
  _P(PIN_SPI3_CS2_X),
  _P(PIN_SPI3_SCK),
  _P(PIN_SPI3_MOSI),
  _P(PIN_SPI3_MISO),
  _P(PIN_I2C0_BCK),
  _P(PIN_I2C0_BDT),
  _P(PIN_PWM0),
  _P(PIN_PWM1),
  _P(PIN_PWM2),
  _P(PIN_PWM3),
  _P(PIN_IS_CLK),
  _P(PIN_IS_VSYNC),
  _P(PIN_IS_HSYNC),
  _P(PIN_IS_DATA0),
  _P(PIN_IS_DATA1),
  _P(PIN_IS_DATA2),
  _P(PIN_IS_DATA3),
  _P(PIN_IS_DATA4),
  _P(PIN_IS_DATA5),
  _P(PIN_IS_DATA6),
  _P(PIN_IS_DATA7),
  _P(PIN_UART2_TXD),
  _P(PIN_UART2_RXD),
  _P(PIN_UART2_CTS),
  _P(PIN_UART2_RTS),
  _P(PIN_SPI4_CS_X),
  _P(PIN_SPI4_SCK),
  _P(PIN_SPI4_MOSI),
  _P(PIN_SPI4_MISO),
  _P(PIN_EMMC_CLK),
  _P(PIN_EMMC_CMD),
  _P(PIN_EMMC_DATA0),
  _P(PIN_EMMC_DATA1),
  _P(PIN_EMMC_DATA2),
  _P(PIN_EMMC_DATA3),
  _P(PIN_SDIO_CLK),
  _P(PIN_SDIO_CMD),
  _P(PIN_SDIO_DATA0),
  _P(PIN_SDIO_DATA1),
  _P(PIN_SDIO_DATA2),
  _P(PIN_SDIO_DATA3),
  _P(PIN_SDIO_CD),
  _P(PIN_SDIO_WP),
  _P(PIN_SDIO_CMDDIR),
  _P(PIN_SDIO_DIR0),
  _P(PIN_SDIO_DIR1_3),
  _P(PIN_SDIO_CLKI),
  _P(PIN_I2S0_BCK),
  _P(PIN_I2S0_LRCK),
  _P(PIN_I2S0_DATA_IN),
  _P(PIN_I2S0_DATA_OUT),
  _P(PIN_I2S1_BCK),
  _P(PIN_I2S1_LRCK),
  _P(PIN_I2S1_DATA_IN),
  _P(PIN_I2S1_DATA_OUT),
  _P(PIN_MCLK),
  _P(PIN_PDM_CLK),
  _P(PIN_PDM_IN),
  _P(PIN_PDM_OUT),
  _P(PIN_USB_VBUSINT),
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void gpio_status_pin(uint32_t pin, const char *name)
{
  bool input = false;
  bool output = false;
  bool drive = false;
  int  pull;
  int  mode;
  int  intmode;
  char *pud;
  int irq;
  char *type;
  bool filter = false;
  bool enabled = false;

  mode = board_gpio_status(pin, &input, &output, &drive, &pull);

  if (mode < 0)
    {
      return;
    }

  switch (pull)
    {
      case PIN_PULLUP:
        pud = "PU";
        break;
      case PIN_PULLDOWN:
        pud = "PD";
        break;
      case PIN_BUSKEEPER:
        pud = "BK";
        break;
      default:
      case PIN_FLOAT:
        pud = "--";
        break;
    }

  irq = board_gpio_intstatus(pin, &intmode, &filter, &enabled);

  switch (intmode)
    {
      case INT_HIGH_LEVEL:
        type = "High";
        break;
      case INT_LOW_LEVEL:
        type = "Low";
        break;
      case INT_RISING_EDGE:
        type = "Rise";
        break;
      case INT_FALLING_EDGE:
        type = "Fall";
        break;
      case INT_BOTH_EDGE:
        type = "Both";
        break;
      default:
        type = "";
        break;
    }

  printf("(%3ld)%-17s : %-4d %c/%c %-2d %-4s %-4d %-3d %-4s %-2s %-2s\n",
         pin, name, mode,
         (input) ? 'I' : ' ',
         (output) ? 'O' : ' ',
         (drive) ? 4 : 2,
         pud,
         board_gpio_read(pin),
         (irq > 0) ? irq : -1,
         (irq > 0) ? type : "",
         (filter) ? "NF" : "",
         (enabled) ? "EN" : "");
  return;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int gpio_status(int argc, FAR char *argv[])
{
  int i;
  bool all = true;
  uint32_t fpin;
  uint32_t epin;

  if (argc > 2)
    {
      fpin = strtoul(argv[2], NULL, 10);
      epin = fpin;

      if (argc > 3)
        {
          epin = strtoul(argv[3], NULL, 10);
        }
      all = false;
    }

  printf("-------------------------------------------------------------\n");
  printf("( No)PIN NAME          : Mode I/O mA Pull Read IRQ Type NF EN\n");
  printf("-------------------------------------------------------------\n");

  for (i = 0; i < sizeof(g_pinlist) / sizeof(g_pinlist[0]); i++)
    {
      if (all || ((fpin <= g_pinlist[i].pin) && (g_pinlist[i].pin <= epin)))
        {
          gpio_status_pin(g_pinlist[i].pin, g_pinlist[i].name);
        }
    }
  return OK;
}

