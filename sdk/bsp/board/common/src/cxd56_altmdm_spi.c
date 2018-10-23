/****************************************************************************
 * bsp/board/common/src/cxd56_altmdm_spi.c
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

#if defined(CONFIG_CXD56_SPI) && defined(CONFIG_MODEM_ALTMDM)

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/modem/altmdm.h>
#include <arch/board/common/cxd56_altmdm.h>
#include "cxd56_spi.h"
#include "cxd56_pinconfig.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static void *g_devhandle = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_pincontrol
 *
 * Description:
 *   Configure the SPI pin
 *
 * Input Parameter:
 *   on - true: enable pin, false: disable pin
 *
 ****************************************************************************/

static void spi_pincontrol(int bus, bool on)
{
  if (bus == 5)
    {
#ifdef CONFIG_CXD56_SPI5_PINMAP_EMMC
      if (on)
        {
          CXD56_PIN_CONFIGS(PINCONFS_EMMCA_SPI5);
        }
      else
        {
          CXD56_PIN_CONFIGS(PINCONFS_EMMCA_GPIO);
        }
#endif /* CONFIG_CXD56_SPI5_PINMAP_EMMC */
#ifdef CONFIG_CXD56_SPI5_PINMAP_SDIO
      if (on)
        {
          CXD56_PIN_CONFIGS(PINCONFS_SDIOA_SPI5);
        }
      else
        {
          CXD56_PIN_CONFIGS(PINCONFS_SDIOA_GPIO);
        }
#endif /* CONFIG_CXD56_SPI5_PINMAP_SDIO */
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_altmdm_initialize
 *
 * Description:
 *   Initialize Altair modem
 *
 ****************************************************************************/

int board_altmdm_initialize(FAR const char *devpath, int bus)
{
  FAR struct spi_dev_s *spi;

  m_info("Initializing ALTMDM..\n");

  if (!g_devhandle)
    {
      /* Initialize spi deivce */

      spi = cxd56_spibus_initialize(bus);
      if (!spi)
        {
          m_err("ERROR: Failed to initialize spi%d.\n", bus);
          return -ENODEV;
        }

      spi_pincontrol(5, false);

      g_devhandle = altmdm_register(devpath, spi);
      if (!g_devhandle)
        {
          m_err("ERROR: Failed to register altmdm driver.\n");
          return -ENODEV;
        }

      board_altmdm_poweroff();
    }

  return OK;
}

/****************************************************************************
 * Name: board_altmdm_uninitialize
 *
 * Description:
 *   Uninitialize Altair modem
 *
 ****************************************************************************/

int board_altmdm_uninitialize(void)
{
  m_info("Uninitializing ALTMDM..\n");

  if (g_devhandle)
    {
      altmdm_unregister(g_devhandle);

      g_devhandle = NULL;
    }

  return OK;
}

/****************************************************************************
 * Name: board_altmdm_power_control
 *
 * Description:
 *   Power on/off the Altair modem device on the board.
 *
 ****************************************************************************/

void board_altmdm_power_control(bool en)
{
  if (en)
    {
      /* power on altair modem device */

      board_altmdm_poweron();

      /* enable the SPI pin */

      spi_pincontrol(5, true);
    }
  else
    {
      /* disable the SPI pin */

      spi_pincontrol(5, false);

      /* power off Altair modem device */

      board_altmdm_poweroff();

    }
}

#endif /* CONFIG_CXD56_SPI && CONFIG_MODEM_ALTMDM */

