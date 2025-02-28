/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/audiolite_gnss1pps_dpll_main.c
 *
 *   Copyright 2025 Sony Semiconductor Solutions Corporation
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

#include <nuttx/config.h>
#include <stdio.h>
#include <poll.h>
#include <fcntl.h>
#include <ctype.h>

#include <arch/board/board.h>
#include <arch/chip/pin.h>
#include <audiolite/audiolite.h>

#include "alusr_pll.h"
#include "alusr_filter.h"
#include "pll/pll_worker_main.h"
#include "netserver.h"

#ifdef CONFIG_SENSORS_CXD5610_GNSS
#  include "gnss_addon.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAPTURE_NONE        (0)
#define CAPTURE_FILTER_OUT  (1)
#define CAPTURE_PLL_OUT     (2)

#define LED_1PPS_READY    PIN_LED3

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SENSORS_CXD5610_GNSS
static gnss_ctrl_t g_gnss;
#endif

static uint8_t g_xferdata[XFER_DATA_BYTES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_SENSORS_CXD5610_GNSS
static int check_kbd(alusr_pll *pll)
{
  struct pollfd pfd;

  /* Check the key-board press */

  pfd.fd = fileno(stdin);
  pfd.events = POLLIN;
  poll(&pfd, 1, 0);
  if (pfd.revents & POLLIN)
    {
      switch (getchar())
        {
          case 'q': return 1; /* Exit loop */
          case 's':
            pll->start_pll();
            board_gpio_write(LED_1PPS_READY, 1);
            break;
          case 'e':
            pll->stop_pll();
            board_gpio_write(LED_1PPS_READY, 0);
            break;
          case 'd':
            pll->send_xferdata(g_xferdata, XFER_DATA_BYTES);
            pll->enable_xfer();
            break;
        }
    }

  return 0;
}
#endif

static void init_xferdata(void)
{
  int i;
  for (i = 0; i < XFER_DATA_BYTES; i++)
    {
      g_xferdata[i] = 0x5a;
    }
}

static void init_pin(void)
{
  board_gpio_config(PIN_LED0, 0, false, true, PIN_FLOAT);
  board_gpio_write(PIN_LED0, 0);

  board_gpio_config(PIN_LED1, 0, false, true, PIN_FLOAT);
  board_gpio_write(PIN_LED1, 0);

  board_gpio_config(PIN_LED2, 0, false, true, PIN_FLOAT);
  board_gpio_write(PIN_LED2, 0);

  board_gpio_config(PIN_LED3, 0, false, true, PIN_FLOAT);
  board_gpio_write(PIN_LED3, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int main(int argc, FAR char *argv[])
{
  int capture_data = CAPTURE_NONE;
#ifdef CONFIG_SENSORS_CXD5610_GNSS
  int next_state, cur_state = GNSS_STATE_NOTLOCK;
#endif

  init_xferdata();

  audiolite_simplelistener lsn;
  audiolite_inputcomp    *aindev  = new audiolite_inputcomp;
  audiolite_mempoolapbuf *mempool = new audiolite_mempoolapbuf;
  audiolite_outputcomp   *aoutdev = new audiolite_outputcomp;
  alusr_pll              *pll     = new alusr_pll;
  alusr_filter           *filter  = new alusr_filter;
  netserver              *sender  = new netserver();

  if (argc >= 2)
    {
      if (!strncmp(argv[1], "-pll", 5))
        {
          capture_data = CAPTURE_PLL_OUT;
        }
      else if (!strncmp(argv[1], "-fil", 5))
        {
          capture_data = CAPTURE_FILTER_OUT;
        }
      else
        {
          printf("Usage : %s (-pll/-fil)\n", argv[0]);
          goto err_out;
        }
    }

  init_pin();

#ifdef CONFIG_SENSORS_CXD5610_GNSS
  if (gnss_setup(&g_gnss) != OK)
    {
      printf("Error GNSS Setup\n");
      goto err_out;
    }
#endif

  audiolite_set_evtlistener(&lsn);

  audiolite_set_systemparam(SAMPLE_FS, SAMPLE_BITS, SAMPLE_CHS);
  mempool->create_instance(BLK_SAMPLES * SAMPLE_CHS * sizeof(int16_t), 16);
  aindev->set_mempool(mempool);

  aoutdev->set_volume(1000); /* Set SPK volume 1000(MAX) */
  aindev->set_micgain(0); /* Set MIC gain 0 to minimize noise gain */

  aindev->bind(filter)->bind(pll)->bind(aoutdev);
  switch (capture_data)
    {
      case CAPTURE_FILTER_OUT:
        printf("Capturing Filter Output\n");
        filter->bind(sender);
        break;
      case CAPTURE_PLL_OUT:
        printf("Capturing PLL Output\n");
        pll->bind(sender);
        break;
    }

  /* Start Audio Loop */

  aindev->start();

  while (1)
    {
      /* Controll PLL according to GNSS 1PPS signal status */

#ifdef CONFIG_SENSORS_CXD5610_GNSS
      next_state = gnss_wait_statechange(&g_gnss, cur_state);
      if (cur_state != next_state)
        {
          switch (next_state)
            {
              case GNSS_STATE_NOTLOCK:
                printf("GNSS is unlocked. Stop PLL\n");
                pll->stop_pll();
                board_gpio_write(LED_1PPS_READY, 0);
                break;

              case GNSS_STATE_LOCKED:
                printf("GNSS is LOCKed. Start PLL\n");
                pll->start_pll();
                board_gpio_write(LED_1PPS_READY, 1);
                break;
            }

          cur_state = next_state;
        }
#else /* Case of external 1PPS signal, not used CXD5610 */
      if (check_kbd(pll)) break;
      usleep(300 * 1000);
#endif
    }

  aindev->stop();
  aindev->unbindall();

err_out:

  audiolite_eventdestroy();

  delete sender;
  delete filter;
  delete pll;
  delete aoutdev;
  delete aindev;
  delete mempool;

  return 0;
}
