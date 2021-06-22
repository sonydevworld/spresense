/****************************************************************************
 * system/gpioint/gpioint_main.c
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

static struct irqlist_s
{
  uint32_t  pin;
  bool      edge;
  int       count;
} g_irqlist[12];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(FAR const char *progname, int errcode)
{
  fprintf(stderr, "USAGE: %s command\n", progname);
  fprintf(stderr, " stat\n");
  fprintf(stderr, " conf <pin> [-m <mode>] [-f]\n");
  fprintf(stderr, "  -m: 2=high, 3=low, 4=rise, 5=fall, 7=both\n");
  fprintf(stderr, "  -f: noise filter enabled\n");
  fprintf(stderr, " ena <pin> : enable\n");
  fprintf(stderr, " dis <pin> : disable\n");
  exit(errcode);
}

static int gpio_handler(int irq, FAR void *context, FAR void *arg)
{
  uint32_t pin = (uint32_t)arg;
  struct irqlist_s *pirq;

  pirq = &g_irqlist[irq - CXD56_IRQ_EXDEVICE_0];

  /* Record a number of the occured interrupt */

  pirq->count++;

  if (pirq->edge == false)
    {
      /* if level trigger, disable interrupt */

      board_gpio_int(pin, false);
    }
  return OK;
}

static int gpioint_config(int argc, FAR char *argv[])
{
  uint32_t pin;
  int option;
  int mode = 0;
  bool filter = false;
  int irq;
  struct irqlist_s *pirq;
  xcpt_t handler;

  if (argc < 3)
    {
      show_usage(argv[0], EXIT_FAILURE);
      return -EINVAL;
    }

  pin = strtoul(argv[2], NULL, 10);

  argv += 2;
  argc -= 2;
  while ((option = getopt(argc, argv, "m:f")) >= 0)
    {
      switch (option)
        {
          case 'm':
            mode = atoi(optarg);
            break;

          case 'f':
            filter = true;
            break;

          default:
          case '?':
            show_usage(argv[0], EXIT_FAILURE);
            return -EINVAL;
        }
    }

  handler = (mode) ? gpio_handler : NULL;
  irq = board_gpio_intconfig(pin, mode, filter, handler);
  if ((CXD56_IRQ_EXDEVICE_0 <= irq) &&
      (irq < CXD56_IRQ_EXDEVICE_0 +12))
    {
      pirq = &g_irqlist[irq - CXD56_IRQ_EXDEVICE_0];
      pirq->pin = (mode) ? pin : 0;
      pirq->edge = (mode >= INT_RISING_EDGE);
      pirq->count = 0;
    }
  return OK;
}

static int gpioint_enable(int argc, FAR char *argv[])
{
  uint32_t pin;

  if (argc < 3)
    {
      show_usage(argv[0], EXIT_FAILURE);
      return -EINVAL;
    }

  pin = strtoul(argv[2], NULL, 10);

  return board_gpio_int(pin, true);
}

static int gpioint_disable(int argc, FAR char *argv[])
{
  uint32_t pin;

  if (argc < 3)
    {
      show_usage(argv[0], EXIT_FAILURE);
      return -EINVAL;
    }

  pin = strtoul(argv[2], NULL, 10);

  return board_gpio_int(pin, false);;
}

static int gpioint_status(int argc, FAR char *argv[])
{
  int i;

  printf("IRQ Pin Count\n");
  printf("--- --- -----\n");
  
  for (i = 0; i < sizeof(g_irqlist) / sizeof(g_irqlist[0]); i++)
    {
      if (g_irqlist[i].pin)
        {
          printf("%-3d %3ld %5d\n", i + CXD56_IRQ_EXDEVICE_0,
                 g_irqlist[i].pin, g_irqlist[i].count);
        }
      else
        {
          printf("%-3d\n", i + CXD56_IRQ_EXDEVICE_0);
        }
    }
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int gpioint_main(int argc, char **argv)
#endif
{
  int i;
  struct subcmd_s
    {
      const char *name;
      int (*func)(int argc, char *argv[]);
    }
  subcmd[] =
    {
      { "stat", gpioint_status },
      { "conf", gpioint_config },
      { "ena", gpioint_enable },
      { "dis", gpioint_disable },
    };


  /* argument check */

  if (argc < 2)
    {
      goto error;
    }

  for (i = 0; i < sizeof(subcmd) / sizeof(subcmd[0]); i++)
    {
      if (!strncmp(subcmd[i].name, argv[1], strlen(subcmd[i].name)))
        {
          return subcmd[i].func(argc, argv);
        }
    }
error:
  show_usage(argv[0], EXIT_FAILURE);
  return -EINVAL;
}
