/****************************************************************************
 * system/gpio/gpio_main.c
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
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <arch/board/board.h>
#include <arch/chip/pin.h>

#include "gpio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(FAR const char *progname, int errcode)
{
  fprintf(stderr, "USAGE: %s command\n", progname);
#ifdef CONFIG_SYSTEM_GPIO_STATUS
  fprintf(stderr, " stat [<from_pin>] [<end_pin>]\n");
#endif
  fprintf(stderr, " conf <pin> [-m <0|1|2|3>] [-i] [-H] [-p <0|1|2|3>]\n");
  fprintf(stderr, "  -m: function mode\n");
  fprintf(stderr, "  -i: input enable\n");
  fprintf(stderr, "  -H: Higher drive current/slew rate\n");
  fprintf(stderr, "  -p: 0=float, 1=pullup, 2=pulldown, 3=buskeeper\n");
  fprintf(stderr, " read <pin>\n");
  fprintf(stderr, " write <pin> <0|1|-1>\n");
  exit(errcode);
}

static int gpio_read(int argc, FAR char *argv[])
{
  uint32_t pin;
  int val;

  if (argc < 3)
    {
      show_usage(argv[0], EXIT_FAILURE);
      return -EINVAL;
    }

  pin = strtoul(argv[2], NULL, 10);

  val = board_gpio_read(pin);
  printf("%d\n", val);
  return val;
}

static int gpio_write(int argc, FAR char *argv[])
{
  uint32_t pin;
  int val;

  if (argc < 4)
    {
      show_usage(argv[0], EXIT_FAILURE);
      return -EINVAL;
    }

  pin = strtoul(argv[2], NULL, 10);
  val = strtol(argv[3], NULL, 10);

  board_gpio_write(pin, val);
  return OK;
}

static int gpio_config(int argc, FAR char *argv[])
{
  uint32_t pin;
  int option;
  int mode = 0;
  bool input = false;
  bool drive = false;
  int pull = PIN_FLOAT;

  if (argc < 3)
    {
      show_usage(argv[0], EXIT_FAILURE);
      return -EINVAL;
    }

  pin = strtoul(argv[2], NULL, 10);

  argv += 2;
  argc -= 2;
  while ((option = getopt(argc, argv, "m:iHp:")) >= 0)
    {
      switch (option)
        {
          case 'm':
            mode = atoi(optarg);
            break;

          case 'i':
            input = true;
            break;

          case 'H':
            drive = true;
            break;

          case 'p':
            pull = atoi(optarg);
            break;

          default:
          case '?':
            show_usage(argv[0], EXIT_FAILURE);
            return -EINVAL;
        }
    }

  return board_gpio_config(pin, mode, input, drive, pull);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int gpio_main(int argc, char **argv)
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
      { "read", gpio_read },
      { "write", gpio_write },
      { "conf", gpio_config },
#ifdef CONFIG_SYSTEM_GPIO_STATUS
      { "stat", gpio_status },
#endif
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
