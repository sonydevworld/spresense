/****************************************************************************
 * system/pmic/pmic_main.c
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
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <arch/chip/pm.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct {
  const char *str;
  int target;
}
g_list[] =
{
  { "DDC_IO",       POWER_DDC_IO },
  { "LDO_EMMC",     POWER_LDO_EMMC },
  { "DDC_ANA",      POWER_DDC_ANA },
  { "LDO_ANA",      POWER_LDO_ANA },
  { "DDC_CORE",     POWER_DDC_CORE },
  { "LDO_PERI",     POWER_LDO_PERI },
  { "LSW2",         PMIC_LSW(2) },
  { "LSW3",         PMIC_LSW(3) },
  { "LSW4",         PMIC_LSW(4) },
  { "GPO0",         PMIC_GPO(0) },
  { "GPO1",         PMIC_GPO(1) },
  { "GPO2",         PMIC_GPO(2) },
  { "GPO3",         PMIC_GPO(3) },
  { "GPO4",         PMIC_GPO(4) },
  { "GPO5",         PMIC_GPO(5) },
  { "GPO6",         PMIC_GPO(6) },
  { "GPO7",         PMIC_GPO(7) },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int getid(const char* str)
{
  int i;
  for (i = 0; i < sizeof(g_list) / sizeof(g_list[0]); i++)
    {
      if (!strncmp(str, g_list[i].str, strlen(g_list[i].str)))
        {
          return g_list[i].target;
        }
    }
  return 0;
}

static void show_usage(FAR const char *progname)
{
  printf("\nUsage: %s [-h] [-l] [-e <target>] [-d <target>]\n", progname);
  printf("            [-r <addr>] [-w <addr> -v <value>]\n\n");
  printf("Description:\n");
  printf(" PMIC utility tool\n");
  printf("Options:\n");
  printf(" -l: Show power status of the target\n");
  printf(" -e <target>: Enable power to the target\n");
  printf(" -d <target>: Disable power to the target\n");
  printf(" -z <target>: Set GPO to HiZ to the target\n");
  printf(" -r <addr>: Single read from <addr>\n");
  printf(" -w <addr> -v <value>: Single write <value> to <addr>\n");
  printf(" -h: Show this message\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * pmic_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int pmic_main(int argc, char *argv[])
#endif
{
  int ret;
  int i;
  int opt;
  int en;
  int id;
  uint8_t addr = 0;
  uint8_t value = 0;
  int list = 0;
  int rx = 0;
  int tx = 0;
  int txval = 0;

  optind = -1;
  while ((opt = getopt(argc, argv, "le:d:z:r:w:v:h")) != -1)
    {
      switch (opt)
        {
          case 'l':
            list = 1;
            break;
          case 'e':
            en = 1;
            goto skip;
          case 'd':
            en = 0;
            goto skip;
          case 'z':
            en = -1;
          skip:
            list = 1;
            id = getid(optarg);
            if (id)
              {
                printf("%s: %s\n",
                       (en > 0) ? "Enable" : (en == 0) ? "Disable" : "HiZ",
                       optarg);
                board_power_control_tristate(id, en);
              }
            else
              {
                printf("Invalid name: %s\n", optarg);
                return EXIT_FAILURE;
              }
            break;
          case 'r':
            rx = 1;
            addr = strtoul(optarg, NULL, 16);
            break;
          case 'w':
            tx = 1;
            addr = strtoul(optarg, NULL, 16);
            break;
          case 'v':
            txval = 1;
            value = strtoul(optarg, NULL, 16);
            break;
          case 'h':
          case ':':
          case '?':
            show_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

  /* No operation */

  if (!tx && !rx && !list)
    {
      show_usage(argv[0]);
      return EXIT_FAILURE;
    }

  /* read operation */

  if (rx)
    {
      ret = board_pmic_read(addr, &value, sizeof(value));
      if (ret)
        {
          printf("@[%02x]=>read error!\n", addr);
        }
      else
        {
          printf("@[%02x]=>%02x\n", addr, value);
        }
      return ret;
    }

  /* write operation */

  if (tx && txval)
    {
      ret = board_pmic_write(addr, &value, sizeof(value));
      if (ret)
        {
          printf("@[%02x]<=write error!\n", addr);
        }
      else
        {
          printf("@[%02x]<=%02x\n", addr, value);
        }
      return ret;
    }

  /* show power status */

  if (list)
    {
      printf("%16s : %s\n", "Target Name", "on/off");
      printf("%16s : %s\n", "-----------", "------");
      for (i = 0; i < sizeof(g_list) / sizeof(g_list[0]); i++)
        {
          en = board_power_monitor_tristate(g_list[i].target);
          printf("%16s : %s\n", g_list[i].str,
                 (en > 0) ? "on" : (en == 0) ? "off" : "hiz");
        }
    }
  return 0;
}
