/****************************************************************************
 * system/lte_daemon/lte_daemon.c
 *
 *   Copyright 2020, 2021 Sony Semiconductor Solutions Corporation
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

#include "lte/lte_api.h"
#include "lte/lte_daemon.h"
#include "lte/altcom/altcom_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LTE_DAEMON_STRTOL_BASE     (10)
#define LTE_DAEMON_STRTOL_BASE_HEX (16)

/* APN settings */

#ifdef CONFIG_LTE_DAEMON_APN_NAME
#  define APP_APN_NAME   CONFIG_LTE_DAEMON_APN_NAME
#else
#  define APP_APN_NAME   "lte_daemon_sample_apn"
#endif

#ifdef CONFIG_LTE_DAEMON_APN_IPTYPE_IPV6
#  define APP_APN_IPTYPE   LTE_APN_IPTYPE_IPV6
#elif defined CONFIG_LTE_DAEMON_APN_IPTYPE_IPV4V6
#  define APP_APN_IPTYPE   LTE_APN_IPTYPE_IPV4V6
#else
#  define APP_APN_IPTYPE   LTE_APN_IPTYPE_IP
#endif

#ifdef CONFIG_LTE_DAEMON_APN_AUTHTYPE_PAP
#  define APP_APN_AUTHTYPE LTE_APN_AUTHTYPE_PAP
#elif defined CONFIG_LTE_DAEMON_APN_AUTHTYPE_CHAP
#  define APP_APN_AUTHTYPE LTE_APN_AUTHTYPE_CHAP
#else
#  define APP_APN_AUTHTYPE LTE_APN_AUTHTYPE_NONE
#endif

#ifdef CONFIG_LTE_DAEMON_APN_USERNAME
#  define APP_APN_USR_NAME CONFIG_LTE_DAEMON_APN_USERNAME
#else
#  define APP_APN_USR_NAME ""
#endif

#ifdef CONFIG_LTE_DAEMON_APN_PASSWD
#  define APP_APN_PASSWD   CONFIG_LTE_DAEMON_APN_PASSWD
#else
#  define APP_APN_PASSWD   ""
#endif

#define LTE_DAEMON_ERR_FMT_NUM "%s: %s failed: %d\n"
#define LTE_DAEMON_ERR_FMT_STR "%s: %s failed: %s\n"
#define LTE_DAEMON_CMD_START "start"
#define LTE_DAEMON_CMD_STOP "stop"
#define LTE_DAEMON_CMD_STAT "stat"
#define LTE_DAEMON_CMD_RAT_CATM1 "M1"
#define LTE_DAEMON_CMD_RAT_NB "NB"

#define MATCH_STRING(str1, str2) ((strlen(str1) == strlen(str2)) && \
                                  (strncmp(str1, str2, strlen(str2)) == 0))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(FAR const char *progname, int exitcode)
{
  fprintf(stderr, "\nUSAGE: %s command\n", progname);
  fprintf(stderr, " [-a <apn_name>] [-i <ip_type>] [-v <auth_type>] [-u <user_name>] [-p <password>] [-r <rat_type>] start\n");
  fprintf(stderr, "  -a: APN name\n");
  fprintf(stderr, "  -i: IP type 0=IPv4, 1=IPv6, 2=IPv4 and IPv6\n");
  fprintf(stderr, "  -v: Authenticaion type 0=NONE, 1=PAP, 2=CHAP\n");
  fprintf(stderr, "  -u: User name for authenticaion\n");
  fprintf(stderr, "  -p: Password for authenticaion\n");
  fprintf(stderr, "  -r: Radio Access Technology type M1=CAT-M1, NB=NB-IoT\n");
  fprintf(stderr, " stop\n");
  fprintf(stderr, " stat\n");
  fprintf(stderr, " [-h]: Show this message\n");
  exit(exitcode);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int               ret         = 0;
  int               opt         = 0;
  char              *cmd        = NULL;
  lte_apn_setting_t setting_apn = {};
  char              *rat_str    = NULL;
  uint8_t           rat         = LTE_DAEMON_RAT_KEEP;
  long apn_type;
  long ip_type;
  long auth_type;

  setting_apn.apn       = (int8_t*)APP_APN_NAME;
  setting_apn.apn_type  = LTE_APN_TYPE_DEFAULT | LTE_APN_TYPE_IA;
  setting_apn.ip_type   = APP_APN_IPTYPE;
  setting_apn.auth_type = APP_APN_AUTHTYPE;
  setting_apn.user_name = (int8_t*)APP_APN_USR_NAME;
  setting_apn.password  = (int8_t*)APP_APN_PASSWD;

  optind = -1;
  while ((opt = getopt(argc, argv, "a:t:i:v:u:p:r:h")) != -1)
    {
      switch (opt)
        {
          case 'a':
            if (strlen(optarg) >= LTE_APN_LEN)
              {
                fprintf(stderr, "APN name is too long\n");
                show_usage(argv[0], EXIT_FAILURE);
              }

            setting_apn.apn = (int8_t *)optarg;
            break;
          case 't':
            apn_type = strtol(optarg, NULL, LTE_DAEMON_STRTOL_BASE_HEX);
            if (apn_type != (LTE_APN_TYPE_IA | LTE_APN_TYPE_DEFAULT))
              {
                fprintf(stderr, "Currently supported APN type is 0x%x",
                        LTE_APN_TYPE_DEFAULT | LTE_APN_TYPE_IA);
                fprintf(stderr, ": LTE_APN_TYPE_DEFAULT | LTE_APN_TYPE_IA\n");
                show_usage(argv[0], EXIT_FAILURE);
              }

            setting_apn.apn_type = (uint32_t)apn_type;
            break;
          case 'i':
            ip_type = strtol(optarg, NULL, LTE_DAEMON_STRTOL_BASE);
            if ((ip_type != LTE_IPTYPE_V4) &&
                (ip_type != LTE_IPTYPE_V6) &&
                (ip_type != LTE_IPTYPE_V4V6))
              {
                fprintf(stderr, "Invalid IP type:%ld\n", ip_type);
                show_usage(argv[0], EXIT_FAILURE);
              }

            setting_apn.ip_type = (uint8_t)ip_type;
            break;
          case 'v':
            auth_type = strtol(optarg, NULL, LTE_DAEMON_STRTOL_BASE);
            if ((auth_type != LTE_APN_AUTHTYPE_NONE) &&
                (auth_type != LTE_APN_AUTHTYPE_PAP) &&
                (auth_type != LTE_APN_AUTHTYPE_CHAP))
              {
                fprintf(stderr, "Invalid authenticaion type:%ld\n",
                        auth_type);
                show_usage(argv[0], EXIT_FAILURE);
              }

            setting_apn.auth_type = (uint8_t)auth_type;
            break;
          case 'u':
            if (strlen(optarg) >= LTE_APN_USER_NAME_LEN)
              {
                fprintf(stderr, "User name is too long\n");
                show_usage(argv[0], EXIT_FAILURE);
              }

            setting_apn.user_name = (int8_t *)optarg;
            break;
          case 'p':
            if (strlen(optarg) >=LTE_APN_PASSWD_LEN)
              {
                fprintf(stderr, "Password is too long\n");
                show_usage(argv[0], EXIT_FAILURE);
              }

            setting_apn.password = (int8_t *)optarg;
            break;
          case 'r':
            rat_str = optarg;
            if (MATCH_STRING(rat_str, LTE_DAEMON_CMD_RAT_CATM1))
              {
                rat = LTE_RAT_CATM;
              }
            else if (MATCH_STRING(rat_str, LTE_DAEMON_CMD_RAT_NB))
              {
                rat = LTE_RAT_NBIOT;
              }
            else
              {
                fprintf(stderr,
                        "Please set Radio Access Technology: M1 or NB\n");
                show_usage(argv[0], EXIT_FAILURE);
              }
            break;
          case 'h':
            show_usage(argv[0], EXIT_SUCCESS);
            break;
          default:
            show_usage(argv[0], EXIT_FAILURE);
            break;
        }
    }

  if (optind >= argc || argv[1] == NULL)
    {
      fprintf(stderr, "%s: missing required argument(s)\n", argv[0]);
      show_usage(argv[0], EXIT_FAILURE);
    }
  cmd = argv[optind++];

  if (optind < argc)
    {
      fprintf(stderr, "%s: Invalid required argument(s)\n", argv[0]);
      show_usage(argv[0], EXIT_FAILURE);
    }

  if (MATCH_STRING(cmd, LTE_DAEMON_CMD_START))
    {
      ret = lte_daemon_init(&setting_apn, rat);
      if (ret < 0)
        {
          if (ret == -EALREADY)
            {
              fprintf(stderr, LTE_DAEMON_ERR_FMT_STR, argv[0], cmd,
                      "lte_daemon is running");
            }
          else
            {
              fprintf(stderr, LTE_DAEMON_ERR_FMT_NUM, argv[0], cmd, -ret);
            }

          goto err_out;
        }

      ret = lte_daemon_power_on();
      if (ret < 0)
        {
          if (ret == -ENOTSUP)
            {
              fprintf(stderr, LTE_DAEMON_ERR_FMT_STR, argv[0], cmd,
              "RAT changes are not supported in the FW version of the modem");
            }
          else
            {
              fprintf(stderr, LTE_DAEMON_ERR_FMT_NUM, argv[0], cmd, -ret);
            }

          lte_daemon_fin();
          goto err_out;
        }
    }
  else if (MATCH_STRING(cmd, LTE_DAEMON_CMD_STOP))
    {
      ret = lte_daemon_fin();
      if (ret < 0)
        {
          if (ret == -EALREADY)
            {
              fprintf(stderr, LTE_DAEMON_ERR_FMT_STR, argv[0], cmd,
                      "lte_daemon is not running");
            }
          else
            {
              fprintf(stderr, LTE_DAEMON_ERR_FMT_NUM, argv[0], cmd, -ret);
            }

          goto err_out;
        }
    }
  else if (MATCH_STRING(cmd, LTE_DAEMON_CMD_STAT))
    {
      ret = lte_daemon_stat();
      if (ret < 0)
        {
          fprintf(stderr, LTE_DAEMON_ERR_FMT_NUM, argv[0], cmd, -ret);

          goto err_out;
        }
    }
  else
    {
      fprintf(stderr, "%s: Invalid required argument(s)\n", argv[0]);
      show_usage(argv[0], EXIT_FAILURE);
    }

  return 0;

err_out:
  return ret;
}
