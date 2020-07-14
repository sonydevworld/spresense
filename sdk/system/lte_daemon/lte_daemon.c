/****************************************************************************
 * system/lte_daemon/lte_daemon.c
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int               ret         = 0;
  int               opt         = 0;
  char              *cmd        = NULL;
  lte_apn_setting_t setting_apn = {};

  setting_apn.apn       = (int8_t*)APP_APN_NAME;
  setting_apn.apn_type  = LTE_APN_TYPE_DEFAULT | LTE_APN_TYPE_IA;
  setting_apn.ip_type   = APP_APN_IPTYPE;
  setting_apn.auth_type = APP_APN_AUTHTYPE;
  setting_apn.user_name = (int8_t*)APP_APN_USR_NAME;
  setting_apn.password  = (int8_t*)APP_APN_PASSWD;

  optind = -1;
  while ((opt = getopt(argc, argv, "a:t:i:v:u:p:h")) != -1)
    {
      switch (opt)
        {
          case 'a':
            setting_apn.apn = (int8_t *)optarg;
            break;
          case 't':
            setting_apn.apn_type = strtol(optarg, NULL, LTE_DAEMON_STRTOL_BASE_HEX);
            break;
          case 'i':
            setting_apn.ip_type =  (uint8_t)strtol(optarg, NULL, LTE_DAEMON_STRTOL_BASE);
            break;
          case 'v':
            setting_apn.auth_type = (uint8_t)strtol(optarg, NULL, LTE_DAEMON_STRTOL_BASE);
            break;
          case 'u':
            setting_apn.user_name = (int8_t *)optarg;
            break;
          case 'p':
            setting_apn.password = (int8_t *)optarg;
            break;
          case 'h':
            printf("lte_daemon usage: lte_daemon\n");
            printf("[-a <apn_name> -t <apn_type> -i <ip_type> -v <auth_type> -u <usr_name> -p <passward>]\n");
            printf("[start/stop]\n");
            printf("** Non-Required option **\n");
            printf("** Set Defaults if not specified **\n");
            printf("** -a :  Set apn name **\n");
            printf("** -t :  Set apn type **\n");
            printf("** -i :  Set ip type **\n");
            printf("** -v :  Set auth type **\n");
            printf("** -u :  Set user name **\n");
            printf("** -p :  Set passward **\n");
            printf("------\n");
            printf("** Required option **\n");
            printf("** start : start lte_daemon **\n");
            printf("** stop : stop lte_daemon **\n");
            break;
          default:
            break;
        }
    }

  if (optind >= argc || argv[1] == NULL)
    {
      printf("lte_daemon: missing required argument(s)\n");
      return -1;
    }
  cmd = argv[optind++];

  if (0 == strncmp(cmd, "start", strlen(cmd)))
    {
      ret = lte_daemon_init(&setting_apn);
      if (0 > ret)
        {
          printf("lte_daemon_init() error. %d\n", ret);
          goto err_out;
        }

      ret = lte_daemon_power_on();
      if (0 > ret)
        {
          printf("daemon_power_on() error. %d\n", ret);
          goto err_out;
        }
    }

    if (0 == strncmp(cmd, "stop", strlen(cmd)))
      {
        ret = lte_daemon_fin();
        if (0 > ret)
          {
            printf("lte_finalize() error. %d\n", ret);
            goto err_out;
          }
      }

  return 0;

err_out:
  return ret;
}
