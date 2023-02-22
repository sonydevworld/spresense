/****************************************************************************
 * examples/lte_lwm2mstub/app_lte_util.c
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <lte/lte_api.h>

#include "app_message.h"
#include "app_parameter.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_restart_cb
 ****************************************************************************/

static void lte_restart_cb(uint32_t reason)
{
  struct app_message_s msg;
  switch (reason)
    {
      case LTE_RESTART_USER_INITIATED:
      case LTE_RESTART_MODEM_INITIATED:
      case LTE_RESTART_VERSION_ERROR:
        msg.msgid = MESSAGE_ID_LTE_RESTARTED;
        msg.arg.code = reason;
        send_message(MESSAGE_QUEUE_NAME, &msg);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: wait_for_lterestart
 ****************************************************************************/

static int wait_for_lterestart(void)
{
  int ret;
  struct app_message_s msg;
  lte_version_t version = {0};
  if (receive_message(MESSAGE_QUEUE_NAME, &msg) == OK)
    {
      if (msg.msgid == MESSAGE_ID_LTE_RESTARTED &&
          msg.arg.code == LTE_RESTART_USER_INITIATED)
        {
          return OK;
        }
      else if (msg.msgid == MESSAGE_ID_LTE_RESTARTED &&
               msg.arg.code == LTE_RESTART_VERSION_ERROR)
        {
          printf("Please enable the disabled protocol version"
                 " and flash the application.\n");

          ret = lte_get_version_sync(&version);
          if (ret == 0)
            {
              printf("Modem IC Type : %s\n", version.bb_product);
              printf("      FW Ver. : %s\n", version.np_package);
            }
        }
    }

  return ERROR;
}

/****************************************************************************
 * Name: localtime_cb
 ****************************************************************************/

static void localtime_cb(lte_localtime_t *localtm)
{
  struct tm tmtime;
  struct timespec cur_time;

  memset(&cur_time, 0, sizeof(cur_time));

  tmtime.tm_year = localtm->year + 100;
  tmtime.tm_mon  = localtm->mon - 1;
  tmtime.tm_mday = localtm->mday;
  tmtime.tm_hour = localtm->hour;
  tmtime.tm_min  = localtm->min;
  tmtime.tm_sec  = localtm->sec;

  cur_time.tv_sec = mktime(&tmtime);

  clock_settime(CLOCK_REALTIME, &cur_time);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: power_on_ltemodem
 ****************************************************************************/

int power_on_ltemodem(struct app_parameter_s *param)
{
  int ret;

  ret = lte_initialize();
  if ((ret < 0) && (ret != -EALREADY))
    {
      printf("lte_initialize() returned error : %d\n", ret);
      return ERROR;
    }

  lte_set_report_restart(lte_restart_cb);

  ret = lte_power_on();
  if (ret >= 0)
    {
      if (wait_for_lterestart() != OK)
        {
          printf("Waiting restart error\n");
          lte_finalize();
          return ERROR;
        }
    }
  else if (ret != -EALREADY)
    {
      printf("lte_power_on() returned error : %d\n", ret);
      lte_finalize();
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: connect_to_ltenetwork
 ****************************************************************************/

int connect_to_ltenetwork(struct app_parameter_s *param)
{
  int ret;
  int i;
  lte_pdn_t pdn;
  lte_apn_setting_t apnsetting;

  ret = lte_radio_on_sync();
  if (ret < 0)
    {
      printf("Radio on error:%d\n", ret);
      return ERROR;
    }

  lte_set_report_localtime(localtime_cb);

#ifdef CONFIG_EXAMPLES_LTE_LWM2MSTUB_SUPPORT_RATCHANGE
  lte_set_rat_sync(param->rat, LTE_DISABLE);
#endif

  apnsetting.apn       = param->apn_name;
  apnsetting.apn_type  = LTE_APN_TYPE_DEFAULT | LTE_APN_TYPE_IA;
  apnsetting.ip_type   = param->ip_type;
  apnsetting.auth_type = param->auth_type;
  apnsetting.user_name = param->user_name;
  apnsetting.password  = param->passwd;

  if (lte_activate_pdn_sync(&apnsetting, &pdn) < 0)
    {
      printf("lte_activate_pdn_sync() returned error\n");
      return ERROR;
    }

  printf("LTE PDN session ID : %d\n", pdn.session_id);
  if (pdn.ipaddr_num > 0)
    {
      for (i = 0; i < pdn.ipaddr_num; i++)
        {
          printf("        IP Addr[%d] : %s\n", i, pdn.address[i].address);
        }
    }
  else
    {
      printf("           IP Addr : Nothing...\n");
    }

  return OK;
}
