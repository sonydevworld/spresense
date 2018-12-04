/****************************************************************************
 * examples/tlstest/lte_sub.c
 *
 *   Copyright 2018 Sony Corporation
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
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
#include <string.h>
#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>

#include "lte/lte_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LTE_SAMPLE_MQUEUE_NAME_API  "lte_sample_queue_api"
#define MAX_MQUEUE_MSG_API          1
#define MQUEUE_MODE                 0666

extern void board_altmdm_power_control(bool en);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: app_init
 ****************************************************************************/

static int app_init(void)
{
  int errcode;
  mqd_t mqd;
  struct mq_attr mq_attr;

  mq_attr.mq_maxmsg  = MAX_MQUEUE_MSG_API;
  mq_attr.mq_msgsize = sizeof(int);
  mq_attr.mq_flags   = 0;

  /* Create message queue resource */

  mqd = mq_open(LTE_SAMPLE_MQUEUE_NAME_API, (O_RDWR | O_CREAT),
                MQUEUE_MODE, &mq_attr);
  if (mqd < 0)
    {
      errcode = errno;
      printf("mq_open() failed: %d\n", errcode);
      return -1;
    }
  mq_close(mqd);

  return 0;
}

/****************************************************************************
 * Name: app_fin
 ****************************************************************************/

static void app_fin(void)
{
  mq_unlink(LTE_SAMPLE_MQUEUE_NAME_API);
}

/****************************************************************************
 * Name: app_notify_response
 ****************************************************************************/

static void app_notify_response(int response)
{
  int ret;
  mqd_t mqd;
  int errcode;
  int buffer = response;

  /* Open message queue for send */

  mqd = mq_open(LTE_SAMPLE_MQUEUE_NAME_API, O_WRONLY);
  if (mqd < 0)
    {
      errcode = errno;
      printf("mq_open() failed: %d\n", errcode);
      return;
    }

  /* Send response */

  ret = mq_send(mqd, (FAR const char*)&buffer, sizeof(buffer), 0);
  if (ret < 0)
    {
      errcode = errno;
      printf("mq_send() failed: %d\n", errcode);
      mq_close(mqd);
      return;
    }
  mq_close(mqd);
}

/****************************************************************************
 * Name: app_wait_response
 ****************************************************************************/

static int app_wait_response(int *response)
{
  int ret;
  mqd_t mqd;
  int errcode;
  int resp;

  /* Open message queue for receive */

  mqd = mq_open(LTE_SAMPLE_MQUEUE_NAME_API, O_RDONLY);
  if (mqd < 0)
    {
      errcode = errno;
      printf("mq_open() failed: %d\n", errcode);
      return -1;
    }

  /* Receive response */

  ret = mq_receive(mqd, (FAR char*)&resp, sizeof(resp), 0);
  if (ret < 0)
    {
      errcode = errno;
      printf("mq_send() failed: %d\n", errcode);
      mq_close(mqd);
      return -1;
    }
  mq_close(mqd);

  *response = resp;

  return 0;
}

/****************************************************************************
 * Name: app_errorcase_print
 ***************************************************************************/

static void app_errorcase_print(uint32_t result, uint32_t errcause)
{
  if (LTE_RESULT_ERROR == result)
    {
      switch (errcause)
      {
        case LTE_ERR_WAITENTERPIN:
          {
            printf("[ERR] : Waiting for PIN enter\n");
          }
        break;
        case LTE_ERR_REJECT:
          {
            printf("[ERR] : Rejected from the network\n");
          }
        break;
        case LTE_ERR_MAXRETRY:
          {
            printf("[ERR] : No response from the network\n");
          }
        break;
        case LTE_ERR_BARRING:
          {
            printf("[ERR] : Network barring\n");
          }
        break;
        default:
          {
            printf("[ERR] : Unexpected cause\n");
          }
        break;
      }
    }
}

/****************************************************************************
 * Name: app_poweron_cb
 ****************************************************************************/

static void app_poweron_cb(uint32_t result)
{
  printf("%s called\n", __func__);

  app_notify_response(result);
}

/****************************************************************************
 * Name: app_poweroff_cb
 ****************************************************************************/

static void app_poweroff_cb(uint32_t result)
{
  printf("%s called\n", __func__);

  app_notify_response(result);
}

/****************************************************************************
 * Name: app_get_imsi_cb
 ****************************************************************************/

static void app_get_imsi_cb(uint32_t result, uint8_t errcause, int8_t *imsi)
{
  if (result == LTE_RESULT_OK)
    {
      printf("%s called: result:%d, imsi:%s\n", __func__,
         result, imsi);
    }
  else
    {
      printf("%s called: result:%d, errcause:%d\n", __func__,
        result, errcause);
      app_errorcase_print(result, errcause);
    }
  app_notify_response(result);
}

/****************************************************************************
 * Name: app_attach_net_cb
 ****************************************************************************/

static void app_attach_net_cb(uint32_t result, uint32_t errcause)
{
  if (LTE_RESULT_ERROR == result)
    {
      printf("%s called: result:%d errorcause:%d\n",
             __func__, result, errcause);
      app_errorcase_print(result, errcause);
    }
  else
    {
      printf("%s called: result:%d\n", __func__, result);
    }

  app_notify_response(result);
}

/****************************************************************************
 * Name: app_detach_net_cb
 ****************************************************************************/

static void app_detach_net_cb(uint32_t result)
{
  printf("%s called: result:%d\n", __func__, result);
  app_notify_response(result);
}

/****************************************************************************
 * Name: app_data_on_cb
 ****************************************************************************/

static void app_data_on_cb(uint32_t result, uint32_t errcause)
{
  if (LTE_RESULT_ERROR == result)
    {
      printf("%s called: result:%d errorcause:%d\n",
             __func__, result, errcause);
    }
  else
    {
      printf("%s called: result:%d\n", __func__, result);
    }

  app_notify_response(result);
}

/****************************************************************************
 * Name: app_data_off_cb
 ****************************************************************************/

static void app_data_off_cb(uint32_t result, uint32_t errcause)
{
  if (LTE_RESULT_ERROR == result)
    {
      printf("%s called: result:%d errorcause:%d\n",
             __func__, result, errcause);
    }
  else
    {
      printf("%s called: result:%d\n", __func__, result);
    }

  app_notify_response(result);
}

/****************************************************************************
 * Name: app_lte_init
 ****************************************************************************/

static int app_lte_init(void)
{
  int ret;
  int response = LTE_RESULT_OK;

  /* Initialize the LTE library */

  ret = lte_initialize();
  if (ret < 0)
    {
      printf("Failed to initialize LTE library :%d\n", ret);
      goto errout;
    }

  /* Power on the modem
   * If it succeeds, it will be able to accept requests from all APIs */

  ret = lte_power_control(LTE_POWERON, app_poweron_cb);
  if (ret < 0)
    {
      printf("Failed to power on the modem :%d\n", ret);
      goto errout_with_fin;
    }

  /* Wait until the modem startup completed */

  ret = app_wait_response(&response);
  if ((ret < 0) || (response != LTE_RESULT_OK))
    {
      goto errout_with_fin;
    }

  return 0;

errout_with_fin:
  lte_finalize();

errout:
  return ret;
}

/****************************************************************************
 * Name: app_lte_fin
 ****************************************************************************/

static int app_lte_fin(void)
{
  int ret;
  int response = LTE_RESULT_OK;

  /* Need wait before power_off command */

  sleep(1);

  /* Power off the modem
   * If it succeeds, it will be not able to accept requests from all APIs */

  ret = lte_power_control(LTE_POWEROFF, app_poweroff_cb);
  if (ret < 0)
    {
      printf("Failed to power off the modem :%d\n", ret);
      goto errout_with_fin;
    }

  /* Wait until the modem shutdown completed */

  ret = app_wait_response(&response);
  if ((ret < 0) || (response != LTE_RESULT_OK))
    {
      goto errout_with_fin;
    }

  /* Finalize LTE library */

  ret = lte_finalize();
  if (ret < 0)
    {
      printf("Failed to finalize LTE library :%d\n", ret);
      goto errout;
    }


  return 0;

errout_with_fin:
  lte_finalize();

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int lte_init(void)
{
  int response = LTE_RESULT_OK;
  int ret;

  ret = app_init();
  if (ret < 0)
    {
      return ret;
    }

  ret = app_lte_init();
  if (ret < 0)
    {
      goto errout_app;
    }

  ret = lte_get_imsi(app_get_imsi_cb);
  if (ret < 0)
    {
      printf("lte_get_imsi() failed :%d\n", ret);
      goto errout_off;
    }

  ret = app_wait_response(&response);
  if ((ret < 0) || (response != LTE_RESULT_OK))
    {
      goto errout_off;
    }

  ret = lte_attach_network(app_attach_net_cb);
  if (ret < 0)
    {
      printf("Failed to attach to LTE network :%d\n", ret);
      goto errout_off;
    }

  ret = app_wait_response(&response);
  if ((ret < 0) || (response == LTE_RESULT_ERROR))
    {
      goto errout_off;
    }

  ret = lte_data_on(LTE_SESSION_ID_MIN, app_data_on_cb);
  if (ret < 0)
    {
      printf("Failed to Data-ON :%d\n", ret);
      goto errout_off;
    }

  ret = app_wait_response(&response);
  if ((ret < 0) || (response == LTE_RESULT_ERROR))
    {
      goto errout_off;
    }

  return 0;

errout_off:
  app_lte_fin();

errout_app:
  app_fin();

  if (ret == 0)
    {
      ret = -1;
    }
  return ret;
}

int lte_fin(void)
{
  int ret;
  int response = LTE_RESULT_OK;

  ret = lte_data_off(LTE_SESSION_ID_MIN, app_data_off_cb);
  if (ret < 0)
    {
      printf("Failed to Data-OFF :%d\n", ret);
    }
  app_wait_response(&response);

  ret = lte_detach_network(app_detach_net_cb);
  if (ret < 0)
    {
      printf("Failed to detach to LTE network :%d\n", ret);
    }
  app_wait_response(&response);

  app_lte_fin();

  app_fin();

  return 0;
}

void lte_pon(void)
{
  board_altmdm_power_control(true);
  printf("Power on the modem\n");
}

void lte_poff(void)
{
  board_altmdm_power_control(false);
  printf("Power off the modem\n");
}
