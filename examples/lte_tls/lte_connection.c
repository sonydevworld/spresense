/****************************************************************************
 * lte_tls/lte_connection.c
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
#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>

#include "lte/lte_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APP_MQUEUE_NAME    "lte_tls_sample_queue"
#define APP_MAX_MQUEUE_MSG 1
#define APP_MQUEUE_MODE    0666
#define APP_SESSION_ID     1

/* APN settings */

#ifdef CONFIG_EXAMPLES_LTE_TLS_APN_NAME
#  define APP_APN_NAME     CONFIG_EXAMPLES_LTE_TLS_APN_NAME
#else
#  define APP_APN_NAME     "lte_tls_sample_apn"
#endif

#ifdef CONFIG_EXAMPLES_LTE_TLS_APN_IPTYPE
#  define APP_APN_IPTYPE   CONFIG_EXAMPLES_LTE_TLS_APN_IPTYPE
#else
#  define APP_APN_IPTYPE   LTE_APN_IPTYPE_IP
#endif

#ifdef CONFIG_EXAMPLES_LTE_TLS_APN_AUTHTYPE
#  define APP_APN_AUTHTYPE CONFIG_EXAMPLES_LTE_TLS_APN_AUTHTYPE
#else
#  define APP_APN_AUTHTYPE LTE_APN_AUTHTYPE_NONE
#endif

#ifdef CONFIG_EXAMPLES_LTE_TLS_APN_USERNAME
#  define APP_APN_USR_NAME CONFIG_EXAMPLES_LTE_TLS_APN_USERNAME
#else
#  define APP_APN_USR_NAME ""
#endif

#ifdef CONFIG_EXAMPLES_LTE_TLS_APN_PASSWD
#  define APP_APN_PASSWD   CONFIG_EXAMPLES_LTE_TLS_APN_PASSWD
#else
#  define APP_APN_PASSWD   ""
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: app_lte_setlocaltime
 ****************************************************************************/

static void app_lte_setlocaltime(FAR lte_localtime_t *localtime)
{
  int            ret;
  struct tm      calTime;
  struct timeval current_time = {0};

  /* lte_localtime_t -> struct tm */

  memset(&calTime, 0, sizeof(struct tm));
  calTime.tm_year = localtime->year + 100; /* 1900 + 100 + year(0-99) */
  calTime.tm_mon  = localtime->mon - 1;    /* mon(1-12) - 1 */
  calTime.tm_mday = localtime->mday;
  calTime.tm_hour = localtime->hour;
  calTime.tm_min  = localtime->min;
  calTime.tm_sec  = localtime->sec;

  /* struct tm -> struct time_t */

  current_time.tv_sec = mktime(&calTime);
  if (current_time.tv_sec < 0)
    {
      printf("%s: mktime falied\n");
      return;
    }

  /* Set time */

  ret = settimeofday(&current_time, NULL);
  if (ret < 0)
    {
      printf("%s: settimeofday falied: %d\n", errno);
      return;
    }

  printf("set localtime completed: %4d/%02d/%02d,%02d:%02d:%02d\n",
         localtime->year + 1900 + 100, localtime->mon, localtime->mday,
         localtime->hour, localtime->min, localtime->sec);
}

/****************************************************************************
 * Name: app_mq_create
 ****************************************************************************/

static int app_mq_create(FAR const char *mq_name)
{
  int            errcode;
  mqd_t          mqd;
  struct mq_attr mq_attr;

  mq_attr.mq_maxmsg  = APP_MAX_MQUEUE_MSG;
  mq_attr.mq_msgsize = sizeof(int);
  mq_attr.mq_flags   = 0;

  /* Create message queue */

  mqd = mq_open(mq_name, (O_RDWR | O_CREAT), APP_MQUEUE_MODE, &mq_attr);
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
 * Name: app_mq_delete
 ****************************************************************************/

static void app_mq_delete(FAR const char *mq_name)
{
  /* Delete message queue */

  mq_unlink(mq_name);
}

/****************************************************************************
 * Name: app_mq_notify_result
 ****************************************************************************/

static void app_mq_notify_result(int result)
{
  int   ret;
  mqd_t mqd;
  int   errcode;
  int   buffer = result;

  /* Open message queue for send */

  mqd = mq_open(APP_MQUEUE_NAME, O_WRONLY);
  if (mqd < 0)
    {
      errcode = errno;
      printf("mq_open() failed: %d\n", errcode);
      return;
    }

  /* Send result of callback */

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
 * Name: app_wait_lte_callback
 ****************************************************************************/

static int app_wait_lte_callback(int *result)
{
  int   ret;
  mqd_t mqd;
  int   errcode;
  int   buffer;

  /* Open message queue for receive */

  mqd = mq_open(APP_MQUEUE_NAME, O_RDONLY);
  if (mqd < 0)
    {
      errcode = errno;
      printf("mq_open() failed: %d\n", errcode);
      return -1;
    }

  /* Receive result of callback */

  ret = mq_receive(mqd, (FAR char*)&buffer, sizeof(buffer), 0);
  if (ret < 0)
    {
      errcode = errno;
      printf("mq_send() failed: %d\n", errcode);
      mq_close(mqd);
      return -1;
    }
  mq_close(mqd);

  *result = buffer;

  return 0;
}

/****************************************************************************
 * Name: app_poweron_cb
 *
 * Description:
 *   This callback is called when the startup is completed
 *   after power on the modem.
 ****************************************************************************/

static void app_poweron_cb(uint32_t result)
{
  printf("%s called\n", __func__);

  /* Notify the result to the lte_http_get sample application task */

  app_mq_notify_result(result);
}

/****************************************************************************
 * Name: app_poweroff_cb
 *
 * Description:
 *   This callback is called when shutdown is completed
 *   after power off the modem.
 ****************************************************************************/

static void app_poweroff_cb(uint32_t result)
{
  printf("%s called\n", __func__);

  /* Notify the result to the lte_http_get sample application task */

  app_mq_notify_result(result);
}

/****************************************************************************
 * Name: app_set_apn_cb
 *
 * Description:
 *   This callback is called when the APN setting is completed.
 ****************************************************************************/

static void app_set_apn_cb(uint32_t result)
{
  printf("%s called\n", __func__);

  /* Notify the result to the lte_http_get sample application task */

  app_mq_notify_result(result);
}

/****************************************************************************
 * Name: app_attach_net_cb
 *
 * Description:
 *   This callback is called when connect to the LTE network is completed.
 ****************************************************************************/

static void app_attach_net_cb(uint32_t result, uint32_t errcause)
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

  /* Notify the result to the lte_http_get sample application task */

  app_mq_notify_result(result);
}

/****************************************************************************
 * Name: app_detach_net_cb
 *
 * Description:
 *   This callback is called when disconnect to the LTE network is completed.
 ****************************************************************************/

static void app_detach_net_cb(uint32_t result)
{
  printf("%s called: result:%d\n", __func__, result);

  /* Notify the result to the lte_http_get sample application task */

  app_mq_notify_result(result);
}

/****************************************************************************
 * Name: app_data_on_cb
 *
 * Description:
 *   This callback is called when data on is completed.
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

  /* Notify the result to the lte_http_get sample application task */

  app_mq_notify_result(result);
}

/****************************************************************************
 * Name: app_localtime_report_cb
 *
 * Description:
 *   This callback is called when local time has changed.
 ****************************************************************************/

static void app_localtime_report_cb(FAR lte_localtime_t *localtime)
{
  printf("%s called: localtime : \"%02d/%02d/%02d : %02d:%02d:%02d\"\n",
         __func__, localtime->year, localtime->mon, localtime->mday,
         localtime->hour, localtime->min, localtime->sec);

  app_lte_setlocaltime(localtime);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * app_connect_to_lte
 ****************************************************************************/

int app_connect_to_lte(void)
{
  int ret;
  int result = LTE_RESULT_OK;

  /* Create a message queue. It is used to receive result from the
   * asynchronous API callback.*/

  ret = app_mq_create(APP_MQUEUE_NAME);
  if (ret < 0)
    {
      goto errout;
    }

  /* Initialize the LTE library */

  ret = lte_initialize();
  if (ret < 0)
    {
      printf("Failed to initialize LTE library :%d\n", ret);
      goto errout_with_fin;
    }

  /* Power on the LTE modem */

  ret = lte_power_control(LTE_POWERON, app_poweron_cb);
  if (ret < 0)
    {
      printf("Failed to power on the modem :%d\n", ret);
      goto errout_with_lte_fin;
    }

  /* Wait until the modem startup normally and notification
   * comes from the callback(app_poweron_cb)
   * registered by lte_power_control. */

  ret = app_wait_lte_callback(&result);
  if ((ret < 0) || (result != LTE_RESULT_OK))
    {
      goto errout_with_lte_fin;
    }

  /* Enable to receive events of local time change */

  ret = lte_set_report_localtime(app_localtime_report_cb);
  if (ret < 0)
    {
      printf("Failed to set report local time :%d\n", ret);
      goto errout_with_lte_fin;
    }

  /* Do APN setting for session ID 1 which is used by lte_attach_network.
   * It is necessary to set the correct APN according to your environment. */

  ret = lte_set_apn(APP_SESSION_ID, (int8_t*)APP_APN_NAME, APP_APN_IPTYPE,
                    APP_APN_AUTHTYPE, (int8_t*)APP_APN_USR_NAME,
                    (int8_t*)APP_APN_PASSWD, app_set_apn_cb);
  if (ret < 0)
    {
      printf("Failed to set access point name :%d\n", ret);
      goto errout_with_lte_fin;
    }

  /* Wait until the APN setting is completed and notification
   * comes from the callback(app_set_apn_cb)
   * registered by lte_set_apn. */

  ret = app_wait_lte_callback(&result);
  if ((ret < 0) || (result == LTE_RESULT_ERROR))
    {
      goto errout_with_lte_fin;
    }

  /* Attach to LTE network */

  ret = lte_attach_network(app_attach_net_cb);
  if (ret < 0)
    {
      printf("Failed to attach to LTE network :%d\n", ret);
      goto errout_with_lte_fin;
    }

  /* Wait until the LTE network is connected and notification
   * comes from the callback(app_attach_net_cb)
   * registered by lte_attach_network. */

  ret = app_wait_lte_callback(&result);
  if ((ret < 0) || (result == LTE_RESULT_ERROR))
    {
      goto errout_with_lte_fin;
    }

  /* Data on */

  ret = lte_data_on(APP_SESSION_ID, app_data_on_cb);
  if (ret < 0)
    {
      printf("Failed to data on :%d\n", ret);
      goto errout_with_lte_fin;
    }

  /* Wait until the data on completed and notification
   * comes from the callback(app_attach_net_cb)
   * registered by lte_data_on. */

  ret = app_wait_lte_callback(&result);
  if ((ret < 0) || (result == LTE_RESULT_ERROR))
    {
      goto errout_with_lte_fin;
    }

  return 0;

errout_with_lte_fin:
  lte_finalize();

errout_with_fin:
  app_mq_delete(APP_MQUEUE_NAME);

errout:
  return -1;
}

int app_disconnect_from_lte(void)
{
  int ret;
  int result = LTE_RESULT_OK;

  /* Detach from LTE network */

  ret = lte_detach_network(app_detach_net_cb);
  if (ret < 0)
    {
      printf("Failed to detach from LTE network :%d\n", ret);
      goto errout_with_lte_fin;
    }

  /* Wait until the LTE network is disconnected and notification
   * comes from the callback(app_detach_net_cb)
   * registered by lte_detach_network.
   */

  ret = app_wait_lte_callback(&result);
  if ((ret < 0) || (result == LTE_RESULT_ERROR))
    {
      goto errout_with_lte_fin;
    }

  /* Power off the modem. If asynchronous API has not notified
   * the result by callback, it will be canceled
   */

  ret = lte_power_control(LTE_POWEROFF, app_poweroff_cb);
  if (ret < 0)
    {
      printf("Failed to power off the modem :%d\n", ret);
      goto errout_with_lte_fin;
    }

  /* Wait until the modem shutdown complete and notification
   * comes from the callback(app_poweroff_cb)
   * registered by lte_power_control.
   */

  ret = app_wait_lte_callback(&result);
  if ((ret < 0) || (result != LTE_RESULT_OK))
    {
      goto errout_with_lte_fin;
    }

  /* Finalize LTE library
   * If this function is called while the modem power is on,
   * shutdown the modem
   */

  ret = lte_finalize();
  if (ret < 0)
    {
      printf("Failed to finalize LTE library :%d\n", ret);
      goto errout_with_fin;
    }

  /* Delete a message queue */

  app_mq_delete(APP_MQUEUE_NAME);

  return 0;

errout_with_lte_fin:
  lte_finalize();

errout_with_fin:
  app_mq_delete(APP_MQUEUE_NAME);

  return -1;
}
