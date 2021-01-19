/****************************************************************************
 * lte_rat/lte_rat_main.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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
#include <string.h>
#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>

#include "lte/lte_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APP_MQUEUE_NAME    "lte_rat_sample_queue"
#define APP_MAX_MQUEUE_MSG 1
#define APP_MQUEUE_MODE    0666

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const int g_app_rat_table[] =
{
  LTE_RAT_CATM,
  LTE_RAT_NBIOT
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
      printf("mq_receive() failed: %d\n", errcode);
      mq_close(mqd);
      return -1;
    }
  mq_close(mqd);

  *result = buffer;

  return 0;
}

/****************************************************************************
 * Name: app_restart_cb
 *
 * Description:
 *   This callback is called when the startup is completed
 *   after power on the modem.
 ****************************************************************************/

static void app_restart_cb(uint32_t reason)
{

  printf("Modem powered on: %d\n", reason);

  if (reason == LTE_RESTART_USER_INITIATED)
    {

      /* Notify the result to the lte_rat sample application task */

      app_mq_notify_result(reason);
    }
}

/****************************************************************************
 * app_lte_poweron
 ****************************************************************************/

static int app_lte_poweron(void)
{
  int            ret    = 0;
  int            result = LTE_RESULT_OK;
  int            errcode;
  mqd_t          mqd;
  struct mq_attr mq_attr;

  mq_attr.mq_maxmsg  = APP_MAX_MQUEUE_MSG;
  mq_attr.mq_msgsize = sizeof(int);
  mq_attr.mq_flags   = 0;

  /* Create message queue */

  mqd = mq_open(APP_MQUEUE_NAME,
                (O_RDWR | O_CREAT), APP_MQUEUE_MODE, &mq_attr);
  if (mqd < 0)
    {
      errcode = errno;
      printf("mq_open() failed: %d\n", errcode);
      return -1;
    }
  mq_close(mqd);

  /* Initialize the LTE library */

  ret = lte_initialize();
  if ((ret < 0) && (ret != -EALREADY))
    {
      printf("Failed to initialize LTE library :%d\n", ret);
      goto errout;
    }

  /* Register callback for modem restart.
   * It must call this function after lte_initialize.
   * The callback will be invoked if the modem starts successfully
   * after calling lte_power_on.
   */

  ret = lte_set_report_restart(app_restart_cb);
  if (ret < 0)
    {
      printf("Failed to set report restart :%d\n", ret);
      goto errout;
    }

  /* Power on the LTE modem */

  ret = lte_power_on();
  if (ret >= 0)
    {

      /* Wait until the modem startup normally and notification
       * comes from the callback(app_restart_cb)
       * registered by lte_set_report_restart.
       */

      ret = app_wait_lte_callback(&result);
      if (ret < 0)
        {
          goto errout;
        }
    }
  else if (ret == -EALREADY)
    {

      /* Already powered on
       * Nothing to do.
       */

    }
  else
    {
      printf("Failed to power on the modem :%d\n", ret);
      goto errout;
    }

  return ret;

errout:
  lte_finalize();

  /* Delete a message queue */

  mq_unlink(APP_MQUEUE_NAME);
  return ret;

}

/****************************************************************************
 * app_print_help()
 ****************************************************************************/

static void app_print_help(void)
{
  printf("***************************************************\n");
  printf("Usage: lte_rat [-s <0-1>]\n");
  printf("-s : Set RAT to USE.\n");
  printf("0 : Cat.M, 1 : NB-IoT\n");
  printf("Outputs the current RAT if no option is attached\n");
  printf("\ne.g.\n");
  printf("- Display the currently set RAT\n");
  printf("> lte_rat\n");
  printf("- Set RAT to Cat.M1\n");
  printf("> lte_rat -s 0\n");
  printf("***************************************************\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * lte_rat_main
 * Description:
 *   This application sets/checks the RAT(Radio Access Technology)
 *   of the modem.
 *
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret     = 0;
  int opt_idx = 0;

  ret = app_lte_poweron();

  if (ret < 0 && ret != -EALREADY)
    {
      return ret;
    }

  if (argc > 1)
    {

      if (strncmp(argv[1], "-s", strlen("-s")) == 0)
        {
          if (argc > 2)
            {
              opt_idx = atoi(argv[2]);

              /* Set RAT setting
                * The first parameter is the type of RAT and
                * the second parameter is a flag to keep this setting
                * after the modem is powered off.
                */

              ret = lte_set_rat_sync((uint8_t)g_app_rat_table[opt_idx], true);
              if (ret < 0)
                {
                  if (ret == -ENOTSUP)
                    {
                      printf("Sorry. This modem does not support RAT API.\n");
                    }
                  else
                    {
                      printf("Failed to RAT set: %d\n", ret);
                    }
                }
              else
                {
                  printf("RAT setting succeeded: %d\n", ret);
                }
            }
          else
            {
              printf("Please specify RAT type.\n");
            }
        }
      else if (strncmp(argv[1], "-h", strlen("-h")) == 0)
        {
          app_print_help();
        }
      else
        {
          printf("Unexpected option.\n");
        }
    }
  else
    {

      /* Get current RAT type */

      ret = lte_get_rat_sync();
      if (ret < 0)
        {
          if (ret == -ENOTSUP)
            {
              printf("Sorry. This modem does not support RAT API.\n");
            }
          else
            {
              printf("Failed to get RAT: %d\n", ret);
            }
        }
      else
        {
          switch(ret)
            {
              case LTE_RAT_CATM:
                printf("The current RAT is Cat.M\n");
                break;
              case LTE_RAT_NBIOT:
                printf("The current RAT is NB-IoT\n");
                break;
              default:
                printf("The current RAT is unknown type: %d\n", ret);
                break;
            }
        }
    }

  /* Restart callback unregistered. */
  
  lte_set_report_restart(NULL);

  /* Delete a message queue */

  mq_unlink(APP_MQUEUE_NAME);

  return ret;

}
