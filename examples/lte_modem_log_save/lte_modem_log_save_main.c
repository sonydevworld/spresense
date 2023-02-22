/****************************************************************************
 * lte_modem_log_save/lte_modem_log_save_main.c
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

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <pthread.h>
#include <lte/lte_api.h>
#include <lte/lte_log.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_mutex_t g_restart_mutex;
static pthread_cond_t  g_restart_cond;
static bool g_vererr = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: app_restart_cb
 *
 * Description:
 *   This callback is called when the startup is completed
 *   after power on the modem.
 ****************************************************************************/

static void app_restart_cb(uint32_t reason)
{
  int ret = 0;
  char *reason_string[] =
  {
    "Modem restart by application.",
    "Modem restart by self.",
    "Modem version mismatch."
  };

  if (reason == LTE_RESTART_VERSION_ERROR)
    {
      g_vererr = true;
    }

  /* Send a signal only for the first call. */

  pthread_mutex_lock(&g_restart_mutex);
  ret = pthread_cond_signal(&g_restart_cond);
  if (ret < 0)
    {
      printf("Failed to pthread_cond_signal:%d\n", ret);
    }

  pthread_mutex_unlock(&g_restart_mutex);

  printf("%s\n", reason_string[reason]);
}

/****************************************************************************
 * Name: app_cond_init
 ****************************************************************************/

static int app_cond_init(void)
{
  int ret;

  ret = pthread_mutex_init(&g_restart_mutex, NULL);
  if (ret < 0)
    {
      printf("Failed to pthread_mutex_init:%d", ret);
      return ret;
    }

  ret = pthread_cond_init(&g_restart_cond, NULL);

  if (ret < 0)
    {
      printf("Failed to pthread_cond_init:%d", ret);
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: app_cond_fin
 ****************************************************************************/

static int app_cond_fin(void)
{
  int ret;

  ret = pthread_mutex_destroy(&g_restart_mutex);
  if (ret < 0)
    {
      printf("Failed to pthread_mutex_destroy:%d", ret);
      return ret;
    }

  ret = pthread_cond_destroy(&g_restart_cond);
  if (ret < 0)
    {
      printf("Failed to pthread_cond_destroy:%d", ret);
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: app_lte_poweron
 ****************************************************************************/

static int app_lte_poweron(void)
{
  int ret = 0;
  lte_version_t version = {0};

  /* Initialize the LTE library */

  ret = lte_initialize();
  if ((ret < 0) && (ret != -EALREADY))
    {
      printf("Failed to initialize LTE library :%d\n", ret);
      return ret;
    }

  /* Set callback for notification of Modem restart */

  ret = lte_set_report_restart(CODE app_restart_cb);
  if ((ret < 0) && (ret != -EALREADY))
    {
      printf("Failed to set Restart Callback :%d\n", ret);
      return ret;
    }

  /* Initialize condition signal to wait for modem poweron */

  ret = app_cond_init();
  if (ret < 0)
    {
      printf("Failed to cond initialize :%d\n", ret);
      return ret;
    }

  pthread_mutex_lock(&g_restart_mutex);

  /* Power on the LTE modem */

  ret = lte_power_on();
  if (ret == 0)
    {
      /* Wait until the modem startup normally and notification
       * comes from the callback(app_restart_cb)
       * registered by lte_power_control.
       */

      pthread_cond_wait(&g_restart_cond, &g_restart_mutex);
      pthread_mutex_unlock(&g_restart_mutex);
    }
  else if ((ret < 0) && (ret != -EALREADY))
    {
      printf("Failed to power on the modem :%d\n", ret);
      pthread_mutex_unlock(&g_restart_mutex);
      return ret;
    }

  ret = app_cond_fin();

  if (ret != 0)
    {
      printf("Failed to cond finalize :%d\n", ret);

      return ret;
    }

  if (g_vererr)
    {
      g_vererr = false;
      printf("Please enable the disabled protocol version"
             " and flash the application.\n");

      ret = lte_get_version_sync(&version);
      if (ret == 0)
        {
          printf("Modem IC Type : %s\n", version.bb_product);
          printf("      FW Ver. : %s\n", version.np_package);
        }

      lte_finalize();
      return -EPERM;
    }

  return 0;
}

/****************************************************************************
 * Name: app_lte_poweroff
 ****************************************************************************/

static int app_lte_poweroff(void)
{
  int ret = 0;

  ret = lte_power_off();
  if (ret < 0)
    {
      printf("Failed to power off the modem :%d\n", ret);
    }

  ret = lte_finalize();
  if (ret < 0)
    {
      printf("Failed to finalize LTE library :%d\n", ret);
    }
  else if (ret == 0)
    {
      printf("Modem Power OFF.\n");
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * lte_modem_log_save_main
 * Description:
 * This application saves modem FW logs.
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret = 0;
  int get_num = 0;
  int i = 0;
  char filelist[LTE_LOG_LIST_SIZE][LTE_LOG_NAME_LEN];
  char filename[LTE_LOG_NAME_LEN];
  lte_errinfo_t info;

  ret = app_lte_poweron();
  if (ret < 0)
    {
      return ret;
    }

  /* Collect logs */

  ret = lte_log_collect(filename, LTE_LOG_NAME_LEN);
  if (ret < 0)
    {
      printf("lte_log_collect() error. ret=%d\n", ret);
      if (ret == -EPROTO)
        {
          lte_get_errinfo(&info);
          printf("errno=%ld [%s]\n", info.err_no, info.err_string);
        }

      goto exit;
    }

  printf("Saved the log as %s\n", filename);

  /* Get collect logs file list */

  get_num = lte_log_getlist(LTE_LOG_LIST_SIZE,
                            LTE_LOG_NAME_LEN, filelist);
  if (get_num < 0)
    {
      printf("lte_log_getlist() error. ret=%d\n", get_num);
      if (get_num == -EPROTO)
        {
          lte_get_errinfo(&info);
          printf("errno=%ld [%s]\n", info.err_no, info.err_string);
        }

      goto exit;
    }

  printf("The file name of the log stored in the modem:\n");
  for (i = 0; i < get_num; i++)
    {
      printf("\t%s\n", filelist[i]);
    }

exit:
  ret = app_lte_poweroff();

  return ret;
}
