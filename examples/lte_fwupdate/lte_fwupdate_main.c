/****************************************************************************
 * examples/lte_fwupdate/lte_fwupdate_main.c
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
 * 3. Neither the name Sony Semiconductor Solutions nor the names of
 *    its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
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
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>

#include <lte/lte_api.h>
#include <lte/lte_fwupdate.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PROGRESS_BAR_LEN  (50)
#define INJECTION_SIZE  (1024)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char fw_data[INJECTION_SIZE];

static volatile bool modem_boot_up = false;
static volatile bool modem_restarted = false;
static volatile bool modem_ver_err = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void modem_restart_cb(uint32_t reason)
{
  if(reason == LTE_RESTART_USER_INITIATED)
    {
      printf("Modem is booted up.\n");
      modem_boot_up = true;
    }
  else if (reason == LTE_RESTART_VERSION_ERROR)
    {
      printf("Modem version mismatch.\n");
      modem_boot_up = true;
      modem_ver_err = true;
    }
  else
    {
      /* Modem is reset by itself with any critical reason
       * or finished FW update.
       * When this case is happened. Modem status is the same
       * as just after lte_power_on() is called.
       */

      modem_restarted = true;
    }
}

static void wait_for_modem_bootup(int ret)
{
  if (ret != -EALREADY)
    {
      while (modem_boot_up == false)
        {
          usleep(1);
        }
    }
}

static void wait_for_modem_restart(void)
{
  int i = 0;
  printf("\n");
  while (modem_restarted == false)
    {
      sleep(1);
      i++;
      printf("\r.. %d seconds is passed.\n", i);
    }
}

static int file_size(const char* path)
{
  int ret = -1;
  struct stat st;

  if (stat(path, &st) == 0)
    {
      if (S_ISREG(st.st_mode))
        {
          ret = (int)st.st_size;
        }
    }
  return ret;
}

static int fwimage_injection(const char *fname, int sz, int injected_size)
{
  int i;
  int ret;
  int read_size;
  FILE *fp;
  int inject_num;

  int step_len = sz / PROGRESS_BAR_LEN;
  int step_cur = 0;
  int inc_steps;

  fp = fopen(fname, "rb");
  if (!fp)
    {
      return -1;
    }

  if (injected_size != 0)
    {
      fseek(fp, (long)injected_size, SEEK_SET);
    }

  printf("Start injection update image into modem..\n");

  /* To show initial status bar */

  if (step_len)
    {
      printf("0");
      for (i = 1; i < (PROGRESS_BAR_LEN - 3); i++)
        {
          printf("-");
        }
      printf("100\n");
      step_cur = injected_size / step_len;
      for (i = 0; i < step_cur; i++)
        {
          printf("#");
        }
      fflush(stdout);
    }

  /* Start reading data and injection */
  /* To call injection, lte_get_version_sync() and
   * ltefwupdate_injected_datasize() must be called before.
   */

  while (injected_size < sz)
    {
      read_size = sz - injected_size;
      read_size
        = (read_size > INJECTION_SIZE) ? INJECTION_SIZE : read_size;

      ret = fread(fw_data, 1, read_size, fp);
      if (ret <= 0)
        {
          printf("Failed: Reading data from the file.\n");
          fclose(fp);
          return ret;
        }

      read_size = (read_size != ret) ? ret : read_size;

      /* actual injection execute */

      inject_num = 0;
      while (inject_num < read_size)
        {
          if (injected_size == 0)
            {
              /* At start, you need use ltefwupdate_initialize() */

              ret = ltefwupdate_initialize(&fw_data[inject_num],
                  read_size - inject_num);
            }
          else
            {
              /* If you want to inject from the continuation,
               * use ltefwupdate_injectrest()
               */

              ret = ltefwupdate_injectrest(&fw_data[inject_num],
                  read_size - inject_num);
            }

          if (ret < 0)
            {
              printf("Failed: Injection was failed.\n");
              fclose(fp);
              return ret;
            }

          inject_num += ret;
          injected_size += ret;
        }

      /* Progress bar maintainance */

      if (step_len)
        {
          if (step_cur < (injected_size / step_len))
            {
              inc_steps = (injected_size / step_len) - step_cur;
              for (i = 0; i < inc_steps; i++)
                {
                  printf("#");
                }
              fflush(stdout);
              step_cur += inc_steps;
            }
        }
    }
  printf("\nInjection done.\n\n");

  fclose(fp);
  return sz;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;
  int injected_size;
  lte_version_t version;
  int img_size;
  char user_resp[32] = {0};

  printf("= Alt1250 modem FW update example =\n\n");

  if (argc != 2)
    {
      printf("Need update image file path\n");
      printf("Usage : nsh> %s <file path>\n", argv[0]);
      return -1;
    }

  modem_boot_up = false;
  modem_restarted = false;
  modem_ver_err = false;

  img_size = file_size(argv[1]);
  if (img_size < 0)
    {
      printf("Couldn't find %s\n", argv[1]);
      return -1;
    }

  printf("Booting up LTE board...\n\n");
  ret = lte_initialize();
  if ((ret < 0) && (ret != -EALREADY))
    {
      printf("Initialization failed..:%d\n", ret);
      return -1;
    }

  ret = lte_set_report_restart(modem_restart_cb);
  if (ret < 0)
    {
      printf("Set restart call back failed...%d\n", ret);
      lte_finalize();
      return -1;
    }

  ret = lte_power_on();
  if ((ret < 0) && (ret != -EALREADY))
    {
      printf("Alt1250 power on failed...%d\n", ret);
      lte_finalize();
      return -1;
    }

  wait_for_modem_bootup(ret);

  ret = lte_get_version_sync(&version);
  if (ret != 0)
    {
      printf("Couldn't get lte version.. err(%d)\n", ret);
      lte_finalize();
      return -1;
    }

  printf("Modem IC Type : %s\n", version.bb_product);
  printf("      FW Ver. : %s\n", version.np_package);

  if (modem_ver_err)
    {
      printf("Please enable the disabled protocol version"
             " and flash the application.\n");
      lte_finalize();
      return -1;
    }

  printf("Confirm if the FW Version is the target version"
         " you want to update.\n\n");

  printf("CAUTION : DO NOT turn off the power of your device\n"
         "          until finish this example.\n"
         "          In the worst case, the modem may be damaged.\n\n");
  printf("Are you sure to start?\n"
         "Type \"yes\" and press enter, when you are ready.\n"
         "If you want to stop, type any other word and press enter.\n"
         "Ready ? : ");
  fflush(stdout);
  fgets(user_resp, sizeof(user_resp), stdin);
  if (strncmp(user_resp, "yes", 3))
    {
      printf("Stopped.\n");
      lte_finalize();
      return -1;
    }
  printf("\n\n");

  injected_size = ltefwupdate_injected_datasize();
  if (ret < 0)
    {
      printf("Couldn't get injected data size err(%d)\n", injected_size);
      lte_finalize();
      return -1;
    }
  printf("Injected data size already = %d\n", injected_size);

  ret = fwimage_injection(argv[1], img_size, injected_size);
  if (ret < 0)
    {
      printf("Injection is failed.. : %d\n", ret);
      lte_finalize();
      return -1;
    }

  modem_restarted = false;

  /* During the update run, the modem must be woken up using
   * lte_acquire_wakelock() to safely update the modem.
   */

  lte_acquire_wakelock();
  ret = ltefwupdate_execute();
  if (ret < 0)
    {
      printf("Execution is failed.. : %d\n", ret);
      lte_release_wakelock();
      lte_finalize();
      return -1;
    }
  printf("execution = [%d]\n", ret);

  printf("Now FW update is executing.."
         "It can take several minutes.\n"
         "Please wait.\n");

  wait_for_modem_restart();
  lte_release_wakelock();

  printf("\nNow the modem is rebooted.\n"
           "Check updated status\n");

  ret = ltefwupdate_result();
  printf("Result = [%d]\n", ret);

  ret = lte_get_version_sync(&version);
  if (ret != 0)
    {
      printf("Couldn't get lte version.. err(%d)\n", ret);
      lte_finalize();
      return -1;
    }

  printf("Modem version after updated.\n");
  printf("Modem IC Type : %s\n", version.bb_product);
  printf("      FW Ver. : %s\n", version.np_package);
  printf("Done.\n");

  lte_finalize();

  return 0;
}
