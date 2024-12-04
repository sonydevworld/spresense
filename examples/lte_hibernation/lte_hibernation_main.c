/****************************************************************************
 * examples/lte_hibernation/lte_hibernation_main.c
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <arch/chip/pm.h>
#include <nuttx/timers/rtc.h>
#include <arch/board/board.h>

#include "netutils/webclient.h"
#include "lte/lte_api.h"
#include "lte_connection.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LTE_HIBERNATION_CONTEXT_PATH "/mnt/spif/.ltectx"
#define LTE_HIBERNATION_INTERVAL    10 * 60

#define ALARM_DEVPATH "/dev/rtc0"
#define ALARM_SIGNO 1

#define APP_IOBUFFER_LEN   512
#define APP_WGET_URL       "http://example.com"

/* APN settings */

#ifdef CONFIG_EXAMPLES_LTE_HIBERNATION_APN_IPTYPE_IPV6
#  define APP_APN_IPTYPE   LTE_IPTYPE_V6
#elif defined CONFIG_EXAMPLES_LTE_HIBERNATION_APN_IPTYPE_IPV4V6
#  define APP_APN_IPTYPE   LTE_IPTYPE_V4V6
#else
#  define APP_APN_IPTYPE   LTE_IPTYPE_V4
#endif

#ifdef CONFIG_EXAMPLES_LTE_HIBERNATION_APN_AUTHTYPE_PAP
#  define APP_APN_AUTHTYPE LTE_APN_AUTHTYPE_PAP
#elif defined CONFIG_EXAMPLES_LTE_HIBERNATION_APN_AUTHTYPE_CHAP
#  define APP_APN_AUTHTYPE LTE_APN_AUTHTYPE_CHAP
#else
#  define APP_APN_AUTHTYPE LTE_APN_AUTHTYPE_NONE
#endif

#define MATCH_STRING(str1, str2) ((strlen(str1) == strlen(str2)) && \
                                  (strncmp(str1, str2, strlen(str2)) == 0))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_app_iobuffer[APP_IOBUFFER_LEN];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_file_size
 *
 * Description:
 *   Get the size of the specified file.
 ****************************************************************************/

static int get_file_size(char *filename)
{
  struct stat tmp;

  if (stat(filename, &tmp) == 0)
    {
      return tmp.st_size;
    }

  return -1;
}

/****************************************************************************
 * Name: set_rtc_alarm
 *
 * Description:
 *   This function set a RTC alarm in specified seconds.
 ****************************************************************************/

static int set_rtc_alarm(const time_t seconds)
{
  int ret;
  int fd;
  struct rtc_setrelative_s setrel;

  /* Clear all boot mask */

  up_pm_clr_bootmask((uint32_t)-1);

  /* Set boot mask RTC and RTC-Alarm0 */

  up_pm_set_bootmask(PM_BOOT_COLD_RTC | PM_BOOT_COLD_RTC_ALM0);

  fd = open(ALARM_DEVPATH, O_WRONLY);
  if (fd < 0)
    {
      printf("Could not open %s\n", ALARM_DEVPATH);
      return -1;
    }

  /* Set the alarm expired after the specified time */

  setrel.id      = 0;
  setrel.pid     = getpid();
  setrel.reltime = seconds;

  setrel.event.sigev_notify = SIGEV_SIGNAL;
  setrel.event.sigev_signo  = ALARM_SIGNO;
  setrel.event.sigev_value.sival_int = 0;

  ret = ioctl(fd, RTC_SET_RELATIVE, (unsigned long)&setrel);
  close(fd);
  if (ret < 0)
    {
      printf("Could not set alarm..\n");
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: context_save_cb
 *
 * Description:
 *   Function to receive context data that needs to be saved when the LTE
 *   enters hibernation mode.
 *   The context data needs to save in this function.
 ****************************************************************************/

static void context_save_cb(uint8_t *data, int size)
{
  struct stat tmp;
  int fd;
  size_t s;

  /* If previous context data exists, remove it. */

  if (stat(LTE_HIBERNATION_CONTEXT_PATH, &tmp) == 0)
    {
      unlink(LTE_HIBERNATION_CONTEXT_PATH);
    }

  fd = open(LTE_HIBERNATION_CONTEXT_PATH, O_RDONLY | O_WRONLY | O_CREAT);
  if (fd < 0)
    {
      printf("Failed to save context data to %s.\n",
             LTE_HIBERNATION_CONTEXT_PATH);
      return;
    }

  s = write(fd, data, size);
  if (s != size)
    {
      printf("Failed to write context data.\n");
    }

  close(fd);
}

/****************************************************************************
 * Name: lte_enter_hibernation
 *
 * Description:
 *   Function to transition LTE functions to hibernation mode.
 ****************************************************************************/

static int lte_enter_hibernation(void)
{
  int ret;
  struct boardioc_pm_ctrl_s pmc = {
    .action = BOARDIOC_PM_CHANGESTATE,
    .domain = BOARD_PM_APPS,
    .state  = PM_SLEEP
  };

  ret = lte_set_context_save_cb(context_save_cb);
  if (ret < 0)
    {
      printf("lte_set_context_save_cb failed (%d)\n", ret);
      return ret;

    }

  ret = boardctl(BOARDIOC_PM_CONTROL, (uintptr_t) &pmc);
  if (ret < 0)
    {
      printf("Sleep failed.\n");
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: lte_resume_from_hibernation
 *
 * Description:
 *   Function to return LTE functions from hibernation mode.
 ****************************************************************************/

static int lte_resume_from_hibernation(void)
{
  int size = 0;
  int ctx_size;
  int ret = 0;
  int fd = 0;
  uint8_t data[64];

  ret = lte_initialize();
  if (ret < 0 && ret != -EALREADY)
    {
      printf("lte_initialize failed.(%d)\n", ret);
      return -1;
    }

  size = get_file_size(LTE_HIBERNATION_CONTEXT_PATH);
  if (size > 0)
    {
      /* Get the context size required for resume. */

      ctx_size = lte_hibernation_resume(NULL, 0);
      if (ctx_size != size)
        {
          printf("File size does not match."
                 " file size = %d, context size = %d\n",
                 size, ctx_size);
          ret = -1;
        }
      else
        {
          fd = open(LTE_HIBERNATION_CONTEXT_PATH, O_RDONLY);

          do
            {
              size = read(fd, data, sizeof(data));
              ret = lte_hibernation_resume(data, size);
              if (ret < 0)
                {
                  printf("lte_hibernation_resume failed.\n");
                }
              else
                {
                  printf("remain size = %d\n", ret);
                }
            }
          while (ret > 0);

          close(fd);
        }

      unlink(LTE_HIBERNATION_CONTEXT_PATH);
    }
  else
    {
      ret = -1;
    }

  if (ret != 0)
    {
      lte_finalize();
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: app_wget_initialize
 *
 * Description:
 *   This function initialize a context that contains a webclient setting.
 ****************************************************************************/

static void app_wget_initialize(struct webclient_context *ctx,
                                FAR char *buffer, int buflen,
                                webclient_sink_callback_t callback)
{
  webclient_set_defaults(ctx);

  ctx->method = "GET";
  ctx->buffer = buffer;
  ctx->buflen = buflen;
  ctx->sink_callback = callback;
  ctx->sink_callback_arg = NULL;
}

/****************************************************************************
 * Name: app_wget_cb
 *
 * Description:
 *   As data is obtained from the host, this function is to output of
 *   each block of file data as it is received.
 ****************************************************************************/

static int app_wget_cb(FAR char **buffer, int offset, int datend,
                        FAR int *buflen, FAR void *arg)
{
  /* Write HTTP data to standard output */

  (void)write(1, &((*buffer)[offset]), datend - offset);

  return 0;
}

/****************************************************************************
 * Name: perform_wget
 ****************************************************************************/

static int perform_wget(void)
{
  int ret;
  struct webclient_context ctx;

  /* Initialilze a webclient context */

  app_wget_initialize(&ctx, g_app_iobuffer, APP_IOBUFFER_LEN, app_wget_cb);

  ctx.url = APP_WGET_URL;

  /* Retrieve the file with the specified URL. */

  ret = webclient_perform(&ctx);

  if (ret != 0)
    {
      printf("webclient_perform failed with %d\n", ret);
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: ip_parse
 ****************************************************************************/

static int ip_parse(FAR char *s)
{
  int ret = -1;

  if (MATCH_STRING(s, "v4v6"))
    {
      ret = LTE_IPTYPE_V4V6;
    }
  else if (MATCH_STRING(s, "v4"))
    {
      ret = LTE_IPTYPE_V4;
    }
  else if (MATCH_STRING(s, "v6"))
    {
      ret = LTE_IPTYPE_V6;
    }

  return ret;
}

/****************************************************************************
 * Name: auth_parse
 ****************************************************************************/

static int auth_parse(FAR char *s)
{
  int ret = -1;

  if (MATCH_STRING(s, "none"))
    {
      ret = LTE_APN_AUTHTYPE_NONE;
    }
  else if (MATCH_STRING(s, "pap"))
    {
      ret = LTE_APN_AUTHTYPE_PAP;
    }
  else if (MATCH_STRING(s, "chap"))
    {
      ret = LTE_APN_AUTHTYPE_CHAP;
    }

  return ret;
}

/****************************************************************************
 * Name: show_usage
 ****************************************************************************/

static void show_usage(FAR const char *progname, int exitcode)
{
  fprintf(stderr, "\nUSAGE: %s command\n", progname);
  fprintf(stderr, " [-a <apn_name>] [-i <ip_type>] [-t <auth_type>]"
                  " [-u <user_name>] [-p <password>]\n");
  fprintf(stderr, "  -a: APN name\n");
  fprintf(stderr, "  -i: IP type : v4 or v6 or v4v6\n");
  fprintf(stderr, "  -t: Authenticaion type : none or pap or chap\n");
  fprintf(stderr, "  -u: User name for authenticaion\n");
  fprintf(stderr, "  -p: Password for authenticaion\n");
  fprintf(stderr, " [-h]: Show this message\n");
  exit(exitcode);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;
  int opt;
  struct lte_apn_setting apnsetting;
  int ip_type;
  int auth_type;

  apnsetting.apn =  CONFIG_EXAMPLES_LTE_HIBERNATION_APN_NAME;
  apnsetting.ip_type   = APP_APN_IPTYPE;
  apnsetting.auth_type = APP_APN_AUTHTYPE;
  apnsetting.apn_type  = LTE_APN_TYPE_DEFAULT | LTE_APN_TYPE_IA;
  apnsetting.user_name =
    CONFIG_EXAMPLES_LTE_HIBERNATION_APN_USERNAME;
  apnsetting.password  =
    CONFIG_EXAMPLES_LTE_HIBERNATION_APN_PASSWD;

  optind = -1;
  while ((opt = getopt(argc, argv, "a:i:t:u:p:h")) != -1)
    {
      switch (opt)
        {
          case 'a':
            if (strlen(optarg) >= LTE_APN_LEN)
              {
                fprintf(stderr, "APN name is too long\n");
                show_usage(argv[0], EXIT_FAILURE);
              }

            apnsetting.apn = optarg;
            break;
          case 'i':
            ip_type = ip_parse(optarg);
            if (ip_type == -1)
              {
                fprintf(stderr, "Invalid IP type:%s\n", optarg);
                show_usage(argv[0], EXIT_FAILURE);
              }

            apnsetting.ip_type = ip_type;
            break;
          case 't':
            auth_type = auth_parse(optarg);
            if (auth_type == -1)
              {
                fprintf(stderr, "Invalid authenticaion type:%s\n", optarg);
                show_usage(argv[0], EXIT_FAILURE);
              }

            apnsetting.auth_type = auth_type;
            break;
          case 'u':
            if (strlen(optarg) >= LTE_APN_USER_NAME_LEN)
              {
                fprintf(stderr, "User name is too long\n");
                show_usage(argv[0], EXIT_FAILURE);
              }

            apnsetting.user_name = optarg;
            break;
          case 'p':
            if (strlen(optarg) >= LTE_APN_PASSWD_LEN)
              {
                fprintf(stderr, "Password is too long\n");
                show_usage(argv[0], EXIT_FAILURE);
              }

            apnsetting.password = optarg;
            break;
          case 'h':
            show_usage(argv[0], EXIT_SUCCESS);
            break;
          default:
            show_usage(argv[0], EXIT_FAILURE);
            break;
        }
    }

  if (up_pm_get_bootcause() &
      (PM_BOOT_COLD_RTC | PM_BOOT_COLD_RTC_ALM0 | PM_BOOT_COLD_GPIO))
    {
      /* Resume from suspend mode */

      printf("Resume from LTE hibernation mode.\n");

      ret = lte_resume_from_hibernation();
      if (ret == 0)
        {
          /* Run wget */

          ret = perform_wget();
          if (ret < 0)
            {
              return -1;
            }

          printf("End of this sample\n");
          return 0;
        }
    }

  /* Normal boot */

  printf("Turn On LTE.\n");

  ret = app_connect_to_lte(&apnsetting);
  if (ret < 0)
    {
      return -1;
    }

  /* Run wget */

  ret = perform_wget();
  if (ret < 0)
    {
      return -1;
    }

  /* Set RTC alarm for resuming from cold sleep */

  ret = set_rtc_alarm(LTE_HIBERNATION_INTERVAL);
  if (ret < 0)
    {
      return -1;
    }

  /* LTE module entering hibernation mode */

  printf("Entering LTE hibernation mode.\n");

  ret = lte_enter_hibernation();
  if (ret < 0)
    {
      return -1;
    }

  /* Entering cold sleep */

  boardctl(BOARDIOC_POWEROFF, 1);

  return 0;
}
