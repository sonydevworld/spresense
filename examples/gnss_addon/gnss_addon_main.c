/****************************************************************************
 * examples/gnss_addon/gnss_addon_main.c
 *
 *   Copyright 2023, 2024 Sony Semiconductor Solutions Corporation
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
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <time.h>
#include <arch/chip/gnss.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>
#include <arch/chip/pm.h>
#include "gnss_addon_nmea.h"
#include "gnss_addon_pps.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If the time difference is greater than this threshold, adjust the time. */

#define THRESHOLD_TIME_DIFF_MSEC 150

/* Toggle LED for debug */

#define LED   GPIO_LED4

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct args_s
{
  bool nmea;
  bool qzqsm;
  bool pps;
  int  cycle;
  int  fixcnt;
  int  sleep;
  char *filepath;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cxd56_gnss_positiondata2_s posdat;
static struct cxd56_gnss_dcreport_data_s dcreport;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(const char *progname)
{
  printf("Usage: %s [-n] [-q] [-p] [-c <cycle>] [-f <fixcnt>] [-s <sleep>]"
         " [-o <filepath>]\n"
         "Options:\n"
         "  -n: Enable NMEA output\n"
         "  -q: Enable NMEA DC Report output with \"-n\" option\n"
         "  -p: Enable 1PPS signal\n"
         "  -c <cycle>: Positioning cycle\n"
         "     (100, 125, 200, 250, 500 or 1000 x N) [msec] (default:1000)\n"
         "  -f <fixcnt>: Positioning fix count (default:300)\n"
         "  -s <sleep>: Sleeping time [sec] (default:0)\n"
         "  -o <filepath>: Full path to a log file\n"
         "\n", progname);
}

static int parse_args(struct args_s *args, int argc, char *argv[])
{
  int opt;

  /* default settings */

  args->nmea     = false;
  args->qzqsm    = false;
  args->pps      = false;
  args->cycle    = 1000;
  args->fixcnt   = 300;
  args->sleep    = 0;
  args->filepath = NULL;

  while ((opt = getopt(argc, argv, "nqpc:f:s:o:")) != ERROR)
    {
      switch (opt)
        {
          case 'n':
            args->nmea = true;
            break;
          case 'q':
            args->qzqsm = true;
            break;
          case 'p':
            args->pps = true;
            break;
          case 'c':
            args->cycle = atoi(optarg);
            break;
          case 'f':
            args->fixcnt = atoi(optarg);
            break;
          case 's':
            args->sleep = atoi(optarg);
            break;
          case 'o':
            args->filepath = optarg;
            break;
          case '?':
          case ':':
          default:
            show_usage(argv[0]);
            return ERROR;
        }
    }

  /* Check arguments */

  if ((args->cycle <= 0) ||
      ((args->cycle != 100) && (args->cycle != 125) &&
       (args->cycle != 200) && (args->cycle != 250) &&
       (args->cycle != 500) && (args->cycle % 1000)))
    {
      printf("ERROR: Invalid cycle '%d'\n", args->cycle);
      show_usage(argv[0]);
      return -EINVAL;
    }

  if (args->qzqsm && !args->nmea)
    {
      printf("ERROR: Also specify the \"-n\" option to use \"-q\"\n");
      show_usage(argv[0]);
      return -EINVAL;
    }

  if (strstr(args->filepath, "sd0"))
    {
      struct stat buf;
      int retry;
      int ret;

      /* Wait until sdcard is mounted. */

      for (retry = 30; retry > 0; retry--)
        {
          ret = stat("/mnt/sd0", &buf);
          if (ret == 0)
            {
              break;
            }

          usleep(100 * 1000);
        }

      if (retry == 0)
        {
          printf("ERROR: Invalid path '%s'\n", args->filepath);
          show_usage(argv[0]);
          return -EINVAL;
        }
    }

  return OK;
}

static void get_datetime(struct cxd56_gnss_datetime_s *dt)
{
  time_t tt;
  struct tm tm;
  struct timespec ts;

  /* Get the current date time from RTC. */

  clock_gettime(CLOCK_REALTIME, &ts);

  tt = ts.tv_sec;
  gmtime_r(&tt, &tm);

  dt->date.year   = tm.tm_year + 1900;
  dt->date.month  = tm.tm_mon + 1;
  dt->date.day    = tm.tm_mday;
  dt->time.hour   = tm.tm_hour;
  dt->time.minute = tm.tm_min;
  dt->time.sec    = tm.tm_sec;
  dt->time.usec   = ts.tv_nsec / NSEC_PER_USEC;
}

static int64_t diff_msec(struct timespec *t1, struct timespec *t2)
{
  int64_t msec;

  if (t1->tv_sec == t2->tv_sec)
    {
      if (t1->tv_nsec >= t2->tv_nsec)
        {
          msec = (t1->tv_nsec - t2->tv_nsec) / NSEC_PER_MSEC;
        }
      else
        {
          msec = (t2->tv_nsec - t1->tv_nsec) / NSEC_PER_MSEC;
        }
    }
  else if (t1->tv_sec >= t2->tv_sec)
    {
      msec = (t1->tv_sec - t2->tv_sec) * MSEC_PER_SEC;
      msec += (t1->tv_nsec - t2->tv_nsec) / NSEC_PER_MSEC;
    }
  else
    {
      msec = (t2->tv_sec - t1->tv_sec) * MSEC_PER_SEC;
      msec += (t2->tv_nsec - t1->tv_nsec) / NSEC_PER_MSEC;
    }

  return msec;
}

static void set_datetime(struct cxd56_gnss_date_s *date,
                         struct cxd56_gnss_time_s *time,
                         bool use_pps)
{
  time_t tt;
  struct tm tm;
  struct timespec ts;

  if (time->usec == 0)
    {
      memset(&tm, 0, sizeof(tm));
      tm.tm_year = date->year - 1900;
      tm.tm_mon  = date->month - 1;
      tm.tm_mday = date->day;
      tm.tm_hour = time->hour;
      tm.tm_min  = time->minute;
      tm.tm_sec  = time->sec;

      tt = mktime(&tm);

      if (use_pps)
        {
          /* Set more accurate time to the RTC using the PPS signal.
           * Here hold the time after 1 second set by the PPS interrupt.
           */

          start_ppsint();
          ts.tv_sec = tt + 1;
          ts.tv_nsec = 0;
          set_ppstime(ts);
        }
      else
        {
          struct timespec now;

          /* Simply set the positioning time to the RTC. It has a time lag
           * of a few hundred milliseconds from atcual time.
           */

          clock_gettime(CLOCK_REALTIME, &now);
          ts.tv_sec = tt;
          ts.tv_nsec = 0;
          if (diff_msec(&now, &ts) > THRESHOLD_TIME_DIFF_MSEC)
            {
              clock_settime(CLOCK_REALTIME, &ts);
            }
        }
    }
}

static void print_posdat(struct cxd56_gnss_positiondata2_s *pos, FILE *stream)
{
  struct cxd56_gnss_receiver2_s *rcv = &pos->receiver;

  fprintf(stream,
          "%04d/%02d/%02d %02d:%02d:%02d.%03ld %.6lf %.6lf %.3lf [%c]\n",
          rcv->date.year, rcv->date.month, rcv->date.day,
          rcv->time.hour, rcv->time.minute, rcv->time.sec,
          rcv->time.usec / 1000,
          rcv->latitude, rcv->longitude, rcv->altitude,
          (rcv->fix_indicator == 0) ? 'N' :       /* Not valid */
          (rcv->fix_indicator == 1) ? 'A' :       /* Autonomous mode */
          (rcv->fix_indicator == 2) ? 'D' :       /* Differential mode */
          (rcv->fix_indicator == 6) ? 'E' : 'A'); /* Estimated mode */
}

static void toggle_led(void)
{
  static int toggle = 0;
  board_gpio_write(LED, toggle = !toggle);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[])
{
  int ret;
  struct args_s args;
  int fd;
  uint32_t bootcause;
  int fixcnt = 0;
  FILE *fp = stdout;
  char version[CXD56_GNSS_VERSION_MAXLEN];
  uint32_t vernum = 0;

  /* Argument settings */

  memset(&args, 0, sizeof(args));

  ret = parse_args(&args, argc, argv);
  if (ret < 0)
    {
      return ret;
    }

  /* Get the boot cause and executes different processes.
   * Specifically, if the system is started by RTC from DeepSleep state,
   * it injects the RTC time to GNSS and run GNSS hot start.
   */

  bootcause = up_pm_get_bootcause();

  if ((bootcause == PM_BOOT_POR_NORMAL) ||
      (bootcause == PM_BOOT_POR_DEADBATT) ||
      (bootcause == PM_BOOT_WDT_REBOOT))
    {
      printf("GNSS Add-on example application:\n");
      printf("NMEA: %s (DC Report: %s), 1PPS: %s\n",
             args.nmea ? "Enable" : "Disable",
             args.qzqsm ? "Enable" : "Disable",
             args.pps ? "Enable" : "Disable");
      printf("After positioning fix %d times, sleep %d sec.\n",
             args.fixcnt, args.sleep);
    }

  /* Open a log file with append mode by option. */

  if (args.filepath)
    {
      fp = fopen(args.filepath, "a");
      if (fp == NULL)
        {
          /* If the file fails to open, switch to standard output. */

          fp = stdout;
          args.filepath = NULL;
        }
    }

  /* Support NMEA output by option. */

  if (args.nmea)
    {
      setup_nmea(fp);
    }

  /* Open a GNSS Add-on device driver. */

  fd = open("/dev/gps2", O_RDONLY);
  if (fd < 0)
    {
      printf("ERROR: open ret=%d, errno=%d\n", ret, errno);
      return -ENODEV;
    }

  /* Wakeup as GNSS may be in sleep mode. */

  ret = ioctl(fd, CXD56_GNSS_IOCTL_WAKEUP, 0);
  if (ret < 0)
    {
      printf("ERROR: wakeup ret=%d, errno=%d\n", ret, errno);
    }

  /* Display the firmware version. */

  memset(version, 0, sizeof(version));
  ret = ioctl(fd, CXD56_GNSS_IOCTL_GET_VERSION, (unsigned long)&version);
  if (ret < 0)
    {
      printf("ERROR: get version ret=%d, errno=%d\n", ret, errno);
    }
  else
    {
      printf("FW version: %s\n", version);
      vernum = atof(version + 1) * 1000;
    }

  /* Check if supported by the firmware version. */

  if (args.qzqsm && (vernum < 144))
    {
      printf("ERROR: QZQSM is supported in v00.144 or later.\n");
      return -ENOTSUP;
    }

  /* Adjust the system time using 1PPS signal.
   * You need to connect the 1PPS signal to any interrupt pin.
   */

  if (args.pps)
    {
      setup_pps();

      ret = ioctl(fd, CXD56_GNSS_IOCTL_SET_1PPS_OUTPUT, 1);
      if (ret < 0)
        {
          printf("ERROR: 1pps ret=%d, errno=%d\n", ret, errno);
        }
    }

  /* If the system is started by RTC from DeepSleep state,
   * set the RTC time to GNSS since the GNSS does not keep time.
   */

  if (bootcause == PM_BOOT_DEEP_RTC)
    {
      struct cxd56_gnss_datetime_s datetime;

      get_datetime(&datetime);

      ret = ioctl(fd, CXD56_GNSS_IOCTL_SET_TIME, &datetime);
      if (ret < 0)
        {
          printf("ERROR: settime ret=%d, errno=%d\n", ret, errno);
        }
    }

  /* Set the positioning cycle. */

  if (args.cycle != 1000)
    {
      struct cxd56_gnss_ope_mode_param_s opemode;

      opemode.mode = 1;
      opemode.cycle = args.cycle;

      ret = ioctl(fd, CXD56_GNSS_IOCTL_SET_OPE_MODE, (uint32_t)&opemode);
      if (ret < 0)
        {
          printf("ERROR: cycle ret=%d, errno=%d\n", ret, errno);
          goto errout;
        }
    }

  /* Start GNSS Add-on. */

  ret = ioctl(fd, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_HOT);
  if (ret < 0)
    {
      printf("ERROR: start ret=%d, errno=%d\n", ret, errno);
      goto errout;
    }

  while (1)
    {
      struct pollfd fds[1];

      fds[0].fd     = fd;
      fds[0].events = POLLIN;

      /* Wait for positioning data to be notified. */

      ret = poll(fds, 1, -1);
      if (ret < 0)
        {
          printf("ERROR: poll ret=%d, errno=%d\n", ret, errno);
          break;
        }

      /* Read the positioning data. */

      ret = read(fd, &posdat, sizeof(posdat));
      if (ret != sizeof(posdat))
        {
          printf("ERROR: read ret=%d, errno=%d\n", ret, errno);
          break;
        }

      toggle_led();

      /* If the position is fixed, set the date time to RTC and increment
       * the fix counter. If not, clear the counter.
       */

      if (posdat.receiver.fix_indicator > 0)
        {
          set_datetime(&posdat.receiver.date, &posdat.receiver.time,
                       args.pps);
          fixcnt++;
        }
      else
        {
          fixcnt = 0;
        }

      /* Print the positioning data. */

      if (args.nmea)
        {
          print_nmea(&posdat);

          if (args.qzqsm)
            {
              /* Read the DC Report. */

              ret = lseek(fd, CXD56_GNSS_READ_OFFSET_DCREPORT, SEEK_SET);
              if (ret < 0)
                {
                  printf("ERROR: lseek ret=%d, errno=%d\n", ret, errno);
                  break;
                }

              ret = read(fd, &dcreport, sizeof(dcreport));
              if (ret != sizeof(dcreport))
                {
                  printf("ERROR: read ret=%d, errno=%d\n", ret, errno);
                  break;
                }

              print_dcreport(&dcreport);

              /* Return the read position to the top. */

              ret = lseek(fd, CXD56_GNSS_READ_OFFSET_LAST_GNSS, SEEK_SET);
              if (ret < 0)
                {
                  printf("ERROR: lseek ret=%d, errno=%d\n", ret, errno);
                  break;
                }
            }
        }
      else
        {
          print_posdat(&posdat, fp);
        }

      /* Check if the fix count reaches the specified number. */

      if (args.fixcnt <= fixcnt)
        {
          if (args.pps)
            {
              stop_ppsint();
            }

          break;
        }
    }

  /* Stop GNSS, save the backup data and put it deep sleep mode
   * for power saving.
   */

  ret = ioctl(fd, CXD56_GNSS_IOCTL_STOP, 0);
  if (ret < 0)
    {
      printf("ERROR: stop ret=%d, errno=%d\n", ret, errno);
    }

  ret = ioctl(fd, CXD56_GNSS_IOCTL_SAVE_BACKUP_DATA, 0);
  if (ret < 0)
    {
      printf("ERROR: save ret=%d, errno=%d\n", ret, errno);
    }

  ret = ioctl(fd, CXD56_GNSS_IOCTL_SLEEP, CXD56_GNSS_DEEPSLEEP);
  if (ret < 0)
    {
      printf("ERROR: sleep ret=%d, errno=%d\n", ret, errno);
    }

  /* Close a log file. */

  if (args.filepath)
    {
      fclose(fp);
    }

  /* The system goes into deep sleep after setting RTC alarm. */

  if (args.sleep > 0)
    {
      char command[32];

      printf("RTC alarm after %d sec\n", args.sleep);
      snprintf(command, sizeof(command), "alarm %d", args.sleep);
      system(command);

      boardctl(BOARDIOC_POWEROFF, 0);

      /* should never come here */
    }

errout:

  /* Close a GNSS Add-on device driver. */

  ret = close(fd);
  if (ret < 0)
    {
      printf("ERROR: close ret=%d, errno=%d\n", ret, errno);
    }

  return ret;
}
