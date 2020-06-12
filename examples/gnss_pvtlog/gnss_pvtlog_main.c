/****************************************************************************
 * gnss_pvtlog/gnss_pvtlog_main.c
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

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <arch/chip/gnss.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MY_GNSS_SIG0              18
#define MY_GNSS_SIG1              19
#define TEST_LOOP_TIME            600
#define TEST_RECORDING_CYCLE      1
#define TEST_NOTIFY_THRESHOLD     CXD56_GNSS_PVTLOG_THRESHOLD_HALF
#define FILE_NAME_LEN             256

#if(TEST_NOTIFY_THRESHOLD == CXD56_GNSS_PVTLOG_THRESHOLD_HALF)
#  define PVTLOG_UNITNUM          (CXD56_GNSS_PVTLOG_MAXNUM/2)
#else
#  define PVTLOG_UNITNUM          (CXD56_GNSS_PVTLOG_MAXNUM)
#endif
#define TEST_FILE_COUNT         (1 + (int)(TEST_LOOP_TIME / PVTLOG_UNITNUM))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cxd56_gnss_dms_s
{
  int8_t   sign;
  uint8_t  degree;
  uint8_t  minute;
  uint32_t frac;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cxd56_gnss_positiondata_s posdat;
static struct cxd56_pvtlog_s            pvtlogdat;

/****************************************************************************
 * Name: double_to_dmf()
 *
 * Description:
 *   Convert from double format to degree-minute-frac format.
 *
 * Input Parameters:
 *   x   - double value.
 *   dmf - Address to store the conversion result.
 *
 * Returned Value:
 *   none.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void double_to_dmf(double x, struct cxd56_gnss_dms_s * dmf)
{
  int    b;
  int    d;
  int    m;
  double f;
  double t;

  if (x < 0)
    {
      b = 1;
      x = -x;
    }
  else
    {
      b = 0;
    }
  d = (int)x; /* = floor(x), x is always positive */
  t = (x - d) * 60;
  m = (int)t; /* = floor(t), t is always positive */
  f = (t - m) * 10000;

  dmf->sign   = b;
  dmf->degree = d;
  dmf->minute = m;
  dmf->frac   = f;
}

/****************************************************************************
 * Name: read_and_print()
 *
 * Description:
 *   Read and print POS and PVTLOG data.
 *
 * Input Parameters:
 *   fd - File descriptor.
 *
 * Returned Value:
 *   Zero (OK) on success; Negative value on error.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int read_and_print(int fd)
{
  int ret;
  struct cxd56_gnss_dms_s      dmf;
  struct cxd56_pvtlog_status_s pvtlog_status;

  /* Seek data */

  ret = lseek(fd, CXD56_GNSS_READ_OFFSET_LAST_GNSS, SEEK_SET);
  if (ret < 0)
    {
      ret = errno;
      printf("lseek error %d\n", ret);
      goto _err1;
    }

  /* Read POS data */

  ret = read(fd, &posdat, sizeof(posdat));
  if (ret < 0)
    {
      ret = errno;
      printf("read error %d\n", ret);
      goto _err1;
    }
  else if (ret < sizeof(posdat))
    {
      ret = ERROR;
      printf("read size error %d\n", ret);
      goto _err1;
    }
  else
    {
      ret = OK;
    }

  if (posdat.receiver.pos_dataexist &&
      (posdat.receiver.pos_fixmode != CXD56_GNSS_PVT_POSFIX_INVALID))
    {
      /* Print POS data */

      printf(" Y=%d, M=%2d, d=%2d", posdat.receiver.date.year,
             posdat.receiver.date.month, posdat.receiver.date.day);

      printf(" h=%2d, m=%2d, s=%2d m=%3d", posdat.receiver.time.hour,
             posdat.receiver.time.minute, posdat.receiver.time.sec,
             (int)(posdat.receiver.time.usec/1000));

      double_to_dmf(posdat.receiver.latitude, &dmf);
      printf(", Lat %d:%d:%d",
             dmf.degree, dmf.minute, dmf.frac);

      double_to_dmf(posdat.receiver.longitude, &dmf);
      printf(" , Lon %d:%d:%d",
             dmf.degree, dmf.minute, dmf.frac);

      /* Get Log status */

      ret = ioctl(fd, CXD56_GNSS_IOCTL_PVTLOG_GET_STATUS,
                  (unsigned long)&pvtlog_status);
      printf(", Log No:%d \n", pvtlog_status.status.log_count);
    }
  else
    {
      printf("No Positioning...\n");
    }

_err1:
  return ret;
}

/****************************************************************************
 * Name: get_pvtlog()
 *
 * Description:
 *   Read PVTLOG data.
 *
 * Input Parameters:
 *   fd - File descriptor.
 *
 * Returned Value:
 *   Zero (OK) on success; Negative value on error.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int get_pvtlog(int fd)
{
  int ret;

  /* Seek. */

  ret = lseek(fd, CXD56_GNSS_READ_OFFSET_PVTLOG, SEEK_SET);
  if (ret < 0)
    {
      ret = errno;
      printf("lseek error %d\n", ret);
      goto _err1;
    }

  /* Read PVTLOG. */

  ret = read(fd, &pvtlogdat, sizeof(pvtlogdat));
  if (ret < 0)
    {
      ret = errno;
      printf("read error %d\n", ret);
      goto _err1;
    }
  else if (ret == 0)
    {
      printf("read data size is 0 \n");
      goto _err1;
    }
  else
    {
      ret = OK;
    }

_err1:
  return ret;
}

/****************************************************************************
 * Name: set_signal()
 *
 * Description:
 *   Call CXD56_GNSS_IOCTL_SIGNAL_SET.
 *
 * Input Parameters:
 *   fd      - File descriptor.
 *   signo   - signal number
 *   gnsssig - GNSS ID
 *   enable  - set enable or disable
 *   mask    - signal mask
 *
 * Returned Value:
 *   Zero (OK) on success; Negative value on error.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int set_signal(int fd, int signo, uint8_t gnsssig, int enable,
                      sigset_t * mask)
{
  int ret;
  struct cxd56_gnss_signal_setting_s setting;

  sigaddset(mask, signo);
  ret = sigprocmask(SIG_BLOCK, mask, NULL);
  if (ret != OK)
    {
      printf("sigprocmask failed. %d\n", ret);
      goto _err1;
    }

  setting.fd      = fd;
  setting.enable  = enable;
  setting.gnsssig = gnsssig;
  setting.signo   = signo;
  setting.data    = NULL;

  ret = ioctl(fd, CXD56_GNSS_IOCTL_SIGNAL_SET, (unsigned long)&setting);

_err1:
  return ret;
}

/****************************************************************************
 * Name: writefile()
 *
 * Description:
 *   Write PVTLOG data.
 *
 * Input Parameters:
 *   file_count - Write file count.
 *
 * Returned Value:
 *   Zero (OK) on success; Negative value on error.
 *
 * Assumptions/Limitations:
 *   Write buffer refers to global variable "pvtlogdat".
 *
 ****************************************************************************/

static int writefile(uint32_t file_count)
{
  int fd_write;
  int ret = OK;
  char filename[FILE_NAME_LEN];

  /* Make file name */

  snprintf(filename, FILE_NAME_LEN, "%s%d.dat", 
           CONFIG_EXAMPLES_GNSS_PVTLOG_FILEPATH, file_count);

  /* Open file */

  fd_write = open(filename, O_WRONLY | O_CREAT | O_BINARY);
  if (fd_write < 0)
    {
      printf("%s open error:%d\n", filename, errno);
      ret = ERROR;
    }
  else
    {
      if (write(fd_write, &pvtlogdat, sizeof(struct cxd56_pvtlog_s)) !=
          sizeof(struct cxd56_pvtlog_s))
        {
          printf("%s write error:%d\n", filename, errno);
          ret = ERROR;
        }
      else
        {
          printf("%s write OK\n", filename);
        }

      close(fd_write);
    }
  fd_write = 0;

  return ret;
}

/****************************************************************************
 * Name: gnss_pvtlog_write()
 *
 * Description:
 *   Enable PVTLOG and run GNSS for 10 minutes.
 *   When notification of PVTLOG comes, save PVTLOG in a file.
 *
 * Input Parameters:
 *   argc - Does not use.
 *   argv - Does not use.
 *
 * Returned Value:
 *   Zero (OK) on success; Negative value on error.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

int gnss_pvtlog_write(int argc, char *argv[])
{
  int      fd;
  int      ret;
  int      timecount = 0;
  int      sig_id    = -1;
  uint32_t file_count = 1;
  sigset_t mask;
  struct cxd56_pvtlog_setting_s pvtlog_setting;

  /* Program start */

  printf("%s() in\n", __func__);

  /* Get file descriptor to control GNSS. */

  fd = open("/dev/gps", O_RDONLY);
  if (fd < 0)
    {
      printf("open error:%d,%d\n", fd, errno);
      return -ENODEV;
    }

  sigemptyset(&mask);

  /* Init positioning signal */
  printf("Set GNSS signal \n");
  ret = set_signal(fd, MY_GNSS_SIG0, CXD56_GNSS_SIG_GNSS, TRUE, &mask);
  if (ret < 0)
    {
      printf("GNSS signal set error\n");
      goto _err3;
    }

  /* Init PVTLOG signal */
  printf("Set PVTLOG signal \n");
  ret = set_signal(fd, MY_GNSS_SIG1, CXD56_GNSS_SIG_PVTLOG, TRUE, &mask);
  if (ret < 0)
    {
      printf("PVTLOG signal set error\n");
      goto _err2;
    }

  /* Delete Log data */
  printf("Delete Log \n");
  ret = ioctl(fd, CXD56_GNSS_IOCTL_PVTLOG_DELETE_LOG, 0);
  if (ret < 0)
    {
      printf("Delete log error\n");
      goto _err1;
    }

  /* Start Log */
  printf("Start Log \n");
  pvtlog_setting.cycle     = TEST_RECORDING_CYCLE;
  pvtlog_setting.threshold = TEST_NOTIFY_THRESHOLD;
  ret = ioctl(fd, CXD56_GNSS_IOCTL_PVTLOG_START,
              (unsigned long)&pvtlog_setting);
  if (ret < 0)
    {
      printf("Start pvtlog error\n");
      goto _err1;
    }

  /* Start GNSS. */

  printf("Start Positioning \n");
  ret = ioctl(fd, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_HOT);
  if (ret < 0)
    {
      printf("start GNSS ERROR %d\n", errno);
      goto _err0;
    }
  else
    {
      printf("start GNSS OK\n");
    }

  do
    {
      /* Wait signal */

      sig_id = sigwaitinfo(&mask, NULL);

      switch (sig_id)
        {
        case MY_GNSS_SIG0:
          /* Read and print POS data. */

          read_and_print(fd);
          timecount++;
          break;

        case MY_GNSS_SIG1:
          /* Receive pvtlog signal */

          get_pvtlog(fd);
          writefile(file_count);
          file_count++;
          break;

        default:
          /* Invalid case */
          printf("ret %d\n", ret);
          break;
        }
    }
  while (timecount < TEST_LOOP_TIME);

  /* Stop Log */

  printf("Stop Log \n");
  ioctl(fd, CXD56_GNSS_IOCTL_PVTLOG_STOP, 0);


  /* If the last signal is positioning, there is a unsaved PVTLOG. */

  if (sig_id == MY_GNSS_SIG0)
    {
      /* Write unsaved logs */

      get_pvtlog(fd);
      writefile(file_count);
    }

_err0:
  /* Stop GNSS. */

  ret = ioctl(fd, CXD56_GNSS_IOCTL_STOP, 0);
  if (ret < 0)
    {
      printf("stop GNSS ERROR\n");
    }
  else
    {
      printf("stop GNSS OK\n");
    }

_err1:
  /* TBD */

  set_signal(fd, MY_GNSS_SIG0, CXD56_GNSS_SIG_GNSS, FALSE, &mask);
_err2:
  /* TBD */

  set_signal(fd, MY_GNSS_SIG1, CXD56_GNSS_SIG_PVTLOG, FALSE, &mask);
_err3:
  /* Release GNSS file descriptor. */

  ret = close(fd);
  if (ret < 0)
    {
      printf("close error\n");
    }

  printf("%s() out %d\n", __func__, ret);

  return ret;
}

/****************************************************************************
 * Name: gnss_pvtlog_read()
 *
 * Description:
 *   Read and dump pvtlog file.
 *   Make sure that the dump logs of gnss_pvtlog_write() and 
 *   gnss_pvtlog_read() match.
 *
 * Input Parameters:
 *   argc - Does not use.
 *   argv - Does not use.
 *
 * Returned Value:
 *   Zero (OK) on success; Negative value on error.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

int gnss_pvtlog_read(int argc, char *argv[])
{
  int      ret = OK;
  int      fd_read;
  uint32_t file_count;
  uint32_t log_count;
  uint32_t log_max;
  char     filename[FILE_NAME_LEN];
  struct cxd56_pvtlog_data_s *log;

  /* Program start */

  printf("%s() in\n", __func__);

  for (file_count = 1; file_count <= TEST_FILE_COUNT; file_count++)
    {
      /* Make file name */

      snprintf(filename, FILE_NAME_LEN, "%s%d.dat",
               CONFIG_EXAMPLES_GNSS_PVTLOG_FILEPATH, file_count);

      /* Open file */

      fd_read = open(filename, O_RDONLY | O_BINARY);
      if (fd_read < 0)
        {
          /* Continue */
        }
      else
        {
          /* Read file */

          ret = read(fd_read, &pvtlogdat, sizeof(pvtlogdat));
          if (ret < 0)
            {
              ret = errno;
              printf("%s read error:%d\n", filename, errno);
            }
          else if (ret != sizeof(pvtlogdat))
            {
              ret = ERROR;
              printf("%s read error:%d\n", filename, errno);
            }
          else
            {
              ret = OK;

              log_max = pvtlogdat.log_count;
              printf("%s read OK(%d line)\n", filename, log_max);

              /* Printf pvtlog file */

              for (log_count = 0; log_count < log_max; log_count++)
                {
                  /* Printf record */

                  log = &pvtlogdat.log_data[log_count];
                  printf(" Y=20%2d, M=%2d, d=%2d",
                         log->date.year, log->date.month, log->date.day);

                  printf(" h=%2d, m=%2d, s=%2d m=%3d", log->time.hour,
                         log->time.minute, log->time.sec, log->time.msec);

                  printf(", Lat %d:%d:%d",log->latitude.degree,
                         log->latitude.minute, log->latitude.frac);

                  printf(" , Lon %d:%d:%d", log->longitude.degree,
                         log->longitude.minute, log->longitude.frac);

                  printf(", Log No:%d \n", (log_count + 1));
                }
            }
        }

      /* Close file */

      close(fd_read);
      fd_read = 0;
    }

  printf("%s() out %d\n", __func__, ret);

  return ret;
}

/****************************************************************************
 * Name: gnss_pvtlog_delete()
 *
 * Description:
 *   Delete pvtlog files.
 *
 * Input Parameters:
 *   argc - Does not use.
 *   argv - Does not use.
 *
 * Returned Value:
 *   Zero (OK) on success; Negative value on error.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

int gnss_pvtlog_delete(int argc, char *argv[])
{
  int ret = OK;
  uint32_t file_count;
  char filename[FILE_NAME_LEN];

  /* Program start */

  printf("%s() in\n", __func__);

  for (file_count = 1; file_count <= TEST_FILE_COUNT; file_count++)
    {
      /* Make file name */

      snprintf(filename, FILE_NAME_LEN, "%s%d.dat",
               CONFIG_EXAMPLES_GNSS_PVTLOG_FILEPATH, file_count);

      /* Delete file */

      if (unlink(filename) == OK)
        {
          printf("%s delete ok\n", filename);
        }
    }


  printf("%s() out %d\n", __func__, ret);

  return ret;
}

/****************************************************************************
 * Name: gnss_pvtlog_main()
 *
 * Description:
 *   Run positioning and write / read PVTLOG file.
 *
 * Input Parameters:
 *   argv[1] - Specify the operation mode of PVTLOG.
 *               "W": positioning and write pvtlog file.
 *                    Including the case of not specifying.
 *               "R": read and dump pvtlog file.
 *               "D": delete pvtlog file.
 *               "A": run W and R and D.
 *
 * Returned Value:
 *   Zero (OK) on success; Negative value on error.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret = OK;
  char *argmode = "w";  /* Default:write */

  /* Program start */

  printf("Hello, PVTLOG SAMPLE!!\n");

  /* Argument check */

  if (argc >= 2)
    {
      argmode = argv[1];
    }

  switch (argmode[0])
    {
    case 'r':
    case 'R':
      /* Read and dump pvtlog file */

      ret = gnss_pvtlog_read(argc, argv);
      break;

    case 'd':
    case 'D':
      /* Delete pvtlog file */

      ret = gnss_pvtlog_delete(argc, argv);
      break;

    case 'a':
    case 'A':
      /* Run W and R and D */

      if (gnss_pvtlog_write(argc, argv) != OK)
        {
          ret = ERROR;
        }
      if (gnss_pvtlog_read(argc, argv) != OK)
        {
          ret = ERROR;
        }
      if (gnss_pvtlog_delete(argc, argv) != OK)
        {
          ret = ERROR;
        }
      break;

    case 'w':
    case 'W':
    default:
      /* Positioning and write pvtlog file */

      ret = gnss_pvtlog_write(argc, argv);
      break;
    }

  printf("End of PVTLOG Sample:%d\n", ret);

  return ret;
}
