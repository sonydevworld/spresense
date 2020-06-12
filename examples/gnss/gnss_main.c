/****************************************************************************
 * gnss/gnss_main.c
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
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <arch/chip/gnss.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GNSS_POLL_FD_NUM          1
#define GNSS_POLL_TIMEOUT_FOREVER -1
#define MY_GNSS_SIG               18

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

static uint32_t                         posfixflag;
static struct cxd56_gnss_positiondata_s posdat;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 *   Read and print POS data.
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
  struct cxd56_gnss_dms_s dmf;

  /* Read POS data. */

  ret = read(fd, &posdat, sizeof(posdat));
  if (ret < 0)
    {
      printf("read error\n");
      goto _err;
    }
  else if (ret != sizeof(posdat))
    {
      ret = ERROR;
      printf("read size error\n");
      goto _err;
    }
  else
    {
      ret = OK;
    }

  /* Print POS data. */

  /* Print time. */

  printf(">Hour:%d, minute:%d, sec:%d, usec:%d\n",
         posdat.receiver.time.hour, posdat.receiver.time.minute,
         posdat.receiver.time.sec, posdat.receiver.time.usec);
  if (posdat.receiver.pos_fixmode != CXD56_GNSS_PVT_POSFIX_INVALID)
    {
      /* 2D fix or 3D fix.
       * Convert latitude and longitude into dmf format and print it. */

      posfixflag = 1;

      double_to_dmf(posdat.receiver.latitude, &dmf);
      printf(">LAT %d.%d.%04d\n", dmf.degree, dmf.minute, dmf.frac);

      double_to_dmf(posdat.receiver.longitude, &dmf);
      printf(">LNG %d.%d.%04d\n", dmf.degree, dmf.minute, dmf.frac);
    }
  else
    {
      /* No measurement. */

      printf(">No Positioning Data\n");
    }

_err:
  return ret;
}

/****************************************************************************
 * Name: gnss_setparams()
 *
 * Description:
 *   Set gnss parameters use ioctl.
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

static int gnss_setparams(int fd)
{
  int      ret = 0;
  uint32_t set_satellite;
  struct cxd56_gnss_ope_mode_param_s set_opemode;

  /* Set the GNSS operation interval. */

  set_opemode.mode     = 1;     /* Operation mode:Normal(default). */
  set_opemode.cycle    = 1000;  /* Position notify cycle(msec step). */

  ret = ioctl(fd, CXD56_GNSS_IOCTL_SET_OPE_MODE, (uint32_t)&set_opemode);
  if (ret < 0)
    {
      printf("ioctl(CXD56_GNSS_IOCTL_SET_OPE_MODE) NG!!\n");
      goto _err;
    }

  /* Set the type of satellite system used by GNSS. */

  set_satellite = CXD56_GNSS_SAT_GPS | CXD56_GNSS_SAT_GLONASS;

  ret = ioctl(fd, CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM, set_satellite);
  if (ret < 0)
    {
      printf("ioctl(CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM) NG!!\n");
      goto _err;
    }

_err:
  return ret;
}

/****************************************************************************
 * Name: gnss_main()
 *
 * Description:
 *   Set parameters and run positioning.
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

int main(int argc, FAR char *argv[])
{
  int      fd;
  int      ret;
  int      posperiod;
  sigset_t mask;
  struct cxd56_gnss_signal_setting_s setting;

  /* Program start. */

  printf("Hello, GNSS(USE_SIGNAL) SAMPLE!!\n");

  /* Get file descriptor to control GNSS. */

  fd = open("/dev/gps", O_RDONLY);
  if (fd < 0)
    {
      printf("open error:%d,%d\n", fd, errno);
      return -ENODEV;
    }

  /* Configure mask to notify GNSS signal. */

  sigemptyset(&mask);
  sigaddset(&mask, MY_GNSS_SIG);
  ret = sigprocmask(SIG_BLOCK, &mask, NULL);
  if (ret != OK)
    {
      printf("sigprocmask failed. %d\n", ret);
      goto _err;
    }

  /* Set the signal to notify GNSS events. */

  setting.fd      = fd;
  setting.enable  = 1;
  setting.gnsssig = CXD56_GNSS_SIG_GNSS;
  setting.signo   = MY_GNSS_SIG;
  setting.data    = NULL;

  ret = ioctl(fd, CXD56_GNSS_IOCTL_SIGNAL_SET, (unsigned long)&setting);
  if (ret < 0)
    {
      printf("signal error\n");
      goto _err;
    }

  /* Set GNSS parameters. */

  ret = gnss_setparams(fd);
  if (ret != OK)
    {
      printf("gnss_setparams failed. %d\n", ret);
      goto _err;
    }

  /* Initial positioning measurement becomes cold start if specified hot
   * start, so working period should be long term to receive ephemeris. */

  posperiod  = 200;
  posfixflag = 0;

  /* Start GNSS. */

  ret = ioctl(fd, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_HOT);
  if (ret < 0)
    {
      printf("start GNSS ERROR %d\n", errno);
      goto _err;
    }
  else
    {
      printf("start GNSS OK\n");
    }

  do
    {
      /* Wait for positioning to be fixed. After fixed,
       * idle for the specified seconds. */

      ret = sigwaitinfo(&mask, NULL);
      if (ret != MY_GNSS_SIG)
        {
          printf("sigwaitinfo error %d\n", ret);
          break;
        }

      /* Read and print POS data. */

      ret = read_and_print(fd);
      if (ret < 0)
        {
          break;
        }

      if (posfixflag)
        {
          /* Count down started after POS fixed. */

          posperiod--;
        }
    }
  while (posperiod > 0);

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

_err:

  /* GNSS firmware needs to disable the signal after positioning. */

  setting.enable = 0;
  ret = ioctl(fd, CXD56_GNSS_IOCTL_SIGNAL_SET, (unsigned long)&setting);
  if (ret < 0)
    {
      printf("signal error\n");
    }

  sigprocmask(SIG_UNBLOCK, &mask, NULL);

  /* Release GNSS file descriptor. */

  ret = close(fd);
  if (ret < 0)
    {
      printf("close error %d\n", errno);
    }

  printf("End of GNSS Sample:%d\n", ret);

  return ret;
}
