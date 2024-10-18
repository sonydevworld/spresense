/****************************************************************************
 * lte_lwm2m/system_gnss.c
 *
 *   Copyright 2021, 2023 Sony Semiconductor Solutions Corporation
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
#include <math.h>

#include "lwm2mclient.h"
#include "liblwm2m.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MY_GNSS_SIG               18
#define MY_GNSS_SIGKILL           19

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cxd56_gnss_positiondata_s posdat;
static lwm2m_object_t *locationObj;
static pid_t gnss_pid;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gnss_task(int argc, char **argv)
{
  int      fd;
  int      ret;
  sigset_t mask;
  struct cxd56_gnss_signal_setting_s setting;
  uint32_t set_satellite;
  struct cxd56_gnss_ope_mode_param_s set_opemode;

  /* Program start. */

  printf(">%s()\n", __FUNCTION__);

  /* Get file descriptor to control GNSS. */

  fd = open(CONFIG_EXAMPLES_LTE_LWM2M_GPS_DEVNAME, O_RDONLY);
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
      printf("signal set error: %d\n", errno);
      goto _err;
    }

  /* Set the GNSS operation interval. */

  set_opemode.mode  = 1;    /* Operation mode:Normal(default). */
  set_opemode.cycle = 1000; /* Position notify cycle(msec step). */

  ret = ioctl(fd, CXD56_GNSS_IOCTL_SET_OPE_MODE, (unsigned long)&set_opemode);
  if (ret < 0)
    {
      printf("operation mode error: %d\n", errno);
      goto _err;
    }

  /* Set the type of satellite system used by GNSS. */

  set_satellite = CXD56_GNSS_SAT_GPS | CXD56_GNSS_SAT_GLONASS
    | CXD56_GNSS_SAT_QZ_L1CA | CXD56_GNSS_SAT_QZ_L1S;

  ret = ioctl(fd, CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM, set_satellite);
  if (ret < 0)
    {
      printf("select sattelite error: %d\n", errno);
      goto _err;
    }

  /* Start GNSS. */

  ret = ioctl(fd, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_HOT);
  if (ret < 0)
    {
      printf("start error: %d\n", errno);
      goto _err;
    }

  while (1)
    {
      /* Wait until signal comes from gnss */

      ret = sigwaitinfo(&mask, NULL);
      if (ret != MY_GNSS_SIG)
        {
          printf("sigwaitinfo error: %d\n", ret);
          break;
        }

      /* Read POS data. */

      ret = read(fd, &posdat, sizeof(posdat));
      if (ret != sizeof(posdat))
        {
          ret = ERROR;
          printf("read size error: %d\n", errno);
          break;
        }

      if (posdat.receiver.pos_fixmode != CXD56_GNSS_PVT_POSFIX_INVALID)
        {
          float latitude;
          float longitude;
          float altitude;
          float radius;
          float hvar;
          float vvar;
          float speed;
          uint64_t timestamp;
          uint16_t bearing;
          uint16_t horizontalSpeed;
          uint8_t speedUncertainty;
          struct tm tm;

          latitude  = (float)posdat.receiver.latitude;
          longitude = (float)posdat.receiver.longitude;
          altitude  = (float)posdat.receiver.altitude;
          speed     = (float)posdat.receiver.velocity;

          hvar = posdat.receiver.pos_accuracy.hvar;
          vvar = posdat.receiver.pos_accuracy.vvar;

          radius = sqrtf(hvar * hvar + vvar * vvar);

          memset(&tm, 0, sizeof(tm));
          tm.tm_sec  = posdat.receiver.time.sec;
          tm.tm_min  = posdat.receiver.time.minute;
          tm.tm_hour = posdat.receiver.time.hour;
          tm.tm_mday = posdat.receiver.date.day;
          tm.tm_mon  = posdat.receiver.date.month - 1;
          tm.tm_year = posdat.receiver.date.year - 1900;
          timestamp = (uint64_t)mktime(&tm);

          bearing = (uint16_t)posdat.receiver.direction;
          horizontalSpeed = (uint16_t)(posdat.receiver.velocity / 1000);
          speedUncertainty = 255;

          if (locationObj)
            {
              location_setLocationAtTime(locationObj,
                                         latitude,
                                         longitude,
                                         altitude,
                                         radius,
                                         speed,
                                         timestamp);
              location_setVelocity(locationObj,
                                   bearing,
                                   horizontalSpeed,
                                   speedUncertainty);
            }
        }
    }

  /* Stop GNSS. */

  ret = ioctl(fd, CXD56_GNSS_IOCTL_STOP, 0);
  if (ret < 0)
    {
      printf("stop error: %d\n", errno);
    }

_err:

  /* GNSS firmware needs to disable the signal after positioning. */

  setting.enable = 0;
  ret = ioctl(fd, CXD56_GNSS_IOCTL_SIGNAL_SET, (unsigned long)&setting);
  if (ret < 0)
    {
      printf("signal unset error: %d\n", errno);
    }

  sigprocmask(SIG_UNBLOCK, &mask, NULL);

  /* Release GNSS file descriptor. */

  ret = close(fd);
  if (ret < 0)
    {
      printf("close error: %d\n", errno);
    }

  printf("<%s()\n", __FUNCTION__);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int gnss_start(void *arg)
{
  locationObj = (lwm2m_object_t *)arg;
  gnss_pid = task_create("gnss", 100, CONFIG_DEFAULT_TASK_STACKSIZE,
                         gnss_task, NULL);
  return 0;
}

int gnss_stop(void)
{
  int ret;

  ret = kill(gnss_pid, MY_GNSS_SIGKILL);

  /* wait until gnss task is terminated */
  sleep(2);

  return ret;
}
