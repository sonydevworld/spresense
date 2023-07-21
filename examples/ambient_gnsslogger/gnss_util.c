/****************************************************************************
 * examples/ambient_gnsslogger/gnss_util.c
 *
 *   Copyright 2022, 2023 Sony Semiconductor Solutions Corporation
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
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>

#include <arch/chip/gnss.h>

#include "gnss_util.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MY_GNSS_SIG               18

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cxd56_gnss_positiondata_s posdat;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: init_gnss()
 *
 * Description: Initialize GNSS driver.
 *
 * Parameters:
 *   mask [out]:  pointer of sigset_t to be set on this function.
 *
 * Returned Value:
 *   Negative value is returned when an error is happened.
 *   When the initialization is successed, file descripter is returned.
 *
 ****************************************************************************/

int init_gnss(sigset_t *mask)
{
  int fd;
  int er;
  uint32_t set_satellite;
  struct cxd56_gnss_signal_setting_s setting;
  struct cxd56_gnss_ope_mode_param_s set_opemode;

  fd = open(CONFIG_EXAMPLES_AMBIENT_GNSSLOGGER_GPS_DEVNAME, O_RDONLY);
  if (fd < 0)
    {
      printf("open error:%d,%d\n", fd, errno);
      return -1;
    }

  sigemptyset(mask);
  sigaddset(mask, MY_GNSS_SIG);
  er = sigprocmask(SIG_BLOCK, mask, NULL);
  if (er != OK)
    {
      printf("sigprocmask failed:%d\n", er);
      close(fd);
      return -2;
    }

  setting.fd      = fd;
  setting.enable  = 1;
  setting.gnsssig = CXD56_GNSS_SIG_GNSS;
  setting.signo   = MY_GNSS_SIG;
  setting.data    = NULL;
  er = ioctl(fd, CXD56_GNSS_IOCTL_SIGNAL_SET, (unsigned long)&setting);
  if (er < 0)
    {
      printf("ioctl(SIGNAL_SET) error:%d\n", er);
      sigprocmask(SIG_UNBLOCK, mask, NULL);
      close(fd);
      return -3;
    }

  set_opemode.mode     = 1;     /* Operation mode:Normal(default). */
  set_opemode.cycle    = 1000;  /* Position notify cycle(msec step). */
  er = ioctl(fd, CXD56_GNSS_IOCTL_SET_OPE_MODE, (uint32_t)&set_opemode);
  if (er < 0)
    {
      printf("ioctl(SET_OPE_MODE) error:%d\n", er);
      sigprocmask(SIG_UNBLOCK, mask, NULL);
      close(fd);
      return -4;
    }

  set_satellite = CXD56_GNSS_SAT_GPS
                | CXD56_GNSS_SAT_QZ_L1CA
                | CXD56_GNSS_SAT_QZ_L1S;
  er = ioctl(fd, CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM, set_satellite);
  if (er < 0)
    {
      printf("ioctl(SELECT_SATELLITE_SYSTEM) error:%d\n", er);
      sigprocmask(SIG_UNBLOCK, mask, NULL);
      close(fd);
      return -5;
    }

  return fd;
}

/****************************************************************************
 * Name: start_gnss()
 *
 * Description: start to measure location.
 *
 * Parameters:
 *   fd   [in] :  File descriptor of /dev/gps.
 *
 * Returned Value:
 *   Negative value is returned when an error is happened.
 *
 ****************************************************************************/

int start_gnss(int fd)
{
  return ioctl(fd, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_HOT);
}

/****************************************************************************
 * Name: get_position()
 *
 * Description: get GNSS measured position.
 *
 * Parameters:
 *   fd   [in] :  File descriptor of /dev/gps.
 *   mask [in] :  pointer of sigset_t which was set by init_gnss().
 *   scnt [out]:  Number of captured satellites.
 *   dt   [out]:  Date and time from GNSS measurement.
 *   lat  [out]:  measured latitude. This value is valid when return value is positive.
 *   lng  [out]:  measured longitude. This value is valid when return value is positive.
 *
 * Returned Value:
 *   Negative value is returned when an error is happened.
 *   Zero is returned when some satellites are captured but not fix the position.
 *   Positive value is returned when the position is fixed.
 *
 ****************************************************************************/

int get_position(int fd, sigset_t *mask, int *scnt, struct datetime_s *dt,
    float *lat, float *lng)
{
  int ret;
  struct timespec tout;

  tout.tv_sec = 2;
  tout.tv_nsec = 0;
  ret = sigtimedwait(mask, NULL, &tout);
  if (ret != MY_GNSS_SIG)
    {
      /* Timed out */

      return GNSS_UTIL_STATE_TOUT;
    }

  ret = read(fd, &posdat, sizeof(posdat));
  if ((ret < 0) || (ret != sizeof(posdat)))
    {
      return GNSS_UTIL_STATE_ERROR;
    }

  *scnt = (int)posdat.svcount;

  dt->date = posdat.receiver.date;
  dt->time = posdat.receiver.time;

  if (posdat.receiver.pos_fixmode != CXD56_GNSS_PVT_POSFIX_INVALID)
    {
      *lat = (float)posdat.receiver.latitude;
      *lng = (float)posdat.receiver.longitude;

      return GNSS_UTIL_STATE_FIXED;
    }

  return GNSS_UTIL_STATE_SVCAP;
}

/****************************************************************************
 * Name: fin_gnss()
 *
 * Description: finalize gnss driver.
 *
 * Parameters:
 *   fd   [in] : File descriptor of /dev/gps.
 *   mask [in] : pointer of sigset_t which was set by init_gnss().
 *
 ****************************************************************************/

void fin_gnss(int fd, sigset_t *mask)
{
  struct cxd56_gnss_signal_setting_s setting;

  /* Stop GNSS. */

  ioctl(fd, CXD56_GNSS_IOCTL_STOP, 0);

  memset(&setting, 0, sizeof(setting));

  setting.enable = 0;
  ioctl(fd, CXD56_GNSS_IOCTL_SIGNAL_SET, (unsigned long)&setting);

  sigprocmask(SIG_UNBLOCK, mask, NULL);
  close(fd);
}
