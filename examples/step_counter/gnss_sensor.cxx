/****************************************************************************
 * step_counter/gnss_sensor.cxx
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
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>

#include "gnss_sensor.h"
#include "gpsutils/cxd56_gnss_nmea.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define err(format, ...)        fprintf(stderr, format, ##__VA_ARGS__)

#define CHECK_NULL_RET(expr)                                            \
  do {                                                                  \
    if (expr == NULL) {                                                 \
      err("check failed. %s, %d\n", __FUNCTION__, __LINE__);            \
      return -1;                                                        \
    }                                                                   \
  } while(0)

#define GNSS_POLL_FD_NUM          1
#define GNSS_POLL_TIMEOUT_FOREVER -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool gnss_sensing_stop = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int notifyData(FAR GnssSensor *sensor,
               FAR cxd56_gnss_receiver_s *pos,
               FAR nmea_raw_s *rawdat)
{
  struct timeval       tv;
  struct timezone      tz;

  gettimeofday(&tv, &tz);

  sensor->gnss_pos.raw_latitude  = rawdat->lat;
  sensor->gnss_pos.raw_longitude = rawdat->lon;
  sensor->gnss_pos.latitude      = pos->latitude;
  sensor->gnss_pos.longitude     = pos->longitude;
  sensor->gnss_pos.direction     = pos->direction;
  sensor->gnss_pos.velocity      = pos->velocity;
  sensor->gnss_pos.pos_fix_mode  = pos->velocity;
  sensor->gnss_pos.vel_fix_mode  = pos->velocity;
  sensor->gnss_pos.time_stamp    =
      (uint32_t)(((tv.tv_sec * 1000000) + tv.tv_usec) / 1000);

  if ((sensor->handler != NULL) && (sensor->stopped != true))
    {
      sensor->handler(sensor->context, &sensor->gnss_pos);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int GnssSensorCreate(FAR GnssSensor **sensor)
{
  CHECK_NULL_RET(sensor);

  *sensor = (GnssSensor *)malloc(sizeof(GnssSensor));
  memset(*sensor, 0 , sizeof(GnssSensor));

  return 0;
}

/*--------------------------------------------------------------------------*/
int GnssSensorRegisterHandler(FAR GnssSensor *sensor,
                              GnssEventHandler handler,
                              uint32_t context)
{
  sensor->handler = handler;
  sensor->context = context;

  return 0;
}

/*--------------------------------------------------------------------------*/
int GnssSensorStartSensing(FAR GnssSensor *sensor)
{
  static cxd56_gnss_positiondata_s posdat;
  static nmea_raw_s rawdat;
  int gnss_fd;
  int ret;
  struct pollfd fds[GNSS_POLL_FD_NUM] = {{0}};

  /* Open Gnss device */

  gnss_fd = open("/dev/gps", O_RDONLY);
  if (gnss_fd <= 0)
    {
      err("Gnss device open error %d\n", gnss_fd);
      return -1;
    }

  ret = ioctl(gnss_fd, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_HOT);
  if (ret < 0)
    {
      err("Gnss start error %d\n", ret);
      return -1; 
    }

  fds[0].fd     = gnss_fd;
  fds[0].events = POLLIN;

  sensor->stopped = false;
  while (!sensor->stopped) 
    {
      ret = poll(fds, GNSS_POLL_FD_NUM, GNSS_POLL_TIMEOUT_FOREVER);
      if (ret <= 0)
        {
          err("Gnss poll error %d,%x,%x\n", ret, fds[0].events,
              fds[0].revents);
          break;
        }

      ret = read(gnss_fd, &posdat, sizeof(posdat));
      if (ret < 0)
        {
          err("Gnss read error %d\n", ret);
          break;
        }

      ret = NMEA_ExtractRawData(&posdat, &rawdat);
      if (ret < 0)
        {
          err("Gnss extract raw data error %d\n", ret);
          break;
        }

      notifyData(sensor, &(posdat.receiver), &rawdat);
    }

  ret = ioctl(gnss_fd, CXD56_GNSS_IOCTL_STOP, 0);
  if (ret < 0)
    {
      err("Gnss stop error %d\n", ret);
    }

  ret = close(gnss_fd);
  if (ret < 0)
    {
      err("Gnss close error %d\n", ret);
    }

  gnss_sensing_stop = true;

  return ret;
}

/*--------------------------------------------------------------------------*/
int GnssSensorSetProperty(FAR GnssSensor *sensor,
                          uint32_t key,
                          uint32_t param)
{
  return 0;
}

/*--------------------------------------------------------------------------*/
int GnssSensorDestroy(FAR GnssSensor *sensor)
{
  free(sensor);
  return 0;
}

/*--------------------------------------------------------------------------*/
int GnssSensorStopSensing(FAR GnssSensor *sensor)
{
  CHECK_NULL_RET(sensor);
  sensor->stopped = true;

  while(!gnss_sensing_stop)
    {
      sleep(1);
    }

  gnss_sensing_stop = false;

  return 0;
}
