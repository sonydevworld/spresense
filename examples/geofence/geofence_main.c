/****************************************************************************
 * geofence/geofence_main.c
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
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <arch/chip/gnss.h>
#include <arch/chip/geofence.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define POLL_FD_NUM             1
#define POLL_TIMEOUT_FOREVER   -1
#define GEOFENE_REGION_RADIUS   50
#define GEOFENE_REGION_OFFSET   (10 * GEOFENE_REGION_RADIUS * 2)
#define DOUBLE_TO_LONG(x)       ((long)(x  * 1000000.0))
#define SAMPLE_LOOP_COUNT       10

/* Command test mode */
/* #define GEOFENCE_COMMAND_TEST */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int                              g_fdgnss;
static int                              g_fdgeo;
static struct pollfd                    g_fds[POLL_FD_NUM] = {{0}};
static struct cxd56_gnss_positiondata_s g_posdat;
static struct cxd56_geofence_status_s   g_geofence_status;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_own_position()
 *
 * Description:
 *   Get the current position.
 *
 * Input Parameters:
 *   fd        - File descriptor.
 *   latitude  - Address to store latitude on success.
 *   longitude - Address to store longitude on success.
 *
 * Returned Value:
 *   Zero (OK) on success; Negative value on error.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int get_own_position(int fd, long *latitude, long *longitude)
{
  int           ret;

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

  printf("Detect position...");

  g_fds[0].fd     = g_fdgnss;
  g_fds[0].events = POLLIN;

  /* Loop untill detect position. */

  while (1)
    {
      /* Wait POS notification. */

      ret = poll(g_fds, POLL_FD_NUM, POLL_TIMEOUT_FOREVER);
      if (ret <= 0)
        {
          printf("poll error %d,%x,%x\n", ret,
                  g_fds[0].events, g_fds[0].revents);
          break;
        }

      if (g_fds[0].revents & POLLIN)
        {
          /* Read pos data. */

          ret = read(fd, &g_posdat, sizeof(g_posdat));
          if (ret < 0)
            {
              printf("Error read position data\n");
              break;
            }
          else if(ret != sizeof(g_posdat))
            {
              ret = ERROR;
              printf("Size error read position data\n");
              break;
            }
          else
            {
              ret = OK;
            }

          printf("UTC time : Hour:%d, minute:%d, sec:%d\n",
                 g_posdat.receiver.time.hour, g_posdat.receiver.time.minute,
                 g_posdat.receiver.time.sec);
          if (g_posdat.receiver.pos_dataexist)
            {
              /* Detect position. */

              *latitude   = DOUBLE_TO_LONG(g_posdat.receiver.latitude);
              *longitude  = DOUBLE_TO_LONG(g_posdat.receiver.longitude);

              printf("## POSITION: LAT %d, LNG %d \n", *latitude, *longitude);
              break;
            }
          else
            {
              printf("## No Positioning Data\n");
            }
        }
    }

_err:

  return ret;
}

/****************************************************************************
 * Name: geofence_main()
 *
 * Description:
 *   Set the region to the initial position and its E, W, S and N.
 *   Receive notification when entering or leaving each area.
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
  int   ret;
  int   i;
  int   notify_count;
  long  own_latitude  = 0;
  long  own_longitude = 0;
  struct cxd56_geofence_mode_s mode;

  /* Region data. */

  struct cxd56_geofence_region_s region_center;
  struct cxd56_geofence_region_s region_north;
  struct cxd56_geofence_region_s region_south;
  struct cxd56_geofence_region_s region_east;
  struct cxd56_geofence_region_s region_west;

  /* Program start. */

  printf("Hello, GEOFENCE SAMPLE!!\n");

  /* Get file descriptor to control GNSS. */

  printf("Open /dev/gnss \n");
  g_fdgnss = open("/dev/gps", O_RDONLY);
  if (g_fdgnss <= 0)
    {
      printf("open error:%d,%d\n", g_fdgnss, errno);
      return -ENODEV;
    }

  /* Get Own position for region data setting. */

  ret = get_own_position(g_fdgnss, &own_latitude, &own_longitude);
  if (ret < 0)
    {
      printf("Error GNSS Get position\n");
      close(g_fdgnss);
      return ret;
    }

  /* Get file descriptor to control Geofence. */

  printf("Open /dev/geofence \n");
  g_fdgeo = open("/dev/geofence", O_RDONLY);
  if (g_fdgeo <= 0)
    {
      printf("Error Geofence open\n");
      close(g_fdgnss);
      return -ENODEV;
    }

  /* Set operation mode
   *   Dead zone 5[m]
   *   Dwelling period 10[sec]
   */

  mode.deadzone         = 5;
  mode.dwell_detecttime = 10;
  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_SET_MODE, (unsigned long)&mode);
  if (ret < 0)
    {
      printf("Error Geofence set mode\n");
      return ret;
    }

  printf("Add Region \n");

  /* All clean region data */

  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_ALL_DELETE, 0);
  if (ret < 0)
    {
      printf("All delete reion error\n");
      return ret;
    }

  /* Set region data */

  /* Center */

  region_center.id        = 0;
  region_center.latitude  = own_latitude;
  region_center.longitude = own_longitude;
  region_center.radius    = GEOFENE_REGION_RADIUS;

  printf("  ID0:LAN %d, LNG %d, RAD %d\n", region_center.latitude,
         region_center.longitude, region_center.radius);

  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_ADD,
              (unsigned long)&region_center);
  if (ret < 0)
    {
      printf("Error Add region\n");
    }

  /* Delete region */
  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_DELETE, 0);
  if (ret < 0)
    {
      printf("Error Delete region \n");
    }
  printf("Delete Region ID 0 \n");

  /* Center(2) */

  region_center.id        = 0;
  region_center.latitude  = own_latitude;
  region_center.longitude = own_longitude;
  region_center.radius    = GEOFENE_REGION_RADIUS;

  printf("  ID0:LAN %d, LNG %d, RAD %d\n", region_center.latitude,
         region_center.longitude, region_center.radius);

  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_ADD,
              (unsigned long)&region_center);
  if (ret < 0)
    {
      printf("Error Add region\n");
    }

  /* North */

  region_north.id        = 1;
  region_north.latitude  = own_latitude + GEOFENE_REGION_OFFSET;
  region_north.longitude = own_longitude;
  region_north.radius    = GEOFENE_REGION_RADIUS;

  printf("  ID1:LAN %d, LNG %d, RAD %d\n", region_north.latitude,
         region_north.longitude , region_north.radius);

  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_ADD,
              (unsigned long)&region_north);
  if (ret < 0)
    {
      printf("Error Add region\n");
    }

  /* South */

  region_south.id        = 2;
  region_south.latitude  = own_latitude - GEOFENE_REGION_OFFSET;
  region_south.longitude = own_longitude;
  region_south.radius    = GEOFENE_REGION_RADIUS;

  printf("  ID2:LAN %d, LNG %d, RAD %d\n", region_south.latitude,
         region_south.longitude, region_south.radius);

  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_ADD,
              (unsigned long)&region_south);
  if (ret < 0)
    {
      printf("Error Add region\n");
    }

  /* East */

  region_east.id        = 3;
  region_east.latitude  = own_latitude;
  region_east.longitude = own_longitude + GEOFENE_REGION_OFFSET;
  region_east.radius    = GEOFENE_REGION_RADIUS;

  printf("  ID3:LAN %d, LNG %d, RAD %d\n", region_east.latitude,
         region_east.longitude, region_east.radius);

  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_ADD, (unsigned long)&region_east);
  if (ret < 0)
    {
      printf("Error Add region\n");
    }

  /* West */

  region_west.id        = 4;
  region_west.latitude  = own_latitude;
  region_west.longitude = own_longitude - GEOFENE_REGION_OFFSET;
  region_west.radius    = GEOFENE_REGION_RADIUS;

  printf("  ID4:LAN %d, LNG %d, RAD %d\n", region_west.latitude,
         region_west.longitude, region_west.radius);

  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_ADD, (unsigned long)&region_west);
  if (ret < 0)
    {
      printf("Error Add region\n");
    }

#ifdef GEOFENCE_COMMAND_TEST

  /* Get region data */

  region_center.latitude  = 0;
  region_center.longitude = 0;
  region_center.radius    = 0;
  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_GET_REGION_DATA,
              (unsigned long)&region_center);
  if (ret < 0)
    {
      printf("Error Get region\n");
    }

  printf("Check ID0:LAN %d, LNG %d, RAD %d\n" , region_center.latitude,
         region_center.longitude, region_center.radius);

  /* Delete region */

  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_DELETE, 1);
  if (ret < 0)
    {
      printf("Error Delete region \n");
    }
  printf("Delete Region ID 1 \n");

  /* North */
  region_north.id           = 19;
  region_north.latitude     = own_latitude + GEOFENE_REGION_OFFSET;
  region_north.longitude    = own_longitude;
  region_north.radius       = GEOFENE_REGION_RADIUS;

  printf("  ID19:LAN %d, LNG %d, RAD %d\n", region_north.latitude,
         region_north.longitude, region_north.radius);

  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_ADD,
              (unsigned long)&region_north);
  if (ret < 0)
    {
      printf("Error Add region\n");
    }

#endif  /* GEOFENCE_COMMAND_TEST */

  /* Used id check */

  uint32_t used_region_data;
  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_GET_USED_ID,
              (unsigned long)&used_region_data);
  if (ret < 0)
    {
      printf("Error Get Used ID \n");
    }
  printf("Used ID : 0x%08x\n", used_region_data);

  /* Start geofencing */

  printf("Start Geofencing ...\n");

  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_START, 0);
  if (ret < 0)
    {
      printf("Error start geofence\n");
      goto _err;
    }

  /* Wait transition status notify */

  g_fds[0].fd     = g_fdgeo;
  g_fds[0].events = POLLIN;
  notify_count    = 0;
  while (1)
    {
      ret = poll(g_fds, POLL_FD_NUM, POLL_TIMEOUT_FOREVER);
      if (ret <= 0)
        {
          printf("poll error %d,%x,%x\n", ret,
                 g_fds[0].events, g_fds[0].revents);
          break;
        }

    if (g_fds[0].revents & POLLIN)
        {
          ret = read(g_fdgeo, &g_geofence_status,
                      sizeof(struct cxd56_geofence_status_s));
          if (ret < 0)
            {
              printf("Error read geofence status data\n");
              break;
            }
          else if(ret != sizeof(struct cxd56_geofence_status_s))
            {
              printf("Size error read geofence status data %d:%d\n",
                     ret, sizeof(struct cxd56_geofence_status_s));
              ret = ERROR;
              break;
            }
          else
            {
              ret = OK;
            }

          /* Check updated region */

          printf("[GEO] Updated region:%d \n", g_geofence_status.update);

          /* Check region status */

          for (i=0; i<g_geofence_status.update; i++)
           {
              printf("      ID:%d, Status:", g_geofence_status.status[i].id);

              switch (g_geofence_status.status[i].status)
                {
                  case CXD56_GEOFENCE_TRANSITION_EXIT:
                    {
                      printf("EXIT\n");
                    }
                    break;
                  case CXD56_GEOFENCE_TRANSITION_ENTER:
                    {
                      printf("ENTER\n");
                    }
                    break;
                  case CXD56_GEOFENCE_TRANSITION_DWELL:
                    {
                      printf("DWELL\n");
                    }
                    break;
                  default:
                    {
                      printf("UNKNOWN\n");
                    }
                    break;
                }
           }

          ret = read(g_fdgnss, &g_posdat, sizeof(g_posdat));
          if (ret < 0)
            {
              printf("Error read GNSS position data\n");
              goto _err;
            }
          else if(ret != sizeof(g_posdat))
            {
              ret = ERROR;
              printf("Size error read GNSS position data\n");
              goto _err;
            }
          else
            {
              ret = OK;
            }

          printf("[GNSS] POS: LAT %d, LNG %d \n",
                 DOUBLE_TO_LONG(g_posdat.receiver.latitude),
                 DOUBLE_TO_LONG(g_posdat.receiver.longitude));

          notify_count++;
          if (notify_count > SAMPLE_LOOP_COUNT)
            {
              break;
            }
        }
    }

  /* Stop Geofence. */

  ret = ioctl(g_fdgeo, CXD56_GEOFENCE_IOCTL_STOP, 0);
  if (ret < 0)
    {
      printf("Error stop geofence\n");
    }

  /* Stop GNSS. */

  ret = ioctl(g_fdgnss, CXD56_GNSS_IOCTL_STOP, 0);
  if (ret < 0)
    {
      printf("Error stop gnss\n");
    }

_err:

  /* Release Geofence file descriptor. */

  ret = close(g_fdgeo);
  if (ret < 0)
    {
      printf("Error close geofence\n");
    }

  /* Release GNSS file descriptor. */

  ret = close(g_fdgnss);
  if (ret < 0)
    {
      printf("Error close gnss\n");
    }

  printf("End of GEOFENCE Sample:%d\n", ret);

  return ret;
}
