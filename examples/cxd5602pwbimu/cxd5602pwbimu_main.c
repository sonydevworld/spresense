/****************************************************************************
 * examples/cxd5602pwbimu/cxd5602pwbimu_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <inttypes.h>
#include <fcntl.h>
#include <stdio.h>
#include <poll.h>

#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD5602PWBIMU_DEVPATH      "/dev/cxd5602pwbimu0"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * sensor_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  int ret;
  int result = 0;

  fd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      printf("Device %s open failure. %d\n", CXD5602PWBIMU_DEVPATH, fd);
      return -1;
    }

  for (; ; )
    {
      cxd5602pwbimu_packet_st_t data;

      ret = read(fd, &data, sizeof(cxd5602pwbimu_packet_st_t));
      if (ret != sizeof(cxd5602pwbimu_packet_st_t))
        {
          fprintf(stderr, "Read failed.\n");
          result = -1;
          break;
        }
      else
        {
          printf("%d,%"PRIu32",%f,%f,%f,%f,%f,%f,%f\n",
                data.id,
                data.sensor_time,
                data.temp,
                data.gx, data.gy, data.gz,
                data.ax, data.ay, data.az);
          fflush(stdout);
        }
    }

  close(fd);

  return result;
}
