/****************************************************************************
 * examples/audiolite_gnss1pps_dpll/gnss_addon.c
 *
 *   Copyright 2025 Sony Semiconductor Solutions Corporation
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
#include <pthread.h>
#include <fcntl.h>
#include <poll.h>

#include <arch/chip/gnss.h>
#include <arch/board/board.h>

#include "gnss_addon.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cxd56_gnss_positiondata2_s posdat;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void notify_fix(gnss_ctrl_t *gnss, uint8_t fixed)
{
  if (gnss->state == GNSS_STATE_NOTLOCK && fixed > 0)
    {
      pthread_mutex_lock(&gnss->lock);
      gnss->state = GNSS_STATE_LOCKED;
      pthread_cond_signal(&gnss->cond);
      pthread_mutex_unlock(&gnss->lock);
    }
  else if (gnss->state == GNSS_STATE_LOCKED && fixed == 0)
    {
      pthread_mutex_lock(&gnss->lock);
      gnss->state = GNSS_STATE_NOTLOCK;
      pthread_cond_signal(&gnss->cond);
      pthread_mutex_unlock(&gnss->lock);
    }
}

static pthread_addr_t gnss_location(pthread_addr_t arg)
{
  int ret;
  struct pollfd fds;
  gnss_ctrl_t *gnss = (gnss_ctrl_t *)arg;

  ioctl(gnss->devfd, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_HOT);

  while (gnss->is_running)
    {
      fds.fd     = gnss->devfd;
      fds.events = POLLIN;

      ret = poll(&fds, 1, -1);
      if (ret < 0)
        {
          printf("ERROR: poll ret=%d, errno=%d\n", ret, errno);
          break;
        }

      ret = read(gnss->devfd, &posdat, sizeof(posdat));
      if (ret != sizeof(posdat))
        {
          printf("ERROR: read ret=%d, errno=%d\n", ret, errno);
          break;
        }

      notify_fix(gnss, posdat.receiver.fix_indicator);
    }

  ioctl(gnss->devfd, CXD56_GNSS_IOCTL_STOP, 0);
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int gnss_wait_statechange(gnss_ctrl_t *gnss, int cur_state)
{
  pthread_mutex_lock(&gnss->lock);
  while (cur_state == gnss->state)
    {
      pthread_cond_wait(&gnss->cond, &gnss->lock);
    }

  cur_state = gnss->state;
  pthread_mutex_unlock(&gnss->lock);

  return cur_state;
}

int gnss_setup(gnss_ctrl_t *gnss)
{
  int ret;
  pthread_attr_t attr;
  struct sched_param param;

  gnss->state = GNSS_STATE_NOTLOCK;
  gnss->is_running = 1;
  pthread_mutex_init(&gnss->lock, NULL);
  pthread_cond_init(&gnss->cond, NULL);

  gnss->devfd = open("/dev/gps2", O_RDONLY);
  if (gnss->devfd < 0)
    {
      return -ENODEV;
    }

  ret = ioctl(gnss->devfd, CXD56_GNSS_IOCTL_WAKEUP, 0);
  if (ret < 0)
    {
      printf("ERROR: wakeup ret=%d, errno=%d\n", ret, errno);
    }

  ret = ioctl(gnss->devfd, CXD56_GNSS_IOCTL_SET_1PPS_OUTPUT, 1);
  if (ret < 0)
    {
      printf("ERROR: 1pps ret=%d, errno=%d\n", ret, errno);
      goto err_out;
    }
  
  pthread_attr_init(&attr);
  pthread_attr_getschedparam(&attr, &param);
  param.sched_priority++;
  pthread_attr_setschedparam(&attr, &param);
  ret = pthread_create(&gnss->thd, &attr, gnss_location, (void *)gnss);
  if (ret == OK)
    {
      pthread_setname_np(gnss->thd, "gnss_location");
    }
  else
    {
      printf("ERROR: create pthread ret=%d, errno=%d\n", ret, errno);
      goto err_out;
    }

  return OK;

err_out:
  close(gnss->devfd);
  gnss->devfd = -1;

  return ret;
}

void gnss_cleanup(gnss_ctrl_t *gnss)
{
  gnss->is_running = 0;
  pthread_join(gnss->thd, NULL);
  
  close(gnss->devfd);
  gnss->devfd = -1;
}
