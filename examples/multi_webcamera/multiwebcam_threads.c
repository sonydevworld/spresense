/****************************************************************************
 * examples/multi_webcamera/multiwebcam_threads.c
 *
 *   Copyright 2019, 2020 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include "multiwebcam_perf.h"
#include "multiwebcam_util.h"
#include "multiwebcam_server.h"

static pthread_cond_t queue_cond;
static pthread_mutex_t queue_mutex;

static bool is_run = false;

static void push_video_buffer(int v_fd, struct v_buffer *vbuf)
{
  struct v4l2_buffer buf;

  memset(&buf, 0, sizeof(v4l2_buffer_t));
  buf.type = V4L2_BUF_TYPE_STILL_CAPTURE;
  buf.memory = V4L2_MEMORY_USERPTR;
  buf.index = vbuf->id;
  buf.m.userptr = (unsigned long)vbuf->start;
  buf.length = vbuf->length;

  multiwebcam_set_picture_buf(v_fd, &buf);
}

static void *camera_thread(void *param)
{
  struct v_buffer *vbuf;
  struct v4l2_buffer buf;
  int v_fd = (int)param;

  printf("== Start Camera thread ==\n");

  while(1)
    {
      multiwebcam_get_picture_buf(v_fd, &buf, V4L2_BUF_TYPE_STILL_CAPTURE);

      pthread_mutex_lock(&queue_mutex);

      /* Wait signal until jpeg_sender started */

      while (!is_run)
        {

          /* Clean up action queue */

          for (vbuf = multiwebcam_pull_action(); vbuf != NULL; vbuf = multiwebcam_pull_action())
            {
              push_video_buffer(v_fd, vbuf);
            }

          pthread_cond_wait(&queue_cond, &queue_mutex);
        }

      /* Enqueue buffer into action queue */

      multiwebcam_push_action(multiwebcam_get_vbuffer(&buf));
      pthread_cond_signal(&queue_cond);

      /* Waiting empty buffers */

      while (multiwebcam_is_emptyqueue_empty() && is_run)
        {
          pthread_cond_wait(&queue_cond, &queue_mutex);
        }

      /* Queue all buffer into video driver */

      for (vbuf = multiwebcam_pull_empty(); vbuf != NULL; vbuf = multiwebcam_pull_empty())
        {
          push_video_buffer(v_fd, vbuf);
        }

      pthread_mutex_unlock(&queue_mutex);
    }

  return NULL;
}

pthread_t multiwebcam_start_camerathread(int v_fd)
{
  pthread_t thd;
  pthread_attr_t attr;
  struct sched_param sparam;

  pthread_mutex_init(&queue_mutex, NULL);
  pthread_cond_init(&queue_cond, NULL);

  pthread_attr_init(&attr);
  sparam.sched_priority = 110;
  pthread_attr_setschedparam(&attr,&sparam);
  pthread_create(&thd, &attr, camera_thread, (void *)v_fd);

  return thd;
}

static void *jpeg_sender(void *param)
{
  PREPARE_PERF_VARIABLES();
  int ret;
  struct v_buffer *buf;
  int sock = (int)param;

  printf("-- Start JPEG thread --\n");

  /* activate camera thread */

  pthread_mutex_lock(&queue_mutex);
  is_run = true;
  pthread_cond_signal(&queue_cond);
  pthread_mutex_unlock(&queue_mutex);

  multiwabcam_sendheader(sock);
  SET_INITIAL_TIME();

  while(1)
    {
      pthread_mutex_lock(&queue_mutex);
      while(multiwebcam_is_actionqueue_empty())
        {
          pthread_cond_wait(&queue_cond, &queue_mutex);
        }
      buf = multiwebcam_pull_action();
      pthread_mutex_unlock(&queue_mutex);
 
      ret = multiwebcam_sendframe(sock, (char *)buf->start, (int)buf->jpg_len);
      PRINT_DIFF_TIME(buf->jpg_len);

      pthread_mutex_lock(&queue_mutex);
      multiwebcam_push_empty(buf);

      if (ret < 0)
        {
          close(sock);
          is_run = false;
        }

      pthread_cond_signal(&queue_cond);
      pthread_mutex_unlock(&queue_mutex);

      if (ret < 0)
        {
          break;
        }
    }

  return NULL;
}

pthread_t multiwebcam_start_jpegsender(int sock)
{
  pthread_t thd;
  pthread_attr_t attr;
  struct sched_param sparam;

  pthread_attr_init(&attr);
  sparam.sched_priority = 101;
  pthread_attr_setschedparam(&attr,&sparam);
  pthread_create(&thd, &attr, jpeg_sender, (void *)sock);

  return thd;
}
