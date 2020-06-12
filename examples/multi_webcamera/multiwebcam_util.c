/****************************************************************************
 * examples/multi_webcamera/multiwebcam_util.c
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
#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nuttx/video/video.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>

#include "multiwebcam_util.h"

static struct v_buffer *alloced_buffers = NULL;
static struct v_buffer *empty_queue = NULL;
static struct v_buffer *action_queue = NULL;

int multiwebcam_prepare_camera_buf(int    fd,
                       enum v4l2_buf_type type,
                       uint32_t           buf_mode,
                       uint8_t            buffernum,
                       struct v_buffer  **buffers)
{
  int ret;
  int cnt;
  uint8_t n_buffers;
  uint32_t fsize;
  struct v4l2_format         fmt = {0};
  struct v4l2_requestbuffers req = {0};
  struct v4l2_buffer         buf = {0};

  /* VIDIOC_REQBUFS initiate user pointer I/O */

  req.type   = type;
  req.memory = V4L2_MEMORY_USERPTR;
  req.count  = buffernum;
  req.mode   = buf_mode;
  
  ret = ioctl(fd, VIDIOC_REQBUFS, (unsigned long)&req);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_REQBUFS: errno = %d\n", errno);
      return ret;
    }

  /* VIDIOC_S_FMT set format */

  fmt.type                = type;
  fmt.fmt.pix.width       = EXAMPLE_CAMERA_HSIZE;
  fmt.fmt.pix.height      = EXAMPLE_CAMERA_VSIZE;
  fmt.fmt.pix.field       = V4L2_FIELD_ANY;
  fmt.fmt.pix.pixelformat = EXAMPLE_CAMERA_PIXFMT;

  ret = ioctl(fd, VIDIOC_S_FMT, (unsigned long)&fmt);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_S_FMT: errno = %d\n", errno);
      return -1;
    }

  /* VIDIOC_QBUF enqueue buffer */

  *buffers = malloc(sizeof(struct v_buffer) * buffernum);
  if (!*buffers)
    {
      printf("Out of memory\n");
      return -1;
    }

  /* fsize is divided by 7 from YUV full size because of JPEG compression */

  fsize = EXAMPLE_CAMERA_HSIZE * EXAMPLE_CAMERA_VSIZE * sizeof(uint16_t) / 7;

  for (n_buffers = 0; n_buffers < buffernum; ++n_buffers)
    {
      (*buffers)[n_buffers].length = fsize;
      (*buffers)[n_buffers].id = n_buffers;

      /* Note: VIDIOC_QBUF set buffer pointer.
       *       Buffer pointer must be 32bytes aligned.
       */

      (*buffers)[n_buffers].start  = memalign(32, fsize);
      if (!(*buffers)[n_buffers].start)
        {
          printf("Out of memory\n");
          for (; n_buffers > 0; n_buffers--)
            {
              free((*buffers)[n_buffers-1].start);
            }
          free(*buffers);
          *buffers = NULL;
          return -1;
        }
      (*buffers)[n_buffers].jpg_len = 0;
      (*buffers)[n_buffers].next = NULL;
    }

  for (cnt = 0; cnt < n_buffers; cnt++)
    {
      memset(&buf, 0, sizeof(v4l2_buffer_t));
      buf.type = type;
      buf.memory = V4L2_MEMORY_USERPTR;
      buf.index = (*buffers)[cnt].id;
      buf.m.userptr = (unsigned long)(*buffers)[cnt].start;
      buf.length = (*buffers)[cnt].length;

      ret = ioctl(fd, VIDIOC_QBUF, (unsigned long)&buf);
      if (ret)
        {
          printf("Fail QBUF %d\n", errno);
          return -errno;
        }
    }

  /* VIDIOC_STREAMON start stream */

  if (type == V4L2_BUF_TYPE_STILL_CAPTURE)
    {
      int vtype = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      ret = ioctl(fd, VIDIOC_STREAMON, (unsigned long)&vtype);
      if (ret < 0)
        {
          printf("Failed to VIDIOC_STREAMON: errno = %d\n", errno);
          multiwebcam_release_camera_buf(*buffers, buffernum);
          return -errno;
        }

      ret = ioctl(fd, VIDIOC_TAKEPICT_START, 0);
      if (ret)
        {
          printf("Fail TAKEPICT_START: errno = %d\n", errno);
          multiwebcam_release_camera_buf(*buffers, buffernum);
          return -errno;
        }
    }

  ret = ioctl(fd, VIDIOC_STREAMON, (unsigned long)&type);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_STREAMON: errno = %d\n", errno);
      return -errno;
    }

  alloced_buffers = *buffers;
  empty_queue = NULL; /* The buffer is already queued into video driver */
  action_queue = NULL;

  return OK;
}

int multiwebcam_get_picture_buf(int v_fd, v4l2_buffer_t *buf, enum v4l2_buf_type type)
{
  int ret;

  memset(buf, 0, sizeof(v4l2_buffer_t));
  buf->type = type;
  buf->memory = V4L2_MEMORY_USERPTR;

  ret = ioctl(v_fd, VIDIOC_DQBUF, (unsigned long)buf);
  if (ret)
    {
      printf("Fail DQBUF %d\n", errno);
      ret = -errno;
    }

  return ret;
}

int multiwebcam_set_picture_buf(int v_fd, v4l2_buffer_t *buf)
{
  int ret;

  ret = ioctl(v_fd, VIDIOC_QBUF, (unsigned long)buf);
  if (ret)
    {
      printf("Fail QBUF %d\n", errno);
      ret = -errno;
    }

  return ret;
}

void multiwebcam_release_camera_buf(struct v_buffer  *buffers, uint8_t bufnum)
{
  uint8_t cnt;
  if (buffers)
    {
      for (cnt = 0; cnt < bufnum; cnt++)
        {
          if (buffers[cnt].start)
            {
              free(buffers[cnt].start);
            }
        }

      free(buffers);
    }
}

int multiwebcam_set_ext_ctrls(int v_fd, uint16_t ctl_cls, uint16_t cid, int32_t  value)
{
  struct v4l2_ext_controls param = {0};
  struct v4l2_ext_control ctl_param = {0};

  ctl_param.id = cid;
  ctl_param.value = value;

  param.ctrl_class = ctl_cls;
  param.count = 1;
  param.controls = &ctl_param;

  return ioctl(v_fd, VIDIOC_S_EXT_CTRLS, (unsigned long)&param);
}

struct v_buffer *multiwebcam_get_vbuffer(struct v4l2_buffer *buf)
{
  alloced_buffers[buf->index].next = NULL;
  alloced_buffers[buf->index].jpg_len = (uint32_t)buf->bytesused;

  return &alloced_buffers[buf->index];
}

static void push_vbuf_queue(struct v_buffer *buf, struct v_buffer **v_queue)
{
  struct v_buffer *tmp;

  for (tmp = *v_queue; tmp != NULL; tmp = tmp->next)
    {
      if (tmp->next == NULL) /* find the tail */
        {
          break;
        }
    }

  if (tmp != NULL)
    {
      tmp->next = buf;
    }
  else
    {
      *v_queue = buf;
    }
}

static struct v_buffer * pull_vbuf_queue(struct v_buffer **v_queue)
{
  struct v_buffer *tmp = *v_queue;

  if (*v_queue != NULL)
    {
      *v_queue = (*v_queue)->next;
      (*v_queue)->next = NULL;
    }

  return tmp;
}

void multiwebcam_push_empty(struct v_buffer *buf)
{
  return push_vbuf_queue(buf, &empty_queue);
}

void multiwebcam_push_action(struct v_buffer *buf)
{
  return push_vbuf_queue(buf, &action_queue);
}

struct v_buffer * multiwebcam_pull_empty(void)
{
  return pull_vbuf_queue(&empty_queue);
}

struct v_buffer * multiwebcam_pull_action(void)
{
  return pull_vbuf_queue(&action_queue);
}

bool multiwebcam_is_actionqueue_empty(void)
{
  return (action_queue == NULL);
}

bool multiwebcam_is_emptyqueue_empty(void)
{
  return (empty_queue == NULL);
}
