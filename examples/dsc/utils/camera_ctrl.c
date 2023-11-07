/****************************************************************************
 * dsc/utils/camera_ctrl.c
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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
#include <nuttx/video/video.h>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "camera_ctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIDEO_DEVF "/dev/video"
#define PREVIEW_BUFFER_NUM  (2)
#define STILL_BUFFER_NUM  (1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int request_camerabuffs(int fd, enum v4l2_buf_type type)
{
  struct v4l2_requestbuffers req;

  req.type   = type;
  req.memory = V4L2_MEMORY_USERPTR;
  req.count  = type == V4L2_BUF_TYPE_STILL_CAPTURE
                 ? STILL_BUFFER_NUM
                 : PREVIEW_BUFFER_NUM;
  req.mode   = V4L2_BUF_MODE_FIFO;

  return ioctl(fd, VIDIOC_REQBUFS, (unsigned long)&req);
}

static int allocate_framebuffer(int width, int height,
                                uint32_t pix, unsigned char **buf)
{
  int fbsize = width * height * 2;

  fbsize = pix == V4L2_PIX_FMT_JPEG ? fbsize / 7 : fbsize;
  *buf = (unsigned char *)memalign(32, fbsize);

  return *buf != NULL ? fbsize : -ENOMEM;
}

static int enqueue_framebuffer(int fd, unsigned char *buf, int buf_size,
                               enum v4l2_buf_type type)
{
  struct v4l2_buffer vbuf;

  vbuf.type = type;
  vbuf.memory = V4L2_MEMORY_USERPTR;
  vbuf.index = 0;
  vbuf.m.userptr = (unsigned long)buf;
  vbuf.length = buf_size;

  return ioctl(fd, VIDIOC_QBUF, (unsigned long)&vbuf);
}

static int start_stream_local(int fd, enum v4l2_buf_type type)
{
  return ioctl(fd, VIDIOC_STREAMON, (unsigned long)&type);
}

static int stop_stream_local(int fd, enum v4l2_buf_type type)
{
  return ioctl(fd, VIDIOC_STREAMOFF, (unsigned long)&type);
}

static int start_stillcapture(int fd)
{

  return ioctl(fd, VIDIOC_TAKEPICT_START, 0);
}

static int stop_stillcapture(int fd)
{
  return ioctl(fd, VIDIOC_TAKEPICT_STOP, 0);
}

static int set_cameraformat(int fd, int width, int height,
                            enum v4l2_buf_type type, uint32_t pixfmt)
{
  struct v4l2_format fmt;

  fmt.type                = type;
  fmt.fmt.pix.width       = (uint16_t)width;
  fmt.fmt.pix.height      = (uint16_t)height;
  fmt.fmt.pix.field       = V4L2_FIELD_ANY;
  fmt.fmt.pix.pixelformat = pixfmt;

  return ioctl(fd, VIDIOC_S_FMT, (unsigned long)&fmt);
}

static int dequeue_framebuffer(int fd, unsigned char **buf,
                               enum v4l2_buf_type type)
{
  int ret = -EBUSY;

  struct v4l2_buffer v_buf;

  memset(&v_buf, 0, sizeof(v_buf));
  v_buf.type = type;
  v_buf.memory = V4L2_MEMORY_USERPTR;

  if (ioctl(fd, VIDIOC_DQBUF, (unsigned long)&v_buf) == 0)
    {
      *buf = (unsigned char *)v_buf.m.userptr;
      ret = v_buf.bytesused;
    }

  return ret;
}

static const char *get_sensorname(int fd)
{
  static struct v4l2_capability cap;

  ioctl(fd, VIDIOC_QUERYCAP, (unsigned long)&cap);

  return (const char *)cap.driver;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

const char *initialize_cameractrl(int *fd)
{
  *fd = video_initialize(VIDEO_DEVF);
  if (*fd != 0)
    {
      *fd = -ENODEV;
      return NULL;
    }

  *fd = open(VIDEO_DEVF, 0);
  if (*fd < 0)
    {
      video_uninitialize(VIDEO_DEVF);
      *fd = -ENODEV;
      return NULL;
    }

  if (request_camerabuffs(*fd, V4L2_BUF_TYPE_STILL_CAPTURE) < 0)
    {
      close(*fd);
      video_uninitialize(VIDEO_DEVF);
      *fd = -EINVAL;
      return NULL;
    }

  if (request_camerabuffs(*fd, V4L2_BUF_TYPE_VIDEO_CAPTURE) < 0)
    {
      close(*fd);
      video_uninitialize(VIDEO_DEVF);
      *fd = -EINVAL;
      return NULL;
    }

  return get_sensorname(*fd);
}

void finalize_cameractrl(int fd)
{
  close(fd);
  video_uninitialize(VIDEO_DEVF);
}

unsigned char *camera_framebuffer(int *sz)
{
  unsigned char *ret;

  /* Allocate biggest size of this usecase (JPEG on FullHD) */

  *sz = allocate_framebuffer(VIDEO_HSIZE_FULLHD, VIDEO_VSIZE_FULLHD,
                             V4L2_PIX_FMT_JPEG, &ret);

  return ret;
}

int start_streaming(int fd, unsigned char *buf, int sz)
{
  int ret;
  int preview_size = VIDEO_HSIZE_QVGA * VIDEO_VSIZE_QVGA * 2;

  request_camerabuffs(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE);

  ret = set_cameraformat(fd, VIDEO_HSIZE_QVGA, VIDEO_VSIZE_QVGA,
                     V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_PIX_FMT_RGB565);
  if (ret < 0)
    {
      return ret;
    }

  ret = release_previewimage(fd, buf);
  if (ret < 0)
    {
      return ret;
    }

  ret = release_previewimage(fd, buf + preview_size);
  if (ret < 0)
    {
      return ret;
    }

  return start_stream_local(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE);
}

int get_previewimage(int fd, unsigned char **buf)
{
  return dequeue_framebuffer(fd, buf, V4L2_BUF_TYPE_VIDEO_CAPTURE);
}

int release_previewimage(int fd, unsigned char *buf)
{
  int preview_size = VIDEO_HSIZE_QVGA * VIDEO_VSIZE_QVGA * 2;
  return enqueue_framebuffer(fd, buf, preview_size, V4L2_BUF_TYPE_VIDEO_CAPTURE);
}

int take_pictureimage(int fd, unsigned char **buf, int sz, int w, int h)
{
  int ret;

  ret = stop_stream_local(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE);

  request_camerabuffs(fd, V4L2_BUF_TYPE_STILL_CAPTURE);
  set_cameraformat(fd, w, h, V4L2_BUF_TYPE_STILL_CAPTURE, V4L2_PIX_FMT_JPEG);

  ret = enqueue_framebuffer(fd, *buf, sz, V4L2_BUF_TYPE_STILL_CAPTURE);
  if (ret < 0)
    {
      goto takepict_exit;
    }

  ret = start_stillcapture(fd);
  if (ret < 0)
    {
      return ret;
    }

  ret = dequeue_framebuffer(fd, buf, V4L2_BUF_TYPE_STILL_CAPTURE);

takepict_exit:

  stop_stillcapture(fd);

  return ret;
}

int set_ext_ctrls(int fd, uint16_t ctl_cls, uint16_t cid, int value)
{
  struct v4l2_ext_controls ctrls;
  struct v4l2_ext_control control;

  control.id = cid;
  control.value = value;

  ctrls.count = 1;
  ctrls.ctrl_class = ctl_cls;
  ctrls.controls = &control;

  return ioctl(fd, VIDIOC_S_EXT_CTRLS, (unsigned long)&ctrls);
}

int check_jpgsize(int fd, int w, int h)
{
  return set_cameraformat(fd, w, h, V4L2_BUF_TYPE_STILL_CAPTURE,
                          V4L2_PIX_FMT_JPEG);
}
