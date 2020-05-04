/****************************************************************************
 * examples/multi_webcamera/multiwebcam_util.h
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
#ifndef __EXAMPLE_MULTIWEBCAM_UTIL_H__
#define __EXAMPLE_MULTIWEBCAM_UTIL_H__

#include <nuttx/config.h>
#include <nuttx/video/video.h>

#define EXAMPLE_CAMERA_PIXFMT V4L2_PIX_FMT_JPEG

#ifdef CONFIG_EXAMPLES_MULTIWEBCAM_IMAGESIZE_QQVGA
#  define EXAMPLE_CAMERA_HSIZE  (VIDEO_HSIZE_QVGA / 2)
#  define EXAMPLE_CAMERA_VSIZE  (VIDEO_VSIZE_QVGA / 2)
#endif

#ifdef CONFIG_EXAMPLES_MULTIWEBCAM_IMAGESIZE_QVGA
#  define EXAMPLE_CAMERA_HSIZE  VIDEO_HSIZE_QVGA
#  define EXAMPLE_CAMERA_VSIZE  VIDEO_VSIZE_QVGA
#endif

#ifdef CONFIG_EXAMPLES_MULTIWEBCAM_IMAGESIZE_VGA
#  define EXAMPLE_CAMERA_HSIZE  VIDEO_HSIZE_VGA
#  define EXAMPLE_CAMERA_VSIZE  VIDEO_VSIZE_VGA
#endif

#ifdef CONFIG_EXAMPLES_MULTIWEBCAM_IMAGESIZE_HD
#  define EXAMPLE_CAMERA_HSIZE  VIDEO_HSIZE_HD
#  define EXAMPLE_CAMERA_VSIZE  VIDEO_VSIZE_HD
#endif

#ifndef EXAMPLE_CAMERA_HSIZE
#  error "You need to choice one MULTIWEBCAM_IMAGESIZE on configuration."
#endif

struct v_buffer {
  uint32_t             *start;
  uint32_t             length;
  uint32_t             jpg_len;
  int                  id;
  struct v_buffer      *next;
};

int multiwebcam_prepare_camera_buf(int    fd,
                       enum v4l2_buf_type type,
                       uint32_t           buf_mode,
                       uint8_t            buffernum,
                       struct v_buffer  **buffers);

void multiwebcam_release_camera_buf(struct v_buffer  *buffers, uint8_t bufnum);

int multiwebcam_get_picture_buf(int v_fd, v4l2_buffer_t *buf, enum v4l2_buf_type type);
int multiwebcam_set_picture_buf(int v_fd, v4l2_buffer_t *buf);
int multiwebcam_set_ext_ctrls(int v_fd, uint16_t ctl_cls, uint16_t cid, int32_t  value);

struct v_buffer *multiwebcam_get_vbuffer(struct v4l2_buffer *buf);
void multiwebcam_push_empty(struct v_buffer *buf);
void multiwebcam_push_action(struct v_buffer *buf);
struct v_buffer *multiwebcam_pull_empty(void);
struct v_buffer *multiwebcam_pull_action(void);
bool multiwebcam_is_actionqueue_empty(void);
bool multiwebcam_is_emptyqueue_empty(void);

#endif  /* __EXAMPLE_MULTIWEBCAM_UTIL_H__ */
