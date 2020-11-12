/****************************************************************************
 * examples/multi_webcamera/multiwebcam_main.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <pthread.h>

#include "multiwebcam_perf.h"
#include "multiwebcam_util.h"
#include "multiwebcam_server.h"
#include "multiwebcam_threads.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIDEO_DEV_PATH  "/dev/video0"

#ifdef CONFIG_EXAMPLES_MULTIWEBCAM_USE_HTTPMJPEG
#  define MULTIWEBCAM_PORT_NO  (80)
#else
#  define MULTIWEBCAM_PORT_NO  (10080)
#endif

/****************************************************************************
 * multiwebcam_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;
  int rsock;
  int wsock;
  struct sockaddr_in client;
  pthread_t cam_thd;
  pthread_t jpeg_thd;

  int v_fd;
  struct v_buffer *vbuffs;

  video_initialize(VIDEO_DEV_PATH);

  v_fd = open(VIDEO_DEV_PATH, 0);
  if (v_fd < 0)
    {
      printf("ERROR: Failed to open video.errno = %d\n", errno);
      return -1;
    }

  ret = multiwebcam_prepare_camera_buf(v_fd, V4L2_BUF_TYPE_STILL_CAPTURE,
                                       V4L2_BUF_MODE_RING, 2 /* buffer num */,
                                       &vbuffs);
  if (ret < 0)
    {
      printf("ERROR: prepare_camera_buf failed: %d\n", ret);
      close(v_fd);
      return -1;
    }


  /* Set Auto Whiltebalance */

  ret = multiwebcam_set_ext_ctrls(v_fd, V4L2_CTRL_CLASS_CAMERA,
                                  V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
                                  V4L2_WHITE_BALANCE_FLUORESCENT);

  /* make socket */

  rsock = multiwebcam_initserver(MULTIWEBCAM_PORT_NO /* Port Number */);

  /* Start Camera loop */

  cam_thd = multiwebcam_start_camerathread(v_fd);
  (void)cam_thd;

  while(1)
    {

      /* accept TCP connection from client */

      wsock = multiwebcam_waitconnection(rsock, &client);

      if (wsock > 0)
        {
          printf("Start Jpeg thread.\n");
          jpeg_thd = multiwebcam_start_jpegsender(wsock);
          pthread_join(jpeg_thd, NULL);
          printf("Finith Jpeg thread.\n");
          wsock = 0;
          sleep(1);
        }
    }

  /* NOTREACHED */

  multiwebcam_release_camera_buf(vbuffs, 1);

  close(rsock);
  close(v_fd);

  return 0;
}
