/****************************************************************************
 * examples/ble_mouse_central/mouse_reader_main.c
 *
 *   Copyright 2024 Sony Semiconductor Solutions Corporation
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
#include <nuttx/input/mouse.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  int ret;
  struct mouse_report_s sample;

  /* Open the mouse device for blocking reading */

  fd = open(CONFIG_EXAMPLES_BLE_MOUSE_INPUT_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      printf("ERROR: open ret=%d, errno=%d\n", fd, errno);
      return -ENODEV;
    }

  /* Display the collected mouse samples */

  for (; ; )
    {
      /* Wait for data and read */

      ret = read(fd, &sample, sizeof(sample));
      if (ret < 0)
        {
          printf("ERROR: read ret=%d, errno=%d\n", ret, errno);
          break;
        }

      printf("Button: %d (X, Y): (%4d, %4d)"
#ifdef CONFIG_INPUT_MOUSE_WHEEL
             " Wheel: %d"
#endif
             "\n",
             sample.buttons, sample.x, sample.y
#ifdef CONFIG_INPUT_MOUSE_WHEEL
             , sample.wheel
#endif
            );
    }

  close(fd);

  return ret;
}
