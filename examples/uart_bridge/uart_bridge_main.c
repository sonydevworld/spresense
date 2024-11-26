/****************************************************************************
 * examples/uart_bridge/uart_bridge_main.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <fcntl.h>

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#define DEV_UART0 "/dev/ttyS0"
#define DEV_UART2 "/dev/ttyS2"

#define BUFLEN  (256)

/****************************************************************************
 * Privete Data
 ****************************************************************************/

static char buf[BUFLEN];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;
  int uart0;
  int uart2;
  struct pollfd pfd[2];

  uart0 = open(DEV_UART0, O_RDWR);
  uart2 = open(DEV_UART2, O_RDWR);

  if (uart0 < 0 || uart2 < 0)
    {
      printf("Could not open device files\n");
      close(uart0); /* Maybe opened, try close */
      close(uart2); /* Maybe opened, try close */
      return -1;
    }

  fclose(stdin);
  fclose(stdout);

  while (1)
    {
      pfd[0].fd     = uart0;
      pfd[0].events = POLLIN;

      pfd[1].fd     = uart2;
      pfd[1].events = POLLIN;

      /* Wait for receiving data from both side */

      poll(pfd, 2, -1);
      
      /* Check if it receives some from uart0 */

      if (pfd[0].revents & POLLIN)
        {
          ret = read(uart0, buf, BUFLEN);
          write(uart2, buf, ret);
        }

      /* Check if it receives some from uart2 */

      if (pfd[1].revents & POLLIN)
        {
          ret = read(uart2, buf, BUFLEN);
          write(uart0, buf, ret);
        }
    }

  /* Never reach here, but just in case */

  close(uart0);
  close(uart2);

  return 0;
}
