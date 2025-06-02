/****************************************************************************
 * system/xmodem/xmodem_main.c
 *
 *   Copyright 2019,2021,2025 Sony Semiconductor Solutions Corporation
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

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <libgen.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/boardctl.h>
#include <system/xmodem.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define XMODEM_VERSION        "1.0.0"
#define XMODEM_BUFF_SIZE      128 * 1024
#define XMODEM_DEV	      "/dev/console"
#define XMODEM_PADDING_BYTE   0x1A
#define XMODEM_DATA_CONTINUE  0xFE
#define XMODEM_DATA_COMPLETE  0xFF

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int xmodem_receive_data_to_file(FILE *fptr)
{
  int uart_fd;
  int recv_size;
  int end_condition;
  unsigned char buff[XMODEM_BUFF_SIZE];
  XMHANDLE handle;
  struct termios saveterm;
  struct termios term;

  /* Open UART for transferring X-Modem */

  uart_fd = open(XMODEM_DEV, O_RDWR);

  if (uart_fd < 0)
    {
      printf("Error: Failed to open %s.\n", XMODEM_DEV);
      return -1;
    }

  /* Get X-Modem handle for transferring */

  handle = xmodemHandleInit(uart_fd);

  if (!handle)
    {
      printf("Error: Failed to get X-Modem handle.\n");
      return -1;
    }

  /* Set the terminal to the raw mode */

  tcgetattr(uart_fd, &term);
  memcpy(&saveterm, &term, sizeof(struct termios));
  cfmakeraw(&term);
  tcsetattr(uart_fd, TCSANOW, &term);

  /* Receive file body */

  while (true)
    {
      /* Receive data from target */

      recv_size = xmodemReceive(handle, buff, XMODEM_BUFF_SIZE);

      if (recv_size < 0)
        {
          printf("Error: Failed to receive data(%d).\n", recv_size);
          goto errout;
        }

      /* Remove padding bytes */

      while (recv_size > 1)
        {
          if (buff[recv_size - 1] != XMODEM_PADDING_BYTE)
            {
              break;
            }

            recv_size--;
        }

      /* End of receive byte is file transfer status
       * XMODEM_DATA_CONTINUE: There are next data
       * XMODEM_DATA_COMPLETE: End of file
       */

      end_condition = buff[recv_size - 1];

      recv_size--;

      if (end_condition == XMODEM_DATA_CONTINUE ||
          end_condition == XMODEM_DATA_COMPLETE)
        {
          size_t i = 0;
          while (i < recv_size)
            {
              int ret = fwrite(buff, 1, recv_size, fptr);
              if (ret < 0)
                {
                  printf("Error: Failed to write file.\n");
                  goto errout;
                }

              i += ret;
            }

          if (end_condition == XMODEM_DATA_COMPLETE)
            {
              break;
            }
        }
      else
        {
          printf("Error: Invalid receive data(0x%02X).\n", end_condition);
          break;
        }
    }

errout:

  /* Release X-Modem handle */

  xmodemHandleRelease(handle);

  /* Flush the serial output to assure do not hang trying to drain it */

  tcflush(uart_fd, TCIOFLUSH);

  /* Restore the saved terminal setting */

  tcsetattr(uart_fd, TCSANOW, &saveterm);

  /* Close UART */

  close(uart_fd);

  return 0;
}

static int dir_exists(const char *path)
{
  struct stat st;
  return (stat(path, &st) == 0 && S_ISDIR(st.st_mode));
}

static int make_dirs(const char *path)
{
  char tmp[64];
  char *p = NULL;
  size_t len;

  snprintf(tmp, sizeof(tmp), "%s", path);
  len = strlen(tmp);
  if (tmp[len - 1] == '/')
    {
      tmp[len - 1] = '\0';
    }

  for (p = tmp + 1; *p; p++)
    {
      if (*p == '/')
        {
          *p = 0;
          mkdir(tmp, 0777);
          *p = '/';
        }
    }

  return mkdir(tmp, 0777);
}

/****************************************************************************
 * xmodem_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  char *file_path;
  FILE *fptr;
  int ret;
  char dir_path[64];

  if (argc < 2)
    {
      printf("Version: %s\n", XMODEM_VERSION);
      printf("Usage: xmodem <file path>\n");
      return -1;
    }

  file_path = argv[1];

  /* If the directory does not exist, create it recursively */

  snprintf(dir_path, sizeof(dir_path), "%s", file_path);
  dirname(dir_path);

  if (!dir_exists(dir_path))
    {
      if (make_dirs(dir_path) != 0 && errno != EEXIST)
        {
          return -1;
        }
    }

  fptr = fopen(file_path, "wb");

  if (!fptr)
    {
      printf("File %s open failed.\n", file_path);
      return -1;
    }

  ret = xmodem_receive_data_to_file(fptr);

  fclose(fptr);

  return ret;
}
