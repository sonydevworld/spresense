/****************************************************************************
 * examples/lte_hibernation_wake_socket/file_utils.c
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: app_save_file
 ****************************************************************************/

int app_save_file(char *filename, uint8_t *data, int size)
{
  int fd;
  struct stat tmp;
  size_t s;

  /* If previous context data exists, remove it. */

  if (stat(filename, &tmp) == 0)
    {
      unlink(filename);
    }

  printf("write to %s\n", filename);

  fd = open(filename, O_RDONLY | O_WRONLY | O_CREAT);
  if (fd < 0)
    {
      printf("Failed to open %s. errno:%d\n", filename, errno);
      return -1;
    }

  s = write(fd, data, size);
  if (s != size)
    {
      printf("Failed to write context data. errno:%d\n", errno);
    }

  close(fd);

  return 0;
}

/****************************************************************************
 * Name: app_read_file
 ****************************************************************************/

int app_read_file(char *filename, uint8_t *data, int size)
{
  int fd;
  struct stat tmp;
  size_t s;

  if (stat(filename, &tmp) == 0)
    {
      printf("read from %s\n", filename);

      fd = open(filename, O_RDONLY);
      if (fd < 0)
        {
          printf("Failed to open %s. errno:%d\n", filename, errno);
          return -1;
        }

      s = read(fd, data, size);
      close(fd);
      unlink(filename);
    }
  else
    {
      return -1;
    }

  return s;
}

/****************************************************************************
 * Name: app_file_exist
 ****************************************************************************/

bool app_file_exist(char *filename)
{
  struct stat tmp;

  if (stat(filename, &tmp) == 0)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: app_file_size
 ****************************************************************************/

int app_file_size(char *filename)
{
  struct stat tmp;

  if (stat(filename, &tmp) == 0)
    {
      return tmp.st_size;
    }

  return 0;
}
