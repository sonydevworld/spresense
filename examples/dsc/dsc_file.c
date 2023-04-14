/****************************************************************************
 * dsc/dsc_file.cxx
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>
#include <semaphore.h>

#include "dsc_file.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMAGE_FILENAME_LEN (32)

#define STORAGE_PATH "/mnt/sd0/DCIM"
#define PREFIX_IMAGE "DSC"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_image_cnt;
static sem_t g_sem;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int get_next_index(const char *filter)
{
  struct dirent **list;
  int n;
  int n_free;
  char *name;
  int id = 0;

  n = scandir(STORAGE_PATH, &list, NULL, alphasort);
  if (n <= 0)
    {
      return 0;
    }

  n_free = n;

  while (n--)
    {
      name = list[n]->d_name;
      if (strncmp(name, filter, strlen(filter)) == 0)
        {
          id = atoi(name + strlen(filter)) + 1;
          break;
        }
    };

  while (n_free--)
    {
      free(list[n_free]);
    }

  free(list);

  return id;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_initialize()
 *
 * Description:
 *   Save path to write a file.
 ****************************************************************************/

bool file_initialize(void)
{
  sem_init(&g_sem, 0, 1);
  return check_sdcard_content();
}

/****************************************************************************
 * Name: check_sdcard_content()
 *
 * Description:
 *   Check the DCIM directory in the SDCard
 ****************************************************************************/

bool check_sdcard_content(void)
{
  bool retval;
  int ret;
  struct stat stat_buf;

  sem_wait(&g_sem);
  ret = stat("/mnt/sd0", &stat_buf);
  if (ret >= 0)
    {
      ret = stat(STORAGE_PATH, &stat_buf);
      if (ret < 0)
        {
          mkdir(STORAGE_PATH, 0666);
        }

      g_image_cnt = get_next_index(PREFIX_IMAGE);
      retval = true;
    }
  else
    {
      retval = false;
    }

  sem_post(&g_sem);
  return retval;
}

/****************************************************************************
 * Name: file_finalize()
 *
 * Description:
 *   Finalize file utility
 ****************************************************************************/

void file_finalize(void)
{
  sem_destroy(&g_sem);
}

/****************************************************************************
 * Name: file_writeimage()
 *
 * Description:
 *   Write a image file to selected storage.
 ****************************************************************************/

int file_writeimage(uint8_t *data, size_t len)
{
  static char s_fname[IMAGE_FILENAME_LEN];
  FILE *fp;
  int ret = OK;

  snprintf(s_fname,
           IMAGE_FILENAME_LEN,
           "%s/%s%04d.JPG",
           STORAGE_PATH, PREFIX_IMAGE, g_image_cnt++);

  printf("FILENAME:%s (%d)\n", s_fname, len);

  fp = fopen(s_fname, "wb");
  if (NULL == fp)
    {
      printf("fopen error : %d\n", errno);
      return -1;
    }

  if (len != fwrite(data, 1, len, fp))
    {
      printf("fwrite error : %d\n", errno);
      ret = -1;
    }

  fclose(fp);

  return ret;
}
