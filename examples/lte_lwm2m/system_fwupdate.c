/****************************************************************************
 * lte_lwm2m/system_fwupdate.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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
#include <stdlib.h>
#include <string.h>
#include <libgen.h>
#include <errno.h>
#include <debug.h>

#include <sys/stat.h>
#include <fcntl.h>

#include "fwuputils/fwup_client.h"

#include "lwm2mclient.h"
#include "liblwm2m.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PRV_PACKAGE_FILE      "/mnt/spif/package.bin"

#define TYPE_VALUE(a,b,c,d)   ((a) | ((b) << 8) | ((c) << 16) | ((d) << 24))
#define TYPE_PACKAGE_NAME     TYPE_VALUE('N', 'A', 'M', 'E')
#define TYPE_PACKAGE_VERSION  TYPE_VALUE('V', 'E', 'R', '.')

#define FWUPDATE_WORK_SIZE    4096

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct package_header_s
{
  uint32_t  fwtype;
  uint32_t  fwsize;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t get_file_size(const char *pathname)
{
  struct stat stbuf;

  if (stat(pathname, &stbuf) < 0)
    {
      return 0;
    }

  return stbuf.st_size;
}

static int do_partial_download(struct fwup_client_s *fwup, FILE *fp,
                               uint32_t fwtype, uint32_t fwsize)
{
  int ret = OK;
  uint32_t *buf = NULL;
  uint32_t size;
  uint32_t remain = fwsize;

  buf = (uint32_t*)malloc(FWUPDATE_WORK_SIZE); /* buf must be 4byte alignment */
  if (buf == NULL)
    {
      return -ENOMEM;
    }

  do {

    size = (remain < FWUPDATE_WORK_SIZE) ? remain : FWUPDATE_WORK_SIZE;

    ret = fread(buf, 1, size, fp);
    if (size != ret)
      {
        break;
      }

    if ((fwtype != TYPE_PACKAGE_NAME) && (fwtype != TYPE_PACKAGE_VERSION))
      {
        ret = fwup->download(fwtype, fwsize, buf, size);
      }
    else
      {
        ret = OK;
      }

    remain -= size;

    printf("->dl(0x%08lx, %ld / %ld): ret=%d\n",
           (uint32_t)buf, fwsize - remain, fwsize, ret);

  } while ((0 < remain) && (ret == OK));

  free(buf);

  return ret;
}

static int do_package_download(struct fwup_client_s *fwup, char* pathname)
{
  int ret = OK;
  uint32_t fwtype;
  uint32_t fwsize;
  FILE *fp;
  int fwnum = 0;

  struct package_header_s header;

  fp = fopen(pathname, "rb");

  while (1)
    {
      /* get package header */

      ret = fread(&header, 1, sizeof(struct package_header_s), fp);
      if (sizeof(struct package_header_s) != ret)
        {
          break; /* end of byte stream */
        }

      fwtype = header.fwtype;
      fwsize = header.fwsize;

      /* debug information */

      printf("File: %s(%d)\n", pathname, fwnum++);
      printf("Size: %ld\n", fwsize);

      ret = do_partial_download(fwup, fp, fwtype, fwsize);
      if (ret)
        {
          break;
        }
    }

  fclose(fp);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int get_package_info(char *pkg_name, size_t pkg_name_len,
                     char *pkg_version, size_t pkg_ver_len)
{
  int    ret = 0;
  FILE   *fp;
  size_t len;
  struct package_header_s header;
  long   offset;

  /* parameter check */

  if (!pkg_name || !pkg_version)
    {
      return -1;
    }

  if ((pkg_name_len <= 0) || (pkg_ver_len <= 0))
    {
      return -1;
    }

  /* buffer clear */

  memset(pkg_name, 0, pkg_name_len);
  memset(pkg_version, 0, pkg_ver_len);

  fp = fopen(PRV_PACKAGE_FILE, "rb");
  if (fp == NULL)
    {
      return -1;
    }

  offset = 0;

  while (1)
    {
      /* get package header */

      len = fread(&header, 1, sizeof(struct package_header_s), fp);
      if (len != sizeof(struct package_header_s))
        {
          printf("fread header error: %d\n", errno);
          break;
        }

      len = header.fwsize;

      switch (header.fwtype)
        {
          case TYPE_PACKAGE_NAME:
            if (header.fwsize > pkg_name_len - 1)
              {
                goto exit;
              }
            len = fread(pkg_name, 1, header.fwsize, fp);
            break;
          case TYPE_PACKAGE_VERSION:
            if (header.fwsize > pkg_ver_len - 1)
              {
                goto exit;
              }
            len = fread(pkg_version, 1, header.fwsize, fp);
            break;
          default:
            break;
        }

      if (len != header.fwsize)
        {
          printf("fread payload error: %d\n", errno);
          break;
        }

      offset += sizeof(struct package_header_s) + header.fwsize;

      ret = fseek(fp, offset, SEEK_SET);
      if (ret != 0)
        {
          printf("fseek error: %d\n", errno);
          break;
        }
    }

exit:
  fclose(fp);

  return OK;
}

int save_package(void *buffer, size_t length)
{
  int    ret = 0;
  FILE   *fp;
  size_t len;

  fp = fopen(PRV_PACKAGE_FILE, "wb");
  if (fp == NULL)
    {
      return -1;
    }

  len = fwrite(buffer, 1, length, fp);
  if (len != length)
    {
      ret = -1;
    }

  fclose(fp);

  return ret;
}

int execute_fwupdate(void)
{
  int ret = OK;
  char *pathname = PRV_PACKAGE_FILE;

  /*
   * Setup for FW Update
   */

  struct fwup_client_s *fwup = fwup_client_setup();

  /*
   * FW Update Sequence Initialization
   */

  fwup->init();

  /* debug information */

  printf("File: %s\n", pathname);
  printf("Size: %ld\n", get_file_size(pathname));

  /*
   * FW Download into SPI-Flash
   * - support the partial download depending on the size of work memory
   */

  ret = do_package_download(fwup, pathname);
  if (ret < 0)
    {
      return ret;
    }

  /*
   * FW Update Sequence Start after reboot
   */

  ret = fwup->update();
  printf("->update: ret=%d\n", ret);

  return ret;
}
