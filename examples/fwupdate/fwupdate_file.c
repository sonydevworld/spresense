/****************************************************************************
 * fwupdate/fwupdate_file.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
#include <strings.h>
#include <libgen.h>
#include <errno.h>
#include <debug.h>

#include <sys/stat.h>
#include <fcntl.h>

#include "fwuputils/fwup_client.h"

#include "fwupdate_local.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

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

static enum fw_type_e get_file_type(char *pathname)
{
  char *filename = basename(pathname);

  if (0 == strncasecmp(filename, "sbl", 3))
    {
      return FW_SBL;
    }
  else if (0 == strncasecmp(filename, "updater", 7))
    {
      return FW_UPDATER;
    }
  else if (0 == strncasecmp(filename, "usb", 3))
    {
      return FW_UPDATER;
    }
  else if (0 == strncasecmp(filename, "nuttx", 5))
    {
      return FW_APP;
    }
  else if (0 == strncasecmp(filename, "bcm20706fw", 10))
    {
      return FW_APP;
    }
  else
    {
      return FW_SYS;
    }
}

static const char *get_file_type_string(enum fw_type_e fwtype)
{
  switch (fwtype)
    {
    case FW_APP: return "FW_APP";
    case FW_SYS: return "FW_SYS";
    case FW_UPDATER: return "FW_UPDATER";
    case FW_SBL: return "FW_SBL";
    default: return "";
    }
}

static int do_partial_download(struct fwup_client_s *fwup, char* pathname,
                               enum fw_type_e fwtype, uint32_t fwsize)
{
  int ret = OK;
  FILE *fp;
  uint32_t *buf = NULL;
  uint32_t size;
  uint32_t remain = fwsize;

  fp = fopen(pathname, "rb");

  do {

    size = MIN(remain, CONFIG_EXAMPLES_FWUPDATE_DOWNLOAD_WORK_SIZE);
    buf = (uint32_t*)malloc(size); /* buf must be 4byte alignment */
    if (buf == NULL)
      {
        ret = -ENOMEM;
        break;
      }

    ret = fread(buf, 1, size, fp);
    if (size != ret)
      {
        /* sanity check */

        free(buf);
        ret = -ENODATA;
        break;
      }

    ret = fwup->download(fwtype, fwsize, buf, size);

    remain -= size;

    printf("->dl(0x%08x, %d / %d): ret=%d\n",
           (uint32_t)buf, fwsize - remain, fwsize, ret);

    fwup->msgsync();
    free(buf);

  } while (0 < remain);

  fclose(fp);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int fwupdate_file(char* pathname)
{
  int ret = OK;
  enum fw_type_e fwtype;
  uint32_t       fwsize;

  /*
   * Setup for FW Update
   */

  struct fwup_client_s *fwup = fwup_client_setup();

  /*
   * FW Update Sequence Initialization
   */

  fwup->init();

  fwsize = get_file_size(pathname);
  fwtype = get_file_type(pathname);

  /* debug information */

  printf("File: %s\n", pathname);
  printf("Size: %d\n", fwsize);
  printf("Type: %s\n", get_file_type_string(fwtype));
  if (fwsize <= 0)
    {
      return -ENOENT;
    }

  /*
   * FW Download into SPI-Flash
   * - support the partial download depending on the size of work memory
   */

  ret = do_partial_download(fwup, pathname, fwtype, fwsize);
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

