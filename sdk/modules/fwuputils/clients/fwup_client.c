/****************************************************************************
 * modules/fwuputils/clients/fwup_client.c
 *
 *   Copyright 2018, 2021 Sony Semiconductor Solutions Corporation
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
#include <debug.h>
#include <assert.h>

#include <errno.h>
#include <sched.h>
#include <mqueue.h>
#include <fcntl.h>

#include <arch/chip/chip.h>
#include "hardware/cxd5602_backupmem.h"

#include "fwuputils/fwup_client.h"
#include "sys_update_mgr.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int fwup_client_initialize(void);
static int fwup_client_msgsync(void);
static int fwup_client_download(enum fw_type_e fwtype, uint32_t fwsize,
                                void *data, uint32_t size);
static int fwup_client_update(void);
static int fwup_client_suspend(void);
static int fwup_client_resume(void);
static int fwup_client_abort(void);
static int fwup_client_finalize(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static UM_Handle g_handle;
static uint32_t  g_remain;

static struct fwup_client_s g_client =
{
  .init     = fwup_client_initialize,
  .msgsync  = fwup_client_msgsync,
  .download = fwup_client_download,
  .update   = fwup_client_update,
  .suspend  = fwup_client_suspend,
  .resume   = fwup_client_resume,
  .abort    = fwup_client_abort,
  .uninit   = fwup_client_finalize,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct fwup_client_s *get_client(void)
{
  return &g_client;
}

static int fwup_client_initialize(void)
{
  return fw_um_init(false);
}

static int fwup_client_msgsync(void)
{
  /* old function to keep for compatibility */

  return 0;
}

static int get_fw_property(enum fw_type_e fwtype,
                           uint32_t *type, char *keyfile)
{
  int ret = 0;

  switch (fwtype)
    {
      case FW_APP:
        *type = UM_TYPE_FIRMWARE;
        strncpy(keyfile, "app.key", 16);
        break;
     case FW_SYS:
        *type = UM_TYPE_FIRMWARE;
        strncpy(keyfile, "sys.key", 16);
        break;
      case FW_UPDATER:
        *type = UM_TYPE_FIRMWARE;
        strncpy(keyfile, "updater.key", 16);
        break;
     case FW_SBL:
        *type = UM_TYPE_SBL;
        break;
      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

static int fwup_open(enum fw_type_e fwtype, uint32_t fwsize)
{
  int ret = 0;
  char keyfile[32];
  uint32_t type = 0;

  memset(keyfile, 0, sizeof(keyfile));

  ret = get_fw_property(fwtype, &type, keyfile);
  if (ret)
    {
      DEBUGASSERT(ret == 0);
      return ret;
    }

  g_handle = fw_um_open(keyfile, fwsize, type);
  g_remain = fwsize;

  return ret;
}

static int fwup_write(void *data, uint32_t size)
{
  int ret = 0;

  ret = fw_um_commit(g_handle, (void *)CXD56_PHYSADDR(data), size);
  if (ret == 0)
    {
      g_remain -= size;
    }

  return ret;
}

static int fwup_close(void)
{
  int ret;

  ret = fw_um_close(g_handle);
  g_remain = 0;

  return ret;
}

static int fwup_client_download(enum fw_type_e fwtype, uint32_t fwsize,
                                void *data, uint32_t size)
{
  int ret;

  /* if now on downloading or not */

  if (g_remain == 0)
    {
      /* start to download */

      ret = fwup_open(fwtype, fwsize);
    }

  /* downloading */

  ret = fwup_write(data, size);
  if (ret)
    {
      return -EIO;
    }

  /* if download finish or not */

  if (g_remain == 0)
    {
      /* stop to download */

      ret = fwup_close();
    }

  return ret;
}

static int fwup_client_update(void)
{
  return fw_um_doupdatesequence();
}

static int fwup_client_suspend(void)
{
  return fw_um_checkpoint();
}

static int fwup_client_resume(void)
{
  return fw_um_init(true);
}

static int fwup_client_abort(void)
{
  fw_um_abort();

  return 0;
}

static int fwup_client_finalize(void)
{
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct fwup_client_s *fwup_client_setup(void)
{
  struct fwup_client_s *client = get_client();

  return client;
}

uint32_t fwup_client_getfreespace(void)
{
  return BKUP->fw_free_space;
}
