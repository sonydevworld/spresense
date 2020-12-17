/****************************************************************************
 * modules/fwuputils/manager/fwup_manager.c
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

#include <sdk/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <debug.h>

#include <errno.h>
#include <sched.h>
#include <mqueue.h>
#include <fcntl.h>

#include "fwuputils/fwup_manager.h"
#include "sys_update_mgr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fwup_mgr_s
{
  pid_t         pid;
  mqd_t         mqd;
  UM_Handle     handle;
  uint32_t      remain;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct fwup_mgr_s g_mgr;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct fwup_mgr_s *get_manager(void)
{
  return &g_mgr;
}

static int fwup_init(void)
{
  int ret = 0;

  ret = fw_um_init(false);

  return ret;
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
  struct fwup_mgr_s *mgr = get_manager();

  memset(keyfile, 0, sizeof(keyfile));

  ret = get_fw_property(fwtype, &type, keyfile);
  if (ret)
    {
      DEBUGASSERT(ret == 0);
      return ret;
    }

  mgr->handle = fw_um_open(keyfile, fwsize, type);
  mgr->remain = fwsize;

  return ret;
}

static int fwup_write(void *data, uint32_t size)
{
  int ret = 0;
  struct fwup_mgr_s *mgr = get_manager();

  ret = fw_um_commit(mgr->handle, data, size);

  mgr->remain -= size;

  return ret;
}

static int fwup_close(void)
{
  int ret = 0;
  struct fwup_mgr_s *mgr = get_manager();

  ret = fw_um_close(mgr->handle);
  mgr->remain = 0;

  return ret;
}

static int fwup_download(enum fw_type_e fwtype, uint32_t fwsize,
                         void *data, uint32_t size)
{
  int ret;
  struct fwup_mgr_s *mgr = get_manager();

  /* if now on downloading or not */

  if (mgr->remain == 0)
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

  if (mgr->remain == 0)
    {
      /* stop to download */

      ret = fwup_close();
    }

  return ret;
}

static int fwup_update(void)
{
  int ret = 0;

  ret = fw_um_doupdatesequence();

  return ret;
}

static int fwup_suspend(void)
{
  int ret = 0;

  ret = fw_um_checkpoint();

  return ret;
}

static int fwup_resume(void)
{
  int ret = 0;

  ret = fw_um_init(true);

  return ret;
}

static int fwup_abort(void)
{
  //fw_um_abort();
  return 0;
}

static void fwup_mgr_task(void)
{
  int ret = 0;
  struct fwup_mgr_s *mgr = get_manager();
  struct fwup_msg_s msg;
  struct fwup_msg_sync_param_s* syncparam;
  struct fwup_msg_dl_param_s* dlparam;

  for (;;)
    {
      ret = mq_receive(mgr->mqd, (char*)&msg, sizeof(msg), NULL);

      if (ret != sizeof(msg))
        {
          continue;
        }

      switch (msg.cmd)
        {
        case FWUP_INIT:
          ret = fwup_init();
          break;
        case FWUP_MSGSYNC:
          syncparam = (struct fwup_msg_sync_param_s*)&msg.u.syncparam;
          sem_post(syncparam->sem);
          break;
        case FWUP_DOWNLOAD:
          dlparam = (struct fwup_msg_dl_param_s*)&msg.u.dlparam;
          ret = fwup_download(dlparam->fwtype, dlparam->fwsize,
                              dlparam->data, dlparam->size);
          break;
        case FWUP_UPDATE:
          ret = fwup_update();
          break;
        case FWUP_SUSPEND:
          ret = fwup_suspend();
          break;
        case FWUP_RESUME:
          ret = fwup_resume();
          break;
        case FWUP_ABORT:
          ret = fwup_abort();
          break;
        default:
          break;
        }

    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  fwup_initialize
 *
 * Description:
 *   Initialize for FW Update Manager
 *
 ****************************************************************************/

int fwup_initialize(void)
{
  int ret = 0;
  struct fwup_mgr_s *mgr = get_manager();
  struct mq_attr mq_attr;

  mq_attr.mq_maxmsg  = 1;
  mq_attr.mq_msgsize = sizeof(struct fwup_msg_s);
  mq_attr.mq_flags   = 0;

  mgr->mqd = mq_open(FWUP_MSGQ_NAME, O_RDONLY | O_CREAT, 0666, &mq_attr);
  mgr->pid = task_create("fwup_task", 100, 2048, (main_t)fwup_mgr_task, NULL);
  mgr->remain = 0;

  return ret;
}

/****************************************************************************
 * Name:  fwup_finalize
 *
 * Description:
 *   Finalize for FW Update Manager
 *
 ****************************************************************************/

int fwup_finalize(void)
{
  int ret = 0;
  struct fwup_mgr_s *mgr = get_manager();

  mq_close(mgr->mqd);
  task_delete(mgr->pid);

  return ret;
}

