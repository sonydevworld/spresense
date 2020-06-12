/****************************************************************************
 * modules/fwuputils/clients/fwup_client.c
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

#include "hardware/cxd5602_backupmem.h"

#include "fwuputils/fwup_manager.h"
#include "fwuputils/fwup_client.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

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

static mqd_t g_fwup_mqd;
static sem_t g_fwup_sem;

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
  struct fwup_msg_s msg;

  msg.cmd = FWUP_INIT;

  return mq_send(g_fwup_mqd, (const char*)&msg, sizeof(msg), 0);
}

static int fwup_client_msgsync(void)
{
  int ret;
  struct fwup_msg_s msg;

  msg.cmd = FWUP_MSGSYNC;

  msg.u.syncparam.sem = &g_fwup_sem;

  ret = mq_send(g_fwup_mqd, (const char*)&msg, sizeof(msg), 0);

  sem_wait(&g_fwup_sem);
  return ret;
}

static int fwup_client_download(enum fw_type_e fwtype, uint32_t fwsize,
                                void *data, uint32_t size)
{
  struct fwup_msg_s msg;

  msg.cmd = FWUP_DOWNLOAD;

  msg.u.dlparam.fwtype = fwtype;
  msg.u.dlparam.fwsize = fwsize;
  msg.u.dlparam.data = data;
  msg.u.dlparam.size = size;

  return mq_send(g_fwup_mqd, (const char*)&msg, sizeof(msg), 0);
}

static int fwup_client_update(void)
{
  struct fwup_msg_s msg;

  msg.cmd = FWUP_UPDATE;

  return mq_send(g_fwup_mqd, (const char*)&msg, sizeof(msg), 0);
}

static int fwup_client_suspend(void)
{
  struct fwup_msg_s msg;

  msg.cmd = FWUP_SUSPEND;

  return mq_send(g_fwup_mqd, (const char*)&msg, sizeof(msg), 0);
}

static int fwup_client_resume(void)
{
  struct fwup_msg_s msg;

  msg.cmd = FWUP_RESUME;

  return mq_send(g_fwup_mqd, (const char*)&msg, sizeof(msg), 0);
}

static int fwup_client_abort(void)
{
  struct fwup_msg_s msg;

  msg.cmd = FWUP_ABORT;

  return mq_send(g_fwup_mqd, (const char*)&msg, sizeof(msg), 0);
}

static int fwup_client_finalize(void)
{
  /* close message queue to send a message to manager */

  mq_close(g_fwup_mqd);

  /* finalize manager */

  fwup_finalize();

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct fwup_client_s *fwup_client_setup(void)
{
  struct fwup_client_s *client = get_client();

  /* initialize manager */

  fwup_initialize();

  /* open message queue to send a message to manager */

  g_fwup_mqd = mq_open(FWUP_MSGQ_NAME, O_WRONLY, 0666, NULL);

  sem_init(&g_fwup_sem, 0, 0);

  return client;
}

uint32_t fwup_client_getfreespace(void)
{
  return BKUP->fw_free_space;
}
