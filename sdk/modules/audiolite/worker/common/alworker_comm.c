/****************************************************************************
 * modules/audiolite/worker/mp3dec/al_comm_comm.c
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

#include <string.h>
#include <asmp/types.h>
#include <asmp/mpmq.h>
#include <asmp/mpshm.h>

#ifdef BUILD_TGT_SUPERVISOR
#  include <stdio.h>
#  include <asmp/mptask.h>
#endif

#include "almsgq_name.h"
#include "alworker_comm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MSGBUF_DEPTH_POW  (4)
#define MSGBUF_DEPTH      (1 << MSGBUF_DEPTH_POW)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct msg_s
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef BUILD_TGT_SUPERVISOR
static mptask_t g_mp3task = {0};
#endif

static mpmq_t g_mqsend = {0};
static mpmq_t g_mqrecv = {0};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct msg_s *get_send_msgbody(void)
{
  static struct msg_s msg[MSGBUF_DEPTH];
  static int msg_index = 0;

  struct msg_s *ret = &msg[msg_index];

  msg_index++;
  msg_index &= (MSGBUF_DEPTH - 1);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/** alworker_addr_convert() */

void *alworker_addr_convert(void *a)
{
  void *tmp = (void *)mpshm_virt2phys(NULL, a);
  return tmp == NULL ? a : tmp;
}

/** initialize_alworker() */

int initialize_alworker(char *dspfname)
{
  cpuid_t cid = 0;

#ifdef BUILD_TGT_SUPERVISOR
  if (mptask_init(&g_mp3task, dspfname) < 0)
    {
      return AL_COMM_ERR_WORKERINIT;
    }

  if (mptask_assign(&g_mp3task) < 0)
    {
      mptask_destroy(&g_mp3task, false, NULL);
      return AL_COMM_ERR_WORKERASSIGN;
    }

  cid = mptask_getcpuid(&g_mp3task);
#else
  (void)dspfname;
#endif

  /* Initialize MP message queue,
   * On the worker side, 3rd argument is ignored.
   */

  if (mpmq_init(&g_mqsend, AL_COMM_MQ_NAMESEND, cid) < 0)
    {
#ifdef BUILD_TGT_SUPERVISOR
      mptask_destroy(&g_mp3task, false, NULL);
#endif
      return AL_COMM_ERR_SENDMQCREATE;
    }

#ifdef BUILD_TGT_SUPERVISOR
  if (mptask_bindobj(&g_mp3task, &g_mqsend) < 0)
    {
      mptask_destroy(&g_mp3task, false, NULL);
      mpmq_destroy(&g_mqsend);
      return AL_COMM_ERR_SENDMQBIND;
    }
#endif

  if (mpmq_init(&g_mqrecv, AL_COMM_MQ_NAMERECV, cid) < 0)
    {
#ifdef BUILD_TGT_SUPERVISOR
      mptask_destroy(&g_mp3task, false, NULL);
#endif
      mpmq_destroy(&g_mqsend);
      return AL_COMM_ERR_RECVMQCREATE;
    }

#ifdef BUILD_TGT_SUPERVISOR
  if (mptask_bindobj(&g_mp3task, &g_mqrecv) < 0)
    {
      mptask_destroy(&g_mp3task, false, NULL);
      mpmq_destroy(&g_mqsend);
      mpmq_destroy(&g_mqrecv);
      return AL_COMM_ERR_SENDMQBIND;
    }

  if (mptask_exec(&g_mp3task) < 0)
    {
      mptask_destroy(&g_mp3task, false, NULL);
      mpmq_destroy(&g_mqsend);
      mpmq_destroy(&g_mqrecv);
      return AL_COMM_ERR_EXECWORKER;
    }
#endif

  return AL_COMM_ERR_SUCCESS;
}

int finalize_alworker(void)
{
#ifdef BUILD_TGT_SUPERVISOR
  mptask_destroy(&g_mp3task, false, NULL);
  mpmq_destroy(&g_mqsend);
  mpmq_destroy(&g_mqrecv);
#endif

  return 0;
}

/** al_receive_message() */

al_comm_msghdr_t al_receive_message(al_comm_msgopt_t *opt, int block)
{
  al_comm_msghdr_t hdr;
  uint32_t msgdata;
  int ret;
  struct msg_s *msg;

  hdr.u32 = AL_COMM_NO_MSG;

  if (block)
    {
      ret = mpmq_receive(&g_mqrecv, &msgdata);
    }
  else
    {
      ret = mpmq_tryreceive(&g_mqrecv, &msgdata);
    }

  if (ret >= 0)
    {
      msg = (struct msg_s *)msgdata;
      hdr.u32 = msg->hdr.u32;
      memcpy(opt, &msg->opt, sizeof(al_comm_msgopt_t));
    }

  return hdr;
}

/** al_send_message() */

int al_send_message(al_comm_msghdr_t hdr, al_comm_msgopt_t *opt)
{
  struct msg_s *msg = alworker_addr_convert(get_send_msgbody());

  msg->hdr.u32 = hdr.u32;
  memcpy(&msg->opt, opt, sizeof(al_comm_msgopt_t));

  return mpmq_send(&g_mqsend, msg->hdr.type, (uint32_t)msg);
}
