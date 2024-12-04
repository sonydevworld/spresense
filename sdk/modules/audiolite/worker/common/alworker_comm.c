/****************************************************************************
 * modules/audiolite/worker/common/alworker_comm.c
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

#ifndef BUILD_TGT_ASMPWORKER
#  include <asmp/mptask.h>
#endif

#include "almsgq_name.h"
#include "audiolite/alworker_comm.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct al_msg_s *get_send_msgbody(al_wtask_t *inst)
{
  struct al_msg_s *ret = &inst->msg[inst->msg_index];

  inst->msg_index++;
  inst->msg_index &= (AL_MSGBUF_DEPTH - 1);

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

int initialize_alworker(al_wtask_t *inst, const char *dspfname, bool is_spk)
{
#ifndef BUILD_TGT_ASMPWORKER
  int ret;
#endif
  key_t key = AL_COMM_MQ_NAMESEND;
  cpuid_t cid = 0;

  memset(inst, 0, sizeof(al_wtask_t));

#ifndef BUILD_TGT_ASMPWORKER
  if (is_spk)
    {
      ret = mptask_init_secure(&inst->wtask, dspfname);
      if (ret < 0)
        {
          return AL_COMM_ERR_WORKERINIT;
        }
    }
  else
    {
      ret = mptask_init(&inst->wtask, dspfname);
      if (ret < 0)
        {
          return AL_COMM_ERR_WORKERINIT;
        }
    }

  if (mptask_assign(&inst->wtask) < 0)
    {
      mptask_destroy(&inst->wtask, false, NULL);
      return AL_COMM_ERR_WORKERASSIGN;
    }

  cid = mptask_getcpuid(&inst->wtask);
#endif

  /* Initialize MP message queue,
   * If worker is used in spk format,
   * key and cid must be fixed values.
   */

#if defined(BUILD_TGT_ASMPWORKER) && defined(CONFIG_AUDIO_LITE_MP3DEC_SUBCORE_SPK)
      key = 0;
      cid = 2;
#endif

  if (mpmq_init(&inst->mqsend, key, cid) < 0)
    {
#ifndef BUILD_TGT_ASMPWORKER
      mptask_destroy(&inst->wtask, false, NULL);
#endif
      return AL_COMM_ERR_SENDMQCREATE;
    }

#ifndef BUILD_TGT_ASMPWORKER
  if (!is_spk)
    {
      if (mptask_bindobj(&inst->wtask, &inst->mqsend) < 0)
        {
          mptask_destroy(&inst->wtask, false, NULL);
          mpmq_destroy(&inst->mqsend);
          return AL_COMM_ERR_SENDMQBIND;
        }
    }
#endif

#ifndef BUILD_TGT_ASMPWORKER
  if (mptask_exec(&inst->wtask) < 0)
    {
      mptask_destroy(&inst->wtask, false, NULL);
      mpmq_destroy(&inst->mqsend);
      return AL_COMM_ERR_EXECWORKER;
    }
#endif

  return AL_COMM_ERR_SUCCESS;
}

int finalize_alworker(al_wtask_t *inst)
{
#ifndef BUILD_TGT_ASMPWORKER
  mpmq_destroy(&inst->mqsend);
  mptask_destroy(&inst->wtask, true, NULL);
#endif

  return 0;
}

/** al_receive_message() */

al_comm_msghdr_t al_receive_message(al_wtask_t *inst,
                                    al_comm_msgopt_t *opt, int block)
{
  al_comm_msghdr_t hdr;
  uint32_t msgdata;
  int ret;
  struct al_msg_s *msg;

  hdr.u32 = AL_COMM_NO_MSG;

  if (block)
    {
      ret = mpmq_receive(&inst->mqsend, &msgdata);
    }
  else
    {
      ret = mpmq_tryreceive(&inst->mqsend, &msgdata);
    }

  if (ret >= 0)
    {
      msg = (struct al_msg_s *)msgdata;
      hdr.u32 = msg->hdr.u32;
      memcpy(opt, &msg->opt, sizeof(al_comm_msgopt_t));
    }

  return hdr;
}

/** al_receive_messageto() */

al_comm_msghdr_t al_receive_messageto(al_wtask_t *inst,
                                      al_comm_msgopt_t *opt, int ms)
{
  al_comm_msghdr_t hdr;
  uint32_t msgdata;
  int ret;
  struct al_msg_s *msg;

  hdr.u32 = AL_COMM_NO_MSG;

  ret = mpmq_timedreceive(&inst->mqsend, &msgdata, ms);

  if (ret >= 0)
    {
      msg = (struct al_msg_s *)msgdata;
      hdr.u32 = msg->hdr.u32;
      memcpy(opt, &msg->opt, sizeof(al_comm_msgopt_t));
    }

  return hdr;
}

/** al_send_message() */

int al_send_message(al_wtask_t *inst,
                    al_comm_msghdr_t hdr, al_comm_msgopt_t *opt)
{
  struct al_msg_s *msg = alworker_addr_convert(get_send_msgbody(inst));

  msg->hdr.u32 = hdr.u32;
  memcpy(&msg->opt, opt, sizeof(al_comm_msgopt_t));

  while (mpmq_send(&inst->mqsend, msg->hdr.type, (uint32_t)msg) != OK);
  return OK;
}
