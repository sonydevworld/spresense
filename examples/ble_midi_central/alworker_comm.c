/****************************************************************************
 * modules/audiolite/worker/mp3dec/midi_comm_comm.c
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

#include "synth_worker_comm.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct midi_msg_s *get_send_msgbody(midi_wtask_t *inst)
{
  struct midi_msg_s *ret = &inst->msg[inst->msg_index];

  inst->msg_index++;
  inst->msg_index &= (MIDI_MSGBUF_DEPTH - 1);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/** alworker_addr_convert() */

void *midiworker_addr_convert(void *a)
{
  void *tmp = (void *)mpshm_virt2phys(NULL, a);
  return tmp == NULL ? a : tmp;
}

/** initialize_alworker() */

int initialize_midiworker(midi_wtask_t *inst, const char *dspfname, bool is_spk)
{
#ifndef BUILD_TGT_ASMPWORKER
  int ret;
#endif
  key_t key = MIDI_COMM_MQ_NAMESEND;
  cpuid_t cid = 0;

  memset(inst, 0, sizeof(midi_wtask_t));

#ifndef BUILD_TGT_ASMPWORKER
  if (is_spk)
    {
      ret = mptask_init_secure(&inst->wtask, dspfname);
      if (ret < 0)
        {
          return MIDI_COMM_ERR_WORKERINIT;
        }
    }
  else
    {
      ret = mptask_init(&inst->wtask, dspfname);
      if (ret < 0)
        {
          return MIDI_COMM_ERR_WORKERINIT;
        }
    }

  if (mptask_assign(&inst->wtask) < 0)
    {
      mptask_destroy(&inst->wtask, false, NULL);
      return MIDI_COMM_ERR_WORKERASSIGN;
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
      return MIDI_COMM_ERR_SENDMQCREATE;
    }

#ifndef BUILD_TGT_ASMPWORKER
  if (!is_spk)
    {
      if (mptask_bindobj(&inst->wtask, &inst->mqsend) < 0)
        {
          mptask_destroy(&inst->wtask, false, NULL);
          mpmq_destroy(&inst->mqsend);
          return MIDI_COMM_ERR_SENDMQBIND;
        }
    }
#endif

#ifndef BUILD_TGT_ASMPWORKER
  if (mptask_exec(&inst->wtask) < 0)
    {
      mptask_destroy(&inst->wtask, false, NULL);
      mpmq_destroy(&inst->mqsend);
      return MIDI_COMM_ERR_EXECWORKER;
    }
#endif

  return MIDI_COMM_ERR_SUCCESS;
}

int finalize_midiworker(midi_wtask_t *inst)
{
#ifndef BUILD_TGT_ASMPWORKER
  mpmq_destroy(&inst->mqsend);
  mptask_destroy(&inst->wtask, true, NULL);
#endif

  return 0;
}

/** midi_receive_message() */

midi_comm_msghdr_t midi_receive_message(midi_wtask_t *inst,
                                    midi_comm_msgopt_t *opt, int block)
{
  midi_comm_msghdr_t hdr;
  uint32_t msgdata;
  int ret;
  struct midi_msg_s *msg;

  hdr.u32 = MIDI_COMM_NO_MSG;

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
      msg = (struct midi_msg_s *)msgdata;
      hdr.u32 = msg->hdr.u32;
      memcpy(opt, &msg->opt, sizeof(midi_comm_msgopt_t));
    }

  return hdr;
}

/** midi_receive_messageto() */

midi_comm_msghdr_t midi_receive_messageto(midi_wtask_t *inst,
                                      midi_comm_msgopt_t *opt, int ms)
{
  midi_comm_msghdr_t hdr;
  uint32_t msgdata;
  int ret;
  struct midi_msg_s *msg;

  hdr.u32 = MIDI_COMM_NO_MSG;

  ret = mpmq_timedreceive(&inst->mqsend, &msgdata, ms);

  if (ret >= 0)
    {
      msg = (struct midi_msg_s *)msgdata;
      hdr.u32 = msg->hdr.u32;
      memcpy(opt, &msg->opt, sizeof(midi_comm_msgopt_t));
    }

  return hdr;
}

/** midi_send_message() */

int midi_send_message(midi_wtask_t *inst,
                    midi_comm_msghdr_t hdr, midi_comm_msgopt_t *opt)
{
  struct midi_msg_s *msg = midiworker_addr_convert(get_send_msgbody(inst));

  msg->hdr.u32 = hdr.u32;
  memcpy(&msg->opt, opt, sizeof(midi_comm_msgopt_t));

  return mpmq_send(&inst->mqsend, msg->hdr.type, (uint32_t)msg);
}
