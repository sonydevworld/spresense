/****************************************************************************
 * modules/audiolite/src/base/al_workercmd.cxx
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

#include <audiolite/al_workercmd.h>
#include <audiolite/alworker_comm.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int send_iframe(al_wtask_t *wtask, void *frame, int sz,
                       bool eof)
{
  int ret;
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_FMEM;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODEMEM_INJECT;
  hdr.opt  = 0;

  opt.addr = (unsigned char *)alworker_addr_convert(frame);
  opt.size = sz;
  opt.eof = eof;

  ret = al_send_message(wtask, hdr, &opt);
  return ret;
}

static int send_oframe(al_wtask_t *wtask, void *frame, int sz)
{
  int ret;
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_OMEM;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODEMEM_INJECT;
  hdr.opt  = 0;

  opt.addr = (unsigned char *)alworker_addr_convert(frame);
  opt.size = sz;
  opt.eof = 0;

  ret = al_send_message(wtask, hdr, &opt);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int alworker_send_systemparam(al_wtask_t *wtask,
                              int chnum, int hz, int mode)
{
  int ret;
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_SYS;
  hdr.type = AL_COMM_MSGTYPE_SYNC;
  hdr.code = AL_COMM_MSGCODESYS_PARAM;
  hdr.opt  = 0;

  opt.chs  = chnum;
  opt.hz   = hz;
  opt.mode = mode;

  ret = al_send_message(wtask, hdr, &opt);
  return ret;
}

int alworker_send_startframe(al_wtask_t *wtask)
{
  int ret;
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_INST;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODEINST_START;
  hdr.opt  = 0;

  ret = al_send_message(wtask, hdr, &opt);
  return ret;
}

int alworker_send_instgain(al_wtask_t *wtask, float gain)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_INST;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODEINST_GAIN;
  hdr.opt  = 0;

  opt.gain = gain;

  return al_send_message(wtask, hdr, &opt);
}

int alworker_send_start(al_wtask_t *wtask, al_comm_msgopt_t *opts)
{
  int ret;
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_SYS;
  hdr.type = AL_COMM_MSGTYPE_SYNC;
  hdr.code = AL_COMM_MSGCODESYS_PLAY;
  hdr.opt  = 0;

  if (opts != NULL)
    {
      ret = al_send_message(wtask, hdr, opts);
    }
  else
    {
      memset(&opt, 0, sizeof(opt));
      ret = al_send_message(wtask, hdr, &opt);
    }

  return ret;
}

int alworker_send_stop(al_wtask_t *wtask)
{
  int ret;
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_SYS;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODESYS_STOP;
  hdr.opt  = 0;

  ret = al_send_message(wtask, hdr, &opt);
  return ret;
}

int alworker_send_term(al_wtask_t *wtask)
{
  int ret;
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_SYS;
  hdr.type = AL_COMM_MSGTYPE_SYNC;
  hdr.code = AL_COMM_MSGCODESYS_TERM;
  hdr.opt  = 0;

  ret = al_send_message(wtask, hdr, &opt);
  return ret;
}

int alworker_send_resp(al_wtask_t *wtask, al_comm_msghdr_t hdr, int ret)
{
  al_comm_msgopt_t opt;

  hdr.type = AL_COMM_MSGTYPE_RESP;
  hdr.opt = ret;

  ret = al_send_message(wtask, hdr, &opt);
  return ret;
}

int alworker_inject_omem(al_wtask_t *wtask, audiolite_mem *mem)
{
  return send_oframe(wtask, mem->get_data(), mem->get_fullsize());
}

int alworker_inject_imem(al_wtask_t *wtask, audiolite_mem *mem)
{
  return send_iframe(wtask, mem->get_data(),
                            mem->get_storedsize(),
                            mem->is_eof());
}

int alworker_send_usrcmd(al_wtask_t *wtask, al_comm_msgopt_t *opt)
{
  int ret;
  al_comm_msghdr_t hdr;

  hdr.grp  = AL_COMM_MESSAGE_USER;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = 0;
  hdr.opt  = 0;

  ret = al_send_message(wtask, hdr, opt);
  return ret;
}
