/****************************************************************************
 * modules/audiolite/worker/mp3dec/sprmp3_sendback.c
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

#include "sprmp3_sendback.h"
#include "audiolite/alworker_comm.h"
#include "minimp3_spresense.h"

#ifdef SPRMP3_DEBUG
#include "sprmp3_debug.h"
#endif

extern al_wtask_t g_mp3dec_task;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*** name: send_frameinfo */

int send_frameinfo(int id, mp3dec_frame_info_t *info)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_INST;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODEINST_INFO;
  hdr.opt  = id;

  opt.dec_chs = info->channels;
  opt.dec_hz = info->hz;
  opt.dec_layer = info->layer;
  opt.dec_kbps = info->bitrate_kbps;

  return al_send_message(&g_mp3dec_task, hdr, &opt);
}

/*** name: send_framedone */

int send_framedone(int id)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_INST;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODEINST_DONE;
  hdr.opt  = id;

#ifdef SPRMP3_DEBUG
  sprmp3_dprintf("[MSG] FrameDone:%d\n", id);
#endif

  return al_send_message(&g_mp3dec_task, hdr, &opt);
}

/*** name: send_errormsg */

int send_errormsg(int id, int errcode)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_SYS;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODESYS_ERR;
  hdr.opt  = id;

  opt.errcode = errcode;

#ifdef SPRMP3_DEBUG
  sprmp3_dprintf("[MSG] Error on %d err:%d\n", id, errcode);
#endif

  return al_send_message(&g_mp3dec_task, hdr, &opt);
}

/*** name: send_bootmsg */

int send_bootmsg(void *d)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_SYS;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODESYS_BOOT;
  hdr.opt  = AL_MP3DECWORKER_VERSION;
  opt.addr = (unsigned char *)alworker_addr_convert(d);

#ifdef SPRMP3_DEBUG
  sprmp3_dprintf("[MSG] BootUp\n");
#endif

  return al_send_message(&g_mp3dec_task, hdr, &opt);
}

int send_debug(unsigned char hdr_opt)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  hdr.grp  = AL_COMM_MESSAGE_SYS;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODESYS_DBG;
  hdr.opt  = hdr_opt;

#ifdef SPRMP3_DEBUG
  sprmp3_dprintf("[MSG] BootUp\n");
#endif

  return al_send_message(&g_mp3dec_task, hdr, &opt);
}

/*** name: release_framemem */

int release_framemem(int id, sprmp3_fmemqueue_t *queue)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  sprmp3_fmemcont_t *mem = (sprmp3_fmemcont_t *)sq_remfirst(&queue->queued);

  hdr.grp  = AL_COMM_MESSAGE_FMEM;
  hdr.type = AL_COMM_MSGTYPE_ASYNC;
  hdr.code = AL_COMM_MSGCODEMEM_RELEASE;
  hdr.opt  = (unsigned char)id;

  opt.addr = mem->addr;
  opt.size = mem->size;
  opt.eof = mem->eof ? 1 : 0;

  sq_addlast((sq_entry_t *)mem, &queue->free);
  queue->copied_ofst = 0;

#ifdef SPRMP3_DEBUG
  sprmp3_dprintf("[MSG] FMemFree:%d sz:%d %s\n",
                  id, mem->size, opt.eof ? "EOF" : "");
#endif

  return al_send_message(&g_mp3dec_task, hdr, &opt);
}

/*** name: deliver_outpcm */

int deliver_outpcm(sprmp3_outmemqueue_t *outq, int eof)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  sprmp3_outmemcont_t *mem =
                   (sprmp3_outmemcont_t *)sq_remfirst(&outq->queued);

  if (mem)
    {
      hdr.grp  = AL_COMM_MESSAGE_OMEM;
      hdr.type = AL_COMM_MSGTYPE_ASYNC;
      hdr.code = AL_COMM_MSGCODEMEM_RELEASE;
      hdr.opt  = AL_COMM_MSGCODEERR_OK;

      opt.addr = mem->addr;
      opt.size = (outq->mode == SPRMP3_MODE_JUSTDECODE) ?
                 outq->filled_size : mem->size;
      opt.eof = eof;

      sq_addlast((sq_entry_t *)mem, &outq->free);
      outq->done = 0;
      outq->filled_size = 0;

#ifdef SPRMP3_DEBUG
      sprmp3_dprintf("[MSG] PCMmemFree sz:%d\n", opt.size);
#endif

      return al_send_message(&g_mp3dec_task, hdr, &opt);
    }

  return ERROR;
}
