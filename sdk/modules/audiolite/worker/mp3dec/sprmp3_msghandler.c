/****************************************************************************
 * modules/audiolite/worker/mp3dec/sprmp3_msghandler.c
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

#include <string.h>
#include <nuttx/queue.h>

#include "minimp3_spresense.h"
#include "audiolite/alworker_comm.h"

#include "sprmp3_msghandler.h"
#include "sprmp3_sendback.h"

#include "sprmp3_debug.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

al_wtask_t g_mp3dec_task;

/****************************************************************************
 * Private functions
 ****************************************************************************/

/*** name: release_all_mem */

static void release_all_mem(sprmp3_sys_t *sys)
{
  int i;

  while (sq_peek(&sys->outqueue.queued) != NULL)
    {
      deliver_outpcm(&sys->outqueue, 0);
    }

  for (i = 0; i < SPRMP3_MAX_INSTANCE; i++)
    {
      reset_instance(&sys->insts[i]);
    }
}

/*** name: handle_system_msg */

static void handle_system_msg(sprmp3_sys_t *sys,
                           al_comm_msghdr_t hdr,
                           al_comm_msgopt_t *opt)
{
  unsigned char retcode = AL_COMM_MSGCODEERR_OK;

  switch (hdr.code)
    {
      case AL_COMM_MSGCODESYS_STOP:
        if (sys->system_state == SPRMP3_SYSSTATE_PLAY ||
            sys->system_state == SPRMP3_SYSSTATE_RESUMING ||
            sys->system_state == SPRMP3_SYSSTATE_PAUSING)
          {
            sys->fade_counter = 0;
            sys->system_state = SPRMP3_SYSSTATE_STOPPING;
            sys->sys_gain = 1.f;
          }
        else if (sys->system_state == SPRMP3_SYSSTATE_PAUSE)
          {
            sys->system_state = SPRMP3_SYSSTATE_STOP;
          }
        break;

      case AL_COMM_MSGCODESYS_PAUSE:
        if (sys->system_state == SPRMP3_SYSSTATE_PLAY)
          {
            sys->sys_gain = 1.f;
            sys->system_state = SPRMP3_SYSSTATE_PAUSING;
            sys->fade_counter = 0;
          }
        break;

      case AL_COMM_MSGCODESYS_PLAY:
        if (sys->system_state == SPRMP3_SYSSTATE_STOP)
          {
             sys->sys_gain = 1.f;
             sys->system_state = SPRMP3_SYSSTATE_PLAY;
          }
        else if (sys->system_state == SPRMP3_SYSSTATE_PAUSE)
          {
             sys->sys_gain = 0.f;
             sys->system_state = SPRMP3_SYSSTATE_RESUMING;
          }
        break;

      case AL_COMM_MSGCODESYS_PARAM:
        sys->outqueue.chs   = opt->chs;
        sys->outqueue.hz    = opt->hz;
        sys->outqueue.mode  = opt->mode;
        break;

      case AL_COMM_MSGCODESYS_TERM:

        sprmp3_dprintf("[RCV] Terminate\n");
        sys->system_state = SPRMP3_SYSSTATE_TERM;
        release_all_mem(sys);
        break;

      case AL_COMM_MSGCODESYS_DBG:
      default:

        /* do nothing now */

        retcode = AL_COMM_MSGCODEERR_UNKNOWN;
        break;
    }

  if (hdr.type == AL_COMM_MSGTYPE_SYNC)
    {
      hdr.opt = retcode;
      al_send_message(&g_mp3dec_task, hdr, opt);
    }
}

/*** name: handle_outpcm_msg */

static void handle_outpcm_msg(sprmp3_sys_t *sys,
                           al_comm_msghdr_t hdr,
                           al_comm_msgopt_t *opt)
{
  unsigned char ret = AL_COMM_MSGCODEERR_OK;
  sprmp3_outmemcont_t *memblk;

  if (hdr.code == AL_COMM_MSGCODEMEM_INJECT)
    {
      if (opt->addr && opt->size > 0)
        {
          memblk = (sprmp3_outmemcont_t *)sq_remfirst(&sys->outqueue.free);
          if (memblk)
            {
              memblk->addr = opt->addr;
              memblk->size = opt->size;
              memset(memblk->addr, 0, memblk->size);
              sq_addlast((sq_entry_t *)memblk, &sys->outqueue.queued);
            }
          else
            {
              ret = AL_COMM_MSGCODEERR_OVFLOW;
            }
        }
      else
        {
          ret = AL_COMM_MSGCODEERR_INVALIDADDR;
        }
    }
  else
    {
      ret = AL_COMM_MSGCODEERR_UNKNOWN;
    }

  if (ret != AL_COMM_MSGCODEERR_OK)
    {
      hdr.opt = ret;
      al_send_message(&g_mp3dec_task, hdr, opt);
    }
}

/*** name: handle_framemem_msg */

static void handle_framemem_msg(sprmp3_sys_t *sys,
                                al_comm_msghdr_t hdr,
                                al_comm_msgopt_t *opt)
{
  unsigned char ret = AL_COMM_MSGCODEERR_OK;
  sprmp3_t *inst;
  sprmp3_fmemcont_t *memblk;

  if (hdr.code == AL_COMM_MSGCODEMEM_INJECT)
    {
      if (hdr.opt < SPRMP3_MAX_INSTANCE)
        {
          inst = &sys->insts[hdr.opt];

          if (opt->addr && opt->size >= 0)
            {
              memblk = (sprmp3_fmemcont_t *)sq_remfirst(&inst->fqueue.free);
              if (memblk)
                {
                  memblk->addr = opt->addr;
                  memblk->size = opt->size;
                  memblk->eof  = opt->eof;
                  sq_addlast((sq_entry_t *)memblk, &inst->fqueue.queued);
                }
              else
                {
                  ret = AL_COMM_MSGCODEERR_OVFLOW;
                }
            }
          else
            {
              ret = AL_COMM_MSGCODEERR_INVALIDADDR;
            }
        }
      else
        {
          ret = AL_COMM_MSGCODEERR_INVALIDINST;
        }
    }
  else
    {
      ret = AL_COMM_MSGCODEERR_UNKNOWN;
    }

  if (ret != AL_COMM_MSGCODEERR_OK)
    {
      hdr.opt = ret;
      al_send_message(&g_mp3dec_task, hdr, opt);
    }
}

/*** name: handle_instance_msg */

static void handle_instance_msg(sprmp3_sys_t *sys,
                                al_comm_msghdr_t hdr,
                                al_comm_msgopt_t *opt)
{
  unsigned char ret = AL_COMM_MSGCODEERR_OK;
  sprmp3_t *inst;

  if (hdr.opt < SPRMP3_MAX_INSTANCE)
    {
      inst = &sys->insts[hdr.opt];

      switch (hdr.code)
        {
          case AL_COMM_MSGCODEINST_START:
            if (inst->state == SPRMP3_STATE_END)
              {
                inst->state = SPRMP3_STATE_INIT;
              }
            break;

          case AL_COMM_MSGCODEINST_STOP:
            hdr.opt = sq_count(&inst->fqueue.queued);
            al_send_message(&g_mp3dec_task, hdr, opt);
            reset_instance(inst);
            break;

          case AL_COMM_MSGCODEINST_GAIN:
            inst->gain = opt->gain;
            break;

          default:
            ret = AL_COMM_MSGCODEERR_UNKNOWN;
            break;
        }
    }
  else
    {
      ret = AL_COMM_MSGCODEERR_INVALIDINST;
    }

  if (ret != AL_COMM_MSGCODEERR_OK)
    {
      opt->errcode = ret;
      al_send_message(&g_mp3dec_task, hdr, opt);
    }
}

/*** name: handle_message */

static void handle_message(sprmp3_sys_t *sys,
                           al_comm_msghdr_t hdr,
                           al_comm_msgopt_t *opt)
{
  sprmp3_dprintf("[mp3dec] GRP:%d TYP:%d COD:%d OPT:%d\n",
                 hdr.grp, hdr.type, hdr.code, hdr.opt);
  switch (hdr.grp)
    {
      case AL_COMM_MESSAGE_SYS:
        handle_system_msg(sys, hdr, opt);
        break;

      case AL_COMM_MESSAGE_FMEM:
        handle_framemem_msg(sys, hdr, opt);
        break;

      case AL_COMM_MESSAGE_OMEM:
        handle_outpcm_msg(sys, hdr, opt);
        break;

      case AL_COMM_MESSAGE_INST:
        handle_instance_msg(sys, hdr, opt);
        break;

      default:
        hdr.opt = AL_COMM_MSGCODEERR_UNKNOWN;
        al_send_message(&g_mp3dec_task, hdr, opt);
        break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*** name: sprmp3_initmsgloop */

int sprmp3_initmsgloop(void)
{
  return initialize_alworker(&g_mp3dec_task, "", false);
}

/*** name: sprmp3_pollmessage */

void sprmp3_pollmessage(sprmp3_sys_t *sys, int block)
{
  al_comm_msghdr_t hdr;
  al_comm_msgopt_t opt;

  do
    {
      hdr = al_receive_message(&g_mp3dec_task, &opt, block);
      if (hdr.u32 != AL_COMM_NO_MSG)
        {
          handle_message(sys, hdr, &opt);
        }
    }
  while (!block && hdr.u32 != AL_COMM_NO_MSG);
}

/*** name: reset_instance */

void reset_instance(sprmp3_t *inst)
{
  while (sq_peek(&inst->fqueue.queued) != NULL)
    {
      release_framemem(inst->id, &inst->fqueue);
    }

  inst->fqueue.copied_ofst = 0;
  inst->fcache.fillsize = 0;
  inst->fcache.usedsize = 0;
  inst->fcache.eof = 0;

  inst->pcmcache.decsize = 0;
  inst->resamcache.decsize = 0;

  inst->tgtcache.size = 0;
  inst->tgtcache.decsize = 0;
  inst->tgtcache.remofst = 0;
  inst->tgtcache.addr = NULL;

  inst->omem_wofst = 0;

  inst->gain = 1.f;
  inst->request = 0;

  inst->frame_info.channels = 0;
  inst->frame_info.hz = 0;
  inst->frame_info.layer = 0;
  inst->frame_info.bitrate_kbps = 0;
  inst->frame_info.frame_bytes = 0;

  inst->state = SPRMP3_STATE_END;
}
