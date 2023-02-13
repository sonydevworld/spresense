/****************************************************************************
 * modules/audiolite/worker/mp3dec/mp3dec_main.c
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

#include "mp3dec_main.h"
#include "alworker_comm.h"
#include "minimp3_spresense.h"
#include "sprmp3_msghandler.h"
#include "sprmp3_sendback.h"

#ifdef SPRMP3_DEBUG
#include "sprmp3_debug.h"
#endif

#ifdef ENABME_PERFORMANCE
#include <stdio.h>
#include "perfmon.h"
#endif

/****************************************************************************
 * Privete Types
 ****************************************************************************/

struct state_proc_s
{
  int (*proc)(sprmp3_t *inst, sprmp3_outmemqueue_t *outq);
  int playing;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int exec_endstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq);
static int exec_initstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq);
static int exec_readystate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq);
static int exec_decodestate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq);
static int exec_fillupstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq);
static int exec_fillremstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq);
static int exec_waitinstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq);
static int exec_winremstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq);
static int exec_endingstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq);

/****************************************************************************
 * Privete Data
 ****************************************************************************/

static struct state_proc_s state_procs[] =
{
  [SPRMP3_STATE_END]          = {exec_endstate,     0},
  [SPRMP3_STATE_INIT]         = {exec_initstate,    0},
  [SPRMP3_STATE_READY]        = {exec_readystate,   0},
  [SPRMP3_STATE_DECODE]       = {exec_decodestate,  1},
  [SPRMP3_STATE_FILLUP]       = {exec_fillupstate,  1},
  [SPRMP3_STATE_FILLUPREMAIN] = {exec_fillremstate, 1},
  [SPRMP3_STATE_WAITIN]       = {exec_waitinstate,  1},
  [SPRMP3_STATE_WAITINREMAIN] = {exec_winremstate,  1},
  [SPRMP3_STATE_ENDING]       = {exec_endingstate,  1}
};

static sprmp3_sys_t g_sys;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*** name: init_system */

static void init_system(sprmp3_sys_t *sys)
{
  int i;
  int j;
  sprmp3_t *inst;
  inst = &sys->insts[0];

  for (i = 0; i < SPRMP3_MAX_INSTANCE; i++)
    {
      inst->id = i;
      inst->fcache.size = sizeof(inst->fcache.addr);
      inst->fcache.fillsize = 0;
      inst->fcache.usedsize = 0;
      inst->fcache.eof = false;

      inst->pcmcache.size = sizeof(inst->pcmcache.addr);
      inst->pcmcache.decsize = 0;

      inst->resamcache.size = sizeof(inst->resamcache.addr);
      inst->resamcache.decsize = 0;

      inst->tgtcache.size = 0;
      inst->tgtcache.decsize = 0;
      inst->tgtcache.remofst = 0;
      inst->tgtcache.addr = NULL;

      sq_init(&inst->fqueue.queued);
      sq_init(&inst->fqueue.free);
      inst->fqueue.copied_ofst = 0;
      for (j = 0; j < SPRMP3_FRAMEMEM_QSIZE; j++)
        {
          inst->fqueue.inst[j].addr = NULL;
          inst->fqueue.inst[j].size = 0;
          inst->fqueue.inst[j].eof = false;
          sq_addlast((sq_entry_t *)&inst->fqueue.inst[j],
                     &inst->fqueue.free);
        }

      inst->state = SPRMP3_STATE_END;
      inst->omem_wofst = 0;
      inst->gain = 1.f;
      inst->request = 0;

      inst->frame_info.channels = 0;
      inst->frame_info.hz = 0;
      inst->frame_info.layer = 0;
      inst->frame_info.bitrate_kbps = 0;
      inst->frame_info.frame_bytes = 0;

      inst++;
    }

  sq_init(&sys->outqueue.queued);
  sq_init(&sys->outqueue.free);
  sys->outqueue.chs = 0;
  sys->outqueue.hz = 0;
  sys->outqueue.mode = SPRMP3_MODE_STREAM;
  sys->outqueue.done = 0;
  sys->outqueue.filled_size = 0;
  for (i = 0; i < SPRMP3_OUTMEM_QSIZE; i++)
    {
      sys->outqueue.inst[i].addr = NULL;
      sys->outqueue.inst[i].size = 0;
      sq_addlast((sq_entry_t *)&sys->outqueue.inst[i], &sys->outqueue.free);
    }

  sys->sys_gain = 1.f;
  sys->system_state = SPRMP3_SYSSTATE_STOP;
}

/*** name: is_decode_done */

static bool is_decode_done(sprmp3_t *inst)
{
  return inst->fcache.eof &&
         (inst->fcache.usedsize == inst->fcache.fillsize) &&
         (inst->tgtcache.decsize == inst->tgtcache.remofst);
}

/*** name: exec_endstate */

static int exec_endstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq)
{
  /* Changing state from END to INIT,
   * is triggered by receiving command.
   */

  (void)inst;
  (void)outq;

  return SPRMP3_STATE_END;
}

/*** name: exec_endingstate */

static int exec_endingstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq)
{
  int ret = SPRMP3_STATE_ENDING;
  unsigned int mask = (1 << inst->id);

  if (!(outq->done & mask))
    {
      if (is_decode_done(inst))
        {
          send_framedone(inst->id);
          reset_instance(inst);
          ret = SPRMP3_STATE_END;
        }
    }

  return ret;
}

/*** name: exec_initstate */

static int exec_initstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq)
{
  int ret = SPRMP3_STATE_INIT;

  if (sq_peek(&inst->fqueue.queued) != NULL &&
      sq_peek(&outq->queued) != NULL)
    {
      ret = SPRMP3_STATE_READY;
    }

  return ret;
}

/*** name: fill_tagsize */

static int fill_tagsize(sprmp3_t *inst)
{
  int ret = 0;
  sprmp3_fmemcont_t *mem =
         (sprmp3_fmemcont_t *)sq_peek(&inst->fqueue.queued);

  if (mem)
    {
      if (mem->size >= MINIMP3_ID3_DETECT_SIZE)
        {
          memcpy(&inst->fcache.addr[inst->fcache.fillsize],
                 mem->addr, MINIMP3_ID3_DETECT_SIZE);
          inst->fcache.fillsize += MINIMP3_ID3_DETECT_SIZE;
          ret = MINIMP3_ID3_DETECT_SIZE;
          inst->fqueue.copied_ofst = MINIMP3_ID3_DETECT_SIZE;
        }
    }

  return ret;
}

/*** name: seek_size */

static size_t seek_size(sprmp3_t *inst, int size)
{
  size_t ret = 0;
  sprmp3_fmemcont_t *mem =
         (sprmp3_fmemcont_t *)sq_peek(&inst->fqueue.queued);

  if (mem)
    {
      if (mem->size >= (size_t)size)
        {
          /* Just skip size bytes */

          inst->fqueue.copied_ofst = size;
          ret = (size_t)size;
        }
    }

  return ret;
}

/*** name: refill_fullspace */

static void refill_fullspace(sprmp3_t *inst,
                             sprmp3_fmemqueue_t *queue,
                             sprmp3_fmemcont_t *mem, int refillsize)
{
  memcpy(&inst->fcache.addr[inst->fcache.fillsize],
         mem->addr + queue->copied_ofst,
         refillsize);

  queue->copied_ofst += refillsize;
  inst->fcache.fillsize += refillsize;

  if (queue->copied_ofst >= mem->size)
    {
      /* Is just finished to use */

      if (mem->eof)
        {
          inst->fcache.eof = true;
          mp3dec_skip_id3v1(&inst->fcache.addr[0],
                            (size_t *)&inst->fcache.fillsize);
        }

      release_framemem(inst->id, queue);
    }
}

/*** name: refill_framecache */

static int refill_framecache(sprmp3_t *inst)
{
  int empty_space;
  sprmp3_fmemqueue_t *queue;
  sprmp3_fmemcont_t *mem;
  int contain_size;

  int ret = SPRMP3_STATE_DECODE;

  empty_space = inst->fcache.size -
                inst->fcache.fillsize +
                inst->fcache.usedsize;

  if (!inst->fcache.eof && empty_space > 0)
    {
      if (inst->fcache.usedsize > 0)
        {
          /* Shift remain frame data to top of frame cache */

          inst->fcache.fillsize -= inst->fcache.usedsize;
          memmove(&inst->fcache.addr[0],
                  &inst->fcache.addr[inst->fcache.usedsize],
                  inst->fcache.fillsize);
          inst->fcache.usedsize = 0;
        }

      queue = &inst->fqueue;
      mem = (sprmp3_fmemcont_t *)sq_peek(&queue->queued);
      if (mem)
        {
          contain_size = mem->size - queue->copied_ofst;
          if (contain_size < empty_space)
            {
              /* Under flow case */

              memcpy(&inst->fcache.addr[inst->fcache.fillsize],
                     mem->addr + queue->copied_ofst,
                     contain_size);

              inst->fcache.fillsize += contain_size;

              ret = SPRMP3_STATE_WAITINREMAIN;

              if (mem->eof)
                {
                  inst->fcache.eof = true;
                  mp3dec_skip_id3v1(&inst->fcache.addr[0],
                                    (size_t *)&inst->fcache.fillsize);
                  ret = SPRMP3_STATE_DECODE;
                }

              release_framemem(inst->id, queue);

              mem = (sprmp3_fmemcont_t *)sq_peek(&queue->queued);
              if (!inst->fcache.eof && mem)
                {
                  /* If next mem is already exists */

                  empty_space = inst->fcache.size -
                                inst->fcache.fillsize +
                                inst->fcache.usedsize;

                  refill_fullspace(inst, queue, mem, empty_space);

                  ret = SPRMP3_STATE_DECODE;
                }
            }
          else
            {
              /* Enough space Case */

              refill_fullspace(inst, queue, mem, empty_space);
            }
        }
      else
        {
          ret = SPRMP3_STATE_WAITINREMAIN;
        }
    }

  return ret;
}

/*** name: initialize_framecache */

static int initialize_framecache(sprmp3_t *inst)
{
  size_t id3v2size;

  mp3dec_init(&inst->core);

  inst->fcache.fillsize = 0;
  inst->fcache.usedsize = 0;

  /* Read and check ID3 tag header */

  fill_tagsize(inst);
  memset(&inst->frame_info, 0, sizeof(inst->frame_info));

  if (inst->fcache.fillsize > MINIMP3_ID3_DETECT_SIZE)
    {
      return ERROR;
    }

  if (MINIMP3_ID3_DETECT_SIZE != inst->fcache.fillsize)
    {
      return ERROR;
    }

  id3v2size = mp3dec_skip_id3v2(inst->fcache.addr,
                                inst->fcache.fillsize);

  if (id3v2size)
    {
      /* If tag header exists */

      if (seek_size(inst, id3v2size) != id3v2size)
        {
          return ERROR;
        }

      /* Avoid read data and fill again */

      inst->fcache.fillsize = 0;
    }

  /* Fill empty space on No tag header case */

  refill_framecache(inst);

  if (inst->fcache.eof)
    {
      /* If the stream is very short and it is the end */

      mp3dec_skip_id3v1(&inst->fcache.addr[0],
                        (size_t *)&inst->fcache.fillsize);
    }

  return OK;
}

/*** name: exec_readystate */

static int exec_readystate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq)
{
  (void)outq;

  if (initialize_framecache(inst) == OK)
    {
      return SPRMP3_STATE_DECODE;
    }
  else
    {
      send_errormsg(inst->id, AL_COMM_MSGCODEERR_TOOSHORT);
      return SPRMP3_STATE_ERROR;
    }
}

/*** name: mix_data */

static int mix_data(sprmp3_t *inst, unsigned char *omem,  int och,
                    sprmp3_targetcache_t *pcm, int ich, int mix_bytes)
{
  (void)och;
  (void)ich;
  memcpy(&omem[inst->omem_wofst], &pcm->addr[pcm->remofst], mix_bytes);
  inst->omem_wofst += mix_bytes;
  pcm->remofst += mix_bytes;
  return OK;
}

/*** name: write_outmem */

static int write_outmem(sprmp3_t *inst, sprmp3_outmemqueue_t *outq, int st)
{
  int remain_space;
  int write_size;
  int has_remain = 0;

  sprmp3_outmemcont_t *mem = (sprmp3_outmemcont_t *)sq_peek(&outq->queued);

  if (mem)
    {
      remain_space = mem->size - inst->omem_wofst;
      write_size = (inst->tgtcache.decsize - inst->tgtcache.remofst) *
                    outq->chs / inst->frame_info.channels;

      if (write_size > remain_space)
        {
          write_size = remain_space;
          has_remain = 1;
        }

      mix_data(inst, mem->addr, outq->chs,
               &inst->tgtcache,
               inst->frame_info.channels,
               write_size);

      if (outq->filled_size < inst->omem_wofst)
        {
          outq->filled_size = inst->omem_wofst;
        }

      if (inst->omem_wofst >= mem->size)
        {
          outq->done |= SPRMP3_OUTDONE(inst);
          st = SPRMP3_STATE_FILLUP;
          inst->omem_wofst = 0;

          if (has_remain)
            {
              st = SPRMP3_STATE_FILLUPREMAIN;
            }
        }
    }
  else
    {
      st = SPRMP3_STATE_FILLUP;
    }

  return st;
}

/*** name: duplicate_sample */

static void duplicate_sample(int16_t *src, int samples, int16_t *dst)
{
  int i;

  for (i = 0; i < samples; i++)
    {
      *dst++ = *src;
      *dst++ = *src++;
    }
}

/*** name: pick_lch_sample */

static void pick_lch_sample(int16_t *src, int samples, int16_t *dst)
{
  int i;

  for (i = 0; i < samples; i++)
    {
      *dst++ = *src;
      src += 2;
    }
}

/*** name: split_plane_sample */

static void split_plane_sample(int16_t *src, int samples,
                              int16_t *dstl, int16_t *dstr)
{
  int i;

  for (i = 0; i < samples; i++)
    {
      *dstl++ = *src++;
      *dstr++ = *src++;
    }
}

/*** name: embed_lr_sample */

static void embed_lr_sample(int samples, int16_t *srcl, int16_t *srcr,
                            int16_t *dst)
{
  int i;

  for (i = 0; i < samples; i++)
    {
      *dst++ = *srcl++;
      *dst++ = *srcr++;
    }
}

/*** name: resample_onech */

static int resample_onech(SpeexResamplerState *st,
                          int16_t *src, unsigned int srcsamples,
                          int16_t *dst, unsigned int dstsamples)
{
  unsigned int inl;
  unsigned int outl;
  unsigned int indone = 0;
  unsigned int outdone = 0;

  do
    {
      inl  = srcsamples - indone;
      outl = dstsamples - outdone;
      speex_resampler_process_int(st, 0,
                                  &src[indone], &inl,
                                  &dst[outdone], &outl);
      outdone += outl;
      indone  += inl;
    }
  while (indone < srcsamples);

  return outdone;
}

/*** name: resampling */

static int resampling(sprmp3_t *inst, int chs, int hz)
{
  int resam_samples;

  if (hz == 0 ||
      (inst->frame_info.channels == chs &&
       inst->frame_info.hz == hz))
    {
      inst->tgtcache.size = inst->pcmcache.size;
      inst->tgtcache.decsize = inst->pcmcache.decsize;
      inst->tgtcache.remofst = 0;
      inst->tgtcache.addr = inst->pcmcache.addr;
    }
  else if (inst->frame_info.hz == hz) /* No need resampling */
    {
      inst->tgtcache.size = inst->resamcache.size;
      inst->tgtcache.remofst = 0;
      inst->tgtcache.addr = inst->resamcache.addr;

      switch (inst->frame_info.channels)
        {
          case 1: /* case of 1ch input 2ch output */
            duplicate_sample((int16_t *)inst->pcmcache.addr,
                             inst->pcmcache.decsize / 2,
                             (int16_t *)inst->resamcache.addr);
            inst->tgtcache.decsize = inst->pcmcache.decsize * 2;
            break;

          case 2: /* case of 2ch input 1ch output */
            pick_lch_sample((int16_t *)inst->pcmcache.addr,
                            inst->pcmcache.decsize / 4,
                            (int16_t *)inst->resamcache.addr);
            inst->tgtcache.decsize = inst->pcmcache.decsize / 2;
            break;

          default:
            return -1;
        }
    }
  else /* case of re-sampling */
    {
      if (inst->frame_info.channels == 1)
        {
          resam_samples =
            resample_onech(&inst->resampler.lch,
                           (int16_t *)inst->pcmcache.addr,
                           inst->pcmcache.decsize / 2,
                           (int16_t *)inst->resampler.lch_out,
                           sizeof(inst->resampler.lch_out) / 2);

          if (chs == 1)
            {
              inst->tgtcache.size = sizeof(inst->resampler.lch_out);
              inst->tgtcache.decsize = resam_samples * sizeof(int16_t);
              inst->tgtcache.remofst = 0;
              inst->tgtcache.addr = inst->resampler.lch_out;
            }
          else
            {
              duplicate_sample((int16_t *)inst->resampler.lch_out,
                               resam_samples,
                               (int16_t *)inst->resamcache.addr);

              inst->tgtcache.size = inst->resamcache.size;
              inst->tgtcache.decsize = resam_samples * 2 * sizeof(int16_t);
              inst->tgtcache.remofst = 0;
              inst->tgtcache.addr = inst->resamcache.addr;
            }
        }
      else /* case of 2ch inputs */
        {
          split_plane_sample((int16_t *)inst->pcmcache.addr,
                             inst->pcmcache.decsize / 4,
                             (int16_t *)inst->resampler.lch_in,
                             (int16_t *)inst->resampler.rch_in);

          resam_samples =
            resample_onech(&inst->resampler.lch,
                           (int16_t *)inst->resampler.lch_in,
                           inst->pcmcache.decsize / 4,
                           (int16_t *)inst->resampler.lch_out,
                           sizeof(inst->resampler.lch_out) / 2);

          if (chs == 1)
            {
              inst->tgtcache.size = sizeof(inst->resampler.lch_out);
              inst->tgtcache.decsize = resam_samples * sizeof(int16_t);
              inst->tgtcache.remofst = 0;
              inst->tgtcache.addr = inst->resampler.lch_out;
            }
          else
            {
              /* result of resampled size is the same as l-ch */

              resam_samples =
                resample_onech(&inst->resampler.rch,
                               (int16_t *)inst->resampler.rch_in,
                               inst->pcmcache.decsize / 4,
                               (int16_t *)inst->resampler.rch_out,
                               sizeof(inst->resampler.rch_out) / 2);

              embed_lr_sample(resam_samples,
                              (int16_t *)inst->resampler.lch_out,
                              (int16_t *)inst->resampler.rch_out,
                              (int16_t *)inst->resamcache.addr);

              inst->tgtcache.size = inst->resamcache.size;
              inst->tgtcache.decsize = resam_samples * sizeof(int16_t) * 2;
              inst->tgtcache.remofst = 0;
              inst->tgtcache.addr = inst->resamcache.addr;
            }
        }
    }

  return 0;
}

/*** name: exec_decodestate */

static int exec_decodestate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq)
{
  int frame_size;
  int frame_ofst;
  int dummy;
  const unsigned char *hdr;
  int samples;
  int next_state;
  mp3dec_frame_info_t info;

#ifdef ENABME_PERFORMANCE
  struct timeval dec_time;
  measure_interval(&dec_time);
#endif

  frame_size = 0;
  frame_ofst = mp3d_find_frame(&inst->fcache.addr[inst->fcache.usedsize],
                               inst->fcache.fillsize - inst->fcache.usedsize,
                               &dummy, &frame_size);

  if (frame_ofst && !frame_size)
    {
      inst->fcache.usedsize += frame_ofst;
      return SPRMP3_STATE_DECODE;
    }
  else if (!frame_size)
    {
      send_errormsg(inst->id, AL_COMM_MSGCODEERR_ILLIGALFRAME);
      return SPRMP3_STATE_ERROR;
    }

  /* Update frame information */

  /* TODO: Consider move this logic after decode_frame()
   * because it is also implemented in decode_frame().
   */

  hdr = &inst->fcache.addr[inst->fcache.usedsize + frame_ofst];
  info.channels = HDR_IS_MONO(hdr) ? 1 : 2;
  info.hz = hdr_sample_rate_hz(hdr);
  info.layer = 4 - HDR_GET_LAYER(hdr);
  info.bitrate_kbps = hdr_bitrate_kbps(hdr);

  if (inst->frame_info.hz == 0)
    {
      inst->frame_info.channels = info.channels;
      inst->frame_info.hz = info.hz;
      inst->frame_info.layer = info.layer;
      inst->frame_info.bitrate_kbps = info.bitrate_kbps;

      /* Initialize resampler here */

      speex_resampler_init(&inst->resampler.lch, 1,
                           inst->frame_info.hz, outq->hz, 1);
      speex_resampler_init(&inst->resampler.rch, 1,
                           inst->frame_info.hz, outq->hz, 1);

      send_frameinfo(inst->id, &inst->frame_info);
    }
  else if (inst->frame_info.channels != info.channels ||
           inst->frame_info.hz != info.hz ||
           inst->frame_info.layer != info.layer)
    {
      send_errormsg(inst->id, AL_COMM_MSGCODEERR_MULTIFRAME);
      return SPRMP3_STATE_ERROR;
    }

#ifdef SPRMP3_DEBUG_COMPARE
  dbg_compare(hdr, frame_size);
#endif

  samples = mp3dec_decode_frame(inst->gain, &inst->core, &inst->work,
                                hdr, frame_size,
                                (mp3d_sample_t *)&inst->pcmcache.addr[0],
                                &inst->frame_info);
  inst->fcache.usedsize += frame_ofst + frame_size;

  next_state = refill_framecache(inst);

  if (samples)
    {
      inst->pcmcache.decsize = samples * inst->frame_info.channels
                                       * sizeof(mp3d_sample_t);
      resampling(inst, outq->chs, outq->hz);
      next_state = write_outmem(inst, outq, next_state);

      if (next_state == SPRMP3_STATE_DECODE && is_decode_done(inst))
        {
          next_state = SPRMP3_STATE_ENDING;
        }
    }

#ifdef ENABME_PERFORMANCE
  printf("Decode Time : %ld\n", measure_interval(&dec_time));
#endif

  return next_state;
}

/*** name: has_newoutmem */

static bool has_newoutmem(int id, sprmp3_outmemqueue_t *outq)
{
  unsigned int mask = (1 << id);
  return (sq_peek(&outq->queued) != NULL) && !(outq->done & mask);
}

/*** name: exec_fillupstate */

static int exec_fillupstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq)
{
  int next_state = SPRMP3_STATE_FILLUP;

  if (has_newoutmem(inst->id, outq))
    {
      next_state =  refill_framecache(inst);
      if (next_state == SPRMP3_STATE_DECODE && is_decode_done(inst))
        {
          next_state = SPRMP3_STATE_ENDING;
        }
    }

  return next_state;
}

/*** name: exec_fillremstate */

static int exec_fillremstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq)
{
  int next_state;

  if (has_newoutmem(inst->id, outq))
    {
      next_state = refill_framecache(inst);
      next_state = write_outmem(inst, outq, next_state);

      if (next_state == SPRMP3_STATE_DECODE && is_decode_done(inst))
        {
          next_state = SPRMP3_STATE_ENDING;
        }

      return next_state;
    }

  return SPRMP3_STATE_FILLUPREMAIN;
}

/*** name: exec_waitinstate */

static int exec_waitinstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq)
{
  (void)outq;

  if (sq_peek(&inst->fqueue.queued) != NULL)
    {
      return SPRMP3_STATE_DECODE;
    }

  return SPRMP3_STATE_WAITIN;
}

/*** name: exec_winremstate */

static int exec_winremstate(sprmp3_t *inst, sprmp3_outmemqueue_t *outq)
{
  (void)outq;

  if (sq_peek(&inst->fqueue.queued) != NULL)
    {
      return refill_framecache(inst);
    }

  return SPRMP3_STATE_WAITINREMAIN;
}

/*** name: process_instance */

static void process_instances(sprmp3_sys_t *sys)
{
  int i;
  sprmp3_t *inst;

  for (i = 0, inst = &sys->insts[0]; i < SPRMP3_MAX_INSTANCE; i++, inst++)
    {
      inst->state = state_procs[inst->state].proc(inst, &sys->outqueue);
      if (inst->state == SPRMP3_STATE_ERROR)
        {
          reset_instance(inst);
          inst->state = SPRMP3_STATE_END;
        }
    }
}

/*** name: exec_playing */

static void exec_playing(sprmp3_sys_t *sys)
{
  switch (sys->system_state)
    {
      case SPRMP3_SYSSTATE_PLAY:
        process_instances(sys);
        break;

      case SPRMP3_SYSSTATE_STOPPING:
      case SPRMP3_SYSSTATE_PAUSING:
        if (sys->fade_counter >= SPRMP3_FADEOUT_COUNT)
          {
            sys->system_state =
              sys->system_state == SPRMP3_SYSSTATE_PAUSING ?
                SPRMP3_SYSSTATE_PAUSE : SPRMP3_SYSSTATE_STOP;
          }
        else
          {
            process_instances(sys);
          }
        break;

      case SPRMP3_SYSSTATE_RESUMING:
        if (sys->fade_counter >= SPRMP3_FACEIN_COUNT)
          {
            sys->system_state = SPRMP3_SYSSTATE_PLAY;
          }
        else
          {
            process_instances(sys);
          }
        break;

      default:
        sys->system_state = SPRMP3_SYSSTATE_STOP;
        break;
    }
}

/*** name: exist_playing */

static bool exist_playing(sprmp3_t *insts)
{
  int i;
  bool ret = false;

  for (i = 0; i < SPRMP3_MAX_INSTANCE; i++)
    {
      if (state_procs[insts->state].playing)
        {
          ret = true;
          break;
        }

      insts++;
    }

  return ret;
}

/*** name: all_player_done */

static bool all_player_done(sprmp3_t *insts, unsigned int done)
{
  int i;
  unsigned int mask;

  for (i = 0; i < SPRMP3_MAX_INSTANCE; i++)
    {
      mask = 1 << insts->id;
      if (state_procs[insts->state].playing && !(mask & done))
        {
          return false;
        }

      insts++;
    }

  return true;
}

/*** name: fade_proc */

static void fade_proc(mp3d_sample_t *sample, int sample_bytes,
                      float *gain, float gain_diff, float termval,
                      int *cnt, int ch)
{
  int i;
  int bytes;

  bytes = 0;
  while (bytes < sample_bytes)
    {
      *cnt = *cnt + 1;
      *gain = *cnt < SPRMP3_FADEOUT_COUNT ?
              *gain + gain_diff :
              termval;

      for (i = 0; i < ch; i++)
        {
          *sample = (mp3d_sample_t)(*sample * *gain);
          bytes += sizeof(mp3d_sample_t);
          sample++;
        }
    }
}

/*** name: deliver_decodedpcm */

static int deliver_decodedpcm(sprmp3_t *insts, sprmp3_outmemqueue_t *outq,
                              float *gain, int *cnt, int state)
{
  sprmp3_outmemcont_t *mem =
                   (sprmp3_outmemcont_t *)sq_peek(&outq->queued);

  if (mem)
    {
      if (all_player_done(insts, outq->done))
        {
          switch (state)
            {
              case SPRMP3_SYSSTATE_PAUSING:
              case SPRMP3_SYSSTATE_STOPPING:
                fade_proc((mp3d_sample_t *)mem->addr, mem->size,
                          gain, -SPRMP3_GAIN_DIFF, 0.f, cnt, outq->chs);
                break;

              case SPRMP3_SYSSTATE_RESUMING:
                fade_proc((mp3d_sample_t *)mem->addr, mem->size,
                          gain, SPRMP3_GAIN_DIFF, 1.f, cnt, outq->chs);
                break;
            }

          return deliver_outpcm(outq);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mp3dec_main(void)
{
  init_system(&g_sys);
  if (sprmp3_initmsgloop() != OK)
    {
      return 0;
    }

#ifdef SPRMP3_DEBUG_COMPARE
  dbg_init_compare();
#endif

#ifdef SPRMP3_DEBUG
  print_status(&g_sys);
#endif

  send_bootmsg();

  while (g_sys.system_state != SPRMP3_SYSSTATE_TERM)
    {
#ifdef SPRMP3_DEBUG
      sprmp3_dprintf("\n::::::::: Start Loop ::::::::::\n");
#endif
      if (g_sys.system_state == SPRMP3_SYSSTATE_STOP)
        {
          sprmp3_pollmessage(&g_sys, 1);
        }
      else
        {
          sprmp3_pollmessage(&g_sys, 0);
#ifdef SPRMP3_DEBUG
          print_status(&g_sys);
          print_buffer_status(&g_sys);
#endif
          if (g_sys.system_state != SPRMP3_SYSSTATE_TERM)
            {
              exec_playing(&g_sys);
#ifdef SPRMP3_DEBUG
              print_buffer_status(&g_sys);
#endif
              if (g_sys.system_state == SPRMP3_SYSSTATE_PAUSE ||
                  !exist_playing(g_sys.insts))
                {
#ifdef SPRMP3_DEBUG
                  sprmp3_dprintf("Just Deliver PCM..\n");
#endif
                  deliver_outpcm(&g_sys.outqueue);
                }
              else
                {
#ifdef SPRMP3_DEBUG
                  sprmp3_dprintf("Deliver PCM if possible..\n");
#endif
                  deliver_decodedpcm(g_sys.insts,
                                     &g_sys.outqueue,
                                     &g_sys.sys_gain,
                                     &g_sys.fade_counter,
                                     g_sys.system_state);
                }
            }
        }
    }

  return 0;
}
