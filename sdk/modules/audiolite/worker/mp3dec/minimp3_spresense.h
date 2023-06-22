/****************************************************************************
 * modules/audiolite/worker/mp3dec/minimp3_spresense.h
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

#ifndef __AUDIOLITE_WORKER_MP3DEC_MINIMP3_SPRESENSE_H
#define __AUDIOLITE_WORKER_MP3DEC_MINIMP3_SPRESENSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/queue.h>

#include <audiolite/sprmp3dec_qsize.h>

#include "speex_resampler.h"
#include "minimp3.h"
#include "minimp3_ex.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPRMP3_MAX_INSTANCE:
 *  MP3 stream decoding instance number.
 *  Unfortunately now it supports just 1,
 *  because total performance including
 *  resampling is not enough.
 */

#define SPRMP3_MAX_INSTANCE (1)

#define SPRMP3_STATE_END          (0)
#define SPRMP3_STATE_INIT         (1)
#define SPRMP3_STATE_READY        (2)
#define SPRMP3_STATE_WAITSEEK     (3)
#define SPRMP3_STATE_DECODE       (4)
#define SPRMP3_STATE_FILLUP       (5)
#define SPRMP3_STATE_FILLUPREMAIN (6)
#define SPRMP3_STATE_WAITIN       (7)
#define SPRMP3_STATE_WAITINREMAIN (8)
#define SPRMP3_STATE_ENDING       (9)
#define SPRMP3_STATE_ERROR        (10)

#define SPRMP3_SYSSTATE_STOP      (0)
#define SPRMP3_SYSSTATE_PAUSE     (1)
#define SPRMP3_SYSSTATE_PAUSING   (2)
#define SPRMP3_SYSSTATE_PLAY      (3)
#define SPRMP3_SYSSTATE_RESUMING  (4)
#define SPRMP3_SYSSTATE_STOPPING  (5)
#define SPRMP3_SYSSTATE_TERM      (6)

#define SPRMP3_MODE_STREAM        (0)
#define SPRMP3_MODE_JUSTDECODE    (1)

#define SPRMP3_OUTDONE(inst) (1 << (inst)->id)

#define SPRMP3_TEMPSAMP_RATE (32000)
#define SPRMP3_FADE_TIME_MS  (50)

#define SPRMP3_FADEOUT_COUNT \
          (SPRMP3_TEMPSAMP_RATE * SPRMP3_FADE_TIME_MS / 1000)
#define SPRMP3_FACEIN_COUNT \
          (SPRMP3_TEMPSAMP_RATE * SPRMP3_FADE_TIME_MS / 1000)

#define SPRMP3_GAIN_DIFF  (1.f / (float)SPRMP3_FADEOUT_COUNT)

#define SPRMP3_PCMCACHE_SZ (MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(mp3d_sample_t))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Frame cache */

struct sprmp3_fcache_s
{
  unsigned int size;
  unsigned int fillsize;
  unsigned int usedsize; /* TODO: Rename this to usedofst; */
  bool eof;
  unsigned char addr[MINIMP3_IO_SIZE];
};
typedef struct sprmp3_fcache_s sprmp3_fcache_t;

/* Decoded PCM cache */

struct sprmp3_pcmcache_s
{
  unsigned int size;
  unsigned int decsize;
  unsigned char addr[SPRMP3_PCMCACHE_SZ];
};
typedef struct sprmp3_pcmcache_s sprmp3_pcmcache_t;

/* Resampled PCM cache */

struct sprmp3_resamcache_s
{
  unsigned int size;
  unsigned int decsize;
  unsigned char addr[SPRMP3_PCMCACHE_SZ * 2];
};
typedef struct sprmp3_resamcache_s sprmp3_resamcache_t;

struct sprmp3_resample_s
{
  SpeexResamplerState lch;
  unsigned char lch_in[SPRMP3_PCMCACHE_SZ / 2];
  unsigned char lch_out[SPRMP3_PCMCACHE_SZ];

  SpeexResamplerState rch;
  unsigned char rch_in[SPRMP3_PCMCACHE_SZ / 2];
  unsigned char rch_out[SPRMP3_PCMCACHE_SZ];
};
typedef struct sprmp3_resample_s sprmp3_resample_t;

/* Output PCM cache reference */

struct sprmp3_targetcache_s
{
  unsigned int size;
  unsigned int decsize;
  unsigned int remofst;
  unsigned char *addr;
};
typedef struct sprmp3_targetcache_s sprmp3_targetcache_t;

/* Frame memory block container */

struct sprmp3_fmemcont_s
{
  sq_entry_t link;
  unsigned char *addr;
  unsigned int size;
  bool eof;
};
typedef struct sprmp3_fmemcont_s sprmp3_fmemcont_t;

/* Frame memory queue */

struct sprmp3_fmemqueue_s
{
  sq_queue_t queued;
  sq_queue_t free;

  unsigned int copied_ofst;
  sprmp3_fmemcont_t inst[SPRMP3_FRAMEMEM_QSIZE];
};
typedef struct sprmp3_fmemqueue_s sprmp3_fmemqueue_t;

/* Output memory container */

struct sprmp3_outmemcont_s
{
  sq_entry_t link;
  unsigned char *addr;
  unsigned int size;
};
typedef struct sprmp3_outmemcont_s sprmp3_outmemcont_t;

/* Output memory queue */

struct sprmp3_outmemqueue_s
{
  sq_queue_t queued;
  sq_queue_t free;

  int chs;
  int hz;
  int mode;

  unsigned int done;
  unsigned int filled_size;

  sprmp3_outmemcont_t inst[SPRMP3_OUTMEM_QSIZE];
};
typedef struct sprmp3_outmemqueue_s sprmp3_outmemqueue_t;

/* Frame decoder instance */

struct sprmp3_s
{
  int id;

  mp3dec_t core;
  mp3dec_work_t work;
  mp3dec_frame_info_t frame_info;

  sprmp3_fcache_t   fcache;
  sprmp3_pcmcache_t pcmcache;
  sprmp3_targetcache_t tgtcache;

  sprmp3_resample_t resampler;
  sprmp3_resamcache_t resamcache;

  sprmp3_fmemqueue_t fqueue;

  int state;
  unsigned int omem_wofst;

  float gain;
  int32_t request;
};
typedef struct sprmp3_s sprmp3_t;

/* MP3 Decoder system instance */

struct sprmp3_sys_s
{
  sprmp3_outmemqueue_t outqueue;

  int fade_counter;
  int system_state;
  float sys_gain;

  sprmp3_t insts[SPRMP3_MAX_INSTANCE];
};
typedef struct sprmp3_sys_s sprmp3_sys_t;

#endif /* __AUDIOLITE_WORKER_MP3DEC_MINIMP3_SPRESENSE_H */
