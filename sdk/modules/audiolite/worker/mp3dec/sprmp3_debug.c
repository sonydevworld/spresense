/****************************************************************************
 * modules/audiolite/worker/mp3dec/sprmp3_debug.c
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

#ifdef BUILD_TGT_ASMPWORKER
#  include <asmp/stdio.h>
#else
#  include <stdio.h>
#endif

#include "sprmp3_debug.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef SPRMP3_DEBUG

#ifdef SPRMP3_DEBUG_COMPARE
#include <stdlib.h>

static int dbg_framesz;
static int dbg_frame_ofst;
static unsigned char *dbg_mp3framemem;

void dbg_compare(const unsigned char *hdr, int frame_size)
{
  int i;
  for (i = 0; i < frame_size; i++)
    {
      if (hdr[i] != dbg_mp3framemem[dbg_frame_ofst + i])
        {
          printf("Error: offst:%d(0x%08x), exp(0x%02x), act(0x%02x)\n",
            dbg_frame_ofst + i, dbg_frame_ofst + i, hdr[i],
            dbg_mp3framemem[dbg_frame_ofst + i]);
          while (1)
            {
            }
        }
    }

  printf("Frame ofst:%d(0x%08x) sz:%d OK\n", dbg_frame_ofst + i,
                                             dbg_frame_ofst + i,
                                             frame_size);
  dbg_frame_ofst += frame_size;
}

static int dbg_load_mp3frame(const char *fname, unsigned char **fmem)
{
  FILE *ifp;
  int data_bytes;

  ifp = fopen(fname, "rb");

  fseek(ifp, 0L, SEEK_END);
  data_bytes = ftell(ifp);
  fseek(ifp, 0L, SEEK_SET);

  *fmem = (unsigned char *)malloc(data_bytes);

  fread(*fmem, data_bytes, 1, ifp);
  fclose(ifp);

  printf("Load data: %s(%d)\n", fname, data_bytes);
  return data_bytes;
}

int dbg_init_compare(void)
{
  dbg_frame_ofst = 0;
  dbg_framesz = dbg_load_mp3frame(TEST_MP3FNAME, &dbg_mp3framemem);
}
#endif

#ifdef SPRMP3_DEBUG_DETAIL
static const char *inststatelog(int state)
{
  switch (state)
    {
      case SPRMP3_STATE_END: return "END";
      case SPRMP3_STATE_INIT: return "INIT";
      case SPRMP3_STATE_READY: return "READY";
      case SPRMP3_STATE_DECODE: return "DECODE";
      case SPRMP3_STATE_FILLUP: return "FILLUP";
      case SPRMP3_STATE_FILLUPREMAIN: return "FILLUPREMAIN";
      case SPRMP3_STATE_WAITIN: return "WAITIN";
      case SPRMP3_STATE_WAITINREMAIN: return "WAITINREMAIN";
      case SPRMP3_STATE_ENDING: return "ENDING";
    }

  return "Unknown";
}

static const char *statelog(int state)
{
  switch (state)
    {
      case SPRMP3_SYSSTATE_STOP: return "STOP";
      case SPRMP3_SYSSTATE_PAUSE: return "PAUSE";
      case SPRMP3_SYSSTATE_PAUSING: return "PAUSING";
      case SPRMP3_SYSSTATE_PLAY: return "PLAY";
      case SPRMP3_SYSSTATE_RESUMING: return "RESUMING";
      case SPRMP3_SYSSTATE_TERM: return "TERM";
    }

  return "Unknown";
}

void print_status(sprmp3_sys_t *sys)
{
  int i;
  sprmp3_t *inst;
  inst = &sys->insts[0];

  printf("==== System State : %s =====\n", statelog(sys->system_state));
  printf("     OutQueue: Free:%d, Queued:%d \n",
          sq_count(&sys->outqueue.free),
          sq_count(&sys->outqueue.queued));

  for (i = 0; i < SPRMP3_MAX_INSTANCE; i++)
    {
      printf("     Inst:%d Status:%s, Queue:<Free:%d, Queued:%d>,"
             " Gain:%f\n",
             inst->id, inststatelog(inst->state),
             sq_count(&inst->fqueue.free),
             sq_count(&inst->fqueue.queued), inst->gain);
      inst++;
    }

  printf("\n");
}

void print_buffer_status(sprmp3_sys_t *sys)
{
  int i;
  sprmp3_t *inst;
  inst = &sys->insts[0];

  for (i = 0; i < 1; i++)
    {
      printf("==== Inst:%d ====\n", inst->id);
      printf("+----+\n");
      printf("|    |\n");
      printf("|FMem|<- cpy_ofst:%d\n", inst->fqueue.copied_ofst);
      printf("|    |\n");
      printf("+----+ sz:%d\n", 4096);
      printf("+----+\n");
      printf("|    |\n");
      printf("|FCas|<- used:%d\n", inst->fcache.usedsize);
      printf("| %c  |<- fill:%d\n", inst->fcache.eof ? 'E' : ' ',
                                    inst->fcache.fillsize);
      printf("+----+ sz:%d\n", inst->fcache.size);
      printf("+----+\n");
      printf("|    |\n");
      printf("|PCM |\n");
      printf("|    |<- decsize:%d\n", inst->pcmcache.decsize);
      printf("+----+ sz:%d\n", inst->pcmcache.size);
      printf("+----+\n");
      printf("|    |\n");
      printf("|TGT |<- remofst:%d\n", inst->tgtcache.remofst);
      printf("|    |<- decsize:%d\n", inst->tgtcache.decsize);
      printf("+----+ sz:%d(remain:%d)\n", inst->tgtcache.size,
                                          inst->tgtcache.decsize -
                                          inst->tgtcache.remofst);
      printf("+----+\n");
      printf("|    |\n");
      printf("|OUT |<- writeofst:%d\n", inst->omem_wofst);
      printf("| %c  |<- filled_sz:%d\n", sys->outqueue.done &
                                         (1 << inst->id) ? 'D' : ' ',
                                         sys->outqueue.filled_size);
      printf("+----+ sz:%d(space:%d)\n", 4096, 4096 - inst->omem_wofst);
      inst++;
    }
}
#endif  /* SPRMP3_DEBUG_DETAIL */

#endif /* SPRMP3_DEBUG */
