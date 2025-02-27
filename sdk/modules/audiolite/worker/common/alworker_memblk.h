/****************************************************************************
 * modules/audiolite/worker/common/alworker_memblk.h
 *
 *   Copyright 2024 Sony Semiconductor Solutions Corporation
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

#ifndef __AUDIOLITE_WORKER_COMMON_ALWORKER_MEMBLK_H
#define __AUDIOLITE_WORKER_COMMON_ALWORKER_MEMBLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/queue.h>

#ifdef ALWORKER_ENABLE_ASSERTION
#include <worker/stdio.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef ALWORKER_ENABLE_ASSERTION
#  define ALWORKER_DBGASSERT(cond)  \
    do {  \
      if (!(cond)) {  \
        printf("!! ASSERT @ %s(%d): Condition [" #cond "] is failed\n", \
               __FILE__, __LINE__); \
        while (1);  \
      } \
    } while(0)
#else
#  define ALWORKER_DBGASSERT(cond)
#endif

#define MEMBLK_TYPE_INPUT  (0)
#define MEMBLK_TYPE_OUTPUT (1)
#define MEMBLK_TYPE_LOCAL  (2)

#define memblk_initialize(mb, ad, sz, e, t) \
  do {                                      \
    (mb)->addr = (ad);                      \
    (mb)->size = (sz);                      \
    (mb)->eof  = (e);                       \
    (mb)->filled = 0;                       \
    (mb)->used = 0;                         \
    (mb)->type = (t);                       \
  } while (0)

#define memblk_init(mb, ad, sz)  \
        memblk_initialize(mb, (char *)ad, sz, 0, MEMBLK_TYPE_LOCAL)

#define memblk_initin(mb, ad, sz, e)  \
        memblk_initialize(mb, (char *)ad, sz, e, MEMBLK_TYPE_INPUT)

#define memblk_initout(mb, ad, sz)  \
        memblk_initialize(mb, (char *)ad, sz, 0, MEMBLK_TYPE_OUTPUT)

#define memblk_setfilled(mb, sz)    ((mb)->filled = (sz))
#define memblk_setused(mb, sz)      ((mb)->used = (sz))
#define memblk_updatefilled(mb, sz) ((mb)->filled += (sz))
#define memblk_commit(mb, sz)       (memblk_updatefilled(mb, sz))
#define memblk_updateused(mb, sz)   ((mb)->used += (sz))
#define memblk_drop(mb, sz)         (memblk_updateused(mb, sz))
#define memblk_remain(mb)           ((mb)->filled - (mb)->used)
#define memblk_space(mb)            ((mb)->size - (mb)->filled)
#define memblk_is_empty(mb)         ((mb)->used == (mb)->filled)
#define memblk_is_full(mb)          ((mb)->filled == (mb)->size)
#define memblk_is_eof(mb)           ((mb)->eof != 0)
#define memblk_is_input(mb)         ((mb)->type == MEMBLK_TYPE_INPUT)
#define memblk_is_output(mb)        ((mb)->type == MEMBLK_TYPE_OUTPUT)
#define memblk_is_local(mb)         ((mb)->type == MEMBLK_TYPE_LOCAL)
#define memblk_set_eof(mb)          ((mb)->eof = 1)
#define memblk_reset_eof(mb)        ((mb)->eof = 0)
#define memblk_reset(mb)            ((mb)->used = (mb)->filled = 0)
#define memblk_fillptr(mb)          (&(mb)->addr[(mb)->filled])
#define memblk_dataptr(mb)          (&(mb)->addr[(mb)->used])
#define memblk_nextblk(mb)          ((memblk_t *)(mb)->link.flink)

#define memblk_fillptrint16(mb)     ((int16_t *)memblk_fillptr(mb))
#define memblk_fillptrfloat(mb)     ((float *)memblk_fillptr(mb))
#define memblk_fillptruint8(mb)     ((uint8_t *)memblk_fillptr(mb))

#define memblk_dataptrint16(mb)     ((int16_t *)memblk_dataptr(mb))
#define memblk_dataptrfloat(mb)     ((float *)memblk_dataptr(mb))
#define memblk_dataptruint8(mb)     ((uint8_t *)memblk_dataptr(mb))

#define memblk_remainint16(mb)      (memblk_remain(mb) / sizeof(int16_t))
#define memblk_remainfloat(mb)      (memblk_remain(mb) / sizeof(float))
#define memblk_remainint32(mb)      (memblk_remain(mb) / sizeof(int32_t))

#define memblk_spaceint16(mb)       (memblk_space(mb) / sizeof(int16_t))
#define memblk_spacefloat(mb)       (memblk_space(mb) / sizeof(float))
#define memblk_spaceint32(mb)       (memblk_space(mb) / sizeof(int32_t))

#define memblk_commitint16(mb, sz)  (memblk_updatefilled(mb, (sz) * 2))
#define memblk_commitfloat(mb, sz)  (memblk_updatefilled(mb, (sz) * 4))
#define memblk_commituint8(mb, sz)  (memblk_updatefilled(mb, sz))

#define memblk_dropint16(mb, sz)    (memblk_updateused(mb, (sz) * 2))
#define memblk_dropfloat(mb, sz)    (memblk_updateused(mb, (sz) * 4))
#define memblk_dropuint8(mb, sz)    (memblk_updateused(mb, sz))

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct memblk_s
{
  sq_entry_t link;
  char *addr;
  int size;
  int filled;
  int used;
  int eof;
  int type;
};
typedef struct memblk_s memblk_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void memblk_shift(memblk_t *mb);
void memblk_shift_drop(memblk_t *mb, int sz);
int  memblk_fillup(memblk_t *dst, memblk_t *src);
int  memblk_splitch(memblk_t *lch, memblk_t *rch, memblk_t *itrleave,
                    memblk_t *acc);
int  memblk_fillupraw(memblk_t *dst, char *src, int sz);

int  memblk_single16(memblk_t *lch, memblk_t *src);
int  memblk_duplicate16(memblk_t *dst, memblk_t *src);
int  memblk_split_lr16(memblk_t *lch, memblk_t *rch, memblk_t *src);
int  memblk_conbine_lr16(memblk_t *dst, memblk_t *lch, memblk_t *rch);
int  memblk_conbine_lr16acc(memblk_t *dst, memblk_t *lch,
                            memblk_t *rch, memblk_t *acc);
int memblk_conv_pcm16tofloat(memblk_t *flt, memblk_t *pcm16);
int memblk_conv_floattopcm16(memblk_t *pcm16, memblk_t *flt, float gain);
int memblk_normalizef(memblk_t *flt, float min, float max);

float   memblk_pop_float(memblk_t *mb);
void    memblk_push_float(memblk_t *mb, float val);
int16_t memblk_pop_int16(memblk_t *mb);
void    memblk_push_int16(memblk_t *mb, int16_t val);
uint8_t memblk_pop_uint8(memblk_t *mb);
void    memblk_push_uint8(memblk_t *mb, uint8_t val);

#endif /* __AUDIOLITE_WORKER_COMMON_ALWORKER_MEMBLK_H */
