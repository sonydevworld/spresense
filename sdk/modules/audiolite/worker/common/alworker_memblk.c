/****************************************************************************
 * modules/audiolite/worker/common/alworker_memblk.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>
#include <asmp/stdio.h>
#include "alworker_memblk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#define TEMPLATE_PUSH_POP(name, type) \
  type memblk_pop_##name(memblk_t *mb) \
    { \
      type ret = *((type *)memblk_dataptr(mb)); \
      memblk_updateused(mb, sizeof(type)); \
      return ret; \
    } \
  void memblk_push_##name(memblk_t *mb, type val) \
    { \
      memblk_fillupraw(mb, (char *)&val, sizeof(type)); \
    }

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* memblk_shift
 *
 * Shift remain data to top of the memory
 *
 * [After]
 * +--------------------------+
 * |12341234                  |
 * +--------------------------+
 * ^        ^          ^
 * used    filled      |
 *                     |
 * [Before]            |
 * +--------------------------+
 * |        12341234          |
 * +--------------------------+
 *          ^       ^
 *         used    filled
 */

void memblk_shift(memblk_t *mb)
{
  int remain = memblk_remain(mb);

  if (remain && mb->used != 0)
    {
      memmove(mb->addr, &mb->addr[mb->used], remain);
    }

  memblk_setfilled(mb, remain);
  memblk_setused(mb, 0);
}

/* memblk_shift_drop
 *
 * Drop data and Shift remain data to top of the memory
 *
 * [After]
 * +--------------------------+
 * |34                        |
 * +--------------------------+
 * ^  ^filled          ^
 * used                |
 *                     |
 * [Before] <-sz->     |
 * +--------------------------+
 * |        12341234          |
 * +--------------------------+
 *          ^       ^
 *         used    filled
 */

void memblk_shift_drop(memblk_t *mb, int sz)
{
  int remain = memblk_remain(mb);
  remain = remain - MIN(remain, sz);

  if (remain && (mb->used + sz) != 0)
    {
      memmove(mb->addr, &mb->addr[mb->used + sz], remain);
    }

  memblk_setfilled(mb, remain);
  memblk_setused(mb, 0);
}

/* memblk_fillup
 *
 * Copy remaining data in src to the space of dst.
 */

int  memblk_fillup(memblk_t *dst, memblk_t *src)
{
  int cpysize = MIN(memblk_space(dst), memblk_remain(src));

  memcpy(memblk_fillptr(dst), memblk_dataptr(src), cpysize);
  memblk_updatefilled(dst, cpysize);
  memblk_updateused(src, cpysize);

  return cpysize;
}

/* memblk_fillup
 *
 * Copy data stored in src to the space of dst.
 */

int  memblk_fillupraw(memblk_t *dst, char *src, int sz)
{
  int cpysize = MIN(memblk_space(dst), sz);

  memcpy(memblk_fillptr(dst), src, cpysize);
  memblk_updatefilled(dst, cpysize);

  return cpysize;
}

/* memblk_single16
 *
 * Reduce channel from embedded LR dual channels.
 *
 *   Dstination(Lch only)
 * +------------+
 * |131313131313|
 * +------------+
 *       ^
 *       |
 * +--------------------------+
 * |12341234123412341234123412|
 * +--------------------------+
 *     Dual channel
 */

int  memblk_single16(memblk_t *lch, memblk_t *src)
{
  int i;
  int samples;
  short *lchadr = (short *)memblk_fillptr(lch);
  short *srcadr = (short *)memblk_dataptr(src);

  samples = MIN(memblk_space(lch), memblk_remain(src)/2);
  samples /= sizeof(short);

  for (i = 0; i < samples; i++)
    {
      *lchadr++ = *srcadr++;
      srcadr++;
    }

  samples *= sizeof(short);

  memblk_updatefilled(lch, samples);
  memblk_updateused(src, samples * 2);

  return samples;
}

/* memblk_duplicate16
 *
 * Duplicate embedded single channel data into distination.
 *
 *          Dstination
 * +--------------------------+
 * |11221122112211221122112211|
 * +--------------------------+
 *       ^
 *       |
 * +------------+
 * |121212121212|
 * +------------+
 *  Source Plane
 */

int  memblk_duplicate16(memblk_t *dst, memblk_t *src)
{
  int i;
  int samples;
  short *srcadr = (short *)memblk_dataptr(src);
  short *dstadr = (short *)memblk_fillptr(dst);

  samples = MIN(memblk_remain(src), memblk_space(dst) / 2);
  samples /= sizeof(short);

  for (i = 0; i < samples; i++)
    {
      *dstadr++ = *srcadr;
      *dstadr++ = *srcadr++;
    }

  samples *= sizeof(short);

  src->used   += samples;
  dst->filled += samples * 2;

  return samples * 2;
}

/* memblk_split_lr16
 *
 * Split embedded LR data into  Lch and Rch plane.
 *
 *   Lch Plane      Rch Plane
 * +------------+ +------------+
 * |LLLLLLLLLLLL| |RRRRRRRRRRRR|
 * +------------+ +------------+
 *       ^              ^
 *       |              |
 * +--------------------------+
 * |LRLRLRLRLRLRLRLRLRLRLRLRLR|
 * +--------------------------+
 *          Source data
 */

int  memblk_split_lr16(memblk_t *lch, memblk_t *rch, memblk_t *src)
{
  int i;
  int split_samples;
  short *lchadr = (short *)memblk_fillptr(lch);
  short *rchadr = (short *)memblk_fillptr(rch);
  short *srcadr = (short *)memblk_dataptr(src);

  ALWORKER_DBGASSERT(memblk_space(rch) == memblk_space(lch));

  split_samples = MIN(memblk_remain(src) / 2, memblk_space(lch));
  split_samples /= sizeof(short);

  for (i = 0; i < split_samples; i++)
    {
      *lchadr++ = *srcadr++;
      *rchadr++ = *srcadr++;
    }

  split_samples *= sizeof(short);

  memblk_updatefilled(lch, split_samples);
  memblk_updatefilled(rch, split_samples);
  memblk_updateused(src, split_samples * 2);

  return split_samples;
}

/* memblk_conbine_lr16
 *
 * Conbine Lch and Rch plane data to LR embedded in dst.
 *
 *          Dstination
 * +--------------------------+
 * |LRLRLRLRLRLRLRLRLRLRLRLRLR|
 * +--------------------------+
 *       ^              ^
 *       |              |
 * +------------+ +------------+
 * |LLLLLLLLLLLL| |RRRRRRRRRRRR|
 * +------------+ +------------+
 *   Lch Plane      Rch Plane
 */

int  memblk_conbine_lr16(memblk_t *dst, memblk_t *lch, memblk_t *rch)
{
  int i;
  int samples;
  short *dstadr = (short *)memblk_fillptr(dst);
  short *lchadr = (short *)memblk_dataptr(lch);
  short *rchadr = (short *)memblk_dataptr(rch);

  samples = MIN(memblk_remain(lch), memblk_remain(rch));
  samples = MIN(memblk_space(dst) / 2, samples);

  samples = samples / sizeof(short);

  for (i = 0; i < samples; i++)
    {
      dstadr[i * 2 + 1] = *rchadr++; /* Odd is copied first */
      dstadr[i * 2 + 0] = *lchadr++;
    }

  samples *= sizeof(short);

  memblk_updatefilled(dst, samples * 2);
  memblk_updateused(lch, samples);
  memblk_updateused(rch, samples);

  return samples * 2;
}

/* memblk_conbine_lr16acc
 *
 * Conbine Lch and Rch plane data to LR embedded.
 * To reduce using memory, distination memory, lch memory
 * and rch memory are shared.
 *
 *  <---- Distination ------>     Half of Lch(or Rch)
 * +------------+------------+    +----+
 * | Lch plane  | Rch plane  |    |Acc |
 * +------------+------------+    +----+
 */

int  memblk_conbine_lr16acc(memblk_t *dst, memblk_t *lch,
                            memblk_t *rch, memblk_t *acc)
{
  memblk_t tmp_rch;

  ALWORKER_DBGASSERT(lch->addr == dst->addr);
  ALWORKER_DBGASSERT(&lch->addr[lch->size] == rch->addr);
  ALWORKER_DBGASSERT(lch->size == rch->size);
  ALWORKER_DBGASSERT(dst->size == lch->size * 2);

  /* Make sure Lch and Rch data is started from the top */

  memblk_shift(lch);
  memblk_shift(rch);

  /* Make the destination instance empty */

  memblk_reset(dst);

  /* Swap data between lch bottom half and rch top half */

  memcpy(acc->addr,                 &lch->addr[lch->size / 2], lch->size / 2);
  memcpy(&lch->addr[lch->size / 2], rch->addr,                 lch->size / 2);
  memcpy(rch->addr,                 acc->addr,                 lch->size / 2);

  /* Copy top half of lch to accumulater for making space on top of lch mem */

  memblk_reset(acc);
  memcpy(acc->addr, lch->addr, lch->size / 2);
  memblk_init(&tmp_rch, &lch->addr[lch->size / 2], lch->size / 2);
  acc->filled    = lch->size / 2;
  tmp_rch.filled = lch->size / 2;

  /* Conbine Lch and Rch half data */

  memblk_conbine_lr16(dst, acc, &tmp_rch);

  /* Copy top half of lch to accumlater for making space on top of rch mem */

  memblk_reset(acc);
  memcpy(acc->addr, rch->addr, rch->size / 2);
  memblk_init(&tmp_rch, &rch->addr[rch->size / 2], rch->size / 2);
  acc->filled    = lch->size / 2;
  tmp_rch.filled = lch->size / 2;

  /* Conbine Lch and Rch half data */

  memblk_conbine_lr16(dst, acc, &tmp_rch);

  /* Data size in destination is total of lch and rch */

  dst->filled = memblk_remain(lch) + memblk_remain(rch);

  memblk_reset(lch);  /* Make the instance empty */
  memblk_reset(rch);  /* Make the instance empty */

  return memblk_remain(dst);
}

int memblk_conv_pcm16tofloat(memblk_t *flt, memblk_t *pcm16)
{
  int i;
  int fltspace = memblk_space(flt) / sizeof(float);
  int samp_num = memblk_remain(pcm16) / sizeof(short);
  float *fdat = (float *)memblk_fillptr(flt);
  short *pcm  = (short *)memblk_dataptr(pcm16);

  samp_num = (samp_num > fltspace) ? fltspace : samp_num;

  for (i = 0; i < samp_num; i++)
    {
      *fdat++ = (float)*pcm++;
    }

  memblk_updatefilled(flt, sizeof(float) * samp_num);
  memblk_updateused(pcm16, sizeof(short) * samp_num);

  return samp_num;
}

int memblk_conv_floattopcm16(memblk_t *pcm16, memblk_t *flt, float gain)
{
  int i;
  int fltspace = memblk_remain(flt) / sizeof(float);
  int samp_num = memblk_space(pcm16) / sizeof(short);
  float *fdat = (float *)memblk_dataptr(flt);
  short *pcm  = (short *)memblk_fillptr(pcm16);

  samp_num = (samp_num > fltspace) ? fltspace : samp_num;

  for (i = 0; i < samp_num; i++)
    {
      /* TODO: Use CMSIS for the gain */

      *pcm++ = gain * (short)*fdat++;
    }

  memblk_updateused(flt, sizeof(float) * samp_num);
  memblk_updatefilled(pcm16, sizeof(short) * samp_num);

  return samp_num;
}

int memblk_normalizef(memblk_t *flt, float min, float max)
{
  int i;
  int remain = memblk_remain(flt) / sizeof(float);
  float *dat = (float *)memblk_dataptr(flt);
  float div = max - min;

  if (div <= 0.f)
    {
      return -1;
    }

  for (i = 0; i < remain; i++, dat++)
    {
      *dat = (*dat - min) / div;
      if (*dat < -1) *dat = -1;
      else if (*dat >  1) *dat =  1;
    }

  return remain;
}

TEMPLATE_PUSH_POP(float, float)
TEMPLATE_PUSH_POP(uint8, uint8_t)
TEMPLATE_PUSH_POP(int16, int16_t)
