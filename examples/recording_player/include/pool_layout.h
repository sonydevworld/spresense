/****************************************************************************
 * recording_player/include/pool_layout.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#ifndef POOL_LAYOUT_H_INCLUDED
#define POOL_LAYOUT_H_INCLUDED

#include "memutils/memory_manager/MemMgrTypes.h"

namespace MemMgrLite {

MemPool* static_pools[NUM_MEM_POOLS];

extern const PoolAttr MemoryPoolLayouts[NUM_MEM_LAYOUTS][NUM_MEM_POOLS] = {
 {/* Layout:0 */
  /* pool_ID          type       seg fence  addr        size         */
  { DEC_ES_MAIN_BUF_POOL, BasicType,   4, true, 0x000c0008, 0x00006000 },  /* AUDIO_WORK_AREA */
  { REND_PCM_BUF_POOL, BasicType,   9, true, 0x000c6010, 0x00012048 },  /* AUDIO_WORK_AREA */
  { DEC_APU_CMD_POOL, BasicType,  10, true, 0x000d8060, 0x00000398 },  /* AUDIO_WORK_AREA */
  { PF0_PCM_BUF_POOL, BasicType,   1, true, 0x000d8400, 0x00002008 },  /* AUDIO_WORK_AREA */
  { PF1_PCM_BUF_POOL, BasicType,   1, true, 0x000da410, 0x00002008 },  /* AUDIO_WORK_AREA */
  { PF0_APU_CMD_POOL, BasicType,  10, true, 0x000dc420, 0x00000398 },  /* AUDIO_WORK_AREA */
  { PF1_APU_CMD_POOL, BasicType,  10, true, 0x000dc7c0, 0x00000398 },  /* AUDIO_WORK_AREA */
  { ES_BUF_POOL     , BasicType,   2, true, 0x000dcb60, 0x00006000 },  /* AUDIO_WORK_AREA */
  { INPUT_BUF_POOL  , BasicType,   5, true, 0x000e2b68, 0x0000f000 },  /* AUDIO_WORK_AREA */
  { ENC_APU_CMD_POOL, BasicType,   3, true, 0x000f1b70, 0x00000114 },  /* AUDIO_WORK_AREA */
  { SRC_APU_CMD_POOL, BasicType,   3, true, 0x000f1c90, 0x00000114 },  /* AUDIO_WORK_AREA */
 },
}; /* end of MemoryPoolLayouts */

}  /* end of namespace MemMgrLite */

#endif /* POOL_LAYOUT_H_INCLUDED */
