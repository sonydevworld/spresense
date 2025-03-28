/* This file is generated automatically. */
/****************************************************************************
 * mem_layout.h
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

#ifndef MEM_LAYOUT_H_INCLUDED
#define MEM_LAYOUT_H_INCLUDED

/*
 * Memory devices
 */

/* AUD_SRAM: type=RAM, use=0x0003f300, remainder=0x00000d00 */

#define AUD_SRAM_ADDR  0x000c0000
#define AUD_SRAM_SIZE  0x00040000

/*
 * Fixed areas
 */

#define AUDIO_WORK_AREA_ALIGN   0x00000008
#define AUDIO_WORK_AREA_ADDR    0x000c0000
#define AUDIO_WORK_AREA_DRM     0x000c0000 /* _DRM is obsolete macro. to use _ADDR */
#define AUDIO_WORK_AREA_SIZE    0x0003d000

#define MSG_QUE_AREA_ALIGN   0x00000008
#define MSG_QUE_AREA_ADDR    0x000fd000
#define MSG_QUE_AREA_DRM     0x000fd000 /* _DRM is obsolete macro. to use _ADDR */
#define MSG_QUE_AREA_SIZE    0x00002000

#define MEMMGR_WORK_AREA_ALIGN   0x00000008
#define MEMMGR_WORK_AREA_ADDR    0x000ff000
#define MEMMGR_WORK_AREA_DRM     0x000ff000 /* _DRM is obsolete macro. to use _ADDR */
#define MEMMGR_WORK_AREA_SIZE    0x00000200

#define MEMMGR_DATA_AREA_ALIGN   0x00000008
#define MEMMGR_DATA_AREA_ADDR    0x000ff200
#define MEMMGR_DATA_AREA_DRM     0x000ff200 /* _DRM is obsolete macro. to use _ADDR */
#define MEMMGR_DATA_AREA_SIZE    0x00000100

/*
 * Memory Manager max work area size
 */

#define S0_MEMMGR_WORK_AREA_ADDR  MEMMGR_WORK_AREA_ADDR
#define S0_MEMMGR_WORK_AREA_SIZE  0x000000bc

/*
 * Section IDs
 */

#define SECTION_NO0       0

/*
 * Number of sections
 */

#define NUM_MEM_SECTIONS  1

/*
 * Pool IDs
 */

const MemMgrLite::PoolId S0_NULL_POOL                = { 0, SECTION_NO0};  /*  0 */
const MemMgrLite::PoolId S0_REND_PCM_BUF_POOL        = { 1, SECTION_NO0};  /*  1 */
const MemMgrLite::PoolId S0_OSC_APU_CMD_POOL         = { 2, SECTION_NO0};  /*  2 */
const MemMgrLite::PoolId S0_PF0_PCM_BUF_POOL         = { 3, SECTION_NO0};  /*  3 */
const MemMgrLite::PoolId S0_PF1_PCM_BUF_POOL         = { 4, SECTION_NO0};  /*  4 */
const MemMgrLite::PoolId S0_PF0_APU_CMD_POOL         = { 5, SECTION_NO0};  /*  5 */
const MemMgrLite::PoolId S0_PF1_APU_CMD_POOL         = { 6, SECTION_NO0};  /*  6 */

#define NUM_MEM_S0_LAYOUTS   1
#define NUM_MEM_S0_POOLS     7

#define NUM_MEM_LAYOUTS      1
#define NUM_MEM_POOLS        7

/*
 * Pool areas
 */

/* Section0 Layout0: */

#define MEMMGR_S0_L0_WORK_SIZE   0x000000bc

/* Skip 0x0004 bytes for alignment. */

#define S0_L0_REND_PCM_BUF_POOL_ALIGN    0x00000008
#define S0_L0_REND_PCM_BUF_POOL_L_FENCE  0x000c0004
#define S0_L0_REND_PCM_BUF_POOL_ADDR     0x000c0008
#define S0_L0_REND_PCM_BUF_POOL_SIZE     0x00005000
#define S0_L0_REND_PCM_BUF_POOL_U_FENCE  0x000c5008
#define S0_L0_REND_PCM_BUF_POOL_NUM_SEG  0x00000005
#define S0_L0_REND_PCM_BUF_POOL_SEG_SIZE 0x00001000

#define S0_L0_OSC_APU_CMD_POOL_ALIGN    0x00000008
#define S0_L0_OSC_APU_CMD_POOL_L_FENCE  0x000c500c
#define S0_L0_OSC_APU_CMD_POOL_ADDR     0x000c5010
#define S0_L0_OSC_APU_CMD_POOL_SIZE     0x000005c0
#define S0_L0_OSC_APU_CMD_POOL_U_FENCE  0x000c55d0
#define S0_L0_OSC_APU_CMD_POOL_NUM_SEG  0x00000010
#define S0_L0_OSC_APU_CMD_POOL_SEG_SIZE 0x0000005c

#define S0_L0_PF0_PCM_BUF_POOL_ALIGN    0x00000008
#define S0_L0_PF0_PCM_BUF_POOL_L_FENCE  0x000c55d4
#define S0_L0_PF0_PCM_BUF_POOL_ADDR     0x000c55d8
#define S0_L0_PF0_PCM_BUF_POOL_SIZE     0x00001000
#define S0_L0_PF0_PCM_BUF_POOL_U_FENCE  0x000c65d8
#define S0_L0_PF0_PCM_BUF_POOL_NUM_SEG  0x00000001
#define S0_L0_PF0_PCM_BUF_POOL_SEG_SIZE 0x00001000

#define S0_L0_PF1_PCM_BUF_POOL_ALIGN    0x00000008
#define S0_L0_PF1_PCM_BUF_POOL_L_FENCE  0x000c65dc
#define S0_L0_PF1_PCM_BUF_POOL_ADDR     0x000c65e0
#define S0_L0_PF1_PCM_BUF_POOL_SIZE     0x00001000
#define S0_L0_PF1_PCM_BUF_POOL_U_FENCE  0x000c75e0
#define S0_L0_PF1_PCM_BUF_POOL_NUM_SEG  0x00000001
#define S0_L0_PF1_PCM_BUF_POOL_SEG_SIZE 0x00001000

#define S0_L0_PF0_APU_CMD_POOL_ALIGN    0x00000008
#define S0_L0_PF0_APU_CMD_POOL_L_FENCE  0x000c75e4
#define S0_L0_PF0_APU_CMD_POOL_ADDR     0x000c75e8
#define S0_L0_PF0_APU_CMD_POOL_SIZE     0x00000398
#define S0_L0_PF0_APU_CMD_POOL_U_FENCE  0x000c7980
#define S0_L0_PF0_APU_CMD_POOL_NUM_SEG  0x0000000a
#define S0_L0_PF0_APU_CMD_POOL_SEG_SIZE 0x0000005c

#define S0_L0_PF1_APU_CMD_POOL_ALIGN    0x00000008
#define S0_L0_PF1_APU_CMD_POOL_L_FENCE  0x000c7984
#define S0_L0_PF1_APU_CMD_POOL_ADDR     0x000c7988
#define S0_L0_PF1_APU_CMD_POOL_SIZE     0x00000398
#define S0_L0_PF1_APU_CMD_POOL_U_FENCE  0x000c7d20
#define S0_L0_PF1_APU_CMD_POOL_NUM_SEG  0x0000000a
#define S0_L0_PF1_APU_CMD_POOL_SEG_SIZE 0x0000005c

/* Remainder AUDIO_WORK_AREA=0x000352dc */

#endif /* MEM_LAYOUT_H_INCLUDED */
