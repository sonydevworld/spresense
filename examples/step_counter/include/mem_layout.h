/* This file is generated automatically. */
/****************************************************************************
 * mem_layout.h
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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
/* SHM_SRAM: type=RAM, use=0x0001f300, remainder=0x00000d00 */
#define SHM_SRAM_ADDR  0x000e0000
#define SHM_SRAM_SIZE  0x00020000

/*
 * Fixed areas
 */
#define SENSOR_WORK_AREA_ALIGN   0x00000008
#define SENSOR_WORK_AREA_ADDR    0x000e0000
#define SENSOR_WORK_AREA_DRM     0x000e0000 /* _DRM is obsolete macro. to use _ADDR */
#define SENSOR_WORK_AREA_SIZE    0x0001e000

#define MSG_QUE_AREA_ALIGN   0x00000008
#define MSG_QUE_AREA_ADDR    0x000fe000
#define MSG_QUE_AREA_DRM     0x000fe000 /* _DRM is obsolete macro. to use _ADDR */
#define MSG_QUE_AREA_SIZE    0x00001000

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
#define S0_MEMMGR_WORK_AREA_SIZE  0x00000060

/*
 * Section IDs
 */
#define SECTION_NO0       0

#define NUM_MEM_SECTIONS  1

/*
 * Pool IDs
 */
const MemMgrLite::PoolId S0_NULL_POOL                = { 0, SECTION_NO0};  /*  0 */
const MemMgrLite::PoolId S0_SENSOR_DSP_CMD_BUF_POOL  = { 1, SECTION_NO0};  /*  1 */
const MemMgrLite::PoolId S0_ACCEL_DATA_BUF_POOL      = { 2, SECTION_NO0};  /*  2 */
const MemMgrLite::PoolId S0_GNSS_DATA_BUF_POOL       = { 3, SECTION_NO0};  /*  3 */

#define NUM_MEM_S0_LAYOUTS   1
#define NUM_MEM_S0_POOLS     4

#define NUM_MEM_LAYOUTS      1
#define NUM_MEM_POOLS        4


/*
 * Pool areas
 */
/* Section0 Layout0: */
#define MEMMGR_S0_L0_WORK_SIZE   0x00000060

#define S0_L0_SENSOR_DSP_CMD_BUF_POOL_ALIGN    0x00000008
#define S0_L0_SENSOR_DSP_CMD_BUF_POOL_ADDR     0x000e0000
#define S0_L0_SENSOR_DSP_CMD_BUF_POOL_SIZE     0x00000380
#define S0_L0_SENSOR_DSP_CMD_BUF_POOL_NUM_SEG  0x00000008
#define S0_L0_SENSOR_DSP_CMD_BUF_POOL_SEG_SIZE 0x00000070

#define S0_L0_ACCEL_DATA_BUF_POOL_ALIGN    0x00000008
#define S0_L0_ACCEL_DATA_BUF_POOL_ADDR     0x000e0380
#define S0_L0_ACCEL_DATA_BUF_POOL_SIZE     0x00000c00
#define S0_L0_ACCEL_DATA_BUF_POOL_NUM_SEG  0x00000008
#define S0_L0_ACCEL_DATA_BUF_POOL_SEG_SIZE 0x00000180

#define S0_L0_GNSS_DATA_BUF_POOL_ALIGN    0x00000008
#define S0_L0_GNSS_DATA_BUF_POOL_ADDR     0x000e0f80
#define S0_L0_GNSS_DATA_BUF_POOL_SIZE     0x00000180
#define S0_L0_GNSS_DATA_BUF_POOL_NUM_SEG  0x00000008
#define S0_L0_GNSS_DATA_BUF_POOL_SEG_SIZE 0x00000030

/* Remainder SENSOR_WORK_AREA=0x0001cf00 */

#endif /* MEM_LAYOUT_H_INCLUDED */
