/* This file is generated automatically. */
/****************************************************************************
 * msgq_id.h
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

#ifndef MSGQ_ID_H_INCLUDED
#define MSGQ_ID_H_INCLUDED

/* Message area size: 4844 bytes */

#define MSGQ_TOP_DRM 0xfd000
#define MSGQ_END_DRM 0xfe2ec

/* Message area fill value after message poped */

#define MSG_FILL_VALUE_AFTER_POP 0x0

/* Message parameter type match check */

#define MSG_PARAM_TYPE_MATCH_CHECK false

/* Message queue pool IDs */

#define MSGQ_NULL 0
#define MSGQ_AUD_MNG 1
#define MSGQ_AUD_APP 2
#define MSGQ_AUD_DSP 3
#define MSGQ_AUD_PFDSP0 4
#define MSGQ_AUD_PLY 5
#define MSGQ_AUD_OUTPUT_MIX 6
#define MSGQ_AUD_RND_PLY 7
#define MSGQ_AUD_RND_PLY_SYNC 8
#define NUM_MSGQ_POOLS 9

/* User defined constants */

/************************************************************************/
#define MSGQ_AUD_MNG_QUE_BLOCK_DRM 0xfd044
#define MSGQ_AUD_MNG_N_QUE_DRM 0xfd264
#define MSGQ_AUD_MNG_N_SIZE 88
#define MSGQ_AUD_MNG_N_NUM 30
#define MSGQ_AUD_MNG_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_MNG_H_SIZE 0
#define MSGQ_AUD_MNG_H_NUM 0
/************************************************************************/
#define MSGQ_AUD_APP_QUE_BLOCK_DRM 0xfd088
#define MSGQ_AUD_APP_N_QUE_DRM 0xfdcb4
#define MSGQ_AUD_APP_N_SIZE 64
#define MSGQ_AUD_APP_N_NUM 2
#define MSGQ_AUD_APP_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_APP_H_SIZE 0
#define MSGQ_AUD_APP_H_NUM 0
/************************************************************************/
#define MSGQ_AUD_DSP_QUE_BLOCK_DRM 0xfd0cc
#define MSGQ_AUD_DSP_N_QUE_DRM 0xfdd34
#define MSGQ_AUD_DSP_N_SIZE 20
#define MSGQ_AUD_DSP_N_NUM 5
#define MSGQ_AUD_DSP_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_DSP_H_SIZE 0
#define MSGQ_AUD_DSP_H_NUM 0
/************************************************************************/
#define MSGQ_AUD_PFDSP0_QUE_BLOCK_DRM 0xfd110
#define MSGQ_AUD_PFDSP0_N_QUE_DRM 0xfdd98
#define MSGQ_AUD_PFDSP0_N_SIZE 20
#define MSGQ_AUD_PFDSP0_N_NUM 5
#define MSGQ_AUD_PFDSP0_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_PFDSP0_H_SIZE 0
#define MSGQ_AUD_PFDSP0_H_NUM 0
/************************************************************************/
#define MSGQ_AUD_PLY_QUE_BLOCK_DRM 0xfd154
#define MSGQ_AUD_PLY_N_QUE_DRM 0xfddfc
#define MSGQ_AUD_PLY_N_SIZE 48
#define MSGQ_AUD_PLY_N_NUM 5
#define MSGQ_AUD_PLY_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_PLY_H_SIZE 0
#define MSGQ_AUD_PLY_H_NUM 0
/************************************************************************/
#define MSGQ_AUD_OUTPUT_MIX_QUE_BLOCK_DRM 0xfd198
#define MSGQ_AUD_OUTPUT_MIX_N_QUE_DRM 0xfdeec
#define MSGQ_AUD_OUTPUT_MIX_N_SIZE 48
#define MSGQ_AUD_OUTPUT_MIX_N_NUM 8
#define MSGQ_AUD_OUTPUT_MIX_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_OUTPUT_MIX_H_SIZE 0
#define MSGQ_AUD_OUTPUT_MIX_H_NUM 0
/************************************************************************/
#define MSGQ_AUD_RND_PLY_QUE_BLOCK_DRM 0xfd1dc
#define MSGQ_AUD_RND_PLY_N_QUE_DRM 0xfe06c
#define MSGQ_AUD_RND_PLY_N_SIZE 32
#define MSGQ_AUD_RND_PLY_N_NUM 16
#define MSGQ_AUD_RND_PLY_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_RND_PLY_H_SIZE 0
#define MSGQ_AUD_RND_PLY_H_NUM 0
/************************************************************************/
#define MSGQ_AUD_RND_PLY_SYNC_QUE_BLOCK_DRM 0xfd220
#define MSGQ_AUD_RND_PLY_SYNC_N_QUE_DRM 0xfe26c
#define MSGQ_AUD_RND_PLY_SYNC_N_SIZE 16
#define MSGQ_AUD_RND_PLY_SYNC_N_NUM 8
#define MSGQ_AUD_RND_PLY_SYNC_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_RND_PLY_SYNC_H_SIZE 0
#define MSGQ_AUD_RND_PLY_SYNC_H_NUM 0

#endif /* MSGQ_ID_H_INCLUDED */
