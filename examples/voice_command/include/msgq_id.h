/****************************************************************************
 * voice_command/include/msgq_id.h
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

#ifndef MSGQ_ID_H_INCLUDED
#define MSGQ_ID_H_INCLUDED

/* Message area size: 4512 bytes */
#define MSGQ_TOP_DRM	0xfc000
#define MSGQ_END_DRM	0xfd1a0

/* Message area fill value after message poped */
#define MSG_FILL_VALUE_AFTER_POP	0x0

/* Message parameter type match check */
#define MSG_PARAM_TYPE_MATCH_CHECK	false

/* Message queue pool IDs */
#define MSGQ_NULL	0
#define MSGQ_AUD_MGR	1
#define MSGQ_AUD_APP	2
#define MSGQ_AUD_DSP	3
#define MSGQ_AUD_OUTPUT_MIX	4
#define MSGQ_AUD_SOUND_EFFECT	5
#define MSGQ_AUD_RCG_CMD	6
#define MSGQ_AUD_CAP_MIC	7
#define MSGQ_AUD_CAP_MIC_SYNC	8
#define MSGQ_AUD_CAP_I2S	9
#define MSGQ_AUD_CAP_I2S_SYNC	10
#define MSGQ_AUD_RND_SPHP	11
#define MSGQ_AUD_RND_SPHP_SYNC	12
#define MSGQ_AUD_RND_I2S	13
#define MSGQ_AUD_RND_I2S_SYNC	14
#define NUM_MSGQ_POOLS	15

/* User defined constants */

/************************************************************************/
#define MSGQ_AUD_MGR_QUE_BLOCK_DRM	0xfc044
#define MSGQ_AUD_MGR_N_QUE_DRM	0xfc3fc
#define MSGQ_AUD_MGR_N_SIZE	88
#define MSGQ_AUD_MGR_N_NUM	3
#define MSGQ_AUD_MGR_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_MGR_H_SIZE	0
#define MSGQ_AUD_MGR_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_APP_QUE_BLOCK_DRM	0xfc088
#define MSGQ_AUD_APP_N_QUE_DRM	0xfc504
#define MSGQ_AUD_APP_N_SIZE	40
#define MSGQ_AUD_APP_N_NUM	2
#define MSGQ_AUD_APP_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_APP_H_SIZE	0
#define MSGQ_AUD_APP_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_DSP_QUE_BLOCK_DRM	0xfc0cc
#define MSGQ_AUD_DSP_N_QUE_DRM	0xfc554
#define MSGQ_AUD_DSP_N_SIZE	20
#define MSGQ_AUD_DSP_N_NUM	5
#define MSGQ_AUD_DSP_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_DSP_H_SIZE	0
#define MSGQ_AUD_DSP_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_OUTPUT_MIX_QUE_BLOCK_DRM	0xfc110
#define MSGQ_AUD_OUTPUT_MIX_N_QUE_DRM	0xfc5b8
#define MSGQ_AUD_OUTPUT_MIX_N_SIZE	48
#define MSGQ_AUD_OUTPUT_MIX_N_NUM	8
#define MSGQ_AUD_OUTPUT_MIX_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_OUTPUT_MIX_H_SIZE	0
#define MSGQ_AUD_OUTPUT_MIX_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_SOUND_EFFECT_QUE_BLOCK_DRM	0xfc154
#define MSGQ_AUD_SOUND_EFFECT_N_QUE_DRM	0xfc738
#define MSGQ_AUD_SOUND_EFFECT_N_SIZE	52
#define MSGQ_AUD_SOUND_EFFECT_N_NUM	5
#define MSGQ_AUD_SOUND_EFFECT_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_SOUND_EFFECT_H_SIZE	0
#define MSGQ_AUD_SOUND_EFFECT_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_RCG_CMD_QUE_BLOCK_DRM	0xfc198
#define MSGQ_AUD_RCG_CMD_N_QUE_DRM	0xfc83c
#define MSGQ_AUD_RCG_CMD_N_SIZE	20
#define MSGQ_AUD_RCG_CMD_N_NUM	5
#define MSGQ_AUD_RCG_CMD_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_RCG_CMD_H_SIZE	0
#define MSGQ_AUD_RCG_CMD_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_CAP_MIC_QUE_BLOCK_DRM	0xfc1dc
#define MSGQ_AUD_CAP_MIC_N_QUE_DRM	0xfc8a0
#define MSGQ_AUD_CAP_MIC_N_SIZE	24
#define MSGQ_AUD_CAP_MIC_N_NUM	16
#define MSGQ_AUD_CAP_MIC_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_CAP_MIC_H_SIZE	0
#define MSGQ_AUD_CAP_MIC_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_CAP_MIC_SYNC_QUE_BLOCK_DRM	0xfc220
#define MSGQ_AUD_CAP_MIC_SYNC_N_QUE_DRM	0xfca20
#define MSGQ_AUD_CAP_MIC_SYNC_N_SIZE	16
#define MSGQ_AUD_CAP_MIC_SYNC_N_NUM	8
#define MSGQ_AUD_CAP_MIC_SYNC_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_CAP_MIC_SYNC_H_SIZE	0
#define MSGQ_AUD_CAP_MIC_SYNC_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_CAP_I2S_QUE_BLOCK_DRM	0xfc264
#define MSGQ_AUD_CAP_I2S_N_QUE_DRM	0xfcaa0
#define MSGQ_AUD_CAP_I2S_N_SIZE	24
#define MSGQ_AUD_CAP_I2S_N_NUM	16
#define MSGQ_AUD_CAP_I2S_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_CAP_I2S_H_SIZE	0
#define MSGQ_AUD_CAP_I2S_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_CAP_I2S_SYNC_QUE_BLOCK_DRM	0xfc2a8
#define MSGQ_AUD_CAP_I2S_SYNC_N_QUE_DRM	0xfcc20
#define MSGQ_AUD_CAP_I2S_SYNC_N_SIZE	16
#define MSGQ_AUD_CAP_I2S_SYNC_N_NUM	8
#define MSGQ_AUD_CAP_I2S_SYNC_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_CAP_I2S_SYNC_H_SIZE	0
#define MSGQ_AUD_CAP_I2S_SYNC_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_RND_SPHP_QUE_BLOCK_DRM	0xfc2ec
#define MSGQ_AUD_RND_SPHP_N_QUE_DRM	0xfcca0
#define MSGQ_AUD_RND_SPHP_N_SIZE	32
#define MSGQ_AUD_RND_SPHP_N_NUM	16
#define MSGQ_AUD_RND_SPHP_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_RND_SPHP_H_SIZE	0
#define MSGQ_AUD_RND_SPHP_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_RND_SPHP_SYNC_QUE_BLOCK_DRM	0xfc330
#define MSGQ_AUD_RND_SPHP_SYNC_N_QUE_DRM	0xfcea0
#define MSGQ_AUD_RND_SPHP_SYNC_N_SIZE	16
#define MSGQ_AUD_RND_SPHP_SYNC_N_NUM	8
#define MSGQ_AUD_RND_SPHP_SYNC_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_RND_SPHP_SYNC_H_SIZE	0
#define MSGQ_AUD_RND_SPHP_SYNC_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_RND_I2S_QUE_BLOCK_DRM	0xfc374
#define MSGQ_AUD_RND_I2S_N_QUE_DRM	0xfcf20
#define MSGQ_AUD_RND_I2S_N_SIZE	32
#define MSGQ_AUD_RND_I2S_N_NUM	16
#define MSGQ_AUD_RND_I2S_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_RND_I2S_H_SIZE	0
#define MSGQ_AUD_RND_I2S_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_RND_I2S_SYNC_QUE_BLOCK_DRM	0xfc3b8
#define MSGQ_AUD_RND_I2S_SYNC_N_QUE_DRM	0xfd120
#define MSGQ_AUD_RND_I2S_SYNC_N_SIZE	16
#define MSGQ_AUD_RND_I2S_SYNC_N_NUM	8
#define MSGQ_AUD_RND_I2S_SYNC_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_RND_I2S_SYNC_H_SIZE	0
#define MSGQ_AUD_RND_I2S_SYNC_H_NUM	0
#endif /* MSGQ_ID_H_INCLUDED */
