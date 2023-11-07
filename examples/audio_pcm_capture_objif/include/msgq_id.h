/* This file is generated automatically. */
/****************************************************************************
 * msgq_id.h
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

#ifndef MSGQ_ID_H_INCLUDED
#define MSGQ_ID_H_INCLUDED

/* Message area size: 6312 bytes */

#define MSGQ_TOP_DRM 0xfd000
#define MSGQ_END_DRM 0xfe8a8

/* Message area fill value after message poped */

#define MSG_FILL_VALUE_AFTER_POP 0x0

/* Message parameter type match check */

#define MSG_PARAM_TYPE_MATCH_CHECK false

/* Message queue pool IDs */

#define MSGQ_NULL 0
#define MSGQ_AUD_APP 1
#define MSGQ_AUD_MGR 2
#define MSGQ_AUD_CAP 3
#define MSGQ_AUD_CAP_SYNC 4
#define MSGQ_AUD_FRONTEND 5
#define MSGQ_AUD_PREDSP 6
#define NUM_MSGQ_POOLS 7

/* User defined constants */

/************************************************************************/
#define MSGQ_AUD_APP_QUE_BLOCK_DRM 0xfd04c
#define MSGQ_AUD_APP_N_QUE_DRM 0xfd214
#define MSGQ_AUD_APP_N_SIZE 64
#define MSGQ_AUD_APP_N_NUM 32
#define MSGQ_AUD_APP_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_APP_H_SIZE 0
#define MSGQ_AUD_APP_H_NUM 0
/************************************************************************/
#define MSGQ_AUD_MGR_QUE_BLOCK_DRM 0xfd098
#define MSGQ_AUD_MGR_N_QUE_DRM 0xfda14
#define MSGQ_AUD_MGR_N_SIZE 88
#define MSGQ_AUD_MGR_N_NUM 30
#define MSGQ_AUD_MGR_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_MGR_H_SIZE 0
#define MSGQ_AUD_MGR_H_NUM 0
/************************************************************************/
#define MSGQ_AUD_CAP_QUE_BLOCK_DRM 0xfd0e4
#define MSGQ_AUD_CAP_N_QUE_DRM 0xfe464
#define MSGQ_AUD_CAP_N_SIZE 24
#define MSGQ_AUD_CAP_N_NUM 16
#define MSGQ_AUD_CAP_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_CAP_H_SIZE 0
#define MSGQ_AUD_CAP_H_NUM 0
/************************************************************************/
#define MSGQ_AUD_CAP_SYNC_QUE_BLOCK_DRM 0xfd130
#define MSGQ_AUD_CAP_SYNC_N_QUE_DRM 0xfe5e4
#define MSGQ_AUD_CAP_SYNC_N_SIZE 16
#define MSGQ_AUD_CAP_SYNC_N_NUM 8
#define MSGQ_AUD_CAP_SYNC_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_CAP_SYNC_H_SIZE 0
#define MSGQ_AUD_CAP_SYNC_H_NUM 0
/************************************************************************/
#define MSGQ_AUD_FRONTEND_QUE_BLOCK_DRM 0xfd17c
#define MSGQ_AUD_FRONTEND_N_QUE_DRM 0xfe664
#define MSGQ_AUD_FRONTEND_N_SIZE 48
#define MSGQ_AUD_FRONTEND_N_NUM 10
#define MSGQ_AUD_FRONTEND_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_FRONTEND_H_SIZE 0
#define MSGQ_AUD_FRONTEND_H_NUM 0
/************************************************************************/
#define MSGQ_AUD_PREDSP_QUE_BLOCK_DRM 0xfd1c8
#define MSGQ_AUD_PREDSP_N_QUE_DRM 0xfe844
#define MSGQ_AUD_PREDSP_N_SIZE 20
#define MSGQ_AUD_PREDSP_N_NUM 5
#define MSGQ_AUD_PREDSP_H_QUE_DRM 0xffffffff
#define MSGQ_AUD_PREDSP_H_SIZE 0
#define MSGQ_AUD_PREDSP_H_NUM 0

#endif /* MSGQ_ID_H_INCLUDED */
