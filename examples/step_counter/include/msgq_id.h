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

/* Message area size: 456 bytes */
#define MSGQ_TOP_DRM 0xfe000
#define MSGQ_END_DRM 0xfe1c8

/* Message area fill value after message poped */
#define MSG_FILL_VALUE_AFTER_POP 0x0

/* Message parameter type match check */
#define MSG_PARAM_TYPE_MATCH_CHECK false

/* Message queue pool IDs */
#define MSGQ_NULL 0
#define MSGQ_SEN_MGR 1
#define NUM_MSGQ_POOLS 2

/* User defined constants */

/************************************************************************/
#define MSGQ_SEN_MGR_QUE_BLOCK_DRM 0xfe044
#define MSGQ_SEN_MGR_N_QUE_DRM 0xfe088
#define MSGQ_SEN_MGR_N_SIZE 40
#define MSGQ_SEN_MGR_N_NUM 8
#define MSGQ_SEN_MGR_H_QUE_DRM 0xffffffff
#define MSGQ_SEN_MGR_H_SIZE 0
#define MSGQ_SEN_MGR_H_NUM 0
#endif /* MSGQ_ID_H_INCLUDED */
