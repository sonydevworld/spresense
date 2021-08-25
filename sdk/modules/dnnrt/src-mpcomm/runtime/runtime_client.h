/****************************************************************************
 * modules/dnnrt/src-mpcomm/runtime/runtime_client.h
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

#ifndef _DNNRT_MPCOMM_RUNTIME_RUNTIME_CLIENT_H_
#define _DNNRT_MPCOMM_RUNTIME_RUNTIME_CLIENT_H_

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  DNNRT_MSG_NRT_INIT,
  DNNRT_MSG_NRT_FINI,
  DNNRT_MSG_NRT_FOWARD,
  DNNRT_MSG_NRT_INPUT_NUM,
  DNNRT_MSG_NRT_INPUT_SIZE,
  DNNRT_MSG_NRT_INPUT_NDIM,
  DNNRT_MSG_NRT_INPUT_SHAPE,
  DNNRT_MSG_NRT_INPUT_VARIABLE,
  DNNRT_MSG_NRT_OUTPUT_NUM,
  DNNRT_MSG_NRT_OUTPUT_SIZE,
  DNNRT_MSG_NRT_OUTPUT_NDIM,
  DNNRT_MSG_NRT_OUTPUT_SHAPE,
  DNNRT_MSG_NRT_OUTPUT_BUFFER,
  DNNRT_MSG_NRT_OUTPUT_VARIABLE,
  DNNRT_MSG_NRT_ASMP_MALLINFO,
} dnn_msg_id_t;

typedef struct dnn_msg
{
  dnn_msg_id_t id;
  int arg[3];
  int ret;
} dnn_msg_t;

#endif /* _DNNRT_MPCOMM_RUNTIME_RUNTIME_CLIENT_H_ */
