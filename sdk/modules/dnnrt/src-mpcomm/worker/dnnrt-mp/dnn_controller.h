/****************************************************************************
 * modules/dnnrt/src-mpcomm/worker/dnnrt-mp/dnn_controller.h
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

#ifndef _DNNRT_MPCOMM_WORKER_DNN_CONTROLLER_H_
#define _DNNRT_MPCOMM_WORKER_DNN_CONTROLLER_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "dnn_variable_buffer.h"
#include "dnn_scratch_buffer.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct dnn_runtime
{
  void *impl_ctx;
} dnn_runtime_t;

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
} dnn_msg_id_t;

typedef enum
{
  DNN_HELPER_MSG_INIT,
  DNN_HELPER_EXEC_AFFINE,
  DNN_HELPER_EXEC_CONVOLUTION,
} dnn_helper_msg_id_t;

typedef struct dnn_msg
{
  uint8_t id;
  int arg[3];
  int ret;
} dnn_msg_t;

typedef struct dnn_controller_context
{
  uint8_t helpers_num;
  dnn_shared_chunk_t *chunks;
  dnn_vbuffer_alloc_info_t *alloc_info;
  void *helpers_exec_function[MAX_HELPERS_NUM];
} dnn_controller_context_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void dnn_controller_handle_msg(void *data);
void *dnn_controller_malloc(size_t size);
void dnn_controller_free(void *ptr);

#endif /* _DNNRT_MPCOMM_WORKER_DNN_CONTROLLER_H_ */
