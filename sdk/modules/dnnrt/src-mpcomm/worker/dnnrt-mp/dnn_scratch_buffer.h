/****************************************************************************
 * modules/dnnrt/src-mpcomm/worker/dnnrt-mp/dnn_scratch_buffer.h
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

#ifndef _DNNRT_MPCOMM_WORKER_DNN_SCRATCH_BUFFER_H_
#define _DNNRT_MPCOMM_WORKER_DNN_SCRATCH_BUFFER_H_

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_HELPERS_NUM (4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct dnn_scratch_buffer
{
  int requested_size;
  int buffer_size;
  void *buffer_addr[MAX_HELPERS_NUM + 1];
} dnn_scratch_buffer_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int dnn_scratch_buffer_create(void);
int dnn_scratch_buffer_destroy(void);
void dnn_scratch_buffer_request_size(int size);
void *dnn_scratch_buffer_get(void);
void *dnn_scratch_buffer_get_index(uint8_t index);
void dnn_scratch_buffer_set(void *buffer);

#endif /* _DNNRT_MPCOMM_WORKER_DNN_SCRATCH_BUFFER_H_ */
