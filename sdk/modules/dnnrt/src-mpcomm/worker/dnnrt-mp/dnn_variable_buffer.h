/****************************************************************************
 * modules/dnnrt/src-mpcomm/worker/dnnrt-mp/dnn_variable_buffer.h
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

#ifndef _DNNRT_MPCOMM_WORKER_DNN_VARIABLE_BUFFER_H_
#define _DNNRT_MPCOMM_WORKER_DNN_VARIABLE_BUFFER_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>

#include <nnablart/network.h>
#include <nnablart/runtime.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* dnnrt does NOT support rt_context_t with buffer variables
 * with more than MAX_VBUFFER_NUM
 */

#define MAX_VBUFFER_NUM (16)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* structure to manage shared_chunks, which underlie
 * variable buffers in dnn_runtime_t.
 */

struct dnn_shared_chunk;
typedef struct dnn_shared_chunk dnn_shared_chunk_t;
struct dnn_shared_chunk
{
  void *data;               /* start address of this chunk */
  size_t allocated_bsize;   /* size of a buffer to which
                             * dnn_shared_chunk_t::data points */
  size_t used_bsize;        /* size of already used portion in the buffer
                             * to which dnn_shared_chunk_t::data points */
  uint8_t ref_count;        /* reference counter of this chunk */
  dnn_shared_chunk_t *next; /* point to next shared_chunk in linked-list */
};

/* structure to hold information about how to allocate
 * dnn_shared_chunk_t::data to each rt_variable_buffer_context_t::buffer
 */

typedef struct dnn_vbuffer_alloc_info
{
  size_t bsize_list[MAX_VBUFFER_NUM]; /* size of each variable buffer in bytes */
  void *addr_list[MAX_VBUFFER_NUM];   /* address of pre-allocated buffer */
  size_t vbuffer_num;                 /* length of bsize_list/addr_list */
  uint8_t actual_alloc_count;         /* how many times to allocate a shared_chunk to
                                       * variable buffers in rt_initialize_context() */
} dnn_vbuffer_alloc_info_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void dnn_vbuffer_initialize(dnn_vbuffer_alloc_info_t *info);
int dnn_peek_vbuffers(const nn_network_t *net,
                      dnn_vbuffer_alloc_info_t *info);
void dnn_reset_chunk_usage(void);
int dnn_preallocate_chunks(dnn_vbuffer_alloc_info_t *info);
void dnn_deallocate_chunks(dnn_vbuffer_alloc_info_t *info);
void dnn_destroy_unused_chunks(void);
void *dnn_variable_malloc(size_t size);
void dnn_variable_free(void *p);

#endif /* _DNNRT_MPCOMM_WORKER_DNN_VARIABLE_BUFFER_H_ */
