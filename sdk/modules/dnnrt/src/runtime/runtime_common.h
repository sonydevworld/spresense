/****************************************************************************
 * modules/dnnrt/src/runtime/runtime_common.h
 *
 *   Copyright 2018 Sony Corporation
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
 * 3. Neither the name of Sony Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#ifndef RUNTIME_COMMON_H
#  define RUNTIME_COMMON_H

#  include <nuttx/config.h>
#  include <errno.h>
#  include <debug.h>
#  include <nnablart/functions.h>
#  include <nnablart/runtime.h>

#  ifdef __cplusplus
extern "C"
{
#  endif

#  define dnn_info(x...) _info(x)
#  define dnn_err(x...) _err(x)

#  define DNN_CHECK_NULL_RET(b, r)                                          \
  do {                                                                      \
    if ((b) == NULL) {                                                      \
      dnn_err("Null-check failed. %s()@%s:L%d\n", __FUNCTION__, __FILE__,   \
              __LINE__);                                                    \
      return r;                                                             \
    }                                                                       \
  } while (0)

/* dnnrt does NOT support rt_context_t with buffer variables
   with more than MAX_VBUFFER_NUM */
#  define MAX_VBUFFER_NUM  (16)

  /* structure to manage shared_chunks, which underlie
   * variable buffers in dnn_runtime_t. */
  struct dnn_shared_chunk;
  typedef struct dnn_shared_chunk dnn_shared_chunk_t;
  struct dnn_shared_chunk
  {
    void *data;                 /* start address of this chunk */
    size_t allocated_bsize;     /* size of a buffer to which
                                 * dnn_shared_chunk_t::data points */
    size_t used_bsize;          /* size of already used portion in the buffer
                                 * to which dnn_shared_chunk_t::data points */
    uint8_t ref_count;          /* reference counter of this chunk */
    dnn_shared_chunk_t *next;   /* point to next shared_chunk in linked-list */
  };

  /* structure to hold information about how to allocate
   * dnn_shared_chunk_t::data to each rt_variable_buffer_context_t::buffer */
  typedef struct dnn_vbuffer_alloc_info dnn_vbuffer_alloc_info_t;
  struct dnn_vbuffer_alloc_info
  {
    size_t bsize_list[MAX_VBUFFER_NUM]; /* size of each variable buffer in bytes */
    void *addr_list[MAX_VBUFFER_NUM];   /* address of pre-allocated buffer */
    size_t vbuffer_num;         /* length of bsize_list/addr_list */
    uint8_t actual_alloc_count; /* how many times to allocate a shared_chunk to
                                 * variable buffers in rt_initialize_context() */
  };

  typedef struct dnn_global_context
  {
    int rt_count;
    int req_scratch_buf_bsize;
    int scratch_buf_bsize;
    void *scratch_buf;
    dnn_shared_chunk_t *chunks;
    dnn_vbuffer_alloc_info_t *alloc_info;       /* allocation info of current
                                                 * network. the alloc_info is
                                                 * placed on stack of
                                                 * dnn_runtime_initialize() for
                                                 * memory saving, so it
                                                 * shouldn't be access after
                                                 * dnn_runtime_initialize()
                                                 * stack frame inactive */
  } dnn_global_context_t;

  dnn_global_context_t *dnn_get_global_context(void);
  rt_function_error_t dnnrt_exec_convolution(rt_function_t * f);
  rt_function_error_t dnnrt_exec_affine(rt_function_t * f);
  rt_return_value_t dnnrt_affine_alloc(nn_network_t * net,
                                       void *function_context);
  rt_return_value_t dnnrt_convolution_alloc(nn_network_t * net,
                                            void *function_context);

  void dnn_req_scratch_buf(int size);
  void *dnn_scratch_buf(void);

  int dnn_peek_vbuffers(const nn_network_t * net,
                        dnn_vbuffer_alloc_info_t * alloc_info);
  void dnn_reset_chunk_usage(dnn_global_context_t * ctx);
  int dnn_preallocate_chunks(dnn_global_context_t * ctx,
                             dnn_vbuffer_alloc_info_t * alloc_info);
  void dnn_deallocate_chunks(dnn_global_context_t * ctx,
                             dnn_vbuffer_alloc_info_t * alloc_info);
  void dnn_destroy_unused_chunks(dnn_global_context_t * ctx);
  void *dnn_variable_malloc(size_t size);
  void dnn_variable_free(void *p);

#  ifdef __cplusplus
}
#  endif

#endif                          /* RUNTIME_COMMON_H */
