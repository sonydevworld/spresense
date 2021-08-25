/****************************************************************************
 * modules/dnnrt/src-mpcomm/worker/dnnrt-mp/dnn_exec_function.h
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

#ifndef _DNNRT_MPCOMM_WORKER_DNN_EXEC_FUNCTION_H_
#define _DNNRT_MPCOMM_WORKER_DNN_EXEC_FUNCTION_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nnablart/network.h>
#include <nnablart/runtime.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct dnn_task
{
  int begin;
  int end;
} dnn_task_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

rt_function_error_t dnnrt_exec_convolution(rt_function_t *f,
                                           int begin, int end);
rt_function_error_t dnnrt_exec_affine(rt_function_t *f, int begin, int end);
rt_function_error_t dnn_controller_exec_affine(rt_function_t *f);
rt_function_error_t dnn_controller_exec_convolution(rt_function_t *f);
rt_return_value_t dnnrt_affine_alloc(nn_network_t *net,
                                     void *function_context);
rt_return_value_t dnnrt_convolution_alloc(nn_network_t *net,
                                          void *function_context);

#endif /* _DNNRT_MPCOMM_WORKER_DNN_EXEC_FUNCTION_H_ */
