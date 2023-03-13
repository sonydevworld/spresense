/****************************************************************************
 * externals/tensorflow/c-runtime/tf_runtime.h
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

#ifndef __EXTERNALS_TENSORFLOW_C_RUNTIME_TF_RUNTIME_H
#define __EXTERNALS_TENSORFLOW_C_RUNTIME_TF_RUNTIME_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "tensorflow/lite/c/common.h"

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

typedef void *tf_rt_context_pointer;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int tf_rt_allocate_context(tf_rt_context_pointer *context);
int tf_rt_initialize_context(tf_rt_context_pointer context,
                             const void *n, int size);
int tf_rt_free_context(tf_rt_context_pointer *context);
int tf_rt_num_of_input(tf_rt_context_pointer context);
int tf_rt_input_size(tf_rt_context_pointer context, size_t index);
int tf_rt_input_dimension(tf_rt_context_pointer context, size_t index);
int tf_rt_input_shape(tf_rt_context_pointer context, size_t index,
                      size_t shape_index);
void *tf_rt_input_buffer(tf_rt_context_pointer context, size_t index);
int tf_rt_num_of_output(tf_rt_context_pointer context);
int tf_rt_output_size(tf_rt_context_pointer context, size_t index);
int tf_rt_output_dimension(tf_rt_context_pointer context, size_t index);
int tf_rt_output_shape(tf_rt_context_pointer context, size_t index,
                       size_t shape_index);
void *tf_rt_output_buffer(tf_rt_context_pointer context, size_t index);
TfLiteTensor *tf_rt_input_variable(tf_rt_context_pointer context,
                                   size_t index);
TfLiteTensor *tf_rt_output_variable(tf_rt_context_pointer context,
                                    size_t index);
int tf_rt_forward(tf_rt_context_pointer context);
void tf_rt_set_malloc(void *(*user_malloc)(size_t size));
void tf_rt_set_free(void (*user_free)(void *ptr));
size_t tf_rt_arenasize(tf_rt_context_pointer context);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __EXTERNALS_TENSORFLOW_C_RUNTIME_TF_RUNTIME_H */
