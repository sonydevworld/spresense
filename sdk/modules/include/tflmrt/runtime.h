/****************************************************************************
 * modules/include/tflmrt/runtime.h
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

/**
 * @file runtime.h
 */

#ifndef __MODULES_INCLUDE_TFLMRT_RUNTIME_H
#define __MODULES_INCLUDE_TFLMRT_RUNTIME_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TF_LITE_STATIC_MEMORY

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <asmp/types.h>

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/spresense/debug_log_callback.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @typedef tflm_config_t
 * structure to hold tflmrt runtime information
 */

typedef struct tflm_runtime
{
  void *impl_ctx;
} tflm_runtime_t;

/**
 * @typedef tflm_config_t
 * structure to configure tflmrt subsystem
 */

typedef struct tflm_config
{
  unsigned char cpu_num; /**< Number of CPUs involved in forward propagation */
} tflm_config_t;

/**
 * @typedef tflm_mallinfo_t
 * structure to obtain memory allocation information
 */

typedef struct tflm_mallinfo
{
  cpuid_t cpu;
  size_t total_bytes;
  size_t used_bytes;
  size_t largest_bytes;
} tflm_mallinfo_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * Initialize the whole tflmrt subsystem
 *
 * @param [in] config: configuration of multicore processing. <br>
 *                     If CONFIG_TFLM_RT_MPCOMM=y, tflm_config_t::cpu_num
 *                     must be 1 or more, <br>
 *                     otherwise tflm_config_t::cpu_num must be 1.
 *
 * @return 0 on success, otherwise returns error code in errno_t.
 *
 * @note this function must be called before any tflm_runtime_t
 *       object is initialized.
 */

int tflm_initialize(tflm_config_t *config);

/**
 * Finalize the whole tflmrt subsystem.
 * This function frees all the resources allocated through tflm_initialize().
 *
 * @return 0 on success, otherwise returns error code in errno_t.
 *
 * @note this function must be called after all the tflm_runtime_t
 *       is finalized
 */

int tflm_finalize(void);

/**
 * Instantiate a neural network as a tflm_runtime_t object.
 *
 * @param [in,out] rt:      tflmrt_runtime_t object
 * @param [in]     network: pointer to a memory into which network is loaded
 * @param [in]     size:    tensor arena size
 *
 * @return 0 on success, otherwise returns error code in errno_t.
 *
 * @note This function binds tflm_runtime_t and network, <br>
 *       so applications don't have to give the network object to the other
 *       functions except this. <br>
 *       However, the runtime holds reference to the network object. <br>
 *       Applications must NOT free it until tflm_runtime_finalize().
 */

int tflm_runtime_initialize(tflm_runtime_t *rt,
                            const void *network,
                            int size);

/**
 * Free all the memory allocated to a tflm_runtime_t object
 *
 * @param [in,out] rt:      tflmrt_runtime_t object
 *
 * @return 0 on success, otherwise returns error code in errno_t.
 *
 * @note tflmrt are NOT involved in freeing nn_network_t.
 */

int tflm_runtime_finalize(tflm_runtime_t *rt);

/**
 * Execute forward propagation after feeding input data.
 *
 * @param [in,out] rt:        tflmrt_runtime_t object
 * @param [in]     inputs:    an array of pointers to input buffers
 * @param [in]     input_num: length of inputs
 *
 * @return 0 on success, otherwise returns error code in errno_t.
 * @note feed input data taking the following points into account:
 *   - input_num equals to tflm_runtime_input_num()
 *   - length of each input buffer(i.e. inputs[i]) equals to
 *     tflm_runtime_input_size(rt, i)
 *   - internal representation of each input buffer(i.e. inputs[i])
 *     can be obtained by tflm_runtime_input_variable(rt, i)
 */

int tflm_runtime_forward(tflm_runtime_t *rt, const void *inputs[],
                        unsigned char input_num);

/**
 * Return the number of inputs which this network needs.
 *
 * @param [in,out] rt:      tflmrt_runtime_t object
 *
 * @return number of inputs on success, otherwise -EINVAL.
 */

int tflm_runtime_input_num(tflm_runtime_t *rt);

/**
 * Return the number of elements in a specified input.
 *
 * @param [in,out] rt:          tflmrt_runtime_t object
 * @param [in]     input_index: index to specify an input
 *
 * @return number of elements in the specified input
 *         on success, otherwise -EINVAL.
 */

int tflm_runtime_input_size(tflm_runtime_t *rt, unsigned char input_index);

/**
 * Return the number of a specified input's dimensions.
 *
 * @param [in,out] rt:          tflmrt_runtime_t object
 * @param [in]     input_index: index to specify an input
 *
 * @return number of the specified input's dimensions on success,
 *         otherwise -EINVAL.
 */

int tflm_runtime_input_ndim(tflm_runtime_t *rt, unsigned char input_index);

/**
 * Return the number of elements in a specified dimension of an input.
 *
 * @param [in,out] rt:           tflmrt_runtime_t object
 * @param [in]     input_index:  index to specify an input
 * @param [in]     dim_index:    index to specify a dimension
 *
 * @return number of elements in the specified dimension of the input
 *         on success, otherwise -EINVAL.
 */

int tflm_runtime_input_shape(tflm_runtime_t *rt, unsigned char input_index,
                            unsigned char dim_index);

/**
 * Get TfLiteTensor* corresponding to a specified input
 *
 * @param [in,out] rt:          tflmrt_runtime_t object
 * @param [in]     input_index: index to specify an input
 *
 * @return pointer to TfLiteTensor on success,
 *         otherwise NULL
 */

TfLiteTensor *tflm_runtime_input_variable(tflm_runtime_t *rt,
                                          unsigned char input_index);

/**
 * Return the number of outputs which this network emits.
 *
 * @param [in,out] rt:      tflmrt_runtime_t object
 *
 * @return number of outputs on success, otherwise -EINVAL.
 */

int tflm_runtime_output_num(tflm_runtime_t *rt);

/**
 * Return the number of elements in a specified output.
 *
 * @param [in,out] rt:           tflmrt_runtime_t object
 * @param [in]     output_index: index to specify an output
 *
 * @return number of elements in the specified output
 *         on success, otherwise -EINVAL.
 */

int tflm_runtime_output_size(tflm_runtime_t *rt, unsigned char output_index);

/**
 * Return the number of a specified output's dimensions.
 *
 * @param [in,out] rt:           tflmrt_runtime_t object
 * @param [in]     output_index: index to specify an input
 *
 * @return number of the specified input's dimensions on success,
 *         otherwise -EINVAL.
 */

int tflm_runtime_output_ndim(tflm_runtime_t *rt, unsigned char output_index);

/**
 * Return the number of elements in a specified dimension of an output.
 *
 * @param [in,out] rt:            tflmrt_runtime_t object
 * @param [in]     output_index:  index to specify an output
 * @param [in]     dim_index:     index to specify a dimension
 *
 * @return number of elements in the specified dimension of the output
 *         on success, otherwise -EINVAL.
 */

int tflm_runtime_output_shape(tflm_runtime_t *rt, unsigned char output_index,
                              unsigned char dim_index);

/**
 * Get TfLiteTensor* corresponding to a specified output
 *
 * @param [in,out] rt:           tflmrt_runtime_t object
 * @param [in]     output_index: index to specify an output
 *
 * @return pointer to TfLiteTensor on success,
 *         otherwise NULL
 */

TfLiteTensor *tflm_runtime_output_variable(tflm_runtime_t *rt,
                                           unsigned char output_index);

/**
 * Return a pointer to a specified output
 *
 * @param [in,out] rt:           tflmrt_runtime_t object
 * @param [in]     output_index: index to specify an output
 * @return pointer to corresponding the output buffer if
 *         output_index is valid, otherwise NULL.
 * @note read output data taking the following points into account:
 *   - output_index must be less than tflm_runtime_output_num(rt)
 *   - length of the output buffer equals to
 *     tflm_runtime_output_size(rt, output_index)
 *   - internal representation of the output buffer can be obtained
 *     by tflm_runtime_output_variable(rt, output_index)
 */

void *tflm_runtime_output_buffer(tflm_runtime_t *rt,
                                 unsigned char output_index);

/**
 * Obtain information about memory allocation in the Nuttx-side heap
 *
 * @param [out] info: pointer to store memory allocation stats
 *
 * @return 0 on success. otherwise -EINVAL.
 */

int tflm_nuttx_mallinfo(tflm_mallinfo_t *info);

/**
 * Obtain information about memory allocation in the ASMP-side heap
 *
 * @param [in]  array_length: Number of elements in info_array.<br>
 *                            This value must be tflm_config_t::cpu_num
 * @param [out] info_array:   Array to store memory allocation stats
 *
 * @return 0 on success. -EPERM if CONFIG_TFLM_RT_MPCOMM=n
 */

int tflm_asmp_mallinfo(unsigned char array_length,
                       tflm_mallinfo_t *info_array);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __MODULES_INCLUDE_TFLMRT_RUNTIME_H */
