/****************************************************************************
 * modules/include/dnnrt/runtime.h
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

/**
 * @file runtime.h
 */

#ifndef __INCLUDE_DNNRT_RUNTIME_H
#  define __INCLUDE_DNNRT_RUNTIME_H

/**
 * @defgroup dnnrt dnnrt
 * @{
 *
 * dnnrt is an Deep Neural Networks RunTime optimized for for CXD5602
 */

#  include <asmp/types.h>
#  include <dnnrt/nnablart/network.h>

#  ifdef __cplusplus
#    define EXTERN extern "C"
extern "C"
{
#  else
#    define EXTERN extern
#  endif

#  define DNNRT_IMPLEMENT (0)

/**
 * @defgroup dnnrt_datatype Data Types
 * @{
 */
typedef struct dnn_runtime
{
  void *impl_ctx;
} dnn_runtime_t;

/**
 * @typedef dnn_config_t
 * structure to configure dnnrt subsystem
 */
typedef struct dnn_config
{

  unsigned char cpu_num; /**< Number of CPUs involved in forward propagation */
} dnn_config_t;

/**
 * @typedef dnn_mallinfo_t
 * structure to obtain memory allocation information
 */
typedef struct dnn_mallinfo
{
  cpuid_t cpu;
  size_t total_bytes;
  size_t used_bytes;
  size_t largest_bytes;
} dnn_mallinfo_t;

/** @} dnnrt_datatype */

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

/**
 * @defgroup dnnrt_funcs Functions
 * @{
 */

/**
 * Initialize the whole dnnrt subsystem
 *
 * @param [in] config: configuration of multicore processing. <br>
 *                     If CONFIG_DNN_RT_MP=y, dnn_config_t::cpu_num must be 1 or more, <br>
 *                     otherwise dnn_config_t::cpu_num must be 1.
 *
 * @return 0 on success, otherwise returns error code in errno_t.
 *
 * @note this function must be called before any dnn_runtime_t object is initialized.
 */
int dnn_initialize(dnn_config_t * config);

/**
 * Finalize the whole dnnrt subsystem.
 * This function frees all the resources allocated through dnn_initialize().
 *
 * @return 0 on success, otherwise returns error code in errno_t.
 *
 * @note this function must be called after all the dnn_runtime_t is finalized
 */
int dnn_finalize(void);

 /**
  * Instantiate a neural network defined by nn_network_t as a dnn_runtime_t object.
  *
  * @param [in,out] rt:      dnnrt_runtime_t object
  * @param [in]     network: pointer to a memory into which .nnb file is loaded
  *
  * @return 0 on success, otherwise returns error code in rt_return_value_t or errno_t.
  *
  * @note This function binds dnn_runtime_t and nn_network_t, <br>
  *       so applications don't have to give the network object to the other functions except this. <br>
  *       However, the runtime holds reference to the network object. <br>
  *       Applications must NOT free it until dnn_runtime_finalize().
  */
int dnn_runtime_initialize(dnn_runtime_t * rt, const nn_network_t * network);

 /**
  * Free all the memory allocated to a dnn_runtime_t object
  *
  * @param [in,out] rt:      dnnrt_runtime_t object
  *
  * @return 0 on success, otherwise returns error code in rt_return_value_t or errno_t.
  *
  * @note dnnrt are NOT involved in freeing nn_network_t.
  */
int dnn_runtime_finalize(dnn_runtime_t * rt);

/**
 * Execute forward propagation after feeding input data.
 *
 * @param [in,out] rt:        dnnrt_runtime_t object
 * @param [in]     inputs:    an array of pointers to input buffers
 * @param [in]     input_num: length of inputs
 *
 * @return 0 on success, otherwise returns error code in rt_return_value_t or errno_t.
 * @note feed input data taking the following points into account:
 *   - input_num equals to dnn_runtime_input_num()
 *   - length of each input buffer(i.e. inputs[i]) equals to dnn_runtime_input_size(rt, i)
 *   - internal representation of each input buffer(i.e. inputs[i]) can be obtained by dnn_runtime_input_variable(rt, i)
 */
int dnn_runtime_forward(dnn_runtime_t * rt, const void *inputs[],
                        unsigned char input_num);

/**
 * Return the number of inputs which this network needs.
 *
 * @param [in,out] rt:      dnnrt_runtime_t object
 *
 * @return number of inputs on success, otherwise -EINVAL.
 */
int dnn_runtime_input_num(dnn_runtime_t * rt);

/**
 * Return the number of elements in a specified input.
 *
 * @param [in,out] rt:          dnnrt_runtime_t object
 * @param [in]     input_index: index to specify an input
 *
 * @return number of elements in the specified input on success, otherwise -EINVAL.
 */
int dnn_runtime_input_size(dnn_runtime_t * rt, unsigned char input_index);

/**
 * Return the number of a specified input's dimensions.
 *
 * @param [in,out] rt:          dnnrt_runtime_t object
 * @param [in]     input_index: index to specify an input
 *
 * @return number of the specified input's dimensions on success,
 *         otherwise -EINVAL.
 */
int dnn_runtime_input_ndim(dnn_runtime_t * rt, unsigned char input_index);

/**
 * Return the number of elements in a specified dimension of an input.
 *
 * @param [in,out] rt:           dnnrt_runtime_t object
 * @param [in]     input_index:  index to specify an input
 * @param [in]     dim_index:    index to specify a dimension
 *
 * @return number of elements in the specified dimension of the input on success,
 *         otherwise -EINVAL.
 */
int dnn_runtime_input_shape(dnn_runtime_t * rt, unsigned char input_index,
                            unsigned char dim_index);

/**
 * Get nn_variable_t* corresponding to a specified input
 *
 * @param [in,out] rt:          dnnrt_runtime_t object
 * @param [in]     input_index: index to specify an input
 *
 * @return pointer to nn_variable_t in nn_network_t on success,
 *         otherwise NULL
 */
nn_variable_t *dnn_runtime_input_variable(dnn_runtime_t * rt,
                                          unsigned char input_index);

/**
 * Return the number of outputs which this network emits.
 *
 * @param [in,out] rt:      dnnrt_runtime_t object
 *
 * @return number of outputs on success, otherwise -EINVAL.
 */
int dnn_runtime_output_num(dnn_runtime_t * rt);

/**
 * Return the number of elements in a specified output.
 *
 * @param [in,out] rt:           dnnrt_runtime_t object
 * @param [in]     output_index: index to specify an output
 *
 * @return number of elements in the specified output on success, otherwise -EINVAL.
 */
int dnn_runtime_output_size(dnn_runtime_t * rt, unsigned char output_index);

/**
 * Return the number of a specified output's dimensions.
 *
 * @param [in,out] rt:           dnnrt_runtime_t object
 * @param [in]     output_index: index to specify an input
 *
 * @return number of the specified input's dimensions on success,
 *         otherwise -EINVAL.
 */
int dnn_runtime_output_ndim(dnn_runtime_t * rt, unsigned char output_index);

/**
 * Return the number of elements in a specified dimension of an output.
 *
 * @param [in,out] rt:            dnnrt_runtime_t object
 * @param [in]     output_index:  index to specify an output
 * @param [in]     dim_index:     index to specify a dimension
 *
 * @return number of elements in the specified dimension of the output on success,
 *         otherwise -EINVAL.
 */
int dnn_runtime_output_shape(dnn_runtime_t * rt, unsigned char output_index,
                             unsigned char dim_index);

/**
 * Get nn_variable_t* corresponding to a specified output
 *
 * @param [in,out] rt:           dnnrt_runtime_t object
 * @param [in]     output_index: index to specify an output
 *
 * @return pointer to nn_variable_t in nn_network_t on success,
 *         otherwise NULL
 */
nn_variable_t *dnn_runtime_output_variable(dnn_runtime_t * rt,
                                           unsigned char output_index);

/**
 * Return a pointer to a specified output
 *
 * @param [in,out] rt:           dnnrt_runtime_t object
 * @param [in]     output_index: index to specify an output
 * @return pointer to corresponding the output buffer if output_index is valid,
 *         otherwise NULL.
 * @note read output data taking the following points into account:
 *   - output_index must be less than dnn_runtime_output_num(rt)
 *   - length of the output buffer equals to dnn_runtime_output_size(rt, output_index)
 *   - internal representation of the output buffer can be obtained by dnn_runtime_output_variable(rt, output_index)

 */
void *dnn_runtime_output_buffer(dnn_runtime_t * rt, unsigned char output_index);

/**
 * Obtain information about memory allocation in the Nuttx-side heap
 *
 * @param [out] info: pointer to store memory allocation stats
 *
 * @return 0 on success. otherwise -EINVAL.
 */
int dnn_nuttx_mallinfo(dnn_mallinfo_t * info);

/**
 * Obtain information about memory allocation in the ASMP-side heap
 *
 * @param [in]  array_length: Number of elements in info_array.<br>
                              This value must be dnn_config_t::cpu_num
 * @param [out] info_array:   Array to store memory allocation stats
 *
 * @return 0 on success. -EPERM if CONFIG_DNN_RT_MP=n
 */
int dnn_asmp_mallinfo(unsigned char array_length, dnn_mallinfo_t * info_array);

/** @} dnnrt_funcs */

#  undef EXTERN
#  ifdef __cplusplus
}
#  endif

/** @} dnnrt */

#endif                                 /* __INCLUDE_DNNRT_RUNTIME_H */
