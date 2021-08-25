/****************************************************************************
 * modules/dnnrt/src-mpcomm/worker/dnnrt-mp/dnn_controller.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <string.h>

#include <mpcomm/mpcomm.h>

#include <nnablart/network.h>
#include <nnablart/runtime.h>

#include "context.h"

#include "dnn_controller.h"
#include "dnn_exec_function.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static dnn_controller_context_t dnn_controller_ctx;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static dnn_controller_context_t *dnn_controller_get_context(void)
{
  return &dnn_controller_ctx;
}

static int dnn_controller_send_helper_msg(uint8_t helper_index,
                                          int id, int num_args, ...)
{
  int i;
  int ret;
  dnn_msg_t dnn_msg;
  va_list vl;

  int max_args = sizeof(dnn_msg.arg) / sizeof(dnn_msg.arg[0]);
  if (num_args > max_args)
    {
      num_args = max_args;
    }

  memset(&dnn_msg, 0, sizeof(dnn_msg));
  dnn_msg.id = id;
  va_start(vl, num_args);
  for (i = 0; i < num_args; ++i)
    {
      dnn_msg.arg[i] = va_arg(vl, int);
    }

  ret = mpcomm_send_helper(helper_index, MEM_V2P(&dnn_msg));
  if (ret)
    {
      return ret;
    }

  ret = mpcomm_wait_helper_done(helper_index);
  if (ret)
    {
      return ret;
    }

  ret = dnn_msg.ret;

  return ret;
}

static void dnn_initialize_helpers(void)
{
  int i;
  dnn_controller_context_t *ctx = dnn_controller_get_context();

  for (i = 0; i < ctx->helpers_num; ++i)
    {
      dnn_controller_send_helper_msg(i, DNN_HELPER_MSG_INIT, 1,
                                     dnn_scratch_buffer_get_index(i));
    }
}

static int dnn_runtime_initialize(dnn_runtime_t *rt,
                                  const nn_network_t *network)
{
  int ret;
  dnn_vbuffer_alloc_info_t alloc_info = {0};
  dnn_controller_context_t *ctx = dnn_controller_get_context();

  if (rt == NULL || network == NULL)
    {
      return -EINVAL;
    }

  ctx->helpers_num = mpcomm_get_helpers_num();

  rt_set_variable_malloc(dnn_variable_malloc);
  rt_set_variable_free(dnn_variable_free);
  rt_set_malloc(dnn_controller_malloc);
  rt_set_free(dnn_controller_free);

  /* for memory saving a stack varible alloc_info is used */

  dnn_vbuffer_initialize(&alloc_info);

  /* peek variable buffer sizes and pre-allocate shared chunks to them */

  ret = dnn_peek_vbuffers(network, &alloc_info);
  if (ret != RT_RET_NOERROR)
    {
      return ret;
    }

  dnn_reset_chunk_usage();
  ret = dnn_preallocate_chunks(&alloc_info);
  if (ret != RT_RET_NOERROR)
    {
      return ret;
    }

  rt->impl_ctx = NULL;

  ret = (int)rt_allocate_context((rt_context_pointer *)&(rt->impl_ctx));
  if (ret != RT_RET_NOERROR)
    {
      dnn_destroy_unused_chunks();
      return ret;
    }

  rt_context_pointer rt_ctx = (rt_context_pointer)(rt->impl_ctx);

  ret = (int)rt_add_callback(rt_ctx, NN_FUNCTION_CONVOLUTION,
                             dnnrt_convolution_alloc);
  if (ret != RT_RET_NOERROR)
    {
      dnn_deallocate_chunks(&alloc_info);
      rt_free_context(&rt->impl_ctx);
      rt->impl_ctx = NULL;
      dnn_destroy_unused_chunks();
      return ret;
    }

  ret = (int)rt_add_callback(rt_ctx, NN_FUNCTION_CONVOLUTION_0,
                             dnnrt_convolution_alloc);
  if (ret != RT_RET_NOERROR)
    {
      dnn_deallocate_chunks(&alloc_info);
      rt_free_context(&rt->impl_ctx);
      rt->impl_ctx = NULL;
      dnn_destroy_unused_chunks();
      return ret;
    }

  ret = (int)rt_add_callback(rt_ctx, NN_FUNCTION_AFFINE,
                             dnnrt_affine_alloc);
  if (ret != RT_RET_NOERROR)
    {
      dnn_deallocate_chunks(&alloc_info);
      rt_free_context(&rt->impl_ctx);
      rt->impl_ctx = NULL;
      dnn_destroy_unused_chunks();
      return ret;
    }

  dnn_scratch_buffer_request_size(0);

  ret = (int)rt_initialize_context(rt_ctx, (nn_network_t *)network);
  if (ret != RT_RET_NOERROR)
    {
      dnn_deallocate_chunks(&alloc_info);
      rt_free_context(&rt->impl_ctx);
      rt->impl_ctx = NULL;
      dnn_destroy_unused_chunks();
      return ret;
    }

  dnn_scratch_buffer_create();

  dnn_initialize_helpers();

  return ret;
}

static int dnn_runtime_finalize(dnn_runtime_t *rt)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  dnn_scratch_buffer_destroy();

  return (int)rt_free_context((rt_context_pointer *)&(rt->impl_ctx));
}

static int dnn_runtime_forward(dnn_runtime_t *rt,
                               const void *inputs[],
                               unsigned char input_num)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  rt_context_pointer ctx = (rt_context_pointer)rt->impl_ctx;
  if (rt_num_of_input(ctx) != input_num)
    {
      return -EINVAL;
    }

  rt_context_t *c = (rt_context_t *)ctx;

  for (int i = 0; i < input_num; ++i)
    {
      c->variables[c->input_variable_ids[i]].data = (void *)inputs[i];
    }

  return (int)rt_forward(ctx);
}

static int dnn_runtime_input_num(dnn_runtime_t *rt)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return rt_num_of_input((rt_context_pointer)rt->impl_ctx);
}

static int dnn_runtime_input_size(dnn_runtime_t *rt,
                                  unsigned char data_index)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return rt_input_size((rt_context_pointer)rt->impl_ctx, data_index);
}

static int dnn_runtime_input_ndim(dnn_runtime_t *rt,
                                  unsigned char data_index)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return rt_input_dimension((rt_context_pointer)rt->impl_ctx, data_index);
}

static int dnn_runtime_input_shape(dnn_runtime_t *rt,
                                   unsigned char data_index,
                                   unsigned char dim_index)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return rt_input_shape((rt_context_pointer)rt->impl_ctx, data_index,
                        dim_index);
}

static nn_variable_t *dnn_runtime_input_variable(dnn_runtime_t *rt,
                                                 unsigned char data_index)
{
  if (rt == NULL)
    {
      return NULL;
    }

  return rt_input_variable(rt->impl_ctx, data_index);
}

static int dnn_runtime_output_num(dnn_runtime_t *rt)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return rt_num_of_output(rt->impl_ctx);
}

static int dnn_runtime_output_size(dnn_runtime_t *rt,
                                   unsigned char data_index)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return rt_output_size(rt->impl_ctx, data_index);
}

static int dnn_runtime_output_ndim(dnn_runtime_t *rt,
                                   unsigned char data_index)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return rt_output_dimension(rt->impl_ctx, data_index);
}

static int dnn_runtime_output_shape(dnn_runtime_t *rt,
                                    unsigned char data_index,
                                    unsigned char dim_index)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return rt_output_shape(rt->impl_ctx, data_index, dim_index);
}

static void *dnn_runtime_output_buffer(dnn_runtime_t *rt,
                                       unsigned char data_index)
{
  if (rt == NULL)
    {
      return NULL;
    }

  return rt_output_buffer(rt->impl_ctx, (size_t)data_index);
}

static nn_variable_t *dnn_runtime_output_variable(dnn_runtime_t *rt,
                                                  unsigned char data_index)
{
  if (rt == NULL)
    {
      return NULL;
    }

  return rt_output_variable(rt->impl_ctx, data_index);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void dnn_controller_handle_msg(void *data)
{
  dnn_msg_t *dnn_msg = (dnn_msg_t *)data;

  switch (dnn_msg->id)
    {
      case DNNRT_MSG_NRT_INIT:
        dnn_msg->ret =
          dnn_runtime_initialize((dnn_runtime_t *)dnn_msg->arg[0],
                                 (nn_network_t *)dnn_msg->arg[1]);
        break;

      case DNNRT_MSG_NRT_FINI:
        dnn_msg->ret =
          dnn_runtime_finalize((dnn_runtime_t *)dnn_msg->arg[0]);
        break;

      case DNNRT_MSG_NRT_FOWARD:
        dnn_msg->ret =
          dnn_runtime_forward((dnn_runtime_t *)dnn_msg->arg[0],
                              (const void **)dnn_msg->arg[1],
                              (unsigned char)dnn_msg->arg[2]);
        break;

      case DNNRT_MSG_NRT_INPUT_NUM:
        dnn_msg->ret =
          dnn_runtime_input_num((dnn_runtime_t *)dnn_msg->arg[0]);
        break;

      case DNNRT_MSG_NRT_INPUT_SIZE:
        dnn_msg->ret =
          dnn_runtime_input_size((dnn_runtime_t *)dnn_msg->arg[0],
                                 (unsigned char)dnn_msg->arg[1]);
        break;

      case DNNRT_MSG_NRT_INPUT_NDIM:
        dnn_msg->ret =
          dnn_runtime_input_ndim((dnn_runtime_t *)dnn_msg->arg[0],
                                 (unsigned char)dnn_msg->arg[1]);
        break;

      case DNNRT_MSG_NRT_INPUT_SHAPE:
        dnn_msg->ret =
          dnn_runtime_input_shape((dnn_runtime_t *)dnn_msg->arg[0],
                                  (unsigned char)dnn_msg->arg[1],
                                  (unsigned char)dnn_msg->arg[2]);
        break;

      case DNNRT_MSG_NRT_INPUT_VARIABLE:
        dnn_msg->ret =
          (int)dnn_runtime_input_variable((dnn_runtime_t *)dnn_msg->arg[0],
                                          (unsigned char)dnn_msg->arg[1]);
        break;

      case DNNRT_MSG_NRT_OUTPUT_NUM:
        dnn_msg->ret =
          dnn_runtime_output_num((dnn_runtime_t *)dnn_msg->arg[0]);
        break;

      case DNNRT_MSG_NRT_OUTPUT_SIZE:
        dnn_msg->ret =
          dnn_runtime_output_size((dnn_runtime_t *)dnn_msg->arg[0],
                                  (unsigned char)dnn_msg->arg[1]);
        break;

      case DNNRT_MSG_NRT_OUTPUT_NDIM:
        dnn_msg->ret =
          dnn_runtime_output_ndim((dnn_runtime_t *)dnn_msg->arg[0],
                                  (unsigned char)dnn_msg->arg[1]);
        break;

      case DNNRT_MSG_NRT_OUTPUT_SHAPE:
        dnn_msg->ret =
          dnn_runtime_output_shape((dnn_runtime_t *)dnn_msg->arg[0],
                                   (unsigned char)dnn_msg->arg[1],
                                   (unsigned char)dnn_msg->arg[2]);
        break;

      case DNNRT_MSG_NRT_OUTPUT_BUFFER:
        dnn_msg->ret =
          (int)dnn_runtime_output_buffer((dnn_runtime_t *)dnn_msg->arg[0],
                                         (unsigned char)dnn_msg->arg[1]);
        break;

      case DNNRT_MSG_NRT_OUTPUT_VARIABLE:
        dnn_msg->ret =
          (int)dnn_runtime_output_variable((dnn_runtime_t *)dnn_msg->arg[0],
                                           (unsigned char)dnn_msg->arg[1]);
        break;

      default:
        dnn_msg->ret = -EINVAL;
        break;
    }
}

void *dnn_controller_malloc(size_t size)
{
  void *ptr = NULL;

  mpcomm_send_malloc(&ptr, size);

  return ptr;
}

void dnn_controller_free(void *ptr)
{
  mpcomm_send_free(ptr);
}
