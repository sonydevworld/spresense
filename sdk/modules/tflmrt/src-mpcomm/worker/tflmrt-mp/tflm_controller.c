/****************************************************************************
 * modules/tflmrt/src-mpcomm/worker/tflmrt-mp/tflm_controller.c
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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

#include <tf_runtime.h>

#include "tflm_controller.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int tflm_runtime_initialize(tflm_runtime_t *rt,
                                   const void *network,
                                   int size)
{
  int ret;

  if (!rt)
    {
      return -EINVAL;
    }

  if (!network)
    {
      return -EINVAL;
    }

  if (!size)
    {
      return -EINVAL;
    }

  tf_rt_set_malloc(tflm_controller_malloc);
  tf_rt_set_free(tflm_controller_free);

  ret = tf_rt_allocate_context((tf_rt_context_pointer *) &(rt->impl_ctx));
  if (ret != 0)
    {
      return ret;
    }

  tf_rt_context_pointer ctx = (tf_rt_context_pointer) (rt->impl_ctx);

  ret = tf_rt_initialize_context(ctx, network, size);
  if (ret != 0)
    {
      return ret;
    }

  return ret;
}

static int tflm_runtime_finalize(tflm_runtime_t *rt)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return tf_rt_free_context((tf_rt_context_pointer *) &(rt->impl_ctx));
}

static int tflm_runtime_forward(tflm_runtime_t *rt,
                                const void *inputs[],
                                unsigned char input_num)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  tf_rt_context_pointer ctx = (tf_rt_context_pointer)rt->impl_ctx;
  if (tf_rt_num_of_input(ctx) != input_num)
    {
      return -EINVAL;
    }

  for (int i = 0; i < input_num; ++i)
    {
      memcpy(tf_rt_input_buffer(ctx, i),
             inputs[i],
             tf_rt_input_size(ctx, i));
    }

  return tf_rt_forward(ctx);
}

static int tflm_runtime_input_num(tflm_runtime_t *rt)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return tf_rt_num_of_input((tf_rt_context_pointer)rt->impl_ctx);
}

static int tflm_runtime_input_size(tflm_runtime_t *rt,
                                   unsigned char data_index)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return tf_rt_input_size((tf_rt_context_pointer)rt->impl_ctx, data_index);
}

static int tflm_runtime_input_ndim(tflm_runtime_t *rt,
                                   unsigned char data_index)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return tf_rt_input_dimension((tf_rt_context_pointer)rt->impl_ctx,
                                data_index);
}

static int tflm_runtime_input_shape(tflm_runtime_t *rt,
                                    unsigned char data_index,
                                    unsigned char dim_index)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return tf_rt_input_shape((tf_rt_context_pointer)rt->impl_ctx, data_index,
                        dim_index);
}

static TfLiteTensor *tflm_runtime_input_variable(tflm_runtime_t *rt,
                                                 unsigned char data_index)
{
  if (rt == NULL)
    {
      return NULL;
    }

  return MEM_V2P(tf_rt_input_variable(rt->impl_ctx, data_index));
}

static int tflm_runtime_output_num(tflm_runtime_t *rt)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return tf_rt_num_of_output(rt->impl_ctx);
}

static int tflm_runtime_output_size(tflm_runtime_t *rt,
                                    unsigned char data_index)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return tf_rt_output_size(rt->impl_ctx, data_index);
}

static int tflm_runtime_output_ndim(tflm_runtime_t *rt,
                                    unsigned char data_index)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return tf_rt_output_dimension(rt->impl_ctx, data_index);
}

static int tflm_runtime_output_shape(tflm_runtime_t *rt,
                                     unsigned char data_index,
                                     unsigned char dim_index)
{
  if (rt == NULL)
    {
      return -EINVAL;
    }

  return tf_rt_output_shape(rt->impl_ctx, data_index, dim_index);
}

static void *tflm_runtime_output_buffer(tflm_runtime_t *rt,
                                        unsigned char data_index)
{
  if (rt == NULL)
    {
      return NULL;
    }

  return MEM_V2P(tf_rt_output_buffer(rt->impl_ctx, (size_t)data_index));
}

static TfLiteTensor *tflm_runtime_output_variable(tflm_runtime_t *rt,
                                                  unsigned char data_index)
{
  if (rt == NULL)
    {
      return NULL;
    }

  TfLiteTensor *output_variable =
    MEM_V2P(tf_rt_output_variable(rt->impl_ctx, data_index));

  output_variable->data.data = MEM_V2P(output_variable->data.data);

  return output_variable;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void tflm_controller_handle_msg(void *data)
{
  tflm_msg_t *tflm_msg = (tflm_msg_t *)data;

  switch (tflm_msg->id)
    {
      case TFLMRT_MSG_NRT_INIT:
        tflm_msg->ret =
          tflm_runtime_initialize((tflm_runtime_t *)tflm_msg->arg[0],
                                  (void *)tflm_msg->arg[1],
                                  (int)tflm_msg->arg[2]);
        break;

      case TFLMRT_MSG_NRT_FINI:
        tflm_msg->ret =
          tflm_runtime_finalize((tflm_runtime_t *)tflm_msg->arg[0]);
        break;

      case TFLMRT_MSG_NRT_FOWARD:
        tflm_msg->ret =
          tflm_runtime_forward((tflm_runtime_t *)tflm_msg->arg[0],
                               (const void **)tflm_msg->arg[1],
                               (unsigned char)tflm_msg->arg[2]);
        break;

      case TFLMRT_MSG_NRT_INPUT_NUM:
        tflm_msg->ret =
          tflm_runtime_input_num((tflm_runtime_t *)tflm_msg->arg[0]);
        break;

      case TFLMRT_MSG_NRT_INPUT_SIZE:
        tflm_msg->ret =
          tflm_runtime_input_size((tflm_runtime_t *)tflm_msg->arg[0],
                                  (unsigned char)tflm_msg->arg[1]);
        break;

      case TFLMRT_MSG_NRT_INPUT_NDIM:
        tflm_msg->ret =
          tflm_runtime_input_ndim((tflm_runtime_t *)tflm_msg->arg[0],
                                  (unsigned char)tflm_msg->arg[1]);
        break;

      case TFLMRT_MSG_NRT_INPUT_SHAPE:
        tflm_msg->ret =
          tflm_runtime_input_shape((tflm_runtime_t *)tflm_msg->arg[0],
                                   (unsigned char)tflm_msg->arg[1],
                                   (unsigned char)tflm_msg->arg[2]);
        break;

      case TFLMRT_MSG_NRT_INPUT_VARIABLE:
        tflm_msg->ret =
          (int)tflm_runtime_input_variable(
            (tflm_runtime_t *)tflm_msg->arg[0],
            (unsigned char)tflm_msg->arg[1]);
        break;

      case TFLMRT_MSG_NRT_OUTPUT_NUM:
        tflm_msg->ret =
          tflm_runtime_output_num((tflm_runtime_t *)tflm_msg->arg[0]);
        break;

      case TFLMRT_MSG_NRT_OUTPUT_SIZE:
        tflm_msg->ret =
          tflm_runtime_output_size((tflm_runtime_t *)tflm_msg->arg[0],
                                   (unsigned char)tflm_msg->arg[1]);
        break;

      case TFLMRT_MSG_NRT_OUTPUT_NDIM:
        tflm_msg->ret =
          tflm_runtime_output_ndim((tflm_runtime_t *)tflm_msg->arg[0],
                                   (unsigned char)tflm_msg->arg[1]);
        break;

      case TFLMRT_MSG_NRT_OUTPUT_SHAPE:
        tflm_msg->ret =
          tflm_runtime_output_shape((tflm_runtime_t *)tflm_msg->arg[0],
                                    (unsigned char)tflm_msg->arg[1],
                                    (unsigned char)tflm_msg->arg[2]);
        break;

      case TFLMRT_MSG_NRT_OUTPUT_BUFFER:
        tflm_msg->ret =
          (int)tflm_runtime_output_buffer((tflm_runtime_t *)tflm_msg->arg[0],
                                          (unsigned char)tflm_msg->arg[1]);
        break;

      case TFLMRT_MSG_NRT_OUTPUT_VARIABLE:
        tflm_msg->ret =
          (int)tflm_runtime_output_variable(
            (tflm_runtime_t *)tflm_msg->arg[0],
            (unsigned char)tflm_msg->arg[1]);
        break;

      default:
        tflm_msg->ret = -EINVAL;
        break;
    }
}

void *tflm_controller_malloc(size_t size)
{
  void *ptr = NULL;

  mpcomm_send_malloc(&ptr, size);

  return ptr;
}

void tflm_controller_free(void *ptr)
{
  mpcomm_send_free(ptr);
}
