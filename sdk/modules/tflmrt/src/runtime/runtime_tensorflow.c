/****************************************************************************
 * tflmrt/src/runtime/runtime_tensorflow.c
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
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>

#include <tflmrt/runtime.h>

#include "runtime_common.h"

#include "tf_runtime.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void debug_log_printf(const char *s)
{
  printf(s);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tflm_initialize(tflm_config_t *config)
{
  if (config != NULL && config->cpu_num != 1u)
    {
      tflm_err("multicore-processing is NOT enabled.\n");
      tflm_err("tflmrt works with tflm_config_t::cpu_num == 1u\n");
      return -EINVAL;
    }

  return 0;
}

int tflm_finalize(void)
{
  return 0;
}

int tflm_runtime_initialize(tflm_runtime_t *rt,
                            const void *network,
                            int size)
{
  int ret;

  if (!rt)
    {
      tflm_err("rt is null.\n");
      return -EINVAL;
    }

  if (!network)
    {
      tflm_err("network is null.\n");
      return -EINVAL;
    }

  /* Register callback for printing debug log */

  RegisterDebugLogCallback(debug_log_printf);

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

int tflm_runtime_finalize(tflm_runtime_t *rt)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return -EINVAL;
    }

  return tf_rt_free_context((tf_rt_context_pointer *) &(rt->impl_ctx));
}

int tflm_runtime_forward(tflm_runtime_t *rt, const void *inputs[],
                         unsigned char input_num)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return -EINVAL;
    }

  tf_rt_context_pointer ctx = (tf_rt_context_pointer) rt->impl_ctx;
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

int tflm_runtime_input_num(tflm_runtime_t *rt)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return -EINVAL;
    }

  return tf_rt_num_of_input((tf_rt_context_pointer) rt->impl_ctx);
}

int tflm_runtime_input_size(tflm_runtime_t *rt, unsigned char data_index)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return -EINVAL;
    }

  return tf_rt_input_size((tf_rt_context_pointer) rt->impl_ctx, data_index);
}

int tflm_runtime_input_ndim(tflm_runtime_t *rt, unsigned char data_index)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return -EINVAL;
    }

  return tf_rt_input_dimension((tf_rt_context_pointer) rt->impl_ctx,
                               data_index);
}

int
tflm_runtime_input_shape(tflm_runtime_t *rt, unsigned char data_index,
                         unsigned char dim_index)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return -EINVAL;
    }

  return tf_rt_input_shape((tf_rt_context_pointer) rt->impl_ctx, data_index,
                           dim_index);
}

void *tflm_input_buffer(tflm_runtime_t *rt, unsigned char data_index)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return NULL;
    }

  return tf_rt_input_buffer(rt->impl_ctx, (size_t) data_index);
}

TfLiteTensor *tflm_runtime_input_variable(tflm_runtime_t *rt,
                                          unsigned char data_index)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return NULL;
    }

  return tf_rt_input_variable(rt->impl_ctx, data_index);
}

int tflm_runtime_output_num(tflm_runtime_t *rt)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return -EINVAL;
    }

  return tf_rt_num_of_output(rt->impl_ctx);
}

int tflm_runtime_output_size(tflm_runtime_t *rt, unsigned char data_index)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return -EINVAL;
    }

  return tf_rt_output_size(rt->impl_ctx, data_index);
}

int tflm_runtime_output_ndim(tflm_runtime_t *rt, unsigned char data_index)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return -EINVAL;
    }

  return tf_rt_output_dimension(rt->impl_ctx, data_index);
}

int tflm_runtime_output_shape(tflm_runtime_t *rt, unsigned char data_index,
                              unsigned char dim_index)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return -EINVAL;
    }

  return tf_rt_output_shape(rt->impl_ctx, data_index, dim_index);
}

void *tflm_runtime_output_buffer(tflm_runtime_t *rt,
                                 unsigned char data_index)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return NULL;
    }

  return tf_rt_output_buffer(rt->impl_ctx, (size_t) data_index);
}

TfLiteTensor *tflm_runtime_output_variable(tflm_runtime_t *rt,
                                           unsigned char data_index)
{
  if (!rt)
    {
      tflm_err("rt is null.\n");
      return NULL;
    }

  return tf_rt_output_variable(rt->impl_ctx, data_index);
}

int tflm_asmp_mallinfo(unsigned char array_length,
                       tflm_mallinfo_t *info_array)
{
  return -EPERM;
}

int tflm_nuttx_mallinfo(tflm_mallinfo_t *info)
{
  if (!info)
    {
      tflm_err("info is null.\n");
      return -EINVAL;
    }

  struct mallinfo mem;

  mem = mallinfo();
  info->cpu = 2;
  info->total_bytes = mem.arena;
  info->used_bytes = mem.uordblks;
  info->largest_bytes = mem.mxordblk;

  return 0;
}
