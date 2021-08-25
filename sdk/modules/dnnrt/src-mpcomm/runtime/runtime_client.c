/****************************************************************************
 * modules/dnnrt/src-mpcomm/runtime/runtime_client.c
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
#include <stdarg.h>
#include <string.h>

#include <mpcomm/supervisor.h>
#include <dnnrt/runtime.h>

#include "runtime_client.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mpcomm_supervisor_context_t *ctx = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int dnn_supervisor_send_msg(int id, int num_args, ...)
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

  ret = mpcomm_supervisor_send_controller(ctx, &dnn_msg);
  if (ret)
    {
      return ret;
    }

  ret = mpcomm_supervisor_wait_controller_done(ctx, NULL);
  if (ret)
    {
      return ret;
    }

  ret = dnn_msg.ret;

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int dnn_initialize(dnn_config_t *config)
{
  dnn_config_t cfg = {.cpu_num = 1};

  if (config == NULL)
    {
      config = &cfg;
    }

  if (config->cpu_num == 0)
    {
      return -EINVAL;
    }

  return mpcomm_supervisor_init(&ctx, CONFIG_DNN_RT_MPCOMM_PATH,
                                config->cpu_num - 1);
}

int dnn_finalize(void)
{
  return mpcomm_supervisor_deinit(ctx);
}

int dnn_runtime_initialize(dnn_runtime_t *rt, const nn_network_t *network)
{
  return dnn_supervisor_send_msg(DNNRT_MSG_NRT_INIT, 2, rt, network);
}

int dnn_runtime_finalize(dnn_runtime_t *rt)
{
  return dnn_supervisor_send_msg(DNNRT_MSG_NRT_FINI, 1, rt);
}

int dnn_runtime_forward(dnn_runtime_t *rt, const void *inputs[],
                        unsigned char input_num)
{
  return dnn_supervisor_send_msg(DNNRT_MSG_NRT_FOWARD, 3, rt, inputs,
                                 input_num);
}

int dnn_runtime_input_num(dnn_runtime_t *rt)
{
  return dnn_supervisor_send_msg(DNNRT_MSG_NRT_INPUT_NUM, 1, rt);
}

int dnn_runtime_input_size(dnn_runtime_t *rt, unsigned char data_index)
{
  return dnn_supervisor_send_msg(DNNRT_MSG_NRT_INPUT_SIZE, 2, rt,
                                 data_index);
}

int dnn_runtime_input_ndim(dnn_runtime_t *rt, unsigned char data_index)
{
  return dnn_supervisor_send_msg(DNNRT_MSG_NRT_INPUT_NDIM, 2, rt,
                                 data_index);
}

int dnn_runtime_input_shape(dnn_runtime_t *rt, unsigned char data_index,
                            unsigned char dim_index)
{
  return dnn_supervisor_send_msg(DNNRT_MSG_NRT_INPUT_SHAPE, 3, rt,
                                 data_index, dim_index);
}

nn_variable_t *dnn_runtime_input_variable(dnn_runtime_t *rt,
                                          unsigned char data_index)
{
  return
    (nn_variable_t *)dnn_supervisor_send_msg(DNNRT_MSG_NRT_INPUT_VARIABLE,
                                             2, rt, data_index);
}

int dnn_runtime_output_num(dnn_runtime_t *rt)
{
  return dnn_supervisor_send_msg(DNNRT_MSG_NRT_OUTPUT_NUM, 1, rt);
}

int dnn_runtime_output_size(dnn_runtime_t *rt, unsigned char data_index)
{
  return dnn_supervisor_send_msg(DNNRT_MSG_NRT_OUTPUT_SIZE, 2, rt,
                                 data_index);
}

int dnn_runtime_output_ndim(dnn_runtime_t *rt, unsigned char data_index)
{
  return dnn_supervisor_send_msg(DNNRT_MSG_NRT_OUTPUT_NDIM, 2, rt,
                                 data_index);
}

int dnn_runtime_output_shape(dnn_runtime_t *rt, unsigned char data_index,
                             unsigned char dim_index)
{
  return dnn_supervisor_send_msg(DNNRT_MSG_NRT_OUTPUT_SHAPE, 3, rt,
                                 data_index, dim_index);
}

void *dnn_runtime_output_buffer(dnn_runtime_t *rt, unsigned char data_index)
{
  return (void *)dnn_supervisor_send_msg(DNNRT_MSG_NRT_OUTPUT_BUFFER, 2, rt,
                                         data_index);
}

nn_variable_t *dnn_runtime_output_variable(dnn_runtime_t *rt,
                                           unsigned char data_index)
{
  return
    (nn_variable_t *)dnn_supervisor_send_msg(DNNRT_MSG_NRT_OUTPUT_VARIABLE,
                                             2, rt, data_index);
}

int dnn_asmp_mallinfo(unsigned char array_length, dnn_mallinfo_t *info_array)
{
  return dnn_supervisor_send_msg(DNNRT_MSG_NRT_ASMP_MALLINFO, 2,
                                 array_length, info_array);
}

int dnn_nuttx_mallinfo(dnn_mallinfo_t *info)
{
  if (!info)
    {
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
