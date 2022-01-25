/****************************************************************************
 * modules/tflmrt/src-mpcomm/runtime/runtime_client.c
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
#include <stdarg.h>
#include <string.h>
#include <malloc.h>

#include <mpcomm/supervisor.h>
#include <tflmrt/runtime.h>

#include "runtime_client.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mpcomm_supervisor_context_t *ctx = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int tflm_supervisor_send_msg(int id, int num_args, ...)
{
  int i;
  int ret;
  tflm_msg_t tflm_msg;
  va_list vl;

  int max_args = sizeof(tflm_msg.arg) / sizeof(tflm_msg.arg[0]);
  if (num_args > max_args)
    {
      num_args = max_args;
    }

  memset(&tflm_msg, 0, sizeof(tflm_msg));
  tflm_msg.id = id;
  va_start(vl, num_args);
  for (i = 0; i < num_args; ++i)
    {
      tflm_msg.arg[i] = va_arg(vl, int);
    }

  ret = mpcomm_supervisor_send_controller(ctx, &tflm_msg);
  if (ret)
    {
      return ret;
    }

  ret = mpcomm_supervisor_wait_controller_done(ctx, NULL);
  if (ret)
    {
      return ret;
    }

  ret = tflm_msg.ret;

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tflm_initialize(tflm_config_t *config)
{
  tflm_config_t cfg = {.cpu_num = 1};

  if (config == NULL)
    {
      config = &cfg;
    }

  if (config->cpu_num != 1)
    {
      return -EINVAL;
    }

  return mpcomm_supervisor_init(&ctx, CONFIG_TFLM_RT_MPCOMM_PATH,
                                config->cpu_num - 1);
}

int tflm_finalize(void)
{
  return mpcomm_supervisor_deinit(ctx);
}

int tflm_runtime_initialize(tflm_runtime_t *rt, const void *network,
                            int size)
{
  return tflm_supervisor_send_msg(TFLMRT_MSG_NRT_INIT, 3, rt, network,
                                  size);
}

int tflm_runtime_finalize(tflm_runtime_t *rt)
{
  return tflm_supervisor_send_msg(TFLMRT_MSG_NRT_FINI, 1, rt);
}

int tflm_runtime_forward(tflm_runtime_t *rt, const void *inputs[],
                         unsigned char input_num)
{
  return tflm_supervisor_send_msg(TFLMRT_MSG_NRT_FOWARD, 3, rt, inputs,
                                  input_num);
}

int tflm_runtime_input_num(tflm_runtime_t *rt)
{
  return tflm_supervisor_send_msg(TFLMRT_MSG_NRT_INPUT_NUM, 1, rt);
}

int tflm_runtime_input_size(tflm_runtime_t *rt, unsigned char data_index)
{
  return tflm_supervisor_send_msg(TFLMRT_MSG_NRT_INPUT_SIZE, 2, rt,
                                  data_index);
}

int tflm_runtime_input_ndim(tflm_runtime_t *rt, unsigned char data_index)
{
  return tflm_supervisor_send_msg(TFLMRT_MSG_NRT_INPUT_NDIM, 2, rt,
                                  data_index);
}

int tflm_runtime_input_shape(tflm_runtime_t *rt, unsigned char data_index,
                             unsigned char dim_index)
{
  return tflm_supervisor_send_msg(TFLMRT_MSG_NRT_INPUT_SHAPE, 3, rt,
                                  data_index, dim_index);
}

TfLiteTensor *tflm_runtime_input_variable(tflm_runtime_t *rt,
                                          unsigned char data_index)
{
  return
    (TfLiteTensor *)tflm_supervisor_send_msg(TFLMRT_MSG_NRT_INPUT_VARIABLE,
                                             2, rt, data_index);
}

int tflm_runtime_output_num(tflm_runtime_t *rt)
{
  return tflm_supervisor_send_msg(TFLMRT_MSG_NRT_OUTPUT_NUM, 1, rt);
}

int tflm_runtime_output_size(tflm_runtime_t *rt, unsigned char data_index)
{
  return tflm_supervisor_send_msg(TFLMRT_MSG_NRT_OUTPUT_SIZE, 2, rt,
                                  data_index);
}

int tflm_runtime_output_ndim(tflm_runtime_t *rt, unsigned char data_index)
{
  return tflm_supervisor_send_msg(TFLMRT_MSG_NRT_OUTPUT_NDIM, 2, rt,
                                  data_index);
}

int tflm_runtime_output_shape(tflm_runtime_t *rt, unsigned char data_index,
                              unsigned char dim_index)
{
  return tflm_supervisor_send_msg(TFLMRT_MSG_NRT_OUTPUT_SHAPE, 3, rt,
                                  data_index, dim_index);
}

void *tflm_runtime_output_buffer(tflm_runtime_t *rt,
                                 unsigned char data_index)
{
  return (void *)tflm_supervisor_send_msg(TFLMRT_MSG_NRT_OUTPUT_BUFFER,
                                          2, rt, data_index);
}

TfLiteTensor *tflm_runtime_output_variable(tflm_runtime_t *rt,
                                           unsigned char data_index)
{
  return
    (TfLiteTensor *)tflm_supervisor_send_msg(TFLMRT_MSG_NRT_OUTPUT_VARIABLE,
                                             2, rt, data_index);
}

int tflm_asmp_mallinfo(unsigned char array_length,
                       tflm_mallinfo_t *info_array)
{
  return tflm_supervisor_send_msg(TFLMRT_MSG_NRT_ASMP_MALLINFO, 2,
                                  array_length, info_array);
}

int tflm_nuttx_mallinfo(tflm_mallinfo_t *info)
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
