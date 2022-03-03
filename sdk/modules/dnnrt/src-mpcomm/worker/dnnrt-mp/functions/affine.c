/****************************************************************************
 * modules/dnnrt/src-mpcomm/worker/dnnrt-mp/functions/affine.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <nnablart/functions.h>
#include <nnablart/runtime.h>
#include <context.h>
#include <implements/neural_network/affine/affine_internal.h>
#include <runtime_internal.h>
#include <arm_nnfunctions.h>
#include "dnn_scratch_buffer.h"
#include "dnn_exec_function.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define X (0)      // x input
#define WEIGHT (1) // weight
#define BIAS (2)   // bias
#define Y (0)      // y output

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef int16_t fixed16_t;
typedef int8_t fixed8_t;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static rt_function_error_t dnnrt_exec_affine_fixed16(rt_function_t *f,
                                                     int begin, int end)
{
  affine_private_t *p =
      (affine_private_t
           *)(((affine_local_context_t *)(f->local_context))->data);
  int k;
  fixed16_t *input = (fixed16_t *)(p->input->data);
  fixed16_t *weight =
    (fixed16_t *)(p->weight->data) + begin * p->input_loop_size;
  fixed16_t *output = (fixed16_t *)(p->output->data) + begin;
  fixed16_t *bias = 0;
  uint16_t bias_shift = 0;
  uint16_t out_shift =
    p->input->fp_pos + p->weight->fp_pos - p->output->fp_pos;

  if (p->bias)
    {
      bias = (fixed16_t *)(p->bias->data) + begin;
      bias_shift = p->input->fp_pos + p->weight->fp_pos - p->bias->fp_pos;
    }

  for (k = 0; k < p->base_loop_size; k++)
    {
      int output_offset = k * p->output_loop_size;
      int input_offset = k * p->input_loop_size;

      fixed16_t *x = input + input_offset;
      fixed16_t *y = output + output_offset;

      arm_fully_connected_q15(x, weight, p->input_loop_size,
                              end - begin, bias_shift, out_shift,
                              bias, y, 0);
    }

  return RT_FUNCTION_ERROR_NOERROR;
}

static rt_function_error_t dnnrt_exec_affine_fixed8(rt_function_t *f,
                                                    int begin, int end)
{
  affine_private_t *p =
      (affine_private_t
           *)(((affine_local_context_t *)(f->local_context))->data);
  int k;
  fixed8_t *input = (fixed8_t *)(p->input->data);
  fixed8_t *weight =
    (fixed8_t *)(p->weight->data) + begin * p->input_loop_size;
  fixed8_t *output = (fixed8_t *)(p->output->data) + begin;
  fixed8_t *bias = 0;
  uint16_t bias_shift = 0;
  uint16_t out_shift =
    p->input->fp_pos + p->weight->fp_pos - p->output->fp_pos;
  q15_t *vec_buffer = (q15_t *)dnn_scratch_buffer_get();

  if (p->bias)
    {
      bias = (fixed8_t *)(p->bias->data) + begin;
      bias_shift = p->input->fp_pos + p->weight->fp_pos - p->bias->fp_pos;
    }

  for (k = 0; k < p->base_loop_size; k++)
    {
      int output_offset = k * p->output_loop_size;
      int input_offset = k * p->input_loop_size;

      fixed8_t *x = input + input_offset;
      fixed8_t *y = output + output_offset;

      arm_fully_connected_q7(x, weight, p->input_loop_size,
                            end - begin, bias_shift, out_shift,
                            bias, y, vec_buffer);
    }

  return RT_FUNCTION_ERROR_NOERROR;
}

static rt_function_error_t dnnrt_exec_affine_float(rt_function_t *f,
                                                   int begin, int end)
{
  affine_private_t *p =
      (affine_private_t
           *)(((affine_local_context_t *)(f->local_context))->data);
  int i;
  int j;
  int k;
  float *input = (float *)(p->input->data);
  float *output = (float *)(p->output->data);
  float *bias = p->bias ? (float *)(p->bias->data) + begin : 0;
  int output_loop_size = end - begin;
  int input_loop_size = p->input_loop_size;

  for (k = 0; k < p->base_loop_size; k++)
    {
      int output_offset = k * output_loop_size;
      int input_offset = k * input_loop_size;

      float *x = input + input_offset;
      float *y = output + output_offset + begin;
      float *weight = (float *)(p->weight->data) + begin * input_loop_size;

      int input_loop_size_bulk4 = (input_loop_size / 4) * 4;
      int loop = output_loop_size / 2;

      /* process two output in a loop */

      for (i = 0; loop--; )
        {
          float sum1 = 0;
          float sum2 = 0;
          float *weight2 = weight + input_loop_size;

          for (j = 0; j < input_loop_size_bulk4; )
            {
              sum1 += weight[j] * x[j];
              sum2 += weight2[j] * x[j];
              ++j;
              sum1 += weight[j] * x[j];
              sum2 += weight2[j] * x[j];
              ++j;
              sum1 += weight[j] * x[j];
              sum2 += weight2[j] * x[j];
              ++j;
              sum1 += weight[j] * x[j];
              sum2 += weight2[j] * x[j];
              ++j;
            }

          for (; j < input_loop_size; j++)
            {
              sum1 += weight[j] * x[j];
              sum2 += weight2[j] * x[j];
            }

          y[i++] = sum1;
          y[i++] = sum2;

          weight += 2 * input_loop_size;
        }

      /* process the last output if any */

      if (output_loop_size & 1)
        {
          float sum1 = 0;

          for (j = 0; j < input_loop_size_bulk4; )
            {
              sum1 += weight[j] * x[j];
              ++j;
              sum1 += weight[j] * x[j];
              ++j;
              sum1 += weight[j] * x[j];
              ++j;
              sum1 += weight[j] * x[j];
              ++j;
            }

          for (; j < input_loop_size; j++)
            {
              sum1 += weight[j] * x[j];
            }

          y[i++] = sum1;
        }

      if (bias)
        {
          float *b = bias + output_offset;
          for (i = 0; i < output_loop_size; i++)
            {
              y[i] += b[i];
            }
        }
    }

  return RT_FUNCTION_ERROR_NOERROR;
}

static rt_function_error_t dnnrt_exec_affine_generic(rt_function_t *f,
                                                     int begin, int end)
{
  affine_private_t *p =
      (affine_private_t
           *)(((affine_local_context_t *)(f->local_context))->data);
  int input_loop_size = p->input_loop_size;
  int output_loop_size = p->output_loop_size;
  int i;
  int j;
  int k;

  for (k = 0; k < p->base_loop_size; k++)
    {
      int input_offset = k * p->input_loop_size;
      int output_offset = k * p->output_loop_size;

      for (i = 0; i < output_loop_size; i++)
        {
          int weight_offset = i * p->input_loop_size;
          int opos = output_offset + i;
          float sum = p->bias ? p->get_bias(p->bias, i) : 0;

          for (j = 0; j < input_loop_size; j++)
            {
              float x = p->get_input(p->input, input_offset + j);
              float w = p->get_weight(p->weight, weight_offset + j);
              sum += x * w;
            }

          p->set_output(p->output, opos, sum);
        }
    }

  return RT_FUNCTION_ERROR_NOERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

rt_function_error_t dnnrt_exec_affine(rt_function_t *f, int begin, int end)
{
  int same_type = ((f->inputs[X]->type == f->inputs[WEIGHT]->type) &&
                   (f->inputs[X]->type == f->inputs[BIAS]->type) &&
                   (f->inputs[X]->type == f->outputs[Y]->type));

  if (same_type && (f->inputs[WEIGHT]->type == NN_DATA_TYPE_INT8))
    {
      return dnnrt_exec_affine_fixed8(f, begin, end);
    }

  if (same_type && (f->inputs[WEIGHT]->type == NN_DATA_TYPE_INT16))
    {
      return dnnrt_exec_affine_fixed16(f, begin, end);
    }

  if (same_type && (f->inputs[WEIGHT]->type == NN_DATA_TYPE_FLOAT))
    {
      return dnnrt_exec_affine_float(f, begin, end);
    }

  return dnnrt_exec_affine_generic(f, begin, end);
}

rt_return_value_t dnnrt_affine_alloc(nn_network_t *net,
                                     void *function_context)
{
  rt_function_context_t *func = (rt_function_context_t *)function_context;

  if ((int)func->info->impl != 0)
    {
      return RT_RET_FUNCTION_DONT_MATCH;
    }

  allocate_function_context(net, func->info, function_context);
  rt_function_t *f = (rt_function_t *)(&func->func);
  f->exec_func = dnn_controller_exec_affine;

  int scratch_buf_bsize = 0;
  if ((f->inputs[X]->type == f->inputs[WEIGHT]->type) &&
      (f->inputs[X]->type == f->inputs[BIAS]->type) &&
      (f->inputs[X]->type == f->outputs[Y]->type) &&
      (f->inputs[X]->type == NN_DATA_TYPE_INT8))
    {
      affine_private_t *p =
          (affine_private_t
              *)(((affine_local_context_t *)(f->local_context))->data);
      scratch_buf_bsize = sizeof(q15_t) * p->input_loop_size;
    }

  dnn_scratch_buffer_request_size(scratch_buf_bsize);
  return RT_RET_FUNCTION_MATCH;
}
