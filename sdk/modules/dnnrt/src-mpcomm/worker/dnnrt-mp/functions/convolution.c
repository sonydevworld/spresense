/****************************************************************************
 * modules/dnnrt/src-mpcomm/worker/dnnrt-mp/functions/convolution.c
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
#include <implements/neural_network/convolution/convolution_internal.h>
#include <utilities/shape.h>
#include <runtime_internal.h>
#include <arm_nnfunctions_nnabla.h>
#include "dnn_scratch_buffer.h"
#include "dnn_exec_function.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _S(p) (sizeof(p) / sizeof(p[0]))
#define X (0)      // x input
#define WEIGHT (1) // weight
#define BIAS (2)   // bias
#define Y0 (0)     // y0 output

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef int16_t fixed16_t;
typedef int8_t fixed8_t;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* group (g) [default 1]: If g > 1, we restrict the connectivity of each
 * filter to a subset of the input. Specifically, the input and output
 * channels arebseparated into g groups, and the i-th output group
 * channels will be only connected to the i-th input group channels.
 */

static inline nn_size_t var_calc_offset(var_t *var, int *pos, int size)
{
  int *s = var->stride.data;
  int len = var->stride.size > size ? size : var->stride.size;
  int i;
  int offset = 0;

  for (i = 0; i < len; ++i)
    {
      offset += (*pos++) * (*s++);
    }

  return offset;
}

static inline void var_setpos(var_t *var, int *pos, int size)
{
  var->offset = var_calc_offset(var, pos, size);
}

static rt_function_error_t dnnrt_exec_convolution_fixed(rt_function_t *f,
                                                        int begin, int end)
{
  convolution_local_context_t *c =
      (convolution_local_context_t *)f->local_context;
  convolution_private_t ctx_copy;
  ctx_copy = *(convolution_private_t *)(c->data);
  convolution_private_t *p = &ctx_copy;
  nn_size_t g, b;
  var_t *out_var = &p->out_var;
  var_t *in_var = &p->in_var;
  var_t *w_var = &p->w_var;
  var_t *b_var = &p->b_var;
  uint16_t out_shift =
      in_var->v->fp_pos + w_var->v->fp_pos - out_var->v->fp_pos;
  uint16_t bias_shift = 0;
  uint16_t fixed16 = f->inputs[X]->type == NN_DATA_TYPE_INT16;
  q15_t *buffer_a = (q15_t *)dnn_scratch_buffer_get();

  for (b = 0; b < p->in_var.shape.data[0]; ++b)
    {
      for (g = 0; g < c->group; ++g)
        {
          if (fixed16)
            {
              int i_pos[] = {b, g, 0};
              var_setpos(in_var, i_pos, _S(i_pos));
              const q15_t *im_in = (q15_t *)in_var->v->data + in_var->offset;

              int w_pos[] = {g, begin, 0};
              var_setpos(w_var, w_pos, _S(w_pos));
              const q15_t *wt = (q15_t *)w_var->v->data + w_var->offset;

              const q15_t *bias = 0;
              if (p->b_var.v)
                {
                  int b_pos[] = {g, begin};
                  var_setpos(b_var, b_pos, _S(b_pos));
                  bias = (q15_t *)b_var->v->data + b_var->offset;
                  bias_shift =
                    in_var->v->fp_pos + w_var->v->fp_pos - b_var->v->fp_pos;
                }

              int o_pos[] = {b, g, begin};
              var_setpos(out_var, o_pos, _S(o_pos));
              q15_t *im_out = (q15_t *)out_var->v->data + out_var->offset;

              arm_convolve_CHW_q15_basic_nonsquare(im_in,
                                                  in_var->shape.data[W],
                                                  in_var->shape.data[H],
                                                  in_var->shape.data[I], wt,
                                                  end - begin,
                                                  w_var->shape.data[W],
                                                  w_var->shape.data[H],
                                                  c->pad.data[1],
                                                  c->pad.data[0],
                                                  c->stride.data[1],
                                                  c->stride.data[0], bias,
                                                  bias_shift, out_shift,
                                                  im_out,
                                                  out_var->shape.data[W],
                                                  out_var->shape.data[H],
                                                  buffer_a, 0);
            }
          else
            {
              int i_pos[] = {b, g, 0};
              var_setpos(in_var, i_pos, _S(i_pos));
              const q7_t *im_in = (q7_t *)in_var->v->data + in_var->offset;

              int w_pos[] = {g, begin, 0};
              var_setpos(w_var, w_pos, _S(w_pos));
              const q7_t *wt = (q7_t *)w_var->v->data + w_var->offset;

              const q7_t *bias = 0;
              if (p->b_var.v)
                {
                  int b_pos[] = {g, begin};
                  var_setpos(b_var, b_pos, _S(b_pos));
                  bias = (q7_t *)b_var->v->data + b_var->offset;
                  bias_shift =
                    in_var->v->fp_pos + w_var->v->fp_pos - b_var->v->fp_pos;
                }

              int o_pos[] = {b, g, begin};
              var_setpos(out_var, o_pos, _S(o_pos));
              q7_t *im_out = (q7_t *)out_var->v->data + out_var->offset;

              arm_convolve_CHW_q7_basic_nonsquare(im_in,
                                                  in_var->shape.data[W],
                                                  in_var->shape.data[H],
                                                  in_var->shape.data[I], wt,
                                                  end - begin,
                                                  w_var->shape.data[W],
                                                  w_var->shape.data[H],
                                                  c->pad.data[1],
                                                  c->pad.data[0],
                                                  c->stride.data[1],
                                                  c->stride.data[0], bias,
                                                  bias_shift, out_shift,
                                                  im_out,
                                                  out_var->shape.data[W],
                                                  out_var->shape.data[H],
                                                  buffer_a, 0);
            }
        }
    }

  return RT_FUNCTION_ERROR_NOERROR;
}

static rt_function_error_t dnnrt_exec_convolution_float(rt_function_t *f,
                                                        int begin, int end)
{
  convolution_local_context_t *c =
      (convolution_local_context_t *)f->local_context;
  convolution_private_t ctx_copy;
  ctx_copy = *(convolution_private_t *)(c->data);
  convolution_private_t *p = &ctx_copy;
  nn_size_t group = c->group;
  nn_size_t batch_size = p->in_var.shape.data[0];
  nn_size_t g, b;
  var_t *out_var = &p->out_var;
  var_t *in_var = &p->in_var;
  var_t *w_var = &p->w_var;
  var_t *b_var = &p->b_var;
  float *buffer_a = (float *)dnn_scratch_buffer_get();

  for (b = 0; b < batch_size; ++b)
    {
      for (g = 0; g < group; ++g)
        {
          int i_pos[] = {b, g, 0};
          var_setpos(in_var, i_pos, _S(i_pos));
          const float *im_in = (float *)in_var->v->data + in_var->offset;

          int w_pos[] = {g, begin, 0};
          var_setpos(w_var, w_pos, _S(w_pos));
          const float *wt = (float *)w_var->v->data + w_var->offset;

          const float *bias = 0;
          if (p->b_var.v)
            {
              int b_pos[] = {g, begin};
              var_setpos(b_var, b_pos, _S(b_pos));
              bias = (float *)b_var->v->data + b_var->offset;
            }

          int o_pos[] = {b, g, begin};
          var_setpos(out_var, o_pos, _S(o_pos));
          float *im_out = (float *)out_var->v->data + out_var->offset;

          arm_convolve_CHW_f32_basic_nonsquare(im_in,
                                              in_var->shape.data[W],
                                              in_var->shape.data[H],
                                              in_var->shape.data[I], wt,
                                              end - begin,
                                              w_var->shape.data[W],
                                              w_var->shape.data[H],
                                              c->pad.data[1],
                                              c->pad.data[0],
                                              c->stride.data[1],
                                              c->stride.data[0], bias,
                                              im_out,
                                              out_var->shape.data[W],
                                              out_var->shape.data[H],
                                              buffer_a, 0);
        }
    }

  return RT_FUNCTION_ERROR_NOERROR;
}

rt_function_error_t dnnrt_exec_convolution(rt_function_t *f,
                                           int begin, int end)
{
  if (f->inputs[X]->type == NN_DATA_TYPE_FLOAT)
    {
      return dnnrt_exec_convolution_float(f, begin, end);
    }
  else
    {
      return dnnrt_exec_convolution_fixed(f, begin, end);
    }
}

static inline int validate_params(rt_function_t *f, int *scratch_buf_bsize)
{
  convolution_local_context_t *c;
  c = (convolution_local_context_t *)f->local_context;
  convolution_private_t *p = (convolution_private_t *)(c->data);

  if (p->spatial_dims != 2 || c->dilation.data[0] != 1 ||
      c->dilation.data[1] != 1)
    {
      return 0;
    }

  int cond1;
  cond1 = f->inputs[X]->type == NN_DATA_TYPE_INT16;
  cond1 &= f->inputs[WEIGHT]->type == NN_DATA_TYPE_INT16;
  cond1 &= f->outputs[Y0]->type == NN_DATA_TYPE_INT16;
  if (cond1)
    {
      /* ch_im_in*dim_kernel_x*dim_kernel_y */

      *scratch_buf_bsize = sizeof(fixed16_t) * p->kernel_shape.data[0] *
                          p->kernel_shape.data[1] * p->in_var.shape.data[I];
    }

  int cond2;
  cond2 = f->inputs[X]->type == NN_DATA_TYPE_INT8;
  cond2 &= f->inputs[WEIGHT]->type == NN_DATA_TYPE_INT8;
  cond2 &= f->outputs[Y0]->type == NN_DATA_TYPE_INT8;
  if (cond2)
    {
      *scratch_buf_bsize =
        sizeof(fixed16_t) * p->kernel_shape.data[0] *
          p->kernel_shape.data[1] * p->in_var.shape.data[I] * 2;
    }

  int cond3;
  cond3 = f->inputs[X]->type == NN_DATA_TYPE_FLOAT;
  cond3 &= f->inputs[WEIGHT]->type == NN_DATA_TYPE_FLOAT;
  cond3 &= f->outputs[Y0]->type == NN_DATA_TYPE_FLOAT;
  if (cond3)
    {
      *scratch_buf_bsize = sizeof(float) * p->kernel_shape.data[0] *
                          p->kernel_shape.data[1] * p->in_var.shape.data[I];
    }

  return cond1 || cond2 || cond3;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

rt_return_value_t dnnrt_convolution_alloc(nn_network_t *net,
                                          void *function_context)
{
  rt_function_context_t *func = (rt_function_context_t *)function_context;
  int scratch_buf_bsize = 0;

  if ((int)func->info->impl != 0)
    {
      return RT_RET_FUNCTION_DONT_MATCH;
    }

  allocate_function_context(net, func->info, function_context);

  if (!validate_params(&func->func, &scratch_buf_bsize))
    {
      return RT_RET_FUNCTION_MATCH;
    }

  func->func.exec_func = dnn_controller_exec_convolution;

  dnn_scratch_buffer_request_size(scratch_buf_bsize);
  return RT_RET_FUNCTION_MATCH;
}
