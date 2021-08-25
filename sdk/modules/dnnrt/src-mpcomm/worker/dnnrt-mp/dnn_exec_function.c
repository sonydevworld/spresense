/****************************************************************************
 * modules/dnnrt/src-mpcomm/worker/dnnrt-mp/dnn_exec_function.c
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
#include <math.h>
#include <string.h>

#include <mpcomm/mpcomm.h>

#include "dnn_controller.h"
#include "dnn_exec_function.h"

#include <implements/neural_network/affine/affine_internal.h>
#include <implements/neural_network/convolution/convolution_internal.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int controller_exec_function(rt_function_t *rt_func,
                                    int func_type,
                                    int begin,
                                    int end)
{
  switch (func_type)
    {
      case DNN_HELPER_EXEC_AFFINE:
        dnnrt_exec_affine(rt_func, begin, end);
        break;
      case DNN_HELPER_EXEC_CONVOLUTION:
        dnnrt_exec_convolution(rt_func, begin, end);
        break;
      default:
        break;
    }

  return 0;
}

static int exec_dnn_tasks(rt_function_t *rt_func,
                          int func_type,
                          dnn_task_t tasks[],
                          int task_num)
{
  int i;
  dnn_msg_t dnn_msg[MAX_HELPERS_NUM];

  for (i = 0; i < task_num; ++i)
    {
      if (i < mpcomm_get_helpers_num())
        {
          dnn_msg[i].id = func_type;
          dnn_msg[i].arg[0] = (int)rt_func;
          dnn_msg[i].arg[1] = tasks[i].begin;
          dnn_msg[i].arg[2] = tasks[i].end;
          mpcomm_send_helper(i, MEM_V2P(&dnn_msg[i]));
        }
      else
        {
          controller_exec_function(rt_func, func_type,
                                   tasks[i].begin,
                                   tasks[i].end);
        }
    }

  mpcomm_wait_helpers_done();

  return 0;
}

static int split_load(int total, dnn_task_t *tasks, int task_num)
{
  if (total <= 0)
    {
      return -EINVAL;
    }

  int load = (int)ceilf((float)total / task_num);
  int i;
  int offset = 0;

  memset(tasks, 0, sizeof(dnn_task_t) * task_num);

  for (i = 0; i < task_num; ++i)
    {
      tasks[i].begin = offset;
      offset += load;
      tasks[i].end = offset;

      if ((total - offset) < load)
        {
          load = total - offset;
        }
    }

  return 0;
}

static int var_buf_size(rt_variable_t *var)
{
  int elem_size = 0;

  if (var->type == NN_DATA_TYPE_FLOAT)
    {
      elem_size = sizeof(float);
    }
  else if (var->type == NN_DATA_TYPE_INT16)
    {
      elem_size = sizeof(int16_t);
    }
  else if (var->type == NN_DATA_TYPE_INT8)
    {
      elem_size = sizeof(int8_t);
    }
  else
    {
      return 0;
    }

  return elem_size * calc_shape_size(var->shape);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

rt_function_error_t dnn_controller_exec_affine(rt_function_t *f)
{
  affine_private_t *p =
      (affine_private_t
           *)(((affine_local_context_t *)(f->local_context))->data);
  memset(p->output->data, 0, var_buf_size(p->output));

  dnn_task_t tasks[MAX_HELPERS_NUM + 1];
  int task_num = mpcomm_get_helpers_num() + 1;
  split_load(p->output_loop_size, tasks, task_num);

  return exec_dnn_tasks(f, DNN_HELPER_EXEC_AFFINE, tasks, task_num);
}

rt_function_error_t dnn_controller_exec_convolution(rt_function_t *f)
{
  convolution_local_context_t *c =
      (convolution_local_context_t *)f->local_context;
  convolution_private_t *p = (convolution_private_t *)(c->data);

  memset(p->out_var.v->data, 0, var_buf_size(p->out_var.v));

  dnn_task_t tasks[MAX_HELPERS_NUM + 1];
  int task_num = mpcomm_get_helpers_num() + 1;
  split_load(p->out_var.shape.data[I], tasks, task_num);

  return exec_dnn_tasks(f, DNN_HELPER_EXEC_CONVOLUTION, tasks, task_num);
}
