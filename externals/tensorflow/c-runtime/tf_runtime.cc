/****************************************************************************
 * externals/tensorflow/c-runtime/tf_runtime.cc
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

#include <stdlib.h>

#include "tf_runtime.h"
#include "tf_context.h"

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

void *(*tf_rt_malloc_func)(size_t size) = malloc;
void (*tf_rt_free_func)(void *ptr) = free;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tf_rt_allocate_context(tf_rt_context_pointer *context)
{
  tf_rt_context_t *c =
    (tf_rt_context_t *) tf_rt_malloc_func(sizeof(tf_rt_context_t));
  if (c == 0)
    {
      return -ENOMEM;
    }

  memset(c, 0, sizeof(tf_rt_context_t));
  *context = c;

  return 0;
}

int tf_rt_initialize_context(tf_rt_context_pointer context,
                             const void *n, int size)
{
  tf_rt_context_t *c = (tf_rt_context_t *) context;

  tflite::InitializeTarget();
  tflite::MicroErrorReporter micro_error_reporter;
  c->error_reporter = &micro_error_reporter;

  /* Map the model into a usable data structure. This doesn't involve any
   * copying or parsing, it's a very lightweight operation.
   */

  c->model = tflite::GetModel(n);
  if (c->model->version() != TFLITE_SCHEMA_VERSION)
    {
      TF_LITE_REPORT_ERROR(c->error_reporter,
                          "Model provided is schema version %d not equal "
                          "to supported version %d.",
                          c->model->version(), TFLITE_SCHEMA_VERSION);
      return -EPERM;
    }

  tflite::AllOpsResolver resolver;

  c->tensor_arena_size = size;
  c->tensor_arena = (uint8_t *) tf_rt_malloc_func(c->tensor_arena_size);
  if (c->tensor_arena == 0)
    {
      return -ENOMEM;
    }

  memset(c->tensor_arena, 0, c->tensor_arena_size);

  /* Build an interpreter to run the model with. */

  tflite::MicroInterpreter interpreter(
      c->model, resolver, c->tensor_arena,
      c->tensor_arena_size, c->error_reporter);
  c->interpreter = &interpreter;

  /* Allocate memory from the tensor_arena for the model's tensors. */

  TfLiteStatus allocate_status = c->interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk)
    {
      TF_LITE_REPORT_ERROR(c->error_reporter, "AllocateTensors() failed");
      return -ENOMEM;
    }

  return 0;
}

int tf_rt_free_context(tf_rt_context_pointer *context)
{
  tf_rt_context_t *c = (tf_rt_context_t *) *context;

  if (c->tensor_arena)
    {
      tf_rt_free_func(c->tensor_arena);
      c->tensor_arena = NULL;
      c->tensor_arena_size = 0;
    }

  tf_rt_free_func(*context);

  return 0;
}

size_t tf_rt_arenasize(tf_rt_context_pointer context)
{
  return ((tf_rt_context_t *)context)->interpreter->arena_used_bytes();
}

int tf_rt_num_of_input(tf_rt_context_pointer context)
{
  return ((tf_rt_context_t *)context)->interpreter->inputs_size();
}

int tf_rt_input_size(tf_rt_context_pointer context, size_t index)
{
  return ((tf_rt_context_t *)context)->interpreter->input(index)->bytes;
}

int tf_rt_input_dimension(tf_rt_context_pointer context, size_t index)
{
  return ((tf_rt_context_t *)context)->interpreter->input(index)->
                                       dims->size;
}

int tf_rt_input_shape(tf_rt_context_pointer context, size_t index,
                      size_t shape_index)
{
  return ((tf_rt_context_t *)context)->interpreter->input(index)->
                                       dims->data[shape_index];
}

void *tf_rt_input_buffer(tf_rt_context_pointer context, size_t index)
{
  return ((tf_rt_context_t *)context)->interpreter->input(index)->data.raw;
}

int tf_rt_num_of_output(tf_rt_context_pointer context)
{
  return ((tf_rt_context_t *)context)->interpreter->outputs_size();
}

int tf_rt_output_size(tf_rt_context_pointer context, size_t index)
{
  return ((tf_rt_context_t *)context)->interpreter->output(index)->bytes;
}

int tf_rt_output_dimension(tf_rt_context_pointer context, size_t index)
{
  return ((tf_rt_context_t *)context)->interpreter->output(index)->
                                       dims->size;
}

int tf_rt_output_shape(tf_rt_context_pointer context, size_t index,
                       size_t shape_index)
{
  return ((tf_rt_context_t *)context)->interpreter->output(index)->
                                       dims->data[shape_index];
}

void *tf_rt_output_buffer(tf_rt_context_pointer context, size_t index)
{
  return ((tf_rt_context_t *)context)->interpreter->output(index)->data.raw;
}

TfLiteTensor *tf_rt_input_variable(tf_rt_context_pointer context,
                                   size_t index)
{
  return ((tf_rt_context_t *)context)->interpreter->input(index);
}

TfLiteTensor *tf_rt_output_variable(tf_rt_context_pointer context,
                                    size_t index)
{
  return ((tf_rt_context_t *)context)->interpreter->output(index);
}

int tf_rt_forward(tf_rt_context_pointer context)
{
  tf_rt_context_t *c = (tf_rt_context_t *) context;

  /* Run inference, and report any error */

  TfLiteStatus invoke_status = c->interpreter->Invoke();
  if (invoke_status != kTfLiteOk)
    {
      TF_LITE_REPORT_ERROR(c->error_reporter, "Invoke failed\n");
      return -EPERM;
    }

  return 0;
}

void tf_rt_set_malloc(void *(*user_malloc)(size_t size))
{
  if (user_malloc == 0)
    {
      tf_rt_malloc_func = malloc;
    }
  else
    {
      tf_rt_malloc_func = user_malloc;
    }
}

void tf_rt_set_free(void (*user_free)(void *ptr))
{
  if (user_free == 0)
    {
      tf_rt_free_func = free;
    }
  else
    {
      tf_rt_free_func = user_free;
    }
}
