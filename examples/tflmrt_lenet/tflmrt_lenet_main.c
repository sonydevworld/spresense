/****************************************************************************
 * tflmrt_lenet/tflmrt_lenet_main.c
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

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/time.h>

#include <nuttx/config.h>
#include <tflmrt/runtime.h>

#include "loader_tflite.h"
#include "pnm_util.h"

/****************************************************************************
 * Type Definition
 ****************************************************************************/

typedef struct
{
  char *model_path;
  char *pgm_path;
  bool skip_norm;
} my_setting_t;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TFLM_TENSOR_ARENA_SIZE  (40 * 1024)
#define MNIST_SIZE_PX           (28 * 28)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static float s_img_buffer[MNIST_SIZE_PX];
extern unsigned char model_tflite[];
extern int model_tflite_len;
extern unsigned char image0[];
extern int image0_len;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int convert_datatype(tflm_runtime_t *rt)
{
  int ret = 0;

  /* get datatype which this tflm_runtime_t expects */

  TfLiteTensor *var = tflm_runtime_input_variable(rt, 0);

  if (var->type == kTfLiteFloat32)
    {
      /* do nothing since the image data is stored as float */
    }
  else if (var->type == kTfLiteInt16)
    {
      /* convert the image data in-place to 16-bit fixed-point values */

      int16_t *int16_buffer = (int16_t *) s_img_buffer;
      uint16_t px;
      for (px = 0u; px < MNIST_SIZE_PX; px++)
        {
          int16_buffer[px] = (int16_t) (s_img_buffer[px] /
                             var->params.scale +
                             var->params.zero_point);
        }
    }
  else if (var->type == kTfLiteInt8)
    {
      /* convert the image data in-place to 8-bit fixed-point values */

      int8_t *int8_buffer = (int8_t *) s_img_buffer;
      uint16_t px;
      for (px = 0u; px < MNIST_SIZE_PX; px++)
        {
          int8_buffer[px] = (int8_t) (s_img_buffer[px] /
                            var->params.scale +
                            var->params.zero_point);
        }
    }
  else
    {
      ret = -1;
    }

  return ret;
}

static float get_output(tflm_runtime_t *rt, unsigned char index)
{
  float output;
  TfLiteTensor *var = tflm_runtime_output_variable(rt, 0);

  switch (var->type)
  {
    case kTfLiteFloat32:
      output = var->data.f[index];
      break;
    case kTfLiteInt16:
      output = (var->data.i16[index] - var->params.zero_point) *
               var->params.scale;
      break;
    case kTfLiteInt8:
      output = (var->data.int8[index] - var->params.zero_point) *
               var->params.scale;
      break;
    default:
      output = 0.0f;
      printf("Unsupported data type: %d\n", var->type);
      break;
  }

  return output;
}

static void parse_args(int argc, char *argv[], my_setting_t *setting)
{
  /* parse options by getopt() */

  int opt;
  while ((opt = getopt(argc, argv, "s")) != -1)
    {
      switch (opt)
        {
        case 's':              /* skip normalization */
          setting->skip_norm = true;
          break;
        }
    }

  /* set my_setting_t::{model_path,pgm_path} to argv[] if necessary */

  setting->model_path = (optind < argc) &&
                        (strcmp(argv[optind], "default") != 0) ?
                        argv[optind] : NULL;
  optind++;

  setting->pgm_path = (optind < argc) ? argv[optind] : NULL;

  /* print my_setting_t */

  if (setting->model_path)
    {
      printf("Load model file: %s\n", setting->model_path);
    }
  else
    {
      printf("Load built-in model\n");
    }

  if (setting->pgm_path)
    {
      printf("Load pgm image: %s\n", setting->pgm_path);
    }
  else
    {
      printf("Load built-in test image\n");
    }

  if (setting->skip_norm)
    {
      printf("Image Normalization (1.0/255.0): skipped\n");
    }
  else
    {
      printf("Image Normalization (1.0/255.0): enabled\n");
    }
}

/****************************************************************************
 * tflmrt_lenet_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret = 0;
  int i;
  float proc_time;
  float norm_factor;
  const void *inputs[1] = { s_img_buffer };
  tflm_runtime_t rt;
  tflm_config_t config = { .cpu_num = 1 };
  void *network;
  my_setting_t setting = { 0 };
  struct timeval begin;
  struct timeval end;

  parse_args(argc, argv, &setting);

  /* load an hand-written digit image into s_img_buffer,
   * and then divide the pixels by 255.0 for normalization
   */

  norm_factor = setting.skip_norm ? 1.0f : 255.0f;
  if (setting.pgm_path)
    {
      ret = pnm_load(setting.pgm_path, norm_factor,
                    s_img_buffer, sizeof(s_img_buffer));
      if (ret)
        {
          printf("load pgm image failed due to %d\n", ret);
          goto pgm_error;
        }
    }
  else
    {
      for (i = 0; i < MNIST_SIZE_PX; i++)
        {
          s_img_buffer[i] = image0[i] / norm_factor;
        }
    }

  /* load a model file, which holds a network structure and weight values,
   * into a heap memory
   */

  if (setting.model_path)
    {
      network = alloc_tflite_network(setting.model_path);
      if (network == NULL)
        {
          printf("load model file failed\n");
          goto pgm_error;
        }
    }
  else
    {
      network = (void *) model_tflite;
    }

  /* initialize the whole tflmrt subsystem */

  ret = tflm_initialize(&config);
  if (ret)
    {
      printf("tflm_initialize() failed due to %d", ret);
      goto tflm_error;
    }

  /* instantiate a neural network in a tflm_runtime_t object */

  ret = tflm_runtime_initialize(&rt, network, TFLM_TENSOR_ARENA_SIZE);
  if (ret)
    {
      printf("tflm_runtime_initialize() failed due to %d\n", ret);
      goto rt_error;
    }

  printf("ARENA Size: %d\n", tflm_runtime_actual_arenasize(&rt));

  /* convert the image data to datatype this tflm_runtime_t expects */

  ret = convert_datatype(&rt);
  if (ret)
    {
      printf("convert_datatype() failed due to %d\n", ret);
      goto rt_error;
    }

  /* perform inference after feeding inputs */

  printf("start tflm_runtime_forward()\n");
  gettimeofday(&begin, 0);
  ret = tflm_runtime_forward(&rt, inputs, 1);
  gettimeofday(&end, 0);
  if (ret)
    {
      printf("tflm_runtime_forward() failed due to %d\n", ret);
      goto fin;
    }

  /* show the classification result and its processing time */

  for (i = 0; i < tflm_runtime_output_shape(&rt, 0, 1); i++)
    {
      printf("output[%u]=%.6f\n", i, get_output(&rt, i));
    }

  proc_time = (float)end.tv_sec + (float)end.tv_usec / 1.0e6;
  proc_time -= (float)begin.tv_sec + (float)begin.tv_usec / 1.0e6;
  printf("inference time=%.3f\n", proc_time);

fin:

  /* free memories allocated to tflm_runtime_t */

  tflm_runtime_finalize(&rt);
rt_error:

  /* finalize the whole tflmrt subsystem */

  tflm_finalize();
tflm_error:

  /* just call free() */

  if (setting.model_path)
    {
      destroy_tflite_network(network);
    }

pgm_error:
  return ret;
}
