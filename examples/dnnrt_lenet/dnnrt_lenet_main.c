
/****************************************************************************
 * dnnrt_lenet/dnnrt_lenet_main.c
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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/time.h>
#include <nuttx/config.h>
#include <dnnrt/runtime.h>
#include "loader_nnb.h"
#include "pnm_util.h"

/****************************************************************************
 * Type Definition
 ****************************************************************************/
typedef struct
{
  char *nnb_path;
  char *pgm_path;
  bool skip_norm;
} my_setting_t;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define DNN_PNM_PATH    "/mnt/sd0/0.pgm"
#define DNN_NNB_PATH    "/mnt/sd0/lenet-5.nnb"
#define MNIST_SIZE_PX (28*28)

/****************************************************************************
 * Private Data
 ****************************************************************************/
static float s_img_buffer[MNIST_SIZE_PX];

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void convert_datatype(dnn_runtime_t * rt)
{
  /* get datatype which this dnn_runtime_t expects */
  nn_variable_t *var = dnn_runtime_input_variable(rt, 0);
  float coefficient = (float)(1 << var->fp_pos);

  if (var->type == NN_DATA_TYPE_FLOAT)
    {
      /* do nothing since the image data is stored as float */
    }
  else if (var->type == NN_DATA_TYPE_INT16)
    {
      /* convert the image data in-place to 16-bit fixed-point values */
      int16_t *int16_buffer = (int16_t *) s_img_buffer;
      uint16_t px;
      for (px = 0u; px < MNIST_SIZE_PX; px++)
        {
          int16_buffer[px] = (int16_t) (coefficient * s_img_buffer[px]);
        }
    }
  else
    {
      /* convert the image data in-place to 8-bit fixed-point values */
      int8_t *int8_buffer = (int8_t *) s_img_buffer;
      uint16_t px;
      for (px = 0u; px < MNIST_SIZE_PX; px++)
        {
          int8_buffer[px] = (int8_t) (coefficient * s_img_buffer[px]);
        }
    }
}

static void parse_args(int argc, char *argv[], my_setting_t * setting)
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

  /* set my_setting_t::{nnb_path,pgm_path} to argv[] if necessary */
  setting->nnb_path = (optind < argc) ? argv[optind++] : DNN_NNB_PATH;
  setting->pgm_path = (optind < argc) ? argv[optind] : DNN_PNM_PATH;

  /* print my_setting_t */
  printf("Load nnb file: %s\n", setting->nnb_path);
  printf("Load pgm image: %s\n", setting->pgm_path);
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
 * dnnrt_lenet_main
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
  int ret;
  unsigned char i;
  float *output_buffer, proc_time, norm_factor;
  const void *inputs[1] = { s_img_buffer };
  dnn_runtime_t rt;
  dnn_config_t config = {.cpu_num = 1 };
  nn_network_t *network;
  my_setting_t setting = { 0 };
  struct timeval begin, end;

  parse_args(argc, argv, &setting);

  /* load an hand-written digit image into s_img_buffer,
   * and then divide the pixels by 255.0 for normalization */
  norm_factor = setting.skip_norm ? 1.0f : 255.0f;
  ret =
    pnm_load(setting.pgm_path, norm_factor, s_img_buffer, sizeof(s_img_buffer));
  if (ret)
    {
      printf("load pgm image failed due to %d\n", ret);
      goto pgm_error;
    }

  /* load an nnb file, which holds a network structure and weight values,
   * into a heap memory */
  network = alloc_nnb_network(setting.nnb_path);
  if (network == NULL)
    {
      printf("load nnb file failed\n");
      goto pgm_error;
    }

  /* Step-A: initialize the whole dnnrt subsystem */
  ret = dnn_initialize(&config);
  if (ret)
    {
      printf("dnn_initialize() failed due to %d", ret);
      goto dnn_error;
    }

  /* Step-B: instantiate a neural network defined
   * by nn_network_t as a dnn_runtime_t object */
  ret = dnn_runtime_initialize(&rt, network);
  if (ret)
    {
      printf("dnn_runtime_initialize() failed due to %d\n", ret);
      goto rt_error;
    }

  /* convert the image data to datatype this dnn_runtime_t expects */
  convert_datatype(&rt);

  /* Step-C: perform inference after feeding inputs */
  printf("start dnn_runtime_forward()\n");
  gettimeofday(&begin, 0);
  ret = dnn_runtime_forward(&rt, inputs, 1);
  gettimeofday(&end, 0);
  if (ret)
    {
      printf("dnn_runtime_forward() failed due to %d\n", ret);
      goto fin;
    }

  /* Step-D: obtain the output from this dnn_runtime_t */
  output_buffer = dnn_runtime_output_buffer(&rt, 0u);

  /* show the classification result and its processing time */
  for (i = 0u; i < 10u; i++)
    {
      printf("output[%u]=%.6f\n", i, output_buffer[i]);
    }
  proc_time = (float)end.tv_sec + (float)end.tv_usec / 1.0e6;
  proc_time -= (float)begin.tv_sec + (float)begin.tv_usec / 1.0e6;
  printf("inference time=%.3f\n", proc_time);

fin:
  /* Step-F: free memories allocated to dnn_runtime_t */
  dnn_runtime_finalize(&rt);
rt_error:
  /* Step-G: finalize the whole dnnrt subsystem */
  dnn_finalize();
dnn_error:
  /* just call free() */
  destroy_nnb_network(network);
pgm_error:
  return ret;
}
