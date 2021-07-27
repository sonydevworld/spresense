/****************************************************************************
 * tflmrt_lenet/loader_tflite.c
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
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void *alloc_tflite_network(const char *tflite_path)
{
  FILE *tflite_file = NULL;
  uint32_t exp_data_bsize;
  uint32_t act_data_bsize;
  void *network = NULL;

  if (tflite_path == NULL)
    {
      goto file_open_err;
    }

  tflite_file = fopen(tflite_path, "r");
  if (tflite_file == NULL)
    {
      goto file_open_err;
    }

  /* get size of tflite_file in units of bytes */

  fseek(tflite_file, 0, SEEK_END);
  exp_data_bsize = ftell(tflite_file);
  fseek(tflite_file, 0, SEEK_SET);

  /* store content of tflite_file in heap memory */

  network = malloc(exp_data_bsize);
  if (network == NULL)
    {
      goto malloc_error;
    }

  act_data_bsize = fread(network, 1, exp_data_bsize, tflite_file);
  if (exp_data_bsize != act_data_bsize)
    {
      free(network);
      network = NULL;
    }

malloc_error:
  fclose(tflite_file);
file_open_err:
  return network;
}

void destroy_tflite_network(void *network)
{
  free(network);
}
