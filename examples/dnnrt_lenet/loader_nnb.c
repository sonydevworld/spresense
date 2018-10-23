/****************************************************************************
 * dnnrt_lenet/loader_nnb.c
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

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <dnnrt/runtime.h>

static int get_nnb_size(const char *nnb_path, uint32_t * size_ptr)
{
  int ret;
  struct stat nnb_stat;

  if (nnb_path == NULL || size_ptr == NULL)
    {
      return -EINVAL;
    }
  ret = stat(nnb_path, &nnb_stat);
  if (ret == 0)
    {
      *size_ptr = nnb_stat.st_size;
    }
  return ret;
}

nn_network_t *alloc_nnb_network(const char *nnb_path)
{
  int ret;
  FILE *nnb_file = NULL;
  uint32_t exp_data_bsize, act_data_bsize;
  nn_network_t *network = NULL;

  if (nnb_path == NULL)
    {
      goto file_open_err;;
    }
  nnb_file = fopen(nnb_path, "r");
  if (nnb_file == NULL)
    {
      goto file_open_err;
    }
  /* get size of nnb_file in units of bytes */
  ret = get_nnb_size(nnb_path, &exp_data_bsize);
  if (ret)
    {
      goto get_size_error;
    }
  /* store content of nnb_file in heap memory */
  network = malloc(exp_data_bsize);
  if (network == NULL)
    {
      goto malloc_error;
    }
  act_data_bsize = fread(network, 1, exp_data_bsize, nnb_file);
  if (exp_data_bsize != act_data_bsize)
    {
      free(network);
      network = NULL;
    }

malloc_error:
get_size_error:
  fclose(nnb_file);
file_open_err:
  return network;
}

void destroy_nnb_network(nn_network_t * network)
{
  free(network);
}
