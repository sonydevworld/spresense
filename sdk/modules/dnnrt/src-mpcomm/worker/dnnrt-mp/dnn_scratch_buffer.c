/****************************************************************************
 * modules/dnnrt/src-mpcomm/worker/dnnrt-mp/dnn_scratch_buffer.c
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

#include <mpcomm/mpcomm.h>

#include "dnn_scratch_buffer.h"
#include "dnn_controller.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALIGN(x, s) ((void *)((unsigned int)(((void *)(x)) + ((s)-1)) & ~((s)-1)))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static dnn_scratch_buffer_t scratch_buffer;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int dnn_scratch_buffer_create(void)
{
  if (scratch_buffer.requested_size > scratch_buffer.buffer_size)
    {
      int i;
      int size = (int)ALIGN(scratch_buffer.requested_size, sizeof(float));

      for (i = 0; i < mpcomm_get_helpers_num() + 1; ++i)
        {
          if (scratch_buffer.buffer_addr[i])
            {
              dnn_controller_free(scratch_buffer.buffer_addr[i]);
            }

          scratch_buffer.buffer_addr[i] = dnn_controller_malloc(size);
        }

      scratch_buffer.buffer_size = size;
    }

  return 0;
}

int dnn_scratch_buffer_destroy(void)
{
  int i;

  for (i = 0; i < mpcomm_get_helpers_num() + 1; ++i)
    {
      if (scratch_buffer.buffer_addr[i])
        {
          dnn_controller_free(scratch_buffer.buffer_addr[i]);
          scratch_buffer.buffer_addr[i] = NULL;
        }
    }

  scratch_buffer.buffer_size = 0;
  scratch_buffer.requested_size = 0;

  return 0;
}

void dnn_scratch_buffer_request_size(int size)
{
  if (size > scratch_buffer.requested_size)
    {
      scratch_buffer.requested_size = size;
    }
}

void *dnn_scratch_buffer_get(void)
{
  if (mpcomm_is_controller())
    {
      return scratch_buffer.buffer_addr[mpcomm_get_helpers_num()];
    }
  else
    {
      return scratch_buffer.buffer_addr[0];
    }
}

void *dnn_scratch_buffer_get_index(uint8_t index)
{
  return scratch_buffer.buffer_addr[index];
}

void dnn_scratch_buffer_set(void *buffer)
{
  if (mpcomm_is_controller())
    {
      scratch_buffer.buffer_addr[mpcomm_get_helpers_num()] = buffer;
    }
  else
    {
      scratch_buffer.buffer_addr[0] = buffer;
    }
}
