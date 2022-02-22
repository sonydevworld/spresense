/****************************************************************************
 * modules/dnnrt/src-mpcomm/worker/dnnrt-mp/dnn_variable_buffer.c
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

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <nnablart/network.h>
#include <nnablart/runtime.h>
#include <runtime_internal.h>

#include "dnn_variable_buffer.h"
#include "dnn_controller.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALIGN(x, s) ((void *)((unsigned int)(((void *)(x)) + ((s)-1)) & ~((s)-1)))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static dnn_shared_chunk_t *chunks;
static dnn_vbuffer_alloc_info_t *alloc_info;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t round_up(uint32_t num, uint32_t multiple)
{
  uint32_t remain = num % multiple;
  return remain == 0 ? num : num + multiple - remain;
}

static void dnn_shared_chunk_update_ref_count(void *p, int delta)
{
  dnn_shared_chunk_t *chunk = chunks;

  for (; chunk; chunk = chunk->next)
    {
      void *begin = chunk->data;
      void *end = chunk->data + chunk->allocated_bsize;

      if (begin <= p && p < end)
        {
          chunk->ref_count += delta;
          break;
        }
    }

  dnn_destroy_unused_chunks();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void dnn_vbuffer_initialize(dnn_vbuffer_alloc_info_t *info)
{
  alloc_info = info;
}

void dnn_destroy_unused_chunks(void)
{
  dnn_shared_chunk_t *pre = NULL;
  dnn_shared_chunk_t *chunk = chunks;
  dnn_shared_chunk_t *next;

  for (; chunk; chunk = next)
    {
      next = chunk->next;
      if (chunk->ref_count == 0u)
        {
          if (pre)
            {
              pre->next = next;
            }
          else
            {
              chunks = next;
            }

          dnn_controller_free(chunk);
        }
      else
        {
          pre = chunk;
        }
    }
}

void *dnn_variable_malloc(size_t size)
{
  void *p = alloc_info->addr_list[alloc_info->actual_alloc_count++];
  dnn_shared_chunk_update_ref_count(p, 1);
  return p;
}

void dnn_variable_free(void *p)
{
  if (p)
    {
      dnn_shared_chunk_update_ref_count(p, -1);
    }
  else
    {
      assert(0);
    }
}

int dnn_peek_vbuffers(const nn_network_t *n, dnn_vbuffer_alloc_info_t *info)
{
  int i;
  int *list = (int *)NN_GET(n, n->buffers.list);

  if (n->buffers.size > MAX_VBUFFER_NUM)
    {
      return -ENOMEM;
    }

  /* get each variable buffer size from network */

  memset(alloc_info, 0, sizeof(*alloc_info));
  alloc_info->vbuffer_num = n->buffers.size;
  for (i = 0; i < alloc_info->vbuffer_num; i++)
    {
      if (n->version >= 3)
        {
          alloc_info->bsize_list[i] = *(list + i);
        }
      else
        {
          alloc_info->bsize_list[i] = *(list + i) * sizeof(float);
        }
    }

  return RT_RET_NOERROR;
}

static int dnn_shared_chunk_accommodate(dnn_shared_chunk_t *self,
                                        size_t vbuffer_bsize)
{
  uint32_t remain;
  remain = (uint32_t)self->allocated_bsize - (uint32_t)self->used_bsize;
  return remain >= round_up((uint32_t)vbuffer_bsize, 4u);
}

static inline int
  dnn_shared_chunk_preallocate(dnn_shared_chunk_t *self,
                               uint8_t vbuffer_idx,
                               dnn_vbuffer_alloc_info_t *info)
{
  int ret = RT_RET_NOERROR;
  size_t vbuffer_bsize = alloc_info->bsize_list[vbuffer_idx];
  if (vbuffer_idx < alloc_info->vbuffer_num &&
      dnn_shared_chunk_accommodate(self, vbuffer_bsize))
    {
      alloc_info->addr_list[vbuffer_idx] = self->data + self->used_bsize;
      self->used_bsize += round_up((uint32_t)vbuffer_bsize, 4u);
    }
  else
    {
      ret = -EPERM;
    }

  return ret;
}

static size_t
  dnn_vbuffer_alloc_info_remaining_bsize(dnn_vbuffer_alloc_info_t *info)
{
  size_t ret = 0u;
  for (uint8_t idx = 0; idx < info->vbuffer_num; idx++)
    {
      if (info->addr_list[idx] == NULL)
        {
          ret += round_up(info->bsize_list[idx], 4u);
        }
    }

  return ret;
}

static inline dnn_shared_chunk_t *
  dnn_create_chunk(dnn_vbuffer_alloc_info_t *info)
{
  /* reserve memory for new_chunk */

  dnn_shared_chunk_t *new_chunk = NULL, *last;
  size_t chunk_bsize = 0u;
  chunk_bsize += sizeof(dnn_shared_chunk_t);
  chunk_bsize += dnn_vbuffer_alloc_info_remaining_bsize(alloc_info);
  chunk_bsize += (4u - 1u); /* padding to 4-byte align new_chunk->data */
  new_chunk = (dnn_shared_chunk_t *)dnn_controller_malloc(chunk_bsize);
  if (new_chunk != NULL)
    {
      /* initialize dnn_shared_chunk_t */

      memset(new_chunk, 0, chunk_bsize);
      new_chunk->data = ALIGN((void *)(new_chunk + 1), 4u);
      size_t header_bsize = new_chunk->data - (void *)new_chunk;
      new_chunk->allocated_bsize = chunk_bsize - header_bsize;

      /* add new_chunk to a linked list, dnn_controller_context_t::chunks */

      if (chunks)
        {
          for (last = chunks; last->next != NULL; last = last->next);
          last->next = new_chunk;
        }
      else
        {
          chunks = new_chunk; /* create 1st chunk */
        }
    }

  return new_chunk;
}

void dnn_reset_chunk_usage(void)
{
  dnn_shared_chunk_t *chunk;

  if (chunks)
    {
      for (chunk = chunks; chunk != NULL; chunk = chunk->next)
        {
          chunk->used_bsize = 0u;
        }
    }
}

void dnn_deallocate_chunks(dnn_vbuffer_alloc_info_t *info)
{
  for (uint8_t idx = 0; idx < alloc_info->actual_alloc_count; idx++)
    {
      dnn_shared_chunk_update_ref_count(alloc_info->addr_list[idx], -1);
    }
}

/* determine how to allocate shared_chunk to variable buffers (preallocate),
 * and store the result into dnn_vbuffer_alloc_info_t::addr_list.
 * This algorithm is composed of these 3 steps:
 *  1. find a shared_chunk for each variable buffer by the first-fit
 *     algorithm - don't create new shared_chunks in this step
 *  2. create a new single shared_chunk for all the variable buffers
 *     which didn't fit in existing shared_chunk in 1.
 *  3. slice the new single shared_chunk and allocate sliced pieces to
 *     variable buffers which didn't fit in existing shared_chunk in 1
 */

int dnn_preallocate_chunks(dnn_vbuffer_alloc_info_t *info)
{
  int ret = RT_RET_NOERROR;
  dnn_shared_chunk_t *chunk, *new_chunk;

  /* step 1 */

  for (uint8_t idx = 0; idx < alloc_info->vbuffer_num; idx++)
    {
      size_t vbuffer_bsize = alloc_info->bsize_list[idx];
      for (chunk = chunks; chunk != NULL; chunk = chunk->next)
        {
          if (dnn_shared_chunk_accommodate(chunk, vbuffer_bsize))
            {
              dnn_shared_chunk_preallocate(chunk, idx, alloc_info);
            }
        }
    }

  /* count the total size of variable buffers that preallocation
   * is NOT done
   */

  if (dnn_vbuffer_alloc_info_remaining_bsize(alloc_info) != 0u)
    {
      new_chunk = dnn_create_chunk(alloc_info); /* step 2 */
      if (new_chunk != NULL)
        {
          /* step 3 */

          for (uint8_t idx = 0; idx < alloc_info->vbuffer_num; idx++)
            {
              if (alloc_info->addr_list[idx] == NULL)
                {
                  /* new_chunk can accommodate the remaining
                   * variable buffers
                   */

                  dnn_shared_chunk_preallocate(new_chunk, idx, alloc_info);
                }
            }
        }
      else
        {
          ret = -ENOMEM;
        }
    }

  return ret;
}
