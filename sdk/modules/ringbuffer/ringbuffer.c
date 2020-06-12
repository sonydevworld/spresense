/****************************************************************************
 * modules/ringbuffer/ringbuffer.c
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "ringbuffer/ringbuffer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RINGBUFFER
#  define ringbuffer_debug(x, ...) printf("%s "x, __func__, ##__VA_ARGS__)
#else
#  define ringbuffer_debug(x, ...)
#endif

#ifndef MIN
#  define MIN(a,b)  (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ringbuf_end
 ****************************************************************************/

static inline uint8_t *ringbuf_end(FAR struct ringbuf_s *rb)
{
  return rb->buf + ringbuf_buffersize(rb);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ringbuf_new
 *
 * Description:
 *   Allocates a new Ring Buffer.
 *
 * Input Parameters:
 *   size  Size of Ring Buffer to allocate.
 *
 * Returned Value:
 *   On success, the allocated Ring Buffer is returned.
 *   On failure, NULL is returned.
 *
 ****************************************************************************/

FAR struct ringbuf_s *ringbuf_new(size_t size)
{
  FAR struct ringbuf_s *rb;

  rb = (FAR struct ringbuf_s *)malloc(sizeof(struct ringbuf_s));
  if (rb)
    {
      rb->buf = (FAR uint8_t *)calloc(1, size);
      if (rb->buf)
        {
          rb->head = rb->buf;
          rb->tail = rb->buf;
          rb->size = size;
          rb->full = false;

          ringbuffer_debug("buf:%p head:%p tail:%p end:%p\n",
                           rb->buf, rb->head, rb->tail, ringbuf_end(rb));
        }
      else
        {
          free(rb);
          rb = NULL;
        }
    }

  return rb;
}

/****************************************************************************
 * Name: ringbuf_free
 *
 * Description:
 *   Release a Ring Buffer.
 *
 * Input Parameters:
 *   rb  Pointer to a Ring Buffer to release.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ringbuf_free(FAR struct ringbuf_s *rb)
{
  if (rb && rb->buf)
    {
      free(rb->buf);
      free(rb);
    }
}

/****************************************************************************
 * Name: ringbuf_read
 *
 * Description:
 *   Read from a Ring Buffer.
 *
 * Input Parameters:
 *   rb  Pointer to a Ring Buffer to read.
 *   buf  Pointer to buffer to store data that read from a Ring Buffer.
 *   count  Bytes to read.
 *
 * Returned Value:
 *   On success, The number of bytes read.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

ssize_t ringbuf_read(FAR struct ringbuf_s *rb, FAR void *buf, size_t count)
{
  size_t remain;

  if (!rb)
    {
      return -EINVAL;
    }

  ringbuffer_debug("count:%d\n", count);

  count = MIN(ringbuf_bytesused(rb), count);
  if (!count)
    {
      return count;
    }

  /* wrap check */

  if ((rb->head + count) >= ringbuf_end(rb))
    {
      /*
       *  Read buffer from head. (XXX part)
       *  ----------------------------
       *  |                   |XXXXXX|
       *  ----------------------------
       *  buf                 head   end
       */

      memcpy(buf, (FAR const void *)rb->head, ringbuf_end(rb) - rb->head);

      remain = count - (ringbuf_end(rb) - rb->head);
      if (remain)
        {
          buf = (FAR uint8_t *)buf + (ringbuf_end(rb) - rb->head);

          /*
           * Read the remain buffer from the beginning. (XXX part)
           *  ----------------------------
           *  |XXXXX|                    |
           *  ----------------------------
           *  buf   head(latest)        end
           */

          memcpy(buf, (FAR const void *)rb->buf, remain);
        }
      rb->head = rb->buf + remain;
    }
  else
    {
      memcpy(buf, (FAR const void *)rb->head, count);
      rb->head += count;
    }

  if (rb->full)
    {
      rb->full = false;
    }

  ringbuffer_debug("buf:%p head:%p tail:%p end:%p full:%d\n",
                   rb->buf, rb->head, rb->tail, ringbuf_end(rb), rb->full);

  return count;
}

/****************************************************************************
 * Name: ringbuf_write
 *
 * Description:
 *   Write to a Ring Buffer.
 *
 * Input Parameters:
 *   rb  Pointer to a Ring Buffer to read.
 *   buf  Pointer to buffer to stored data that write to a Ring Buffer.
 *   count  Bytes to write.
 *
 * Returned Value:
 *   On success, The number of bytes written.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

ssize_t ringbuf_write(FAR struct ringbuf_s *rb, FAR void *buf, size_t count)
{
  size_t remain;

  if (!rb)
    {
      return -EINVAL;
    }

  ringbuffer_debug("count:%d\n", count);

  if (ringbuf_bytesavail(rb) < count)
    {
      return -ENOSPC;
    }
  if (!count)
    {
      return count;
    }

  /* wrap check */

  if ((rb->tail + count) >= ringbuf_end(rb))
    {
      /*
       *  Write buffer to tail. (XXX part)
       *  ----------------------------
       *  |                   |XXXXXX|
       *  ----------------------------
       *  buf                 tail   end
       */

      memcpy((FAR void *)rb->tail, (FAR const void *)buf,
             ringbuf_end(rb) - rb->tail);

      remain = count - (ringbuf_end(rb) - rb->tail);
      if (remain)
        {
          buf = (FAR uint8_t *)buf + (ringbuf_end(rb) - rb->tail);

          /*
           * Write the remain buffer to the beginning. (XXX part)
           *  ----------------------------
           *  |XXXXX|                    |
           *  ----------------------------
           *  buf   tail(latest)         end
           */

          memcpy((FAR void *)rb->buf, (FAR const void *)buf, remain);
        }
      rb->tail = rb->buf + remain;
    }
  else
    {
      memcpy((FAR void *)rb->tail, (FAR const void *)buf, count);
      rb->tail += count;
    }

  if (rb->head == rb->tail)
    {
      rb->full = true;
    }

  ringbuffer_debug("buf:%p head:%p tail:%p end:%p full:%d\n",
                   rb->buf, rb->head, rb->tail, ringbuf_end(rb), rb->full);

  return count;
}

/****************************************************************************
 * Name: ringbuf_buffersize
 *
 * Description:
 *   Gets the buffer size.
 *
 * Input Parameters:
 *   rb  Pointer to a Ring Buffer.
 *
 * Returned Value:
 *   The buffer size.
 *
 ****************************************************************************/

size_t ringbuf_buffersize(FAR struct ringbuf_s *rb)
{
  if (!rb)
    {
      return 0;
    }

  return rb->size;
}

/****************************************************************************
 * Name: ringbuf_bytesused
 *
 * Description:
 *   Gets the number of bytes used.
 *
 * Input Parameters:
 *   rb  Pointer to a Ring Buffer.
 *
 * Returned Value:
 *   The number of bytes used.
 *
 ****************************************************************************/

size_t ringbuf_bytesused(FAR struct ringbuf_s *rb)
{
  if (!rb)
    {
      return 0;
    }

  if (rb->full)
    {
      return ringbuf_buffersize(rb);
    }

  if (rb->head > rb->tail)
    {
      return ringbuf_buffersize(rb) - (rb->head - rb->tail);
    }
  else
    {
      return (rb->tail - rb->head);
    }
}

/****************************************************************************
 * Name: ringbuf_bytesavail
 *
 * Description:
 *   Gets the number of bytes free.
 *
 * Input Parameters:
 *   rb  Pointer to a Ring Buffer.
 *
 * Returned Value:
 *   The number of bytes free.
 *
 ****************************************************************************/

size_t ringbuf_bytesavail(FAR struct ringbuf_s *rb)
{
  if (!rb)
    {
      return 0;
    }

  return ringbuf_buffersize(rb) - ringbuf_bytesused(rb);
}
