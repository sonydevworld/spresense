/****************************************************************************
 * modules/include/ringbuffer/ringbuffer.h
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

#ifndef __MODULES_INCLUDE_RINGBUFFER_RINGBUFFER_H
#define __MODULES_INCLUDE_RINGBUFFER_RINGBUFFER_H

/**
 * @defgroup rb Library for Ring Buffer
 *
 * @{
 * @file  ringbuffer.h
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ringbuf_s
{
  FAR uint8_t *buf;  /**< Pointer to buffer */
  FAR uint8_t *head; /**< Pointer to head of buffer */
  FAR uint8_t *tail; /**< Pointer to tail of buffer */
  size_t size;       /**< Buffer size */
  bool full;         /**< Buffer full */
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/** @name Functions */
/** @{ */

/**
 * Allocates a new Ring Buffer.
 *
 * @param [in] size: Size of Ring Buffer to allocate.
 *
 * @return On success, the allocated Ring Buffer is returned.
 * On failure, NULL is returned.
 */

FAR struct ringbuf_s *ringbuf_new(size_t size);

/**
 * Release a Ring Buffer.
 *
 * @param [in] rb: Pointer to a Ring Buffer to release.
 *
 */

void ringbuf_free(FAR struct ringbuf_s *rb);

/**
 * Read from a Ring Buffer.
 *
 * @param [in] rb: Pointer to a Ring Buffer to read.
 * @param [in] buf: Pointer to buffer to store data that read from
 *                  a Ring Buffer.
 * @param [in] count: Bytes to read.
 *
 * @return On success, The number of bytes read.
 * On failure, negative value is returned according to <errno.h>.
 */

ssize_t ringbuf_read(FAR struct ringbuf_s *rb, FAR void *buf, size_t count);

/**
 * Write to a Ring Buffer.
 *
 * @param [in] rb: Pointer to a Ring Buffer to write.
 * @param [in] buf: Pointer to buffer to stored data that write to
 *                  a Ring Buffer.
 * @param [in] count: Bytes to write.
 *
 * @return On success, The number of bytes written.
 * On failure, negative value is returned according to <errno.h>.
 */

ssize_t ringbuf_write(FAR struct ringbuf_s *rb, FAR void *buf, size_t count);

/**
 * Gets the buffer size.
 *
 * @param [in] rb: Pointer to a Ring Buffer.
 *
 * @return The buffer size.
 */

size_t ringbuf_buffersize(FAR struct ringbuf_s *rb);

/**
 * Gets the number of bytes used.
 *
 * @param [in] rb: Pointer to a Ring Buffer.
 *
 * @return The number of bytes used.
 */

size_t ringbuf_bytesused(FAR struct ringbuf_s *rb);

/**
 * Gets the number of bytes free.
 *
 * @param [in] rb: Pointer to a Ring Buffer.
 *
 * @return The number of bytes free.
 */

size_t ringbuf_bytesavail(FAR struct ringbuf_s *rb);

/** @} */

/** @} */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MODULES_INCLUDE_RINGBUFFER_RINGBUFFER_H */
