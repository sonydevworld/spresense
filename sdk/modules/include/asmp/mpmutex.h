/****************************************************************************
 * modules/include/asmp/mpmutex.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
/**
 * @file mpmutex.h
 */

#ifndef __INCLUDE_ASMP_MPMUTEX_H
#define __INCLUDE_ASMP_MPMUTEX_H

/**
 * @defgroup mpmutex MP mutex
 * @{
 *
 * MP mutex provide synchronization mechanism between supervisor and worker.
 */

#include <sys/types.h>
#include <asmp/types.h>

/**
 * @defgroup mpmutex_datatype Data Types
 * @{
 */
/**
 * @typedef mpmutex_t
 * MP mutex object
 */

typedef struct mpmutex
{
  mpobj_t     super;            /**< Super class */
  int         fd;               /**< File descriptor for semaphore device */
  int8_t      tag;              /**< The tag */
} mpmutex_t;

/** @} mpmutex_datatype */

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/
/**
 * @defgroup mpmutex_funcs Functions
 * @{
 */

/**
 * Initialize MP mutex
 *
 * @param [in,out] mutex: MP mutex object
 * @param [in] key: Unique object ID
 *
 * @return On success, mpmutex_init() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalild argument
 * @retval -ENOENT: No allocate mutex
 */

int mpmutex_init(mpmutex_t *mutex, key_t key);

/**
 * Destroy MP mutex
 *
 * @param [in,out] mutex: MP mutex object
 *
 * @return On success, mpmutex_destroy() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 */

int mpmutex_destroy(mpmutex_t *mutex);

/**
 * Lock MP mutex
 *
 * mpmutex_lock() is lock specified @a mutex. If @a mutex is already locked,
 * then mpmutex_lock() waits for it to be unlocked by locker.
 *
 * @param [in,out] mutex: MP mutex object
 *
 * @return On success, mpmutex_lock() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 */

int mpmutex_lock(mpmutex_t *mutex);

/**
 * Try to lock MP mutex
 *
 * mpmutex_trylock() is try to lock specified @a mutex. If @a mutex is already
 * locked, then return @c -EBUSY.
 *
 * @param [in,out] mutex: MP mutex object
 *
 * @return On success, mpmutex_trylock() returns 0. On error, it returns an
 * error number.
 * @retval -EINVAL: Invalid argument
 * @retval -EBUSY: Mutex is locked
 */

int mpmutex_trylock(mpmutex_t *mutex);

/**
 * Unlock MP mutex
 *
 * @param [in,out] mutex: MP mutex object
 *
 * @return On success, mpmutex_unlock() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 */

int mpmutex_unlock(mpmutex_t *mutex);

/** @} mpmutex_funcs */

#undef EXTERN
#ifdef __cplusplus
}
#endif

/** @} mpmutex */

#endif /* __INCLUDE_ASMP_MPMUTEX_H */
