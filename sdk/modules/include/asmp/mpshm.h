/****************************************************************************
 * modules/include/asmp/mpshm.h
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
 * @file mpshm.h
 */

#ifndef __INCLUDE_ASMP_MPSHM_H
#define __INCLUDE_ASMP_MPSHM_H

/**
 * @defgroup mpshm MP shared memory
 *
 * MP shared memory provides shared memory management for used by supervisor and
 * worker.
 * Supervisor and worker can be exchange large data.
 *
 * @{
 */

#include <sys/types.h>
#include <asmp/types.h>
#include <semaphore.h>

/* Command definitions for mpshm_control. */

#define MPC_POWERON    1        /**< Set shared memory to power on */
#define MPC_POWEROFF   2        /**< Set shared memory to power off */
#define MPC_RETENTION  3        /**< Set shared memory to retention state */

/**
 * @defgroup mpshm_datatype Data Types
 * @{
 */
/**
 * @typedef mpshm_t
 * MP shared memory object
 */

typedef struct mpshm
{
  mpobj_t     super;            /**< Super class */
  int8_t      tag;              /**< Address conversion tag */
  uintptr_t   paddr;            /**< Physical address */
  size_t      size;             /**< Size of shared memory */
  sem_t       exc;              /**< For exclusive access */
} mpshm_t;

/** @} mpshm_datatype */

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
 * @defgroup mpshm_funcs Functions
 * @{
 */

/**
 * Initialize MP shared memory
 *
 * mpshm_init() initialize MP shared memory object and allocate actual shared
 * memory area.
 *
 * @param [in,out] shm: MP shared memory object
 * @param [in] key: Unique object ID
 * @param [in] size: Size of shared memory
 *
 * @return On success, mpshm_init() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 * @retval -ENOMEM: No memory space left
 *
 * @note MP shared memory area is always allocated in meaningful size for
 * platform. On CXD5602, its size is 128KB.
 */

int mpshm_init(mpshm_t *shm, key_t key, size_t size);

/**
 * Destroy MP shared memory
 *
 * @param [in,out] shm: MP shared memory object
 *
 * @return On success, mpshm_destroy() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 */

int mpshm_destroy(mpshm_t *shm);

/**
 * Attach MP shared memory
 *
 * mpshm_attach() is map shared memory to virtual address. User must be call this
 * function to use allocated shared memory.
 *
 * @param [in,out] shm: MP shared memory object
 * @param [in] shmflg: Flags
 *
 * @return On success, mpshm_attach() returns virtual address. On error, it
 * returns NULL.
 *
 * @note @a shmflg is reserved for future extention. Not affected now.
 */

void *mpshm_attach(mpshm_t *shm, int shmflg);

/**
 * Detach MP shared memory
 *
 * @param [in,out] shm: MP shared memory object
 *
 * @return On success, mpshm_detach() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 */

int mpshm_detach(mpshm_t *shm);

/**
 * MP shared memory control
 *
 * mpshm_control() now provides RAM power control for managed memory area.
 * User can be control power mode to on, off and retention.
 * ASMP framework performs power control automatically, so typically user
 * unnecessary to power control manually.
 *
 * @param [in,out] shm: MP shared memory object
 * @param [in] cmd: Command
 *     - #MPC_POWERON
 *     - #MPC_POWEROFF
 *     - #MPC_RETENTION
 *
 * @param [in,out] buf: Arguments for @a cmd
 *
 * @return On success, mpshm_control() returns 0. On error it returns an error
 * number.
 * @retval -ENOTTY: Unknown command
 *
 * @attention If set #MPC_RETENTION or #MPC_POWEROFF, then can't access to any
 * management area.
 */

int mpshm_control(mpshm_t *shm, int cmd, void *buf);

/**
 * Convert virtual address to physical address
 *
 * @param [in,out] shm: MP shared memory object
 * @param [in] vaddr: Virtual address
 *
 * @return On success, mpshm_virt2phys() returns physical address. On error, it
 * returns 0.
 *
 * @note This function affects only mapped virtual addresses.
 */

uintptr_t mpshm_virt2phys(mpshm_t *shm, void *vaddr);

/**
 * Convert physical address to virtial address
 *
 * @param [in,out] shm: MP shared memory object
 * @param [in] paddr: Physical address
 *
 * @return On success, mpshm_phys2virt() returns virtual address. On error, it
 * returns NULL.
 */

void *mpshm_phys2virt(mpshm_t *shm, uintptr_t paddr);

/**
 * Map allocated memory into specified virtual address
 *
 * @param [in,out] shm: MP shared memory object
 * @param [in] vaddr: Virtual address
 *
 * @return On success, mpshm_remap() returns 0. On error, it returns on error
 * number.
 * @retval -EINVAL: Invalid argument
 * @retval -ENOENT: Virtual space is in use
 */

int mpshm_remap(mpshm_t *shm, void *vaddr);

/**
 * Unmap mapped memory
 *
 * It's just an alias of mpshm_detach().
 */

#define mpshm_unmap(shm) mpshm_detach(shm);

/** @} mpshm_funcs */

#undef EXTERN
#ifdef __cplusplus
}
#endif

/** @} mpshm */

#endif /* __INCLUDE_ASMP_MPSHM_H */
