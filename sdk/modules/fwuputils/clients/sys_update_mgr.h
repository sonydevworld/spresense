/****************************************************************************
 * modules/fwuputils/clients/sys_update_mgr.h
 *
 *   Copyright 2018, 2021 Sony Semiconductor Solutions Corporation
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
 * @file       sys_update_mgr.h
 */

#ifndef SYS_UPDATE_MGR_H
#define SYS_UPDATE_MGR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup sys_update_mgr Update Manager
 *
 * <pre>\#include <system/sys_update_mgr.h></pre>
 *
 * @{
 */

/**
 * File types
 */
enum {
    UM_TYPE_FIRMWARE,           /**< Firmware */
    UM_TYPE_SBL,                /**< Second boot loader */
};

/**
 * Update manager instance handle
 */
typedef void * UM_Handle;

/**
 * Initialize Update Manager
 *
 * @param[in] need_recovery: true to need recovery from check point. see fw_um_checkpoint().
 *
 * @return 0 on success, otherwise error
 * @retval -ENOMEM: memory allocation failure
 * @retval -EFAULT: check point not loaded
 *
 * @par Blocking
 *      Yes
 * @par Context
 *      Task only
 * @par Reentrant
 *      No
 */
int fw_um_init(bool need_recovery);

/**
 * Create update manager object for scheduling firmware will be update.
 *
 * @param[in] keyfile: key file name when secure environment, NULL is OK
 * @param[in] filesize: file size for downloading firmware
 * @param[in] type: file type
 *
 * @return UM_Handle on success, NULL is error
 *
 * @par Blocking
 *      Yes
 * @par Context
 *      Task only
 * @par Reentrant
 *      Yes
 */
UM_Handle fw_um_open(const char *keyfile, uint32_t filesize, uint32_t type);

/**
 * Commit firmware data.
 * User can commit a partial or whole firmware image.
 *
 * @param[in] handle: update manager object handle
 * @param[in] data: firmware data. must be 4 byte aligned address.
 * @param[in] size: size of @a data. must be multiple of 16 byte.
 *
 * @return 0 on success, otherwise error
 * @retval -EINVAL: invalid argument
 * @retval -ENOSPC: disk full
 * @retval -EFAULT: file open error
 *
 * @par Blocking
 *      Yes
 * @par Context
 *      Task only
 * @par Reentrant
 *      Yes
 */
int fw_um_commit(UM_Handle handle, const void *data, uint32_t size);

/**
 * Close and delete update manager object.
 * A closed object to be scheduled for update sequence.
 *
 * @param[in] handle: update manager object handle
 *
 * @return 0 on success, otherwise error
 * @retval -ENOENT: invalid handle
 * @retval -EBUSY:  not enough data
 * @retval -EFAULT: internal error
 * @retval -ENOMEM: memory allocation failure
 * @retval -ENOSPC: disk full
 * @retval -EINVAL: firmware is invalid
 *
 * @par Blocking
 *      Yes
 * @par Context
 *      Task only
 * @par Reentrant
 *      Yes
 */
int fw_um_close(UM_Handle handle);

/**
 * Create recovery check point.
 *
 * User can use this feature to stop downloading (also shutdown).
 * Pass true to need_recovery on fw_um_init() to restart downloading,
 * all of closed objects are scheduled again until this check point.
 *
 * Check point is always overwritten.
 *
 * @return 0 on success, otherwise error
 * @retval -EFAULT: recovery point not saved
 *
 * @par Blocking
 *      Yes
 * @par Context
 *      Task only
 * @par Reentrant
 *      Yes
 */
int fw_um_checkpoint(void);

/**
 * Reboot and into update sequence.
 *
 * @return This API never return on success, if returned, then always error.
 * @retval -EFAULT: one or more unclosed objects are exists
 *
 * @attention This API force reboot immediately. User make sure to ready to
 * shutting down. All of updating firmwares are must be closed.
 */
int fw_um_doupdatesequence(void);

/**
 * Abort update manager
 *
 * This API freeing internal used memory and initialize update manager, but
 * checkpoint is not cleared.
 * User can be use this API to pause/resume or restart download sequence without
 * heap dirty.
 */
void fw_um_abort(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
