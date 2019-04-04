/****************************************************************************
 * modules/include/asmp/mpmq.h
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
 * @file mpmq.h
 */

#ifndef __INCLUDE_ASMP_MPMQ_H
#define __INCLUDE_ASMP_MPMQ_H

/**
 * @defgroup mpmq MP message queue
 *
 * MP message queue provides inter process communication between supervisor and
 * MP tasks.
 *
 * @{
 */

#include <sys/types.h>
#include <asmp/types.h>
#include <asmp/mpsignal.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

/* Timeout definitions for mpmq_timedreceive() */

#define MPMQ_NONBLOCK 0xfffffffful  /**< Non-blocking mode */

/********************************************************************************
 * Public Type Declarations
 ********************************************************************************/
/**
 * @defgroup mpmq_datatypes Data types
 * @{
 */

/**
 * @typedef mpmq_t
 * MP message queue object
 */

typedef struct mpmq
{
  mpobj_t     super;            /**< Super class */
  cpuid_t     cpuid;            /**< Target CPU ID */
  uint32_t    flags;            /**< Flags */
} mpmq_t;

/** @} mpmq_datatypes */

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
 * @defgroup mpmq_funcs Functions
 * @{
 */

/**
 * Initialize MP message queue
 *
 * mpmq_init() initialize MP message queue object. This function needs raw CPU
 * ID at @a cpuid. User can use mptask_getcpuid() to get asigned CPU ID.
 *
 * On the worker side, specify known key to initialize with supervisor. In this
 * case, @a cpuid is ignored.
 * If use mpmq for communicate with sub cores, set 0 to key and target CPU ID to
 * @a cpuid.
 *
 * @param [in,out] mq: MP message queue object
 * @param [in] key: Unique object ID
 * @param [in] cpuid: Target CPU ID
 *
 * @return On success, mpmq_init() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 */

int mpmq_init(mpmq_t *mq, key_t key, cpuid_t cpuid);

/**
 * Destroy MP message queue
 *
 * @param [in,out] mq: MP message queue object
 *
 * @return On success, mpmq_destroy() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 */

int mpmq_destroy(mpmq_t *mq);

/**
 * Send message via MP message queue
 *
 * @param [in,out] mq: MP message queue object
 * @param [in] msgid: User defined message ID (0-127)
 * @param [in] data: Message data
 *
 * @return On success, mpmq_send() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 */

int mpmq_send(mpmq_t *mq, int8_t msgid, uint32_t data);

/**
 * Send message via MP message queue with timeout
 *
 * @param [in,out] mq: MP message queue object
 * @param [in] msgid: User defined message ID (0-127)
 * @param [in] data: Message data
 * @param [in] ms: Time out (milliseconds). This parameter is unused.
 *
 * @return On success, mpmq_timedsend() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 */

int mpmq_timedsend(mpmq_t *mq, int8_t msgid, uint32_t data,
                   uint32_t ms);

/**
 * Receive message via MP message queue
 *
 * @param [in,out] mq: MP message queue object
 * @param [out] data: Message data
 *
 * @return On success, mpmq_receive() returns message ID. On error, it returns an
 * error number.
 * @retval -EINVAL: Invalid argument
 */

int mpmq_receive(mpmq_t *mq, uint32_t *data);

/**
 * Receive message via MP message queue with timeout
 *
 * @param [in,out] mq: MP message queue object
 * @param [out] data: Message data
 * @param [in] ms: Time out (milliseconds). If ms is zero, then it waits forever
 * until receiving message. If ms is MPMQ_NONBLOCK, then it behaves as polling
 * without blocking.
 *
 * @return On success, mpmq_timedreceive() returns message ID. On error, it
 * returns an error number.
 * @retval -EINVAL: Invalid argument
 * @retval -ETIMEDOUT: Timed out
 * @retval -EAGAIN: Try again when data hasn't come with non-blocking mode
 */

int mpmq_timedreceive(mpmq_t *mq, uint32_t *data, uint32_t ms);

/**
 * Request signal when MP message arrival
 *
 * @param [in,out] mq: MP message queue object
 * @param [in] signo: Signal number
 * @param [in] sigdata: Signal data
 *
 * @return On success, mpmq_notify() returns 0. On error, it returns an error
 * number.
 * @retval -EINVAL: Invalid argument
 */

int mpmq_notify(mpmq_t *mq, int signo, void *sigdata);

/** @} mpmq_funcs */

#undef EXTERN
#ifdef __cplusplus
}
#endif

/** @} mpmq */

#endif
