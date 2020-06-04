/****************************************************************************
 * modules/include/lte/altcom/altcom_select_ext.h
 *
 *   Copyright 2018, 2020 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_INCLUDE_LTE_ALTCOM_ALTCOM_SELECT_EXT_H
#define __MODULES_INCLUDE_LTE_ALTCOM_ALTCOM_SELECT_EXT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "altcom_select.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*altcom_select_async_cb_t)(int32_t ret_code, int32_t err_code,
                                         int32_t id,
                                         altcom_fd_set *readset,
                                         altcom_fd_set *writeset,
                                         altcom_fd_set *exceptset,
                                         void *priv);


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

/****************************************************************************
 * Name: altcom_select_async
 ****************************************************************************/

int altcom_select_async(int maxfdp1, altcom_fd_set *readset,
                        altcom_fd_set *writeset, altcom_fd_set *exceptset,
                        altcom_select_async_cb_t callback, void* priv);

/****************************************************************************
 * Name: altcom_select_async_exec_callback
 ****************************************************************************/

int altcom_select_async_exec_callback(int32_t id, int32_t ret_code,
                                      int32_t err_code,
                                      altcom_fd_set *readset,
                                      altcom_fd_set *writeset,
                                      altcom_fd_set *exceptset);

/****************************************************************************
 * Name: altcom_select_async_cancel
 ****************************************************************************/

int altcom_select_async_cancel(int id, bool is_send);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MODULES_INCLUDE_LTE_ALTCOM_ALTCOM_SELECT_EXT_H */
