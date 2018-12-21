/****************************************************************************
 * modules/lte/altcom/include/api/altcom_callbacks.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_ALTCOM_CALLBACKS_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_ALTCOM_CALLBACKS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altcomcallbacks_init
 *
 * Description:
 *   Initialize altcom API callbacks.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomcallbacks_init(void);

/****************************************************************************
 * Name: altcomcallbacks_fin
 *
 * Description:
 *   Finalize altcom API callbacks.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomcallbacks_fin(void);

/****************************************************************************
 * Name: altcomcallbacks_reg_cb
 *
 * Description:
 *   Registration altcom API callback.
 *
 * Input Parameters:
 *   cb_ptr     Pointer of registration callback.
 *   id         Callback function id.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomcallbacks_reg_cb(void *cb_ptr, int32_t id);

/****************************************************************************
 * Name: altcomcallbacks_unreg_cb
 *
 * Description:
 *   Unregistration altcom API callback.
 *
 * Input Parameters:
 *   id       Target callback function id.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomcallbacks_unreg_cb(int32_t id);

/****************************************************************************
 * Name: altcomcallbacks_get_cb
 *
 * Description:
 *   Get altcom API callback.
 *
 * Input Parameters:
 *   id    Target callback function id.
 *
 * Returned Value:
 *   Pointer of Callback function. When not registered callbacke return NULL.
 *
 ****************************************************************************/

void *altcomcallbacks_get_cb(int32_t id);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

int32_t altcomcallbacks_get_unreg_cb(int32_t id, void **callback);

/****************************************************************************
 * Name: altcomcallbacks_chk_reg_cb
 *
 * Description:
 *   Check and registration altcom API callback.
 *
 * Input Parameters:
 *   cb         Pointer of altcom API callback function.
 *   id         Callback function id.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomcallbacks_chk_reg_cb(void *cb, int32_t id);

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_ALTCOM_CALLBACKS_H */
