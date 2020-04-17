/****************************************************************************
 * modules/include/lte/altcom/altcom_api.h
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

#ifndef __MODULES_INCLUDE_LTE_ALTCOM_ALTCOM_API_H
#define __MODULES_INCLUDE_LTE_ALTCOM_ALTCOM_API_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "lte/lte_api.h"

/****************************************************************************
 * Public function prototypes
 ****************************************************************************/

int32_t altcom_initialize(void);
int32_t altcom_finalize(void);
int32_t altcom_set_report_restart(restart_report_cb_t restart_callback);
int32_t altcom_set_report_localtime(localtime_report_cb_t localtime_callback);
int32_t altcom_power_on(void);
int32_t altcom_power_off(void);
int32_t altcom_activate_pdn_sync(lte_apn_setting_t *apn, lte_pdn_t *pdn);
int32_t altcom_deactivate_pdn_sync(uint8_t session_id);
int32_t altcom_radio_on_sync(void);
int32_t altcom_radio_off_sync(void);


#endif /* __MODULES_INCLUDE_LTE_ALTCOM_ALTCOM_API_H */
