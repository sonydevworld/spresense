/****************************************************************************
 * modules/lte/altcom/include/api/altcom_status.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_ALTCOM_STATUS_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_ALTCOM_STATUS_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALTCOM_STATUS_UNINITIALIZED     (0)
#define ALTCOM_STATUS_INITIALIZED       (1)
#define ALTCOM_STATUS_POWERON_ONGOING   (2)
#define ALTCOM_STATUS_RESET_ONGOING     (3)
#define ALTCOM_STATUS_POWER_ON          (4)
#define ALTCOM_STATUS_MIN               (ALTCOM_STATUS_UNINITIALIZED)
#define ALTCOM_STATUS_MAX               (ALTCOM_STATUS_POWER_ON)

#define ALTCOM_STATUS_REG_KEEP          (0)
#define ALTCOM_STATUS_REG_CLR           (1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef int32_t (*altcom_stat_chg_cb_t)(int32_t new_stat, int32_t old_stat);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_get_status
 *
 * Description:
 *   Get altcom status.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Return current altcom status.
 *
 ****************************************************************************/

int32_t altcom_get_status(void);

/****************************************************************************
 * Name: altcom_set_status
 *
 * Description:
 *   Set altcom status.
 *
 * Input Parameters:
 *   status     altcom status.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcom_set_status(int32_t status);

/****************************************************************************
 * Name: altcomstatus_reg_statchgcb
 *
 * Description:
 *   Registoration altcom status change callbacks.
 *
 * Input Parameters:
 *   cb     Status change callback.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomstatus_reg_statchgcb(altcom_stat_chg_cb_t cb);

/****************************************************************************
 * Name: altcomstatus_unreg_statchgcb
 *
 * Description:
 *   Unregistration altcom status change callbacks.
 *
 * Input Parameters:
 *   cb     Status change callback.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcomstatus_unreg_statchgcb(altcom_stat_chg_cb_t cb);

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_ALTCOM_STATUS_H */
