/****************************************************************************
 * modules/lte/altcom/api/lte/lte_setpsm.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <errno.h>

#include "lte/lte_api.h"
#include "dbg_if.h"
#include "osal.h"
#include "apiutil.h"
#include "apicmdgw.h"
#include "apicmd_setpsm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SETPSM_DATA_LEN      (sizeof(struct apicmd_cmddat_setpsm_s))
#define SETPSM_RAT_UNIT_MIN  LTE_PSM_T3324_UNIT_2SEC
#define SETPSM_RAT_UNIT_MAX  LTE_PSM_T3324_UNIT_6MIN
#define SETPSM_TAU_UNIT_MIN  LTE_PSM_T3412_UNIT_2SEC
#define SETPSM_TAU_UNIT_MAX  LTE_PSM_T3412_UNIT_320HOUR
#define SETPSM_TIMER_VAL_MIN (1)
#define SETPSM_TIMER_VAL_MAX (31)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern set_psm_cb_t g_setpsm_callback;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_set_psm
 *
 * Description:
 *   Set PSM settings.
 *
 * Input Parameters:
 *   callback Callback function to notify that set PSM settings is
 *            completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_psm(lte_psm_setting_t *settings, set_psm_cb_t callback)
{
  int32_t                        ret;
  struct apicmd_cmddat_setpsm_s *cmddat;

  /* Return error if callback is NULL */

  if (!settings || !callback)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
    }

  /* Check if the library is initialized */

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      return -EPERM;
    }

  if (settings->enable)
    {
      if (settings->req_active_time.unit < SETPSM_RAT_UNIT_MIN ||
        SETPSM_RAT_UNIT_MAX < settings->req_active_time.unit)
        {
          DBGIF_LOG1_ERROR("Invalid argument. RAT unit:%d\n", settings->req_active_time.unit);
          return -EINVAL;
        }

      if (settings->req_active_time.time_val < SETPSM_TIMER_VAL_MIN ||
        SETPSM_TIMER_VAL_MAX < settings->req_active_time.time_val)
        {
          DBGIF_LOG1_ERROR("Invalid argument. RAT time_val:%d\n", settings->req_active_time.time_val);
          return -EINVAL;
        }

      if (settings->ext_periodic_tau_time.unit < SETPSM_TAU_UNIT_MIN ||
        SETPSM_TAU_UNIT_MAX < settings->ext_periodic_tau_time.unit)
        {
          DBGIF_LOG1_ERROR("Invalid argument. TAU unit:%d\n", settings->ext_periodic_tau_time.unit);
          return -EINVAL;
        }

      if (settings->ext_periodic_tau_time.time_val < SETPSM_TIMER_VAL_MIN ||
        SETPSM_TIMER_VAL_MAX < settings->ext_periodic_tau_time.time_val)
        {
          DBGIF_LOG1_ERROR("Invalid argument. TAU time_val:%d\n", settings->ext_periodic_tau_time.time_val);
          return -EINVAL;
        }
    }

  /* Register API callback */

  ALTCOM_REG_CALLBACK(ret, g_setpsm_callback, callback);
  if (ret < 0)
    {
      DBGIF_LOG_ERROR("Currently API is busy.\n");
      return ret;
    }

  /* Allocate API command buffer to send */

  cmddat = (struct apicmd_cmddat_setpsm_s *)apicmdgw_cmd_allocbuff(
    APICMDID_SET_PSM, SETPSM_DATA_LEN);
  if (!cmddat)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
    }
  else
    {
      cmddat->enable = settings->enable ?
        APICMD_SETPSM_ENABLE : APICMD_SETPSM_DISABLE;
      cmddat->rat_val.unit     = settings->req_active_time.unit;
      cmddat->rat_val.time_val = settings->req_active_time.time_val;
      cmddat->tau_val.unit     = settings->ext_periodic_tau_time.unit;
      cmddat->tau_val.time_val =
        settings->ext_periodic_tau_time.time_val;

      /* Send API command to modem */

      ret = altcom_send_and_free((FAR uint8_t *)cmddat);
    }

  /* If fail, there is no opportunity to execute the callback,
   * so clear it here. */

  if (ret < 0)
    {
      /* Clear registered callback */

      ALTCOM_CLR_CALLBACK(g_setpsm_callback);
    }
  else
    {
      ret = 0;
    }

  return ret;
}
