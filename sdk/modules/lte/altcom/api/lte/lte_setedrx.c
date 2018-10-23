/****************************************************************************
 * modules/lte/altcom/api/lte/lte_setedrx.c
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
#include "apicmd_setedrx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SETEDRX_DATA_LEN (sizeof(struct apicmd_cmddat_setedrx_s))
#define SETEDRX_CYC_MIN  LTE_EDRX_CYC_512
#define SETEDRX_CYC_MAX  LTE_EDRX_CYC_262144
#define SETEDRX_PTW_MIN  LTE_EDRX_PTW_128
#define SETEDRX_PTW_MAX  LTE_EDRX_PTW_2048

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern set_edrx_cb_t g_setedrx_callback;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_set_edrx
 *
 * Description:
 *   Get eDRX settings.
 *
 * Input Parameters:
 *   callback Callback function to notify that set eDRX settings is
 *            completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_edrx(lte_edrx_setting_t *settings, set_edrx_cb_t callback)
{
  int32_t                        ret;
  struct apicmd_cmddat_setedrx_s *cmddat;

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
      if (settings->edrx_cycle < SETEDRX_CYC_MIN ||
        SETEDRX_CYC_MAX < settings->edrx_cycle)
        {
          DBGIF_LOG1_ERROR("Invalid argument. edrx_cycle:%d\n", settings->edrx_cycle);
          return -EINVAL;
        }

      if (settings->ptw_val < SETEDRX_PTW_MIN || 
        SETEDRX_PTW_MAX < settings->ptw_val)
        {
          DBGIF_LOG1_ERROR("Invalid argument. ptw_val:%d\n", settings->ptw_val);
          return -EINVAL;
        }
    }

  /* Register API callback */

  ALTCOM_REG_CALLBACK(ret, g_setedrx_callback, callback);
  if (ret < 0)
    {
      DBGIF_LOG_ERROR("Currently API is busy.\n");
      return ret;
    }

  /* Allocate API command buffer to send */

  cmddat = (struct apicmd_cmddat_setedrx_s *)apicmdgw_cmd_allocbuff(
    APICMDID_SET_EDRX, SETEDRX_DATA_LEN);
  if (!cmddat)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
    }
  else
    {
      cmddat->acttype = APICMD_SETEDRX_ACTTYPE_WBS1;
      cmddat->enable  = settings->enable ?
        APICMD_SETEDRX_ENABLE : APICMD_SETEDRX_DISABLE;
      cmddat->edrx_cycle = settings->edrx_cycle;
      cmddat->ptw_val    = settings->ptw_val;

      /* Send API command to modem */

      ret = altcom_send_and_free((FAR uint8_t *)cmddat);
    }

  /* If fail, there is no opportunity to execute the callback,
   * so clear it here. */

  if (ret < 0)
    {
      /* Clear registered callback */

      ALTCOM_CLR_CALLBACK(g_setedrx_callback);
    }
  else
    {
      ret = 0;
    }

  return ret;
}

