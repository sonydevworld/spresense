/****************************************************************************
 * modules/lte/altcom/api/lte/lte_getedrx.c
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
#include "buffpoolwrapper.h"
#include "dbg_if.h"
#include "osal.h"
#include "apiutil.h"
#include "apicmdgw.h"
#include "apicmd_getedrx.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETEDRX_DATA_LEN (0)
#define APICMDHDLR_GETEDRX_CYC_MIN  APICMD_GETEDRX_CYC_512
#define APICMDHDLR_GETEDRX_CYC_MAX  APICMD_GETEDRX_CYC_262144
#define APICMDHDLR_GETEDRX_PTW_MIN  APICMD_GETEDRX_PTW_128
#define APICMDHDLR_GETEDRX_PTW_MAX  APICMD_GETEDRX_PTW_2048

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern get_edrx_cb_t g_getedrx_callback;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getedrx_job
 *
 * Description:
 *   This function is an API callback for get eDRX.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getedrx_job(FAR void *arg)
{
  int32_t                               ret;
  uint32_t                              result = LTE_RESULT_OK;
  FAR struct apicmd_cmddat_getedrxres_s *data;
  lte_edrx_setting_t                    edrx;
  get_edrx_cb_t                         callback;

  data = (FAR struct apicmd_cmddat_getedrxres_s *)arg;
  ALTCOM_GET_AND_CLR_CALLBACK(ret, g_getedrx_callback, callback);

  if ((ret == 0) && (callback))
    {
      if (APICMD_GETEDRX_RES_OK == data->result)
        {
          if (APICMD_GETEDRX_DISABLE == data->enable)
            {
              edrx.enable = LTE_DISABLE;
            }
          else if (APICMD_GETEDRX_ENABLE == data->enable)
            {
              edrx.enable = LTE_ENABLE;
            }
          else
            {
              DBGIF_LOG1_ERROR("Invalid parameter. enable:%d\n", data->enable);
              result = LTE_RESULT_ERROR;
            }

          if (LTE_RESULT_OK == result)
            {
              if (APICMDHDLR_GETEDRX_CYC_MIN <= data->edrx_cycle &&
                data->edrx_cycle <= APICMDHDLR_GETEDRX_CYC_MAX)
                {
                  edrx.edrx_cycle = (uint32_t)data->edrx_cycle;
                }
              else
                {
                  DBGIF_LOG1_ERROR("Invalid parameter. edrx_cycle:%d\n", data->edrx_cycle);
                  result = LTE_RESULT_ERROR;
                }
            }

          if (LTE_RESULT_OK == result)
            {
              if (APICMDHDLR_GETEDRX_PTW_MIN <= data->ptw_val &&
                data->ptw_val <= APICMDHDLR_GETEDRX_PTW_MAX)
                {
                  edrx.ptw_val = (uint32_t)data->ptw_val;
                }
              else
                {
                  DBGIF_LOG1_ERROR("Invalid parameter. ptw_val:%d\n", data->ptw_val);
                  result = LTE_RESULT_ERROR;
                }
            }

          callback(result, &edrx);
          DBGIF_ASSERT(LTE_RESULT_OK == result, "Result parameter error.\n");
        }
      else
        {
          callback(LTE_RESULT_ERROR, NULL);
          DBGIF_ASSERT(APICMD_GETEDRX_RES_ERR == data->result, "Result parameter error.\n");
        }
    }
  else
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
    }

  /* In order to reduce the number of copies of the receive buffer,
   * bring a pointer to the receive buffer to the worker thread.
   * Therefore, the receive buffer needs to be released here. */

  altcom_free_cmd((FAR uint8_t *)arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_get_edrx
 *
 * Description:
 *   Get eDRX settings.
 *
 * Input Parameters:
 *   callback Callback function to notify that get eDRX settings is
 *            completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_get_edrx(get_edrx_cb_t callback)
{
  int32_t     ret;
  FAR uint8_t *cmdbuff;

  /* Return error if callback is NULL */

  if (!callback)
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
  else
    {
      /* Register API callback */

      ALTCOM_REG_CALLBACK(ret, g_getedrx_callback, callback);
      if (ret < 0)
        {
          DBGIF_LOG_ERROR("Currently API is busy.\n");
        }
    }

  /* Accept the API */

  if (ret == 0)
    {
      /* Allocate API command buffer to send */

      cmdbuff = (FAR uint8_t *)apicmdgw_cmd_allocbuff(APICMDID_GET_EDRX,
        GETEDRX_DATA_LEN);
      if (!cmdbuff)
        {
          DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
          ret = -ENOMEM;
        }
      else
        {
          /* Send API command to modem */

          ret = altcom_send_and_free(cmdbuff);
        }

      /* If fail, there is no opportunity to execute the callback,
       * so clear it here. */

      if (ret < 0)
        {
          /* Clear registered callback */

          ALTCOM_CLR_CALLBACK(g_getedrx_callback);
        }
      else
        {
          ret = 0;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: apicmdhdlr_getedrx
 *
 * Description:
 *   This function is an API command handler for get eDRX result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_GET_EDRX_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_getedrx(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt, APICMDID_CONVERT_RES(APICMDID_GET_EDRX),
    getedrx_job);
}
