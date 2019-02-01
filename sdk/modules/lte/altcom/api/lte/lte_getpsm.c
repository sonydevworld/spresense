/****************************************************************************
 * modules/lte/altcom/api/lte/lte_getpsm.c
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
#include "apicmd_getpsm.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETPSM_DATA_LEN (0)
#define APICMDHDLR_GETPSM_RAT_UNIT_MIN   APICMD_GETPSM_RAT_UNIT_2SEC
#define APICMDHDLR_GETPSM_RAT_UNIT_MAX   APICMD_GETPSM_RAT_UNIT_6MIN
#define APICMDHDLR_GETPSM_TAU_UNIT_MIN   APICMD_GETPSM_TAU_UNIT_2SEC
#define APICMDHDLR_GETPSM_TAU_UNIT_MAX   APICMD_GETPSM_TAU_UNIT_320HOUR

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getpsm_status_chg_cb
 *
 * Description:
 *   Notification status change in processing get PSM.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getpsm_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("getpsm_status_chg_cb(%d -> %d)\n",
        new_stat, old_stat);
      altcomcallbacks_unreg_cb(APICMDID_GET_PSM);
    }
}

/****************************************************************************
 * Name: getpsm_job
 *
 * Description:
 *   This function is an API callback for get PSM.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getpsm_job(FAR void *arg)
{
  int32_t                              ret;
  uint32_t                             result = LTE_RESULT_OK;
  FAR struct apicmd_cmddat_getpsmres_s *data;
  lte_psm_setting_t                    psm;
  get_psm_cb_t                         callback;

  data = (FAR struct apicmd_cmddat_getpsmres_s *)arg;

  ret = altcomcallbacks_get_unreg_cb(APICMDID_GET_PSM,
    (void **)&callback);

  if ((ret == 0) && (callback))
    {
      if (APICMD_GETPSM_RES_OK == data->result)
        {
          if (APICMD_GETPSM_DISABLE == data->enable)
            {
              psm.enable = LTE_DISABLE;
            }
          else if (APICMD_GETPSM_ENABLE == data->enable)
            {
              psm.enable = LTE_ENABLE;

              if (APICMDHDLR_GETPSM_RAT_UNIT_MIN <= data->rat_val.unit &&
                data->rat_val.unit <= APICMDHDLR_GETPSM_RAT_UNIT_MAX)
                {
                  psm.req_active_time.unit = (uint32_t)data->rat_val.unit;
                }
              else
                {
                  DBGIF_LOG1_ERROR("Invalid parameter. RAT unit:%d\n", data->rat_val.unit);
                  result = LTE_RESULT_ERROR;
                }

              if (LTE_RESULT_OK == result)
                {
                  if (APICMD_GETPSM_TIMER_MIN <= data->rat_val.time_val &&
                    data->rat_val.time_val <= APICMD_GETPSM_TIMER_MAX)
                    {
                      psm.req_active_time.time_val = (uint32_t)data->rat_val.time_val;
                    }
                  else
                    {
                      DBGIF_LOG1_ERROR("Invalid parameter. RAT time_val:%d\n", data->rat_val.time_val);
                      result = LTE_RESULT_ERROR;
                    }
                }

              if (LTE_RESULT_OK == result)
                {
                  if (APICMDHDLR_GETPSM_TAU_UNIT_MIN <= data->tau_val.unit &&
                    data->tau_val.unit <= APICMDHDLR_GETPSM_TAU_UNIT_MAX)
                    {
                      psm.ext_periodic_tau_time.unit = (uint32_t)data->tau_val.unit;
                    }
                  else
                    {
                      DBGIF_LOG1_ERROR("Invalid parameter. TAU unit:%d\n", data->tau_val.unit);
                      result = LTE_RESULT_ERROR;
                    }
                }

              if (LTE_RESULT_OK == result)
                {
                  if (APICMD_GETPSM_TIMER_MIN <= data->tau_val.time_val &&
                    data->tau_val.time_val <= APICMD_GETPSM_TIMER_MAX)
                    {
                      psm.ext_periodic_tau_time.time_val = (uint32_t)data->tau_val.time_val;
                    }
                  else
                    {
                      DBGIF_LOG1_ERROR("Invalid parameter. TAU time_val:%d\n", data->tau_val.time_val);
                      result = LTE_RESULT_ERROR;
                    }
                }
            }
          else
            {
              DBGIF_LOG1_ERROR("Invalid parameter. enable:%d\n", data->enable);
              result = LTE_RESULT_ERROR;
            }

          callback(result, &psm);
          DBGIF_ASSERT(LTE_RESULT_OK == result, "Result parameter error.\n");
        }
      else
        {
          callback(LTE_RESULT_ERROR, NULL);
          DBGIF_ASSERT(APICMD_GETPSM_RES_ERR == data->result, "Result parameter error.\n");
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

  /* Unregistration status change callback. */

  altcomstatus_unreg_statchgcb((void *)getpsm_status_chg_cb);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_get_psm
 *
 * Description:
 *   Get PSM settings.
 *
 * Input Parameters:
 *   callback Callback function to notify that get PSM settings is
 *            completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_get_psm(get_psm_cb_t callback)
{
  int32_t     ret;
  FAR uint8_t *cmdbuff;

  /* Return error if callback is NULL */

  if (!callback)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
    }

  /* Check Lte library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  /* Register API callback */

  ret = altcomcallbacks_chk_reg_cb((void *)callback, APICMDID_GET_PSM);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Currently API is busy.\n");
      return -EINPROGRESS;
    }

  ret = altcomstatus_reg_statchgcb((void *)getpsm_status_chg_cb);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
      altcomcallbacks_unreg_cb(APICMDID_GET_PSM);
      return ret;
    }

  /* Allocate API command buffer to send */

  cmdbuff = (FAR uint8_t *)apicmdgw_cmd_allocbuff(APICMDID_GET_PSM,
    GETPSM_DATA_LEN);
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

      altcomcallbacks_unreg_cb(APICMDID_GET_PSM);
      altcomstatus_unreg_statchgcb((void *)getpsm_status_chg_cb);
    }
  else
    {
      ret = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: apicmdhdlr_getpsm
 *
 * Description:
 *   This function is an API command handler for get PSM result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_GET_PSM_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_getpsm(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt, APICMDID_CONVERT_RES(APICMDID_GET_PSM),
    getpsm_job);
}
