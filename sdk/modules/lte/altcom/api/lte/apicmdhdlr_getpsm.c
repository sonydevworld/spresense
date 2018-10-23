/****************************************************************************
 * modules/lte/altcom/api/lte/apicmdhdlr_getpsm.c
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

#include <string.h>

#include "lte/lte_api.h"
#include "dbg_if.h"
#include "apiutil.h"
#include "apicmd_getpsm.h"
#include "evthdlbs.h"
#include "apicmdgw.h"
#include "apicmdhdlrbs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMDHDLR_GETPSM_RAT_UNIT_MIN   APICMD_GETPSM_RAT_UNIT_2SEC
#define APICMDHDLR_GETPSM_RAT_UNIT_MAX   APICMD_GETPSM_RAT_UNIT_6MIN
#define APICMDHDLR_GETPSM_TAU_UNIT_MIN   APICMD_GETPSM_TAU_UNIT_2SEC
#define APICMDHDLR_GETPSM_TAU_UNIT_MAX   APICMD_GETPSM_TAU_UNIT_320HOUR

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern get_psm_cb_t g_getpsm_callback;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  ALTCOM_GET_AND_CLR_CALLBACK(ret, g_getpsm_callback, callback);

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
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
