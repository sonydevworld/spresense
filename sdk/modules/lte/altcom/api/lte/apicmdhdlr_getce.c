/****************************************************************************
 * modules/lte/altcom/api/lte/apicmdhdlr_getce.c
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
#include "apicmd_getce.h"
#include "evthdlbs.h"
#include "apicmdgw.h"
#include "apicmdhdlrbs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern get_ce_cb_t g_getce_callback;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getce_job
 *
 * Description:
 *   This function is an API callback for get CE.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getce_job(FAR void *arg)
{
  int32_t                             ret;
  uint32_t                            result = LTE_RESULT_OK;
  FAR struct apicmd_cmddat_getceres_s *data;
  lte_ce_setting_t                    ce;
  get_ce_cb_t                         callback;

  data = (FAR struct apicmd_cmddat_getceres_s *)arg;
  ALTCOM_GET_AND_CLR_CALLBACK(ret, g_getce_callback, callback);

  if ((ret == 0) && (callback))
    {
      if (APICMD_GETCE_RES_OK == data->result)
        {
          if (LTE_RESULT_OK == result)
            {
              if (APICMD_GETCE_DISABLE == data->mode_a_enable)
                {
                  ce.mode_a_enable = LTE_DISABLE;
                }
              else if (APICMD_GETCE_ENABLE == data->mode_a_enable)
                {
                  ce.mode_a_enable = LTE_ENABLE;
                }
              else
                {
                  DBGIF_LOG1_ERROR("Invalid parameter. mode_a_enable:%d\n", data->mode_a_enable);
                  result = LTE_RESULT_ERROR;
                }
            }

          if (LTE_RESULT_OK == result)
            {
              if (APICMD_GETCE_DISABLE == data->mode_b_enable)
                {
                  ce.mode_b_enable = LTE_DISABLE;
                }
              else if (APICMD_GETCE_ENABLE == data->mode_b_enable)
                {
                  ce.mode_b_enable = LTE_ENABLE;
                }
              else
                {
                  DBGIF_LOG1_ERROR("Invalid parameter. mode_b_enable:%d\n", data->mode_b_enable);
                  result = LTE_RESULT_ERROR;
                }
            }

          callback(result, &ce);
          DBGIF_ASSERT(LTE_RESULT_OK == result, "Result parameter error.\n");
        }
      else
        {
          callback(LTE_RESULT_ERROR, NULL);
          DBGIF_ASSERT(APICMD_GETCE_RES_ERR == data->result, "Result parameter error.\n");
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
 * Name: apicmdhdlr_getce
 *
 * Description:
 *   This function is an API command handler for get CE result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_GET_CE_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_getce(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt, APICMDID_CONVERT_RES(APICMDID_GET_CE),
    getce_job);
}
