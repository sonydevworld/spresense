/****************************************************************************
 * modules/lte/altcom/api/lte/lte_getce.c
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
#include "apicmd_getce.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETCE_DATA_LEN (0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getce_status_chg_cb
 *
 * Description:
 *   Notification status change in processing get CE.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getce_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("getce_status_chg_cb(%d -> %d)\n",
        new_stat, old_stat);
      altcomcallbacks_unreg_cb(APICMDID_GET_CE);
    }
}

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

  ret = altcomcallbacks_get_unreg_cb(APICMDID_GET_CE,
    (void **)&callback);

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

  /* Unregistration status change callback. */

  altcomstatus_unreg_statchgcb((void *)getce_status_chg_cb);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_get_ce
 *
 * Description:
 *   Get CE settings.
 *
 * Input Parameters:
 *   callback Callback function to notify that get CE settings is
 *            completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_get_ce(get_ce_cb_t callback)
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

  ret = altcomcallbacks_chk_reg_cb((void *)callback, APICMDID_GET_CE);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Currently API is busy.\n");
      return -EINPROGRESS;
    }

  ret = altcomstatus_reg_statchgcb((void *)getce_status_chg_cb);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
      altcomcallbacks_unreg_cb(APICMDID_GET_CE);
      return ret;
    }

  /* Allocate API command buffer to send */

  cmdbuff = (FAR uint8_t *)apicmdgw_cmd_allocbuff(APICMDID_GET_CE,
    GETCE_DATA_LEN);
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

      altcomcallbacks_unreg_cb(APICMDID_GET_CE);
      altcomstatus_unreg_statchgcb((void *)getce_status_chg_cb);
    }
  else
    {
      ret = 0;
    }

  return ret;
}

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
