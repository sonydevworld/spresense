/****************************************************************************
 * modules/lte/altcom/api/lte/lte_activate_pdn.c
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "lte/lte_api.h"
#include "buffpoolwrapper.h"
#include "evthdlbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"
#include "apicmdhdlrbs.h"
#include "apiutil.h"
#include "apicmd_dataallow.h"
#include "lte_dataallow.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REQ_DATA_LEN (sizeof(struct apicmd_cmddat_dataallow_s))
#define RES_DATA_LEN (sizeof(struct apicmd_cmddat_dataallowres_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dataallow_status_chg_cb
 *
 * Description:
 *   Notification status change in processing data allow.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static int32_t dataallow_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("dataallow_status_chg_cb(%d -> %d)\n",
        old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_DATA_ALLOW);

      return ALTCOM_STATUS_REG_CLR;
    }

  return ALTCOM_STATUS_REG_KEEP;
}

/****************************************************************************
 * Name: dataallow_check_param
 *
 * Description:
 *   Check data allow input parameter.
 *
 * Input Parameters:
 *   session_id      Target connection id.
 *   allow           General data communication allow.
 *   roamallow       Roaming data communication allow.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t dataallow_check_param(uint8_t session_id,
  uint8_t allow, uint8_t roamallow)
{
  if (LTE_PDN_SESSIONID_MIN > session_id ||
     LTE_PDN_SESSIONID_MAX < session_id)
    {
      return -EINVAL;
    }

  if (LTE_DATA_ALLOW != allow &&
     LTE_DATA_DISALLOW != allow)
    {
      return -EINVAL;
    }

  if (LTE_DATA_ALLOW != roamallow &&
     LTE_DATA_DISALLOW != roamallow)
    {
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: dataallow_job
 *
 * Description:
 *   This function is an API callback for data allow.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void dataallow_job(FAR void *arg)
{
  int32_t                                 ret;
  FAR struct apicmd_cmddat_dataallowres_s *data;
  data_allow_cb_t                         callback;
  uint32_t                                result;

  data = (FAR struct apicmd_cmddat_dataallowres_s *)arg;
  ret = altcomcallbacks_get_unreg_cb(APICMDID_DATA_ALLOW,
    (void **)&callback);

  if ((0 != ret) || (!callback))
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
    }
  else
    {
      result =
        data->result == LTE_RESULT_OK ? LTE_RESULT_OK :
        LTE_RESULT_ERROR;

      callback(result);
    }

  /* In order to reduce the number of copies of the receive buffer,
   * bring a pointer to the receive buffer to the worker thread.
   * Therefore, the receive buffer needs to be released here. */

  altcom_free_cmd((FAR uint8_t *)arg);

  /* Unregistration status change callback. */

  altcomstatus_unreg_statchgcb(dataallow_status_chg_cb);
}

/****************************************************************************
 * Name: lte_dataallow_impl
 *
 * Description:
 *   Allow or disallow to data communication for specified PDN.
 *
 * Input Parameters:
 *   session_id        The numeric value of the session ID.
 *   allow             Allow or disallow to data communication
 *                     for all network.
 *   roaming_allow     Allow or disallow to data communication
 *                     for roaming network.
 *   callback          Callback function to notify that
 *                     configuration has changed.
 *                     If the callback is NULL, operates with synchronous API,
 *                     otherwise operates with asynchronous API.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

static int32_t lte_dataallow_impl(uint8_t session_id, uint8_t allow,
                                  uint8_t roaming_allow,
                                  data_allow_cb_t callback)
{
  int32_t                                  ret;
  FAR struct apicmd_cmddat_dataallow_s    *reqbuff    = NULL;
  FAR uint8_t                             *presbuff   = NULL;
  struct apicmd_cmddat_dataallowres_s      resbuff;
  uint16_t                                 resbufflen = RES_DATA_LEN;
  uint16_t                                 reslen     = 0;
  int                                      sync       = (callback == NULL);

  /* Check input parameter */

  ret = dataallow_check_param(session_id, allow, roaming_allow);
  if (0 > ret)
    {
      return ret;
    }

  /* Check LTE library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  if (sync)
    {
      presbuff = (FAR uint8_t *)&resbuff;
    }
  else
    {
      /* Setup API callback */

      ret = altcombs_setup_apicallback(APICMDID_DATA_ALLOW, callback,
                                       dataallow_status_chg_cb);
      if (0 > ret)
        {
          return ret;
        }
    }

  /* Allocate API command buffer to send */

  reqbuff = (FAR struct apicmd_cmddat_dataallow_s *)
              apicmdgw_cmd_allocbuff(APICMDID_DATA_ALLOW, REQ_DATA_LEN);
  if (!reqbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
      goto errout;
    }

  reqbuff->session_id = session_id;
  reqbuff->data_allow = allow == LTE_DATA_ALLOW ?
    APICMD_DATAALLOW_DATAALLOW_ALLOW :
    APICMD_DATAALLOW_DATAALLOW_DISALLOW;
  reqbuff->dataroam_allow = roaming_allow == LTE_DATA_ALLOW ?
    APICMD_DATAALLOW_DATAROAMALLOW_ALLOW :
    APICMD_DATAALLOW_DATAROAMALLOW_DISALLOW;

  /* Send API command to modem */

  ret = apicmdgw_send((FAR uint8_t *)reqbuff, presbuff,
                      resbufflen, &reslen, SYS_TIMEO_FEVR);
  altcom_free_cmd((FAR uint8_t *)reqbuff);

  if (0 > ret)
    {
      goto errout;
    }

  ret = 0;

  if (sync)
    {
      ret = (LTE_RESULT_OK == resbuff.result) ? 0 : -EPROTO;
    }

  return ret;

errout:
  if (!sync)
    {
      altcombs_teardown_apicallback(APICMDID_DATA_ALLOW,
                                    dataallow_status_chg_cb);
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_data_allow_sync
 *
 * Description:
 *   Allow or disallow to data communication for specified PDN.
 *
 * Input Parameters:
 *   session_id        The numeric value of the session ID.
 *   allow             Allow or disallow to data communication
 *                     for all network.
 *   roaming_allow     Allow or disallow to data communication
 *                     for roaming network.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_data_allow_sync(uint8_t session_id, uint8_t allow,
                            uint8_t roaming_allow)
{
  return lte_dataallow_impl(session_id, allow, roaming_allow, NULL);
}

/****************************************************************************
 * Name: lte_data_allow
 *
 * Description:
 *   Allow or disallow to data communication for specified PDN.
 *
 * Input Parameters:
 *   session_id        The numeric value of the session ID.
 *   allow             Allow or disallow to data communication
 *                     for all network.
 *   roaming_allow     Allow or disallow to data communication
 *                     for roaming network.
 *   callback          Callback function to notify that
 *                     configuration has changed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_data_allow(uint8_t session_id, uint8_t allow,
                       uint8_t roaming_allow, data_allow_cb_t callback)
{
  if (!callback) {
    DBGIF_LOG_ERROR("Input argument is NULL.\n");
    return -EINVAL;
  }
  return lte_dataallow_impl(session_id, allow, roaming_allow, callback);
}

/****************************************************************************
 * Name: apicmdhdlr_dataallow
 *
 * Description:
 *   This function is an API command handler for data allow result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_DATA_ALLOW_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_dataallow(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_CONVERT_RES(APICMDID_DATA_ALLOW), dataallow_job);
}
