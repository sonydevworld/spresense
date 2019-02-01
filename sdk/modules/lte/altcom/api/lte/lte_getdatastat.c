/****************************************************************************
 * modules/lte/altcom/api/lte/lte_getdatastat.c
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
#include "apiutil.h"
#include "apicmd_getdatastat.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETDATASTAT_DATA_LEN (0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getdatastat_status_chg_cb
 *
 * Description:
 *   Notification status change in processing get data connection stat result.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getdatastat_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("getdatastat_status_chg_cb(%d -> %d)\n",
        new_stat, old_stat);
      altcomcallbacks_unreg_cb(APICMDID_GET_DATASTAT);
    }
}

/****************************************************************************
 * Name: getdatastat_job
 *
 * Description:
 *   This function is an API callback for get data connection stat result.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getdatastat_job(FAR void *arg)
{
  int32_t                                   ret = -1;
  int32_t                                   result;
  uint8_t                                   cnt;
  FAR struct apicmd_cmddat_getdatastatres_s *data;
  get_datastat_cb_t                         callback;
  lte_getdatastat_t                         state;

  data = (FAR struct apicmd_cmddat_getdatastatres_s *)arg;

  ret = altcomcallbacks_get_unreg_cb(APICMDID_GET_DATASTAT,
    (void **)&callback);

  if ((ret == 0) && (callback))
    {
      state.statelist = NULL;
      result = (int32_t)data->result;
      if (APICMD_GETDATASTAT_RES_ERR != result)
        {
          state.listnum = data->listnum;
          state.statelist = BUFFPOOL_ALLOC(sizeof(lte_getapnset_t) * data->listnum);
          DBGIF_ASSERT(state.statelist, "BUFFPOOL_ALLOC error.");
          for (cnt = 0; cnt < data->listnum; cnt++)
            {
              state.statelist[cnt].session_id = data->datastat[cnt].session_id;
              state.statelist[cnt].state =
                data->datastat[cnt].state == APICMD_GETDATASTAT_RES_CNCT ?
                LTE_NETSTAT_CONNECT : LTE_NETSTAT_DISCONNECT;
            }
        }

      callback(result, &state);
      if(state.statelist)
        {
          BUFFPOOL_FREE((FAR void *)state.statelist);
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

  altcomstatus_unreg_statchgcb((void *)getdatastat_status_chg_cb);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_get_datastat
 *
 * Description:
 *   Get state of the data communication.
 *
 * Input Parameters:
 *   session_id The numeric value of the session ID defined in the apn
 *              setting.
 *   callback   Callback function to notify that get get data communication
 *              state completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_get_datastat(get_datastat_cb_t callback)
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

  ret = altcomcallbacks_chk_reg_cb((void *)callback, APICMDID_GET_DATASTAT);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Currently API is busy.\n");
      return -EINPROGRESS;
    }

  ret = altcomstatus_reg_statchgcb((void *)getdatastat_status_chg_cb);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
      altcomcallbacks_unreg_cb(APICMDID_GET_DATASTAT);
      return ret;
    }

  /* Allocate API command buffer to send */

  cmdbuff = apicmdgw_cmd_allocbuff(APICMDID_GET_DATASTAT, GETDATASTAT_DATA_LEN);
  if (!cmdbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
    }
  else
    {
      /* Send API command to modem */

      ret = altcom_send_and_free((FAR uint8_t *)cmdbuff);
    }

  /* If fail, there is no opportunity to execute the callback,
   * so clear it here. */

  if (0 > ret)
    {
      /* Clear registered callback */

      altcomcallbacks_unreg_cb(APICMDID_GET_DATASTAT);
      altcomstatus_unreg_statchgcb((void *)getdatastat_status_chg_cb);
    }
  else
    {
      ret = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: apicmdhdlr_getdatastat
 *
 * Description:
 *   This function is an API command handler for
 *   get data connection statresult.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_GET_DATASTAT_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_getdatastat(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_CONVERT_RES(APICMDID_GET_DATASTAT), getdatastat_job);
}
