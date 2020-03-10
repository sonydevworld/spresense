/****************************************************************************
 * modules/lte/altcom/api/lte/lte_phoneno.c
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
#include <string.h>
#include <errno.h>

#include "lte/lte_api.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "apicmd_phoneno.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REQ_DATA_LEN (0)
#define RES_DATA_LEN (sizeof(struct apicmd_cmddat_phonenores_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getphoneno_status_chg_cb
 *
 * Description:
 *   Notification status change in processing get phone number.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static int32_t getphoneno_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("getphoneno_status_chg_cb(%d -> %d)\n",
        old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_GET_PHONENO);

      return ALTCOM_STATUS_REG_CLR;
    }

  return ALTCOM_STATUS_REG_KEEP;
}

/****************************************************************************
 * Name: getphoneno_job
 *
 * Description:
 *   This function is an API callback for get phone number.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getphoneno_job(FAR void *arg)
{
  int32_t                               ret;
  int32_t                               result;
  FAR struct apicmd_cmddat_phonenores_s *data;
  get_phoneno_cb_t                      callback;

  data = (FAR struct apicmd_cmddat_phonenores_s *)arg;

  ret = altcomcallbacks_get_unreg_cb(APICMDID_GET_PHONENO,
    (void **)&callback);

  if ((ret == 0) && (callback))
    {
      result = (int32_t)data->result;

      /* Fixed to include "\0" at the end of output string. */

      data->phoneno[LTE_PHONENO_LEN - 1] = '\0';

      callback(result, data->errcause, (FAR int8_t*)data->phoneno);
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

  altcomstatus_unreg_statchgcb(getphoneno_status_chg_cb);
}

/****************************************************************************
 * Name: lte_getphoneno_impl
 *
 * Description:
 *   Get phone number from SIM.
 *
 * Input Parameters:
 *   phoneno   A character string indicating phone number. It is terminated
 *             with '\0'. When using the synchronous API, the maximum number
 *             of phone number areas must be allocated.
 *   callback  Callback function to notify when getting the phone number is
 *             completed.
 *             If the callback is NULL, operates with synchronous API,
 *             otherwise operates with asynchronous API.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t lte_getphoneno_impl(int8_t *phoneno, get_phoneno_cb_t callback)
{
  int32_t                                ret;
  FAR uint8_t                           *reqbuff    = NULL;
  FAR struct apicmd_cmddat_phonenores_s *presbuff   = NULL;
  uint16_t                               resbufflen = RES_DATA_LEN;
  uint16_t                               reslen     = 0;
  int                                    sync       = (callback == NULL);

  /* Check input parameter */

  if (!phoneno && !callback)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
    }

  /* Check LTE library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  if (sync)
    {
      /* Allocate API command buffer to receive */

      presbuff = (FAR struct apicmd_cmddat_phonenores_s *)
                   altcom_alloc_resbuff(resbufflen);
      if (!presbuff)
        {
          DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
          return -ENOMEM;
        }
    }
  else
    {
      /* Setup API callback */

      ret = altcombs_setup_apicallback(APICMDID_GET_PHONENO, callback,
                                       getphoneno_status_chg_cb);
      if (0 > ret)
        {
          return ret;
        }
    }

  /* Allocate API command buffer to send */

  reqbuff = (FAR uint8_t *)apicmdgw_cmd_allocbuff(APICMDID_GET_PHONENO,
                                                  REQ_DATA_LEN);
  if (!reqbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Send API command to modem */

  ret = apicmdgw_send(reqbuff, (FAR uint8_t *)presbuff,
                      resbufflen, &reslen, SYS_TIMEO_FEVR);
  altcom_free_cmd(reqbuff);

  if (0 > ret)
    {
      goto errout;
    }

  ret = 0;

  if (sync)
    {
      ret = (LTE_RESULT_OK == presbuff->result) ? 0 : -EPROTO;
      if (0 == ret)
        {
          strncpy((FAR char *)phoneno, (FAR const char *)presbuff->phoneno,
                  LTE_PHONENO_LEN);
          phoneno[LTE_PHONENO_LEN - 1] = '\0';
        }
      BUFFPOOL_FREE(presbuff);
    }

  return ret;

errout:
  if (!sync)
    {
      altcombs_teardown_apicallback(APICMDID_GET_PHONENO,
                                    getphoneno_status_chg_cb);
    }
  if (presbuff)
    {
      BUFFPOOL_FREE(presbuff);
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_get_phoneno_sync
 *
 * Description:
 *   Get phone number from SIM.
 *
 * Input Parameters:
 *   phoneno   A character string indicating phone number. It is terminated
 *             with '\0'. When using the synchronous API, the maximum number
 *             of phone number areas must be allocated.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_get_phoneno_sync(int8_t *phoneno)
{
  return lte_getphoneno_impl(phoneno, NULL);
}

/****************************************************************************
 * Name: lte_get_phoneno
 *
 * Description:
 *   Get phone number from SIM.
 *
 * Input Parameters:
 *   callback  Callback function to notify when getting the phone number is
 *             completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_get_phoneno(get_phoneno_cb_t callback)
{
  if (!callback) {
    DBGIF_LOG_ERROR("Input argument is NULL.\n");
    return -EINVAL;
  }
  return lte_getphoneno_impl(NULL, callback);
}

/****************************************************************************
 * Name: apicmdhdlr_phoneno
 *
 * Description:
 *   This function is an API command handler for phone number get result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_GET_PHONENO_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_phoneno(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_CONVERT_RES(APICMDID_GET_PHONENO), getphoneno_job);
}
