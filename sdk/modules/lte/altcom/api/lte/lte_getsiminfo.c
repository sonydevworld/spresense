/****************************************************************************
 * modules/lte/altcom/api/lte/lte_getsiminfo.c
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcombs.h"
#include "altcom_callbacks.h"
#include "apicmd_getsiminfo.h"
#include "lte_getsiminfo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REQ_DATA_LEN (sizeof(struct apicmd_cmddat_getsiminfo_s))
#define RES_DATA_LEN (sizeof(struct apicmd_cmddat_getsiminfo_res_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_getsiminfo_checkparam
 *
 * Description:
 *   Get SIM infomation check input param.
 *
 * Input Parameters:
 *   opt Getting SIM infomation select bits.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t lte_getsiminfo_checkparam(uint32_t opt)
{
  uint32_t mask = 0;

  mask = (LTE_SIMINFO_GETOPT_MCCMNC |
          LTE_SIMINFO_GETOPT_SPN |
          LTE_SIMINFO_GETOPT_ICCID |
          LTE_SIMINFO_GETOPT_IMSI |
          LTE_SIMINFO_GETOPT_GID1 |
          LTE_SIMINFO_GETOPT_GID2);

  if (0 == (opt & mask))
    {
      return -EINVAL;
    }

    return 0;
}

/****************************************************************************
 * Name: getsiminfo_status_chg_cb
 *
 * Description:
 *   Notification status change in processing get sim infomation.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static int32_t getsiminfo_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("getsiminfo_status_chg_cb(%d -> %d)\n", old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_GET_SIMINFO);

      return ALTCOM_STATUS_REG_CLR;
    }

  return ALTCOM_STATUS_REG_KEEP;
}

/****************************************************************************
 * Name: getsiminfo_parse_response
 *
 * Description:
 *   Parse SIM information from response buffer.
 *
 * Input Parameters:
 *  resp     Pointer to response buffer.
 *  siminfo  Pointer to store SIM information.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getsiminfo_parse_response(
  FAR struct apicmd_cmddat_getsiminfo_res_s *resp,
  FAR lte_siminfo_t *siminfo)
{
  siminfo->option = ntohl(resp->option);

  if (siminfo->option & LTE_SIMINFO_GETOPT_MCCMNC)
    {
      if (LTE_MNC_DIGIT_MAX < resp->mnc_digit)
        {
          resp->mnc_digit = LTE_MNC_DIGIT_MAX;
        }
      memcpy(siminfo->mcc, resp->mcc, LTE_MCC_DIGIT);
      siminfo->mnc_digit = resp->mnc_digit;
      memcpy(siminfo->mnc, resp->mnc, resp->mnc_digit);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_SPN)
    {
      if (LTE_SIMINFO_SPN_LEN < resp->spn_len)
        {
          resp->spn_len = LTE_SIMINFO_SPN_LEN;
        }
      siminfo->spn_len = resp->spn_len;
      memcpy(siminfo->spn, resp->spn, resp->spn_len);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_ICCID)
    {
      if (LTE_SIMINFO_ICCID_LEN < resp->iccid_len)
        {
          resp->imsi_len = LTE_SIMINFO_ICCID_LEN;
        }
      siminfo->iccid_len = resp->iccid_len;
      memcpy(siminfo->iccid, resp->iccid, resp->iccid_len);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_IMSI)
    {
      if (LTE_SIMINFO_IMSI_LEN < resp->imsi_len)
        {
          resp->imsi_len = LTE_SIMINFO_IMSI_LEN;
        }
      siminfo->imsi_len = resp->imsi_len;
      memcpy(siminfo->imsi, resp->imsi, resp->imsi_len);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_GID1)
    {
      if (LTE_SIMINFO_GID_LEN < resp->gid1_len)
        {
          resp->gid1_len = LTE_SIMINFO_GID_LEN;
        }
      siminfo->gid1_len = resp->gid1_len;
      memcpy(siminfo->gid1, resp->gid1, resp->gid1_len);
    }

  if (siminfo->option & LTE_SIMINFO_GETOPT_GID2)
    {
      if (LTE_SIMINFO_GID_LEN < resp->gid2_len)
        {
          resp->gid2_len = LTE_SIMINFO_GID_LEN;
        }
      siminfo->gid2_len = resp->gid2_len;
      memcpy(siminfo->gid2, resp->gid2, resp->gid2_len);
    }
}

/****************************************************************************
 * Name: get_siminfo_job
 *
 * Description:
 *   This function is an API callback for get SIM infomation.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void get_siminfo_job(FAR void *arg)
{
  int32_t                                   ret;
  FAR struct apicmd_cmddat_getsiminfo_res_s *data;
  get_siminfo_cb_t                          callback;
  int32_t                                   result = LTE_RESULT_ERROR;
  lte_siminfo_t                             siminfo;

  data = (FAR struct apicmd_cmddat_getsiminfo_res_s *)arg;
  ret = altcomcallbacks_get_unreg_cb(APICMDID_GET_SIMINFO,
    (void **)&callback);
  if ((0 != ret) || (!callback))
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
    }
  else
    {
      memset(&siminfo, 0, sizeof(lte_siminfo_t));
      if (LTE_RESULT_OK == data->result)
        {
          getsiminfo_parse_response(data, &siminfo);
          result = LTE_RESULT_OK;
        }

      callback(result, &siminfo);
    }

  /* In order to reduce the number of copies of the receive buffer,
   * bring a pointer to the receive buffer to the worker thread.
   * Therefore, the receive buffer needs to be released here. */

  altcom_free_cmd((FAR uint8_t *)arg);

  /* Unregistration status change callback. */

  altcomstatus_unreg_statchgcb(getsiminfo_status_chg_cb);
}

/****************************************************************************
 * Name: lte_getsiminfo_impl
 *
 * Description:
 *   Get SIM information such as Mobile Country Code/Mobile Network Code.
 *
 * Input Parameters:
 *   option   Indicates which parameter to get.
 *   siminfo  SIM information.
 *   callback Callback function to notify that get of SIM information is
 *            completed.
 *            If the callback is NULL, operates with synchronous API,
 *            otherwise operates with asynchronous API.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

static int32_t lte_getsiminfo_impl(uint32_t option, lte_siminfo_t *siminfo,
                                   get_siminfo_cb_t callback)
{
  int32_t                                    ret;
  FAR struct apicmd_cmddat_getsiminfo_s     *reqbuff    = NULL;
  FAR struct apicmd_cmddat_getsiminfo_res_s *presbuff   = NULL;
  uint16_t                                   resbufflen = RES_DATA_LEN;
  uint16_t                                   reslen     = 0;
  int                                        sync       = (callback == NULL);

  /* Check input parameter */

  if (!siminfo && !callback)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
    }

  ret = lte_getsiminfo_checkparam(option);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("Input option is invalid. option:%d\n", option);
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
      /* Allocate API command buffer to receive */

      presbuff = (FAR struct apicmd_cmddat_getsiminfo_res_s *)
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

      ret = altcombs_setup_apicallback(APICMDID_GET_SIMINFO, callback,
                                       getsiminfo_status_chg_cb);
      if (0 > ret)
        {
          return ret;
        }
    }

  /* Allocate API command buffer to send */

  reqbuff = (FAR struct apicmd_cmddat_getsiminfo_s *)
              apicmdgw_cmd_allocbuff(APICMDID_GET_SIMINFO, REQ_DATA_LEN);
  if (!reqbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
      goto errout;
    }

  reqbuff->option = htonl(option);

  /* Send API command to modem */

  ret = apicmdgw_send((FAR uint8_t *)reqbuff, (FAR uint8_t *)presbuff,
                      resbufflen, &reslen, SYS_TIMEO_FEVR);
  altcom_free_cmd((FAR uint8_t *)reqbuff);

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
          /* Parse SIM information */

          getsiminfo_parse_response(presbuff, siminfo);
        }
      BUFFPOOL_FREE(presbuff);
    }

  return ret;

errout:
  if (!sync)
    {
      altcombs_teardown_apicallback(APICMDID_GET_SIMINFO,
                                    getsiminfo_status_chg_cb);
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
 * Name: lte_get_siminfo_sync
 *
 * Description:
 *   Get SIM information such as Mobile Country Code/Mobile Network Code.
 *
 * Input Parameters:
 *   option   Indicates which parameter to get.
 *   siminfo  SIM information.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_get_siminfo_sync(uint32_t option, lte_siminfo_t *siminfo)
{
  return lte_getsiminfo_impl(option, siminfo, NULL);
}

/****************************************************************************
 * Name: lte_get_siminfo
 *
 * Description:
 *   Get SIM information such as Mobile Country Code/Mobile Network Code.
 *
 * Input Parameters:
 *   option   Indicates which parameter to get.
 *   callback Callback function to notify that get of SIM information is
 *            completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_get_siminfo(uint32_t option, get_siminfo_cb_t callback)
{
  if (!callback) {
    DBGIF_LOG_ERROR("Input argument is NULL.\n");
    return -EINVAL;
  }
  return lte_getsiminfo_impl(option, NULL, callback);
}

/****************************************************************************
 * Name: apicmdhdlr_getsiminfo
 *
 * Description:
 *   This function is an API command handler for get SIM infomation result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_GET_SIMINFO_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_getsiminfo(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_CONVERT_RES(APICMDID_GET_SIMINFO), get_siminfo_job);
}
