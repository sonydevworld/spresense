 /****************************************************************************
 * modules/lte/altcom/api/lte/lte_getnetinfo.c
 *
 *   Copyright 2018, 2019 Sony Semiconductor Solutions Corporation
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

#include <stdio.h>
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
#include "apicmd_getnetinfo.h"
#include "lte_getnetinfo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REQ_DATA_LEN (0)
#define RES_DATA_LEN (sizeof(struct apicmd_cmddat_getnetinfores_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#define LTE_ERR_FORMAT_PDN_BUFFER_OVERFLOW "PDN buffer overflow:%d."

/****************************************************************************
 * Name: getnetinfo_status_chg_cb
 *
 * Description:
 *   Notification status change in processing get net info.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static int32_t getnetinfo_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("getnetinfo_status_chg_cb(%d -> %d)\n",
        old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_GET_NETINFO);

      return ALTCOM_STATUS_REG_CLR;
    }

  return ALTCOM_STATUS_REG_KEEP;
}

/****************************************************************************
 * Name: getnetinfo_parse_response
 *
 * Description:
 *   Parse network information from response buffer.
 *
 * Input Parameters:
 *  resp     Pointer to response buffer.
 *  netinfo  Pointer to store network information.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getnetinfo_parse_response(
  FAR struct apicmd_cmddat_getnetinfores_s *resp,
  FAR lte_netinfo_t *netinfo)
{
  uint8_t i;

  netinfo->nw_stat         = resp->netinfo.nw_stat;
  netinfo->nw_err.err_type = resp->netinfo.err_info.err_type;
  netinfo->nw_err.reject_cause.category =
    resp->netinfo.err_info.reject_cause.category;
  netinfo->nw_err.reject_cause.value =
    resp->netinfo.err_info.reject_cause.value;
  netinfo->pdn_num = resp->netinfo.pdn_count;

  if ((0 < resp->netinfo.pdn_count) && (netinfo->pdn_stat))
    {
      for (i = 0; i < resp->netinfo.pdn_count; i++)
        {
          altcombs_set_pdninfo(&resp->netinfo.pdn[i],
                               &netinfo->pdn_stat[i]);
        }
    }
}

/****************************************************************************
 * Name: getnetinfo_job
 *
 * Description:
 *   This function is an API callback for get netinfo.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getnetinfo_job(FAR void *arg)
{
  int32_t                                  ret;
  FAR struct apicmd_cmddat_getnetinfores_s *data;
  get_netinfo_cb_t                         callback;
  uint32_t                                 result = LTE_RESULT_ERROR;
  FAR lte_netinfo_t                        netinfo;

  ret = altcomcallbacks_get_unreg_cb(APICMDID_GET_NETINFO,
    (FAR void **)&callback);
  netinfo.pdn_stat = NULL;

  if ((0 != ret) || (!callback))
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
    }
  else
    {
      data = (FAR struct apicmd_cmddat_getnetinfores_s *)arg;

      if (data->result == LTE_RESULT_OK)
        {
          if (0 < data->netinfo.pdn_count)
            {
              netinfo.pdn_stat = (FAR lte_pdn_t *)
                BUFFPOOL_ALLOC(sizeof(lte_pdn_t) * data->netinfo.pdn_count);
              if (!netinfo.pdn_stat)
                {
                  DBGIF_LOG_ERROR("memory allocation failed.\n");
                }
            }
          getnetinfo_parse_response(data, &netinfo);
          result = LTE_RESULT_OK;
        }

      callback(result, &netinfo);
    }

  /* In order to reduce the number of copies of the receive buffer,
   * bring a pointer to the receive buffer to the worker thread.
   * Therefore, the receive buffer needs to be released here. */

  if (netinfo.pdn_stat)
    {
      (void)BUFFPOOL_FREE(netinfo.pdn_stat);
    }

  altcom_free_cmd((FAR uint8_t *)arg);

  /* Unregistration status change callback. */

  altcomstatus_unreg_statchgcb(getnetinfo_status_chg_cb);
}

/****************************************************************************
 * Name: lte_getnetinfo_impl
 *
 * Description:
 *   Get LTE network information.
 *
 * Input Parameters:
 *   pdn_num   Number of pdn_stat allocated by the user. The range is from
 *             @ref LTE_PDN_SESSIONID_MIN to @ref LTE_PDN_SESSIONID_MAX.
 *   info      The LTE network information.
 *   callback  Callback function to notify that get network information
 *             completed.
 *             If the callback is NULL, operates with synchronous API,
 *             otherwise operates with asynchronous API.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

static int32_t lte_getnetinfo_impl(uint8_t pdn_num,
                                   lte_netinfo_t *info,
                                   get_netinfo_cb_t callback)
{
  int32_t                                   ret;
  FAR uint8_t                              *reqbuff    = NULL;
  FAR struct apicmd_cmddat_getnetinfores_s *presbuff   = NULL;
  uint16_t                                  resbufflen = RES_DATA_LEN;
  uint16_t                                  reslen     = 0;
  int                                       sync       = (callback == NULL);
  lte_errinfo_t                             errinfo    = {0};

  /* Check input parameter */

  if (!info && !callback)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
    }

  if (!callback && info && !info->pdn_stat)
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
      if (LTE_SESSION_ID_MIN > pdn_num || LTE_SESSION_ID_MAX < pdn_num)
        {
          return -EINVAL;
        }

      /* Allocate API command buffer to receive */

      presbuff = (FAR struct apicmd_cmddat_getnetinfores_s *)
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

      ret = altcombs_setup_apicallback(APICMDID_GET_NETINFO, callback,
                                       getnetinfo_status_chg_cb);
      if (0 > ret)
        {
          return ret;
        }
    }

  /* Allocate API command buffer to send */

  reqbuff = (FAR uint8_t *)apicmdgw_cmd_allocbuff(APICMDID_GET_NETINFO,
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
          if (pdn_num < presbuff->netinfo.pdn_count)
          {
            ret = -EPROTO;
            errinfo.err_indicator = LTE_ERR_INDICATOR_ERRNO |
                                    LTE_ERR_INDICATOR_ERRSTR;
            errinfo.err_no = -EINVAL;
            snprintf((char *)errinfo.err_string,
                     LTE_ERROR_STRING_MAX_LEN - 1,
                     LTE_ERR_FORMAT_PDN_BUFFER_OVERFLOW,
                     presbuff->netinfo.pdn_count);
            altcombs_set_errinfo(&errinfo);
            goto errout;
          }

          /* Parse network information */

          getnetinfo_parse_response(presbuff, info);
        }
      BUFFPOOL_FREE(presbuff);
    }

  return ret;

errout:
  if (!sync)
    {
      altcombs_teardown_apicallback(APICMDID_GET_NETINFO,
                                    getnetinfo_status_chg_cb);
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
 * Name: lte_get_netinfo_sync
 *
 * Description:
 *   Get LTE network information.
 *
 * Input Parameters:
 *   pdn_num   Number of pdn_stat allocated by the user. The range is from
 *             @ref LTE_PDN_SESSIONID_MIN to @ref LTE_PDN_SESSIONID_MAX.
 *   info      The LTE network information.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_get_netinfo_sync(uint8_t pdn_num, lte_netinfo_t *info)
{
  return lte_getnetinfo_impl(pdn_num, info, NULL);
}

/****************************************************************************
 * Name: lte_get_netinfo
 *
 * Description:
 *   Get LTE network information.
 *
 * Input Parameters:
 *   callback  Callback function to notify that get network information
 *             completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_get_netinfo(get_netinfo_cb_t callback)
{
  if (!callback) {
    DBGIF_LOG_ERROR("Input argument is NULL.\n");
    return -EINVAL;
  }
  return lte_getnetinfo_impl(0, NULL, callback);
}

/****************************************************************************
 * Name: apicmdhdlr_getnetinfo
 *
 * Description:
 *   This function is an API command handler for get netinfo result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_GETNETINFO_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_getnetinfo(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_CONVERT_RES(APICMDID_GET_NETINFO), getnetinfo_job);
}
