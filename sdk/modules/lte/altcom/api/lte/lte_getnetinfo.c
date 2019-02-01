/****************************************************************************
 * modules/lte/altcom/api/lte/lte_getnetinfo.c
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
#include "apicmd_getnetinfo.h"
#include "lte_getnetinfo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETNETINFO_DATA_LEN (0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

static void getnetinfo_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("getnetinfo_status_chg_cb(%d -> %d)\n",
        old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_GET_NETINFO);
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
  uint8_t                                  i;
  uint8_t                                  j;
  FAR struct apicmd_cmddat_getnetinfores_s *data;
  get_netinfo_cb_t                         callback;
  uint32_t                                 result = LTE_RESULT_ERROR;
  FAR lte_netinfo_t                        netinfo;

  data = (FAR struct apicmd_cmddat_getnetinfores_s *)arg;
  ret = altcomcallbacks_get_unreg_cb(APICMDID_GET_NETINFO,
    (void **)&callback);

  if ((0 != ret) || (!callback))
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
    }
  else
    {
      if (data->result == APICMD_GETNETINFO_RES_OK)
        {
          /* Fill network infomation */

          netinfo.nw_stat = data->nw_stat;
          if (0 < data->pdn_count)
            {
              netinfo.pdn_stat = (lte_pdn_t *)
                BUFFPOOL_ALLOC(sizeof(lte_pdn_t) * data->pdn_count);
              for (i = 0; i < data->pdn_count; i++)
                {
                  netinfo.pdn_stat[i].session_id =
                    data->pdn[i].session_id;
                  netinfo.pdn_stat[i].active =
                    data->pdn[i].activate == APICMD_PDN_ACT_ACTIVE ?
                  LTE_PDN_ACTIVE : LTE_PDN_DEACTIVE;
                  netinfo.pdn_stat[i].apn_type = htonl(data->pdn[i].apntype);
                  netinfo.pdn_stat[i].ipaddr_num = data->pdn[i].ipaddr_num;
                  for (j = 0; j < data->pdn[i].ipaddr_num; j++)
                    {
                      memcpy(&netinfo.pdn_stat[i].address[j],
                        &data->pdn[i].ip_address[j], sizeof(lte_ipaddr_t));
                    }

                  netinfo.pdn_stat[i].ims_register =
                    data->pdn[i].imsregister == APICMD_PDN_IMS_REG ?
                    LTE_IMS_REGISTERED : LTE_IMS_NOT_REGISTERED;
                  netinfo.pdn_stat[i].data_allow =
                    data->pdn[i].dataallow == APICMD_PDN_DATAALLOW_ALLOW ?
                    LTE_DATA_ALLOW : LTE_DATA_DISALLOW;
                  netinfo.pdn_stat[i].data_roaming_allow =
                    data->pdn[i].dararoamingallow ==
                    APICMD_PDN_DATAROAMALLOW_ALLOW ? LTE_DATA_ALLOW :
                    LTE_DATA_DISALLOW;
                }

              result = LTE_RESULT_OK;
            }
          else
            {
              DBGIF_LOG_ERROR("Unexpected!! memory allocation failed.\n");
              result = LTE_RESULT_ERROR;
            }
        }

      callback(result, &netinfo);
    }

  /* In order to reduce the number of copies of the receive buffer,
   * bring a pointer to the receive buffer to the worker thread.
   * Therefore, the receive buffer needs to be released here. */

  (void)BUFFPOOL_FREE(netinfo.pdn_stat);
  altcom_free_cmd((FAR uint8_t *)arg);

  /* Unregistration status change callback. */

  altcomstatus_unreg_statchgcb((void *)getnetinfo_status_chg_cb);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_get_netinfo
 *
 * Description:
 *   Get network infomation.
 *
 * Input Parameters:
 *   callback   Callback function to notify that get netinfo completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_get_netinfo(get_netinfo_cb_t callback)
{
  int32_t                                ret;
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

  ret = altcomcallbacks_chk_reg_cb((void *)callback, APICMDID_GET_NETINFO);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Currently API is busy.\n");
      return -EINPROGRESS;
    }

  ret = altcomstatus_reg_statchgcb((void *)getnetinfo_status_chg_cb);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
      altcomcallbacks_unreg_cb(APICMDID_GET_NETINFO);
      return ret;
    }

  /* Accept the API
   * Allocate API command buffer to send */

  cmdbuff = (FAR uint8_t *)apicmdgw_cmd_allocbuff(APICMDID_GET_NETINFO, GETNETINFO_DATA_LEN);
  if (!cmdbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
    }
  else
    {
      /* Send API command to modem */

      ret = altcom_send_and_free((uint8_t *)cmdbuff);
    }

  /* If fail, there is no opportunity to execute the callback,
   * so clear it here. */

  if (0 > ret)
    {
      /* Clear registered callback */

      altcomcallbacks_unreg_cb(APICMDID_GET_NETINFO);
      altcomstatus_unreg_statchgcb((void *)getnetinfo_status_chg_cb);
    }
  else
    {
      ret = 0;
    }

  return ret;
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
