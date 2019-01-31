/****************************************************************************
 * modules/lte/altcom/api/lte/lte_setrep_netinfo.c
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
#include "evthdlbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"
#include "apicmdhdlrbs.h"
#include "apiutil.h"
#include "apicmd_repnetinfo.h"
#include "lte_rep_netinfo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SETREP_NETINFO_DATA_LEN (sizeof(struct apicmd_cmddat_set_repnetinfo_s))
#define SETREP_NETINFO_RES_DATA_LEN \
  (sizeof(struct apicmd_cmddat_set_repnetinfores_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: repnetinfo_status_chg_cb
 *
 * Description:
 *   Notification status change in processing set report netinfo.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void repnetinfo_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("repnetinfo_status_chg_cb(%d -> %d)\n",
        new_stat, old_stat);
      altcomcallbacks_unreg_cb(APICMDID_REPORT_NETINFO);
    }
}

/****************************************************************************
 * Name: repnetinfo_job
 *
 * Description:
 *   This function is an API callback for event report receive.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void repnetinfo_job(FAR void *arg)
{
  uint8_t                                i;
  uint8_t                                j;
  FAR struct apicmd_cmddat_rep_netinfo_s *data;
  int32_t                                ret;
  netinfo_report_cb_t                    callback;
  lte_netinfo_t                          netinfo;

  ret = altcomcallbacks_get_unreg_cb(APICMDID_REPORT_NETINFO, (void **)&callback);
  data = (FAR struct apicmd_cmddat_rep_netinfo_s *)arg;

  if ((0 != ret) || (!callback))
    {
      DBGIF_LOG_WARNING("When callback is null called report netstat.\n");
    }
  else
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

        }
      else
        {
          DBGIF_LOG_ERROR("Unexpected!! memory allocation failed.\n");
        }

      callback(&netinfo);
    }

  /* In order to reduce the number of copies of the receive buffer,
   * bring a pointer to the receive buffer to the worker thread.
   * Therefore, the receive buffer needs to be released here. */

  BUFFPOOL_FREE(netinfo.pdn_stat);
  altcom_free_cmd((FAR uint8_t *)arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_set_report_netinfo
 *
 * Description:
 *   Change the report setting of the LTE network state and data
 *   communication state. The default report setting is disable.
 *
 * Input Parameters:
 *   netstat_callback Callback function to notify that LTE network state.
 *                    If NULL is set, the report setting is disabled.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_report_netinfo(netinfo_report_cb_t netinfo_callback)
{
  int32_t                                      ret        = 0;
  FAR struct apicmd_cmddat_set_repnetinfo_s    *cmdbuff   = NULL;
  FAR struct apicmd_cmddat_set_repnetinfores_s *resbuff   = NULL;
  uint16_t                                     resbufflen =
                                               SETREP_NETINFO_RES_DATA_LEN;
  uint16_t                                     reslen     = 0;

  /* Check Lte library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  ret = altcomstatus_reg_statchgcb((void *)repnetinfo_status_chg_cb);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
      return ret;
    }

  /* Accept the API
   * Allocate API command buffer to send */

  cmdbuff = (FAR struct apicmd_cmddat_set_repnetinfo_s *)
    apicmdgw_cmd_allocbuff(APICMDID_SETREP_NETINFO, SETREP_NETINFO_DATA_LEN);
  if (!cmdbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      return -ENOMEM;
    }
  else
    {
      resbuff = (FAR struct apicmd_cmddat_set_repnetinfores_s *)
        BUFFPOOL_ALLOC(resbufflen);
      if (!resbuff)
        {
          DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
          altcom_free_cmd((FAR uint8_t *)cmdbuff);
          return -ENOMEM;
        }

      /* Set command parameter. */

      cmdbuff->report = !netinfo_callback ?
        APICMD_REPNETINFO_REPORT_DISABLE: APICMD_REPNETINFO_REPORT_ENABLE;

      /* Send API command to modem */

      ret = apicmdgw_send((FAR uint8_t *)cmdbuff, (FAR uint8_t *)resbuff,
        resbufflen, &reslen, SYS_TIMEO_FEVR);
    }

  if (0 <= ret)
    {
      /* Register API callback */

      if (APICMD_REPNETINFO_RES_OK == resbuff->result)
        {
          if (netinfo_callback)
            {
              altcomcallbacks_reg_cb((void *)netinfo_callback, APICMDID_REPORT_NETINFO);
            }
          else
            {
              /* Unregistration callback. */

              altcomcallbacks_unreg_cb(APICMDID_REPORT_NETINFO);
              altcomstatus_unreg_statchgcb((void *)repnetinfo_status_chg_cb);
            }
        }
      else
        {
          DBGIF_LOG_ERROR("API command response is err.\n");
          ret = -EIO;
        }
    }

  if (0 <= ret)
    {
      ret = 0;
    }

  altcom_free_cmd((FAR uint8_t *)cmdbuff);
  (void)BUFFPOOL_FREE(resbuff);

  return ret;
}

/****************************************************************************
 * Name: apicmdhdlr_repnetinfo
 *
 * Description:
 *   This function is an API command handler for event report.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_REPORT_NETINFO,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_repnetinfo(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt, APICMDID_REPORT_NETINFO, repnetinfo_job);
}
