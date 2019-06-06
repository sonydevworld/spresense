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

#define GET_SIMINFO_DATA_LEN (sizeof(struct apicmd_cmddat_getsiminfo_s))

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
          siminfo.option = ntohl(data->option);
          if (0 > lte_getsiminfo_checkparam(siminfo.option))
            {
              DBGIF_LOG1_ERROR("Invalid param. Option[%d]\n", siminfo.option);
              result = LTE_RESULT_ERROR;
            }
          else
            {
              result = LTE_RESULT_OK;

              if (siminfo.option & LTE_SIMINFO_GETOPT_MCCMNC)
                {
                  if (LTE_SIMINFO_MNC_DIGIT_MAX >= data->mnc_digit)
                    {
                      memcpy(siminfo.mcc, data->mcc, LTE_SIMINFO_MCC_DIGIT);
                      siminfo.mnc_digit = data->mnc_digit;
                      memcpy(siminfo.mnc, data->mnc, data->mnc_digit);
                    }
                  else
                    {
                      DBGIF_LOG1_ERROR("Invalid param. MNC digit [%d]\n", data->mnc_digit);
                      result = LTE_RESULT_ERROR;
                    }
                }

              if (siminfo.option & LTE_SIMINFO_GETOPT_SPN)
                {
                  if (LTE_SIMINFO_SPN_LEN >= data->spn_len)
                    {
                      siminfo.spn_len = data->spn_len;
                      memcpy(siminfo.spn, data->spn, data->spn_len);
                    }
                  else
                    {
                      DBGIF_LOG1_ERROR("Invalid param. SPN length [%d]\n", data->spn_len);
                      result = LTE_RESULT_ERROR;
                    }
                }

              if (siminfo.option & LTE_SIMINFO_GETOPT_ICCID)
                {
                  if (LTE_SIMINFO_ICCID_LEN >= data->iccid_len)
                    {
                      siminfo.iccid_len = data->iccid_len;
                      memcpy(siminfo.iccid, data->iccid, data->iccid_len);
                    }
                  else
                    {
                      DBGIF_LOG1_ERROR("Invalid param. ICCID length [%d]\n", data->iccid_len);
                      result = LTE_RESULT_ERROR;
                    }
                }

              if (siminfo.option & LTE_SIMINFO_GETOPT_IMSI)
                {
                  if (LTE_SIMINFO_IMSI_LEN >= data->imsi_len)
                    {
                      siminfo.imsi_len = data->imsi_len;
                      memcpy(siminfo.imsi, data->imsi, data->imsi_len);
                    }
                  else
                    {
                      DBGIF_LOG1_ERROR("Invalid param. IMSI length [%d]\n", data->imsi_len);
                      result = LTE_RESULT_ERROR;
                    }
                }

              if (siminfo.option & LTE_SIMINFO_GETOPT_GID1)
                {
                  if (LTE_SIMINFO_GID_LEN >= data->gid1_len)
                    {
                      siminfo.gid1_len = data->gid1_len;
                      memcpy(siminfo.gid1, data->gid1, data->gid1_len);
                    }
                  else
                    {
                      DBGIF_LOG1_ERROR("Invalid param. GID1 length [%d]\n", data->gid1_len);
                      result = LTE_RESULT_ERROR;
                    }
                }

              if (siminfo.option & LTE_SIMINFO_GETOPT_GID2)
                {
                  if (LTE_SIMINFO_GID_LEN >= data->gid2_len)
                    {
                      siminfo.gid2_len = data->gid2_len;
                      memcpy(siminfo.gid2, data->gid2, data->gid2_len);
                    }
                  else
                    {
                      DBGIF_LOG1_ERROR("Invalid param. GID2 length [%d]\n", data->gid2_len);
                      result = LTE_RESULT_ERROR;
                    }
                }
            }
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_get_siminfo
 *
 * Description:
 *   Get SIM infomation.
 *
 * Input Parameters:
 *   callback Callback function to notify that get SIM infomation completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_get_siminfo(uint32_t option, get_siminfo_cb_t callback)
{
  int32_t     ret;
  FAR struct apicmd_cmddat_getsiminfo_s *cmdbuff;

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

  /* Check input param */

  ret = lte_getsiminfo_checkparam(option);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("Input option is invalid. option:%d\n", option);
      return ret;
    }

  /* Register API callback */

  ret = altcomcallbacks_chk_reg_cb(callback, APICMDID_GET_SIMINFO);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Currently API is busy.\n");
      return -EINPROGRESS;
    }

  ret = altcomstatus_reg_statchgcb(getsiminfo_status_chg_cb);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
      altcomcallbacks_unreg_cb(APICMDID_GET_SIMINFO);
      return ret;
    }

  /* Accept the API
   * Allocate API command buffer to send */

  cmdbuff = (FAR struct apicmd_cmddat_getsiminfo_s *)
    apicmdgw_cmd_allocbuff(APICMDID_GET_SIMINFO,
    GET_SIMINFO_DATA_LEN);
  if (!cmdbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
    }
  else
    {
      /* Send API command to modem */

      cmdbuff->option = htonl(option);
      ret = altcom_send_and_free((FAR uint8_t *)cmdbuff);
    }

  /* If fail, there is no opportunity to execute the callback,
   * so clear it here. */

  if (0 > ret)
    {
      /* Clear registered callback */

      altcomcallbacks_unreg_cb(APICMDID_GET_SIMINFO);
      altcomstatus_unreg_statchgcb(getsiminfo_status_chg_cb);
    }
  else
    {
      ret = 0;
    }

  return ret;
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
