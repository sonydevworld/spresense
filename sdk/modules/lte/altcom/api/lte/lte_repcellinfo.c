/****************************************************************************
 * modules/lte/altcom/api/lte/lte_repevt.c
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
#include <string.h>

#include "lte/lte_api.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "apicmd_repcellinfo.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CELLINFO_DATA_LEN (sizeof(struct apicmd_cmddat_setrepcellinfo_s))
#define CELLINFO_SETRES_DATA_LEN \
  (sizeof(struct apicmd_cmddat_setrepcellinfo_res_s))
#define CELLINFO_PERIOD_MIN (1)
#define CELLINFO_PERIOD_MAX (4233600)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_lte_setrepcellinfo_isproc = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: repcellinfo_status_chg_cb
 *
 * Description:
 *   Notification status change in processing report cellinfo .
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void repcellinfo_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("repcellinfo_status_chg_cb(%d -> %d)\n",
        new_stat, old_stat);
      altcomcallbacks_unreg_cb(APICMDID_SET_REP_CELLINFO);
    }
}

/****************************************************************************
 * Name: check_arrydigitnum
 *
 * Description:
 *   Evaluate the validity of each digit of arrayed numerical value.
 *
 * Input Parameters:
 *  number    Array type number.
 *  digit     @number digit.
 *
 * Returned Value:
 *   Returns true if everything is valid. Otherwise it returns false.
 *
 ****************************************************************************/

static bool check_arrydigitnum(FAR uint8_t number[], uint8_t digit)
{
  uint8_t cnt;

  for (cnt = 0; cnt < digit; cnt++)
    {
      if (number[cnt] < APICMD_SET_REPCELLINFO_DIGIT_NUM_MIN ||
        APICMD_SET_REPCELLINFO_DIGIT_NUM_MAX < number[cnt])
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: repcellinfo_job
 *
 * Description:
 *   This function is an API callback for cellinfo report receive.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void repcellinfo_job(FAR void *arg)
{
  FAR struct apicmd_cmddat_repcellinfo_s *data;
  lte_cellinfo_t                         *repdat = NULL;
  cellinfo_report_cb_t                    callback;

  callback = altcomcallbacks_get_cb(APICMDID_SET_REP_CELLINFO);
  if (!callback)
    {
      DBGIF_LOG_WARNING("When callback is null called report cellinfo.\n");
    }
  else
    {
      data = (FAR struct apicmd_cmddat_repcellinfo_s *)arg;

      repdat = (lte_cellinfo_t *)BUFFPOOL_ALLOC(sizeof(lte_cellinfo_t));
      if (!repdat)
        {
          DBGIF_LOG_DEBUG("report data buffer alloc error.\n");
        }
      else
        {
          repdat->valid = APICMD_REP_CELLINFO_ENABLE == data->enability ?
                          LTE_VALID : LTE_INVALID;
          if (repdat->valid)
            {
              repdat->phycell_id = ntohl(data->cell_id);
              repdat->earfcn     = ntohl(data->earfcn);
              memcpy(repdat->mcc, data->mcc,
                     APICMD_SET_REPCELLINFO_MCC_DIGIT);
              repdat->mnc_digit  = data->mnc_digit;
              memcpy(repdat->mnc, data->mnc, data->mnc_digit);

              if (repdat->phycell_id < APICMD_SET_REPCELLINFO_CELLID_MIN ||
                APICMD_SET_REPCELLINFO_CELLID_MAX < repdat->phycell_id)
                {
                  DBGIF_LOG1_ERROR("repdat->phycell_id error:%d\n", repdat->phycell_id);
                  repdat->valid = LTE_INVALID;
                }
              else if (repdat->earfcn < APICMD_SET_REPCELLINFO_EARFCN_MIN ||
                APICMD_SET_REPCELLINFO_EARFCN_MAX < repdat->earfcn)
                {
                  DBGIF_LOG1_ERROR("repdat->earfcn error:%d\n", repdat->earfcn);
                  repdat->valid = LTE_INVALID;
                }
              else if (!check_arrydigitnum(repdat->mcc,
                APICMD_SET_REPCELLINFO_MCC_DIGIT))
                {
                  DBGIF_LOG_ERROR("repdat->mcc error\n");
                  repdat->valid = LTE_INVALID;
                }
              else if (
                repdat->mnc_digit < APICMD_SET_REPCELLINFO_MNC_DIGIT_MIN ||
                APICMD_SET_REPCELLINFO_MNC_DIGIT_MAX < repdat->mnc_digit)
                {
                  DBGIF_LOG1_ERROR("repdat->mnc_digit error:%d\n", repdat->mnc_digit);
                  repdat->valid = LTE_INVALID;
                }
              else if (!check_arrydigitnum(repdat->mnc, repdat->mnc_digit))
                {
                  DBGIF_LOG_ERROR("repdat->mnc error\n");
                  repdat->valid = LTE_INVALID;
                }
            }

          callback(repdat);
          (void)BUFFPOOL_FREE((FAR void *)repdat);
        }
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
 * Name: lte_set_report_cellinfo
 *
 * Description:
 *   Change the report setting of the cell information. The default report
 *   setting is disable.
 *
 * Input Parameters:
 *   cellinfo_callback Callback function to notify that cell information.
 *                     If NULL is set, the report setting is disabled.
 *   period            Reporting cycle in sec (1-4233600).
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_report_cellinfo(cellinfo_report_cb_t cellinfo_callback,
                                uint32_t period)
{
  int32_t                                       ret        = 0;
  FAR struct apicmd_cmddat_setrepcellinfo_s     *cmdbuff   = NULL;
  FAR struct apicmd_cmddat_setrepcellinfo_res_s *resbuff   = NULL;
  uint16_t                                      resbufflen =
                                                  CELLINFO_SETRES_DATA_LEN;
  uint16_t                                      reslen     = 0;
  bool                                          reset_flag = false;
  cellinfo_report_cb_t                          callback;

  if (cellinfo_callback)
    {
      if (CELLINFO_PERIOD_MIN > period || CELLINFO_PERIOD_MAX < period)
        {
          DBGIF_LOG_ERROR("Invalid parameter.\n");
          return -EINVAL;
        }
    }

  /* Check Lte library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  /* Check this process runnning. */

  if (g_lte_setrepcellinfo_isproc)
    {
      return -EBUSY;
    }
  g_lte_setrepcellinfo_isproc = true;

  if (cellinfo_callback)
    {
      /* Check callback is registered */

      callback = altcomcallbacks_get_cb(APICMDID_SET_REP_CELLINFO);
      if (callback)
        {
          reset_flag = true;
        }
      else
        {
          ret = altcomstatus_reg_statchgcb((void *)repcellinfo_status_chg_cb);
          if (0 > ret)
            {
              DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
              g_lte_setrepcellinfo_isproc = false;
              return ret;
            }
        }
    }

  /* Accept the API */
  /* Allocate API command buffer to send */

  cmdbuff = (FAR struct apicmd_cmddat_setrepcellinfo_s *)
    apicmdgw_cmd_allocbuff(APICMDID_SET_REP_CELLINFO, CELLINFO_DATA_LEN);
  if (!cmdbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      g_lte_setrepcellinfo_isproc = false;
      return -ENOMEM;
    }
  else
    {
      resbuff = (FAR struct apicmd_cmddat_setrepcellinfo_res_s *)
        BUFFPOOL_ALLOC(resbufflen);
      if (!resbuff)
        {
          DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
          altcom_free_cmd((FAR uint8_t *)cmdbuff);
          g_lte_setrepcellinfo_isproc = false;
          return -ENOMEM;
        }

      /* Set event field */

      cmdbuff->enability = !cellinfo_callback ?
        APICMD_SET_REPCELLINFO_DISABLE :
        APICMD_SET_REPCELLINFO_ENABLE;
      cmdbuff->interval = htonl(period);

      ret = apicmdgw_send((FAR uint8_t *)cmdbuff, (FAR uint8_t *)resbuff,
        resbufflen, &reslen, SYS_TIMEO_FEVR);
    }

  if (0 <= ret && resbufflen == reslen)
    {
      if (APICMD_SET_REPCELLINFO_RES_OK == resbuff->result)
        {
          if (cellinfo_callback)
            {
              if (!reset_flag)
                {
                  altcomcallbacks_reg_cb((void *)cellinfo_callback,
                                          APICMDID_SET_REP_CELLINFO);
                }
            }
          else
            {
              /* Unregistration callback. */

              altcomcallbacks_unreg_cb(APICMDID_SET_REP_CELLINFO);
              altcomstatus_unreg_statchgcb((void *)repcellinfo_status_chg_cb);
            }
        }
      else
        {
          DBGIF_LOG_ERROR("API command response is err.\n");
          ret= -EIO;
        }
    }

  if (0 <= ret)
    {
      ret = 0;
    }

  altcom_free_cmd((FAR uint8_t *)cmdbuff);
  (void)BUFFPOOL_FREE(resbuff);
  g_lte_setrepcellinfo_isproc = false;

  return ret;
}

/****************************************************************************
 * Name: apicmdhdlr_repcellinfo
 *
 * Description:
 *   This function is an API command handler for cellinfo report.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_REPORT_CELLINFO,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_repcellinfo(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_REPORT_CELLINFO, repcellinfo_job);
}
