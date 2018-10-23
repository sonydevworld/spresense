/****************************************************************************
 * modules/lte/altcom/api/lte/apicmdhdlr_repcellinfo.c
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

#include <string.h>

#include "lte/lte_api.h"
#include "buffpoolwrapper.h"
#include "apicmd_repcellinfo.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern cellinfo_report_cb_t g_cellinfo_callback;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

  if (!g_cellinfo_callback)
    {
      DBGIF_LOG_DEBUG("g_cellinfo_callback is not registered.\n");
      return;
    }

  data = (FAR struct apicmd_cmddat_repcellinfo_s *)arg;

  repdat = (lte_cellinfo_t *)BUFFPOOL_ALLOC(sizeof(lte_cellinfo_t));
  if (!repdat)
    {
      DBGIF_LOG_DEBUG("report data buffer alloc error.\n");
    }
  else
    {
      repdat->valid      = APICMD_REP_CELLINFO_ENABLE == data->enability ?
                           LTE_VALID : LTE_INVALID;
      if (repdat->valid)
        {
          repdat->phycell_id = ntohl(data->cell_id);
          repdat->earfcn     = ntohl(data->earfcn);
          memcpy(repdat->mcc, data->mcc, APICMD_SET_REPCELLINFO_MCC_DIGIT);
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

      g_cellinfo_callback(repdat);
      (void)BUFFPOOL_FREE((FAR void *)repdat);
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
