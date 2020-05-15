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
#include "apicmd_cellinfo.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REQ_DATA_LEN (sizeof(struct apicmd_cmddat_setrepcellinfo_s))
#define RES_DATA_LEN (sizeof(struct apicmd_cmddat_setrepcellinfo_res_s))
#define CELLINFO_PERIOD_MIN (1)
#define CELLINFO_PERIOD_MAX (4233600)

/****************************************************************************
 * Private Data
 ****************************************************************************/

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

static int32_t repcellinfo_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("repcellinfo_status_chg_cb(%d -> %d)\n",
        old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_SET_REP_CELLINFO);

      return ALTCOM_STATUS_REG_CLR;
    }

  return ALTCOM_STATUS_REG_KEEP;
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
  FAR struct apicmd_cmddat_cellinfo_s *data;
  lte_cellinfo_t                      *repdat = NULL;
  cellinfo_report_cb_t                 callback;

  callback = altcomcallbacks_get_cb(APICMDID_SET_REP_CELLINFO);
  if (!callback)
    {
      DBGIF_LOG_WARNING("When callback is null called report cellinfo.\n");
    }
  else
    {
      data = (FAR struct apicmd_cmddat_cellinfo_s *)arg;

      repdat = (lte_cellinfo_t *)BUFFPOOL_ALLOC(sizeof(lte_cellinfo_t));
      if (!repdat)
        {
          DBGIF_LOG_DEBUG("report data buffer alloc error.\n");
        }
      else
        {
          altcombs_set_cellinfo(data, repdat);

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
 *   Invoke the callback at the specified report interval.
 *   The default report setting is disable.
 *
 * Input Parameters:
 *   cellinfo_callback Callback function to notify that cell information.
 *                     If NULL is set, the report setting is disabled.
 *   period            Reporting cycle in sec (1-4233600).
 *
 * Returned Value:
 *   On success, 0 is returned. On failure,
 *   negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_set_report_cellinfo(cellinfo_report_cb_t cellinfo_callback,
                                uint32_t period)
{
  int32_t                                       ret        = 0;
  FAR struct apicmd_cmddat_setrepcellinfo_res_s resbuff    = {0};
  uint16_t                                      resbufflen = RES_DATA_LEN;
  FAR struct apicmd_cmddat_setrepcellinfo_s     *reqbuff    = NULL;
  uint16_t                                      reslen     = 0;

  /* Check input parameter */

  if (cellinfo_callback)
    {
      if (CELLINFO_PERIOD_MIN > period || CELLINFO_PERIOD_MAX < period)
        {
          DBGIF_LOG_ERROR("Invalid parameter.\n");
          return -EINVAL;
        }
    }

  /* Check LTE library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  /* Setup API callback */

  if (cellinfo_callback)
    {
      ret = altcombs_setup_apicallback(APICMDID_SET_REP_CELLINFO,
                                       cellinfo_callback,
                                       repcellinfo_status_chg_cb);
      if (0 > ret)
        {
          return ((ret == -EINPROGRESS) ? -EALREADY : ret);
        }
    }

   /* Allocate API command buffer to send */

  reqbuff = (FAR struct apicmd_cmddat_setrepcellinfo_s *)
             apicmdgw_cmd_allocbuff(APICMDID_SET_REP_CELLINFO, REQ_DATA_LEN);
  if (!reqbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Set event field */

  reqbuff->enability = !cellinfo_callback ? LTE_DISABLE : LTE_ENABLE;
  reqbuff->interval = htonl(period);

  /* Send API command to modem */

  ret = apicmdgw_send((FAR uint8_t *)reqbuff,
                      (FAR uint8_t *)&resbuff,
                      resbufflen,
                      &reslen,
                      SYS_TIMEO_FEVR);
  altcom_free_cmd((FAR uint8_t *)reqbuff);

  if (0 > ret)
    {
      goto errout;
    }
  ret = (LTE_RESULT_OK == resbuff.result) ? 0 : -EIO;
  if (ret == 0 && !cellinfo_callback)
    {
      altcombs_teardown_apicallback(APICMDID_SET_REP_CELLINFO,
                                    repcellinfo_status_chg_cb);
    }

  return ret;

errout:
  if (cellinfo_callback)
    {
      altcombs_teardown_apicallback(APICMDID_SET_REP_CELLINFO,
                                    repcellinfo_status_chg_cb);
    }
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
