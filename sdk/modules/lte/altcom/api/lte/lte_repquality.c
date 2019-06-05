/****************************************************************************
 * modules/lte/altcom/api/lte/lte_repquality.c
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
#include "apicmd_repquality.h"
#include "apicmd_quality.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QUALITY_DATA_LEN (sizeof(struct apicmd_cmddat_setrepquality_s))
#define QUALITY_SETRES_DATA_LEN \
  (sizeof(FAR struct apicmd_cmddat_setrepquality_res_s))
#define QUALITY_PERIOD_MIN (1)
#define QUALITY_PERIOD_MAX (4233600)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_lte_setrepquality_isproc = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: repquality_status_chg_cb
 *
 * Description:
 *   Notification status change in processing report quality.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static int32_t repquality_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("repquality_status_chg_cb(%d -> %d)\n",
        old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_SET_REP_QUALITY);

      return ALTCOM_STATUS_REG_CLR;
    }

  return ALTCOM_STATUS_REG_KEEP;
}

/****************************************************************************
 * Name: repquality_job
 *
 * Description:
 *   This function is an API callback for quality report receive.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void repquality_job(FAR void *arg)
{
  FAR struct apicmd_cmddat_quality_s *data;
  lte_quality_t                         *repdat = NULL;
  quality_report_cb_t                    callback;

  callback = altcomcallbacks_get_cb(APICMDID_SET_REP_QUALITY);
  if (!callback)
    {
      DBGIF_LOG_WARNING("When callback is null called report quality.\n");
    }
  else
    {
      data = (FAR struct apicmd_cmddat_quality_s *)arg;

      repdat = (lte_quality_t *)BUFFPOOL_ALLOC(sizeof(lte_quality_t));
      if (!repdat)
        {
          DBGIF_LOG_DEBUG("report data buffer alloc error.\n");
        }
      else
        {
          altcombs_set_quality(repdat, data);
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
 * Name: lte_set_report_quality
 *
 * Description:
 *   Change the report setting of the quality information.
 *   The default report setting is disable.
 *
 * Input Parameters:
 *   quality_callback Callback function to notify that quality.
 *                    If NULL is set, the report setting is disabled.
 *   period           Reporting cycle in sec (1-4233600).
 *   result_callback  Callback function to notify that report setting has
 *                    changed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_report_quality(quality_report_cb_t quality_callback,
                               uint32_t period)
{
  int32_t                                      ret        = 0;
  FAR struct apicmd_cmddat_setrepquality_s     *cmdbuff   = NULL;
  FAR struct apicmd_cmddat_setrepquality_res_s *resbuff   = NULL;
  uint16_t                                     resbufflen =
                                                 QUALITY_SETRES_DATA_LEN;
  uint16_t                                     reslen     = 0;
  bool                                         reset_flag = false;
  quality_report_cb_t                          callback;

  if (quality_callback)
    {
      if (QUALITY_PERIOD_MIN > period || QUALITY_PERIOD_MAX < period)
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

  if (g_lte_setrepquality_isproc)
    {
      return -EBUSY;
    }
  g_lte_setrepquality_isproc = true;

  if (quality_callback)
    {
      /* Check callback is registered */

      callback = altcomcallbacks_get_cb(APICMDID_SET_REP_QUALITY);
      if (callback)
        {
          reset_flag = true;
        }
      else
        {
          ret = altcomstatus_reg_statchgcb(repquality_status_chg_cb);
          if (0 > ret)
            {
              DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
              g_lte_setrepquality_isproc = false;
              return ret;
            }
        }
    }

  /* Allocate API command buffer to send */

  cmdbuff = (FAR struct apicmd_cmddat_setrepquality_s *)
    apicmdgw_cmd_allocbuff(APICMDID_SET_REP_QUALITY, QUALITY_DATA_LEN);
  if (!cmdbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      g_lte_setrepquality_isproc = false;
      return -ENOSPC;
    }
  else
    {
      resbuff = (FAR struct apicmd_cmddat_setrepquality_res_s *)
        BUFFPOOL_ALLOC(resbufflen);
      if (!resbuff)
        {
          DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
          altcom_free_cmd((FAR uint8_t *)cmdbuff);
          g_lte_setrepquality_isproc = false;
          return -ENOSPC;
        }

      /* Set event field */

      cmdbuff->enability = !quality_callback ?
        APICMD_SET_REPQUALITY_DISABLE :
        APICMD_SET_REPQUALITY_ENABLE;
      cmdbuff->interval = htonl(period);

      ret = apicmdgw_send((FAR uint8_t *)cmdbuff, (FAR uint8_t *)resbuff,
        resbufflen, &reslen, SYS_TIMEO_FEVR);
    }

  if (0 <= ret && resbufflen == reslen)
    {
      if (APICMD_SET_REPQUALITY_RES_OK == resbuff->result)
        {
          if (quality_callback)
            {
              if (!reset_flag)
                {
                  altcomcallbacks_reg_cb((void *)quality_callback,
                                          APICMDID_SET_REP_QUALITY);
                }
            }
          else
            {
              /* Unregistration callback. */

              altcomcallbacks_unreg_cb(APICMDID_SET_REP_QUALITY);
              altcomstatus_unreg_statchgcb(repquality_status_chg_cb);
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
  g_lte_setrepquality_isproc = false;

  return ret;
}

/****************************************************************************
 * Name: apicmdhdlr_repquality
 *
 * Description:
 *   This function is an API command handler for quality report.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_REPORT_QUALITY,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_repquality(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt, APICMDID_REPORT_QUALITY, repquality_job);
}
