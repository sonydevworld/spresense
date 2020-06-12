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

#include "lte/lte_api.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "apicmd_repevt.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REPEVT_DATA_LEN (sizeof(struct apicmd_cmddat_setrepevt_s))
#define REPSETRES_DATA_LEN (sizeof(struct apicmd_cmddat_setrepevtres_s))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_set_repevt = 0;
static bool g_lte_setrepsimstat_isproc = false;
static bool g_lte_setrepltime_isproc = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: repevt_ltime_status_chg_cb
 *
 * Description:
 *   Notification status change in processing report LTIME.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static int32_t repevt_ltime_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("repevt_ltime_status_chg_cb(%d -> %d)\n",
        old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_SET_REP_EVT_LTIME);

      return ALTCOM_STATUS_REG_CLR;
    }

  return ALTCOM_STATUS_REG_KEEP;
}

/****************************************************************************
 * Name: repevt_simstate_status_chg_cb
 *
 * Description:
 *   Notification status change in processing report SIM state.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static int32_t repevt_simstate_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("repevt_simstate_status_chg_cb(%d -> %d)\n",
        old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_SET_REP_EVT_SIMSTATE);

      return ALTCOM_STATUS_REG_CLR;
    }

  return ALTCOM_STATUS_REG_KEEP;
}

/****************************************************************************
 * Name: repevt_ltime_report
 *
 * Description:
 *   This function is an API callback for event report LTIME.
 *
 * Input Parameters:
 *  ltime    Pointer to received LTIME data.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void repevt_ltime_report(FAR struct apicmd_cmddat_ltime_s *ltime)
{
  lte_localtime_t       result;
  localtime_report_cb_t callback;

  callback = altcomcallbacks_get_cb(APICMDID_SET_REP_EVT_LTIME);
  if (!callback)
    {
      DBGIF_LOG_WARNING("When callback is null called report localtime.\n");
      return;
    }
  
  result.year   = ltime->year;
  result.mon    = ltime->month;
  result.mday   = ltime->day;
  result.hour   = ltime->hour;
  result.min    = ltime->minutes;
  result.sec    = ltime->seconds;
  result.tz_sec = ntohl(ltime->timezone);

  callback(&result);
}

/****************************************************************************
 * Name: repevt_simd_report
 *
 * Description:
 *   This function is an API callback for event report SIMD.
 *
 * Input Parameters:
 *  ltime    Pointer to received SIMD data.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void repevt_simd_report(FAR struct apicmd_cmddat_repevt_simd_s *simd)
{
  uint8_t             result;
  simstat_report_cb_t callback;

  callback = altcomcallbacks_get_cb(APICMDID_SET_REP_EVT_SIMSTATE);
  if (!callback)
    {
      DBGIF_LOG_WARNING("When callback is null called report SIM state.\n");
      return;
    }

  switch (simd->status)
    {
      case APICMD_REPORT_EVT_SIMD_REMOVAL:
        {
          result = LTE_SIMSTAT_REMOVAL;
        }
      break;
      case APICMD_REPORT_EVT_SIMD_INSERTION:
        {
          result = LTE_SIMSTAT_INSERTION;
        }
      break;
      default:
        {
          DBGIF_LOG1_ERROR("Unsupport SIMD status. status:%d\n", simd->status);
          return;
        }
    }

  callback(result);
}

/****************************************************************************
 * Name: repevt_simstate_report
 *
 * Description:
 *   This function is an API callback for event report SIMSTATE.
 *
 * Input Parameters:
 *  ltime    Pointer to received SIMSTATE data.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void repevt_simstate_report(
  FAR struct apicmd_cmddat_repevt_simstate_s *simstate)
{
  uint8_t             result;
  simstat_report_cb_t callback;

  callback = altcomcallbacks_get_cb(APICMDID_SET_REP_EVT_SIMSTATE);
  if (!callback)
    {
      DBGIF_LOG_WARNING("When callback is null called report SIM state.\n");
      return;
    }

  switch (simstate->state)
    {
      case APICMD_REPORT_EVT_SIMSTATE_SIM_INIT_WAIT_PIN_UNLOCK:
        {
          result = LTE_SIMSTAT_WAIT_PIN_UNLOCK;
        }
      break;
      case APICMD_REPORT_EVT_SIMSTATE_PERSONALIZATION_FAILED:
        {
          result = LTE_SIMSTAT_PERSONAL_FAILED;
        }
      break;
      case APICMD_REPORT_EVT_SIMSTATE_ACTIVATION_COMPLETED:
        {
          result = LTE_SIMSTAT_ACTIVATE;
        }
      break;
      case APICMD_REPORT_EVT_SIMSTATE_DEACTIVATED:
        {
          result = LTE_SIMSTAT_DEACTIVATE;
        }
      break;
      default:
        {
          DBGIF_LOG1_ERROR("Unsupport SIM state. status:%d\n", simstate->state);
          return;
        }
    }

  callback(result);
}

/****************************************************************************
 * Name: repevt_job
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

static void repevt_job(FAR void *arg)
{
  FAR struct apicmd_cmddat_repevt_s *data;

  data = (FAR struct apicmd_cmddat_repevt_s *)arg;

  switch (data->type)
    {
      case APICMD_REPORT_EVT_TYPE_LTIME:
        {
          repevt_ltime_report(&data->u.ltime);
        }
      break;
      case APICMD_REPORT_EVT_TYPE_SIMD:
        {
          repevt_simd_report(&data->u.simd);
        }
      break;
      case APICMD_REPORT_EVT_TYPE_SIMSTATE:
        {
          repevt_simstate_report(&data->u.simstate);
        }
      break;
      default:
        {
          DBGIF_LOG1_ERROR("Unsupport event type. type:%d\n", data->type);
        }
      break;
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
 * Name: lte_set_report_simstat
 *
 * Description:
 *   Change the report setting of the SIM state.
 *   The default report setting is disable.
 *
 * Input Parameters:
 *   simstat_callback  Callback function to notify that SIM state.
 *                     If NULL is set, the report setting is disabled.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_report_simstat(simstat_report_cb_t simstat_callback)
{
  int32_t                                 ret        = 0;
  FAR struct apicmd_cmddat_setrepevt_s    *cmdbuff   = NULL;
  FAR struct apicmd_cmddat_setrepevtres_s *resbuff   = NULL;
  uint16_t                                resbufflen = REPSETRES_DATA_LEN;
  uint16_t                                reslen     = 0;
  bool                                    reset_flag = false;
  simstat_report_cb_t                     callback;

  /* Check Lte library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  /* Check this process runnning. */

  if (g_lte_setrepsimstat_isproc)
    {
      return -EBUSY;
    }
  g_lte_setrepsimstat_isproc = true;

  if (simstat_callback)
    {
      /* Check callback is registered */

      callback = altcomcallbacks_get_cb(APICMDID_SET_REP_EVT_SIMSTATE);
      if (callback)
        {
          reset_flag = true;
        }
      else
        {
          ret = altcomstatus_reg_statchgcb(repevt_simstate_status_chg_cb);
          if (0 > ret)
            {
              DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
              g_lte_setrepsimstat_isproc = false;
              return ret;
            }
        }
    }

  /* Allocate API command buffer to send */

  cmdbuff = (FAR struct apicmd_cmddat_setrepevt_s *)
    apicmdgw_cmd_allocbuff(APICMDID_SET_REP_EVT, REPEVT_DATA_LEN);
  if (!cmdbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      g_lte_setrepsimstat_isproc = false;
      return -ENOMEM;
    }
  else
    {
      /* Set event field */

      resbuff = (FAR struct apicmd_cmddat_setrepevtres_s *)
        BUFFPOOL_ALLOC(resbufflen);
      if (!resbuff)
        {
          DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
          altcom_free_cmd((FAR uint8_t *)cmdbuff);
          g_lte_setrepsimstat_isproc = false;
          return -ENOMEM;
        }

      if (simstat_callback)
        {
          g_set_repevt |=
            (APICMD_SET_REP_EVT_SIMD | APICMD_SET_REP_EVT_SIMSTATE);
        }
      else
        {
          g_set_repevt &=
            ~(APICMD_SET_REP_EVT_SIMD | APICMD_SET_REP_EVT_SIMSTATE);
        }

      cmdbuff->event = g_set_repevt;

      /* Send API command to modem */

      ret = apicmdgw_send((FAR uint8_t *)cmdbuff, (FAR uint8_t *)resbuff,
        resbufflen, &reslen, SYS_TIMEO_FEVR);
    }

  if (0 <= ret && resbufflen == reslen)
    {
      if (APICMD_SET_REP_EVT_RES_OK == resbuff->result)
        {
          if (simstat_callback)
            {
              if (!reset_flag)
                {
                  altcomcallbacks_reg_cb((void *)simstat_callback,
                                          APICMDID_SET_REP_EVT_SIMSTATE);
                }
            }
          else
            {
              /* Unregistration callback. */

              altcomcallbacks_unreg_cb(APICMDID_SET_REP_EVT_SIMSTATE);
              altcomstatus_unreg_statchgcb(repevt_simstate_status_chg_cb);
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
  g_lte_setrepsimstat_isproc = false;

  return ret;
}

/****************************************************************************
 * Name: altcom_set_report_localtime
 *
 * Description:
 *   Change the report setting of the local time.
 *   The default report setting is disable.
 *
 * Input Parameters:
 *   localtime_callback Callback function to notify that local time.
 *                      If NULL is set, the report setting is disabled.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t altcom_set_report_localtime(localtime_report_cb_t localtime_callback)
{
  int32_t                                 ret        = 0;
  FAR struct apicmd_cmddat_setrepevt_s    *cmdbuff   = NULL;
  FAR struct apicmd_cmddat_setrepevtres_s *resbuff   = NULL;
  uint16_t                                resbufflen = REPSETRES_DATA_LEN;
  uint16_t                                reslen     = 0;
  bool                                    reset_flag = false;
  localtime_report_cb_t                   callback;

  /* Check Lte library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  /* Check this process runnning. */

  if (g_lte_setrepltime_isproc)
    {
      return -EBUSY;
    }
  g_lte_setrepltime_isproc = true;

  if (localtime_callback)
    {
      /* Check callback is registered */

      callback = altcomcallbacks_get_cb(APICMDID_SET_REP_EVT_LTIME);
      if (callback)
        {
          reset_flag = true;
        }
      else
        {
          ret = altcomstatus_reg_statchgcb(repevt_ltime_status_chg_cb);
          if (0 > ret)
            {
              DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
              g_lte_setrepltime_isproc = false;
              return ret;
            }
        }
    }

    /* Allocate API command buffer to send */

    cmdbuff = (FAR struct apicmd_cmddat_setrepevt_s *)
      apicmdgw_cmd_allocbuff(APICMDID_SET_REP_EVT, REPEVT_DATA_LEN);
    if (!cmdbuff)
      {
        DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
        g_lte_setrepltime_isproc = false;
        return -ENOSPC;
      }
    else
      {
        resbuff = (FAR struct apicmd_cmddat_setrepevtres_s *)
          BUFFPOOL_ALLOC(resbufflen);
        if (!resbuff)
          {
            DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
            altcom_free_cmd((FAR uint8_t *)cmdbuff);
            g_lte_setrepltime_isproc = false;
            return -ENOSPC;
          }
        
        /* Set event field */

        if (localtime_callback)
          {
            g_set_repevt |= APICMD_SET_REP_EVT_LTIME;
          }
        else
          {
            g_set_repevt &= ~APICMD_SET_REP_EVT_LTIME;
          }

        cmdbuff->event = g_set_repevt;

        /* Send API command to modem */

        ret = apicmdgw_send((FAR uint8_t *)cmdbuff, (FAR uint8_t *)resbuff,
          resbufflen, &reslen, SYS_TIMEO_FEVR);
      }

  if (0 <= ret && resbufflen == reslen)
    {
      if (APICMD_SET_REP_EVT_RES_OK == resbuff->result)
        {
          if (localtime_callback)
            {
              if (!reset_flag)
                {
                  altcomcallbacks_reg_cb((void *)localtime_callback,
                                          APICMDID_SET_REP_EVT_LTIME);
                }
            }
          else
            {
              /* Unregistration callback. */

              altcomcallbacks_unreg_cb(APICMDID_SET_REP_EVT_LTIME);
              altcomstatus_unreg_statchgcb(repevt_ltime_status_chg_cb);
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
  g_lte_setrepltime_isproc = false;

  return ret;
}

/****************************************************************************
 * Name: lte_set_report_localtime
 *
 * Description:
 *   Change the report setting of the local time.
 *   The default report setting is disable.
 *
 * Input Parameters:
 *   localtime_callback Callback function to notify that local time.
 *                      If NULL is set, the report setting is disabled.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_report_localtime(localtime_report_cb_t localtime_callback)
{
  return altcom_set_report_localtime(localtime_callback);
}

/****************************************************************************
 * Name: apicmdhdlr_repevt
 *
 * Description:
 *   This function is an API command handler for event report.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_REPORT_EVT,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_repevt(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt, APICMDID_REPORT_EVT, repevt_job);
}
