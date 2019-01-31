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
 * Public Data
 ****************************************************************************/

extern simstat_report_cb_t   g_simstat_report_callback;
extern localtime_report_cb_t g_localtime_report_callback;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  lte_localtime_t result;

  if (!g_localtime_report_callback)
    {
      DBGIF_LOG_DEBUG("g_localtime_report_callback is not registered.\n");
      return;
    }
  
  result.year   = ltime->year;
  result.mon    = ltime->month;
  result.mday   = ltime->day;
  result.hour   = ltime->hour;
  result.min    = ltime->minutes;
  result.sec    = ltime->seconds;
  result.tz_sec = ntohl(ltime->timezone);

  g_localtime_report_callback(&result);
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
  uint8_t result;

  if (!g_simstat_report_callback)
    {
      DBGIF_LOG_DEBUG("g_simstat_report_callback is not registered.\n");
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

  g_simstat_report_callback(result);
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
  uint8_t result;

  if (!g_simstat_report_callback)
    {
      DBGIF_LOG_DEBUG("g_simstat_report_callback is not registered.\n");
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
      default:
        {
          DBGIF_LOG1_ERROR("Unsupport SIM state. status:%d\n", simstate->state);
          return;
        }
    }

  g_simstat_report_callback(result);
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

  /* Check if the library is initialized */

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      return -EPERM;
    }

  /* Check this process runnning. */

  if (g_lte_setrepsimstat_isproc)
    {
      return -EBUSY;
    }

  g_lte_setrepsimstat_isproc = true;

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
          /* Register API callback */

          ALTCOM_CLR_CALLBACK(g_simstat_report_callback);
          if (simstat_callback)
            {
              ALTCOM_REG_CALLBACK(
                ret, g_simstat_report_callback, simstat_callback);
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
  int32_t                                 ret        = 0;
  FAR struct apicmd_cmddat_setrepevt_s    *cmdbuff   = NULL;
  FAR struct apicmd_cmddat_setrepevtres_s *resbuff   = NULL;
  uint16_t                                resbufflen = REPSETRES_DATA_LEN;
  uint16_t                                reslen     = 0;

  /* Check if the library is initialized */

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      return -EPERM;
    }

  /* Check this process runnning. */

  if (g_lte_setrepltime_isproc)
    {
      return -EBUSY;
    }

  g_lte_setrepltime_isproc = true;

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
          /* Register API callback */

          ALTCOM_CLR_CALLBACK(g_localtime_report_callback);
          if (localtime_callback)
            {
              ALTCOM_REG_CALLBACK(
                ret, g_localtime_report_callback, localtime_callback);
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
