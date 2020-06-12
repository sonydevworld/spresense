/****************************************************************************
 * modules/lte/altcom/api/lte/lte_set_rep_restart.c
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
#include "altcombs.h"
#include "apicmd.h"
#include "apiutil.h"
#include "altcom_status.h"
#include "altcom_callbacks.h"
#include "lte_report_restart.h"

#include "lte/lte_daemon.h"
#include "lte/altcom/altcom_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t g_lte_set_represtart_reason = LTE_RESTART_USER_INITIATED;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_set_report_restart
 *
 * Description:
 *   Registration Modem restart notification callback.
 *
 * Input Parameters:
 *   restart_callback Callback function to notify that modem restart.
 *                    Stop report restart at set null this parameter.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_report_restart(restart_report_cb_t restart_callback)
{
  int32_t ret;

  ret = lte_daemon_set_cb(restart_callback);

  return ret;
}

/****************************************************************************
 * Name: lte_set_report_reason
 *
 * Description:
 *   Set restart reason.
 *
 * Input Parameters:
 *   reason      Modem restart reason.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_report_reason(int32_t reason)
{
  int32_t ret = 0;

  g_lte_set_represtart_reason = reason;

  return ret;
}

/****************************************************************************
 * Name: lte_do_restartcallback
 *
 * Description:
 *   Call registered modem restart callback function.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_do_restartcallback()
{
  int32_t             ret = 0;
  restart_report_cb_t callback = NULL;

  callback = altcomcallbacks_get_cb(APICMDID_REPORT_RESTART);

  if (!callback)
    {
      DBGIF_LOG_INFO("restart callback is NULL.\n");
      return ret;
    }

  callback(g_lte_set_represtart_reason);
  return 0;
}

/****************************************************************************
 * Name: altcom_set_report_restart
 *
 * Description:
 *   Registration Modem restart notification callback.
 *
 * Input Parameters:
 *   restart_callback Callback function to notify that modem restart.
 *                    Stop report restart at set null this parameter.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t altcom_set_report_restart(restart_report_cb_t restart_callback)
{

  /* Check Lte library status */

  if (ALTCOM_STATUS_UNINITIALIZED == altcom_get_status())
    {
      return -EOPNOTSUPP;
    }

  /* Regist or Unregist report callback */

  if (restart_callback)
    {
      altcomcallbacks_reg_cb(restart_callback, APICMDID_REPORT_RESTART);
    }
  else
    {
      altcomcallbacks_unreg_cb(APICMDID_REPORT_RESTART);
    }
  return 0;
}
