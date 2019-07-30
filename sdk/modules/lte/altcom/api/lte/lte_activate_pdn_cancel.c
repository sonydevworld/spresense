/****************************************************************************
 * modules/lte/altcom/api/lte/lte_activate_pdn_cancel.c
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
#include "altcom_callbacks.h"
#include "altcombs.h"
#include "apicmdhdlrbs.h"
#include "apiutil.h"
#include "apicmd_activatepdn_cancel.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACTPDN_CANCEL_DATA_LEN (0)
#define ACTPDN_CANCEL_RES_DATA_LEN \
  (sizeof(struct apicmd_cmddat_activatepdn_cancel_res_s))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_lte_actpdncancel_isproc = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: actpdncancel_status_chg_cb
 *
 * Description:
 *   Notification status change in processing activate PDN cancel.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static int32_t actpdncancel_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("actpdncancel_status_chg_cb(%d -> %d)\n", old_stat, new_stat);
      return ALTCOM_STATUS_REG_CLR;
    }

  return ALTCOM_STATUS_REG_KEEP;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_activate_pdn_cancel
 *
 * Description:
 *   Activation PDN cancel.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_activate_pdn_cancel(void)
{
  int32_t                                        ret        = 0;
  FAR uint8_t                                   *cmdbuff    = NULL;
  struct apicmd_cmddat_activatepdn_cancel_res_s  resbuff;
  uint16_t                                       resbufflen = 0;
  uint16_t                                       reslen     = 0;

  resbufflen = ACTPDN_CANCEL_RES_DATA_LEN;

  /* Check LTE library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  /* Check this process runnning. */

  if (g_lte_actpdncancel_isproc)
    {
      return -EBUSY;
    }

  g_lte_actpdncancel_isproc = true;
  ret = altcomstatus_reg_statchgcb(actpdncancel_status_chg_cb);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
      g_lte_actpdncancel_isproc = false;
      return ret;
    }

  /*
   * Accept the API
   * Allocate API command buffer to send
   */

  cmdbuff = apicmdgw_cmd_allocbuff(APICMDID_ACTIVATE_PDN_CANCEL,
                                   ACTPDN_CANCEL_DATA_LEN);
  if (!cmdbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      g_lte_actpdncancel_isproc = false;
      altcomstatus_unreg_statchgcb(actpdncancel_status_chg_cb);
      return -ENOMEM;
    }

  memset(&resbuff, 0, sizeof(resbuff));

  /* Send API command to modem */

  ret = apicmdgw_send(cmdbuff, (FAR uint8_t *)&resbuff,
                      resbufflen, &reslen, SYS_TIMEO_FEVR);

  if (0 <= ret)
    {
      ret = 0;

      /* Register API callback */

      if (LTE_RESULT_OK != resbuff.result)
        {
          DBGIF_LOG_ERROR("API command response is err.\n");
          ret = -EIO;
        }
    }

  altcomstatus_unreg_statchgcb(actpdncancel_status_chg_cb);
  altcom_free_cmd(cmdbuff);
  g_lte_actpdncancel_isproc = false;

  return ret;
}
