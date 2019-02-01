/****************************************************************************
 * modules/lte/altcom/api/lte/lte_setapn.c
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
#include <stdint.h>
#include <errno.h>

#include "lte/lte_api.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "apicmd_setapn.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SETAPN_DATA_LEN (sizeof(struct apicmd_cmddat_setapn_s))

#define SETAPN_IPTYPE_MIN LTE_APN_IPTYPE_IP
#define SETAPN_IPTYPE_MAX LTE_APN_IPTYPE_IPV4V6

#define SETAPN_AUTHTYPE_MIN LTE_APN_AUTHTYPE_NONE
#define SETAPN_AUTHTYPE_MAX LTE_APN_AUTHTYPE_CHAP

#define SETAPN_GET_MAX_STR_LEN(len) ((len) - 1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setapn_status_chg_cb
 *
 * Description:
 *   Notification status change in processing set APN setting.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void setapn_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("detachnet_status_chg_cb(%d -> %d)\n",
        old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_SET_APN);
    }
}

/****************************************************************************
 * Name: setapn_job
 *
 * Description:
 *   This function is an API callback for set APN setting.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void setapn_job(FAR void *arg)
{
  int32_t                              ret;
  int32_t                              result;
  FAR struct apicmd_cmddat_setapnres_s *data;
  set_apn_cb_t                         callback;

  data = (FAR struct apicmd_cmddat_setapnres_s *)arg;

  ret = altcomcallbacks_get_unreg_cb(APICMDID_SET_APN,
    (void **)&callback);

  if ((ret == 0) && (callback))
    {
      result = (int32_t)data->result;

      callback(result);
    }
  else
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
    }

  /* In order to reduce the number of copies of the receive buffer,
   * bring a pointer to the receive buffer to the worker thread.
   * Therefore, the receive buffer needs to be released here. */

  altcom_free_cmd((FAR uint8_t *)arg);

  /* Unregistration status change callback. */

  altcomstatus_unreg_statchgcb((void *)setapn_status_chg_cb);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_set_apn
 *
 * Description:
 *   Set access point name settings.
 *
 * Input Parameters:
 *   session_id  The numeric value of the session ID. Set it in the range of
 *               LTE_SESSION_ID_MIN to LTE_SESSION_ID_MAX.
 *   apn         Character string of Access Point Name. The maximum string
 *               length is LTE_APN_LEN, end with '\0'.
 *   ip_type     Internet protocol type.
 *   auth_type   Authentication type.
 *   user_name   Character string of user name. The maximum string length is
 *               LTE_APN_USER_NAME_LEN, end with '\0'.
 *   password    Character string of password. The maximum string length is
 *               LTE_APN_PASSWD_LEN, end with '\0'.
 *   callback    Callback function to notify that set of APN setting
 *               completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_apn(uint8_t session_id, int8_t *apn, uint8_t ip_type,
                    uint8_t auth_type, int8_t *user_name, int8_t *password,
                    set_apn_cb_t callback)
{
  int32_t                           ret;
  FAR struct apicmd_cmddat_setapn_s *cmddat;

  /* Return error if callback is NULL */

  if (!apn || !user_name || !password || !callback)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
    }

  if (LTE_SESSION_ID_MIN > session_id  ||
     LTE_SESSION_ID_MAX < session_id)
    {
      return -EINVAL;
    }

  if (SETAPN_GET_MAX_STR_LEN(LTE_APN_LEN) < strlen((FAR char *)apn))
    {
      return -EINVAL;
    }

  if (SETAPN_IPTYPE_MIN > ip_type  || SETAPN_IPTYPE_MAX < ip_type)
    {
      return -EINVAL;
    }

  if (SETAPN_AUTHTYPE_MIN > auth_type ||
      SETAPN_AUTHTYPE_MAX < auth_type)
    {
      return -EINVAL;
    }

  if (SETAPN_GET_MAX_STR_LEN(LTE_APN_USER_NAME_LEN) <
      strlen((FAR char *)user_name))
    {
      return -EINVAL;
    }

  if (SETAPN_GET_MAX_STR_LEN(LTE_APN_PASSWD_LEN) <
      strlen((FAR char *)password))
    {
      return -EINVAL;
    }

  /* Check Lte library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  /* Register API callback */

  ret = altcomcallbacks_chk_reg_cb((void *)callback, APICMDID_SET_APN);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Currently API is busy.\n");
      return -EINPROGRESS;
    }

  ret = altcomstatus_reg_statchgcb((void *)setapn_status_chg_cb);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
      altcomcallbacks_unreg_cb(APICMDID_SET_APN);
      return ret;
    }

  /* Allocate API command buffer to send */

  cmddat = (FAR struct apicmd_cmddat_setapn_s *)
    apicmdgw_cmd_allocbuff(APICMDID_SET_APN, SETAPN_DATA_LEN);
  if (!cmddat)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
    }
  else
    {
      /* Get access point setting. */

      cmddat->session_id = session_id;
      strncpy((FAR char *)cmddat->apn,
        (FAR char *)apn, sizeof(cmddat->apn));
      cmddat->ip_type = ip_type;
      cmddat->auth_type = auth_type;
      strncpy((FAR char *)cmddat->user_name,
        (FAR char *)user_name, sizeof(cmddat->user_name));
      strncpy((FAR char *)cmddat->password,
        (FAR char *)password, sizeof(cmddat->password));

      /* Send API command to modem */

      ret = altcom_send_and_free((FAR uint8_t *)cmddat);
    }

  /* If fail, there is no opportunity to execute the callback,
   * so clear it here. */

  if (0 > ret)
    {
      /* Clear registered callback */

      altcomcallbacks_unreg_cb(APICMDID_SET_APN);
      altcomstatus_unreg_statchgcb((void *)setapn_status_chg_cb);
    }
  else
    {
      ret = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: apicmdhdlr_setapn
 *
 * Description:
 *   This function is an API command handler for set APN result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_SET_APN_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_setapn(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_CONVERT_RES(APICMDID_SET_APN), setapn_job);
}
