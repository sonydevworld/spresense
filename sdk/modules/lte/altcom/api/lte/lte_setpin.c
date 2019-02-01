/****************************************************************************
 * modules/lte/altcom/api/lte/lte_setpin.c
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
#include "apicmd_setpinlock.h"
#include "apicmd_setpincode.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SETPIN_LOCK_DATA_LEN \
  (sizeof(struct apicmd_cmddat_setpinlock_s))
#define SETPIN_CODE_DATA_LEN \
  (sizeof(struct apicmd_cmddat_setpincode_s))

#define SETPIN_TARGETPIN_MIN LTE_TARGET_PIN
#define SETPIN_TARGETPIN_MAX LTE_TARGET_PIN2

#define SETPIN_MIN_PIN_LEN (4)
#define SETPIN_MAX_PIN_LEN ((APICMD_SETPINLOCK_PINCODE_LEN) - 1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setpin_lock_status_chg_cb
 *
 * Description:
 *   Notification status change in processing set PIN lock.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void setpin_lock_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("setpin_lock_status_chg_cb(%d -> %d)\n",
        new_stat, old_stat);
      altcomcallbacks_unreg_cb(APICMDID_SET_PIN_LOCK);
    }
}

/****************************************************************************
 * Name: setpin_code_status_chg_cb
 *
 * Description:
 *   Notification status change in processing set PIN code.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void setpin_code_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("setpin_code_status_chg_cb(%d -> %d)\n",
        new_stat, old_stat);
      altcomcallbacks_unreg_cb(APICMDID_SET_PIN_CODE);
    }
}

/****************************************************************************
 * Name: setpin_lock_job
 *
 * Description:
 *   This function is an API callback for set PIN lock.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void setpin_lock_job(FAR void *arg)
{
  int32_t                                  ret                = -1;
  set_pinenable_cb_t                       pinenable_callback = NULL;
  FAR struct apicmd_cmddat_setpinlockres_s *resdat            = NULL;

  resdat = (FAR struct apicmd_cmddat_setpinlockres_s *)arg;
  if (APICMD_SETPINLOCK_RES_OK > resdat->result ||
      APICMD_SETPINLOCK_RES_ERR < resdat->result)
    {
      DBGIF_ASSERT(NULL, "Invalid response.\n");
    }

  ret = altcomcallbacks_get_unreg_cb(APICMDID_SET_PIN_LOCK,
                                     (void **)&pinenable_callback);

  if (0 == ret && pinenable_callback)
    {
      pinenable_callback(resdat->result, resdat->attemptsleft);
    }
  else
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
    }

  altcom_free_cmd((FAR uint8_t *)arg);

  /* Unregistration status change callback. */

  altcomstatus_unreg_statchgcb((void *)setpin_lock_status_chg_cb);
}

/****************************************************************************
 * Name: setpin_code_job
 *
 * Description:
 *   This function is an API callback for set PIN code.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void setpin_code_job(FAR void *arg)
{
  int32_t                                  ret                = -1;
  change_pin_cb_t                          changepin_callback = NULL;
  FAR struct apicmd_cmddat_setpincoderes_s *resdat            = NULL;

  resdat = (FAR struct apicmd_cmddat_setpincoderes_s *)arg;
  if (APICMD_SETPINCODE_RES_OK > resdat->result ||
      APICMD_SETPINCODE_RES_ERR < resdat->result)
    {
      DBGIF_ASSERT(NULL, "Invalid response.\n");
    }

  ret = altcomcallbacks_get_unreg_cb(APICMDID_SET_PIN_CODE,
                                     (void **)&changepin_callback);

  if (0 == ret && changepin_callback)
    {
      changepin_callback(resdat->result, resdat->attemptsleft);
    }
  else
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
    }

  altcom_free_cmd((FAR uint8_t *)arg);

  /* Unregistration status change callback. */

  altcomstatus_unreg_statchgcb((void *)setpin_code_status_chg_cb);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_set_pinenable
 *
 * Description:
 *   Set Personal Identification Number enable.
 *
 * Input Parameters:
 *   enable    "Enable" or "Disable".
 *   pincode   Current PIN code.
 *             Specify 4 to 8 digits,end with '\0'. (i.e. Max 9 byte)
 *   callback  Callback function to notify that
 *             set of PIN settings is completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_pinenable(bool enable,
                          int8_t *pincode,
                          set_pinenable_cb_t callback)
{
  int32_t                               ret;
  FAR struct apicmd_cmddat_setpinlock_s *cmddat = NULL;
  uint8_t                               pinlen  = 0;

  /* Return error if argument is NULL */

  if (!pincode || !callback)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
    }

  pinlen = strlen((FAR char *)pincode);
  if (pinlen < SETPIN_MIN_PIN_LEN || SETPIN_MAX_PIN_LEN < pinlen)
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

  ret = altcomcallbacks_chk_reg_cb((void *)callback, APICMDID_SET_PIN_LOCK);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Currently API is busy.\n");
      return -EINPROGRESS;
    }

  ret = altcomstatus_reg_statchgcb((void *)setpin_lock_status_chg_cb);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
      altcomcallbacks_unreg_cb(APICMDID_SET_PIN_LOCK);
      return ret;
    }

  /* Allocate API command buffer to send */

  cmddat = (FAR struct apicmd_cmddat_setpinlock_s *)
    apicmdgw_cmd_allocbuff(APICMDID_SET_PIN_LOCK, SETPIN_LOCK_DATA_LEN);
  if (!cmddat)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      return -ENOSPC;
    }
  else
    {
      /* Get PIN settings parameters */

      cmddat->mode = enable;
      strncpy((FAR char *)cmddat->pincode,
        (FAR char *)pincode, sizeof(cmddat->pincode));

      /* Send API command to modem */

      ret = altcom_send_and_free((FAR uint8_t *)cmddat);
    }

  /* If fail, there is no opportunity to execute the callback,
   * so clear it here. */

  if (0 > ret)
    {
      /* Clear registered callback */

      altcomcallbacks_unreg_cb(APICMDID_SET_PIN_LOCK);
      altcomstatus_unreg_statchgcb((void *)setpin_lock_status_chg_cb);
    }
  else
    {
      ret = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: lte_change_pin
 *
 * Description:
 *   Change Personal Identification Number.
 *
 * Input Parameters:
 *   target_pin   Target of change PIN.
 *   pincode      Current PIN code.
 *                Specify 4 to 8 digits, end with '\0'. (i.e. Max 9 byte)
 *   new_pincode  New PIN code.
 *                Specify 4 to 8 digits, end with '\0'. (i.e. Max 9 byte)
 *   callback     Callback function to notify that
 *                change of PIN settings is completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_change_pin(int8_t target_pin, int8_t *pincode,
                       int8_t *new_pincode, change_pin_cb_t callback)
{
  int32_t                               ret;
  FAR struct apicmd_cmddat_setpincode_s *cmddat = NULL;
  uint8_t                               pinlen  = 0;

  /* Return error if argument is NULL */

  if (!pincode || !new_pincode || !callback)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
    }

  if (SETPIN_TARGETPIN_MIN > target_pin || SETPIN_TARGETPIN_MAX < target_pin)
    {
      DBGIF_LOG1_ERROR("Unsupport change type. type:%d\n", target_pin);
      return -EINVAL;
    }

  pinlen = strlen((FAR char *)pincode);
  if (pinlen < SETPIN_MIN_PIN_LEN || SETPIN_MAX_PIN_LEN < pinlen)
    {
      return -EINVAL;
    }

  pinlen = strlen((FAR char *)new_pincode);
  if (pinlen < SETPIN_MIN_PIN_LEN || SETPIN_MAX_PIN_LEN < pinlen)
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

  ret = altcomcallbacks_chk_reg_cb((void *)callback, APICMDID_SET_PIN_CODE);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Currently API is busy.\n");
      return -EINPROGRESS;
    }

  ret = altcomstatus_reg_statchgcb((void *)setpin_code_status_chg_cb);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
      altcomcallbacks_unreg_cb(APICMDID_SET_PIN_CODE);
      return ret;
    }

  /* Allocate API command buffer to send */

  cmddat = (FAR struct apicmd_cmddat_setpincode_s *)
    apicmdgw_cmd_allocbuff(APICMDID_SET_PIN_CODE, SETPIN_CODE_DATA_LEN);
  if (!cmddat)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOSPC;
    }
  else
    {
      /* Get PIN settings parameters */

      if (LTE_TARGET_PIN == target_pin)
        {
          cmddat->chgtype = APICMD_SETPINCODE_CHGTYPE_PIN;
        }
      else
        {
          cmddat->chgtype = APICMD_SETPINCODE_CHGTYPE_PIN2;
        }

      strncpy((FAR char *)cmddat->pincode,
        (FAR char *)pincode, sizeof(cmddat->pincode));
      strncpy((FAR char *)cmddat->newpincode,
        (FAR char *)new_pincode, sizeof(cmddat->newpincode));

      /* Send API command to modem */

      ret = altcom_send_and_free((FAR uint8_t *)cmddat);
    }

  /* If fail, there is no opportunity to execute the callback,
   * so clear it here. */

  if (0 > ret)
    {
      /* Clear registered callback */

      altcomcallbacks_unreg_cb(APICMDID_SET_PIN_CODE);
      altcomstatus_unreg_statchgcb((void *)setpin_code_status_chg_cb);
    }
  else
    {
      ret = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: apicmdhdlr_setpin
 *
 * Description:
 *   This function is an API command handler for set PIN set result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_SET_PIN_LOCK_RES or
 *   APICMDID_SET_PIN_CODE_RES, EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_setpin(FAR uint8_t *evt, uint32_t evlen)
{
  enum evthdlrc_e ret;

  ret = apicmdhdlrbs_do_runjob(
    evt, APICMDID_CONVERT_RES(APICMDID_SET_PIN_LOCK), setpin_lock_job);
  if (EVTHDLRC_UNSUPPORTEDEVENT == ret)
    {
      ret = apicmdhdlrbs_do_runjob(
        evt, APICMDID_CONVERT_RES(APICMDID_SET_PIN_CODE), setpin_code_job);
    }

  return ret;
}
