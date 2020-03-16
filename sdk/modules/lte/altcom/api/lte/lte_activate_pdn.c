/****************************************************************************
 * modules/lte/altcom/api/lte/lte_activate_pdn.c
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "lte/lte_api.h"
#include "buffpoolwrapper.h"
#include "evthdlbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"
#include "apicmdhdlrbs.h"
#include "apiutil.h"
#include "apicmd_activatepdn.h"
#include "lte_activatepdn.h"

#include "lte/altcom/altcom_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REQ_DATA_LEN (sizeof(struct apicmd_cmddat_activatepdn_s))
#define RES_DATA_LEN (sizeof(struct apicmd_cmddat_activatepdnres_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: activatepdn_check_apn
 *
 * Description:
 *   Check PDN setting parameter.
 *
 * Input Parameters:
 *   apn        APN setting.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t activatepdn_check_apn(lte_apn_setting_t *apn)
{
  int32_t mask = 0;

  if (!apn)
    {
      DBGIF_LOG_ERROR("apn is null.\n");
      return -EINVAL;
    }

  if ((!apn->apn) || (strlen((char *)apn->apn) >= LTE_APN_LEN))
    {
      DBGIF_LOG_ERROR("apn is length overflow.\n");
      return  -EINVAL;
    }

  if ((apn->ip_type < LTE_IPTYPE_V4) ||
      (apn->ip_type > LTE_IPTYPE_V4V6))
    {
      DBGIF_LOG1_ERROR("ip type is invalid. iptype=%d\n", apn->ip_type);
      return -EINVAL;
    }

  if ((apn->auth_type < LTE_APN_AUTHTYPE_NONE) ||
      (apn->auth_type > LTE_APN_AUTHTYPE_CHAP))
    {
      DBGIF_LOG1_ERROR("auth type is invalid. authtype=%d\n", apn->auth_type);
      return -EINVAL;
    }

  if (apn->user_name && apn->password)
    {
      if (strlen((char *)apn->user_name) >= LTE_APN_USER_NAME_LEN)
        {
          DBGIF_LOG_ERROR("username is length overflow.\n");
          return -EINVAL;
        }

      if (strlen((char *)apn->password) >= LTE_APN_PASSWD_LEN)
        {
          DBGIF_LOG_ERROR("password is length overflow.\n");
          return  -EINVAL;
        }
    }
  else
    {
      if (apn->auth_type != LTE_APN_AUTHTYPE_NONE)
        {
          DBGIF_LOG_ERROR("authentication infomation is invalid.\n");
          return -EINVAL;
        }
    }

  mask = (LTE_APN_TYPE_DEFAULT |
    LTE_APN_TYPE_MMS | LTE_APN_TYPE_SUPL | LTE_APN_TYPE_DUN |
    LTE_APN_TYPE_HIPRI | LTE_APN_TYPE_FOTA | LTE_APN_TYPE_IMS |
    LTE_APN_TYPE_CBS | LTE_APN_TYPE_IA | LTE_APN_TYPE_EMERGENCY);
  if (0 == (apn->apn_type & mask))
    {
      DBGIF_LOG2_ERROR("apn type is invalid. apntype=%08x / mask=%08x \n", apn->apn_type, mask);
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: activatepdn_status_chg_cb
 *
 * Description:
 *   Notification status change in processing activate pdn.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static int32_t activatepdn_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("activatepdn_status_chg_cb(%d -> %d)\n",
        old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_ACTIVATE_PDN);

      return ALTCOM_STATUS_REG_CLR;
    }

  return ALTCOM_STATUS_REG_KEEP;
}

/****************************************************************************
 * Name: activatepdn_job
 *
 * Description:
 *   This function is an API callback for activate pdn.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void activatepdn_job(FAR void *arg)
{
  int32_t                                   ret;
  FAR struct apicmd_cmddat_activatepdnres_s *data;
  activate_pdn_cb_t                         callback;
  uint32_t                                  result;
  FAR lte_pdn_t                             *pdn = NULL;

  data = (FAR struct apicmd_cmddat_activatepdnres_s *)arg;
  ret = altcomcallbacks_get_unreg_cb(APICMDID_ACTIVATE_PDN,
    (FAR void **)&callback);

  if ((0 != ret) || (!callback))
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
    }
  else
    {
      result =
        data->result == LTE_RESULT_OK ? LTE_RESULT_OK :
        data->result == LTE_RESULT_ERROR ? LTE_RESULT_ERROR :
        LTE_RESULT_CANCEL;

      if (result == LTE_RESULT_OK)
        {
          /* Fill pdn infomation */

          pdn = (FAR lte_pdn_t *)BUFFPOOL_ALLOC(sizeof(lte_pdn_t));
          if (pdn)
            {
              ret = altcombs_set_pdninfo(&data->pdnset, pdn);
              if (0 > ret)
                {
                  DBGIF_LOG1_ERROR("altcombs_conv_cmdpdn_to_ltepdn() error:%d", ret);
                  result = LTE_RESULT_ERROR;
                }
            }
          else
            {
              DBGIF_LOG_ERROR("Unexpected!! memory allocation failed.\n");
            }
        }

      callback(result, pdn);
    }

  /* In order to reduce the number of copies of the receive buffer,
   * bring a pointer to the receive buffer to the worker thread.
   * Therefore, the receive buffer needs to be released here. */

  if (pdn)
    {
      (void)BUFFPOOL_FREE(pdn);
    }

  altcom_free_cmd((FAR uint8_t *)arg);

  /* Unregistration status change callback. */

  altcomstatus_unreg_statchgcb(activatepdn_status_chg_cb);
}

/****************************************************************************
 * Name: lte_activatepdn_impl
 *
 * Description:
 *   Constructs a PDN with the specified APN settings.
 *
 * Input Parameters:
 *   apn        The pointer of the apn setting.
 *   pdn        The construction PDN information.
 *   callback   Callback function to notify that PDN activation completed.
 *              If the callback is NULL, operates with synchronous API,
 *              otherwise operates with asynchronous API.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

static int32_t lte_activatepdn_impl(lte_apn_setting_t *apn, lte_pdn_t *pdn,
                                    activate_pdn_cb_t callback)
{
  int32_t                                    ret;
  FAR struct apicmd_cmddat_activatepdn_s    *reqbuff    = NULL;
  FAR struct apicmd_cmddat_activatepdnres_s *presbuff   = NULL;
  uint16_t                                   resbufflen = RES_DATA_LEN;
  uint16_t                                   reslen     = 0;
  int                                        sync       = (callback == NULL);

  /* Check input parameter */

  if (!pdn && !callback)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
    }

  ret = activatepdn_check_apn(apn);
  if (0 > ret)
    {
      return ret;
    }

  /* Check LTE library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  if (sync)
    {
      /* Allocate API command buffer to receive */

      presbuff = (FAR struct apicmd_cmddat_activatepdnres_s *)
                   altcom_alloc_resbuff(resbufflen);
      if (!presbuff)
        {
          DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
          return -ENOMEM;
        }
    }
  else
    {
      /* Setup API callback */

      ret = altcombs_setup_apicallback(APICMDID_ACTIVATE_PDN, callback,
                                       activatepdn_status_chg_cb);
      if (0 > ret)
        {
          return ret;
        }
    }

  /* Allocate API command buffer to send */

  reqbuff = (FAR struct apicmd_cmddat_activatepdn_s *)
              apicmdgw_cmd_allocbuff(APICMDID_ACTIVATE_PDN, REQ_DATA_LEN);
  if (!reqbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
      goto errout;
    }

  reqbuff->apntype  = htonl(apn->apn_type);
  reqbuff->iptype   = apn->ip_type;
  reqbuff->authtype = apn->auth_type;
  memcpy(reqbuff->apnname, apn->apn, strlen((FAR char *)apn->apn));
  memcpy(reqbuff->username,
         apn->user_name, strlen((FAR char *)apn->user_name));
  memcpy((FAR void *)reqbuff->password,
         (FAR char *)apn->password, strlen((FAR char *)apn->password));

  /* Send API command to modem */

  ret = apicmdgw_send((FAR uint8_t *)reqbuff, (FAR uint8_t *)presbuff,
                      resbufflen, &reslen, SYS_TIMEO_FEVR);
  altcom_free_cmd((FAR uint8_t *)reqbuff);

  if (0 > ret)
    {
      goto errout;
    }

  ret = 0;

  if (sync)
    {
      if (LTE_RESULT_OK == presbuff->result)
        {
          /* Parse PDN information */

          altcombs_set_pdninfo(&presbuff->pdnset, pdn);
        }
      else
        {
          ret = (LTE_RESULT_CANCEL == presbuff->result) ? -ECANCELED :
                                                          -EPROTO;
        }
      BUFFPOOL_FREE(presbuff);
    }

  return ret;

errout:
  if (!sync)
    {
      altcombs_teardown_apicallback(APICMDID_ACTIVATE_PDN,
                                    activatepdn_status_chg_cb);
    }
  if (presbuff)
    {
      BUFFPOOL_FREE(presbuff);
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_activate_pdn_sync
 *
 * Description:
 *   Constructs a PDN with the specified APN settings.
 *
 * Input Parameters:
 *   apn        The pointer of the apn setting.
 *   pdn        The construction PDN information.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_activate_pdn_sync(lte_apn_setting_t *apn, lte_pdn_t *pdn)
{
  return altcom_activate_pdn_sync(apn, pdn);
}

/****************************************************************************
 * Name: altcom_activate_pdn_sync
 *
 * Description:
 *   Constructs a PDN with the specified APN settings.
 *
 * Input Parameters:
 *   apn        The pointer of the apn setting.
 *   pdn        The construction PDN information.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t altcom_activate_pdn_sync(lte_apn_setting_t *apn, lte_pdn_t *pdn)
{
  return lte_activatepdn_impl(apn, pdn, NULL);
}

/****************************************************************************
 * Name: lte_activate_pdn
 *
 * Description:
 *   Constructs a PDN with the specified APN settings.
 *
 * Input Parameters:
 *   apn        The pointer of the apn setting.
 *   callback   Callback function to notify that PDN activation completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_activate_pdn(lte_apn_setting_t *apn, activate_pdn_cb_t callback)
{
  if (!callback) {
    DBGIF_LOG_ERROR("Input argument is NULL.\n");
    return -EINVAL;
  }
  return lte_activatepdn_impl(apn, NULL, callback);
}

/****************************************************************************
 * Name: apicmdhdlr_activatepdn
 *
 * Description:
 *   This function is an API command handler for activate pdn result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_ACTIVATEPDN_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_activatepdn(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_CONVERT_RES(APICMDID_ACTIVATE_PDN), activatepdn_job);
}
