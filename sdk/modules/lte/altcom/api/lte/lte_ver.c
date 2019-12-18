/****************************************************************************
 * modules/lte/altcom/api/lte/lte_ver.c
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
#include "apicmd_ver.h"
#include "apiutil.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REQ_DATA_LEN (0)
#define RES_DATA_LEN (sizeof(struct apicmd_cmddat_getverres_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getver_status_chg_cb
 *
 * Description:
 *   Notification status change in processing get version.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static int32_t getver_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("getver_status_chg_cb(%d -> %d)\n",
        old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_GET_VERSION);

      return ALTCOM_STATUS_REG_CLR;
    }

  return ALTCOM_STATUS_REG_KEEP;
}

/****************************************************************************
 * Name: getver_parse_response
 *
 * Description:
 *   Parse version information from response buffer.
 *
 * Input Parameters:
 *  resp     Pointer to response buffer.
 *  siminfo  Pointer to store version information.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getver_parse_response(FAR struct apicmd_cmddat_getverres_s *resp,
                                  FAR lte_version_t *version)
{
  memset(version, 0, sizeof(version));
  strncpy((FAR char *)version->bb_product,
          (FAR const char *)resp->bb_product, LTE_VER_BB_PRODUCT_LEN - 1);
  strncpy((FAR char *)version->np_package,
          (FAR const char *)resp->np_package, LTE_VER_NP_PACKAGE_LEN - 1);
}

/****************************************************************************
 * Name: getver_job
 *
 * Description:
 *   This function is an API callback for get version.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void getver_job(FAR void *arg)
{
  int32_t                               ret;
  int32_t                               result;
  FAR struct apicmd_cmddat_getverres_s  *data =
    (FAR struct apicmd_cmddat_getverres_s *)arg;
  FAR lte_version_t                     *version = NULL;
  get_ver_cb_t                          callback;

  ret = altcomcallbacks_get_unreg_cb(APICMDID_GET_VERSION,
    (void **)&callback);

  if ((ret == 0) && (callback))
    {
      result = (int32_t)data->result;

      if (LTE_RESULT_OK == result)
        {
          version =
            (FAR lte_version_t *)BUFFPOOL_ALLOC(sizeof(lte_version_t));
          if (!version)
            {
              DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
              ret = -ENOMEM;
            }
          else
            {
              getver_parse_response(data, version);
            }
        }

      callback(result, version);

      if (NULL != version)
        {
          (void)BUFFPOOL_FREE((FAR void *)version);
        }
    }
  else
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
    }

  /* In order to reduce the number of copies of the receive buffer,
   * bring a pointer to the receive buffer to the worker thread.
   * Therefore, the receive buffer needs to be released here. */

  altcom_free_cmd((FAR void *)arg);

  /* Unregistration status change callback. */

  altcomstatus_unreg_statchgcb(getver_status_chg_cb);
}

/****************************************************************************
 * Name: lte_getversion_impl
 *
 * Description:
 *   Acquires the FW version information of the modem.
 *
 * Input Parameters:
 *   version  The version information of the modem.
 *   callback Callback function to notify when getting the version is
 *            completed.
 *            If the callback is NULL, operates with synchronous API,
 *            otherwise operates with asynchronous API.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

static int32_t lte_getversion_impl(lte_version_t *version,
                                   get_ver_cb_t callback)
{
  int32_t                               ret;
  FAR uint8_t                          *reqbuff    = NULL;
  FAR struct apicmd_cmddat_getverres_s *presbuff   = NULL;
  uint16_t                              resbufflen = RES_DATA_LEN;
  uint16_t                              reslen     = 0;
  int                                   sync       = (callback == NULL);

  /* Check input parameter */

  if (!version && !callback)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
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

      presbuff = (FAR struct apicmd_cmddat_getverres_s *)
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

      ret = altcombs_setup_apicallback(APICMDID_GET_VERSION, callback,
                                       getver_status_chg_cb);
      if (0 > ret)
        {
          return ret;
        }
    }

  /* Allocate API command buffer to send */

  reqbuff = (FAR uint8_t *)apicmdgw_cmd_allocbuff(APICMDID_GET_VERSION,
                                                  REQ_DATA_LEN);
  if (!reqbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Send API command to modem */

  ret = apicmdgw_send(reqbuff, (FAR uint8_t *)presbuff,
                      resbufflen, &reslen, SYS_TIMEO_FEVR);
  altcom_free_cmd(reqbuff);

  if (0 > ret)
    {
      goto errout;
    }

  ret = 0;

  if (sync)
    {
      ret = (LTE_RESULT_OK == presbuff->result) ? 0 : -EPROTO;
      if (ret == 0)
        {
          /* Parse version information */

          getver_parse_response(presbuff, version);
        }
      BUFFPOOL_FREE(presbuff);
    }

  return ret;

errout:
  if (!sync)
    {
      altcombs_teardown_apicallback(APICMDID_GET_VERSION,
                                    getver_status_chg_cb);
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
 * Name: lte_get_version_sync
 *
 * Description:
 *   Acquires the FW version information of the modem.
 *
 * Input Parameters:
 *   version  The version information of the modem.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_get_version_sync(lte_version_t *version)
{
  return lte_getversion_impl(version, NULL);
}

/****************************************************************************
 * Name: lte_get_version
 *
 * Description:
 *   Acquires the FW version information of the modem.
 *
 * Input Parameters:
 *   callback Callback function to notify when getting the version is
 *            completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_get_version(get_ver_cb_t callback)
{
  if (!callback) {
    DBGIF_LOG_ERROR("Input argument is NULL.\n");
    return -EINVAL;
  }
  return lte_getversion_impl(NULL, callback);
}

/****************************************************************************
 * Name: apicmdhdlr_ver
 *
 * Description:
 *   This function is an API command handler for version get result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_GET_VERSION_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_ver(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_CONVERT_RES(APICMDID_GET_VERSION), getver_job);
}
