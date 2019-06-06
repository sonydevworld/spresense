/****************************************************************************
 * modules/lte/altcom/api/lte/lte_setdataconfig.c
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
#include "apicmd_setdataconfig.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"
#include "altcom_callbacks.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SETDATACONFIG_DATA_LEN (sizeof(struct apicmd_cmddat_setdataconfig_s))
#define DATA_TYPE_MIN          LTE_DATA_TYPE_USER
#define DATA_TYPE_MAX          LTE_DATA_TYPE_IMS

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setdataconfig_status_chg_cb
 *
 * Description:
 *   Notification status change in processing set data connection
 *   configuration.
 *
 * Input Parameters:
 *  new_stat    Current status.
 *  old_stat    Preview status.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static int32_t setdataconfig_status_chg_cb(int32_t new_stat, int32_t old_stat)
{
  if (new_stat < ALTCOM_STATUS_POWER_ON)
    {
      DBGIF_LOG2_INFO("setdataconfig_status_chg_cb(%d -> %d)\n",
        old_stat, new_stat);
      altcomcallbacks_unreg_cb(APICMDID_SET_DATACONFIG);

      return ALTCOM_STATUS_REG_CLR;
    }

  return ALTCOM_STATUS_REG_KEEP;
}

/****************************************************************************
 * Name: setdataconfig_job
 *
 * Description:
 *   This function is an API callback for
 *   set data connection configuration.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void setdataconfig_job(FAR void *arg)
{
  int32_t                                     ret;
  int32_t                                     result;
  FAR struct apicmd_cmddat_setdataconfigres_s *data;
  set_dataconfig_cb_t                         callback;

  data = (FAR struct apicmd_cmddat_setdataconfigres_s *)arg;

  ret = altcomcallbacks_get_unreg_cb(APICMDID_SET_DATACONFIG,
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

  altcomstatus_unreg_statchgcb(setdataconfig_status_chg_cb);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_set_dataconfig
 *
 * Description:
 *   Change configuration of the data transfer. There are two types of data
 *   that can be specified: user data or IMS.
 *   Details are shown in the table below.
 *    -----------------------------------------------
 *    | general | roaming | data transfer           |
 *    -----------------------------------------------
 *    | disable | disable | Home    : Not available |
 *    |         |         | Roaming : Not available |
 *    -----------------------------------------------
 *    | disable | enable  | Home    : Not available |
 *    |         |         | Roaming : Not available |
 *    -----------------------------------------------
 *    | enable  | disable | Home    : Available     |
 *    |         |         | Roaming : Not available |
 *    -----------------------------------------------
 *    | enable  | enable  | Home    : Available     |
 *    |         |         | Roaming : Available     |
 *    -----------------------------------------------
 *
 * Input Parameters:
 *   data_type Data type
 *   general   Data transfer for general
 *   roaming   Data transfer for roaming
 *   callback  Callback function to notify that change configuration of the
 *             data transfer completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_dataconfig(uint32_t data_type, bool general, bool roaming,
                           set_dataconfig_cb_t callback)
{
  int32_t                                  ret;
  FAR struct apicmd_cmddat_setdataconfig_s *cmdbuff = NULL;

  /* Return error if callback is NULL */

  if (!callback)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
    }

  if (DATA_TYPE_MIN > data_type || DATA_TYPE_MAX < data_type)
    {
      DBGIF_LOG1_ERROR("Unsupport data type. type:%d\n", data_type);
      return -EINVAL;
    }

  /* Check Lte library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  /* Register API callback */

  ret = altcomcallbacks_chk_reg_cb((void *)callback, APICMDID_SET_DATACONFIG);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Currently API is busy.\n");
      return -EINPROGRESS;
    }

  ret = altcomstatus_reg_statchgcb(setdataconfig_status_chg_cb);
  if (0 > ret)
    {
      DBGIF_LOG_ERROR("Failed to registration status change callback.\n");
      altcomcallbacks_unreg_cb(APICMDID_SET_DATACONFIG);
      return ret;
    }

  /* Allocate API command buffer to send */

  cmdbuff = (struct apicmd_cmddat_setdataconfig_s *)
    apicmdgw_cmd_allocbuff(APICMDID_SET_DATACONFIG, SETDATACONFIG_DATA_LEN);
  if (!cmdbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
    }
  else
    {
      cmdbuff->type    = LTE_DATA_TYPE_USER == data_type ?
                         (uint8_t)LTE_DATA_TYPE_USER :
                         (uint8_t)LTE_DATA_TYPE_IMS;
      cmdbuff->general = LTE_ENABLE == general ?
                         (uint8_t)LTE_ENABLE :
                         (uint8_t)LTE_DISABLE;
      cmdbuff->roaming = LTE_ENABLE == roaming ?
                         (uint8_t)LTE_ENABLE :
                         (uint8_t)LTE_DISABLE;

      /* Send API command to modem */

      ret = altcom_send_and_free((FAR uint8_t *)cmdbuff);
    }

  /* If fail, there is no opportunity to execute the callback,
   * so clear it here. */

  if (0 > ret)
    {
      /* Clear registered callback */

      altcomcallbacks_unreg_cb(APICMDID_SET_DATACONFIG);
      altcomstatus_unreg_statchgcb(setdataconfig_status_chg_cb);
    }
  else
    {
      ret = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: apicmdhdlr_setdataconfig
 *
 * Description:
 *   This function is an API command handler for get data cnct cfg result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_SET_DATASET_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_setdataconfig(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_CONVERT_RES(APICMDID_SET_DATACONFIG), setdataconfig_job);
}
