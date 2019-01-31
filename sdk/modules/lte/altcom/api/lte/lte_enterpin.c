/****************************************************************************
 * modules/lte/altcom/api/lte/lte_enterpin.c
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
#include "apicmd_enterpin.h"
#include "evthdlbs.h"
#include "apicmdhdlrbs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ENTERPIN_DATA_LEN (sizeof(struct apicmd_cmddat_enterpin_s))

#define ENTERPIN_MIN_PIN_LEN (4)
#define ENTERPIN_MAX_PIN_LEN ((APICMD_ENTERPIN_PINCODE_LEN) - 1)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern enter_pin_cb_t g_enterpin_callback;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: enterpin_job
 *
 * Description:
 *   This function is an API callback for enter PIN.
 *
 * Input Parameters:
 *  arg    Pointer to received event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void enterpin_job(FAR void *arg)
{
  int32_t                                ret;
  int32_t                                result;
  FAR struct apicmd_cmddat_enterpinres_s *data;
  enter_pin_cb_t                         callback;

  data = (FAR struct apicmd_cmddat_enterpinres_s *)arg;

  ALTCOM_GET_AND_CLR_CALLBACK(ret, g_enterpin_callback, callback);

  if ((ret == 0) && (callback))
    {
      result = (int32_t)data->result;

      callback(result, data->simstat, data->attemptsleft);
    }
  else
    {
      DBGIF_LOG_ERROR("Unexpected!! callback is NULL.\n");
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
 * Name: lte_enter_pin
 *
 * Description:
 *   Enter Personal Identification Number.
 *
 * Input Parameters:
 *   pincode       Current PIN code.
 *                 Specify 4 to 8 digits,end with '\0'. (i.e. Max 9 byte)
 *   new_pincode   If not used, set NULL. If the PIN is SIM PUK or SIM PUK2,
 *                 the new_pincode is required.
 *                 Specify 4 to 8 digits,end with '\0'. (i.e. Max 9 byte)
 *   callback      Callback function to notify that PIN enter is completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_enter_pin(int8_t *pincode, int8_t *new_pincode,
                      enter_pin_cb_t callback)
{
  int32_t                             ret;
  FAR struct apicmd_cmddat_enterpin_s *cmddat = NULL;
  uint8_t                             pinlen  = 0;

  /* Return error if argument is NULL */

  if (!pincode || !callback)
    {
      DBGIF_LOG_ERROR("Input argument is NULL.\n");
      return -EINVAL;
    }

  /* Check if the library is initialized */

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      return -EPERM;
    }
  else
    {
      pinlen = strlen((FAR char *)pincode);
      if (pinlen < ENTERPIN_MIN_PIN_LEN || ENTERPIN_MAX_PIN_LEN < pinlen)
        {
          return -EINVAL;
        }

      if (new_pincode)
        {
          pinlen = strlen((FAR char *)new_pincode);
          if (pinlen < ENTERPIN_MIN_PIN_LEN || ENTERPIN_MAX_PIN_LEN < pinlen)
            {
              return -EINVAL;
            }
        }

      /* Register API callback */

      ALTCOM_REG_CALLBACK(ret, g_enterpin_callback, callback);
      if (0 > ret)
        {
          DBGIF_LOG_ERROR("Currently API is busy.\n");
          return ret;
        }
    }

  /* Allocate API command buffer to send */

  cmddat = (FAR struct apicmd_cmddat_enterpin_s *)
    apicmdgw_cmd_allocbuff(APICMDID_ENTER_PIN, ENTERPIN_DATA_LEN);
  if (!cmddat)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
    }
  else
    {
      /* Get PIN input parameters */

      strncpy((FAR char *)cmddat->pincode,
        (FAR char *)pincode, sizeof(cmddat->pincode));

      if (new_pincode)
        {
          cmddat->newpincodeuse = APICMD_ENTERPIN_NEWPINCODE_USE;
          strncpy((FAR char *)cmddat->newpincode,
            (FAR char *)new_pincode, sizeof(cmddat->newpincode));
        }

      /* Send API command to modem */

      ret = altcom_send_and_free((FAR uint8_t *)cmddat);
    }

  /* If fail, there is no opportunity to execute the callback,
   * so clear it here. */

  if (0 > ret)
    {
      /* Clear registered callback */

      ALTCOM_CLR_CALLBACK(g_enterpin_callback);
    }
  else
    {
      ret = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: apicmdhdlr_enterpin
 *
 * Description:
 *   This function is an API command handler for enter PIN result.
 *
 * Input Parameters:
 *  evt    Pointer to received event.
 *  evlen  Length of received event.
 *
 * Returned Value:
 *   If the API command ID matches APICMDID_ENTER_PIN_RES,
 *   EVTHDLRC_STARTHANDLE is returned.
 *   Otherwise it returns EVTHDLRC_UNSUPPORTEDEVENT. If an internal error is
 *   detected, EVTHDLRC_INTERNALERROR is returned.
 *
 ****************************************************************************/

enum evthdlrc_e apicmdhdlr_enterpin(FAR uint8_t *evt, uint32_t evlen)
{
  return apicmdhdlrbs_do_runjob(evt,
    APICMDID_CONVERT_RES(APICMDID_ENTER_PIN), enterpin_job);
}
