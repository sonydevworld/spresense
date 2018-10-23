/****************************************************************************
 * modules/lte/altcom/api/lte/lte_getdataconfig.c
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
#include "apiutil.h"
#include "apicmd_getdataconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETDATACONFIG_DATA_LEN (sizeof(struct apicmd_cmddat_getdataconfig_s))
#define DATA_TYPE_MIN          LTE_DATA_TYPE_USER
#define DATA_TYPE_MAX          LTE_DATA_TYPE_IMS

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern get_dataconfig_cb_t g_getdataconfig_callback;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_get_dataconfig
 *
 * Description:
 *   Get Configuration of the data transfer. There are two types of data that
 *   can be specified: user data or IMS.
 *
 * Input Parameters:
 *   data_type Data type.
 *   callback  Callback function to notify that get configuration of the data
 *             transfer completed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_get_dataconfig(uint32_t data_type, get_dataconfig_cb_t callback)
{
  int32_t                                  ret;
  FAR struct apicmd_cmddat_getdataconfig_s *cmddat = NULL;

  /* Return error if callback is NULL */

  if (!callback)
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
  else if (data_type < DATA_TYPE_MIN || DATA_TYPE_MAX < data_type)
    {
      DBGIF_LOG1_ERROR("Unsupport data type. type:%d\n", data_type);
      return -EINVAL;
    }
  else
    {
      /* Register API callback */

      ALTCOM_REG_CALLBACK(ret, g_getdataconfig_callback, callback);
      if (0 > ret)
        {
          DBGIF_LOG_ERROR("Currently API is busy.\n");
          return ret;
        }
    }

  /* Allocate API command buffer to send */

  cmddat = (FAR struct apicmd_cmddat_getdataconfig_s *)
    apicmdgw_cmd_allocbuff(APICMDID_GET_DATACONFIG,
    GETDATACONFIG_DATA_LEN);
  if (!cmddat)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
    }
  else
    {

      if (LTE_DATA_TYPE_USER == data_type)
        {
          cmddat->type = APICMD_GETDATACONFIG_TYPE_USERDATA;
        }
      else
        {
          cmddat->type = APICMD_GETDATACONFIG_TYPE_IMS;
        }

      /* Send API command to modem */

      ret = altcom_send_and_free((FAR uint8_t *)cmddat);
    }

  /* If fail, there is no opportunity to execute the callback,
   * so clear it here. */

  if (0 > ret)
    {
      /* Clear registered callback */

      ALTCOM_CLR_CALLBACK(g_getdataconfig_callback);
    }
  else
    {
      ret = 0;
    }

  return ret;
}
