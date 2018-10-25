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
#include "apicmd_repquality.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QUALITY_DATA_LEN (sizeof(struct apicmd_cmddat_setrepquality_s))
#define QUALITY_SETRES_DATA_LEN \
  (sizeof(FAR struct apicmd_cmddat_setrepquality_res_s))
#define QUALITY_PERIOD_MIN (1)
#define QUALITY_PERIOD_MAX (4233600)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_lte_setrepquality_isproc = false;

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern quality_report_cb_t  g_quality_callback;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_set_report_quality
 *
 * Description:
 *   Change the report setting of the quality information.
 *   The default report setting is disable.
 *
 * Input Parameters:
 *   quality_callback Callback function to notify that quality.
 *                    If NULL is set, the report setting is disabled.
 *   period           Reporting cycle in sec (1-4233600).
 *   result_callback  Callback function to notify that report setting has
 *                    changed.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t lte_set_report_quality(quality_report_cb_t quality_callback,
                               uint32_t period)
{
  int32_t                                      ret        = 0;
  FAR struct apicmd_cmddat_setrepquality_s     *cmdbuff   = NULL;
  FAR struct apicmd_cmddat_setrepquality_res_s *resbuff   = NULL;
  uint16_t                                     resbufflen =
                                                 QUALITY_SETRES_DATA_LEN;
  uint16_t                                     reslen     = 0;

  /* Check if the library is initialized */

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      return -EPERM;
    }

  if (quality_callback)
    {
      if (QUALITY_PERIOD_MIN > period || QUALITY_PERIOD_MAX < period)
        {
          DBGIF_LOG_ERROR("Invalid parameter.\n");
          return -EINVAL;
        }
    }

  if (g_lte_setrepquality_isproc)
    {
      return -EBUSY;
    }

  g_lte_setrepquality_isproc = true;

  /* Accept the API */
  /* Allocate API command buffer to send */

  cmdbuff = (FAR struct apicmd_cmddat_setrepquality_s *)
    apicmdgw_cmd_allocbuff(APICMDID_SET_REP_QUALITY, QUALITY_DATA_LEN);
  if (!cmdbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      g_lte_setrepquality_isproc = false;
      return -ENOSPC;
    }
  else
    {
      resbuff = (FAR struct apicmd_cmddat_setrepquality_res_s *)
        BUFFPOOL_ALLOC(resbufflen);
      if (!resbuff)
        {
          DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
          altcom_free_cmd((FAR uint8_t *)cmdbuff);
          g_lte_setrepquality_isproc = false;
          return -ENOSPC;
        }

      /* Set event field */

      cmdbuff->enability = !quality_callback ?
        APICMD_SET_REPQUALITY_DISABLE :
        APICMD_SET_REPQUALITY_ENABLE;
      cmdbuff->interval = htonl(period);

      ret = apicmdgw_send((FAR uint8_t *)cmdbuff, (FAR uint8_t *)resbuff,
        resbufflen, &reslen, SYS_TIMEO_FEVR);
    }

  if (0 <= ret && resbufflen == reslen)
    {
      if (APICMD_SET_REPQUALITY_RES_OK == resbuff->result)
        {
          /* Register API callback */

          ALTCOM_CLR_CALLBACK(g_quality_callback);
          if (quality_callback)
            {
              ALTCOM_REG_CALLBACK(
                ret, g_quality_callback, quality_callback);
            }
        }
      else
        {
          DBGIF_LOG_ERROR("API command response is err.\n");
          ret= -EIO;
        }
    }

  if (0 <= ret)
    {
      ret = 0;
    }

  altcom_free_cmd((FAR uint8_t *)cmdbuff);
  (void)BUFFPOOL_FREE(resbuff);
  g_lte_setrepquality_isproc = false;

  return ret;
}
