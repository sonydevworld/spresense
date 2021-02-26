/****************************************************************************
 * modules/lte/altcom/api/lte/lte_rat.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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
#include "apicmd_rat.h"
#include "apiutil.h"
#include "apicmdhdlrbs.h"
#include "altcombs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LTE_SETRAT_DATA_LEN ( \
 sizeof(struct apicmd_cmddat_setrat_s))
 
#define LTE_SETRAT_RES_DATA_LEN ( \
 sizeof(struct apicmd_cmddat_setratres_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_get_rat_sync
 *
 * Description:
 *  Get RAT type.
 *
 * Returned Value:
 * On success, RAT type shown below is returned.
 * - LTE_RAT_CATM
 * - LTE_RAT_NBIOT
 * On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_get_rat_sync(void)
{
  lte_ratinfo_t ratinfo = {0};
  int32_t       ret;

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  ret = altcombs_get_ratinfo(&ratinfo);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("Failed to altcombs_get_ratinfo [%d].\n", ret);
    }
  else
    {
      ret = (int32_t)ratinfo.rat;
    }
  return ret;
}

/****************************************************************************
 * Name: lte_set_rat_sync
 *
 * Description:
 *  Set RAT setting.
 *
 * Returned Value:
 * On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_set_rat_sync(uint8_t rat, bool persistent)
{
  int32_t                            ret;
  uint16_t                           reslen = 0;
  FAR struct apicmd_cmddat_setrat_s *cmd;
  struct apicmd_cmddat_setratres_s   res    = {0};

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  /* Check ALTCOM protocol version */

  if (apicmdgw_get_protocolversion() != APICMD_VER_V4)
    {
      return -ENOTSUP;
    }

  /* Return error if parameter is invalid */

  if (rat != LTE_RAT_CATM &&
      rat != LTE_RAT_NBIOT)
    {
      DBGIF_LOG1_ERROR("RAT type is invalid [%d].\n", rat);
      return -EINVAL;
    }

  if (persistent != LTE_ENABLE &&
      persistent != LTE_DISABLE)
    {
      DBGIF_LOG1_ERROR("persistent is invalid [%d].\n", persistent);
      return -EINVAL;
    }

  cmd = (FAR struct apicmd_cmddat_setrat_s *)
        apicmdgw_cmd_allocbuff(APICMDID_SET_RAT,
                               LTE_SETRAT_DATA_LEN);
  if (!cmd)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      return -ENOMEM;
    }

  cmd->rat = (uint8_t)rat;
  cmd->persistency = (uint8_t)persistent;

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)&res,
                      LTE_SETRAT_RES_DATA_LEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("Failed to apicmdgw_send [%d].\n", ret);
    }
  else if (0 == reslen)
    {
      DBGIF_LOG_ERROR("Modem does not support this API.\n");
      ret = -ENOTSUP;
    }
  else
    {
      ret = ntohl(res.result);

      if (0 > ret)
        {
          DBGIF_LOG1_ERROR("Modem returned an error [%d].\n", ret);
        }
    }

  altcom_free_cmd((FAR uint8_t *)cmd);
  return ret;
}

/****************************************************************************
 * Name: lte_get_ratinfo_sync
 *
 * Description:
 *  Get RAT information.
 *
 * Returned Value:
 * On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_get_ratinfo_sync(lte_ratinfo_t *info)
{
  int32_t ret;

  if (!info)
    {
      DBGIF_LOG_ERROR("info parameter is NULL.\n");
      return -EINVAL;
    }

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      return ret;
    }

  ret = altcombs_get_ratinfo(info);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("Failed to altcombs_get_ratinfo [%d].\n", ret);
    }
  return ret;
}
