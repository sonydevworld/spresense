/****************************************************************************
 * modules/lte/altcom/api/lte/lte_getcellinfo.c
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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
#include "altcombs.h"
#include "apicmd_getcellinfo.h"
#include "apicmd_cellinfo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REQ_DATA_LEN (0)
#define RES_DATA_LEN_V4 (sizeof(struct apicmd_cmddat_getcellinfores_v4_s))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_get_cellinfo_sync
 *
 * Description:
 *   Get communication LTE network cell information.
 *
 * Input Parameters:
 *   cellinfo   LTE network cell information.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned according to <errno.h>.
 *
 ****************************************************************************/

int32_t lte_get_cellinfo_sync(lte_cellinfo_t *cellinfo)
{
  int32_t                                ret;
  FAR uint8_t                           *reqbuff    = NULL;
  FAR uint8_t                           *presbuff   = NULL;
  uint16_t                               resbufflen = 0;
  uint16_t                               reslen     = 0;
  uint16_t                               cmdid = 0;
  int                                    protocolver = 0;

  /* Check input parameter */

  if (!cellinfo)
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

  cmdid = apicmdgw_get_cmdid(APICMDID_GET_CELLINFO);
  if (cmdid == APICMDID_UNKNOWN)
    {
      return -ENETDOWN;
    }

  protocolver = apicmdgw_get_protocolversion();
  if (protocolver == APICMD_VER_V1)
    {
      return -EOPNOTSUPP;
    }
  else if (protocolver == APICMD_VER_V4)
    {
      resbufflen = RES_DATA_LEN_V4;
    }
  else
    {
      return -ENETDOWN;
    }

  /* Allocate API command buffer to receive */

  presbuff = altcom_alloc_resbuff(resbufflen);
  if (!presbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      return -ENOMEM;
    }

  /* Allocate API command buffer to send */

  reqbuff = (FAR uint8_t *)apicmdgw_cmd_allocbuff(cmdid,
                                                  REQ_DATA_LEN);
  if (!reqbuff)
    {
      DBGIF_LOG_ERROR("Failed to allocate command buffer.\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Send API command to modem */

  ret = apicmdgw_send(reqbuff, presbuff,
                      resbufflen, &reslen, SYS_TIMEO_FEVR);
  altcom_free_cmd(reqbuff);

  if (0 > ret)
    {
      goto errout;
    }

  /* Parse LTE network cell information */

  altcombs_set_cellinfo_v4(
    &((struct apicmd_cmddat_getcellinfores_v4_s *)presbuff)->cellinfo,
    cellinfo);

  ret = 0;

errout:
  BUFFPOOL_FREE(presbuff);
  return ret;
}
