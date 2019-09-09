/****************************************************************************
 * modules/lte/altcom/api/mbedtls/pk_info_from_type.c
 *
 *   Copyright 2018 Sony Corporation
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
#include "dbg_if.h"
#include "altcom_errno.h"
#include "altcom_seterrno.h"
#include "apicmd_pk_info_from_type.h"
#include "apiutil.h"
#include "mbedtls/pk.h"
#include "mbedtls/pk_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PK_INFO_FROM_TYPE_REQ_DATALEN (sizeof(struct apicmd_pk_info_from_type_s))
#define PK_INFO_FROM_TYPE_RES_DATALEN (sizeof(struct apicmd_pk_info_from_typeres_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pk_info_from_type_req_s
{
  uint32_t   pk_type;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mbedtls_pk_info_t g_pk_info = {0};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t pk_info_from_type_request(FAR struct pk_info_from_type_req_s *req)
{
  int32_t                                  ret;
  uint16_t                                 reslen = 0;
  FAR struct apicmd_pk_info_from_type_s    *cmd = NULL;
  FAR struct apicmd_pk_info_from_typeres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_PK_INFO_FROM_TYPE,
    PK_INFO_FROM_TYPE_REQ_DATALEN,
    (FAR void **)&res, PK_INFO_FROM_TYPE_RES_DATALEN))
    {
      return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
    }

  /* Fill the data */

  cmd->pk_type = htonl(req->pk_type);

  DBGIF_LOG1_DEBUG("[pk_info_from_type]pk_type id: %d\n", req->pk_type);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      PK_INFO_FROM_TYPE_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != PK_INFO_FROM_TYPE_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);
  if (ret == 0)
    {
      g_pk_info.id = ntohl(res->pk_info);
    } 

  DBGIF_LOG1_DEBUG("[pk_info_from_type res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

const mbedtls_pk_info_t *mbedtls_pk_info_from_type(mbedtls_pk_type_t pk_type)
{
  int32_t               result;
  struct pk_info_from_type_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return NULL;
    }

  req.pk_type = pk_type;

  result = pk_info_from_type_request(&req);

  if (result != 0)
    {
      return NULL;
    }
  return &g_pk_info;
}

