/****************************************************************************
 * modules/lte/altcom/api/mbedtls/md_get_size.c
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

#include "dbg_if.h"
#include "altcom_errno.h"
#include "altcom_seterrno.h"
#include "apicmd_md_get_size.h"
#include "apiutil.h"
#include "mbedtls/md.h"
#include "mbedtls/md_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MD_GET_SIZE_REQ_DATALEN (sizeof(struct apicmd_md_get_size_s))
#define MD_GET_SIZE_RES_DATALEN (sizeof(struct apicmd_md_get_sizeres_s))

#define MD_GET_SIZE_SUCCESS 0
#define MD_GET_SIZE_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct md_get_size_req_s
{
  uint32_t id;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t md_get_size_request(FAR struct md_get_size_req_s *req,
                                   uint8_t *md_size)
{
  int32_t                            ret;
  uint16_t                           reslen = 0;
  FAR struct apicmd_md_get_size_s    *cmd = NULL;
  FAR struct apicmd_md_get_sizeres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_MD_GET_SIZE, MD_GET_SIZE_REQ_DATALEN,
    (FAR void **)&res, MD_GET_SIZE_RES_DATALEN))
    {
      return MD_GET_SIZE_FAILURE;
    }

  /* Fill the data */

  cmd->md_info = htonl(req->id);

  DBGIF_LOG1_DEBUG("[md_get_size]md_info id: %d\n", req->id);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      MD_GET_SIZE_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != MD_GET_SIZE_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);
  *md_size = res->md_size;

  DBGIF_LOG1_DEBUG("[md_get_size res]ret: %d\n", ret);
  DBGIF_LOG1_DEBUG("[md_get_size res]md_size: %d\n", (int) res->md_size);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return MD_GET_SIZE_SUCCESS;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MD_GET_SIZE_FAILURE;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

unsigned char mbedtls_md_get_size(const mbedtls_md_info_t *md_info)
{
  int32_t               result;
  uint8_t               md_size = 0;
  struct md_get_size_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return 0;
    }

  req.id = md_info->id;

  result = md_get_size_request(&req, &md_size);

  if (result == MD_GET_SIZE_FAILURE)
    {
      return 0;
    }
  else
    {
      return md_size;
    }
}

