/****************************************************************************
 * modules/lte/altcom/api/mbedtls/md.c
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
#include "apicmd_md.h"
#include "apiutil.h"
#include "mbedtls/md.h"
#include "mbedtls/md_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MD_REQ_DATALEN (sizeof(struct apicmd_md_s))
#define MD_RES_DATALEN (sizeof(struct apicmd_mdres_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct md_req_s
{
  uint32_t            id;
  const unsigned char *input;
  size_t              ilen;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t md_request(FAR struct md_req_s *req,
                          unsigned char *output)
{
  int32_t                   ret;
  uint16_t                  reslen = 0;
  FAR struct apicmd_md_s    *cmd = NULL;
  FAR struct apicmd_mdres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_MD, MD_REQ_DATALEN,
    (FAR void **)&res, MD_RES_DATALEN))
    {
      return MBEDTLS_ERR_MD_ALLOC_FAILED;
    }

  /* Fill the data */

  cmd->md_info = htonl(req->id);
  if (req->ilen <= APICMD_MD_INPUT_LEN)
    {
      memset(cmd->input, 0, APICMD_MD_INPUT_LEN);
      memcpy(cmd->input, req->input, req->ilen);
    }
  else
    {
      ret = MBEDTLS_ERR_MD_BAD_INPUT_DATA;
      goto errout_with_cmdfree;
    }

  cmd->ilen = htonl(req->ilen);

  DBGIF_LOG1_DEBUG("[md]md_info id: %d\n", req->id);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      MD_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != MD_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      ret = MBEDTLS_ERR_MD_FILE_IO_ERROR;
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);
  memcpy(output, res->output, APICMD_MD_OUTPUT_LEN);

  DBGIF_LOG1_DEBUG("[md res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return ret;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mbedtls_md(const mbedtls_md_info_t *md_info, const unsigned char *input,
               size_t ilen, unsigned char *output)
{
  struct md_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return 0;
    }

  req.id = md_info->id;
  req.input = input;
  req.ilen = ilen;

  return md_request(&req, output);
}

