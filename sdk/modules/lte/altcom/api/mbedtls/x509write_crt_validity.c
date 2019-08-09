/****************************************************************************
 * modules/lte/altcom/api/mbedtls/x509write_crt_validity.c
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
#include "apicmd_x509write_crt_validity.h"
#include "apiutil.h"
#include "mbedtls/x509_crt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define X509WRITE_CRT_VALIDITY_REQ_DATALEN (sizeof(struct apicmd_x509write_crt_validity_s))
#define X509WRITE_CRT_VALIDITY_RES_DATALEN (sizeof(struct apicmd_x509write_crt_validityres_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct x509write_crt_validity_req_s
{
  uint32_t   id;
  const char *not_before;
  const char *not_after;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t x509write_crt_validity_request(FAR struct x509write_crt_validity_req_s *req)
{
  int32_t                                           ret;
  int32_t                                           buflen = 0;
  uint16_t                                          reslen = 0;
  FAR struct apicmd_x509write_crt_validity_s    *cmd = NULL;
  FAR struct apicmd_x509write_crt_validityres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_X509WRITE_CRT_VALIDITY,
    X509WRITE_CRT_VALIDITY_REQ_DATALEN,
    (FAR void **)&res, X509WRITE_CRT_VALIDITY_RES_DATALEN))
    {
      return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
    }

  /* Fill the data */

  cmd->ctx = htonl(req->id);
  memset(cmd->not_before, '\0', APICMD_X509WRITE_CRT_VALIDITY_BUF_LEN);
  if (req->not_before != NULL)
    {
      buflen = strlen(req->not_before);
      if (buflen > APICMD_X509WRITE_CRT_VALIDITY_BUF_LEN)
        {
          goto errout_with_cmdfree;
        }
      memcpy(cmd->not_before, req->not_before, buflen);
    }
  else
    {
      goto errout_with_cmdfree;
    }

  memset(cmd->not_after, '\0', APICMD_X509WRITE_CRT_VALIDITY_BUF_LEN);
  if (req->not_after != NULL)
    {
      buflen = strlen(req->not_after);
      if (buflen > APICMD_X509WRITE_CRT_VALIDITY_BUF_LEN)
        {
          goto errout_with_cmdfree;
        }
      memcpy(cmd->not_after, req->not_after, buflen);
    }
  else
    {
      goto errout_with_cmdfree;
    }

  DBGIF_LOG1_DEBUG("[x509write_crt_validity]ctx id: %d\n", req->id);
  DBGIF_LOG1_DEBUG("[x509write_crt_validity]not_before: %s\n", req->not_before);
  DBGIF_LOG1_DEBUG("[x509write_crt_validity]not_after: %s\n", req->not_after);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      X509WRITE_CRT_VALIDITY_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != X509WRITE_CRT_VALIDITY_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);

  DBGIF_LOG1_DEBUG("[x509write_crt_validity res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mbedtls_x509write_crt_set_validity(mbedtls_x509write_cert *ctx,
                                       const char *not_before, const char *not_after)
{
  int32_t                             result;
  struct x509write_crt_validity_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
    }

  req.id = ctx->id;
  req.not_before = not_before;
  req.not_after = not_after;

  result = x509write_crt_validity_request(&req);

  return result;
}

