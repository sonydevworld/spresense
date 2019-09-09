/****************************************************************************
 * modules/lte/altcom/api/mbedtls/x509write_crt_version.c
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
#include "apicmd_x509write_crt_version.h"
#include "apiutil.h"
#include "mbedtls/x509_crt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define X509WRITE_CRT_VERSION_REQ_DATALEN (sizeof(struct apicmd_x509write_crt_version_s))
#define X509WRITE_CRT_VERSION_RES_DATALEN (sizeof(struct apicmd_x509write_crt_versionres_s))

#define X509WRITE_CRT_VERSION_SUCCESS 0
#define X509WRITE_CRT_VERSION_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct x509write_crt_version_req_s
{
  uint32_t   id;
  int        version;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t x509write_crt_version_request(FAR struct x509write_crt_version_req_s *req)
{
  int32_t                                      ret;
  uint16_t                                     reslen = 0;
  FAR struct apicmd_x509write_crt_version_s    *cmd = NULL;
  FAR struct apicmd_x509write_crt_versionres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_X509WRITE_CRT_VERSION,
    X509WRITE_CRT_VERSION_REQ_DATALEN,
    (FAR void **)&res, X509WRITE_CRT_VERSION_RES_DATALEN))
    {
      return X509WRITE_CRT_VERSION_FAILURE;
    }

  /* Fill the data */

  cmd->ctx = htonl(req->id);
  cmd->version = htonl(req->version);

  DBGIF_LOG1_DEBUG("[x509write_crt_version]ctx id: %d\n", req->id);
  DBGIF_LOG1_DEBUG("[x509write_crt_version]version: %d\n", req->version);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      X509WRITE_CRT_VERSION_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != X509WRITE_CRT_VERSION_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);

  DBGIF_LOG1_DEBUG("[x509write_crt_version res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return X509WRITE_CRT_VERSION_SUCCESS;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return X509WRITE_CRT_VERSION_FAILURE;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mbedtls_x509write_crt_set_version(mbedtls_x509write_cert *ctx, int version)
{
  int32_t                            result;
  struct x509write_crt_version_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return;
    }

  req.id = ctx->id;
  req.version = version;

  result = x509write_crt_version_request(&req);

  if (result != X509WRITE_CRT_VERSION_SUCCESS)
    {
      DBGIF_LOG_ERROR("%s error.\n");
    }
}

