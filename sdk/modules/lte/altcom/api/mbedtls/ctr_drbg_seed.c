/****************************************************************************
 * modules/lte/altcom/api/mbedtls/ctr_drbg_seed.c
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
#include "apicmd_ctr_drbg_seed.h"
#include "apiutil.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CTR_DRBG_SEED_REQ_DATALEN (sizeof(struct apicmd_ctr_drbg_seed_s))
#define CTR_DRBG_SEED_RES_DATALEN (sizeof(struct apicmd_ctr_drbg_seedres_s))

#define CTR_DRBG_SEED_SUCCESS 0
#define CTR_DRBG_SEED_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ctr_drbg_seed_req_s
{
  uint32_t             id;
  void                 *p_entropy;
  const unsigned char* custom;
  size_t               len;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t ctr_drbg_seed_request(FAR struct ctr_drbg_seed_req_s *req)
{
  int32_t                              ret;
  uint16_t                             reslen = 0;
  FAR struct apicmd_ctr_drbg_seed_s    *cmd = NULL;
  FAR struct apicmd_ctr_drbg_seedres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_CTR_DRBG_SEED, CTR_DRBG_SEED_REQ_DATALEN,
    (FAR void **)&res, CTR_DRBG_SEED_RES_DATALEN))
    {
      return MBEDTLS_ERR_CTR_DRBG_ENTROPY_SOURCE_FAILED;
    }

  /* Fill the data */

  cmd->ctx = htonl(req->id);
  DBGIF_LOG1_DEBUG("[ctr_drbg_seed]ctx id: %d\n", req->id);

  if (req->p_entropy != NULL)
    {
      mbedtls_entropy_context *ctx = (mbedtls_entropy_context*) req->p_entropy;
      cmd->p_entropy = htonl(ctx->id);
      DBGIF_LOG1_DEBUG("[ctr_drbg_seed]p_entropy: %d\n", ctx->id);
    }
  else
    {
      cmd->p_entropy = 0;
      DBGIF_LOG_DEBUG("[ctr_drbg_seed]p_entropy: 0\n");
    }

  memset(cmd->custom, 0, APICMD_CTR_DRBG_SEED_CUSTOM_LEN);
  memcpy(cmd->custom, req->custom, req->len);

  cmd->len = htonl(req->len);
  DBGIF_LOG1_DEBUG("[ctr_drbg_seed]len: %zu\n", req->len);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      CTR_DRBG_SEED_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != CTR_DRBG_SEED_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);

  DBGIF_LOG1_DEBUG("[ctr_drbg_seed res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MBEDTLS_ERR_CTR_DRBG_ENTROPY_SOURCE_FAILED;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/


int mbedtls_ctr_drbg_seed(mbedtls_ctr_drbg_context *ctx,
                          int (*f_entropy)(void *, unsigned char *, size_t),
                          void *p_entropy,
                          const unsigned char *custom,
                          size_t len)
{
  int32_t                    result;
  struct ctr_drbg_seed_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return MBEDTLS_ERR_CTR_DRBG_ENTROPY_SOURCE_FAILED;
    }

  req.id = ctx->id;
  req.p_entropy = p_entropy;
  req.custom = custom;
  req.len = len;

  result = ctr_drbg_seed_request(&req);

  return result;
}

