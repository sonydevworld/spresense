/****************************************************************************
 * modules/lte/altcom/api/mbedtls/ctr_drbg_seed.c
 *
 *   Copyright 2018 Sony Corporation
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

#include <string.h>
#include "dbg_if.h"
#include "altcom_errno.h"
#include "altcom_seterrno.h"
#include "apicmd_ctr_drbg_seed.h"
#include "apicmd_ctr_drbg.h"
#include "apiutil.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CTR_DRBG_SEED_REQ_DATALEN (sizeof(struct apicmd_ctr_drbg_seed_s))
#define CTR_DRBG_SEED_RES_DATALEN (sizeof(struct apicmd_ctr_drbg_seedres_s))
#define CTR_DRBG_SEED_REQ_DATALEN_V4 (APICMD_TLS_CTR_DRBG_CMD_DATA_SIZE + \
                                     sizeof(struct apicmd_ctr_drbg_seed_v4_s))
#define CTR_DRBG_SEED_RES_DATALEN_V4 (APICMD_TLS_CTR_DRBG_CMDRES_DATA_SIZE)

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
  int32_t  ret;
  uint16_t reslen = 0;
  FAR void *cmd = NULL;
  FAR void *res = NULL;
  int      protocolver = 0;
  uint16_t reqbuffsize = 0;
  uint16_t resbuffsize = 0;

  /* Set parameter from protocol version */

  protocolver = apicmdgw_get_protocolversion();

  if (protocolver == APICMD_VER_V1)
    {
      reqbuffsize = CTR_DRBG_SEED_REQ_DATALEN;
      resbuffsize = CTR_DRBG_SEED_RES_DATALEN;
    }
  else if (protocolver == APICMD_VER_V4)
    {
      reqbuffsize = CTR_DRBG_SEED_REQ_DATALEN_V4;
      resbuffsize = CTR_DRBG_SEED_RES_DATALEN_V4;
    }
  else
    {
      return MBEDTLS_ERR_CTR_DRBG_ENTROPY_SOURCE_FAILED;
    }

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, apicmdgw_get_cmdid(APICMDID_TLS_CTR_DRBG_SEED),
    reqbuffsize, (FAR void **)&res, resbuffsize))
    {
      return MBEDTLS_ERR_CTR_DRBG_ENTROPY_SOURCE_FAILED;
    }

  /* Fill the data */

 if (protocolver == APICMD_VER_V1)
    {
      ((FAR struct apicmd_ctr_drbg_seed_s *)cmd)->ctx = htonl(req->id);
      DBGIF_LOG1_DEBUG("[ctr_drbg_seed]ctx id: %d\n", req->id);

      if (req->p_entropy != NULL)
        {
          mbedtls_entropy_context *ctx =
            (mbedtls_entropy_context*) req->p_entropy;
          ((FAR struct apicmd_ctr_drbg_seed_s *)cmd)->p_entropy =
            htonl(ctx->id);
          DBGIF_LOG1_DEBUG("[ctr_drbg_seed]p_entropy: %d\n", ctx->id);
        }
      else
        {
          ((FAR struct apicmd_ctr_drbg_seed_s *)cmd)->p_entropy = 0;
          DBGIF_LOG_DEBUG("[ctr_drbg_seed]p_entropy: 0\n");
        }

      memset(((FAR struct apicmd_ctr_drbg_seed_s *)cmd)->custom, 0,
        APICMD_CTR_DRBG_SEED_CUSTOM_LEN);
      memcpy(((FAR struct apicmd_ctr_drbg_seed_s *)cmd)->custom, req->custom,
        req->len);

      ((FAR struct apicmd_ctr_drbg_seed_s *)cmd)->len = htonl(req->len);
    }
  else if (protocolver == APICMD_VER_V4)
    {
      ((FAR struct apicmd_ctr_drbgcmd_s *)cmd)->ctx = htonl(req->id);
      DBGIF_LOG1_DEBUG("[ctr_drbg_seed]ctx id: %d\n", req->id);

      ((FAR struct apicmd_ctr_drbgcmd_s *)cmd)->subcmd_id =
        htonl(APISUBCMDID_TLS_CTR_DRBG_SEED);

      if (req->p_entropy != NULL)
        {
          mbedtls_entropy_context *ctx =
            (mbedtls_entropy_context*)req->p_entropy;
          ((FAR struct apicmd_ctr_drbgcmd_s *)cmd)->u.seed.p_entropy =
            htonl(ctx->id);
          DBGIF_LOG1_DEBUG("[ctr_drbg_seed]p_entropy: %d\n", ctx->id);
        }
      else
        {
          ((FAR struct apicmd_ctr_drbgcmd_s *)cmd)->u.seed.p_entropy = 0;
          DBGIF_LOG_DEBUG("[ctr_drbg_seed]p_entropy: 0\n");
        }

      memset(((FAR struct apicmd_ctr_drbgcmd_s *)cmd)->u.seed.custom, 0,
        APICMD_CTR_DRBG_SEED_CUSTOM_LEN);
      memcpy(((FAR struct apicmd_ctr_drbgcmd_s *)cmd)->u.seed.custom,
        req->custom, req->len);

      ((FAR struct apicmd_ctr_drbgcmd_s *)cmd)->u.seed.len = htonl(req->len);
    }

  DBGIF_LOG1_DEBUG("[ctr_drbg_seed]len: %zu\n", req->len);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      resbuffsize, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != resbuffsize)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  if (protocolver == APICMD_VER_V1)
    {
      ret = ntohl(((FAR struct apicmd_ctr_drbg_seedres_s *)res)->ret_code);
    }
  else if (protocolver == APICMD_VER_V4)
    {
      ret = ntohl(((FAR struct apicmd_ctr_drbgcmdres_s *)res)->ret_code);
    }

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

