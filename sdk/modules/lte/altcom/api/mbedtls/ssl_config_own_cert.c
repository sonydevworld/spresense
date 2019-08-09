/****************************************************************************
 * modules/lte/altcom/api/mbedtls/ssl_config_own_cert.c
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
#include "apicmd_config_own_cert.h"
#include "apiutil.h"
#include "mbedtls/ssl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_OWN_CERT_REQ_DATALEN (sizeof(struct apicmd_config_own_cert_s))
#define CONFIG_OWN_CERT_RES_DATALEN (sizeof(struct apicmd_config_own_certres_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct config_own_cert_req_s
{
  uint32_t conf_id;
  uint32_t own_cert_id;
  uint32_t pk_key_id;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t config_own_cert_request(FAR struct config_own_cert_req_s *req)
{
  int32_t                                ret;
  uint16_t                               reslen = 0;
  FAR struct apicmd_config_own_cert_s    *cmd = NULL;
  FAR struct apicmd_config_own_certres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_CONFIG_OWN_CERT,
    CONFIG_OWN_CERT_REQ_DATALEN,
    (FAR void **)&res, CONFIG_OWN_CERT_RES_DATALEN))
    {
      return MBEDTLS_ERR_SSL_ALLOC_FAILED;
    }

  /* Fill the data */

  cmd->conf = htonl(req->conf_id);
  cmd->own_cert = htonl(req->own_cert_id);
  cmd->pk_key = htonl(req->pk_key_id);

  DBGIF_LOG1_DEBUG("[config_authmode]config id: %d\n", req->conf_id);
  DBGIF_LOG1_DEBUG("[config_authmode]own_cert: %d\n", req->own_cert_id);
  DBGIF_LOG1_DEBUG("[config_authmode]pk_key: %d\n", req->pk_key_id);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      CONFIG_OWN_CERT_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != CONFIG_OWN_CERT_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);

  DBGIF_LOG1_DEBUG("[config_own_cert res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MBEDTLS_ERR_SSL_ALLOC_FAILED;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mbedtls_ssl_conf_own_cert(mbedtls_ssl_config *conf,
                              mbedtls_x509_crt *own_cert,
                              mbedtls_pk_context *pk_key)
{
  int32_t                      result;
  struct config_own_cert_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return MBEDTLS_ERR_SSL_ALLOC_FAILED;
    }

  req.conf_id = conf->id;
  req.own_cert_id = (own_cert != NULL) ? own_cert->id : 0;
  req.pk_key_id = (pk_key != NULL) ? pk_key->id : 0;

  result = config_own_cert_request(&req);

  return result;
}

