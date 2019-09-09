/****************************************************************************
 * modules/lte/altcom/api/mbedtls/ssl_config_defaults.c
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
#include "apicmd_config_defaults.h"
#include "apiutil.h"
#include "mbedtls/ssl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_DEFAULTS_REQ_DATALEN (sizeof(struct apicmd_config_defaults_s))
#define CONFIG_DEFAULTS_RES_DATALEN (sizeof(struct apicmd_config_defaultsres_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct config_defaults_req_s
{
  uint32_t id;
  int32_t  endpoint;
  int32_t  transport;
  int32_t  preset;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t config_defaults_request(FAR struct config_defaults_req_s *req)
{
  int32_t                                ret;
  uint16_t                               reslen = 0;
  FAR struct apicmd_config_defaults_s    *cmd = NULL;
  FAR struct apicmd_config_defaultsres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_CONFIG_DEFAULTS,
    CONFIG_DEFAULTS_REQ_DATALEN,
    (FAR void **)&res, CONFIG_DEFAULTS_RES_DATALEN))
    {
      return MBEDTLS_ERR_SSL_ALLOC_FAILED;
    }

  /* Fill the data */

  cmd->conf = htonl(req->id);
  cmd->endpoint = (int32_t) htonl(req->endpoint);
  cmd->transport = (int32_t) htonl(req->transport);
  cmd->preset = (int32_t) htonl(req->preset);

  DBGIF_LOG1_DEBUG("[config_defaults]config id: %d\n", req->id);
  DBGIF_LOG1_DEBUG("[config_defaults]endpoint: %d\n", req->endpoint);
  DBGIF_LOG1_DEBUG("[config_defaults]transport: %d\n", req->transport);
  DBGIF_LOG1_DEBUG("[config_defaults]preset: %d\n", req->preset);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      CONFIG_DEFAULTS_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != CONFIG_DEFAULTS_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);

  DBGIF_LOG1_DEBUG("[config_defaults res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MBEDTLS_ERR_SSL_ALLOC_FAILED;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mbedtls_ssl_config_defaults(mbedtls_ssl_config *conf,
                                int endpoint, int transport, int preset)
{
  int32_t                      result;
  struct config_defaults_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return MBEDTLS_ERR_SSL_ALLOC_FAILED;
    }

  req.id = conf->id;
  req.endpoint = endpoint;
  req.transport = transport;
  req.preset = preset;

  result = config_defaults_request(&req);

  return result;
}

