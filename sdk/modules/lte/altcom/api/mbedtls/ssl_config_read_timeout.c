/****************************************************************************
 * modules/lte/altcom/api/mbedtls/ssl_config_read_timeout.c
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
#include "apicmd_config_read_timeout.h"
#include "apiutil.h"
#include "mbedtls/ssl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_READ_TIMEOUT_REQ_DATALEN (sizeof(struct apicmd_config_read_timeout_s))
#define CONFIG_READ_TIMEOUT_RES_DATALEN (sizeof(struct apicmd_config_read_timeoutres_s))

#define CONFIG_READ_TIMEOUT_SUCCESS 0
#define CONFIG_READ_TIMEOUT_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct config_read_timeout_req_s
{
  uint32_t id;
  uint32_t timeout;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t config_read_timeout_request(FAR struct config_read_timeout_req_s *req)
{
  int32_t                                    ret;
  uint16_t                                   reslen = 0;
  FAR struct apicmd_config_read_timeout_s    *cmd = NULL;
  FAR struct apicmd_config_read_timeoutres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_CONFIG_READ_TIMEOUT,
    CONFIG_READ_TIMEOUT_REQ_DATALEN,
    (FAR void **)&res, CONFIG_READ_TIMEOUT_RES_DATALEN))
    {
      return CONFIG_READ_TIMEOUT_FAILURE;
    }

  /* Fill the data */

  cmd->conf = htonl(req->id);
  cmd->timeout = htonl(req->timeout);

  DBGIF_LOG1_DEBUG("[config_read_timeout]config id: %d\n", req->id);
  DBGIF_LOG1_DEBUG("[config_read_timeout]config id: %d\n", req->timeout);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      CONFIG_READ_TIMEOUT_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != CONFIG_READ_TIMEOUT_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);

  DBGIF_LOG1_DEBUG("[config_read_timeout res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return CONFIG_READ_TIMEOUT_SUCCESS;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return CONFIG_READ_TIMEOUT_FAILURE;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mbedtls_ssl_conf_read_timeout(mbedtls_ssl_config *conf, uint32_t timeout)
{
  int32_t                          result;
  struct config_read_timeout_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return;
    }

  req.id = conf->id;
  req.timeout = timeout;

  result = config_read_timeout_request(&req);

  if (result != CONFIG_READ_TIMEOUT_SUCCESS)
    {
      DBGIF_LOG_ERROR("%s error.\n");
    }
}

