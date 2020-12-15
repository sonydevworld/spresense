/****************************************************************************
 * modules/lte/altcom/api/mbedtls/ssl_session_reset.c
 *
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

#include "dbg_if.h"
#include "altcom_errno.h"
#include "altcom_seterrno.h"
#include "apicmd_session.h"
#include "apiutil.h"
#include "mbedtls/ssl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SES_RESET_REQ_DATALEN_V4 (APICMD_TLS_SESSION_CMD_DATA_SIZE)
#define SES_RESET_RES_DATALEN_V4 (APICMD_TLS_SESSION_CMDRES_DATA_SIZE)

#define SES_RESET_SUCCESS 0
#define SES_RESET_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ses_reset_req_s
{
  uint32_t ssl_id;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t ssl_session_reset_request(FAR struct ses_reset_req_s *req)
{
  int32_t                             ret;
  uint16_t                            reslen = 0;
  FAR struct apicmd_sessioncmd_s      *cmd = NULL;
  FAR struct apicmd_sessioncmdres_s   *res = NULL;

  /* Check ALTCOM protocol version */

  if (apicmdgw_get_protocolversion() != APICMD_VER_V4)
    {
      return MBEDTLS_ERR_SSL_HW_ACCEL_FAILED;
    }

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_SESSION_CMD, SES_RESET_REQ_DATALEN_V4,
    (FAR void **)&res, SES_RESET_RES_DATALEN_V4))
    {
      return MBEDTLS_ERR_SSL_ALLOC_FAILED;
    }

  /* Fill the data */

  cmd->ssl = htonl(req->ssl_id);
  cmd->subcmd_id = htonl(APISUBCMDID_TLS_SESSION_RESET);

  DBGIF_LOG1_DEBUG("[session_reset]ssl id: %d\n", req->ssl_id);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      SES_RESET_RES_DATALEN_V4, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != SES_RESET_RES_DATALEN_V4)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);

  DBGIF_LOG1_DEBUG("[session_reset res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MBEDTLS_ERR_SSL_HW_ACCEL_FAILED;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mbedtls_ssl_session_reset(mbedtls_ssl_context *ssl)
{
  int32_t              result;
  struct ses_reset_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return MBEDTLS_ERR_SSL_BAD_INPUT_DATA;
    }

  req.ssl_id = ssl->id;

  result = ssl_session_reset_request(&req);

  return result;
}

