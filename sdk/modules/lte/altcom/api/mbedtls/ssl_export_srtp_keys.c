/****************************************************************************
 * modules/lte/altcom/api/mbedtls/ssl_export_srtp_keys.c
 *
 *   Copyright (C) 2018 Sony Corporation
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
#include "apicmd_ssl_export_srtp_keys.h"
#include "apiutil.h"
#include "ctx_id_mgr.h"
#include "mbedtls/ssl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SSL_EXPORT_SRTP_KEYS_REQ_DATALEN (sizeof(struct apicmd_ssl_export_srtp_keys_s))
#define SSL_EXPORT_SRTP_KEYS_RES_DATALEN (sizeof(struct apicmd_ssl_export_srtp_keysres_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ssl_export_srtp_keys_req_s
{
  uint32_t id;
  uint32_t buf_size;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t ssl_export_srtp_keys_request(FAR struct ssl_export_srtp_keys_req_s *req,
                                            unsigned char *key_buffer)
{
  int32_t                                     ret;
  uint16_t                                    reslen = 0;
  uint32_t                                    buflen = 0;
  FAR struct apicmd_ssl_export_srtp_keys_s    *cmd = NULL;
  FAR struct apicmd_ssl_export_srtp_keysres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_SSL_EXPORT_SRTP_KEYS, SSL_EXPORT_SRTP_KEYS_REQ_DATALEN,
    (FAR void **)&res, SSL_EXPORT_SRTP_KEYS_RES_DATALEN))
    {
      return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    }

  /* Fill the data */

  cmd->ssl = htonl(req->id);

  DBGIF_LOG1_DEBUG("[ssl_export_srtp_keys]ctx id: %d\n", req->id);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      SSL_EXPORT_SRTP_KEYS_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != SSL_EXPORT_SRTP_KEYS_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);
  buflen = (req->buf_size < APICMD_EXPORT_SRTP_KEY_LEN) ? req->buf_size : APICMD_EXPORT_SRTP_KEY_LEN;
  memcpy(key_buffer, res->key, buflen);

  DBGIF_LOG1_DEBUG("[ssl_export_srtp_keys res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mbedtls_ssl_export_srtp_keys( mbedtls_ssl_context *ssl, uint8_t* key_buffer, uint16_t key_buffer_size )
{
  int32_t                           result;
  struct ssl_export_srtp_keys_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    }

  req.id = ssl->id;
  req.buf_size = key_buffer_size;

  result = ssl_export_srtp_keys_request(&req, key_buffer);

  return result;
}

