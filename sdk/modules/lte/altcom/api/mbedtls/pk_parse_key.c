/****************************************************************************
 * modules/lte/altcom/api/mbedtls/pk_parse_key.c
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
#include "apicmd_pk_parse_key.h"
#include "apicmd_pk.h"
#include "apiutil.h"
#include "mbedtls/x509_crt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PK_PARSE_KEY_REQ_DATALEN (sizeof(struct apicmd_pk_parse_key_s))
#define PK_PARSE_KEY_RES_DATALEN (sizeof(struct apicmd_pk_parse_keyres_s))
#define PK_PARSE_KEY_REQ_DATALEN_V4 (APICMD_TLS_PK_CMD_DATA_SIZE + \
                                    sizeof(struct apicmd_pk_parse_key_v4_s))
#define PK_PARSE_KEY_RES_DATALEN_V4 (APICMD_TLS_PK_CMDRES_DATA_SIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pk_parse_key_req_s
{
  uint32_t   id;
  const unsigned char *key;
  uint32_t   keylen;
  const unsigned char *pwd;
  uint32_t   pwdlen;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t pk_parse_key_request(FAR struct pk_parse_key_req_s *req)
{
  int32_t  ret;
  uint16_t reslen = 0;
  uint32_t buflen = 0;
  FAR void *cmd = NULL;
  FAR void *res = NULL;
  int      protocolver = 0;
  uint16_t reqbuffsize = 0;
  uint16_t resbuffsize = 0;

  /* Set parameter from protocol version */

  protocolver = apicmdgw_get_protocolversion();

  if (protocolver == APICMD_VER_V1)
    {
      reqbuffsize = PK_PARSE_KEY_REQ_DATALEN;
      resbuffsize = PK_PARSE_KEY_RES_DATALEN;
    }
  else if (protocolver == APICMD_VER_V4)
    {
      reqbuffsize = PK_PARSE_KEY_REQ_DATALEN_V4;
      resbuffsize = PK_PARSE_KEY_RES_DATALEN_V4;
    }
  else
    {
      return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
    }

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, apicmdgw_get_cmdid(APICMDID_TLS_PK_PARSE_KEY),
    reqbuffsize, (FAR void **)&res, resbuffsize))
    {
      return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
    }

  /* Fill the data */

  if (protocolver == APICMD_VER_V1)
    {
      ((FAR struct apicmd_pk_parse_key_s *)cmd)->ctx = htonl(req->id);

      if (req->keylen <= APICMD_PK_PARSE_KEY_KEY_LEN)
        {
          buflen = req->keylen;
        }
      else
        {
          goto errout_with_cmdfree;
        }
      memset(((FAR struct apicmd_pk_parse_key_s *)cmd)->key, '\0',
        APICMD_PK_PARSE_KEY_KEY_LEN);
      if (req->key != NULL)
        {
          memcpy(((FAR struct apicmd_pk_parse_key_s *)cmd)->key, req->key,
            buflen);
        }
      ((FAR struct apicmd_pk_parse_key_s *)cmd)->keylen = htonl(buflen);

      if (req->pwdlen <= APICMD_PK_PARSE_KEY_PWD_LEN)
        {
          buflen = req->pwdlen;
        }
      else
        {
          goto errout_with_cmdfree;
        }
      memset(((FAR struct apicmd_pk_parse_key_s *)cmd)->pwd, '\0',
        APICMD_PK_PARSE_KEY_PWD_LEN);
      if (req->pwd != NULL)
        {
          memcpy(((FAR struct apicmd_pk_parse_key_s *)cmd)->pwd, req->pwd,
            buflen);
        }
      ((FAR struct apicmd_pk_parse_key_s *)cmd)->pwdlen = htonl(buflen);
    }
  else if (protocolver == APICMD_VER_V4)
    {
      ((FAR struct apicmd_pkcmd_s *)cmd)->ctx = htonl(req->id);
      ((FAR struct apicmd_pkcmd_s *)cmd)->subcmd_id =
        htonl(APISUBCMDID_TLS_PK_PARSE_KEY);
      if (req->keylen <= APICMD_PK_PARSE_KEY_KEY_LEN)
        {
          buflen = req->keylen;
        }
      else
        {
          goto errout_with_cmdfree;
        }
      memset(((FAR struct apicmd_pkcmd_s *)cmd)->u.parse_key.key, '\0',
        APICMD_PK_PARSE_KEY_KEY_LEN);
      if (req->key != NULL)
        {
          memcpy(((FAR struct apicmd_pkcmd_s *)cmd)->u.parse_key.key, req->key,
            buflen);
        }
      ((FAR struct apicmd_pkcmd_s *)cmd)->u.parse_key.keylen = htonl(buflen);

      if (req->pwdlen <= APICMD_PK_PARSE_KEY_PWD_LEN)
        {
          buflen = req->pwdlen;
        }
      else
        {
          goto errout_with_cmdfree;
        }
      memset(((FAR struct apicmd_pkcmd_s *)cmd)->u.parse_key.pwd, '\0',
        APICMD_PK_PARSE_KEY_PWD_LEN);
      if (req->pwd != NULL)
        {
          memcpy(((FAR struct apicmd_pkcmd_s *)cmd)->u.parse_key.pwd, req->pwd,
            buflen);
        }
      ((FAR struct apicmd_pkcmd_s *)cmd)->u.parse_key.pwdlen = htonl(buflen);
    }

  DBGIF_LOG1_DEBUG("[pk_parse_key]ctx id: %d\n", req->id);
  DBGIF_LOG1_DEBUG("[pk_parse_key]key: %s\n", req->key);
  DBGIF_LOG1_DEBUG("[pk_parse_key]keylen: %d\n", req->keylen);
  DBGIF_LOG1_DEBUG("[pk_parse_key]pwd: %s\n", req->pwd);
  DBGIF_LOG1_DEBUG("[pk_parse_key]pwdlen: %d\n", req->pwdlen);

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
      ret = ntohl(((FAR struct apicmd_pk_parse_keyres_s *)res)->ret_code);
    }
  else if (protocolver == APICMD_VER_V4)
    {
      ret = ntohl(((FAR struct apicmd_pkcmdres_s *)res)->ret_code);
    }

  DBGIF_LOG1_DEBUG("[pk_parse_key res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/


int mbedtls_pk_parse_key(mbedtls_pk_context *ctx,
                         const unsigned char *key, size_t keylen,
                         const unsigned char *pwd, size_t pwdlen)
{
  int32_t                   result;
  struct pk_parse_key_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return MBEDTLS_ERR_PK_BAD_INPUT_DATA;
    }

  req.id = ctx->id;
  req.key = key;
  req.keylen = keylen;
  req.pwd = pwd;
  req.pwdlen = pwdlen;

  result = pk_parse_key_request(&req);

  return result;
}

