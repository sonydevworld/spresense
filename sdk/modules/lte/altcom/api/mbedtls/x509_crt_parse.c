/****************************************************************************
 * modules/lte/altcom/api/mbedtls/x509_crt_parse.c
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
#include "apicmd_x509_crt_parse.h"
#include "apicmd_x509_crt.h"
#include "apiutil.h"
#include "mbedtls/x509_crt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define X509_CRT_PARSE_REQ_DATALEN (sizeof(struct apicmd_x509_crt_parse_s))
#define X509_CRT_PARSE_RES_DATALEN (sizeof(struct apicmd_x509_crt_parseres_s))
#define X509_CRT_PARSE_REQ_DATALEN_V4 (APICMD_TLS_X509_CRT_CMD_DATA_SIZE + \
                                      sizeof(struct apicmd_x509_crt_parse_v4_s))
#define X509_CRT_PARSE_RES_DATALEN_V4 (APICMD_TLS_X509_CRT_CMDRES_DATA_SIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct x509_crt_parse_req_s
{
  uint32_t            id;
  const unsigned char *buf;
  uint32_t            buflen;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t x509_crt_parse_request(FAR struct x509_crt_parse_req_s *req)
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
      reqbuffsize = X509_CRT_PARSE_REQ_DATALEN;
      resbuffsize = X509_CRT_PARSE_RES_DATALEN;
    }
  else if (protocolver == APICMD_VER_V4)
    {
      reqbuffsize = X509_CRT_PARSE_REQ_DATALEN_V4;
      resbuffsize = X509_CRT_PARSE_RES_DATALEN_V4;
    }
  else
    {
      return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
    }

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, apicmdgw_get_cmdid(APICMDID_TLS_X509_CRT_PARSE),
    reqbuffsize, (FAR void **)&res, resbuffsize))
    {
      return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
    }

  /* Fill the data */

  if (protocolver == APICMD_VER_V1)
    {
      ((FAR struct apicmd_x509_crt_parse_s *)cmd)->chain = htonl(req->id);
    }
  else if (protocolver == APICMD_VER_V4)
    {
      ((FAR struct apicmd_x509_crtcmd_s *)cmd)->crt = htonl(req->id);
      ((FAR struct apicmd_x509_crtcmd_s *)cmd)->subcmd_id =
        htonl(APISUBCMDID_TLS_X509_CRT_PARSE);
    }

  if (req->buflen <= APICMD_X509_CRT_PARSE_BUF_LEN)
    {
      buflen = req->buflen;
    }
  else
    {
      goto errout_with_cmdfree;
    }

  if (protocolver == APICMD_VER_V1)
    {
      memset(((FAR struct apicmd_x509_crt_parse_s *)cmd)->buf, '\0',
        APICMD_X509_CRT_PARSE_BUF_LEN);
      if (req->buf != NULL)
        {
          memcpy(((FAR struct apicmd_x509_crt_parse_s *)cmd)->buf, req->buf,
            buflen);
        }
      else
        {
          goto errout_with_cmdfree;
        }
      ((FAR struct apicmd_x509_crt_parse_s *)cmd)->buflen = htonl(buflen);
    }
  else if (protocolver == APICMD_VER_V4)
    {
      memset(((FAR struct apicmd_x509_crtcmd_s *)cmd)->u.parse.buf, '\0',
        APICMD_X509_CRT_PARSE_BUF_LEN);
      if (req->buf != NULL)
        {
          memcpy(((FAR struct apicmd_x509_crtcmd_s *)cmd)->u.parse.buf,
            req->buf, buflen);
        }
      else
        {
          goto errout_with_cmdfree;
        }
      ((FAR struct apicmd_x509_crtcmd_s *)cmd)->u.parse.buflen =
        htonl(buflen);
    }

  DBGIF_LOG1_DEBUG("[x509_crt_parse]ctx id: %d\n", req->id);
  DBGIF_LOG1_DEBUG("[x509_crt_parse]buflen: %d\n", buflen);

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
      ret = ntohl(((FAR struct apicmd_x509_crt_parseres_s *)res)->ret_code);
    }
  else if (protocolver == APICMD_VER_V4)
    {
      ret = ntohl(((FAR struct apicmd_x509_crtcmdres_s *)res)->ret_code);
    }

  DBGIF_LOG1_DEBUG("[x509_crt_parse res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mbedtls_x509_crt_parse(mbedtls_x509_crt *chain, const unsigned char *buf,
                               size_t buflen)
{
  int32_t                     result;
  struct x509_crt_parse_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
    }

  req.id = chain->id;
  req.buf = buf;
  req.buflen = buflen;

  result = x509_crt_parse_request(&req);

  return result;
}

