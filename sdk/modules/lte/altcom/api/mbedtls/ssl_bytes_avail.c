/****************************************************************************
 * modules/lte/altcom/api/mbedtls/ssl_bytes_avail.c
 *
 *   Copyright 2019 Sony Corporation
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
#include "apicmd_ssl_bytes_avail.h"
#include "apicmd_ssl.h"
#include "apiutil.h"
#include "mbedtls/ssl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SSL_BYTES_AVAIL_REQ_DATALEN (sizeof(struct apicmd_ssl_bytes_avail_s))
#define SSL_BYTES_AVAIL_RES_DATALEN (sizeof(struct apicmd_ssl_bytes_availres_s))
#define SSL_BYTES_AVAIL_REQ_DATALEN_V4 (APICMD_TLS_SSL_CMD_DATA_SIZE)
#define SSL_BYTES_AVAIL_RES_DATALEN_V4 (APICMD_TLS_SSL_CMDRES_DATA_SIZE + \
                                       sizeof(struct apicmd_ssl_bytes_availres_v4_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ssl_bytes_avail_req_s
{
  uint32_t id;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static size_t ssl_bytes_avail_request(FAR struct ssl_bytes_avail_req_s *req)
{
  int32_t  ret;
  uint32_t avail_bytes = 0;
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
      reqbuffsize = SSL_BYTES_AVAIL_REQ_DATALEN;
      resbuffsize = SSL_BYTES_AVAIL_RES_DATALEN;
    }
  else if (protocolver == APICMD_VER_V4)
    {
      reqbuffsize = SSL_BYTES_AVAIL_REQ_DATALEN_V4;
      resbuffsize = SSL_BYTES_AVAIL_RES_DATALEN_V4;
    }
  else
    {
      return 0;
    }

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, apicmdgw_get_cmdid(APICMDID_TLS_SSL_BYTES_AVAIL),
    reqbuffsize, (FAR void **)&res, resbuffsize))
    {
      return 0;
    }

  /* Fill the data */

  if (protocolver == APICMD_VER_V1)
    {
      ((FAR struct apicmd_ssl_bytes_avail_s *)cmd)->ssl = htonl(req->id);
    }
  else if (protocolver == APICMD_VER_V4)
    {
      ((FAR struct apicmd_sslcmd_s *)cmd)->ssl = htonl(req->id);
      ((FAR struct apicmd_sslcmd_s *)cmd)->subcmd_id =
        htonl(APISUBCMDID_TLS_SSL_BYTES_AVAIL);
    }

  DBGIF_LOG1_DEBUG("[ssl_bytes_avail]id: %d\n", req->id);

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
      avail_bytes = ntohl(
        ((FAR struct apicmd_ssl_bytes_availres_s *)res)->avail_bytes);
    }
  else if (protocolver == APICMD_VER_V4)
    {
      if (ntohl(((FAR struct apicmd_sslcmdres_s *)res)->subcmd_id) !=
        APISUBCMDID_TLS_SSL_BYTES_AVAIL)
        {
          DBGIF_LOG1_ERROR("Unexpected sub command id: %d\n",
            ntohl(((FAR struct apicmd_sslcmdres_s *)res)->subcmd_id));
          goto errout_with_cmdfree;
        }

      avail_bytes = ntohl(
        ((FAR struct apicmd_sslcmdres_s *)res)->u.bytes_availres.avail_bytes);
    }

  DBGIF_LOG1_DEBUG("[ssl_bytes_avail res]avail_bytes: %d\n", avail_bytes);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return avail_bytes;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return 0;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t mbedtls_ssl_get_bytes_avail( const mbedtls_ssl_context *ssl )
{
  size_t                       avail_bytes = 0;
  struct ssl_bytes_avail_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return 0;
    }

  req.id = ssl->id;

  avail_bytes = ssl_bytes_avail_request(&req);

  return avail_bytes;
}

