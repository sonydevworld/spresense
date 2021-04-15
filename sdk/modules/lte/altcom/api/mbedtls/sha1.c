/****************************************************************************
 * modules/lte/altcom/api/mbedtls/sha1.c
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
#include "apicmd_sha1.h"
#include "apicmd_cipher.h"
#include "apiutil.h"
#include "mbedtls/sha1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SHA1_REQ_DATALEN (sizeof(struct apicmd_sha1_s))
#define SHA1_RES_DATALEN (sizeof(struct apicmd_sha1res_s))
#define SHA1_REQ_DATALEN_V4 (APICMD_TLS_CIPHER_CMD_DATA_SIZE + \
                            sizeof(struct apicmd_sha1_v4_s))
#define SHA1_RES_DATALEN_V4 (APICMD_TLS_CIPHER_CMDRES_DATA_SIZE + \
                            sizeof(struct apicmd_sha1res_v4_s))

#define SHA1_SUCCESS 0
#define SHA1_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sha1_req_s
{
  const unsigned char *input;
  size_t              ilen;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t sha1_request(FAR struct sha1_req_s *req,
                            unsigned char output[20])
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
      reqbuffsize = SHA1_REQ_DATALEN;
      resbuffsize = SHA1_RES_DATALEN;
    }
  else if (protocolver == APICMD_VER_V4)
    {
      reqbuffsize = SHA1_REQ_DATALEN_V4;
      resbuffsize = SHA1_RES_DATALEN_V4;
    }
  else
    {
      return SHA1_FAILURE;
    }


  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, apicmdgw_get_cmdid(APICMDID_TLS_SHA1), reqbuffsize,
    (FAR void **)&res, resbuffsize))
    {
      return SHA1_FAILURE;
    }

  /* Fill the data */
  if (req->ilen <= APICMD_SHA1_INPUT_LEN)
    {
     if (protocolver == APICMD_VER_V1)
        {
          memcpy(((FAR struct apicmd_sha1_s *)cmd)->input, req->input,
            req->ilen);
          ((FAR struct apicmd_sha1_s *)cmd)->ilen = htonl(req->ilen);
        }
      else if (protocolver == APICMD_VER_V4)
        {
          ((FAR struct apicmd_ciphercmd_s *)cmd)->subcmd_id =
            htonl(APISUBCMDID_TLS_SHA1);
          memcpy(((FAR struct apicmd_ciphercmd_s *)cmd)->u.sha1.input,
            req->input, req->ilen);
          ((FAR struct apicmd_ciphercmd_s *)cmd)->u.sha1.ilen =
            htonl(req->ilen);
        }
    }
  else
    {
      goto errout_with_cmdfree;
    }

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
      ret = ntohl(((FAR struct apicmd_sha1res_s *)res)->ret_code);

      memcpy(output, ((FAR struct apicmd_sha1res_s *)res)->output,
        APICMD_SHA1_OUTPUT_LEN);
    }
  else if (protocolver == APICMD_VER_V4)
    {
      ret = ntohl(((struct apicmd_ciphercmdres_s *)res)->ret_code);
      if (ntohl(((struct apicmd_ciphercmdres_s *)res)->subcmd_id) !=
        APISUBCMDID_TLS_SHA1)
        {
          DBGIF_LOG1_ERROR("Unexpected sub command id: %d\n",
            ntohl(((struct apicmd_ciphercmdres_s *)res)->subcmd_id));
          goto errout_with_cmdfree;
        }

      memcpy(output, ((struct apicmd_ciphercmdres_s *)res)->u.sha1res.output,
        APICMD_SHA1_OUTPUT_LEN);
    }

  DBGIF_LOG1_DEBUG("[sha1 res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return SHA1_FAILURE;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/


void mbedtls_sha1(const unsigned char *input, size_t ilen, unsigned char output[20])
{
  int32_t           result;
  struct sha1_req_s req;

  if (input == NULL || output == NULL)
    {
      return;
    }

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return;
    }

  req.input = input;
  req.ilen = ilen;

  result = sha1_request(&req, output);

  if (result != SHA1_SUCCESS)
    {
      DBGIF_LOG_ERROR("%s error.\n");
    }

  return;
}

