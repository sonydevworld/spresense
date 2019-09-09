/****************************************************************************
 * modules/lte/altcom/api/mbedtls/pk_parse_keyfile.c
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

#include <string.h>
#include "dbg_if.h"
#include "altcom_errno.h"
#include "altcom_seterrno.h"
#include "apicmd_pk_parse_key.h"
#include "apicmd_pk_parse_keyfile.h"
#include "apiutil.h"
#include "mbedtls_file_wrapper.h"
#include "mbedtls/x509_crt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PK_PARSE_KEYFILE_REQ_DATALEN (sizeof(struct apicmd_pk_parse_keyfile_s))
#define PK_PARSE_KEYFILE_RES_DATALEN (sizeof(struct apicmd_pk_parse_keyfileres_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pk_parse_keyfile_req_s
{
  uint32_t   id;
  const char *path;
  const char *pwd;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t pk_parse_keyfile_request(FAR struct pk_parse_keyfile_req_s *req)
{
  int32_t                                 ret;
  uint16_t                                reslen = 0;
  FAR struct apicmd_pk_parse_keyfile_s    *cmd = NULL;
  FAR struct apicmd_pk_parse_keyfileres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_PK_PARSE_KEYFILE,
    PK_PARSE_KEYFILE_REQ_DATALEN,
    (FAR void **)&res, PK_PARSE_KEYFILE_RES_DATALEN))
    {
      return MBEDTLS_ERR_PK_FILE_IO_ERROR;
    }

  /* Fill the data */

  cmd->ctx = htonl(req->id);
  memset(cmd->path, '\0', APICMD_PK_PARSE_KEYFILE_PATH_LEN);
  if (req->path != NULL)
    {
      strncpy((char*) cmd->path, req->path, APICMD_PK_PARSE_KEYFILE_PATH_LEN-1);
    }

  memset(cmd->pwd, '\0', APICMD_PK_PARSE_KEYFILE_PWD_LEN);
  if (req->pwd != NULL)
    {
      strncpy((char*) cmd->pwd, req->pwd, APICMD_PK_PARSE_KEYFILE_PWD_LEN-1);
    }

  DBGIF_LOG1_DEBUG("[pk_parse_keyfile]ctx id: %d\n", req->id);
  DBGIF_LOG1_DEBUG("[pk_parse_keyfile]path: %s\n", cmd->path);
  DBGIF_LOG1_DEBUG("[pk_parse_keyfile]pwd: %s\n", cmd->pwd);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      PK_PARSE_KEYFILE_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != PK_PARSE_KEYFILE_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);

  DBGIF_LOG1_DEBUG("[pk_parse_keyfile res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MBEDTLS_ERR_PK_FILE_IO_ERROR;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/


int mbedtls_pk_parse_keyfile(mbedtls_pk_context *ctx,
                             const char *path, const char *password)
{
  int32_t                       result;
  struct pk_parse_keyfile_req_s req;
  unsigned char                 *parse_buf = NULL;
  size_t                        parse_len;


  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return MBEDTLS_ERR_PK_FILE_IO_ERROR;
    }

  result = mbedtls_load_local_file(path, &parse_buf, &parse_len);
  if (result == 0)
    {
      if (password == NULL)
        {
          result = mbedtls_pk_parse_key(ctx, parse_buf, parse_len, NULL, 0);
        }
      else
        {
          result = mbedtls_pk_parse_key(ctx, parse_buf, parse_len,
                      (const unsigned char*)password, strlen(password));
        }
      free(parse_buf);
    }
  else
    {
      req.id = ctx->id;
      req.path = path;
      req.pwd = password;

      result = pk_parse_keyfile_request(&req);
    }

  return result;
}

