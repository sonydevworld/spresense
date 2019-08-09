/****************************************************************************
 * modules/lte/altcom/api/mbedtls/x509_csr_parse_file.c
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
#include "apicmd_x509_csr_parse.h"
#include "apicmd_x509_csr_parse_file.h"
#include "apiutil.h"
#include "mbedtls_file_wrapper.h"
#include "mbedtls/x509_csr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define X509_CSR_PARSE_FILE_REQ_DATALEN (sizeof(struct apicmd_x509_csr_parse_file_s))
#define X509_CSR_PARSE_FILE_RES_DATALEN (sizeof(struct apicmd_x509_csr_parse_fileres_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct x509_csr_parse_file_req_s
{
  uint32_t id;
  const char* path;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t x509_csr_parse_file_request(FAR struct x509_csr_parse_file_req_s *req)
{
  int32_t                                    ret;
  uint16_t                                   reslen = 0;
  FAR struct apicmd_x509_csr_parse_file_s    *cmd = NULL;
  FAR struct apicmd_x509_csr_parse_fileres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_X509_CSR_PARSE_FILE,
    X509_CSR_PARSE_FILE_REQ_DATALEN,
    (FAR void **)&res, X509_CSR_PARSE_FILE_RES_DATALEN))
    {
      return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
    }

  /* Fill the data */

  cmd->csr = htonl(req->id);
  memset(cmd->path, '\0', APICMD_X509_CSR_PARSE_FILE_PATH_LEN);
  if (req->path != NULL)
    {
      strncpy((char*) cmd->path, req->path, APICMD_X509_CSR_PARSE_FILE_PATH_LEN-1);
    }

  DBGIF_LOG1_DEBUG("[x509_csr_parse_file]ctx id: %d\n", req->id);
  DBGIF_LOG1_DEBUG("[x509_csr_parse_file]path: %s\n", cmd->path);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      X509_CSR_PARSE_FILE_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != X509_CSR_PARSE_FILE_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);

  DBGIF_LOG1_DEBUG("[x509_csr_parse_file res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mbedtls_x509_csr_parse_file(mbedtls_x509_csr *csr, const char *path)
{
  int32_t                          result;
  struct x509_csr_parse_file_req_s req;
  unsigned char                    *parse_buf = NULL;
  size_t                           parse_len;


  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return MBEDTLS_ERR_X509_BAD_INPUT_DATA;
    }

  result = mbedtls_load_local_file(path, &parse_buf, &parse_len);
  if (result == 0)
    {
      result = mbedtls_x509_csr_parse(csr, parse_buf, parse_len);
      free(parse_buf);
    }
  else
    {
      req.id = csr->id;
      req.path = path;

      result = x509_csr_parse_file_request(&req);
    }

  return result;
}

