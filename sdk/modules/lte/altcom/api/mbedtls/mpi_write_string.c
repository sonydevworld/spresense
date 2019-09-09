/****************************************************************************
 * modules/lte/altcom/api/mbedtls/mpi_write_string.c
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
#include "apicmd_mpi_write_string.h"
#include "apiutil.h"
#include "mbedtls/bignum.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPI_WRITE_STRING_REQ_DATALEN (sizeof(struct apicmd_mpi_write_string_s))
#define MPI_WRITE_STRING_RES_DATALEN (sizeof(struct apicmd_mpi_write_stringres_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mpi_write_string_req_s
{
  uint32_t   id;
  uint32_t   radix;
  uint32_t   buflen;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t mpi_write_string_request(FAR struct mpi_write_string_req_s *req,
                                        char *buf, size_t *olen)
{
  int32_t                                 ret;
  uint16_t                                reslen = 0;
  uint32_t                                out_len = 0;
  FAR struct apicmd_mpi_write_string_s    *cmd = NULL;
  FAR struct apicmd_mpi_write_stringres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_mbedtls_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_TLS_MPI_WRITE_STRING,
    MPI_WRITE_STRING_REQ_DATALEN,
    (FAR void **)&res, MPI_WRITE_STRING_RES_DATALEN))
    {
      return MBEDTLS_ERR_MPI_BAD_INPUT_DATA;
    }

  /* Fill the data */

  cmd->ctx = htonl(req->id);
  cmd->radix = htonl(req->radix);
  if (req->buflen > APICMD_MPI_WRITE_STRING_MPI_LEN)
    {
      goto errout_with_cmdfree;
    }
  else
    {
      cmd->buflen = htonl(req->buflen);
    }

  DBGIF_LOG1_DEBUG("[mpi_write_string]ctx id: %d\n", req->id);
  DBGIF_LOG1_DEBUG("[mpi_write_string]radix: %d\n", req->radix);
  DBGIF_LOG1_DEBUG("[mpi_write_string]buflen: %d\n", req->buflen);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      MPI_WRITE_STRING_RES_DATALEN, &reslen,
                      SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      goto errout_with_cmdfree;
    }

  if (reslen != MPI_WRITE_STRING_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);
  out_len = ntohl(res->olen);
  memcpy(buf, res->buf, out_len);
  *olen = out_len;

  DBGIF_LOG1_DEBUG("[mpi_write_string res]ret: %d\n", ret);

  altcom_mbedtls_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_mbedtls_free_cmdandresbuff(cmd, res);
  return MBEDTLS_ERR_MPI_BAD_INPUT_DATA;
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/


int mbedtls_mpi_write_string(const mbedtls_mpi *X, int radix,
                             char *buf, size_t buflen, size_t *olen)
{
  int32_t                       result;
  struct mpi_write_string_req_s req;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return MBEDTLS_ERR_MPI_BAD_INPUT_DATA;
    }

  req.id = X->id;
  req.radix = radix;
  req.buflen = buflen;

  result = mpi_write_string_request(&req, buf, olen);

  return result;
}

