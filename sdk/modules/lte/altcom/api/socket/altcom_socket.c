/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_socket.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
#include <stdbool.h>

#include "dbg_if.h"
#include "altcom_sock.h"
#include "altcom_socket.h"
#include "altcom_seterrno.h"
#include "apicmd_socket.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SOCKET_REQ_DATALEN (sizeof(struct apicmd_socket_s))
#define SOCKET_RES_DATALEN (sizeof(struct apicmd_socketres_s))
#define SOKCET_REQ_FAILURE  -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct socket_req_s
{
  int domain;
  int type;
  int protocol;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: socket_request
 ****************************************************************************/

static int32_t socket_request(FAR struct socket_req_s *req)
{
  int32_t                       ret;
  int32_t                       err;
  uint16_t                      reslen = 0;
  FAR struct apicmd_socket_s    *cmd = NULL;
  FAR struct apicmd_socketres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_sock_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_SOCK_SOCKET, SOCKET_REQ_DATALEN,
    (FAR void **)&res, SOCKET_RES_DATALEN))
    {
      return SOKCET_REQ_FAILURE;
    }

  /* Fill the data */

  cmd->domain   = htonl(req->domain);
  cmd->type     = htonl(req->type);
  cmd->protocol = htonl(req->protocol);

  DBGIF_LOG3_DEBUG("[socket-req]domain: %d, type: %d, protocol: %d\n", req->domain, req->type, req->protocol);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                       SOCKET_RES_DATALEN, &reslen, SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      err = -ret;
      goto errout_with_cmdfree;
    }

  if (reslen != SOCKET_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = ALTCOM_EFAULT;
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);
  err = ntohl(res->err_code);

  DBGIF_LOG2_DEBUG("[socket-res]ret: %d, err: %d\n", ret, err);

  if (APICMD_SOCKET_RES_RET_CODE_ERR == ret)
    {
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
      goto errout_with_cmdfree;
    }

  altcom_sock_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_sock_free_cmdandresbuff(cmd, res);
  altcom_seterrno(err);
  return SOKCET_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_socket
 ****************************************************************************/

int altcom_socket(int domain, int type, int protocol)
{
  int32_t                    ret;
  int32_t                    result;
  FAR struct altcom_socket_s *fsock;
  struct socket_req_s        req;

  /* Check Lte library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      altcom_seterrno(-ret);
      return -1;
    }

  req.domain   = domain;
  req.type     = type;
  req.protocol = protocol;

  result = socket_request(&req);

  if (result == SOKCET_REQ_FAILURE)
    {
      return -1;
    }
  else
    {
      fsock = altcom_sockfd_socket(result);

      DBGIF_ASSERT(fsock != NULL, "altcom socket is NULL\n");

      memset(fsock, 0, sizeof(struct altcom_socket_s));
    }

  return result;
}
