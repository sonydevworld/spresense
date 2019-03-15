/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_connect.c
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

#include <stdbool.h>

#include "dbg_if.h"
#include "altcom_socket.h"
#include "altcom_select.h"
#include "altcom_sock.h"
#include "altcom_seterrno.h"
#include "apicmd_connect.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONNECT_REQ_DATALEN    (sizeof(struct apicmd_connect_s))
#define CONNECT_RES_DATALEN    (sizeof(struct apicmd_connectres_s))

#define CONNECT_REQ_SUCCESS    0
#define CONNECT_REQ_FAILURE    -1
#define CONNECT_REQ_INPROGRESS 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct connect_req_s
{
  int                              sockfd;
  FAR const struct altcom_sockaddr *addr;
  altcom_socklen_t                 addrlen;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: connect_request
 ****************************************************************************/

static int32_t connect_request(FAR struct altcom_socket_s *fsock,
                               FAR struct connect_req_s *req)
{
  int32_t                        ret;
  int32_t                        err;
  int32_t                        result;
  uint16_t                       reslen = 0;
  FAR struct apicmd_connect_s    *cmd = NULL;
  FAR struct apicmd_connectres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_sock_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_SOCK_CONNECT, CONNECT_REQ_DATALEN,
    (FAR void **)&res, CONNECT_RES_DATALEN))
    {
      return CONNECT_REQ_FAILURE;
    }

  /* Fill the data */

  altcom_sockaddr_to_sockstorage(req->addr, &cmd->name);

  cmd->sockfd  = htonl(req->sockfd);
  cmd->namelen = htonl(req->addrlen);

  DBGIF_LOG2_DEBUG("[connect-req]sockfd: %d, namelen: %d\n", req->sockfd, req->addrlen);
  DBGIF_LOG3_DEBUG("[connect-req]s2_len: %d, ss_family: %d, port: %d\n", cmd->name.s2_len, cmd->name.ss_family, *(FAR int16_t*)(cmd->name.s2_data1));

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      CONNECT_RES_DATALEN,
                      &reslen, SYS_TIMEO_FEVR);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      err = -ret;
      goto errout_with_cmdfree;
    }

  if (reslen != CONNECT_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = ALTCOM_EFAULT;
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);
  err = ntohl(res->err_code);

  DBGIF_LOG2_DEBUG("[connect-res]ret: %d, err: %d\n", ret, err);

  if (APICMD_CONNECT_RES_RET_CODE_ERR == ret)
    {
      /* When flags is non-blocking, return immediately */

      if (fsock->flags & ALTCOM_O_NONBLOCK)
        {
          goto errout_with_cmdfree;
        }

      if (err != ALTCOM_EINPROGRESS)
        {
          DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
          goto errout_with_cmdfree;
        }
      else
        {
          result = CONNECT_REQ_INPROGRESS;
        }
    }
  else
    {
      result = CONNECT_REQ_SUCCESS;
    }

  altcom_sock_free_cmdandresbuff(cmd, res);

  return result;

errout_with_cmdfree:
  altcom_sock_free_cmdandresbuff(cmd, res);
  altcom_seterrno(err);
  return CONNECT_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_connect
 ****************************************************************************/

int altcom_connect(int sockfd, const struct altcom_sockaddr *addr,
                   altcom_socklen_t addrlen)
{
  int32_t                    ret;
  int32_t                    result;
  int32_t                    val_len;
  FAR struct altcom_socket_s *fsock;
  struct altcom_fd_set_s     writeset;
  struct connect_req_s       req;

  /* Check Lte library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      altcom_seterrno(-ret);
      return -1;
    }

  fsock = altcom_sockfd_socket(sockfd);
  if (!fsock)
    {
      altcom_seterrno(ALTCOM_EINVAL);
      return -1;
    }

  req.sockfd  = sockfd;
  req.addr    = addr;
  req.addrlen = addrlen;

  /* Send connect request */

  result = connect_request(fsock, &req);

  if (result == CONNECT_REQ_INPROGRESS)
    {
      /* Wait until connect complete */

      ALTCOM_FD_ZERO(&writeset);
      ALTCOM_FD_SET(sockfd, &writeset);

      ret = altcom_select_block((sockfd + 1), NULL, &writeset, NULL, NULL);
      if (ret <= 0)
        {
          if (ret == 0)
            {
              altcom_seterrno(ALTCOM_EFAULT);
            }

          DBGIF_LOG1_ERROR("select failed: %d\n", altcom_errno());
          return -1;
        }

      if (!ALTCOM_FD_ISSET(sockfd, &writeset))
        {
          altcom_seterrno(ALTCOM_EFAULT);
          DBGIF_LOG1_ERROR("select failed: %d\n", altcom_errno());
          return -1;
        }

      /* Get result of connect */

      val_len = sizeof(result);
      ret = altcom_getsockopt(sockfd, ALTCOM_SOL_SOCKET,
                              ALTCOM_SO_ERROR, (FAR void*)&result,
                              (altcom_socklen_t*)&val_len);
      if (ret == -1)
        {
          DBGIF_LOG1_ERROR("getsockopt failed: %d\n", altcom_errno());
          return -1;
        }
      else
        {
          if (result == 0)
            {
              ret = result;
            }
          else
            {
              DBGIF_LOG1_ERROR("connect failed: %d\n", result);
              ret = -1;
              altcom_seterrno(result);
            }

          return ret;
        }
    }
  else if (result == CONNECT_REQ_FAILURE)
    {
      return -1;
    }

  return 0;
}
