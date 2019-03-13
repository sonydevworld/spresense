/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_accept.c
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
#include "altcom_socket.h"
#include "altcom_select.h"
#include "altcom_sock.h"
#include "altcom_seterrno.h"
#include "apicmd_accept.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACCEPT_REQ_DATALEN (sizeof(struct apicmd_accept_s))
#define ACCEPT_RES_DATALEN (sizeof(struct apicmd_acceptres_s))

#define ACCEPT_REQ_SUCCESS 0
#define ACCEPT_REQ_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct accept_req_s
{
  int                        sockfd;
  FAR struct altcom_sockaddr *addr;
  FAR altcom_socklen_t       *addrlen;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: accept_request
 ****************************************************************************/

static int32_t accept_request(FAR struct altcom_socket_s *fsock,
                              FAR struct accept_req_s *req)
{
  int32_t                       ret;
  int32_t                       err;
  int32_t                       result;
  uint16_t                      reslen = 0;
  FAR struct apicmd_accept_s    *cmd = NULL;
  FAR struct apicmd_acceptres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_sock_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_SOCK_ACCEPT, ACCEPT_REQ_DATALEN,
    (FAR void **)&res, ACCEPT_RES_DATALEN))
    {
      return ACCEPT_REQ_FAILURE;
    }

  /* Fill the data */

  cmd->sockfd  = htonl(req->sockfd);
  if (req->addrlen)
    {
      cmd->addrlen = htonl(*req->addrlen);
    }
  else
    {
      cmd->addrlen = htonl(0);
    }

  DBGIF_LOG2_DEBUG("[accept-req]sockfd: %d, addrlen: %d\n", req->sockfd, *req->addrlen);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      ACCEPT_RES_DATALEN,
                      &reslen, SYS_TIMEO_FEVR);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      err = -ret;
      goto errout_with_cmdfree;
    }

  if (reslen != ACCEPT_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = ALTCOM_EFAULT;
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);
  err = ntohl(res->err_code);

  DBGIF_LOG2_DEBUG("[accept-res]ret: %d, err: %d\n", ret, err);

  if (APICMD_ACCEPT_RES_RET_CODE_ERR == ret)
    {
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
      goto errout_with_cmdfree;
    }
  else
    {
      DBGIF_LOG1_DEBUG("[accept-res]addrlen: %d\n", ntohl(res->addrlen));

      if (req->addr)
        {
          if (req->addrlen)
            {
              if (*req->addrlen < ntohl(res->addrlen))
                {
                  DBGIF_LOG2_INFO("Input addrlen: %d, Output addrlen: %d\n", *req->addrlen, ntohl(res->addrlen));
                }
              memcpy(req->addr, &res->address, *req->addrlen);
            }
          else
            {
              DBGIF_LOG_ERROR("Unexpected. addrlen is NULL.\n");
            }
        }
      if (req->addrlen)
        {
          *req->addrlen = ntohl(res->addrlen);
        }
      result = ret;
    }

  altcom_sock_free_cmdandresbuff(cmd, res);

  return result;

errout_with_cmdfree:
  altcom_sock_free_cmdandresbuff(cmd, res);
  altcom_seterrno(err);
  return ACCEPT_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_accept
 ****************************************************************************/

int altcom_accept(int sockfd, struct altcom_sockaddr *addr,
                  altcom_socklen_t *addrlen)
{
  int32_t                     ret;
  int32_t                     result;
  FAR struct altcom_socket_s *fsock;
  struct altcom_fd_set_s      readset;
  struct accept_req_s         req;
  FAR struct altcom_timeval  *recvtimeo;

  /* Check Lte library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      altcom_seterrno(-ret);
      return -1;
    }

  if (addr && (!addrlen))
    {
      DBGIF_LOG_ERROR("addrlen is NULL\n");
      altcom_seterrno(ALTCOM_EINVAL);
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

  if (fsock->flags & ALTCOM_O_NONBLOCK)
    {
      ALTCOM_FD_ZERO(&readset);
      ALTCOM_FD_SET(sockfd, &readset);

      ret = altcom_select_nonblock((sockfd + 1), &readset, NULL, NULL);
      if (ret > 0)
        {
          if (!ALTCOM_FD_ISSET(sockfd, &readset))
            {
              altcom_seterrno(ALTCOM_EFAULT);
              DBGIF_LOG1_ERROR("select failed: %d\n", altcom_errno());
              return -1;
            }

          /* Send accept request */

          result = accept_request(fsock, &req);
          if (result == ACCEPT_REQ_FAILURE)
            {
              return -1;
            }
        }
      else
        {
          if (ret == 0)
            {
              altcom_seterrno(ALTCOM_EAGAIN);
            }
          else
            {
              DBGIF_LOG1_ERROR("select failed: %d\n", altcom_errno());
            }
          return -1;
        }
    }
  else
    {
      /* Wait until a connection is present */

      ALTCOM_FD_ZERO(&readset);
      ALTCOM_FD_SET(sockfd, &readset);

      recvtimeo = &fsock->recvtimeo;
      if ((fsock->recvtimeo.tv_sec == 0) && (fsock->recvtimeo.tv_usec == 0))
        {
          recvtimeo = NULL;
        }

      ret = altcom_select_block((sockfd + 1), &readset, NULL, NULL, recvtimeo);
      if (ret <= 0)
        {
          if (ret == 0)
            {
              altcom_seterrno(ALTCOM_EFAULT);
            }

          if (altcom_errno() == ALTCOM_ETIMEDOUT)
            {
              altcom_seterrno(ALTCOM_EAGAIN);
            }
          DBGIF_LOG1_ERROR("select failed: %d\n", altcom_errno());
          return -1;
        }

      if (!ALTCOM_FD_ISSET(sockfd, &readset))
        {
          altcom_seterrno(ALTCOM_EFAULT);
          DBGIF_LOG1_ERROR("select failed: %d\n", altcom_errno());
          return -1;
        }

      /* Send accept request */

      result = accept_request(fsock, &req);
      if (result == ACCEPT_REQ_FAILURE)
        {
          return -1;
        }
    }

  return result;
}
