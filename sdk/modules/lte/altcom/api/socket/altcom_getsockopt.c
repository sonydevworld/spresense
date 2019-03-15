/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_getsockopt.c
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
#include "altcom_inet.h"
#include "altcom_seterrno.h"
#include "apicmd_getsockopt.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETSOCKOPT_REQ_DATALEN (sizeof(struct apicmd_getsockopt_s))
#define GETSOCKOPT_RES_DATALEN (sizeof(struct apicmd_getsockoptres_s))

#define SET_MODE_8BIT          1
#define SET_MODE_32BIT         2
#define SET_MODE_LINGER        3
#define SET_MODE_INADDR        4

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_getsockopt
 ****************************************************************************/

int altcom_getsockopt(int sockfd, int level, int option, void *value,
                      altcom_socklen_t *value_len)
{
  int32_t                           ret;
  int32_t                           err;
  int32_t                           optlen;
  int32_t                           set_mode = 0;
  FAR int32_t                       *pint32_val;
  uint16_t                          reslen = 0;
  FAR struct apicmd_getsockopt_s    *cmd = NULL;
  FAR struct apicmd_getsockoptres_s *res = NULL;
  FAR struct altcom_socket_s        *fsock;
  FAR struct altcom_linger          *plinger;
  FAR struct altcom_in_addr         *pinaddr;

  if (!altcom_isinit())
    {
      DBGIF_LOG_ERROR("Not intialized\n");
      altcom_seterrno(ALTCOM_ENETDOWN);
      return -1;
    }

  if ((!value) || (!value_len))
    {
      DBGIF_LOG_ERROR("Invalid parameter\n");
      altcom_seterrno(ALTCOM_EINVAL);
      return -1;
    }

  fsock = altcom_sockfd_socket(sockfd);
  if (!fsock)
    {
      altcom_seterrno(ALTCOM_EINVAL);
      return -1;
    }

  switch(level)
    {
      /* Level: ALTCOM_SOL_SOCKET*/

      case ALTCOM_SOL_SOCKET:
        switch(option)
          {
            case ALTCOM_SO_ACCEPTCONN:
            case ALTCOM_SO_ERROR:
            case ALTCOM_SO_BROADCAST:
            case ALTCOM_SO_KEEPALIVE:
            case ALTCOM_SO_REUSEADDR:
            case ALTCOM_SO_TYPE:
            case ALTCOM_SO_RCVBUF:
            case ALTCOM_SO_NO_CHECK:
              if (*(FAR int32_t*)value_len < sizeof(int32_t))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_32BIT;
              break;


            case ALTCOM_SO_SNDTIMEO:
              if (*(FAR int32_t*)value_len < sizeof(struct altcom_timeval))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              memcpy(value, &fsock->sendtimeo,
                     sizeof(struct altcom_timeval));
              *value_len = sizeof(struct altcom_timeval);
              return 0;
              break;

            case ALTCOM_SO_RCVTIMEO:
              if (*(FAR int32_t*)value_len < sizeof(struct altcom_timeval))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              memcpy(value, &fsock->recvtimeo,
                     sizeof(struct altcom_timeval));
              *value_len = sizeof(struct altcom_timeval);
              return 0;
              break;

            case ALTCOM_SO_LINGER:
              if (*(FAR int32_t*)value_len < sizeof(struct altcom_linger))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_LINGER;
              break;

            default:
              DBGIF_LOG1_ERROR("Not support option: %d\n", option);
              altcom_seterrno(ALTCOM_EINVAL);
              return -1;
              break;
          }
        break;

      /* Level: ALTCOM_IPPROTO_IP */

      case ALTCOM_IPPROTO_IP:
        switch(option)
          {
            case ALTCOM_IP_TTL:
            case ALTCOM_IP_TOS:
              if (*(FAR int32_t*)value_len < sizeof(int32_t))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_32BIT;
              break;

            case ALTCOM_IP_MULTICAST_TTL:
            case ALTCOM_IP_MULTICAST_LOOP:
              if (*(FAR int32_t*)value_len < sizeof(uint8_t))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_8BIT;
              break;

            case ALTCOM_IP_MULTICAST_IF:
              if (*(FAR int32_t*)value_len < sizeof(struct altcom_in_addr))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_INADDR;
              break;

            default:
              DBGIF_LOG1_ERROR("Not support option: %d\n", option);
              altcom_seterrno(ALTCOM_EINVAL);
              return -1;
              break;
          }
        break;

      /* Level: ALTCOM_IPPROTO_TCP */

      case ALTCOM_IPPROTO_TCP:
        switch(option)
          {
            case ALTCOM_TCP_NODELAY:
            case ALTCOM_TCP_KEEPALIVE:
            case ALTCOM_TCP_KEEPIDLE:
            case ALTCOM_TCP_KEEPINTVL:
            case ALTCOM_TCP_KEEPCNT:
              if (*(FAR int32_t*)value_len < sizeof(int32_t))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_32BIT;
              break;

            default:
              DBGIF_LOG1_ERROR("Not support option: %d\n", option);
              altcom_seterrno(ALTCOM_EINVAL);
              return -1;
              break;
          }
        break;

      /* Level: ALTCOM_IPPROTO_IPV6 */

      case ALTCOM_IPPROTO_IPV6:
        switch(option)
          {
            case ALTCOM_IPV6_V6ONLY:
              if (*(FAR int32_t*)value_len < sizeof(int32_t))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_32BIT;
              break;

            default:
              DBGIF_LOG1_ERROR("Not support option: %d\n", option);
              altcom_seterrno(ALTCOM_EINVAL);
              return -1;
              break;
          }
        break;

      /* Level: ALTCOM_IPPROTO_UDPLITE */

      case ALTCOM_IPPROTO_UDPLITE:
        switch(option)
          {
            case ALTCOM_UDPLITE_SEND_CSCOV:
            case ALTCOM_UDPLITE_RECV_CSCOV:
              if (*(FAR int32_t*)value_len < sizeof(int32_t))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_32BIT;
              break;

            default:
              DBGIF_LOG1_ERROR("Not support option: %d\n", option);
              altcom_seterrno(ALTCOM_EINVAL);
              return -1;
              break;
          }
        break;

      /* Level: ALTCOM_IPPROTO_RAW */

      case ALTCOM_IPPROTO_RAW:
        switch(option)
          {
            case ALTCOM_IPV6_CHECKSUM:
              if (*(FAR int32_t*)value_len < sizeof(int32_t))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_32BIT;
              break;

            default:
              DBGIF_LOG1_ERROR("Not support option: %d\n", option);
              altcom_seterrno(ALTCOM_EINVAL);
              return -1;
              break;
          }
        break;

      default:
        DBGIF_LOG1_ERROR("Not support level: %d\n", level);
        altcom_seterrno(ALTCOM_EINVAL);
        return -1;
        break;
    }

  /* Allocate send and response command buffer */

  if (!altcom_sock_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_SOCK_GETSOCKOPT, GETSOCKOPT_REQ_DATALEN,
    (FAR void **)&res, GETSOCKOPT_RES_DATALEN))
    {
      return -1;
    }

  /* Fill the data */

  cmd->sockfd  = htonl(sockfd);
  cmd->level   = htonl(level);
  cmd->optname = htonl(option);
  cmd->optlen  = htonl(*value_len);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      GETSOCKOPT_RES_DATALEN,
                      &reslen, SYS_TIMEO_FEVR);
  if (0 <= ret)
    {
      ret    = ntohl(res->ret_code);
      err    = ntohl(res->err_code);
      optlen = ntohl(res->optlen);

      if (APICMD_GETSOCKOPT_RES_RET_CODE_ERR == ret)
        {
          DBGIF_LOG_ERROR("API command response is err.\n");
          altcom_seterrno(err);
        }
      else
        {
          switch(set_mode)
            {
              case SET_MODE_8BIT:
                if (optlen == sizeof(uint8_t))
                  {
                    *(FAR uint8_t*)value = res->optval[0];
                    ret = 0;
                  }
                else
                  {
                    DBGIF_LOG1_ERROR("Unexpected option len: %d.\n", optlen);
                    altcom_seterrno(ALTCOM_EFAULT);
                    ret = -1;
                  }
                break;

              case SET_MODE_32BIT:
                if (optlen == sizeof(int32_t))
                  {
                    pint32_val = (FAR int32_t*)&res->optval[0];
                    *(FAR int32_t*)value = ntohl(*pint32_val);
                    ret = 0;
                  }
                else
                  {
                    DBGIF_LOG1_ERROR("Unexpected option len: %d.\n", optlen);
                    altcom_seterrno(ALTCOM_EFAULT);
                    ret = -1;
                  }
                break;

              case SET_MODE_LINGER:
                if (optlen == sizeof(struct altcom_linger))
                  {
                    plinger = (FAR struct altcom_linger*)&res->optval[0];
                    ((FAR struct altcom_linger*)value)->l_onoff =
                      ntohl(plinger->l_onoff);
                    ((FAR struct altcom_linger*)value)->l_linger =
                      ntohl(plinger->l_linger);
                    ret = 0;
                  }
                else
                  {
                    DBGIF_LOG1_ERROR("Unexpected option len: %d.\n", optlen);
                    altcom_seterrno(ALTCOM_EFAULT);
                    ret = -1;
                  }
                break;

              case SET_MODE_INADDR:
                if (optlen == sizeof(struct altcom_in_addr))
                  {
                    pinaddr = (FAR struct altcom_in_addr*)&res->optval[0];
                    ((FAR struct altcom_in_addr*)value)->s_addr =
                      ntohl(pinaddr->s_addr);
                    ret = 0;
                  }
                else
                  {
                    DBGIF_LOG1_ERROR("Unexpected option len: %d.\n", optlen);
                    altcom_seterrno(ALTCOM_EFAULT);
                    ret = -1;
                  }
                break;

              default:
                DBGIF_LOG1_ERROR("Unexpected set mode: %d.\n", set_mode);
                altcom_seterrno(ALTCOM_EFAULT);
                ret = -1;
                break;
            }
        }
    }
  else
    {
      altcom_seterrno(-ret);
      ret = -1;
    }

  altcom_sock_free_cmdandresbuff(cmd, res);

  return ret;
}
