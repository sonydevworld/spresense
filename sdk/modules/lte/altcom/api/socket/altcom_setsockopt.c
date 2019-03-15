/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_setsockopt.c
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
#include "altcom_in.h"
#include "altcom_errno.h"
#include "altcom_seterrno.h"
#include "apicmd_setsockopt.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SETSOCKOPT_REQ_DATALEN (sizeof(struct apicmd_setsockopt_s))
#define SETSOCKOPT_RES_DATALEN (sizeof(struct apicmd_setsockoptres_s))

#define SET_MODE_8BIT          1
#define SET_MODE_32BIT         2
#define SET_MODE_LINGER        3
#define SET_MODE_INADDR        4
#define SET_MODE_IPMREQ        5

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_setsockopt
 ****************************************************************************/

int altcom_setsockopt(int sockfd, int level, int option, const void *value,
                      altcom_socklen_t value_len)
{
  int32_t                           ret;
  int32_t                           err;
  FAR struct altcom_socket_s        *fsock;
  int32_t                           set_mode = 0;
  uint8_t                           optval[APICMD_SETSOCKOPT_OPTVAL_LENGTH];
  uint16_t                          reslen = 0;
  FAR struct apicmd_setsockopt_s    *cmd = NULL;
  FAR struct apicmd_setsockoptres_s *res = NULL;

  /* Check Lte library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      altcom_seterrno(-ret);
      return -1;
    }

  if (!value)
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
      /* Level: ALTCOM_SOL_SOCKET */

      case ALTCOM_SOL_SOCKET:
        switch(option)
          {
            case ALTCOM_SO_BROADCAST:
            case ALTCOM_SO_KEEPALIVE:
            case ALTCOM_SO_REUSEADDR:
            case ALTCOM_SO_RCVBUF:
            case ALTCOM_SO_NO_CHECK:
              if (value_len < sizeof(int32_t))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_32BIT;
              break;

            case ALTCOM_SO_SNDTIMEO:
              if (value_len < sizeof(struct altcom_timeval))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              memcpy(&fsock->sendtimeo, value,
                     sizeof(struct altcom_timeval));
              return 0;
              break;

            case ALTCOM_SO_RCVTIMEO:
              if (value_len < sizeof(struct altcom_timeval))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              memcpy(&fsock->recvtimeo, value,
                     sizeof(struct altcom_timeval));
              return 0;
              break;

            case ALTCOM_SO_LINGER:
              if (value_len < sizeof(struct altcom_linger))
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
              if (value_len < sizeof(int32_t))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_32BIT;
              break;

            case ALTCOM_IP_MULTICAST_TTL:
            case ALTCOM_IP_MULTICAST_LOOP:
              if (value_len < sizeof(uint8_t))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_8BIT;
              break;

            case ALTCOM_IP_MULTICAST_IF:
              if (value_len < sizeof(struct altcom_in_addr))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_INADDR;
              break;

            case ALTCOM_IP_ADD_MEMBERSHIP:
            case ALTCOM_IP_DROP_MEMBERSHIP:
              if (value_len < sizeof(struct altcom_ip_mreq))
                {
                  altcom_seterrno(ALTCOM_EINVAL);
                  return -1;
                }
              set_mode = SET_MODE_IPMREQ;
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
              if (value_len < sizeof(int32_t))
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
              if (value_len < sizeof(int32_t))
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
              if (value_len < sizeof(int32_t))
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
              if (value_len < sizeof(int32_t))
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

  memset(optval, 0, APICMD_SETSOCKOPT_OPTVAL_LENGTH);

  switch(set_mode)
    {
      case SET_MODE_8BIT:
        optval[0] = *(FAR uint8_t*)value;
        break;

      case SET_MODE_32BIT:
        *((FAR int32_t*)optval) = htonl(*(FAR int32_t*)value);
        break;

      case SET_MODE_LINGER:
        ((FAR struct altcom_linger*)optval)->l_onoff =
          htonl(((FAR struct altcom_linger*)value)->l_onoff);
        ((FAR struct altcom_linger*)optval)->l_linger =
          htonl(((FAR struct altcom_linger*)value)->l_linger);
        break;

      case SET_MODE_INADDR:
        ((FAR struct altcom_in_addr*)optval)->s_addr =
          htonl(((FAR struct altcom_in_addr*)value)->s_addr);
        break;

      case SET_MODE_IPMREQ:
        ((FAR struct altcom_ip_mreq*)optval)->imr_multiaddr.s_addr =
          htonl(((FAR struct altcom_ip_mreq*)value)->imr_multiaddr.s_addr);
        ((FAR struct altcom_ip_mreq*)optval)->imr_interface.s_addr =
          htonl(((FAR struct altcom_ip_mreq*)value)->imr_interface.s_addr);
        break;

      default:
        DBGIF_LOG1_ERROR("Unsupported mode: %d.\n", set_mode);
        altcom_seterrno(ALTCOM_EFAULT);
        return -1;
        break;
    }

  /* Allocate send and response command buffer */

  if (!altcom_sock_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_SOCK_SETSOCKOPT, SETSOCKOPT_REQ_DATALEN,
    (FAR void **)&res, SETSOCKOPT_RES_DATALEN))
    {
      return -1;
    }

  /* Fill the data */

  cmd->sockfd  = htonl(sockfd);
  cmd->level   = htonl(level);
  cmd->optname = htonl(option);
  cmd->optlen  = htonl(value_len);
  memcpy(cmd->optval, optval, value_len);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                      SETSOCKOPT_RES_DATALEN,
                      &reslen, SYS_TIMEO_FEVR);
  if (0 <= ret)
    {
      ret = ntohl(res->ret_code);
      err = ntohl(res->err_code);

      if (APICMD_SETSOCKOPT_RES_RET_CODE_ERR == ret)
        {
          DBGIF_LOG_ERROR("API command response is err.\n");
          altcom_seterrno(err);
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
