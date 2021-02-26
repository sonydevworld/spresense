/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_getaddrinfo.c
 *
 *   Copyright 2018, 2020, 2021 Sony Semiconductor Solutions Corporation
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

#include "lte/lte_api.h"
#include "dbg_if.h"
#include "buffpoolwrapper.h"
#include "altcom_netdb.h"
#include "apiutil.h"
#include "apicmd_getaddrinfo.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETADDRINFO_REQ_DATALEN (sizeof(struct apicmd_getaddrinfo_s))
#define GETADDRINFO_RES_DATALEN (sizeof(struct apicmd_getaddrinfores_s))
#define NPDNINFO 2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct getaddrinfo_req_s
{
  const char                   *nodename;
  const char                   *servname;
  const struct altcom_addrinfo *hints;
  struct altcom_addrinfo       **res;
};

struct netinfo_helper
{
  lte_netinfo_t netinfo;
  lte_pdn_t pdninfo[NPDNINFO];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_addrfamily_by_registeredaddr
 ****************************************************************************/

static int get_addrfamily_by_registeredaddr(void)
{
  int ret = ALTCOM_AF_UNSPEC;
  FAR struct netinfo_helper *helper;
  FAR lte_netinfo_t *netinfo;
  int i;

  helper = (FAR struct netinfo_helper *)
             BUFFPOOL_ALLOC(sizeof(struct netinfo_helper));
  if (!helper)
    {
      DBGIF_LOG_ERROR("Failed to allocate memory\n");
      return ALTCOM_AF_UNSPEC;
    }

  helper->netinfo.pdn_stat = &helper->pdninfo[0];
  netinfo = &helper->netinfo;

  /* Get registered IP address */

  ret = lte_get_netinfo_sync(NPDNINFO, netinfo);
  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("Failed to lte_get_netinfo_sync(): %d\n", ret);
      BUFFPOOL_FREE(helper);
      return ALTCOM_AF_UNSPEC;
    }

  ret = ALTCOM_AF_UNSPEC;

  /* Search the data PDN */

  for (i = 0; i < netinfo->pdn_num; i++)
    {
      /* Is this PDN the data PDN? */

      if (netinfo->pdn_stat[i].apn_type & LTE_APN_TYPE_DEFAULT)
        {
          /* Yes.. */

          if (netinfo->pdn_stat[i].ipaddr_num == 2)
            {
              /* It means that IPv4 and IPv6 addresses are registered.
               * Set AF_UNSPEC because it must try to resolve
               * both IP addresses.
               */

              ret = ALTCOM_AF_UNSPEC;
            }
          else if (netinfo->pdn_stat[i].ipaddr_num == 1)
            {
              if (netinfo->pdn_stat[i].address[0].ip_type == LTE_IPTYPE_V4)
                {
                  /* It means that IPv4 address is registered. */

                  ret = ALTCOM_AF_INET;
                }
              else
                {
                  /* It means that IPv6 address is registered. */

                  ret = ALTCOM_AF_INET6;
                }
            }
        }
    }

  BUFFPOOL_FREE(helper);

  return ret;
}

/****************************************************************************
 * Name: getaddrinfo_request
 ****************************************************************************/

static int32_t getaddrinfo_request(FAR struct getaddrinfo_req_s* req)
{
  int32_t                            ret;
  int32_t                            err = 0;
  uint16_t                           reslen = 0;
  uint32_t                           nodenamelen;
  uint32_t                           servnamelen;
  FAR struct apicmd_getaddrinfo_s    *cmd = NULL;
  FAR struct apicmd_getaddrinfores_s *res = NULL;
  FAR struct altcom_addrinfo         *ai = NULL;
  FAR struct altcom_addrinfo         *tmpai = NULL;
  FAR struct altcom_sockaddr_storage *ss = NULL;
  FAR char                           *cname = NULL;
  int32_t                            i;
  int32_t                            ai_size;
  int32_t                            cname_len;
  bool                               alloc_fail = false;
  int32_t                            ai_family = 0;

  /* get_addrfamily_by_registeredaddr() uses the ALTCOM command,
   * so the buffer pool is consumed internally.
   * Calling this function before allocating memory from the buffer pool
   * with altcom_sock_alloc_cmdandresbuff() reduces the temporary consumption
   * of the buffer pool.
   */

  if (req->hints)
    {
      if (req->hints->ai_family == ALTCOM_AF_UNSPEC)
        {
          ai_family = htonl(get_addrfamily_by_registeredaddr());
        }
    }

  /* Allocate send and response command buffer */

  if (!altcom_sock_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_SOCK_GETADDRINFO, GETADDRINFO_REQ_DATALEN,
    (FAR void **)&res, GETADDRINFO_RES_DATALEN))
    {
      return ALTCOM_EAI_MEMORY;
    }

  memset(cmd, 0, sizeof(struct apicmd_getaddrinfo_s));

  /* Fill the data */

  if (req->nodename)
    {
      nodenamelen = strnlen(req->nodename,
                            APICMD_GETADDRINFO_NODENAME_MAX_LENGTH);
      cmd->nodenamelen = htonl(nodenamelen);
      memcpy(cmd->nodename, req->nodename, nodenamelen);
    }

  if (req->servname)
    {
      servnamelen = strnlen(req->servname,
                            APICMD_GETADDRINFO_SERVNAME_MAX_LENGTH);
      memcpy(cmd->servname, req->servname, servnamelen);
      cmd->servnamelen = htonl(servnamelen);
    }

  if (req->hints)
    {
      cmd->hints_flag  = htonl(APICMD_GETADDRINFO_HINTS_FLAG_ENABLE);
      cmd->ai_flags    = htonl(req->hints->ai_flags);
      cmd->ai_family   = htonl(req->hints->ai_family);

      if (req->hints->ai_family == ALTCOM_AF_UNSPEC)
        {
          cmd->ai_family = ai_family;
        }

      cmd->ai_socktype = htonl(req->hints->ai_socktype);
      cmd->ai_protocol = htonl(req->hints->ai_protocol);
    }
  else
    {
      cmd->hints_flag = htonl(APICMD_GETADDRINFO_HINTS_FLAG_DISABLE);
    }

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                     GETADDRINFO_RES_DATALEN, &reslen,
                     SYS_TIMEO_FEVR);
  if (0 > ret)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);
      err = -ret;
      goto errout_with_cmdfree;
    }

  if (reslen != GETADDRINFO_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = ALTCOM_EFAULT;
      goto errout_with_cmdfree;
    }

  /* Check api command response */

  ret = ntohl(res->ret_code);

  if (APICMD_GETADDRINFO_RES_RET_CODE_OK != ret)
    {
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", ret);
      err = ret;
      goto errout_with_cmdfree;
    }

  /* Fill result */

  if (ntohl(res->ai_num) > APICMD_GETADDRINFO_RES_AI_COUNT)
    {
      DBGIF_LOG1_ERROR("Invalid ai_num: %d\n", ntohl(res->ai_num));
      res->ai_num = 0;
    }

  for (i = 0; i < ntohl(res->ai_num); i++)
    {
      cname_len = ntohl(res->ai[i].ai_cnamelen);
      if (cname_len > APICMD_GETADDRINFO_AI_CANONNAME_LENGTH)
        {
          DBGIF_LOG1_ERROR("Invalid cname_len: %d\n", cname_len);
          cname_len = 0;
        }

      /* Allocate the whole area for each ai */

      ai_size = sizeof(struct altcom_addrinfo) +
                sizeof(struct altcom_sockaddr_storage) +
                cname_len;

      ai = (FAR struct altcom_addrinfo *)BUFFPOOL_ALLOC(ai_size);
      if (!ai)
        {
          DBGIF_LOG_ERROR("BUFFPOOL_ALLOC()\n");
          alloc_fail = true;
        }
     else
       {
         ai->ai_flags    = ntohl(res->ai[i].ai_flags);
         ai->ai_family   = ntohl(res->ai[i].ai_family);
         ai->ai_socktype = ntohl(res->ai[i].ai_socktype);
         ai->ai_protocol = ntohl(res->ai[i].ai_protocol);
         ai->ai_addrlen  = ntohl(res->ai[i].ai_addrlen);

         /* Fill the ai_addr area */

         ss = (FAR struct altcom_sockaddr_storage*)((FAR uint8_t*)ai +
                sizeof(struct altcom_addrinfo));
         memcpy(ss, &res->ai[i].ai_addr,
                sizeof(struct altcom_sockaddr_storage));
         ai->ai_addr = (FAR struct altcom_sockaddr*)ss;

         /* Fill the ai_canonname area */

         ai->ai_canonname = NULL;
         if (cname_len != 0)
           {
             cname = (FAR char*)((FAR uint8_t*)ss +
                                 sizeof(struct altcom_sockaddr_storage));
             memcpy(cname, res->ai[i].ai_canonname, cname_len);
             ai->ai_canonname = cname;
           }
         ai->ai_next = NULL;

         if (*req->res == NULL)
           {
             *req->res = ai;
           }
         else
           {
             tmpai = *req->res;
             while(tmpai->ai_next)
               {
                 tmpai = tmpai->ai_next;
               }
             tmpai->ai_next = ai;
           }
       }
   }
  if (alloc_fail)
    {
      ai = *req->res;
      while(ai)
        {
          tmpai = ai->ai_next;
          BUFFPOOL_FREE(ai);
          ai = tmpai;
        }
      *req->res = NULL;
      goto errout_with_cmdfree;
    }


  altcom_sock_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_sock_free_cmdandresbuff(cmd, res);
  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_getaddrinfo
 ****************************************************************************/

int altcom_getaddrinfo(const char *nodename, const char *servname,
                       const struct altcom_addrinfo *hints,
                       struct altcom_addrinfo **res)
{
  int32_t                  ret;
  int32_t                  result;
  struct getaddrinfo_req_s req;

  /* Check LTE library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      altcom_seterrno(-ret);
      return ALTCOM_EAI_FAIL;
    }

  if (!nodename && !servname)
    {
      DBGIF_LOG_ERROR("Invalid parameter\n");
      return ALTCOM_EAI_NONAME;
    }

  *res = NULL;

  req.nodename = nodename;
  req.servname = servname;
  req.hints    = hints;
  req.res      = res;

  result = getaddrinfo_request(&req);

  if (APICMD_GETADDRINFO_RES_RET_CODE_OK != result)
    {
      return result;
    }

  return 0;
}
