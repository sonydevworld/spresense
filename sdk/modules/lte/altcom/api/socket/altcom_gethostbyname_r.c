/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_gethostbyname_r.c
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
#include "altcom_netdb.h"
#include "apiutil.h"
#include "altcom_inet.h"
#include "altcom_netdb.h"
#include "apicmd_gethostbynamer.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETHOSTBYNAMER_REQ_DATALEN (sizeof(struct apicmd_gethostbynamer_s))
#define GETHOSTBYNAMER_RES_DATALEN (sizeof(struct apicmd_gethostbynamer_res_s))

#define GETHOSTBYNAMER_REQ_SUCCESS 0
#define GETHOSTBYNAMER_REQ_FAILURE -1

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct gethostbyname_r_helper
{
   FAR struct altcom_in6_addr *addr_list[2];  /* Pointer to h_addr_list */
  struct altcom_in6_addr       addr;
  FAR char                    *aliases[2];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gethostbynamer_request
 ****************************************************************************/

static int32_t gethostbynamer_request(FAR const char *name, int32_t namelen,
                                      FAR struct altcom_hostent *ret,
                                      FAR char *buf, size_t buflen,
                                      FAR struct altcom_hostent **result,
                                      FAR int *h_errnop)
{
  int32_t                                cmdret;
  int32_t                                err;
  uint16_t                               reslen = 0;
  size_t                                 h_namelen;
  size_t                                 h_aliaseslen;
  size_t                                 expect_buflen;
  FAR char                               *pname;
  FAR char                               *paliases;
  FAR struct apicmd_gethostbynamer_s     *cmd;
  FAR struct apicmd_gethostbynamer_res_s *res;
  FAR struct gethostbyname_r_helper      *phelper;

  if (!altcom_sock_alloc_cmdandresbuff(
    (FAR void **)&cmd, APICMDID_SOCK_GETHOSTBYNAMER,
    GETHOSTBYNAMER_REQ_DATALEN,
    (FAR void **)&res, GETHOSTBYNAMER_RES_DATALEN))
    {
      err = ALTCOM_HOST_NOT_FOUND;
      goto errout;
    }

  cmd->namelen = htonl(namelen);
  memcpy(cmd->name, name, namelen);
  cmd->buflen  = htonl(buflen);
  cmdret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                         GETHOSTBYNAMER_RES_DATALEN,
                         &reslen, SYS_TIMEO_FEVR);
  if (0 > cmdret)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", cmdret);
      err = ALTCOM_HOST_NOT_FOUND;
      goto errout_with_cmdfree;
    }

  if (GETHOSTBYNAMER_RES_DATALEN != reslen)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = ALTCOM_HOST_NOT_FOUND;
      goto errout_with_cmdfree;
    }

  cmdret = ntohl(res->ret_code);
  err    = ntohl(res->err_code);

  if (APICMD_GETHOSTBYNAMER_RES_RET_CODE_OK != cmdret)
    {
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
      goto errout_with_cmdfree;
    }

  expect_buflen = sizeof(struct gethostbyname_r_helper);

  h_namelen = strnlen((FAR const char *)res->h_name,
                       APICMD_GETHOSTBYNAMER_NAME_MAX_LENGTH);
  if (h_namelen != 0)
    {
      expect_buflen += h_namelen + 1;
    }
  DBGIF_LOG1_DEBUG("[gethostbyname_r-res] h_name len: %d\n", h_namelen);

  h_aliaseslen = strnlen((FAR const char *)res->h_aliases,
                         APICMD_GETHOSTBYNAMER_RES_H_ALIASES_LENGTH);
  if (h_aliaseslen != 0)
    {
      expect_buflen += h_aliaseslen + 1;
    }
  DBGIF_LOG1_DEBUG("[gethostbyname_r-res] h_aliases len: %d\n", h_aliaseslen);

  /* Check buffer length */

  if (buflen < expect_buflen)
    {
      DBGIF_LOG2_ERROR("buffer length is not enough: %d, expected: %d\n", buflen, expect_buflen);

      /* buffer length is not enough */

      err = ALTCOM_ERANGE;
      goto errout_with_cmdfree;
    }

  /* Fill command result */

  phelper = (struct gethostbyname_r_helper *)buf;

  /* Set h_name parameter */

  pname = ((FAR char*)phelper) + sizeof(struct gethostbyname_r_helper);

  if (h_namelen != 0)
    {
      memcpy(pname, (FAR const char *)res->h_name, h_namelen);
      pname[h_namelen] = 0;
      ret->h_name = pname;
    }
  else
    {
      ret->h_name = NULL;
    }

  /* Set h_aliases parameter */

  if (h_aliaseslen != 0)
    {
      paliases = pname;
      if  (h_namelen != 0)
        {
          paliases = pname + (h_namelen + 1);
        }

      memcpy(paliases, (FAR char *)res->h_aliases, h_aliaseslen);
      paliases[h_aliaseslen] = 0;
      phelper->aliases[0] = paliases;
    }
  else
    {
      phelper->aliases[0] = NULL;
    }
  phelper->aliases[1] = NULL;
  ret->h_aliases = (FAR char**)&phelper->aliases;

  /* Set h_addrtype parameter */

  ret->h_addrtype = ntohl(res->h_addrtype);

  DBGIF_LOG1_DEBUG("[gethostbyname_r-res] h_addrtype: %d\n", ret->h_addrtype);

  /* Set h_length parameter */

  ret->h_length = (ALTCOM_AF_INET == ret->h_addrtype) ?
    sizeof(struct altcom_in_addr) : sizeof(struct altcom_in6_addr);

  DBGIF_LOG1_DEBUG("[gethostbyname_r-res] h_length: %d\n", ret->h_length);

  /* Set h_addr_list parameter */

  memcpy(&phelper->addr, &res->h_addr_list[0], ret->h_length);
  phelper->addr_list[0] = &phelper->addr;
  phelper->addr_list[1] = NULL;
  ret->h_addr_list = (FAR char**)&phelper->addr_list;

  *result = ret;

  altcom_sock_free_cmdandresbuff(cmd, res);

  return GETHOSTBYNAMER_REQ_SUCCESS;

errout_with_cmdfree:
  altcom_sock_free_cmdandresbuff(cmd, res);

errout:
  if (h_errnop)
    {
      *h_errnop = err;
    }

  return GETHOSTBYNAMER_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_gethostbyname_r
 ****************************************************************************/

int altcom_gethostbyname_r(const char *name, struct altcom_hostent *ret,
                           char *buf, size_t buflen,
                           struct altcom_hostent **result, int *h_errnop)
{
  int32_t status;
  int32_t namelen = 0;
  int32_t retval;

  /* Check Lte library status */

  status = altcombs_check_poweron_status();
  if (0 > status)
    {
      if (h_errnop)
        {
          *h_errnop = ALTCOM_HOST_NOT_FOUND;
        }

      return -1;
    }

  if (!name || !ret || !buf || !buflen || !result)
    {
      DBGIF_LOG_ERROR("Invalid paramaeter\n");
      if (h_errnop)
        {
          *h_errnop = ALTCOM_HOST_NOT_FOUND;
        }

      return -1;
    }

  namelen = strlen(name);
  if (!namelen || APICMD_GETHOSTBYNAMER_NAME_MAX_LENGTH < namelen)
    {
      DBGIF_LOG1_ERROR("Invalid param. namelen = [%d]\n", namelen);
      if (h_errnop)
        {
          *h_errnop = ALTCOM_HOST_NOT_FOUND;
        }

      return -1;
    }

  retval = gethostbynamer_request(name, namelen, ret,
                                  buf, buflen, result, h_errnop);
  if (retval != GETHOSTBYNAMER_REQ_SUCCESS)
    {
      return -1;
    }

  return 0;
}
