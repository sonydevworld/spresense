/****************************************************************************
 * modules/include/lte/altcom/net/altcom_netdb.h
 *
 *   Copyright 2018, 2020 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_INCLUDE_LTE_ALTCOM_NET_ALTCOM_NETDB_H
#define __MODULES_INCLUDE_LTE_ALTCOM_NET_ALTCOM_NETDB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "altcom_socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* input flags for struct altcom_addrinfo */

#define ALTCOM_AI_PASSIVE     0x01
#define ALTCOM_AI_CANONNAME   0x02
#define ALTCOM_AI_NUMERICHOST 0x04
#define ALTCOM_AI_NUMERICSERV 0x08
#define ALTCOM_AI_V4MAPPED    0x10
#define ALTCOM_AI_ALL         0x20
#define ALTCOM_AI_ADDRCONFIG  0x40

/* Errors used by the DNS API functions, h_errno can be one of them */

#define ALTCOM_EAI_NONAME     200
#define ALTCOM_EAI_SERVICE    201
#define ALTCOM_EAI_FAIL       202
#define ALTCOM_EAI_MEMORY     203
#define ALTCOM_EAI_FAMILY     204

#define ALTCOM_HOST_NOT_FOUND 210
#define ALTCOM_NO_DATA        211
#define ALTCOM_NO_RECOVERY    212
#define ALTCOM_TRY_AGAIN      213

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct altcom_hostent
{
  char  *h_name;
  char **h_aliases;
  int    h_addrtype;
  int    h_length;
  char **h_addr_list;
#define h_addr h_addr_list[0]
};

struct altcom_addrinfo
{
  int                     ai_flags;
  int                     ai_family;
  int                     ai_socktype;
  int                     ai_protocol;
  altcom_socklen_t        ai_addrlen;
  struct altcom_sockaddr *ai_addr;
  char                   *ai_canonname;
  struct altcom_addrinfo *ai_next;
};

/* application accessible error code set by the DNS API functions */

extern int altcom_h_errno;

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_freeaddrinfo
 ****************************************************************************/

void altcom_freeaddrinfo(struct altcom_addrinfo *res);

/****************************************************************************
 * Name: altcom_getaddrinfo
 ****************************************************************************/

int altcom_getaddrinfo(const char *nodename, const char *servname,
                       const struct altcom_addrinfo *hints,
                       struct altcom_addrinfo **res);

/****************************************************************************
 * Name: altcom_gethostbyname
 ****************************************************************************/

struct altcom_hostent *altcom_gethostbyname(const char *name);

/****************************************************************************
 * Name: altcom_gethostbyname_r
 ****************************************************************************/

int altcom_gethostbyname_r(const char *name, struct altcom_hostent *ret,
                           char *buf, size_t buflen,
                           struct altcom_hostent **result, int *h_errnop);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MODULES_INCLUDE_LTE_ALTCOM_NET_ALTCOM_NETDB_H */
