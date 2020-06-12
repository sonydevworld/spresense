/****************************************************************************
 * modules/include/lte/altcom/net/altcom_inet.h
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

#ifndef __MODULES_INCLUDE_LTE_ALTCOM_NET_ALTCOM_INET_H
#define __MODULES_INCLUDE_LTE_ALTCOM_NET_ALTCOM_INET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "altcom_socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*  Length of the string form for IP address (excludes NULL termination) */

#define ALTCOM_INET_ADDRSTRLEN 16

/*  Length of the string form for IPv6 address (excludes NULL termination) */

#define ALTCOM_INET6_ADDRSTRLEN 46

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint32_t altcom_in_addr_t;

struct altcom_in_addr
{
  altcom_in_addr_t s_addr;
};

struct altcom_in6_addr
{
  union
  {
    uint32_t u32_addr[4];
    uint16_t u16_addr[8];
    uint8_t  u8_addr[16];
  } un;
#define altcom_s6_addr  un.u8_addr
};

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
 * Name: altcom_htonl
 ****************************************************************************/

uint32_t altcom_htonl(uint32_t x);

/****************************************************************************
 * Name: altcom_htons
 ****************************************************************************/

uint16_t altcom_htons(uint16_t x);

/****************************************************************************
 * Name: altcom_ntohl
 ****************************************************************************/

uint32_t altcom_ntohl(uint32_t x);

/****************************************************************************
 * Name: altcom_ntohs
 ****************************************************************************/

uint16_t altcom_ntohs(uint16_t x);

/****************************************************************************
 * Name: altcom_inet_addr
 ****************************************************************************/

altcom_in_addr_t altcom_inet_addr(const char *cp);

/****************************************************************************
 * Name: altcom_inet_ntoa
 ****************************************************************************/

char *altcom_inet_ntoa(struct altcom_in_addr addr);

/****************************************************************************
 * Name: altcom_inet_ntop
 ****************************************************************************/

const char *altcom_inet_ntop(int af, const void *src, char *dst,
                             altcom_socklen_t size);

/****************************************************************************
 * Name: altcom_inet_pton
 ****************************************************************************/

int altcom_inet_pton(int af, const char *src, void *dst);

/****************************************************************************
 * Name: altcom_inet_aton
 ****************************************************************************/

int altcom_inet_aton(const char *cp, struct altcom_in_addr *inp);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MODULES_INCLUDE_LTE_ALTCOM_NET_ALTCOM_INET_H */
