/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_inet_ntop.c
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

#include <stdio.h>
#include <string.h>

#include "dbg_if.h"
#include "altcom_inet.h"
#include "altcom_errno.h"
#include "altcom_seterrno.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define s6_addr16  un.u16_addr

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_ipv4_ntop
 ****************************************************************************/

static int inet_ipv4_ntop(FAR const void *src, FAR char *dest,
                          altcom_socklen_t size)
{
  FAR uint8_t *ptr;

  if (size < ALTCOM_INET_ADDRSTRLEN)
    {
      return -ALTCOM_ENOSPC;
    }

  ptr = (FAR uint8_t *)src;
  sprintf(dest, "%u.%u.%u.%u", ptr[0], ptr[1], ptr[2], ptr[3]);
  return 0;
}

/****************************************************************************
 * Name: inet_ipv6_ntop
 ****************************************************************************/

static int inet_ipv6_ntop(FAR const void *src, FAR char *dest,
                          altcom_socklen_t size)
{
  FAR const struct altcom_in6_addr *in6_addr;
  uint16_t                         warray[8];
  int                              offset;
  int                              entry;
  int                              count;
  int                              maxentry;
  int                              maxcount;

  if (size < ALTCOM_INET6_ADDRSTRLEN)
    {
      return -ALTCOM_ENOSPC;
    }

  in6_addr = (FAR const struct altcom_in6_addr *)src;
  entry    = -1;
  maxentry = -1;
  maxcount = 0;
  offset   = 0;

  while (offset < 8)
    {
      warray[offset] = altcom_ntohs(in6_addr->s6_addr16[offset]);
      if (warray[offset] == 0)
        {
          entry = offset;
          count = 1;
          offset++;

          while (offset < 8)
            {
              warray[offset] = altcom_ntohs(in6_addr->s6_addr16[offset]);
              if (warray[offset] != 0)
                {
                  break;
                }
              offset++;
              count++;
            }

          if (count > maxcount)
            {
              maxentry = entry;
              maxcount = count;
            }
        }
      offset++;
    }

  offset = 0;
  dest[0] = '\0';

  while (offset < 8)
    {
      if (offset == maxentry)
        {
          size   -= snprintf(&dest[strlen(dest)], size, ":");
          offset += maxcount;
          if (offset >= 8)
            {
              size -= snprintf(&dest[strlen(dest)], size, ":");
            }
        }
      else
        {
          if (offset > 0)
            {
              size -= snprintf(&dest[strlen(dest)], size, ":");
            }

          size -= snprintf(&dest[strlen(dest)], size, "%x", warray[offset]);
          offset++;
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_inet_ntop
 ****************************************************************************/

const char *altcom_inet_ntop(int af, const void *src, char *dst,
                             altcom_socklen_t size)
{
  int ret;

  DBGIF_ASSERT(src && dst, "Invalid parameter\n");

  /* Do the conversion according to the IP version */

  switch (af)
    {
    case ALTCOM_AF_INET:
      ret = inet_ipv4_ntop(src, dst, size);
      break;

    case ALTCOM_AF_INET6:
      ret = inet_ipv6_ntop(src, dst, size);
      break;

    default:
      ret = -ALTCOM_EAFNOSUPPORT;
      break;
    }

  /* Handle errors in the conversion */

  if (ret < 0)
    {
      altcom_seterrno(-ret);
      memset(dst, 0, size);
      return NULL;
    }

  /* Return success */

  return dst;
}
