/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_inet_aton.c
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

#include <ctype.h>

#include "dbg_if.h"
#include "altcom_inet.h"
#include "cc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_inet_aton
 ****************************************************************************/

int altcom_inet_aton(const char *cp, struct altcom_in_addr *inp)
{
  uint32_t     val;
  uint8_t      base;
  char         c;
  uint32_t     parts[4];
  FAR uint32_t *pp = parts;

  c = *cp;
  for (;;)
    {
      /* Collect number up to ``.''.
       * Values are specified as for C:
       * 0x=hex, 0=octal, 1-9=decimal. */

      if (!isdigit(c))
        {
          return 0;
        }

      val = 0;
      base = 10;

      if (c == '0')
        {
          c = *++cp;
          if (c == 'x' || c == 'X')
            {
              base = 16;
              c = *++cp;
            }
          else
            {
              base = 8;
            }
        }

      for (;;)
        {
          if (isdigit(c))
            {
              val = (val * base) + (uint32_t)(c - '0');
              c = *++cp;
            }
          else if (base == 16 && isxdigit(c))
            {
              val = (val << 4) | (uint32_t)(c + 10 - (islower(c) ? 'a' : 'A'));
              c = *++cp;
            }
          else
            {
              break;
            }
        }

      if (c == '.')
        {
          /* Internet format:
           *  a.b.c.d
           *  a.b.c   (with c treated as 16 bits)
           *  a.b (with b treated as 24 bits) */

          if (pp >= parts + 3)
            {
              return 0;
            }
          *pp++ = val;
          c = *++cp;
        }
      else
        {
          break;
        }
    }

  /* Check for trailing characters. */

  if (c != '\0' && !isspace(c))
    {
      return 0;
    }

  /* Concoct the address according to
   * the number of parts specified. */

  switch (pp - parts + 1)
    {
      case 0:
        /* initial nondigit */

        return 0;

      /* a -- 32 bits */

      case 1:
        break;

      /* a.b -- 8.24 bits */

      case 2:  
        if (val > 0xffffffUL)
          {
            return 0;
          }

        if (parts[0] > 0xff)
          {
            return 0;
          }
        val |= parts[0] << 24;
        break;

      /* a.b.c -- 8.8.16 bits */

      case 3:
        if (val > 0xffff)
          {
            return 0;
          }

        if ((parts[0] > 0xff) || (parts[1] > 0xff))
          {
            return 0;
          }
        val |= (parts[0] << 24) | (parts[1] << 16);
        break;

      /* a.b.c.d -- 8.8.8.8 bits */

      case 4:  
        if (val > 0xff)
          {
            return 0;
          }

        if ((parts[0] > 0xff) || (parts[1] > 0xff) || (parts[2] > 0xff))
          {
            return 0;
          }
        val |= (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8);
        break;

      default:
        DBGIF_ASSERT(0, "unhandled\n");
        break;
    }

  if (inp)
    {
      inp->s_addr = altcom_htonl(val);
    }

  return 1;
}
