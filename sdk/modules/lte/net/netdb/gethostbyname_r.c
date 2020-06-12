/****************************************************************************
 * modules/lte/net/netdb/gethostbyname_r.c
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

#include <nuttx/config.h>
#include <sdk/config.h>

#if defined(CONFIG_NET) && defined(CONFIG_LTE_NETDB)

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <netdb.h>
#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "altcom_socket.h"
#include "altcom_netdb.h"
#include "altcom_errno.h"
#include "dbg_if.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: convherrno_local()
 ****************************************************************************/

static int convherrno_local(int herr)
{
  int ret;

  switch(herr)
    {
      case ALTCOM_HOST_NOT_FOUND:
        ret = HOST_NOT_FOUND;
        break;

      case ALTCOM_NO_DATA:
        ret = NO_DATA;
        break;

      case ALTCOM_NO_RECOVERY:
        ret = NO_RECOVERY;
        break;

      case ALTCOM_TRY_AGAIN:
        ret = TRY_AGAIN;
        break;

      default:
        ret = herr;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: gethostbyname_r
 ****************************************************************************/

int gethostbyname_r(const char *name, struct hostent *ret, char *buf,
                    size_t buflen, struct hostent **result, int *h_errnop)
{
  int retval;

  retval = altcom_gethostbyname_r(name, (struct altcom_hostent*)ret, buf,
                                  buflen, (struct altcom_hostent**)result,
                                  h_errnop);
  if (retval != 0)
    {
      if (h_errnop)
        {
          *h_errnop = convherrno_local(*h_errnop);
        }
    }

  return retval;
}

#endif /* CONFIG_NET && CONFIG_LTE_NETDB */
