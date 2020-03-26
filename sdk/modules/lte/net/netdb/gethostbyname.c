/****************************************************************************
 * modules/lte/net/netdb/gethostbyname.c
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
 * Public Data
 ****************************************************************************/

int h_errno;

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
 * Name: gethostbyname
 ****************************************************************************/

struct hostent *gethostbyname(const char *name)
{
  FAR struct altcom_hostent* h;

  h = altcom_gethostbyname(name);
  if (!h)
    {
      h_errno = convherrno_local(altcom_h_errno);
      return NULL;
    }

  return (struct hostent*)h;
}

#endif /* CONFIG_NET && CONFIG_LTE_NETDB */
