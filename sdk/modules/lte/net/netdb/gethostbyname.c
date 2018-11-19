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
#include "stubsock.h"
#include "dbg_if.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

int h_errno;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gethostbyname
 *
 * Description:
 *   The gethostbyname() function returns a structure of type hostent
 *   for the given host name. Here name is either a hostname, or an IPv4
 *   address in standard dot notation (as for inet_addr(3)), or an IPv6
 *   address in colon (and possibly dot) notation.
 *
 *   If name is an IPv4 or IPv6 address, no lookup is performed and
 *   gethostbyname_r() simply copies name into the h_name field
 *   and its struct in_addr equivalent into the h_addr_list[0] field of the
 *   returned hostent structure.
 *
 * Input Parameters:
 *   name - The name of the host to find.
 *
 * Returned Value:
 *   Upon successful completion, this function will return a pointer to a
 *   hostent structure if the requested entry was found, and a null pointer
 *   if the end of the database was reached or the requested entry was not
 *   found.
 *
 *   Upon unsuccessful completion, gethostbyname() will set h_errno to
 *   indicate the error
 *
 ****************************************************************************/

struct hostent *gethostbyname(const char *name)
{
  FAR struct altcom_hostent* h;

  h = altcom_gethostbyname(name);
  if (!h)
    {
      h_errno = stubsock_convherrno_local(altcom_h_errno);
      return NULL;
    }

  return (struct hostent*)h;
}

#endif /* CONFIG_NET && CONFIG_LTE_NETDB */
