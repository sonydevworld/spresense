/****************************************************************************
 * modules/lte/net/stubsock/stubsock_poll.c
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

#if defined(CONFIG_NET) && !defined(CONFIG_DISABLE_POLL) && \
    defined(CONFIG_NET_DEV_SPEC_SOCK)

#include <sys/types.h>
#include <sys/socket.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>

#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "devspecsock/devspecsock.h"
#include "stubsock.h"
#include "altcom_sock.h"
#include "altcom_socket.h"
#include "altcom_select_ext.h"
#include "altcom_errno.h"
#include "stubsock_mem.h"
#include "dbg_if.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct stubsock_poll_s
{
  FAR struct socket *psock;        /* Needed to handle loss of connection */
  FAR struct pollfd *fds;          /* Needed to handle poll events */
  int32_t           select_id;     /* Needed to handle select async */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void select_async_callback(int32_t ret_code, int32_t err_code,
                                  int32_t id, FAR altcom_fd_set *readset,
                                  FAR altcom_fd_set *writeset,
                                  FAR altcom_fd_set *exceptset, FAR void* priv)
{
  FAR struct stubsock_poll_s    *info;
  FAR struct devspecsock_conn_s *ds_conn;
  FAR struct stubsock_conn_s    *conn;

  info = (FAR struct stubsock_poll_s *)priv;

  DBGIF_ASSERT(info && info->psock && info->fds, "Parameter is NULL\n");
  DBGIF_ASSERT(id == info->select_id, "Unexpected select id\n");

  ds_conn = info->psock->s_conn;
  conn    = ds_conn->devspec_conn;

  net_lock();

  if (info->select_id == -1)
    {
      net_unlock();

      DBGIF_LOG_DEBUG("Select has already canceled\n");
      return;
    }

  /* Reset select id. It means callback has called. */

  info->select_id = -1;

  net_unlock();

  if (ret_code < 0)
    {
      DBGIF_LOG2_ERROR("select response failed: %d err_code: %d\n", ret_code, err_code);
      return;
    }

  if (readset)
    {
      if ((info->fds->events & POLLIN) &&
          ALTCOM_FD_ISSET(conn->stubsockid, readset))
        {
          info->fds->revents |= POLLIN;
        }
    }

  if (writeset)
    {
      if ((info->fds->events & POLLOUT) &&
           ALTCOM_FD_ISSET(conn->stubsockid, writeset))
        {
          info->fds->revents |= POLLOUT;
        }
    }

  if (info->fds->revents != 0)
    {
      sem_post(info->fds->sem);
    }
}

/****************************************************************************
 * Name: stubsock_pollsetup
 ****************************************************************************/

static int stubsock_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct devspecsock_conn_s *ds_conn =
    (FAR struct devspecsock_conn_s*)psock->s_conn;
  FAR struct stubsock_conn_s    *conn = ds_conn->devspec_conn;
  FAR struct stubsock_poll_s    *info;
  altcom_fd_set                  readset;
  FAR altcom_fd_set             *preadset = NULL;
  altcom_fd_set                  writeset;
  FAR altcom_fd_set             *pwriteset = NULL;
  int32_t                        ret;

  if (!conn || !fds)
    {
      return -EINVAL;
    }

  if (conn->stubsockid < 0)
    {
      return -EBADR;
    }

  if (!(fds->events & (POLLIN | POLLOUT)))
    {
      return -EINVAL;
    }

  if (fds->events & POLLIN)
    {
      ALTCOM_FD_ZERO(&readset);
      ALTCOM_FD_SET(conn->stubsockid, &readset);

      preadset = &readset;
    }

  if (fds->events & POLLOUT)
    {
      ALTCOM_FD_ZERO(&writeset);
      ALTCOM_FD_SET(conn->stubsockid, &writeset);

      pwriteset = &writeset;
    }

  /* Check if any requested events are already in effect */

  ret = altcom_select_nonblock((conn->stubsockid + 1),
                               preadset, pwriteset, NULL);
  if (ret < 0)
    {
      ret = altcom_errno();
      return -ret;
    }
  else if (ret > 0)
    {
      /* Yes.. then signal the poll logic */

      if ((fds->events & POLLIN) &&
           ALTCOM_FD_ISSET(conn->stubsockid, preadset))
        {
          fds->revents |= POLLIN;
        }

      if ((fds->events & POLLOUT) &&
           ALTCOM_FD_ISSET(conn->stubsockid, pwriteset))
        {
          fds->revents |= POLLOUT;
        }
    }

  if (fds->revents != 0)
    {
      sem_post(fds->sem);
      return OK;
    }

  if (fds->events & POLLIN)
    {
      ALTCOM_FD_ZERO(&readset);
      ALTCOM_FD_SET(conn->stubsockid, &readset);

      preadset = &readset;
    }

  if (fds->events & POLLOUT)
    {
      ALTCOM_FD_ZERO(&writeset);
      ALTCOM_FD_SET(conn->stubsockid, &writeset);

      pwriteset = &writeset;
    }

  info = (FAR struct stubsock_poll_s*)stubsock_mem_alloc
           (sizeof(struct stubsock_poll_s));
  if (!info)
    {
      DBGIF_LOG_ERROR("Failed to allocate memory.\n");
      return -ENOMEM;
    }

  info->psock = psock;
  info->fds = fds;
  fds->priv = (FAR void*)info;

  ret = altcom_select_async((conn->stubsockid + 1),
                            preadset, pwriteset, NULL,
                            select_async_callback, (void*)info);
  if (ret < 0)
    {
      stubsock_mem_free(info);
      fds->priv = NULL;
      ret = altcom_errno();
      return -ret;
    }

  info->select_id = ret;

  return ret;
}

/****************************************************************************
 * Name: stubsock_pollteardown
 ****************************************************************************/

static int stubsock_pollteardown(FAR struct socket *psock,
                                 FAR struct pollfd *fds)
{
  FAR struct devspecsock_conn_s *ds_conn =
    (FAR struct devspecsock_conn_s*)psock->s_conn;
  FAR struct stubsock_conn_s    *conn = ds_conn->devspec_conn;
  FAR struct stubsock_poll_s    *info;
  int32_t                        select_id;

  if (!conn)
    {
      return -EINVAL;
    }

  /* Recover the socket descriptor poll state info from the poll structure */

  info = (FAR struct stubsock_poll_s *)fds->priv;
  if (info)
    {
      net_lock();

      if (info->select_id != -1)
        {
          select_id = info->select_id;

          /* Reset select id. It means select canceling. */

          info->select_id = -1;

          net_unlock();

          /* Select async canceling */

          altcom_select_async_cancel(select_id);
        }
      else
        {
          net_unlock();
        }

      stubsock_mem_free(info);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stubsock_poll
 ****************************************************************************/

int stubsock_poll(FAR struct socket *psock, FAR struct pollfd *fds, bool setup)
{
  if (setup)
    {
      return stubsock_pollsetup(psock, fds);
    }
  else
    {
      return stubsock_pollteardown(psock, fds);
    }
}

#endif /* CONFIG_NET && !CONFIG_DISABLE_POLL && CONFIG_NET_DEV_SPEC_SOCK */
