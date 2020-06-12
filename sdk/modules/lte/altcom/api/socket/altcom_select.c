/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_select.c
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

#include <stdbool.h>

#include "dbg_if.h"
#include "altcom_sock.h"
#include "altcom_socket.h"
#include "altcom_select.h"
#include "altcom_select_ext.h"
#include "altcom_seterrno.h"
#include "apicmd_select.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SELECT_REQ_DATALEN (sizeof(struct apicmd_select_s))
#define SELECT_RES_DATALEN (sizeof(struct apicmd_selectres_s))

#define SELECT_REQ_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct select_req_s
{
  int32_t           select_id;
  int               maxfdp1;
  uint32_t          request;
  FAR altcom_fd_set *readset;
  FAR altcom_fd_set *writeset;
  FAR altcom_fd_set *exceptset;
  int32_t           timeout;
};

struct select_asynccb_s
{
  int32_t                     select_id;
  altcom_select_async_cb_t    callback;
  FAR void                    *priv;
  FAR struct select_asynccb_s *next;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int32_t g_select_id = 0;
static FAR struct select_asynccb_s *g_callbacklist_head = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: generate_selectid
 ****************************************************************************/

static int32_t generate_selectid(void)
{
  g_select_id++;

  return (g_select_id & 0x7fffffff);
}


/****************************************************************************
 * Name: allocate_callbacklist
 ****************************************************************************/

static FAR struct select_asynccb_s *allocate_callbacklist(int32_t select_id)
{
  FAR struct select_asynccb_s *list;

  /* Allocate callback list */

  list = (FAR struct select_asynccb_s*)BUFFPOOL_ALLOC
           (sizeof(struct select_asynccb_s));
  if (!list)
    {
      DBGIF_LOG_ERROR("Failed to allocate memory.\n");
      return NULL;
    }

  memset(list, 0, sizeof(struct select_asynccb_s));

  /* Initialize cllback list */

  list->select_id = select_id;
  list->next      = NULL;

  return list;
}

/****************************************************************************
 * Name: free_callbacklist
 ****************************************************************************/

static void free_callbacklist(FAR struct select_asynccb_s *list)
{
  (void)BUFFPOOL_FREE((void*)list);
}


/****************************************************************************
 * Name: add_callbacklist
 ****************************************************************************/

static void add_callbacklist(FAR struct select_asynccb_s *list)
{
  FAR struct select_asynccb_s *list_ptr = g_callbacklist_head;

  if (g_callbacklist_head)
    {
      /* Search end of list */

      while(list_ptr->next)
        {
          list_ptr = list_ptr->next;
        }
      list_ptr->next = list;
    }
  else
    {
      g_callbacklist_head = list;
    }

}

/****************************************************************************
 * Name: delete_callbacklist
 ****************************************************************************/

static int32_t delete_callbacklist(FAR struct select_asynccb_s *list)
{
  FAR struct select_asynccb_s *list_ptr = g_callbacklist_head;
  FAR struct select_asynccb_s *prev_ptr = NULL;

  if (g_callbacklist_head)
    {
      while (list_ptr)
        {
          if (list_ptr == list)
            {
              /* Check begining of the list */

              if (prev_ptr)
                {
                  prev_ptr->next = list_ptr->next;
                }
              else
                {
                  g_callbacklist_head = list_ptr->next;
                }

              return 0;
            }
          prev_ptr = list_ptr;
          list_ptr = list_ptr->next;
        }
    }

  return -1;
}

/****************************************************************************
 * Name: search_callbacklist
 ****************************************************************************/

static FAR struct select_asynccb_s *search_callbacklist(int32_t select_id)
{
  FAR struct select_asynccb_s *list_ptr = g_callbacklist_head;

  while (list_ptr)
    {
      if (list_ptr->select_id == select_id)
        {
          return list_ptr;
        }
      list_ptr = list_ptr->next;
    }

  return NULL;
}

/****************************************************************************
 * Name: setup_callback
 ****************************************************************************/

static void setup_callback(int32_t select_id, altcom_select_async_cb_t cb,
                           FAR void *priv)
{
  FAR struct select_asynccb_s* list;

  /* Check if select ID is already in use */

  if (!search_callbacklist(select_id))
    {
      list = allocate_callbacklist(select_id);
      if (list)
        {
          list->callback = cb;
          list->priv     = priv;

          /* Add list to the end of list */

          add_callbacklist(list);
        }
    }
  else
    {
      DBGIF_LOG1_WARNING("This select ID[%d] already in use.\n", select_id);
    }
}

/****************************************************************************
 * Name: teardown_callback
 ****************************************************************************/

static void teardown_callback(int32_t select_id)
{
  FAR struct select_asynccb_s *list;
  int32_t                     ret;

  list = search_callbacklist(select_id);
  if (list)
    {
      /* Delte list from callback list */

      ret = delete_callbacklist(list);
      if (ret < 0)
        {
          DBGIF_LOG_ERROR("Failed to delete callbck list.\n");
        }
      else
        {
          free_callbacklist(list);
        }
    }
}

/****************************************************************************
 * Name: fill_select_request_command
 ****************************************************************************/

static void fill_select_request_command(FAR struct apicmd_select_s *cmd,
                                        FAR struct select_req_s *req)
{
  memset(cmd, 0, sizeof(struct apicmd_select_s));

  cmd->request = htonl(req->request);
  cmd->id      = htonl(req->select_id);
  cmd->maxfds  = htonl(req->maxfdp1);
  if (req->readset)
    {
      memcpy(&cmd->readset, req->readset,
             sizeof(altcom_fd_set));
      cmd->used_setbit |= APICMD_SELECT_USED_BIT_READSET;
    }
  if (req->writeset)
    {
      memcpy(&cmd->writeset, req->writeset,
             sizeof(altcom_fd_set));
      cmd->used_setbit |= APICMD_SELECT_USED_BIT_WRITESET;
    }
  if (req->exceptset)
    {
      memcpy(&cmd->exceptset, req->exceptset,
             sizeof(altcom_fd_set));
      cmd->used_setbit |= APICMD_SELECT_USED_BIT_EXCEPTSET;
    }
  cmd->used_setbit = htons(cmd->used_setbit);

  DBGIF_LOG3_DEBUG("[select-req]request: %d, id: %d, maxfdp1: %d\n", req->request, req->select_id, req->maxfdp1);
  DBGIF_LOG1_DEBUG("[select-req]used_setbit: %x\n", ntohs(cmd->used_setbit));
  if (req->readset)
    {
      DBGIF_LOG2_DEBUG("[select-req]readset: %x,%x\n", req->readset->fd_bits[0], req->readset->fd_bits[1]);
    }
  if (req->writeset)
    {
      DBGIF_LOG2_DEBUG("[select-req]writeset: %x,%x\n", req->writeset->fd_bits[0], req->writeset->fd_bits[1]);
    }
  if (req->exceptset)
    {
      DBGIF_LOG2_DEBUG("[select-req]exceptset: %x,%x\n", req->exceptset->fd_bits[0], req->exceptset->fd_bits[1]);
    }
}

/****************************************************************************
 * Name: select_request
 ****************************************************************************/

static int32_t select_request(FAR struct select_req_s *req)
{
  int32_t                       ret;
  int32_t                       err;
  uint16_t                      reslen = 0;
  FAR struct apicmd_select_s    *cmd = NULL;
  FAR struct apicmd_selectres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_sock_alloc_cmdandresbuff((FAR void **)&cmd,
                                       APICMDID_SOCK_SELECT,
                                       SELECT_REQ_DATALEN,
                                       (FAR void **)&res,
                                       SELECT_RES_DATALEN))
    {
      err = ALTCOM_ENOMEM;
      goto errout;
    }

  /* Fill the data */

  fill_select_request_command(cmd, req);

  /* Send command and block until receive a response or timeout */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res,
                       SELECT_RES_DATALEN,
                       &reslen, req->timeout);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);

      if (ret == -ETIMEDOUT)
        {
          memset(cmd, 0, sizeof(struct apicmd_select_s));
          cmd->request = htonl(APICMD_SELECT_REQUEST_BLOCKCANCEL);
          cmd->id      = htonl(req->select_id);

          DBGIF_LOG2_DEBUG("[select-req]request: %d, id: %d\n", APICMD_SELECT_REQUEST_BLOCKCANCEL, req->select_id);

          /* Send command */

          apicmdgw_send((FAR uint8_t *)cmd, NULL, 0, NULL, 0);

          err = ALTCOM_ETIMEDOUT;
          goto errout_with_cmdfree;
        }
      else
        {
          err = -ret;
          goto errout_with_cmdfree;
        }
    }

  if (reslen != SELECT_RES_DATALEN)
    {
      DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
      err = ALTCOM_EFAULT;
      goto errout_with_cmdfree;
    }

  ret = ntohl(res->ret_code);
  err = ntohl(res->err_code);

  DBGIF_LOG2_DEBUG("[select-res]ret: %d, err: %d\n", ret, err);

  if (APICMD_SELECT_RES_RET_CODE_ERR == ret)
    {
      DBGIF_LOG1_ERROR("API command response is err :%d.\n", err);
      goto errout_with_cmdfree;
    }

  if (req->readset)
    {
      memcpy(req->readset, &res->readset, sizeof(altcom_fd_set));
    }
  if (req->writeset)
    {
      memcpy(req->writeset, &res->writeset, sizeof(altcom_fd_set));
    }
  if (req->exceptset)
    {
      memcpy(req->exceptset, &res->exceptset, sizeof(altcom_fd_set));
    }

  altcom_sock_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_sock_free_cmdandresbuff(cmd, res);

errout:
  altcom_seterrno(err);
  return SELECT_REQ_FAILURE;
}

/****************************************************************************
 * Name: select_request_async
 ****************************************************************************/

static int32_t select_request_async(FAR struct select_req_s *req)
{
  int32_t                       ret;
  int32_t                       err;
  FAR struct apicmd_select_s    *cmd = NULL;

  /* Allocate send command buffer only */

  if (!ALTCOM_SOCK_ALLOC_CMDBUFF(cmd, APICMDID_SOCK_SELECT,
                                 SELECT_REQ_DATALEN))
    {
      err = ALTCOM_ENOMEM;
      goto errout;
    }

  /* Fill the data */

  fill_select_request_command(cmd, req);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, NULL, 0, NULL, SYS_TIMEO_FEVR);

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("apicmdgw_send error: %d\n", ret);

      err = -ret;
      goto errout_with_cmdfree;
    }

  ret = req->select_id;

  altcom_free_cmd((FAR uint8_t *)cmd);

  return ret;

errout_with_cmdfree:
  altcom_free_cmd((FAR uint8_t *)cmd);

errout:
  altcom_seterrno(err);
  return SELECT_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_select_nonblock
 ****************************************************************************/

int altcom_select_nonblock(int maxfdp1, altcom_fd_set *readset,
                           altcom_fd_set *writeset,
                           altcom_fd_set *exceptset)
{

  int32_t             ret;
  int32_t             result;
  struct select_req_s req;

  /* Check Lte library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      altcom_seterrno(-ret);
      return -1;
    }

  req.select_id = generate_selectid();
  req.maxfdp1   = maxfdp1;
  req.request   = APICMD_SELECT_REQUEST_NONBLOCK;
  req.readset   = readset;
  req.writeset  = writeset;
  req.exceptset = exceptset;
  req.timeout   = SYS_TIMEO_FEVR;

  result = select_request(&req);

  if (result == SELECT_REQ_FAILURE)
    {
      return -1;
    }

  return result;
}

/****************************************************************************
 * Name: altcom_select_block
 ****************************************************************************/

int altcom_select_block(int maxfdp1, altcom_fd_set *readset,
                        altcom_fd_set *writeset, altcom_fd_set *exceptset,
                        struct altcom_timeval *timeout)
{
  int32_t             ret;
  int32_t             result;
  struct select_req_s req;

  /* Check Lte library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      altcom_seterrno(-ret);
      return -1;
    }

  req.select_id = generate_selectid();
  req.maxfdp1   = maxfdp1;
  req.request   = APICMD_SELECT_REQUEST_BLOCK;
  req.readset   = readset;
  req.writeset  = writeset;
  req.exceptset = exceptset;
  if (timeout)
    {
      req.timeout = ALTCOM_SOCK_TIMEVAL2MS(timeout);
    }
  else
    {
      req.timeout = SYS_TIMEO_FEVR;
    }

  result = select_request(&req);

  if (result == SELECT_REQ_FAILURE)
    {
      return -1;
    }

  return result;
}

/****************************************************************************
 * Name: altcom_select
 ****************************************************************************/

int altcom_select(int maxfdp1, altcom_fd_set *readset,
                  altcom_fd_set *writeset, altcom_fd_set *exceptset,
                  struct altcom_timeval *timeout)
{
  int ret;

  if (timeout && (timeout->tv_sec == 0) && (timeout->tv_usec == 0))
    {
      ret = altcom_select_nonblock(maxfdp1, readset, writeset, exceptset);
    }
  else
    {
      ret = altcom_select_block(maxfdp1, readset, writeset, exceptset, timeout);
    }

  if ((ret == SELECT_REQ_FAILURE) && (altcom_errno() == ALTCOM_ETIMEDOUT))
    {
      ret = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: altcom_select_async
 ****************************************************************************/

int altcom_select_async(int maxfdp1, altcom_fd_set *readset,
                        altcom_fd_set *writeset, altcom_fd_set *exceptset,
                        altcom_select_async_cb_t callback, void* priv)
{
  int ret;
  struct select_req_s req;

  if (!callback)
    {
      altcom_seterrno(ALTCOM_EINVAL);
      return -1;
    }

  /* Check LTE library status */

  ret = altcombs_check_poweron_status();
  if (0 > ret)
    {
      altcom_seterrno(-ret);
      return -1;
    }

  req.select_id = generate_selectid();
  req.maxfdp1   = maxfdp1;
  req.request   = APICMD_SELECT_REQUEST_BLOCK;
  req.readset   = readset;
  req.writeset  = writeset;
  req.exceptset = exceptset;
  req.timeout   = SYS_TIMEO_FEVR; /* ignore when do select_request_async */

  setup_callback(req.select_id, callback, priv);

  ret = select_request_async(&req);

  if (ret == SELECT_REQ_FAILURE)
    {
      teardown_callback(req.select_id);
      return -1;
    }

  return ret;
}


/****************************************************************************
 * Name: altcom_select_async_exec_callback
 ****************************************************************************/

int altcom_select_async_exec_callback(int32_t id, int32_t ret_code,
                                      int32_t err_code,
                                      altcom_fd_set *readset,
                                      altcom_fd_set *writeset,
                                      altcom_fd_set *exceptset)
{
  int ret = -1;
  FAR struct select_asynccb_s *list;

  list = search_callbacklist(id);
  if (list)
    {
      /* execute callback */

      list->callback(ret_code, err_code, id, readset, writeset, exceptset,
                     list->priv);

      teardown_callback(id);
      ret = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: altcom_select_async_cancel
 ****************************************************************************/

int altcom_select_async_cancel(int id, bool is_send)
{
  int ret = 0;
  struct select_req_s req;

  teardown_callback(id);

  if (is_send)
    {
      /* Check LTE library status */

      ret = altcombs_check_poweron_status();
      if (0 > ret)
        {
          altcom_seterrno(-ret);
          return -1;
        }

      req.select_id = id;
      req.maxfdp1   = 0;
      req.request   = APICMD_SELECT_REQUEST_BLOCKCANCEL;
      req.readset   = NULL;
      req.writeset  = NULL;
      req.exceptset = NULL;
      req.timeout   = SYS_TIMEO_FEVR; /* ignore when do select_request_async */

      ret = select_request_async(&req);

      if (ret == SELECT_REQ_FAILURE)
        {
          return -1;
        }
    }

  return ret;
}
