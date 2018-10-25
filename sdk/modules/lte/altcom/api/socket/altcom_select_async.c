/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_select_async.c
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

#include "dbg_if.h"
#include "altcom_sock.h"
#include "altcom_select_ext.h"
#include "altcom_select.h"
#include "altcom_errno.h"
#include "altcom_seterrno.h"
#include "buffpoolwrapper.h"
#include "cc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

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

static FAR struct select_asynccb_s *g_callbacklist_head = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: allocate_callbacklist
 *
 * Description:
 *   Allocate select callback list.
 *
 * Input Parameters:
 *   select_id  This is ID that identifies the select request.
 *
 * Returned Value:
 *   If the allocation succeeds, it returns the allocated pointer.
 *   Otherwise NULL is returned.
 *
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
 *
 * Description:
 *   Free select callback list.
 *
 * Input Parameters:
 *   list  Pointer to callback list.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void free_callbacklist(FAR struct select_asynccb_s *list)
{
  (void)BUFFPOOL_FREE((void*)list);
}


/****************************************************************************
 * Name: add_callbacklist
 *
 * Description:
 *   Add callback list to the end of list.
 *
 * Input Parameters:
 *   list  Pointer to callback list.
 *
 * Returned Value:
 *   None.
 *
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
 *
 * Description:
 *   Delete list from callback list.
 *
 * Input Parameters:
 *   list  Pointer to callback list.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
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
 *
 * Description:
 *   Search list by select ID.
 *
 * Input Parameters:
 *   select_id  Target select ID.
 *
 * Returned Value:
 *   If list is found, return that pointer.
 *   Otherwise NULL is returned.
 *
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
 *
 * Description:
 *   Register callback to be used for select async.
 *
 * Input Parameters:
 *   select_id  This is ID that identifies the select request.
 *   cb         Callback function when received select response.
 *
 * Returned Value:
 *   None
 *
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
 *
 * Description:
 *   Unregister callback to be used for select async.
 *
 * Input Parameters:
 *   select_id  This is ID that identifies the select request.
 *
 * Returned Value:
 *   None
 *
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_select_async
 *
 * Description:
 *   altcom_select_async() is a function that operates select asynchronously.
 *   Notified by callback when one or more of the file descriptors become
 *   "ready" for some class of I/O operation (e.g., input possible). It will
 *   be notified until callback is called once or canceled
 *   by altcom_select_async_cancel().
 *
 * Input parameters:
 *   maxfdp1 - the maximum socket file descriptor number (+1) of any
 *             descriptor in any of the three sets.
 *   readset - the set of descriptions to monitor for read-ready events
 *   writeset - the set of descriptions to monitor for write-ready events
 *   exceptset - the set of descriptions to monitor for error events
 *   callback - Callback function that informs that one or more file
 *              descriptors are in the "ready" state
 *   priv - For use by caller
 *
 *  Return:
 *   A non-negative id on success; -1 on error with errno set
 *   appropriately.
 *
 ****************************************************************************/

int altcom_select_async(int maxfdp1, altcom_fd_set *readset,
                        altcom_fd_set *writeset, altcom_fd_set *exceptset,
                        altcom_select_async_cb_t callback, void* priv)
{
  int32_t id;

  if (!callback)
    {
      altcom_seterrno(ALTCOM_EINVAL);
      return -1;
    }

  id = altcom_select_request_asyncsend(maxfdp1, readset, writeset, exceptset);
  if (id < 0)
    {
      return -1;
    }

  setup_callback(id, callback, priv);

  return id;
}


/****************************************************************************
 * Name: altcom_select_async_exec_callback
 *
 * Description:
 *   Execute callback that registered by altcom_select_async().
 *
 * Input parameters:
 *   id - id returned by altcom_select_async()
 *   ret_code - result of altcom_select_async()
 *   err_code - error code when altcom_select_async() failure
 *   readset - the set of descriptions to monitor for read-ready events
 *   writeset - the set of descriptions to monitor for write-ready events
 *   exceptset - the set of descriptions to monitor for error events
 *
 *  Return:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int altcom_select_async_exec_callback(int32_t id, int32_t ret_code,
                                      int32_t err_code,
                                      altcom_fd_set *readset,
                                      altcom_fd_set *writeset,
                                      altcom_fd_set *exceptset)
{
  int32_t                     ret = -1;
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
 *
 * Description:
 *   altcom_select_async_cancel() cancel altcom_select_async() based on the
 *   id.
 *
 * Input parameters:
 *   id - id returned by altcom_select_async()
 *
 *  Return:
 *   0 on success; -1 on error with errno set appropriately
 *
 ****************************************************************************/

int altcom_select_async_cancel(int id)
{
  int32_t ret;

  teardown_callback(id);

  ret = altcom_select_cancel_request_send(id);

  return ret;
}
