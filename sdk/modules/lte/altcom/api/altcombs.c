/****************************************************************************
 * modules/lte/altcom/api/altcombs.c
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

#include <errno.h>
#include <string.h>

#include "lte/lte_api.h"
#include "dbg_if.h"
#include "buffpoolwrapper.h"
#include "altcombs.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static lte_errinfo_t g_errinfo = { 0, 0, 0, ""};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcombs_get_cb
 *
 * Description:
 *   Get Callback from callback list.
 *
 * Input Parameters:
 *   cb_list  List of callback function.
 *   id       Callback id.
 *
 * Returned Value:
 *   Pointer of Callback function. When not registered callbacke return NULL.
 *
 ****************************************************************************/

void *altcombs_get_cb(FAR struct altcombs_cb_block* cb_list, int32_t id)
{
  FAR struct altcombs_cb_block *list_ptr = NULL;

  if (!cb_list)
    {
      DBGIF_LOG_WARNING("null parameter\n");
      return NULL;
    }

  list_ptr = cb_list;
  while (list_ptr)
    {
      if (list_ptr->id == id)
      {
        return list_ptr->cb_list;
      }

      list_ptr = list_ptr->next;
    }

  return NULL;
}

/****************************************************************************
 * Name: altcombs_add_cblist
 *
 * Description:
 *   Add callback list to the end of list.
 *
 * Input Parameters:
 *   head   Pointer of callback list head.
 *   block  Pointer to callback list.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcombs_add_cblist(struct altcombs_cb_block **head,
                            struct altcombs_cb_block *block)
{
  FAR struct altcombs_cb_block *list_ptr = *head;

  if (!block) return -EINVAL;

  if (*head)
    {
      /* Search end of list */

      while(list_ptr->next)
        {
          list_ptr = list_ptr->next;
        }
      list_ptr->next = block;
      block->prev = list_ptr;
    }
  else
    {
      *head = block;
    }

  return 0;
}

/****************************************************************************
 * Name: delete_callbacklist
 *
 * Description:
 *   Delete list from callback list.
 *
 * Input Parameters:
 *   head        Pointer to callback list head.
 *   cblist_ptr  Target callback list address.
 *
 * Returned Value:
 *   Pointer of removed callback list.
 *   Otherwise NULL is returned.
 *
 ****************************************************************************/

void *altcombs_remove_cblist(struct altcombs_cb_block **head,
                                   void *cblist_ptr)
{
  FAR struct altcombs_cb_block *list_ptr = *head;
  FAR struct altcombs_cb_block *prev_ptr = NULL;

  if (list_ptr)
    {
      while (list_ptr)
        {
          if (list_ptr->cb_list == cblist_ptr)
            {
              /* Check begining of the list */

              if (prev_ptr)
                {
                  prev_ptr->next = list_ptr->next;
                  if(list_ptr->next)
                    {
                      list_ptr->next->prev = prev_ptr;
                    }
                }
              else
                {
                  *head = list_ptr->next;
                }

              return list_ptr;
            }

          prev_ptr = list_ptr;
          list_ptr = list_ptr->next;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: altcombs_search_callbacklist
 *
 * Description:
 *   Search list by callback list address.
 *
 * Input Parameters:
 *   head        Pointer of callback list head.
 *   id          Target callback list id.
 *
 * Returned Value:
 *   If list is found, return that pointer.
 *   Otherwise NULL is returned.
 *
 ****************************************************************************/

struct altcombs_cb_block *altcombs_search_callbacklist(
  struct altcombs_cb_block *head, int32_t id)
{
  FAR struct altcombs_cb_block *list_ptr = head;
  while (list_ptr)
    {
      if (list_ptr->id == id)
        {
          return list_ptr;
        }
      list_ptr = list_ptr->next;
    }

  return NULL;
}

/****************************************************************************
 * Name: altcombs_alloc_callbacklist
 *
 * Description:
 *   Allocation callback block.
 *
 * Input Parameters:
 *   cb_ptr     Pointer of callback list.
 *   cb_block   Pointer of Callback block address.
 *              This argument is out parameter.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcombs_alloc_callbacklist(void *cb_ptr,
                                    struct altcombs_cb_block **cb_block)
{
  if (!cb_ptr)
    {
      return -EINVAL;
    }

  *cb_block = (FAR struct altcombs_cb_block *)
    BUFFPOOL_ALLOC(sizeof(struct altcombs_cb_block));
  if (!*cb_block)
    {
      DBGIF_LOG_ERROR("BUFFPOOL_ALLOC() failed\n");
      return -ENOMEM;
    }

  return 0;
}

/****************************************************************************
 * Name: altcombs_alloc_callbacklist
 *
 * Description:
 *   Free callback block.
 *
 * Input Parameters:
 *   cb_block   Pointer of Callback block address.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcombs_free_callbacklist(struct altcombs_cb_block **cb_block)
{
  if (!*cb_block)
    {
      return -EINVAL;
    }

  (void)BUFFPOOL_FREE(*cb_block);
  return 0;
}

/****************************************************************************
 * Name: altcombs_set_errinfo
 *
 * Description:
 *   Get LTE api last error information.
 *
 * Input Parameters:
 *   err_code    LTE error code.
 *   err_no      ALTCOM error no.
 *   err_str     Error string.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void altcombs_set_errinfo(int32_t err_code,
  int32_t err_no, uint8_t *err_str)
{
  g_errinfo.err_indicator   = 0;
  g_errinfo.err_result_code = 0;
  g_errinfo.err_no          = 0;
  memset(g_errinfo.err_string, 0, LTE_ERROR_STRING_MAX_LEN);
  if (0 != err_code)
    {
      g_errinfo.err_indicator |= LTE_ERR_INDICATOR_ERRCODE;
      g_errinfo.err_result_code = err_code;
    }

  if (0 != err_no)
    {
      g_errinfo.err_indicator |= LTE_ERR_INDICATOR_ERRNO;
      g_errinfo.err_no = err_no;
    }

  if (err_str)
    {
      g_errinfo.err_indicator |= LTE_ERR_INDICATOR_ERRSTR;
      strncpy((char *)g_errinfo.err_string,
        (char *)err_str, LTE_ERROR_STRING_MAX_LEN);
      (g_errinfo.err_string)[LTE_ERROR_STRING_MAX_LEN - 1] = '\0';
    }
}

/****************************************************************************
 * Name: altcombs_get_errinfo
 *
 * Description:
 *   Get LTE api last error information.
 *
 * Input Parameters:
 *   info    Pointer of lte error info.
 *
 * Returned Value:
 *   When get success is returned 0.
 *   When get failed return negative value.
 *
 ****************************************************************************/

int32_t altcombs_get_errinfo(lte_errinfo_t *info)
{
  if (!info)
    {
      return -EINVAL;
    }

  memcpy(info, &g_errinfo, sizeof(lte_errinfo_t));
  return 0;
}
