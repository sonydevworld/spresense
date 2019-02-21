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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcombs_alloc_cbblock
 *
 * Description:
 *   Allocation callback block.
 *
 * Input Parameters:
 *   id  Callback block id.
 *   cb  Pointer to callback.
 *
 * Returned Value:
 *   If the process succeeds, it returns allocated callback block.
 *   Otherwise NULL is returned.
 *
 ****************************************************************************/

static FAR struct altcombs_cb_block *altcombs_alloc_cbblock(int32_t id,
                                                            FAR void *cb)
{
  FAR struct altcombs_cb_block *block;

  if (!cb)
    {
      return NULL;
    }

  block = (FAR struct altcombs_cb_block *)BUFFPOOL_ALLOC(
            sizeof(struct altcombs_cb_block));
  if (!block)
    {
      DBGIF_LOG_ERROR("Failed to allocate memory\n");
      return NULL;
    }

  block->id      = id;
  block->cb      = cb;
  block->removal = false;
  block->prev    = NULL;
  block->next    = NULL;

  return block;
}

/****************************************************************************
 * Name: altcombs_free_cbblock
 *
 * Description:
 *   Free callback block.
 *
 * Input Parameters:
 *   cb_block   Pointer to callback block.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

static int32_t altcombs_free_cbblock(FAR struct altcombs_cb_block *cb_block)
{
  if (!cb_block)
    {
      return -EINVAL;
    }

  (void)BUFFPOOL_FREE(cb_block);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcombs_add_cbblock
 *
 * Description:
 *   Add callback block to the end of list.
 *
 * Input Parameters:
 *   head  Pointer of callback block list head.
 *   id    Callback block id.
 *   cb    Pointer to callback.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcombs_add_cbblock(FAR struct altcombs_cb_block **head,
                             int32_t                        id,
                             FAR void                      *cb)
{
  FAR struct altcombs_cb_block *block;
  FAR struct altcombs_cb_block *curr_block;

  if ((!cb) || (!head))
    {
      return -EINVAL;
    }

  block = altcombs_alloc_cbblock(id, cb);
  if (!block)
    {
      return -ENOMEM;
    }

  curr_block = *head;

  if (*head)
    {
      /* Search end of block */

      while(curr_block->next)
        {
          curr_block = curr_block->next;
        }
      curr_block->next = block;
      block->prev      = curr_block;
    }
  else
    {
      *head = block;
    }

  return 0;
}

/****************************************************************************
 * Name: altcombs_remove_cbblock
 *
 * Description:
 *   Delete callback block from callback block list.
 *
 * Input Parameters:
 *   head   Pointer to callback block list head.
 *   block  Pointer to callback block.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcombs_remove_cbblock(FAR struct altcombs_cb_block **head,
                                FAR struct altcombs_cb_block  *block)
{
  FAR struct altcombs_cb_block *curr_block = *head;

  if (!block)
    {
      return -EINVAL;
    }

  while (curr_block)
    {
      if (curr_block->cb == block->cb)
        {
          /* Check begining of the list */

          if (curr_block->prev)
            {
              curr_block->prev->next = curr_block->next;
              if(curr_block->next)
                {
                  curr_block->next->prev = curr_block->prev;
                }
            }
          else
            {
              *head = curr_block->next;
              if(curr_block->next)
                {
                  curr_block->next->prev = NULL;
                }
            }
          altcombs_free_cbblock(curr_block);

          return 0;
        }
      curr_block = curr_block->next;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: altcombs_search_cbblock
 *
 * Description:
 *   Search callback block by callback id.
 *
 * Input Parameters:
 *   head  Pointer to callback block list head.
 *   id    Target callback block id.
 *
 * Returned Value:
 *   If list is found, return that pointer.
 *   Otherwise NULL is returned.
 *
 ****************************************************************************/

FAR struct altcombs_cb_block *altcombs_search_cbblock(
  FAR struct altcombs_cb_block *head, int32_t id)
{
  FAR struct altcombs_cb_block *curr_block = head;

  while (curr_block)
    {
      if (curr_block->id == id)
        {
          return curr_block;
        }
      curr_block = curr_block->next;
    }

  return NULL;
}

/****************************************************************************
 * Name: altcombs_search_cbblock_bycb
 *
 * Description:
 *   Search callback block by callback pointer.
 *
 * Input Parameters:
 *   head  Pointer to callback block list head.
 *   cb    Pointer to callback.
 *
 * Returned Value:
 *   If list is found, return that pointer.
 *   Otherwise NULL is returned.
 *
 ****************************************************************************/

FAR struct altcombs_cb_block *altcombs_search_cbblock_bycb(
  FAR struct altcombs_cb_block *head, FAR void *cb)
{
  FAR struct altcombs_cb_block *curr_block = head;

  while (curr_block)
    {
      if (curr_block->cb == cb)
        {
          return curr_block;
        }
      curr_block = curr_block->next;
    }

  return NULL;
}

/****************************************************************************
 * Name: altcombs_mark_removal_cbblock
 *
 * Description:
 *   Mark as removal callback block.
 *
 * Input Parameters:
 *   block  Pointer to callback block.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcombs_mark_removal_cbblock(FAR struct altcombs_cb_block *block)
{
  if (!block)
    {
      return -EINVAL;
    }

  block->removal = true;

  return 0;
}

/****************************************************************************
 * Name: altcombs_remove_removal_cbblock
 *
 * Description:
 *   Delete callback block marked for removal.
 *
 * Input Parameters:
 *   head   Pointer to callback block list head.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/

int32_t altcombs_remove_removal_cbblock(FAR struct altcombs_cb_block **head)
{
  FAR struct altcombs_cb_block *curr_block = *head;

  while (curr_block)
    {
      if (curr_block->removal)
        {
          /* Check begining of the list */

          if (curr_block->prev)
            {
              curr_block->prev->next = curr_block->next;
              if(curr_block->next)
                {
                  curr_block->next->prev = curr_block->prev;
                }
            }
          else
            {
              *head = curr_block->next;
              if(curr_block->next)
                {
                  curr_block->next->prev = NULL;
                }
            }
          altcombs_free_cbblock(curr_block);
        }
      curr_block = curr_block->next;
    }

  return 0;
}

/****************************************************************************
 * Name: altcombs_get_next_cbblock
 *
 * Description:
 *   Get pointer to next callback block.
 *
 * Input Parameters:
 *   block  Pointer to callback block.
 *
 * Returned Value:
 *   If next is found, return that pointer.
 *   Otherwise NULL is returned.
 *
 ****************************************************************************/

FAR struct altcombs_cb_block *altcombs_get_next_cbblock(
  FAR struct altcombs_cb_block *block)
{
  if (!block)
    {
      return NULL;
    }

  return block->next;
}

/****************************************************************************
 * Name: altcombs_set_errinfo
 *
 * Description:
 *   Get LTE API last error information.
 *
 * Input Parameters:
 *   info    Pointer of LTE error information.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void altcombs_set_errinfo(FAR lte_errinfo_t *info)
{
  if (info)
    {
      memcpy(&g_errinfo, info, sizeof(lte_errinfo_t));
    }
}

/****************************************************************************
 * Name: altcombs_get_errinfo
 *
 * Description:
 *   Get LTE API last error information.
 *
 * Input Parameters:
 *   info    Pointer of LTE error information.
 *
 * Returned Value:
 *   When get success is returned 0.
 *   When get failed return negative value.
 *
 ****************************************************************************/

int32_t altcombs_get_errinfo(FAR lte_errinfo_t *info)
{
  if (!info)
    {
      return -EINVAL;
    }

  memcpy(info, &g_errinfo, sizeof(lte_errinfo_t));
  return 0;
}
