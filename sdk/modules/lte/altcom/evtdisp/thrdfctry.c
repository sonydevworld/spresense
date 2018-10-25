/****************************************************************************
 * modules/lte/altcom/evtdisp/thrdfctry.c
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

#include <stdlib.h>
#include <string.h>
#include "thrdfctry.h"
#include "dbg_if.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define THRDFCTRY_SEQ_MAXTHRDNUM (1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct thrdfctry_table_s
{
  uint8_t                      id;
  FAR struct thrdpool_s        *pool;
  FAR struct thrdfctry_table_s *next;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void thrdfctry_dellist(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct thrdfctry_table_s *g_thrdfctry_list = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: thrdfctry_dellist
 *
 * Description:
 *   Delete link list of g_thrdfctry_list.
 *
 * Input Parameters:
 *   addr  Start address of linked list to delete.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void thrdfctry_dellist(void)
{
  int32_t ret = 0;
  FAR struct thrdfctry_table_s *target = g_thrdfctry_list;
  FAR struct thrdfctry_table_s *next   = NULL;

  while (target)
    {
      next = target->next;
      if (target->pool)
        {
          ret  = thrdpool_delete(target->pool);
          DBGIF_ASSERT(0 == ret, "thrdpool_delete failed.\n");
        }

      SYS_FREE(target);
      target = next;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: thrdfctry_init
 *
 * Description:
 *   Initialize the worker thread factory.
 *
 * Input Parameters:
 *   set     Parameter list of the thread to be created.
 *   setnum  Number of @set.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t thrdfctry_init(FAR struct thrdfctry_thrdset_s set[], int8_t setnum)
{
  FAR struct thrdfctry_table_s **targettbl = NULL;
  int8_t                       num         = 0;
  struct thrdpool_set_s        poolset;
  int32_t                      ret         = 0;

  if (g_thrdfctry_list)
    {
      DBGIF_LOG_ERROR("Work thread factory is initialized.\n");
      return -EPERM;
    }

  if (!set || !setnum)
    {
      DBGIF_LOG_ERROR("Incorrect argument.\n");
      return -EINVAL;
    }

  if (THRDFCTRY_WRKTYPE_NUM <= set[num].type)
    {
      DBGIF_LOG_ERROR("Incorrect argument.\n");
      return -EINVAL;
    }

  targettbl = &g_thrdfctry_list;
  for (num = 0; num < setnum; num++)
    {
      *targettbl = (FAR struct thrdfctry_table_s *)SYS_MALLOC
        (sizeof(struct thrdfctry_table_s));
      if (!(*targettbl))
        {
          DBGIF_LOG_ERROR("Allocate failed.\n");
          thrdfctry_dellist();
          g_thrdfctry_list = NULL;
          return -ENOMEM;
        }

      memset(*targettbl, 0, (sizeof(struct thrdfctry_table_s)));

      switch (set[num].type)
        {
          case THRDFCTRY_PARALLEL:
            poolset.thrdstacksize = set[num].u.paraset.thrdstacksize;
            poolset.thrdpriority  = set[num].u.paraset.thrdpriority;
            poolset.maxthrdnum    = set[num].u.paraset.maxthrdnum;
            poolset.maxquenum     = set[num].u.paraset.maxquenum;
            break;
          case THRDFCTRY_SEQUENTIAL:
            poolset.thrdstacksize = set[num].u.seqset.thrdstacksize;
            poolset.thrdpriority  = set[num].u.seqset.thrdpriority;
            poolset.maxthrdnum    = THRDFCTRY_SEQ_MAXTHRDNUM;
            poolset.maxquenum     = set[num].u.seqset.maxquenum;
            break;
          default:
            DBGIF_LOG1_ERROR("unexpected type:%d\n", set[num].type);
            thrdfctry_dellist();
            g_thrdfctry_list = NULL;
            return -EINVAL;
            break;
        }

      (*targettbl)->pool = thrdpool_create(&poolset);
      if (!(*targettbl)->pool)
        {
          ret = -errno;
          DBGIF_LOG_ERROR("thrdpool_create failed.\n");
          thrdfctry_dellist();
          g_thrdfctry_list = NULL;
          return ret;
        }

      (*targettbl)->id   = set[num].id;
      (*targettbl)->next = NULL;

      targettbl = &(*targettbl)->next;
    }

  return 0;
}

/****************************************************************************
 * Name: thrdfctry_fin
 *
 * Description:
 *   Finalize the worker thread factory.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t thrdfctry_fin(void)
{
  if (!g_thrdfctry_list)
    {
      DBGIF_LOG_ERROR("Work thread factory is uninitialized.\n");
      return -EPERM;
    }

  thrdfctry_dellist();
  g_thrdfctry_list = NULL;
  return 0;
}

/****************************************************************************
 * Name: thrdfctry_getwrkr
 *
 * Description:
 *   Gets the worker thread with the specified ID.
 *
 * Input Parameters:
 *   id  The ID of the worker thread to retrieve.
 *
 * Returned Value:
 *   struct thrdpool_s pointer(i.e. instance of threadpool).
 *   If failed, returned NULL. 
 *
 ****************************************************************************/

FAR struct thrdpool_s *thrdfctry_getwrkr(uint8_t id)
{
  FAR struct thrdfctry_table_s *table = NULL;
  
  if (!g_thrdfctry_list)
    {
      DBGIF_LOG_ERROR("Work thread factory is uninitialized.\n");
      return NULL;
    }

  table = g_thrdfctry_list;
  while (table)
    {
      if (table->id == id)
        {
          return table->pool;
        }

      table = table->next;
    }

  return NULL;
}
