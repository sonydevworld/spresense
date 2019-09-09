/****************************************************************************
 * modules/lte/altcom/evtdisp/evtdispfctry.c
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
#include "evtdispfctry.h"
#include "buffpoolwrapper.h"
#include "evtdisp.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct evtdispfctry_table_s
{
  uint8_t                         id;
  FAR struct evtdisp_obj_s        *disp;
  FAR struct evtdispfctry_table_s *next;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct evtdispfctry_table_s *g_disp_tbl = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: evtdispfctry_init
 *
 * Description:
 *   Initialize the event dispatcher.
 *
 * Input Parameters:
 *   set     Parameter list of the event dispatcher to be created.
 *   setnum  Number of @set.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t evtdispfctry_init(struct evtdispfctry_evtdispset_s set[],
                           int8_t setnum)
{
  int8_t                          i;
  int32_t                         ret         = -EINVAL;
  FAR struct evtdisp_obj_s        *obj        = NULL;
  FAR struct evtdispfctry_table_s *disp_block = NULL;
  FAR struct evtdispfctry_table_s *table      = g_disp_tbl;

  for (i = 0; i < setnum; i++)
    {
      disp_block = (FAR struct evtdispfctry_table_s *)
        BUFFPOOL_ALLOC(sizeof(struct evtdispfctry_table_s));
      if (!disp_block)
        {
          DBGIF_LOG_ERROR("BUFFPOOL_ALLOC() failed.\n");
          return -ENOMEM;
        }

      obj = (FAR struct evtdisp_obj_s *)evtdisp_create(set[i].evthdllist);
      if (!obj)
        {
          BUFFPOOL_FREE(disp_block);
          ret = -1;
          goto errout_with_fin;
        }
      
      disp_block->id   = set[i].dispid;
      disp_block->disp = obj;
      disp_block->next = NULL;
      ret = 0;

      /* Add table */

      if (!g_disp_tbl)
        {
          g_disp_tbl = disp_block;
        }
      else
        {
          table = g_disp_tbl;
          while (table->next)
            {
              table = table->next;
            }

          table->next = disp_block;
        }
    }

  return ret;

errout_with_fin:
  evtdispfctry_fin();

  return ret;
}

/****************************************************************************
 * Name: evtdispfctry_fin
 *
 * Description:
 *   Finalize the event dispatcher.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t evtdispfctry_fin(void)
{
  int32_t ret;
  FAR struct evtdispfctry_table_s *tgt  = NULL;
  FAR struct evtdispfctry_table_s *next = NULL;

  if (!g_disp_tbl)
    {
      DBGIF_LOG_ERROR("dispatcher is not created.\n");
      return 0;
    }

  tgt = g_disp_tbl;
  while (tgt)
    {
      next = tgt->next;
      ret = evtdisp_delete((FAR struct evtdisp_s *)tgt->disp);
      if (0 > ret)
        {
          DBGIF_LOG_ERROR("evtdispfctry_remallhdllist() failed.\n");
        }

      (void)BUFFPOOL_FREE(tgt);
      tgt = next;
    }

  g_disp_tbl = NULL;

  return ret;
}

/****************************************************************************
 * Name: evtdispfctry_get_instance
 *
 * Description:
 *   Gets theevent dispatcher with the specified ID.
 *
 * Input Parameters:
 *   id  The ID of the event dispatcher to retrieve.
 *
 * Returned Value:
 *   Instance of event dispatcher.
 *   If failed, returned NULL.
 *
 ****************************************************************************/

struct evtdisp_s *evtdispfctry_get_instance(uint8_t id)
{
  FAR struct evtdisp_s            *disp  = NULL;
  FAR struct evtdispfctry_table_s *block = NULL;

  if (!g_disp_tbl)
    {
      return NULL;
    }

  block = g_disp_tbl;
  while (block)
    {
      if (block->id == id)
        {
          disp = (FAR struct evtdisp_s *)block->disp;
          break;
        }

      block = block->next;
    }

  return disp;
}
