/****************************************************************************
 * modules/lte/altcom/evtdisp/evtdisp.c
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
#include "dbg_if.h"
#include "buffpool.h"
#include "evtdisp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EVTDISP_EXLIST_MAX (0x0A)

#define EVTDISP_BUFFPOOL_ALLOC(pool, size)(buffpool_alloc(pool, size))
#define EVTDISP_BUFFPOOL_FREE(pool, buff)(buffpool_free(pool, buff))

#define EVTDISP_DISPATCH(ret, hdlr) \
  do \
    { \
      while (EVTDISP_EVTHDLLIST_TERMINATION != *hdlr) \
        { \
          const enum evthdlrc_e retcode = (*hdlr)(evt, evtln); \
          if (EVTHDLRC_STARTHANDLE == retcode) \
            { \
              ret = 0; \
              break; \
            } \
          hdlr++; \
        } \
    } \
  while (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct evtdisp_exhdl_s
{
  uint8_t                    id;
  FAR evthdl_if_t            *evthdllist;
  FAR struct evtdisp_exhdl_s *next;
};

struct evtdisp_obj_s
{
  struct evtdisp_s           evtdispif;
  FAR buffpool_t             buffpool;
  FAR evthdl_if_t            *evthdllist;
  FAR struct evtdisp_exhdl_s *exhdllist;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int32_t evtdisp_dispatch(FAR struct evtdisp_s *thiz,
                        FAR uint8_t *evt, uint32_t evtln);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: evtdisp_dispatch
 *
 * Description:
 *  Dispatch event.
 *
 * Input Parameters: 
 *  thiz  EVTDISP object pointer.
 *  evt  event.
 *  evtln  length of event.
 *
 * Returned Value: 
 *  result of dispatch event.
 *
 ****************************************************************************/

static int32_t evtdisp_dispatch(FAR struct evtdisp_s *thiz,
                        FAR uint8_t *evt, uint32_t evtln)
{
  int32_t                    ret         = -EINVAL;
  FAR struct evtdisp_obj_s   *obj        = (FAR struct evtdisp_obj_s *)thiz;
  FAR evthdl_if_t            *evthandler = NULL;
  FAR struct evtdisp_exhdl_s *exhdl      = NULL;

  if (!thiz)
    {
      DBGIF_LOG_ERROR("NULL parameter\n");
      return -EINVAL;
    }

  evthandler = obj->evthdllist;

  EVTDISP_DISPATCH(ret, evthandler);

  if (0 != ret)
    {
      exhdl = obj->exhdllist;
      while (exhdl)
        {
          evthandler = exhdl->evthdllist;
          EVTDISP_DISPATCH(ret, evthandler);
          if (0 == ret)
            {
              break;
            }

          exhdl = exhdl->next;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: evtdisp_addhdlr
 *
 * Description:
 *  Add event handler for evet dispatcher.
 *
 * Input Parameters:
 *  thiz       Event dispatcher object pointer.
 *  hdllist    Event handler list.
 *
 * Returned Value:
 *   If the process succeeds, it returns ex hamdler id.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

static int32_t evtdisp_addhdlr(FAR struct evtdisp_s *thiz,
  FAR evthdl_if_t *hdllist)
{
  FAR struct evtdisp_obj_s   *obj    = NULL;
  FAR struct evtdisp_exhdl_s *exhdl  = NULL;
  FAR struct evtdisp_exhdl_s *tmphdl = NULL;
  FAR struct evtdisp_exhdl_s *prvhdl = NULL;
  uint8_t                    i;

  /* Check input param. */

  if (!thiz || !hdllist)
    {
      DBGIF_LOG_ERROR("NULL parameter\n");
      return -EINVAL;
    }

  obj = (FAR struct evtdisp_obj_s *)thiz;

  /* Alloc memory. */

  exhdl = (FAR struct evtdisp_exhdl_s *)
    EVTDISP_BUFFPOOL_ALLOC(obj->buffpool, sizeof(struct evtdisp_exhdl_s));
  if (!exhdl)
    {
      DBGIF_LOG_ERROR("EVTDISP_BUFFPOOL_ALLOC() failed.\n");
      return -ENOMEM;
    }

  /* Fill parameter. */

  exhdl->evthdllist = hdllist;
  exhdl->next = NULL;

  /* Add hdlr list */

  exhdl->id = 0;
  if (!obj->exhdllist)
    {
      obj->exhdllist = exhdl;
    }
  else
    {
      tmphdl = obj->exhdllist;

      for (i = 0; i < EVTDISP_EXLIST_MAX; i++)
        {
          if (i != tmphdl->id)
            {
              exhdl->id = i;
              exhdl->next = tmphdl;
              prvhdl->next = exhdl;
              break;
            }
          else
            {
              if (!tmphdl->next)
                {
                  exhdl->id = i + 1;
                  tmphdl->next = exhdl;
                  break;
                }
            }

          prvhdl = tmphdl;
          tmphdl = tmphdl->next;
        }
    }

  return exhdl->id;
}

/****************************************************************************
 * Name: evtdisp_rmvhdlr
 *
 * Description:
 *  Add event handler for evet dispatcher.
 *
 * Input Parameters:
 *  thiz       Event dispatcher object pointer.
 *  id         Event handler list id.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

static int32_t evtdisp_rmvhdlr(FAR struct evtdisp_s *thiz, uint8_t id)
{
  int32_t                    ret      = -EINVAL;
  FAR struct evtdisp_obj_s   *obj     = NULL;
  FAR struct evtdisp_exhdl_s *exhdl   = NULL;
  FAR struct evtdisp_exhdl_s *prevtbl = NULL;
  FAR struct evtdisp_exhdl_s *tgttbl  = NULL;

  /* Check input param. */

  if (!thiz)
    {
      DBGIF_LOG_ERROR("NULL parameter\n");
      return -EINVAL;
    }

  obj = (FAR struct evtdisp_obj_s *)thiz;
  exhdl = obj->exhdllist;
  if (!exhdl)
    {
      DBGIF_LOG_ERROR("hdl_table is null\n");
      return -EINVAL;
    }

  while (exhdl)
    {
      if (exhdl->id == id)
        {
          tgttbl = exhdl;
          ret = 0;

          if (prevtbl)
            {
              prevtbl->next = exhdl->next;
            }
          else
            {
              obj->exhdllist = exhdl->next;
            }

          break;
        }

      prevtbl = exhdl;
      exhdl = exhdl->next;
    }

  if (ret < 0)
    {
      DBGIF_LOG1_ERROR("handl list not found id %d\n", id);
    }
  else
    {
      (void)EVTDISP_BUFFPOOL_FREE(obj->buffpool, tgttbl);
      tgttbl = NULL;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: evthdisp_create
 *
 * Description:
 *  Create EVTDISP object.
 *
 * Input Parameters: 
 *  evtlist  handling process function pointer.
 *
 * Returned Value: 
 *  EVTDISP object pointer.
 *
 ****************************************************************************/

FAR struct evtdisp_s *evtdisp_create(FAR evthdl_if_t *evthdllist)
{
  FAR struct evtdisp_obj_s   *obj = NULL;
  FAR buffpool_t             pool = NULL;
  struct buffpool_blockset_s buffset[] =
    {
      { sizeof(struct evtdisp_obj_s), 1 },
      { sizeof(struct evtdisp_exhdl_s), EVTDISP_EXLIST_MAX }
    };

  pool = buffpool_create(buffset, sizeof(buffset) / sizeof(buffset[0]));
  if (!pool)
    {
      DBGIF_LOG_ERROR("buffpool_create() failed\n");
      return NULL;
    }

  obj = (FAR struct evtdisp_obj_s *)EVTDISP_BUFFPOOL_ALLOC(pool, sizeof(struct evtdisp_obj_s));
  if (!obj)
    {
      DBGIF_LOG_ERROR("EVTDISP_BUFFPOOL_ALLOC() failed\n");
      (void)buffpool_delete(pool);
      return NULL;
    }

  obj->evthdllist         = evthdllist;
  obj->exhdllist          = NULL;
  obj->buffpool           = pool;
  obj->evtdispif.dispatch = evtdisp_dispatch;
  obj->evtdispif.addhdlr  = evtdisp_addhdlr;
  obj->evtdispif.rmvhdlr  = evtdisp_rmvhdlr;

  return (FAR struct evtdisp_s *)obj;
}

/****************************************************************************
 * Name: evthdisp_delete
 *
 * Description:
 *  Delete EVTDISP object.
 *
 * Input Parameters: 
 *  thiz  EVTDISP object pointer.
 *
 * Returned Value: 
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t evtdisp_delete(FAR struct evtdisp_s *thiz)
{
  FAR struct evtdisp_obj_s   *obj = (FAR struct evtdisp_obj_s *)thiz;
  FAR struct evtdisp_exhdl_s *exhdl = NULL;
  FAR struct evtdisp_exhdl_s *tgt = NULL;
  FAR buffpool_t             pool = NULL;

  if (!thiz)
    {
      DBGIF_LOG_ERROR("NULL parameter\n");
      return -EINVAL;
    }

  if (obj->exhdllist)
    {
      exhdl = obj->exhdllist;
      while(exhdl)
        {
          tgt = exhdl;
          exhdl = exhdl->next;
          (void)EVTDISP_BUFFPOOL_FREE(obj->buffpool, tgt);
        }
    }

  pool = obj->buffpool;
  EVTDISP_BUFFPOOL_FREE(pool, obj);
  obj = NULL;
  buffpool_delete(pool);

  return 0;
}
