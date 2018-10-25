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
#include "buffpoolwrapper.h"
#include "evtdisp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct evtdisp_obj_s
{
  struct evtdisp_s evtdispif;
  FAR evthdl_if_t  *evthdllist;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int32_t evtdisp_dispatch(FAR struct evtdisp_s *thiz,
                        FAR uint8_t *evt, uint32_t evtln);

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
  int32_t                  ret        = -EINVAL;
  enum evthdlrc_e          returncode = EVTHDLRC_UNSUPPORTEDEVENT;
  FAR struct evtdisp_obj_s *obj       = (FAR struct evtdisp_obj_s *)thiz;
  FAR evthdl_if_t          *evthandler = NULL;

  if (!thiz)
    {
      DBGIF_LOG_ERROR("NULL parameter\n");
      return -EINVAL;
    }

  evthandler = obj->evthdllist;

  while (EVTDISP_EVTHDLLIST_TERMINATION != *evthandler)
    {
      returncode = (*evthandler)(evt, evtln);
      if (EVTHDLRC_STARTHANDLE == returncode)
        {
          ret = 0;
          break;
        }
      evthandler++;
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
  FAR struct evtdisp_obj_s *obj = NULL;

  obj = (FAR struct evtdisp_obj_s *)BUFFPOOL_ALLOC(sizeof(struct evtdisp_obj_s));
  if (!obj)
    {
      DBGIF_LOG_ERROR("Memory allocate\n");
      return NULL;
    }

  obj->evthdllist         = evthdllist;
  obj->evtdispif.dispatch = evtdisp_dispatch;

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
 *  result.
 *
 ****************************************************************************/

int32_t evtdisp_delete(FAR struct evtdisp_s *thiz)
{
  FAR struct evtdisp_obj_s *obj = (FAR struct evtdisp_obj_s *)thiz;

  if (!thiz)
    {
      DBGIF_LOG_ERROR("NULL parameter\n");
      return -EINVAL;
    }

  BUFFPOOL_FREE(obj);
  obj = NULL;

  return 0;
}
  
