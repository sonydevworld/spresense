/****************************************************************************
 * modules/lte/altcom/include/api/lte/apicmdhdlrbs.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMDHDLRBS_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMDHDLRBS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "dbg_if.h"
#include "wrkrid.h"
#include "apiutil.h"
#include "evthdlbs.h"
#include "apicmdgw.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*apicmdhdlrbs_cb_job)(FAR void *arg);

/****************************************************************************
 * Inline functions
 ****************************************************************************/

static inline enum evthdlrc_e apicmdhdlrbs_do_runjob(FAR uint8_t *evt,
  uint16_t cmdid, FAR apicmdhdlrbs_cb_job job)
{

  if (!evt)
    {
      DBGIF_LOG_ERROR("NULL parameter");
      return EVTHDLRC_INTERNALERROR;
    }

  if (!apicmdgw_cmdid_compare(evt, cmdid))
    {
      return EVTHDLRC_UNSUPPORTEDEVENT;
    }

  if (0 > evthdlbs_runjob(WRKRID_API_CALLBACK_THREAD,
    (CODE thrdpool_jobif_t)job, (FAR void*)evt))
    {
      altcom_free_cmd((FAR uint8_t *)evt);
      return EVTHDLRC_INTERNALERROR;
    }

  return EVTHDLRC_STARTHANDLE;
}

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMDHDLRBS_H */
