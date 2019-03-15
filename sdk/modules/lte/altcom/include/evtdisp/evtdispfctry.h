/****************************************************************************
 * modules/lte/altcom/include/evtdisp/evtdispfctry.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_EVTDISP_EVTDISPFCTRY_H
#define __MODULES_LTE_ALTCOM_INCLUDE_EVTDISP_EVTDISPFCTRY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "evthdl_if.h"
#include "evtdisp.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct evtdispfctry_evtdispset_s
{
  uint8_t     dispid;
  evthdl_if_t *evthdllist;
};

/****************************************************************************
 * Public Function Prototypes
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
                           int8_t setnum);

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

int32_t evtdispfctry_fin(void);

/****************************************************************************
 * Name: evtdispfctry_get_instance
 *
 * Description:
 *   Gets event dispatcher with the specified ID.
 *
 * Input Parameters:
 *   id  The ID of the event dispatcher to retrieve.
 *
 * Returned Value:
 *   Instance of event dispatcher.
 *   If failed, returned NULL.
 *
 ****************************************************************************/

struct evtdisp_s *evtdispfctry_get_instance(uint8_t id);

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_EVTDISP_EVTDISPFCTRY_H */
