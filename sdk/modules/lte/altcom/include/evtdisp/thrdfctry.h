/****************************************************************************
 * modules/lte/altcom/include/evtdisp/thrdfctry.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_EVTDISP_THRDFCTRY_H
#define __MODULES_LTE_ALTCOM_INCLUDE_EVTDISP_THRDFCTRY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "osal.h"
#include "thrdpool.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum thrdfctry_wrktyp_e
{
  THRDFCTRY_PARALLEL  = 0,
  THRDFCTRY_SEQUENTIAL,
  THRDFCTRY_WRKTYPE_NUM
};

typedef struct thrdpool_set_s thrdfctry_parawrkset_t;

struct thrdfctry_seqwrkset_s
{
  uint32_t thrdstacksize;
  int32_t  thrdpriority;
  uint8_t  maxquenum;
};

struct thrdfctry_thrdset_s
{
  uint8_t                 id;
  enum thrdfctry_wrktyp_e type;
  union
  {
    thrdfctry_parawrkset_t       paraset;
    struct thrdfctry_seqwrkset_s seqset;
  } u;
};

/****************************************************************************
 * Public Function Prototypes
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

int32_t thrdfctry_init(FAR struct thrdfctry_thrdset_s set[], int8_t setnum);

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

int32_t thrdfctry_fin(void);

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

FAR struct thrdpool_s *thrdfctry_getwrkr(uint8_t id);

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_EVTDISP_THRDFCTRY_H */
