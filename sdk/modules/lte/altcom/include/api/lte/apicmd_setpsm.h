/****************************************************************************
 * modules/lte/altcom/include/api/lte/apicmd_setpsm.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_SETPSM_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_SETPSM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_SETPSM_DISABLE          (0)
#define APICMD_SETPSM_ENABLE           (1)

#define APICMD_SETPSM_TIMER_MIN        (1)
#define APICMD_SETPSM_TIMER_MAX        (31)

/* Unit value of Requested Active Time */

#define APICMD_SETPSM_RAT_UNIT_2SEC    (0)
#define APICMD_SETPSM_RAT_UNIT_1MIN    (1)
#define APICMD_SETPSM_RAT_UNIT_6MIN    (2)

/* Unit value of extended periodic Tracking Area Update */

#define APICMD_SETPSM_TAU_UNIT_2SEC    (0)
#define APICMD_SETPSM_TAU_UNIT_30SEC   (1)
#define APICMD_SETPSM_TAU_UNIT_1MIN    (2)
#define APICMD_SETPSM_TAU_UNIT_10MIN   (3)
#define APICMD_SETPSM_TAU_UNIT_1HOUR   (4)
#define APICMD_SETPSM_TAU_UNIT_10HOUR  (5)
#define APICMD_SETPSM_TAU_UNIT_320HOUR (6)

#define APICMD_SETPSM_RES_OK           (0)
#define APICMD_SETPSM_RES_ERR          (1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

/* APICMDID_GET_PSM */

begin_packed_struct struct apicmd_cmddat_setpsm_timeval_s
{
  uint8_t unit;
  uint8_t time_val;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setpsm_s
{
  uint8_t enable;
  struct apicmd_cmddat_setpsm_timeval_s rat_val;
  struct apicmd_cmddat_setpsm_timeval_s tau_val;
} end_packed_struct;

/* APICMDID_GET_PSM_RES */

begin_packed_struct struct apicmd_cmddat_setpsmres_s
{
  uint8_t result;
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_SETPSM_H */
