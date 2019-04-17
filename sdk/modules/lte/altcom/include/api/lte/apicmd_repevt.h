/****************************************************************************
 * modules/lte/altcom/include/api/lte/apicmd_repevt.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_REPEVT_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_REPEVT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "apicmd.h"
#include "apicmd_ltime.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_SET_REP_EVT_LTIME                            (0x01)
#define APICMD_SET_REP_EVT_SIMD                             (0x02)
#define APICMD_SET_REP_EVT_SIMSTATE                         (0x04)

#define APICMD_SET_REP_EVT_RES_OK                           (0)
#define APICMD_SET_REP_EVT_RES_ERR                          (1)

#define APICMD_REPORT_EVT_TYPE_LTIME                        (0)
#define APICMD_REPORT_EVT_TYPE_SIMD                         (1)
#define APICMD_REPORT_EVT_TYPE_SIMSTATE                     (2)

#define APICMD_REPORT_EVT_SIMD_REMOVAL                      (0)
#define APICMD_REPORT_EVT_SIMD_INSERTION                    (1)

#define APICMD_REPORT_EVT_SIMSTATE_DEACTIVATED              (0)
#define APICMD_REPORT_EVT_SIMSTATE_SIM_INIT_WAIT_PIN_UNLOCK (1)
#define APICMD_REPORT_EVT_SIMSTATE_PERSONALIZATION_FAILED   (2)
#define APICMD_REPORT_EVT_SIMSTATE_ACTIVATION_COMPLETED     (3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

begin_packed_struct struct apicmd_cmddat_setrepevt_s
{
  uint8_t event;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setrepevtres_s
{
  uint8_t result;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repevt_simd_s
{
  uint8_t status;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repevt_simstate_s
{
  uint8_t state;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repevt_s
{
  uint8_t type;
  begin_packed_struct union
  {
    struct apicmd_cmddat_ltime_s           ltime;
    struct apicmd_cmddat_repevt_simd_s     simd;
    struct apicmd_cmddat_repevt_simstate_s simstate;
  } end_packed_struct u;
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_REPEVT_H */
