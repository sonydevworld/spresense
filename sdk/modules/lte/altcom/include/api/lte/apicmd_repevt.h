/****************************************************************************
 * modules/lte/altcom/include/api/lte/apicmd_repevt.h
 *
 *   Copyright 2018, 2020 Sony Semiconductor Solutions Corporation
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
#include "apicmd_psm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_SET_REP_EVT_DISABLE                          (0)
#define APICMD_SET_REP_EVT_ENABLE                           (1)

#define APICMD_SET_REP_EVT_LTIME                            (1 << 0)
#define APICMD_SET_REP_EVT_SIMD                             (1 << 1)
#define APICMD_SET_REP_EVT_SIMSTATE                         (1 << 2)
#define APICMD_SET_REP_EVT_REGSTATE                         (1 << 3)
#define APICMD_SET_REP_EVT_PSMSTATE                         (1 << 4)
#define APICMD_SET_REP_EVT_DYNPSM                           (1 << 5)
#define APICMD_SET_REP_EVT_DYNEDRX                          (1 << 6)
#define APICMD_SET_REP_EVT_CONNPHASE                        (1 << 7)
#define APICMD_SET_REP_EVT_ANTITAMPER                       (1 << 8)
#define APICMD_SET_REP_EVT_MAX                              (1 << 15)

#define APICMD_SET_REP_EVT_RES_OK                           (0)
#define APICMD_SET_REP_EVT_RES_ERR                          (1)

#define APICMD_REPORT_EVT_TYPE_LTIME                        (0)
#define APICMD_REPORT_EVT_TYPE_SIMD                         (1)
#define APICMD_REPORT_EVT_TYPE_SIMSTATE                     (2)
#define APICMD_REPORT_EVT_TYPE_REGSTATE                     (3)
#define APICMD_REPORT_EVT_TYPE_PSMSTATE                     (4)
#define APICMD_REPORT_EVT_TYPE_DYNPSM                       (5)
#define APICMD_REPORT_EVT_TYPE_DYNEDRX                      (6)
#define APICMD_REPORT_EVT_TYPE_CONNPHASE                    (7)
#define APICMD_REPORT_EVT_TYPE_ANTITAMPER                   (8)

#define APICMD_REPORT_EVT_SIMD_REMOVAL                      (0)
#define APICMD_REPORT_EVT_SIMD_INSERTION                    (1)

#define APICMD_REPORT_EVT_SIMSTATE_DEACTIVATED              (0)
#define APICMD_REPORT_EVT_SIMSTATE_SIM_INIT_WAIT_PIN_UNLOCK (1)
#define APICMD_REPORT_EVT_SIMSTATE_PERSONALIZATION_FAILED   (2)
#define APICMD_REPORT_EVT_SIMSTATE_ACTIVATION_COMPLETED     (3)

#define APICMD_REPORT_EVT_ALERTU_TEMP                       (0)
#define APICMD_REPORT_EVT_ALERTU_ANTIAMPER                  (1)

#define APICMD_REP_PSMST_NOT_ACTIVE                         (0)
#define APICMD_REP_PSMST_ACTIVE                             (1)
#define APICMD_REP_PSMST_ACTIVE_AND_CAMPED                  (2)
#define APICMD_REP_PSMST_CAMP_INTERRUPTED                   (3)

#define APICMD_REP_CONNPHASE_START_SCAN                     (0)
#define APICMD_REP_CONNPHASE_FAIL_SCAN                      (1)
#define APICMD_REP_CONNPHASE_ENTER_CAMPED                   (2)
#define APICMD_REP_CONNPHASE_CONNECTION_ESTABLISHMENT       (3)
#define APICMD_REP_CONNPHASE_START_RESCAN                   (4)
#define APICMD_REP_CONNPHASE_ENTER_CONNECTED                (5)
#define APICMD_REP_CONNPHASE_NO_SUITABLE_CELL               (6)
#define APICMD_REP_CONNPHASE_REG_ATTEMPT_FAILED             (7)
#define APICMD_REP_CONNPHASE_NOT_AVAIL                      (99)

#define APICMD_REP_CONPHASE_RAT_CATM                        (0)
#define APICMD_REP_CONPHASE_RAT_NBIOT                       (1)
#define APICMD_REP_CONPHASE_RAT_GSM                         (3)
#define APICMD_REP_CONPHASE_RAT_NOT_AVAIL                   (99)

#define APICMD_REP_CONPHASE_SCAN_MRU_ONLY                   (0)
#define APICMD_REP_CONPHASE_SCAN_MRU_AND_FULL_SCAN          (1)
#define APICMD_REP_CONPHASE_SCAN_MRU_AND_COUNTRY_SCAN       (2)
#define APICMD_REP_CONPHASE_SCAN_MRU_AND_LS                 (3)
#define APICMD_REP_CONPHASE_SCAN_MRU_NOT_AVAIL              (99)

#define APICMD_REP_CONPHASE_SCAN_REASON_INIT_SCAN           (0)
#define APICMD_REP_CONPHASE_SCAN_REASON_OUT_OF_COVERAGE     (1)
#define APICMD_REP_CONPHASE_SCAN_REASON_HIGH_PRIORITY       (2)
#define APICMD_REP_CONPHASE_SCAN_REASON_LIMITED_SERVICE     (3)
#define APICMD_REP_CONPHASE_SCAN_REASON_COPS                (4)
#define APICMD_REP_CONPHASE_SCAN_REASON_OTHER               (5)
#define APICMD_REP_CONPHASE_SCAN_REASON_NOT_AVAIL           (99)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

begin_packed_struct struct apicmd_cmddat_setrepevt_s
{
  uint8_t event;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setrepevt_v4_s
{
  uint16_t event;
  uint8_t enability;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setrepevtres_s
{
  uint8_t result;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setrepevtres_v4_s
{
  uint8_t result;
  uint16_t event;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repevt_simd_s
{
  uint8_t status;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repevt_simstate_s
{
  uint8_t state;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repregstate_s
{
  uint8_t state;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_reppsmstate_s
{
  uint8_t state;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repantitamper_s
{
  uint8_t data;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repdynpsm_s
{
  struct apicmd_cmddat_psm_timeval_s at_val;
  struct apicmd_cmddat_psm_timeval_s tau_val;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repdynedrx_s
{
  uint8_t acttype;
  uint8_t edrx_cycle;
  uint8_t ptw_val;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_repconnphase_s
{
  uint8_t state;
  uint8_t rat;
  uint8_t scantype;
  uint8_t scanreason;
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

begin_packed_struct struct apicmd_cmddat_repevt_v4_s
{
  uint16_t type;
  begin_packed_struct union
  {
    struct apicmd_cmddat_ltime_s ltime;
    struct apicmd_cmddat_repevt_simd_s simd;
    struct apicmd_cmddat_repevt_simstate_s simstate;
    struct apicmd_cmddat_repregstate_s regstate;
    struct apicmd_cmddat_reppsmstate_s psmstate;
    struct apicmd_cmddat_repdynpsm_s dynpsm;
    struct apicmd_cmddat_repdynedrx_s dynedrx;
    struct apicmd_cmddat_repconnphase_s connphase;
    struct apicmd_cmddat_repantitamper_s antitamper;
  } end_packed_struct u;
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_REPEVT_H */
