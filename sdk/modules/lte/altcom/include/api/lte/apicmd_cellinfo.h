/****************************************************************************
 * modules/lte/altcom/include/api/lte/apicmd_cellinfo.h
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_CELLINFO_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_CELLINFO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "lte/lte_api.h"
#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_CELLINFO_CELLID_MIN    (0)
#define APICMD_CELLINFO_CELLID_MAX    (503)
#define APICMD_CELLINFO_EARFCN_MIN    (0)
#define APICMD_CELLINFO_EARFCN_MAX    (262143)
#define APICMD_CELLINFO_DIGIT_NUM_MIN (0)
#define APICMD_CELLINFO_DIGIT_NUM_MAX (9)
#define APICMD_CELLINFO_MCC_DIGIT             (3)
#define APICMD_CELLINFO_MNC_DIGIT_MIN         (2)
#define APICMD_CELLINFO_MNC_DIGIT_MAX         (3)
#define APICMD_CELLINFO_GCID_MAX              (16)
#define APICMD_CELLINFO_TIMEDIFF_INDEX_MAX    (4095)
#define APICMD_CELLINFO_TA_MAX                (1282)
#define APICMD_CELLINFO_SFN_MAX               (0x03FF)
#define APICMD_CELLINFO_NEIGHBOR_CELL_NUM_MAX (32)
#define APICMD_CELLINFO_VALID_TIMEDIFFIDX     (1 << 1)
#define APICMD_CELLINFO_VALID_TA              (1 << 2)
#define APICMD_CELLINFO_VALID_SFN             (1 << 3)
#define APICMD_CELLINFO_VALID_RSRP            (1 << 4)
#define APICMD_CELLINFO_VALID_RSRQ            (1 << 5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

begin_packed_struct struct apicmd_cmddat_cellinfo_s
{
  uint8_t  valid;
  uint32_t cell_id;
  uint32_t earfcn;
  uint8_t  mcc[LTE_MCC_DIGIT];
  uint8_t  mnc_digit;
  uint8_t  mnc[LTE_MNC_DIGIT_MAX];
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_neighbor_cell_s
{
  uint8_t  valid;
  uint32_t cell_id;
  uint32_t earfcn;

  /* When setting "sfn",
   * APICMD_CELLINFO_VALID_SFN flag has been added to "valid".
   */

  uint16_t sfn;

  /* When setting "rsrp",
   * APICMD_CELLINFO_VALID_RSRP flag has been added to "valid".
   */

  int16_t  rsrp;

  /* When setting "rsrq",
   * APICMD_CELLINFO_VALID_RSRQ flag has been added to "valid".
   */

  int16_t  rsrq;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_cellinfo_v4_s
{
  uint8_t enability;
  uint32_t cell_id;
  uint32_t earfcn;
  uint8_t mcc[APICMD_CELLINFO_MCC_DIGIT];
  uint8_t mnc_digit;
  uint8_t mnc[APICMD_CELLINFO_MNC_DIGIT_MAX];
  uint8_t cgid[APICMD_CELLINFO_GCID_MAX + 1];
  uint16_t tac;

  /* When setting "time_diffidx",
   * APICMD_CELLINFO_VALID_TIMEDIFFIDX flag has been added to "valid".
   */

  uint16_t time_diffidx;

  /* When setting "ta",
   * APICMD_CELLINFO_VALID_TA flag has been added to "valid".
   */

  uint16_t ta;

  /* When setting "sfn",
   * APICMD_CELLINFO_VALID_SFN flag has been added to "valid".
   */

  uint16_t sfn;

  /* When setting "rsrp",
   * APICMD_CELLINFO_VALID_RSRP flag has been added to "valid".
   */

  int16_t rsrp;

  /* When setting "rsrq",
   * APICMD_CELLINFO_VALID_RSRQ flag has been added to "valid".
   */

  int16_t rsrq;

  uint8_t neighbor_num;
  struct apicmd_cmddat_neighbor_cell_s
           neighbor_cell[APICMD_CELLINFO_NEIGHBOR_CELL_NUM_MAX];
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_CELLINFO_H */
