/****************************************************************************
 * modules/lte/altcom/include/api/lte/apicmd_rat.h
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_RAT_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_RAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_RAT_RES_OK            (0)
#define APICMD_RAT_RES_ERR           (1)
#define APICMD_RAT_RAT_DEFAULT       (1)
#define APICMD_RAT_RAT_CATM          (2)
#define APICMD_RAT_RAT_NBIOT         (3)
#define APICMD_RAT_RAT_GSM           (4)
#define APICMD_RAT_RAT_C2D           (5)
#define APICMD_RAT_RAT_N2D           (6)
#define APICMD_RAT_RAT_G2D           (7)
#define APICMD_RAT_NOT_PERSIS        (0)
#define APICMD_RAT_PERSIS            (1)
#define APICMD_RAT_RAT_MODE_SINGLE   (0)
#define APICMD_RAT_RAT_MODE_MULTIPLE (1)
#define APICMD_RAT_SOURCE_NONE       (0)
#define APICMD_RAT_SOURCE_HOST       (1)
#define APICMD_RAT_SOURCE_LWM2M      (2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

begin_packed_struct struct apicmd_cmddat_setrat_s
{
  uint8_t rat;
  uint8_t persistency;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_setratres_s
{
  uint8_t result;
} end_packed_struct;

/* APICMDID_GET_RAT
 * no data
 */

begin_packed_struct struct apicmd_cmddat_getratres_s
{
  uint8_t result;
  uint8_t rat;
  uint8_t rat_mode;
  uint8_t source;
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_RAT_H */
