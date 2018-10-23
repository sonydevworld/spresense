/****************************************************************************
 * modules/lte/altcom/include/api/lte/apicmd_setedrx.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_SETEDRX_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_SETEDRX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_SETEDRX_ACTTYPE_WBS1 (4)

#define APICMD_SETEDRX_DISABLE      (0)
#define APICMD_SETEDRX_ENABLE       (1)

#define APICMD_SETEDRX_CYC_512      (0) /**< eDRX cycle     5.12 sec */
#define APICMD_SETEDRX_CYC_1024     (1) /**< eDRX cycle    10.24 sec */
#define APICMD_SETEDRX_CYC_2048     (2) /**< eDRX cycle    20.48 sec */
#define APICMD_SETEDRX_CYC_4096     (3) /**< eDRX cycle    40.96 sec */
#define APICMD_SETEDRX_CYC_6144     (4) /**< eDRX cycle    61.44 sec */
#define APICMD_SETEDRX_CYC_8192     (5) /**< eDRX cycle    81.92 sec */
#define APICMD_SETEDRX_CYC_10240    (6) /**< eDRX cycle   102.40 sec */
#define APICMD_SETEDRX_CYC_12288    (7) /**< eDRX cycle   122.88 sec */
#define APICMD_SETEDRX_CYC_14336    (8) /**< eDRX cycle   143.36 sec */
#define APICMD_SETEDRX_CYC_16384    (9) /**< eDRX cycle   163.84 sec */
#define APICMD_SETEDRX_CYC_32768   (10) /**< eDRX cycle   327.68 sec */
#define APICMD_SETEDRX_CYC_65536   (11) /**< eDRX cycle   655.36 sec */
#define APICMD_SETEDRX_CYC_131072  (12) /**< eDRX cycle  1310.72 sec */
#define APICMD_SETEDRX_CYC_262144  (13) /**< eDRX cycle  2621.44 sec */

#define APICMD_SETEDRX_PTW_128      (0) /**< Paging time window  1.28 sec */
#define APICMD_SETEDRX_PTW_256      (1) /**< Paging time window  2.56 sec */
#define APICMD_SETEDRX_PTW_384      (2) /**< Paging time window  3.84 sec */
#define APICMD_SETEDRX_PTW_512      (3) /**< Paging time window  5.12 sec */
#define APICMD_SETEDRX_PTW_640      (4) /**< Paging time window  6.40 sec */
#define APICMD_SETEDRX_PTW_768      (5) /**< Paging time window  7.68 sec */
#define APICMD_SETEDRX_PTW_896      (6) /**< Paging time window  8.96 sec */
#define APICMD_SETEDRX_PTW_1024     (7) /**< Paging time window 10.24 sec */
#define APICMD_SETEDRX_PTW_1152     (8) /**< Paging time window 11.52 sec */
#define APICMD_SETEDRX_PTW_1280     (9) /**< Paging time window 12.80 sec */
#define APICMD_SETEDRX_PTW_1408    (10) /**< Paging time window 14.08 sec */
#define APICMD_SETEDRX_PTW_1536    (11) /**< Paging time window 15.36 sec */
#define APICMD_SETEDRX_PTW_1664    (12) /**< Paging time window 16.64 sec */
#define APICMD_SETEDRX_PTW_1792    (13) /**< Paging time window 17.92 sec */
#define APICMD_SETEDRX_PTW_1920    (14) /**< Paging time window 19.20 sec */
#define APICMD_SETEDRX_PTW_2048    (15) /**< Paging time window 20.48 sec */

#define APICMD_SETEDRX_RES_OK       (0)
#define APICMD_SETEDRX_RES_ERR      (1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

/* APICMDID_SET_EDRX */

begin_packed_struct struct apicmd_cmddat_setedrx_s
{
  uint8_t acttype;
  uint8_t enable;
  uint8_t edrx_cycle;
  uint8_t ptw_val;
} end_packed_struct;

/* APICMDID_SET_EDRX_RES */

begin_packed_struct struct apicmd_cmddat_setedrxres_s
{
  uint8_t result;
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_SETEDRX_H */
