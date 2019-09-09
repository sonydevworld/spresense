/****************************************************************************
 * modules/lte/altcom/include/api/lte/apicmd_getsiminfo.h
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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


#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_GETSIMINFO_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_GETSIMINFO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_GETSIMINFO_MCC_DIXIT_MAX          (3)
#define APICMD_GETSIMINFO_MNC_DIXIT_MAX          (3)
#define APICMD_GETSIMINFO_SPN_MAX_LEN            (16)
#define APICMD_GETSIMINFO_ICCID_MAX_LEN          (10)
#define APICMD_GETSIMINFO_IMSI_LEN               (15)
#define APICMD_GETSIMINFO_GID_LEN                (128)

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct apicmd_cmddat_getsiminfo_s
{
  uint32_t option;
} end_packed_struct;

begin_packed_struct struct apicmd_cmddat_getsiminfo_res_s
{
  uint8_t  result;
  uint32_t option;
  uint8_t  mcc[APICMD_GETSIMINFO_MCC_DIXIT_MAX];
  uint8_t  mnc_digit;
  uint8_t  mnc[APICMD_GETSIMINFO_MNC_DIXIT_MAX];
  uint8_t  spn_len;
  uint8_t  spn[APICMD_GETSIMINFO_SPN_MAX_LEN];
  uint8_t  iccid_len;
  uint8_t  iccid[APICMD_GETSIMINFO_ICCID_MAX_LEN];
  uint8_t  imsi_len;
  uint8_t  imsi[APICMD_GETSIMINFO_IMSI_LEN];
  uint8_t  gid1_len;
  uint8_t  gid1[APICMD_GETSIMINFO_GID_LEN];
  uint8_t  gid2_len;
  uint8_t  gid2[APICMD_GETSIMINFO_GID_LEN];
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_GETSIMINFO_H */
