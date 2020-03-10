/****************************************************************************
 * modules/lte/altcom/include/api/lte/apicmd_edrx.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_EDRX_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_EDRX_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Access technology is not using eDRX */

#define APICMD_EDRX_ACTTYPE_NOTUSE   (0) /* eDRX is not running */
#define APICMD_EDRX_ACTTYPE_ECGSMIOT (1) /* EC-GSM-IoT (A/Gb mode) */
#define APICMD_EDRX_ACTTYPE_GSM      (2) /* GSM (A/Gb mode) */
#define APICMD_EDRX_ACTTYPE_IU       (3) /* UTRAN (Iu mode) */
#define APICMD_EDRX_ACTTYPE_WBS1     (4) /* E-UTRAN (WB-S1 mode) */
#define APICMD_EDRX_ACTTYPE_NBS1     (5) /* E-UTRAN (NB-S1 mode) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct apicmd_edrxset_s
{
  uint8_t acttype;
  uint8_t enable;
  uint8_t edrx_cycle;
  uint8_t ptw_val;
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_EDRX_H */
