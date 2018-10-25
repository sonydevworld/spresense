/****************************************************************************
 * modules/lte/altcom/include/api/lte/apicmd_getpinset.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_GETPINSET_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_GETPINSET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_GETPINSET_RES_OK                (0)
#define APICMD_GETPINSET_RES_ERR               (1)

#define APICMD_GETPINSET_ACTIVE_DISABLE        (0)
#define APICMD_GETPINSET_ACTIVE_ENABLE         (1)

#define APICMD_GETPINSET_STATUS_READY          (0)
#define APICMD_GETPINSET_STATUS_SIM_PIN        (1)
#define APICMD_GETPINSET_STATUS_SIM_PUK        (2)
#define APICMD_GETPINSET_STATUS_PH_SIM_PIN     (3)
#define APICMD_GETPINSET_STATUS_PH_FSIM_PIN    (4)
#define APICMD_GETPINSET_STATUS_PH_FSIM_PUK    (5)
#define APICMD_GETPINSET_STATUS_SIM_PIN2       (6)
#define APICMD_GETPINSET_STATUS_SIM_PUK2       (7)
#define APICMD_GETPINSET_STATUS_PH_NET_PIN     (8)
#define APICMD_GETPINSET_STATUS_PH_NET_PUK     (9)
#define APICMD_GETPINSET_STATUS_PH_NETSUB_PIN  (10)
#define APICMD_GETPINSET_STATUS_PH_NETSUB_PUK  (11)
#define APICMD_GETPINSET_STATUS_PH_SP_PIN      (12)
#define APICMD_GETPINSET_STATUS_PH_SP_PUK      (13)
#define APICMD_GETPINSET_STATUS_PH_CORP_PIN    (14)
#define APICMD_GETPINSET_STATUS_PH_CORP_PUK    (15)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

/* APICMDID_GET_PINSET
 * no data
 */

begin_packed_struct struct apicmd_cmddat_getpinsetres_s
{
  uint8_t result;
  uint8_t active;
  uint8_t status;
  uint8_t pin_attemptsleft;
  uint8_t puk_attemptsleft;
  uint8_t pin2_attemptsleft;
  uint8_t puk2_attemptsleft;
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_GETPINSET_H */
