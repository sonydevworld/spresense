/****************************************************************************
 * modules/lte/altcom/include/api/lte/apicmd_pdn.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_PDN_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_PDN_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_PDN_IMS_REG                (0)
#define APICMD_PDN_IMS_UNREG              (1)

#define APICMD_PDN_DATAALLOW_ALLOW        (0)
#define APICMD_PDN_DATAALLOW_DISALLOW     (1)

#define APICMD_PDN_DATAROAMALLOW_ALLOW    (0)
#define APICMD_PDN_DATAROAMALLOW_DISALLOW (1)

#define APICMD_PDN_IPCOUNT_MAX            (2)
#define APICMD_PDN_IPADDR_MAXLEN          (40)

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct apicmd_ipaddr_s
{
  uint8_t iptype;
  uint8_t address[APICMD_PDN_IPADDR_MAXLEN];
} end_packed_struct;

begin_packed_struct struct apicmd_pdnset_s
{
  uint8_t session_id;
  uint8_t activate;
  uint32_t apntype;
  uint8_t ipaddr_num;
  struct apicmd_ipaddr_s
    ip_address[APICMD_PDN_IPCOUNT_MAX];
  uint8_t imsregister;
  uint8_t dataallow;
  uint8_t dararoamingallow;
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_LTE_APICMD_PDN_H */
