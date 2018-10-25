/****************************************************************************
 * modules/lte/altcom/include/api/socket/apicmd_gethostbyname.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_APICMD_GETHOSTBYNAME_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_APICMD_GETHOSTBYNAME_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "altcom_socket.h"
#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_GETHOSTBYNAME_NAME_MAX_LENGTH      (256)
#define APICMD_GETHOSTBYNAME_RES_H_ALIASES_LENGTH (256)
#define APICMD_GETHOSTBYNAME_RES_H_ADDR_LENGTH    (16)

#define APICMD_GETHOSTBYNAME_RES_ADDRTYPEUNSPEC (ALTCOM_AF_UNSPEC)
#define APICMD_GETHOSTBYNAME_RES_ADDRTYPE_INET  (ALTCOM_AF_INET)
#define APICMD_GETHOSTBYNAME_RES_ADDRTYPE_INET6 (ALTCOM_AF_INET6)

#define APICMD_GETHOSTBYNAME_RES_RET_CODE_OK  (0)
#define APICMD_GETHOSTBYNAME_RES_RET_CODE_ERR (-1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

begin_packed_struct struct apicmd_gethostbyname_s
{
  uint32_t namelen;
  int8_t name[APICMD_GETHOSTBYNAME_NAME_MAX_LENGTH];
} end_packed_struct;

begin_packed_struct struct apicmd_gethostbynameres_s
{
  int32_t ret_code;
  int8_t h_name[APICMD_GETHOSTBYNAME_NAME_MAX_LENGTH];
  int8_t h_aliases[APICMD_GETHOSTBYNAME_RES_H_ALIASES_LENGTH];
  int32_t h_addrtype;
  int32_t h_length;
  int8_t h_addr_list[APICMD_GETHOSTBYNAME_RES_H_ADDR_LENGTH];
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_APICMD_GETHOSTBYNAME_H */
