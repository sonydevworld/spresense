/****************************************************************************
 * modules/lte/altcom/include/api/socket/apicmd_recv.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_APICMD_RECV_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_APICMD_RECV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "altcom_socket.h"
#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_RECV_FLAGS_MSG_PEEK      (ALTCOM_MSG_PEEK)
#define APICMD_RECV_FLAGS_MSG_WAITALL   (ALTCOM_MSG_WAITALL)
#define APICMD_RECV_FLAGS_MSG_OOB       (ALTCOM_MSG_OOB)
#define APICMD_RECV_FLAGS_MSG_DONTWAIT  (ALTCOM_MSG_DONTWAIT)
#define APICMD_RECV_FLAGS_MSG_MORE      (ALTCOM_MSG_MORE)
#define APICMD_RECV_RES_RECVDATA_LENGTH (1500)

#define APICMD_RECV_RES_RET_CODE_OK  (0)
#define APICMD_RECV_RES_RET_CODE_ERR (-1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

begin_packed_struct struct apicmd_recv_s
{
  int32_t sockfd;
  int32_t recvlen;
  int32_t flags;
} end_packed_struct;

begin_packed_struct struct apicmd_recvres_s
{
  int32_t ret_code;
  int32_t err_code;
  int8_t recvdata[APICMD_RECV_RES_RECVDATA_LENGTH];
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_APICMD_RECV_H */
