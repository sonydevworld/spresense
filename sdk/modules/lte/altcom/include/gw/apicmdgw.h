/****************************************************************************
 * modules/lte/altcom/gw/apicmdgw.h
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

#ifndef __MODULES_LTE_ALTCOM_GW_APICMDGW_H
#define __MODULES_LTE_ALTCOM_GW_APICMDGW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stdbool.h>

#include "osal.h"
#include "hal_if.h"
#include "evtdisp.h"
#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMDGW_RECVBUFF_SIZE_MAX \
  (APICMD_PAYLOAD_SIZE_MAX + sizeof(struct apicmd_cmdhdr_s) \
    + sizeof(struct apicmd_cmdftr_s))

#define APICMDGW_SEND_ONLY(cmd) \
  (apicmdgw_send(cmd, NULL, 0, NULL, 0))

#define APICMDGW_REPLY(cmd) \
  (apicmdgw_send(cmd, NULL, 0, NULL, 0))

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct apicmdgw_set_s
{
  FAR struct hal_if_s  *halif;
  FAR struct evtdisp_s *dispatcher;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: apicmdgw_init
 *
 * Description:
 *   Initialize api command gate way instance.
 *
 * Input Parameters:
 *   set    Initialize setting struct pointer.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t apicmdgw_init(FAR struct apicmdgw_set_s *set);

/****************************************************************************
 * Name: apicmdgw_fin
 *
 * Description:
 *   Finalize api command gate way instance.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t apicmdgw_fin(void);

/****************************************************************************
 * Name: apicmdgw_send
 *
 * Description:
 *   Send api command.
 *   And wait to response for parameter of timeout_ms value when
 *   parameter of respbuff set valid buffer.
 *   Non wait to response when parameter of respbuff set NULL.
 *
 * Input Parameters:
 *   cmd         Send command payload pointer.
 *   respbuff    Response buffer.
 *   bufflen     @respbuff length.
 *   resplen     Response length.
 *   timeout_ms  Response wait timeout value (msec).
 *               When use SYS_TIMEO_FEVR to waiting non timeout.
 *
 * Returned Value:
 *   On success, the length of the sent command in bytes is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t apicmdgw_send(FAR uint8_t *cmd, FAR uint8_t *respbuff,
    uint16_t bufflen, FAR uint16_t *resplen, int32_t timeout_ms);

/****************************************************************************
 * Name: apicmdgw_sendabort
 *
 * Description:
 *   Abort api command send, And release waiting syun command response.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, the length of the sent command in bytes is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

int32_t apicmdgw_sendabort(void);

/****************************************************************************
 * Name: apicmdgw_cmd_allocbuff
 *
 * Description:
 *   Allocate buffer for API command to be sent. The length to be allocated
 *   is the sum of the data length and header length.
 *   And this function is make api command header in allocated buffer.
 *
 * Input Parameters:
 *   cmdid    Api command id.
 *   len      Length of data field.
 *
 * Returned Value:
 *   If succeeds allocate buffer, start address of the data field
 *   is returned. Otherwise NULL is returned.
 *
 ****************************************************************************/

FAR uint8_t *apicmdgw_cmd_allocbuff(uint16_t cmdid, uint16_t len);

/****************************************************************************
 * Name: apicmdgw_reply_allocbuff
 *
 * Description:
 *   Allocate buffer for API command to be sent. The length to be allocated
 *   is the sum of the data length and header length.
 *   And this function is make api resopnse command header in allocated buffer.
 *
 * Input Parameters:
 *   cmd      Replyning to api command payload pointer.
 *   len      Length of data field.
 *
 * Returned Value:
 *   If succeeds allocate buffer, start address of the data field
 *   is returned. Otherwise NULL is returned.
 *
 ****************************************************************************/

FAR uint8_t *apicmdgw_reply_allocbuff(FAR const uint8_t *cmd, uint16_t len);

/****************************************************************************
 * Name: apicmdgw_freebuff
 *
 * Description:
 *   Free allocated buffer for cmd and reply.
 *
 * Input Parameters:
 *   buff  Pointer to data field.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno in errno.h is returned.
 *
 ****************************************************************************/

int32_t apicmdgw_freebuff(FAR uint8_t *buff);

/****************************************************************************
 * Name: apicmdgw_cmdid_compare
 *
 * Description:
 *   Compare to receive event command id and waiting command id.
 *
 * Input Parameters:
 *   cmd      Receive command payload pointer.
 *   cmdid    Waiting command id.
 *
 * Returned Value:
 *   If the process succeeds, it returns true.
 *   Otherwise false in errno.h is returned.
 *
 ****************************************************************************/

bool apicmdgw_cmdid_compare(FAR uint8_t *cmd, uint16_t cmdid);

#endif /* __MODULES_LTE_ALTCOM_GW_APICMDGW_H */