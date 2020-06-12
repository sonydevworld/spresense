/****************************************************************************
 * modules/lte/altcom/include/api/sms/apicmd_sms.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_SMS_APICMD_SMS_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_SMS_APICMD_SMS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "apicmd.h"
#include "altcom_sms.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALTCOM_SMS_ADDR_BUFF_LEN (32)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct altcom_apicmd_sms_addr_s
{

  /* Type of address is reserved. */

  uint8_t toa;
  uint8_t length;
  uint8_t address[ALTCOM_SMS_ADDR_BUFF_LEN];
};

struct altcom_apicmd_sms_msg_s
  {

    uint8_t  type;

    union
      {

        /* SMS-DELIVER */

        struct
          {
            uint8_t                         valid_indicator;
            struct altcom_apicmd_sms_addr_s src_addr;
            struct altcom_sms_time_s        sc_time;
            struct altcom_sms_concat_hdr_s  concat_hdr;
            struct altcom_sms_userdata_s    userdata;

            /* Variable length array */

            uint8_t user_data[1];
          } recv;


        /* SMS-STATUS-REPORT */

        struct
          {
            uint8_t                  status;
            struct altcom_sms_time_s sc_time;
            uint8_t                  ref_id;
            struct altcom_sms_time_s discharge_time;
          } delivery_report;
      } u;
  };

struct altcom_sms_apicmd_msg_info_s
{
  uint8_t                         type;
  uint8_t                         ref_id;
  uint16_t                        index;
  struct altcom_sms_time_s        sc_time;
  struct altcom_apicmd_sms_addr_s addr;
};

/* This structure discribes the data structure of the API command */

/* APICMDID_SMS_INIT */

begin_packed_struct struct apicmd_cmddat_sms_init_s
{
  uint8_t types;
  uint8_t storage_use;
} end_packed_struct;

/* APICMDID_SMS_INITRES */

begin_packed_struct struct apicmd_cmddat_sms_initres_s
{
  int32_t result;
} end_packed_struct;

/* APICMDID_SMS_FIN
 * NO DATA
 */

/* APICMDID_SMS_FINRES */

begin_packed_struct struct apicmd_cmddat_sms_finres_s
{
  int32_t result;
} end_packed_struct;

/* APICMDID_SMS_SEND */

begin_packed_struct struct apicmd_cmddat_sms_send_s
{
  struct altcom_apicmd_sms_addr_s sc_addr;

  /* SMS-SUBMIT */

  uint8_t                         valid_indicator;
  struct altcom_apicmd_sms_addr_s dest_addr;
  struct altcom_sms_userdata_s    userdata;

  /* Variable length array */

  uint8_t user_data[1];
} end_packed_struct;

/* APICMDID_SMS_SENDRES */

begin_packed_struct struct apicmd_cmddat_sms_sendres_s
{
  int32_t result;
  uint8_t mr_num;
  uint8_t mr_list[ALTCOM_SMS_MSG_REF_ID_MAX_NUM];
} end_packed_struct;

/* APICMDID_SMS_REPORT_RECV */

begin_packed_struct struct apicmd_cmddat_sms_reprecv_s
{
  uint16_t                        index;
  struct altcom_apicmd_sms_addr_s sc_addr;
  struct altcom_apicmd_sms_msg_s  msg;

} end_packed_struct;

/* APICMDID_SMS_RECVRES */

begin_packed_struct struct apicmd_cmddat_sms_reprecvres_s
{
  int32_t result;
} end_packed_struct;

/* APICMDID_SMS_DELETE */

begin_packed_struct struct apicmd_cmddat_sms_delete_s
{
  uint16_t index;
  uint8_t  types;
} end_packed_struct;

/* APICMDID_SMS_DELETERES */

begin_packed_struct struct apicmd_cmddat_sms_deleteres_s
{
  int32_t result;
} end_packed_struct;

/* APICMDID_SMS_GET_STGEINFO
 * NO DATA
 */

/* APICMDID_SMS_GET_STGEINFORES */

begin_packed_struct struct apicmd_cmddat_sms_getstgeinfores_s
{
  int32_t                          result;
  struct altcom_sms_storage_info_s info;
} end_packed_struct;

/* APICMDID_SMS_GET_LIST */

begin_packed_struct struct apicmd_cmddat_sms_getlist_s
{
  uint8_t types;
} end_packed_struct;

/* APICMDID_SMS_GET_LISTRES */

begin_packed_struct struct apicmd_cmddat_sms_getlistres_s
{
  int32_t                             result;
  uint16_t                            num;
  struct altcom_sms_apicmd_msg_info_s msg[ALTCOM_SMS_MSG_LIST_MAX_NUM];
} end_packed_struct;

/* APICMDID_SMS_READ */

begin_packed_struct struct apicmd_cmddat_sms_read_s
{
  uint16_t index;
} end_packed_struct;

/* APICMDID_SMS_READRES */

begin_packed_struct struct apicmd_cmddat_sms_readres_s
{
  int32_t                         result;
  struct altcom_apicmd_sms_addr_s sc_addr;
  struct altcom_apicmd_sms_msg_s  msg;
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_SMS_APICMD_SMS_H */
