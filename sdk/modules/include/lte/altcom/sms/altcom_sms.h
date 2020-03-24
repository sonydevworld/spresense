/****************************************************************************
 * modules/include/lte/altcom/sms/altcom_sms.h
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

#ifndef __MODULES_INCLUDE_LTE_ALTCOM_SMS_ALTCOM_SMS_H
#define __MODULES_INCLUDE_LTE_ALTCOM_SMS_ALTCOM_SMS_H

/**
 * @defgroup ltesms SMS Library Interface
 * @brief SMS Interface definitions.
 *
 * #### Interface call type
 *
 * |             Sync Interface            |
 * | :------------------------------------ |
 * | @ref altcom_sms_initialize            |
 * | @ref altcom_sms_finalize              |
 * | @ref altcom_sms_send                  |
 * | @ref altcom_sms_delete                |
 * | @ref altcom_sms_get_storage_info      |
 * | @ref altcom_sms_get_list              |
 *
 * #### Interface return value
 *
 * This interface notifies the processing result as a return value.
 *
 * @attention If the return value is -EPROTO, you can get the error code 
 *            with @ref lte_get_errinfo .
 *            The value of err_result_code in the @ref lte_errinfo_t
 *            structure conforms to 3GPP TS 27.005.
 *
 * @note References
 *   3GPP TS 23.040: "Technical realization of the SMS"
 *   3GPP TS 27.005: "Use of DTE-DCE interface for SMS and CBS"
 *   3GPP TS 23.038: "Alphabets and language-specific information".
 *
 * @{
 * @file  altcom_sms.h
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * This valid flag is used in the following messages:
 * - SMS-SUBMIT
 *  Indicates User-Data are set by the user.
 * - SMS-DELIVER
 *  Indicates User-Data parameter is valid.
 */

#define ALTCOM_SMS_MSG_VALID_UD   (0x01 << 0)

/**
 * This valid flag is used in the following messages:
 * - SMS-SUBMIT
 *  Indicates status report request by the user.
 */

#define ALTCOM_SMS_MSG_VALID_SRR  (0x01 << 1)

/**
 * This valid flag is used in the following messages:
 * - SMS-DELIVER
 *  Indicates concatenated short messages information.
 */

#define ALTCOM_SMS_MSG_VALID_CONCAT_HDR (0x01 << 3)


/**
 * This valid flag is used in the following messages:
 * - SMS-SUBMIT
 *  Indicates Type-Of-Address by the user.
 * - SMS-DELIVER
 *  Indicates Type-Of-Address is valid.
 */

#define ALTCOM_SMS_MSG_VALID_TOA  (0x01 << 4)

/**
 * Maximum length of telephone number defined by MSISDN(E.164) +
 * length of prefix and terminal character.
 */

#define ALTCOM_SMS_ADDR_MAX_LEN (15 + 2)

/** Extract categories from delivery status. */

#define ALTCOM_SMS_DELIVERY_STAT_CAT_MASK (0x60)

/** Sent SMS has been delivered to the destination. */

#define ALTCOM_SMS_DELIVERY_STAT_CAT_OK (0x00)

/** Service center failed to deliver sent SMS, but will be retried later. */

#define ALTCOM_SMS_DELIVERY_STAT_CAT_PEND (0x01 << 6)

/** Service center failed to deliver sent SMS and discarded it. */

#define ALTCOM_SMS_DELIVERY_STAT_CAT_FAIL (0x01 << 7)

/** Maximum number of messages that can be acquired */

#define ALTCOM_SMS_MSG_LIST_MAX_NUM (50)

/** Maximum number of reference ID that can be acquired */

#define ALTCOM_SMS_MSG_REF_ID_MAX_NUM (10)

/** Message type of send. */

#define ALTCOM_SMS_MSG_TYPE_SEND (0x01 << 0)

/** Message type of receive. */

#define ALTCOM_SMS_MSG_TYPE_RECV (0x01 << 1)

/** Message type of deliver report. */

#define ALTCOM_SMS_MSG_TYPE_DELIVER_REPORT (0x01 << 2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** @name Enumerations */
/** @{ */

/**
 * @enum altcom_sms_addr_ton_e
 * 
 * Definition of type of Number in address field.
 *
 * @details See 3GPP TS 23.040.
 */

enum altcom_sms_addr_ton_e
{

  /** Type of Number: Unknown */

  ALTCOM_SMS_ADDR_TON_UNKNOWN = 0x00,

  /** Type of Number: International number */

  ALTCOM_SMS_ADDR_TON_INTERNATIONAL_NUMBER,

  /** Type of Number: National number */

  ALTCOM_SMS_ADDR_TON_NATIONAL_NUMBER,

  /** Type of Number: Network specific number */

  ALTCOM_SMS_ADDR_TON_NETWORK_SPECIFIC_NUMBER,

  /** Type of Number: Subscriber number */

  ALTCOM_SMS_ADDR_TON_SUBSCRIBER_NUMBER,

  /** Type of Number: Alphanumeric */

  ALTCOM_SMS_ADDR_TON_ALPHANUMERIC,

  /** Type of Number: Abbreviated number */ 

  ALTCOM_SMS_ADDR_TON_ABBREVIATED_NUMBER,

  /** Type of Number: Reserved for extension */

  ALTCOM_SMS_ADDR_TON_RESERVED_EXTENSION

};

/**
 * @enum altcom_sms_addr_npi_e
 * 
 * Definition of numbering plan identification
 *        in address field.
 *
 * @details See 3GPP TS 23.040.
 */

enum altcom_sms_addr_npi_e
{

  /** Numbering plan identification: Unknown */

  ALTCOM_SMS_ADDR_NPI_UNKNOWN = 0x00,

  /** Numbering plan identification: ISDN numbering plan */

  ALTCOM_SMS_ADDR_NPI_ISDN_NUMBERING_PLAN = 0x01,

  /** Numbering plan identification: Data numbering plan */

  ALTCOM_SMS_ADDR_NPI_DATA_NUMBERING_PLAN = 0x03,

  /** Numbering plan identification: Telex numbering plan */

  ALTCOM_SMS_ADDR_NPI_TELEX_NUMBERING_PLAN = 0x04,

  /** Numbering plan identification: Service Centre Specific plan */

  ALTCOM_SMS_ADDR_NPI_SERVICE_CENTRE_SPECIFIC_PLAN = 0x05,

  /** Numbering plan identification: Service Centre Specific plan */

  ALTCOM_SMS_ADDR_NPI_SERVICE_CENTRE_SPECIFIC_PLAN_2 = 0x06,

  /** Numbering plan identification: National numbering plan */

  ALTCOM_SMS_ADDR_NPI_NATIONAL_NUMBERING_PLAN = 0x08,

  /** Numbering plan identification: Private numbering plan */

  ALTCOM_SMS_ADDR_NPI_PRIVATE_NUMBERING_PLAN = 0x09,

  /** Numbering plan identification: ERMES numbering plan */

  ALTCOM_SMS_ADDR_NPI_ERMES_NUMBERING_PLAN = 0x0A,

  /** Numbering plan identification: Reserved for extension */

  ALTCOM_SMS_ADDR_NPI_RESERVED_EXTENSION = 0x0F
};

/**
 * @enum altcom_sms_msg_chset_e
 *
 * Definition of character set of User-Data
 * field of sent / received message.
 */

enum altcom_sms_msg_chset_e
{

  /** User-Data character set is GSM 7bit default alphabet. */

  ALTCOM_SMS_MSG_CHSET_GSM7BIT = 0,

  /**
   * User-Data character set is 8 bit data.
   *
   * @attention 8 bit data character set is not supported.
   */

  ALTCOM_SMS_MSG_CHSET_BINARY,

  /** User-Data character set is UCS2. */

  ALTCOM_SMS_MSG_CHSET_UCS2,

  /** Reserved for extention. */

  ALTCOM_SMS_MSG_CHSET_RESERVED
};

/** @} */

/**
 * @struct altcom_sms_addr_s
 *
 * Definition of address settings that message destination,
 * source or service center. 
 */

struct altcom_sms_addr_s
{

  /**
   * Type-Of-Address
   *
   * This parameter is optional.
   * When setting this parameter, @ref ALTCOM_SMS_MSG_VALID_TOA flag
   * has been added to valid_parameter.
   * 
   * @attention Service center address (SCA) is not supported.
   */

  uint8_t toa;

  /**
   * Address of the destination, source or SMSC. It is terminated with '\0'.
   * When using the International phone number,
   * prefix it with a '+'. Others are treated as National
   * phone numbers. @n
   * e.g. "09012345678" or "+819012345678"
   */

  char address[ALTCOM_SMS_ADDR_MAX_LEN];
};

/**
 * @struct altcom_sms_time_s
 *
 * Definition of time structure.
 */

struct altcom_sms_time_s
{
  uint8_t year; /**< Years (0-99) */
  uint8_t mon;  /**< Month (1-12) */
  uint8_t mday; /**< Day of the month (1-31) */
  uint8_t hour; /**< Hours (0-23) */
  uint8_t min;  /**< Minutes (0-59) */
  uint8_t sec;  /**< Seconds (0-59) */
  int8_t  tz;   /**< Time zone in hour (-24 - +24) */
};

/**
 * @struct altcom_sms_userdata_s
 *
 * Definition of User-Data setting.
 */

struct altcom_sms_userdata_s
{

  /**
   * Set the character set used for SMS.
   * Definition is refer to @ref altcom_sms_msg_chset_e.
   */

  uint8_t chset;

  /** Buffer length of User-Data field. */

  uint16_t data_len;

  /** User data in utf-8 format. */

  uint8_t *data;
};

/**
 * @struct altcom_sms_concat_hdr_s
 *
 * Definition of Concatenated short messages header.
 */

struct altcom_sms_concat_hdr_s
{

  /**
   * Contain a modulo 256 counter indicating the reference number 
   * for a particular concatenated short message.
   */

  uint8_t ref_num;

  /** 
   * Contain a value in the range 0 to 255 indicating the total number of 
   * short messages within the concatenated short message.
   */

  uint8_t max_num;

  /**
   * Contain a value in the range 0 to 255 indicating the sequence number 
   * of a particular short message within the concatenated short message.
   */

  uint8_t seq_num;
};

/**
 * @struct altcom_sms_submit_s
 *
 * Definition of send message (SMS-SUBMIT) settings.
 *
 * @note When setting all parameters, it is necessary to understand
 *       the bit pattern of the SMS-SUBMIT message.
 *       See 3GPP TS 23.040.
 */

struct altcom_sms_submit_s
{

  /**
   * Parameter input flag.
   * Parameters without flags are completed internally,
   * but dest_addr is mandatory.
   *
   * Definition is as below:
   * - @ref ALTCOM_SMS_MSG_VALID_UD
   * - @ref ALTCOM_SMS_MSG_VALID_SRR
   * - @ref ALTCOM_SMS_MSG_VALID_TOA
   *
   * e.g. When sending UTF-8 text.
   * valid_indicator = ALTCOM_SMS_MSG_VALID_UD
   */

  uint8_t valid_indicator;

  /**
   * Destination-Address (TP-DA).
   * This parameter is mandatory.
   */

  struct altcom_sms_addr_s dest_addr;

  /**
   * User-Data (TP-UD).
   * This parameter is optional.
   * When setting this parameter, @ref ALTCOM_SMS_MSG_VALID_UD flag
   * has been added to valid_parameter.
   */

  struct altcom_sms_userdata_s userdata;
};

/**
 * @struct altcom_sms_deliver_s
 *
 * Definition of received message (SMS-DELIVER) settings.
 */

struct altcom_sms_deliver_s
{

  /**
   * Parameter output flag.
   * Definition is as below:
   * - @ref ALTCOM_SMS_MSG_VALID_TOA
   * - @ref ALTCOM_SMS_MSG_VALID_CONCAT_HDR
   * - @ref ALTCOM_SMS_MSG_VALID_UD
   */

  uint8_t valid_indicator;

  /**
   * Originating-Address (TP-OA).
   * This parameter is mandatory.
   */

  struct altcom_sms_addr_s src_addr;

  /**
   * Service-Center-Time-Stamp (TP-SCTS) without timezone.
   * This parameter is mandatory.
   */

  struct altcom_sms_time_s sc_time;

  /**
   * Concatenated short messages header.
   * This parameter is optional.
   * When setting this parameter, @ref ALTCOM_SMS_MSG_VALID_CONCAT_HDR flag
   * has been added to valid_parameter.
   */

  struct altcom_sms_concat_hdr_s concat_hdr;

  /**
   * User-Data (TP-UD).
   * This parameter is optional.
   * When setting this parameter, 
   * @ref ALTCOM_SMS_MSG_VALID_UD flag has been added to valid_parameter.
   */

  struct altcom_sms_userdata_s userdata;
};

/**
 * @struct altcom_sms_status_report_s
 *
 * Definition of Delivery report of sent message (SMS-STATUS-REPORT).
 */

struct altcom_sms_status_report_s
{

  /**
   * SMS delivery status to destination.
   * To extract only category,
   * mask with @ref ALTCOM_SMS_DELIVERY_STAT_CAT_MASK.
   * See 3GPP TS 23.040 for details of status. @n
   * Resulting category is defined below:
   * - @ref ALTCOM_SMS_DELIVERY_STAT_CAT_OK
   * - @ref ALTCOM_SMS_DELIVERY_STAT_CAT_PEND
   * - @ref ALTCOM_SMS_DELIVERY_STAT_CAT_FAIL
   */

  uint8_t status;

  /**
   * The time when sent SMS arrived at the service center.
   * It is almost equal to the sent time.
   */

  struct altcom_sms_time_s sc_time;

  /**
   * Message-Reference (TP-MR)
   * Parameter identifying the previously send message (SMS-SUBMIT).
   */

  uint8_t ref_id;

  /**
   * If the status category is @ref SMS_DELIVER_STAT_CAT_OK ,
   * indicates the delivery completion time. Otherwise,
   * it indicates the last time the service center attempted a delivery.
   */

  struct altcom_sms_time_s discharge_time;
};

/**
 * @struct altcom_sms_msg_s
 *
 * Definition of message settings to be sent and received.
 */

struct altcom_sms_msg_s
{

  /**
   * Service center address (SCA) setting. See @ref altcom_sms_addr_s .
   *
   * In case of send message, default SCA
   * will be set if SCA is not set.
   */

  struct altcom_sms_addr_s sc_addr;

  /**
   * Type of SMS message.
   * Definition is as below:
   * - @ref ALTCOM_SMS_MSG_TYPE_SEND
   * - @ref ALTCOM_SMS_MSG_TYPE_RECV
   * - @ref ALTCOM_SMS_MSG_TYPE_DELIVER_REPORT
   */

  uint8_t type;

  union
  {

    /** Send message. See @ref sms_submit_s */

    struct altcom_sms_submit_s send;

    /** Received message. See @ref altcom_sms_deliver_s */

    struct altcom_sms_deliver_s recv;

    /**
     * Delivery report for sent message.
     * See @ref altcom_sms_status_report_s
     */

    struct altcom_sms_status_report_s delivery_report;
  } u;
};

/**
 * @struct altcom_sms_storage_info_s
 *
 * Definition of the message information stored in
 * the reception storage.
 */

struct altcom_sms_storage_info_s
{

  /** Maximum number of received messages can be stored in storage. */

  uint16_t max_record;

  /** Number of received messages stored in storage. */

  uint16_t used_record;
};

/**
 * @struct altcom_sms_msg_info_s
 *
 * Definition of the message information stored in
 * the reception storage.
 */

struct altcom_sms_msg_info_s
{

  /** Type of SMS message. 
   *  Definition is as below:
   *  - @ref ALTCOM_SMS_MSG_TYPE_SEND
   *  - @ref ALTCOM_SMS_MSG_TYPE_RECV
   *  - @ref ALTCOM_SMS_MSG_TYPE_DELIVER_REPORT
   */

  uint8_t type;

  /** Message-Reference (TP-MR)
   * Parameter identifying the previously send message (SMS-SUBMIT).
   */

  uint8_t ref_id;

  /** The index of the reception storage where the message is stored. */

  uint16_t index;

  /**
   * Service-Center-Time-Stamp (TP-SCTS) without timezone.
   * See @ref altcom_sms_time_s
   */

  struct altcom_sms_time_s sc_time;

  /**
   * Set the following address for each type:
   * - ALTCOM_SMS_MSG_TYPE_RECV
   *  Set the source address.
   * - ALTCOM_SMS_MSG_TYPE_DELIVER_REPORT
   *  Set the destination address.
   */

  struct altcom_sms_addr_s addr;
};

/**
 * @struct altcom_sms_msg_list_s
 *
 * Definition of the list of message information.
 */

struct altcom_sms_msg_list_s
{

  /** Number of messages. */

  uint16_t num;

  /** Information list of the message in the reception storage. */

  struct altcom_sms_msg_info_s msg[ALTCOM_SMS_MSG_LIST_MAX_NUM];
};

/**
 *  Callback function to receiving SMS.
 *
 * @param [in] msg: The pointer of structure for received message.
 *
 * @param [in] index: Index number of the storage.
 *                    When storage not use, set this parameter to 0.
 *
 * @return On success, 0 is returned. On failure,
 *         negative value is returned according to <errno.h>.
 *
 * @attention Callback not called after return negative value.
 *            To resume calling the callback, run altcom_sms_finalize()
 *            then altcom_sms_initialize().
 * 
 */

typedef int (*sms_recv_cb_t)(struct altcom_sms_msg_s *msg, uint16_t index);

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/** @name Functions */
/** @{ */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * Initialize resources used in SMS API.
 *
 * @param [in] callback: Callback function to receiving SMS.
 *
 * @param [in] storage_use: Save received SMS to storage.
 *                          True is save a SMS to storage,
 *                          False is not save a SMS to storage.
 *
 * @return On success, 0 is returned. On failure,
 *         negative value is returned according to <errno.h>.
 */

int altcom_sms_initialize(sms_recv_cb_t callback, bool storage_use);

/**
 * Release resources used in SMS API.
 *
 * @return On success, 0 is returned. On failure,
 *         negative value is returned according to <errno.h>.
 */

int altcom_sms_finalize(void);

/**
 * Send SMS message.
 *
 * @param [in] msg: The pointer of structure for send message.
 *
 * @param [out] mr_list: List of reference ID of sent SMS message.
 *                       See @ref ALTCOM_SMS_MSG_REF_ID_MAX_NUM for
 *                       the maximum value that can be set.
 *
 * @return On success, effective number of mr_list is returned. On failure,
 *         negative value is returned according to <errno.h>.
 */

int altcom_sms_send(struct altcom_sms_msg_s *msg,
                    uint8_t mr_list[ALTCOM_SMS_MSG_REF_ID_MAX_NUM]);

/**
 * Delete the message from reception storage.
 *
 * @param [in] index: Index number of the storage to delete.
 *                    When message is deleted by types,
 *                    set this parameter to 0.
 *
 * @param [in] types: Types for which messages to delete.
 *                    Bit setting definition is as below:
 *                    - @ref ALTCOM_SMS_MSG_TYPE_RECV
 *                    - @ref ALTCOM_SMS_MSG_TYPE_DELIVER_REPORT
 *                    Does not support other bit setting definitions.
 *                    This parameter is valid only when index is 0.
 *
 * @return On success, 0 is returned. On failure,
 *         negative value is returned according to <errno.h>.
 */

int altcom_sms_delete(uint16_t index, uint8_t types);

/**
 * Check reception storage of SMS.
 *
 * @param [out] info: The pointer of structure to store the reception
 *                    storage information.
 *                    See @ref altcom_sms_storage_info_s .
 *
 * @return On success, 0 is returned. On failure,
 *         negative value is returned according to <errno.h>.
 */

int altcom_sms_get_storage_info(struct altcom_sms_storage_info_s *info);

/**
 * Get the information list of the message with the message type
 * specified by the argument.
 *
 * The information list is chronological order.
 *
 * @param [in] types: Type of messages to get.
 *                    Definition is as below:
 *                    - @ref ALTCOM_SMS_MSG_TYPE_RECV
 *                    - @ref ALTCOM_SMS_MSG_TYPE_DELIVER_REPORT
 *                    Does not support other bit setting definitions.
 *
 * @param [out] list: The pointer of list of messages stored in
 *                    the reception storage.
 *                    See @ref altcom_sms_msg_list_s .
 *
 * @return On success, 0 is returned. On failure,
 *         negative value is returned according to <errno.h>.
 */

int altcom_sms_get_list(uint8_t types,
                        struct altcom_sms_msg_list_s *list);

/** @} */

/** @} */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MODULES_INCLUDE_LTE_ALTCOM_SMS_ALTCOM_SMS_H */
