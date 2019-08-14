/****************************************************************************
 * modules/include/lte/lte_api.h
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

#ifndef __MODULES_INCLUDE_LTE_LTE_API_H
#define __MODULES_INCLUDE_LTE_LTE_API_H

/**
 * @defgroup lte_utility LTE Utility
 * @{
 */

/**
 * @defgroup lte_utility_lte_connecter_api LTE Connecter API
 *
 * #### API call type
 *
 * |         Sync API                |         Async API               |
 * | :------------------------------ | :------------------------------ |
 * | @ref lte_initialize             | @ref lte_radio_on               |
 * | @ref lte_finalize               | @ref lte_radio_off              |
 * | @ref lte_set_report_restart     | @ref lte_activate_pdn           |
 * | @ref lte_power_on               | @ref lte_deactivate_pdn         |
 * | @ref lte_power_off              | @ref lte_data_allow             |
 * | @ref lte_set_report_netinfo     | @ref lte_get_netinfo            |
 * | @ref lte_set_report_simstat     | @ref lte_get_imscap             |
 * | @ref lte_set_report_localtime   | @ref lte_get_version            |
 * | @ref lte_set_report_quality     | @ref lte_get_phoneno            |
 * | @ref lte_set_report_cellinfo    | @ref lte_get_imsi               |
 * | @ref lte_get_errinfo            | @ref lte_get_imei               |
 * | @ref lte_activate_pdn_cancel    | @ref lte_get_pinset             |
 * |                                 | @ref lte_set_pinenable          |
 * |                                 | @ref lte_change_pin             |
 * |                                 | @ref lte_enter_pin              |
 * |                                 | @ref lte_get_localtime          |
 * |                                 | @ref lte_get_operator           |
 * |                                 | @ref lte_get_edrx               |
 * |                                 | @ref lte_set_edrx               |
 * |                                 | @ref lte_get_psm                |
 * |                                 | @ref lte_set_psm                |
 * |                                 | @ref lte_get_ce                 |
 * |                                 | @ref lte_set_ce                 |
 * |                                 | @ref lte_get_siminfo            |
 * |                                 | @ref lte_get_dynamic_edrx_param |
 * |                                 | @ref lte_get_dynamic_psm_param  |
 * |                                 | @ref lte_get_quality            |
 *
 *
 * @{
 *
 * @file  lte_api.h
 * @brief LTE API definitions.
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LTE_RESULT_OK     (0)      /**< Result code on success */
#define LTE_RESULT_ERROR  (1)      /**< Result code on failure */
#define LTE_RESULT_CANCEL (2)      /**< Result code on cancel */

#define LTE_VALID   (true)         /**< Valid */
#define LTE_INVALID (false)        /**< Invalid */

#define LTE_ENABLE  (true)         /**< Enable */
#define LTE_DISABLE (false)        /**< Disable */

#define LTE_ERR_WAITENTERPIN (1)   /**< Waiting for PIN enter */
#define LTE_ERR_REJECT       (2)   /**< Rejected from the network */
#define LTE_ERR_MAXRETRY     (3)   /**< No response from the network */
#define LTE_ERR_BARRING      (4)   /**< Network barring */
#define LTE_ERR_DETACHED     (5)   /**< Network detached */
#define LTE_ERR_UNEXPECTED   (255) /**< Unexpected cause */

#define LTE_SESSION_ID_MIN (1)     /**< Minimum value of session ID */
#define LTE_SESSION_ID_MAX (5)     /**< Maximum value of session ID */

/** Network status: Not registered, MT is not currently searching
                    a new operator to register to */

#define LTE_NETSTAT_NOT_REG_NOT_SEARCHING     (0)

/** Network status: Registered, home network */

#define LTE_NETSTAT_REG_HOME                  (1)

/** Network status: Not registered, but MT is currently searching
                    a new operator to register to */

#define LTE_NETSTAT_NOT_REG_SEARCHING         (2)

/** Network status: Registration denied */

#define LTE_NETSTAT_REG_DENIED                (3)

/** Network status: Unknown */

#define LTE_NETSTAT_UNKNOWN                   (4)

/** Network status: Registered, roaming */

#define LTE_NETSTAT_REG_ROAMING               (5)

/** Network status: Registered for "SMS only", home network */

#define LTE_NETSTAT_REG_SMS_ONLY_HOME         (6)

/** Network status: Registered for "SMS only", roaming */

#define LTE_NETSTAT_REG_SMS_ONLY_ROAMING      (7)

/** Network status: Attached for emergency bearer services only */

#define LTE_NETSTAT_NOT_REG_EMERGENCY         (8)

/** Network status: Registered for "CSFB not preferred", home network */

#define LTE_NETSTAT_REG_CSFB_NOT_PREF_HOME    (9)

/** Network status: Registered for "CSFB not preferred", roaming */

#define LTE_NETSTAT_REG_CSFB_NOT_PREF_ROAMING (10)

/** The maximum string length of the APN name */

#define LTE_APN_LEN           (101)

/** The maximum string length of the APN user name */

#define LTE_APN_USER_NAME_LEN (64)

/** The maximum string length of the APN password */

#define LTE_APN_PASSWD_LEN    (32)

#define LTE_APN_IPTYPE_IP     (0) /**< Internet protocol type: IP */
#define LTE_APN_IPTYPE_IPV6   (1) /**< Internet protocol type: IPv6 */
#define LTE_APN_IPTYPE_IPV4V6 (2) /**< Internet protocol type: IPv4/v6 */

#define LTE_APN_AUTHTYPE_NONE (0) /**< PPP authentication type: NONE */
#define LTE_APN_AUTHTYPE_PAP  (1) /**< PPP authentication type: PAP */
#define LTE_APN_AUTHTYPE_CHAP (2) /**< PPP authentication type: CHAP */

/** APN type: Unknown  */

#define LTE_APN_TYPE_UNKNOWN   (0x01)

/** APN type: Default data traffic */

#define LTE_APN_TYPE_DEFAULT   (0x02)

/** APN type: MMS traffic(Multimedia Messaging Service) */

#define LTE_APN_TYPE_MMS       (0x04)

/** APN type: SUPL assisted GPS */

#define LTE_APN_TYPE_SUPL      (0x08)

/** APN type: DUN traffic(Dial Up Networking bridge ) */

#define LTE_APN_TYPE_DUN       (0x10)

/** APN type: HiPri traffic(High Priority Mobile data) */

#define LTE_APN_TYPE_HIPRI     (0x20)

/** APN type: FOTA(Firmware On The Air) */

#define LTE_APN_TYPE_FOTA      (0x40)

/** APN type: IMS(IP Multimedia Subsystem) */

#define LTE_APN_TYPE_IMS       (0x80)

/** APN type: CBS(Carrier Branded Services) */

#define LTE_APN_TYPE_CBS       (0x100)

/** APN type: IA(Initial Attach APN) */

#define LTE_APN_TYPE_IA        (0x200)

/** APN type: Emergency PDN */

#define LTE_APN_TYPE_EMERGENCY (0x400)

/** Network error type: MAX_RETRY */

#define LTE_NETERR_MAXRETRY    (0)

/** Network error type: REJECT */

#define LTE_NETERR_REJECT      (1)

/** Network error type: Network Detach */

#define LTE_NETERR_NWDTCH      (2)

/** Network reject category: NAS-EMM */

#define LTE_REJECT_CATEGORY_EMM  (0)

/** Network reject category: NAS-ESM */

#define LTE_REJECT_CATEGORY_ESM  (1)

/** Length of character string for BB product */

#define LTE_VER_BB_PRODUCT_LEN (5)

/** Length of character string for NP package */

#define LTE_VER_NP_PACKAGE_LEN (32)

#define LTE_PIN_ENABLE  (true)  /**< Enable setting of PIN lock */
#define LTE_PIN_DISABLE (false) /**< Disable setting of PIN lock */

/** PIN status: Not pending for any password */

#define LTE_PINSTAT_READY         (0)

/** PIN status: Waiting SIM PIN to be given */

#define LTE_PINSTAT_SIM_PIN       (1)

/** PIN status: Waiting SIM PUK to be given */

#define LTE_PINSTAT_SIM_PUK       (2)

/** PIN status: Waiting phone to SIM card password to be given */

#define LTE_PINSTAT_PH_SIM_PIN    (3)

/** PIN status: Waiting phone-to-very first SIM card password to be given */

#define LTE_PINSTAT_PH_FSIM_PIN   (4)

/** PIN status: Waiting phone-to-very first SIM card unblocking
                password to be given */

#define LTE_PINSTAT_PH_FSIM_PUK   (5)

/** PIN status: Waiting SIM PIN2 to be given */

#define LTE_PINSTAT_SIM_PIN2      (6)

/** PIN status: Waiting SIM PUK2 to be given */

#define LTE_PINSTAT_SIM_PUK2      (7)

/** PIN status: Waiting network personalization password to be given */

#define LTE_PINSTAT_PH_NET_PIN    (8)

/** PIN status: Waiting network personalization unblocking password
                to be given */

#define LTE_PINSTAT_PH_NET_PUK    (9)

/** PIN status: Waiting network subset personalization password to be given */

#define LTE_PINSTAT_PH_NETSUB_PIN (10)

/** PIN status: Waiting network subset personalization unblocking password
                to be given */

#define LTE_PINSTAT_PH_NETSUB_PUK (11)

/** PIN status: Waiting service provider personalization password
                to be given */

#define LTE_PINSTAT_PH_SP_PIN     (12)

/** PIN status: Waiting service provider personalization unblocking password
                to be given */

#define LTE_PINSTAT_PH_SP_PUK     (13)

/** PIN status: Waiting corporate personalization password to be given */

#define LTE_PINSTAT_PH_CORP_PIN   (14)

/** PIN status: Waiting corporate personalization unblocking password
                to be given */

#define LTE_PINSTAT_PH_CORP_PUK   (15)

#define LTE_TARGET_PIN  (0)  /**< Select of PIN change */
#define LTE_TARGET_PIN2 (1)  /**< Select of PIN2 change */

/** SIM status: SIM removal signal detected */

#define LTE_SIMSTAT_REMOVAL         (0)

/** SIM status: SIM insertion signal detected */

#define LTE_SIMSTAT_INSERTION       (1)

/** SIM status: SIM init passed, wait for PIN unlock */

#define LTE_SIMSTAT_WAIT_PIN_UNLOCK (2)

/** SIM status: Personalization failed, wait for run-time depersonalization */

#define LTE_SIMSTAT_PERSONAL_FAILED (3)

/** SIM status: Activation completed. Event is sent always
                at any SIM activation completion */

#define LTE_SIMSTAT_ACTIVATE        (4)

/** SIM status: SIM is deactivated */

#define LTE_SIMSTAT_DEACTIVATE      (5)

#define LTE_MCC_DIGIT     (3)  /**< Digit number of mcc */
#define LTE_MNC_DIGIT_MAX (3)  /**< Max digit number of mnc */

/** Digit number of mcc */

#define LTE_CELLINFO_MCC_DIGIT     LTE_MCC_DIGIT

/** Max digit number of mnc */

#define LTE_CELLINFO_MNC_DIGIT_MAX LTE_MNC_DIGIT_MAX

#define LTE_EDRX_ACTTYPE_WBS1 (0) /**< E-UTRAN (WB-S1 mode) */
#define LTE_EDRX_CYC_512      (0) /**< eDRX cycle:    5.12 sec */
#define LTE_EDRX_CYC_1024     (1) /**< eDRX cycle:   10.24 sec */
#define LTE_EDRX_CYC_2048     (2) /**< eDRX cycle:   20.48 sec */
#define LTE_EDRX_CYC_4096     (3) /**< eDRX cycle:   40.96 sec */
#define LTE_EDRX_CYC_6144     (4) /**< eDRX cycle:   61.44 sec */
#define LTE_EDRX_CYC_8192     (5) /**< eDRX cycle:   81.92 sec */
#define LTE_EDRX_CYC_10240    (6) /**< eDRX cycle:  102.40 sec */
#define LTE_EDRX_CYC_12288    (7) /**< eDRX cycle:  122.88 sec */
#define LTE_EDRX_CYC_14336    (8) /**< eDRX cycle:  143.36 sec */
#define LTE_EDRX_CYC_16384    (9) /**< eDRX cycle:  163.84 sec */
#define LTE_EDRX_CYC_32768   (10) /**< eDRX cycle:  327.68 sec */
#define LTE_EDRX_CYC_65536   (11) /**< eDRX cycle:  655.36 sec */
#define LTE_EDRX_CYC_131072  (12) /**< eDRX cycle: 1310.72 sec */
#define LTE_EDRX_CYC_262144  (13) /**< eDRX cycle: 2621.44 sec */
#define LTE_EDRX_PTW_128      (0) /**< Paging time window:  1.28 sec */
#define LTE_EDRX_PTW_256      (1) /**< Paging time window:  2.56 sec */
#define LTE_EDRX_PTW_384      (2) /**< Paging time window:  3.84 sec */
#define LTE_EDRX_PTW_512      (3) /**< Paging time window:  5.12 sec */
#define LTE_EDRX_PTW_640      (4) /**< Paging time window:  6.40 sec */
#define LTE_EDRX_PTW_768      (5) /**< Paging time window:  7.68 sec */
#define LTE_EDRX_PTW_896      (6) /**< Paging time window:  8.96 sec */
#define LTE_EDRX_PTW_1024     (7) /**< Paging time window: 10.24 sec */
#define LTE_EDRX_PTW_1152     (8) /**< Paging time window: 11.52 sec */
#define LTE_EDRX_PTW_1280     (9) /**< Paging time window: 12.80 sec */
#define LTE_EDRX_PTW_1408    (10) /**< Paging time window: 14.08 sec */
#define LTE_EDRX_PTW_1536    (11) /**< Paging time window: 15.36 sec */
#define LTE_EDRX_PTW_1664    (12) /**< Paging time window: 16.64 sec */
#define LTE_EDRX_PTW_1792    (13) /**< Paging time window: 17.92 sec */
#define LTE_EDRX_PTW_1920    (14) /**< Paging time window: 19.20 sec */
#define LTE_EDRX_PTW_2048    (15) /**< Paging time window: 20.48 sec */

/** Unit of request active time(T3324): 2 sec */

#define LTE_PSM_T3324_UNIT_2SEC    (0)

/** Unit of request active time(T3324): 1 min */

#define LTE_PSM_T3324_UNIT_1MIN    (1)

/** Unit of request active time(T3324): 6 min */

#define LTE_PSM_T3324_UNIT_6MIN    (2)

/** Unit of request active time(T3324): The value indicates that
 *  the timer is deactivated. */

#define LTE_PSM_T3324_UNIT_DEACT   (3)

/** Unit of extended periodic TAU time(T3412): 2 sec */

#define LTE_PSM_T3412_UNIT_2SEC    (0)

/** Unit of extended periodic TAU time(T3412): 30 sec */

#define LTE_PSM_T3412_UNIT_30SEC   (1)

/** Unit of extended periodic TAU time(T3412): 1 min */

#define LTE_PSM_T3412_UNIT_1MIN    (2)

/** Unit of extended periodic TAU time(T3412): 10 min */

#define LTE_PSM_T3412_UNIT_10MIN   (3)

/** Unit of extended periodic TAU time(T3412): 1 hour */

#define LTE_PSM_T3412_UNIT_1HOUR   (4)

/** Unit of extended periodic TAU time(T3412): 10 hour */

#define LTE_PSM_T3412_UNIT_10HOUR  (5)

/** Unit of extended periodic TAU time(T3412): 320 hour */

#define LTE_PSM_T3412_UNIT_320HOUR (6)

/** Unit of extended periodic TAU time(T3412): The value indicates that
 *  the timer is deactivated. */

#define LTE_PSM_T3412_UNIT_DEACT   (7)

/** The minimum timer value used by PSM related timers */

#define LTE_PSM_TIMEVAL_MIN        (0)

/** The maxmum timer value used by PSM related timers */

#define LTE_PSM_TIMEVAL_MAX        (31)

#define LTE_IPTYPE_V4      (0)  /**< IP address type: IPv4 */
#define LTE_IPTYPE_V6      (1)  /**< IP address type: IPv6 */
#define LTE_IPADDR_MAX_LEN (40) /**< Maximum length of the IP address */

/** Invalid Session ID */

#define LTE_PDN_SESSIONID_INVALID_ID (0)

/** Minimum value of Session ID */

#define LTE_PDN_SESSIONID_MIN        (LTE_PDN_SESSIONID_INVALID_ID)

/** Maximum value of Session ID */

#define LTE_PDN_SESSIONID_MAX        (255)

#define LTE_PDN_DEACTIVE         (0) /**< PDN status: Not active */
#define LTE_PDN_ACTIVE           (1) /**< PDN status: Active */

#define LTE_PDN_IPADDR_MAX_COUNT (2) /**< Maximum number of IP addresses */

#define LTE_IMS_NOT_REGISTERED   (0) /**< IMS status: Not registered */
#define LTE_IMS_REGISTERED       (1) /**< IMS status: Registered */
#define LTE_DATA_DISALLOW        (0) /**< Data communication: Not allow */
#define LTE_DATA_ALLOW           (1) /**< Data communication: Allow */

/** Modem restert cause: User initiated */

#define LTE_RESTART_USER_INITIATED  (0)

/** Modem restert cause: Modem initiated */

#define LTE_RESTART_MODEM_INITIATED (1)

/** Error indicator for error code */

#define LTE_ERR_INDICATOR_ERRCODE  (0x01)

/** Error indicator for error number */

#define LTE_ERR_INDICATOR_ERRNO    (0x02)

/** Error indicator for error string */

#define LTE_ERR_INDICATOR_ERRSTR   (0x04)

/** Maximum length of the error string */

#define LTE_ERROR_STRING_MAX_LEN   (64)

/** Indicates to get for MCC/MNC of SIM */

#define LTE_SIMINFO_GETOPT_MCCMNC (1 << 0)

/** Indicates to get for SPN of SIM */

#define LTE_SIMINFO_GETOPT_SPN    (1 << 1)

/** Indicates to get for ICCID of SIM */

#define LTE_SIMINFO_GETOPT_ICCID  (1 << 2)

/** Indicates to get for IMSI of SIM */

#define LTE_SIMINFO_GETOPT_IMSI   (1 << 3)

/** Indicates to get for GID1(Group Identifier Level 1) of SIM */

#define LTE_SIMINFO_GETOPT_GID1   (1 << 4)

/** Indicates to get for GID2(Group Identifier Level 2) of SIM */

#define LTE_SIMINFO_GETOPT_GID2   (1 << 5)

/** Digit number of mcc */

#define LTE_SIMINFO_MCC_DIGIT      LTE_MCC_DIGIT

/** Max digit number of mnc */

#define LTE_SIMINFO_MNC_DIGIT_MAX  LTE_MNC_DIGIT_MAX

#define LTE_SIMINFO_SPN_LEN   (16)  /**< Maximum length of SPN */
#define LTE_SIMINFO_ICCID_LEN (10)  /**< Maximum length of ICCCID */
#define LTE_SIMINFO_IMSI_LEN  (15)  /**< Maximum length of IMSI */
#define LTE_SIMINFO_GID_LEN   (128) /**< Maximum length of GID */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @struct lte_version
 * Definition of version information of the modem.
 * This is notified by get_ver_cb_t
 * @typedef lte_version_t
 * See @ref lte_version
 */

typedef struct lte_version
{
  /** BB product version. It is terminated with '\0'. */

  int8_t bb_product[LTE_VER_BB_PRODUCT_LEN];

  /** NP package version. It is terminated with '\0'. */

  int8_t np_package[LTE_VER_NP_PACKAGE_LEN];
} lte_version_t;

/**
 * @struct lte_getpin
 * Definition of PIN setting information.
 * This is notified by get_pinset_cb_t
 * @typedef lte_getpin_t
 * See @ref lte_getpin
 */

typedef struct lte_getpin
{
  /** PIN enable. Definition is as below.@n
   *  - @ref LTE_PIN_ENABLE@n
   *  - @ref LTE_PIN_DISABLE@n */

  uint8_t enable;

  /** PIN status. Refer to the this parameter only
      when enable is @ref LTE_PIN_ENABLE. */

  uint8_t status;

  /** PIN attempts left */

  uint8_t pin_attemptsleft;

  /** PUK attempts left */

  uint8_t puk_attemptsleft;

  /** PIN2 attempts left */

  uint8_t pin2_attemptsleft;

  /** PUK2 attempts left */

  uint8_t puk2_attemptsleft;
} lte_getpin_t;

/**
 * @struct lte_localtime
 * Definition of local time. This is notified by
 * get_localtime_cb_t and localtime_report_cb_t
 * @typedef lte_localtime_t
 * See @ref lte_localtime
 */

typedef struct lte_localtime
{
  int32_t year;   /**< Years (0-99) */
  int32_t mon;    /**< Month (1-12) */
  int32_t mday;   /**< Day of the month (1-31) */
  int32_t hour;   /**< Hours (0-23) */
  int32_t min;    /**< Minutes (0-59) */
  int32_t sec;    /**< Seconds (0-59) */
  int32_t tz_sec; /**< Time zone in seconds (-86400-86400) */
} lte_localtime_t;

/**
 * @struct lte_quality
 * Definition of parameters for quality information.
 * This is reported by quality_report_cb_t
 * and notified by get_quality_cb_t
 * @typedef lte_quality_t
 * See @ref lte_quality
 */

typedef struct lte_quality
{
  /** Valid flag. Definition is as below.@n
   *  - @ref LTE_VALID@n
   *  - @ref LTE_INVALID@n
   *  Refer to the following parameters only when this flag
   *  is @ref LTE_VALID. This is because valid parameters can not be
   *  acquired when RF function is OFF and so on */

  bool    valid;

  /** RSRP in dBm (-140-0) */

  int16_t rsrp;

  /** RSRQ in dBm (-60-0) */

  int16_t rsrq;

  /** SINR in dBm (-128-40) */

  int16_t sinr;

  /** RSSI in dBm */

  int16_t rssi;
} lte_quality_t;

/**
 * @struct lte_cellinfo
 * Definition of parameters for cell information.
 * This is reported by cellinfo_report_cb_t
 * @typedef lte_cellinfo_t
 * See @ref lte_cellinfo
 */

typedef struct lte_cellinfo
{
  /** Valid flag. Definition is as below.@n
   *  - @ref LTE_VALID@n
   *  - @ref LTE_INVALID@n
   *  Refer to the following parameters only when this flag
   *  is @ref LTE_VALID. This is because valid parameters can not be
   *  acquired when RF function is OFF and so on */

  bool     valid;

  /** Physical cell ID (0-503) */

  uint32_t phycell_id;

  /** EARFCN (0-262143) */

  uint32_t earfcn;

  /** Mobile Country Code (000-999) */

  uint8_t  mcc[LTE_CELLINFO_MCC_DIGIT];

  /** Digit number of mnc(2-3) */

  uint8_t  mnc_digit;

  /** Mobile Network Code (00-999) */

  uint8_t  mnc[LTE_CELLINFO_MNC_DIGIT_MAX];
} lte_cellinfo_t;

/**
 * @struct lte_edrx_setting
 * Definition of eDRX settings used in lte_set_edrx().
 * This is notified by get_edrx_cb_t
 * @typedef lte_edrx_setting_t
 * See @ref lte_edrx_setting
 */

typedef struct lte_edrx_setting
{
  /** eDRX act type. Definition is as below.@n
   *  - @ref LTE_EDRX_ACTTYPE_WBS1@n */

  uint8_t  act_type;

  /** eDRX enable. Definition is as below.@n
   *  - @ref LTE_ENABLE@n
   *  - @ref LTE_DISABLE@n */

  bool     enable;

  /** eDRX cycle. Definition is as below.@n
   *  - @ref LTE_EDRX_CYC_512@n
   *  - @ref LTE_EDRX_CYC_1024@n
   *  - @ref LTE_EDRX_CYC_2048@n
   *  - @ref LTE_EDRX_CYC_4096@n
   *  - @ref LTE_EDRX_CYC_6144@n
   *  - @ref LTE_EDRX_CYC_8192@n
   *  - @ref LTE_EDRX_CYC_10240@n
   *  - @ref LTE_EDRX_CYC_12288@n
   *  - @ref LTE_EDRX_CYC_14336@n
   *  - @ref LTE_EDRX_CYC_16384@n
   *  - @ref LTE_EDRX_CYC_32768@n
   *  - @ref LTE_EDRX_CYC_65536@n
   *  - @ref LTE_EDRX_CYC_131072@n
   *  - @ref LTE_EDRX_CYC_262144@n */

  uint32_t edrx_cycle;

  /** Paging time window. Definition is as below.@n
   *  - @ref LTE_EDRX_PTW_128@n
   *  - @ref LTE_EDRX_PTW_256@n
   *  - @ref LTE_EDRX_PTW_384@n
   *  - @ref LTE_EDRX_PTW_512@n
   *  - @ref LTE_EDRX_PTW_640@n
   *  - @ref LTE_EDRX_PTW_768@n
   *  - @ref LTE_EDRX_PTW_896@n
   *  - @ref LTE_EDRX_PTW_1024@n
   *  - @ref LTE_EDRX_PTW_1152@n
   *  - @ref LTE_EDRX_PTW_1280@n
   *  - @ref LTE_EDRX_PTW_1408@n
   *  - @ref LTE_EDRX_PTW_1536@n
   *  - @ref LTE_EDRX_PTW_1664@n
   *  - @ref LTE_EDRX_PTW_1792@n
   *  - @ref LTE_EDRX_PTW_1920@n
   *  - @ref LTE_EDRX_PTW_2048@n */

  uint32_t ptw_val;
} lte_edrx_setting_t;

/**
 * @struct lte_psm_timeval
 * Definition of timer information for PSM
 * @typedef lte_psm_timeval_t
 * See @ref lte_psm_timeval
 */

typedef struct lte_psm_timeval
{
  /** Unit of timer value. Definition is as below.@n
   *  - When kind of timer is Requested Active Time
   *    - @ref LTE_PSM_T3324_UNIT_2SEC@n
   *    - @ref LTE_PSM_T3324_UNIT_1MIN@n
   *    - @ref LTE_PSM_T3324_UNIT_6MIN@n
   *    - @ref LTE_PSM_T3324_UNIT_DEACT@n
   *  - When kind of timer is Extended periodic TAU Time
   *    - @ref LTE_PSM_T3412_UNIT_2SEC@n
   *    - @ref LTE_PSM_T3412_UNIT_30SEC@n
   *    - @ref LTE_PSM_T3412_UNIT_1MIN@n
   *    - @ref LTE_PSM_T3412_UNIT_10MIN@n
   *    - @ref LTE_PSM_T3412_UNIT_1HOUR@n
   *    - @ref LTE_PSM_T3412_UNIT_10HOUR@n
   *    - @ref LTE_PSM_T3412_UNIT_320HOUR@n
   *    - @ref LTE_PSM_T3412_UNIT_DEACT@n */

  uint8_t unit;

  /** Timer value (0-31) */

  uint8_t time_val;
} lte_psm_timeval_t;

/**
 * @struct lte_psm_setting
 * Definition of PSM settings used in lte_set_psm().
 * This is notified by get_psm_cb_t
 * @typedef lte_psm_setting_t
 * See @ref lte_psm_setting
 */

typedef struct lte_psm_setting
{
  /** PSM enable. Definition is as below.@n
   *  - @ref LTE_ENABLE@n
   *  - @ref LTE_DISABLE@n */

  bool              enable;

  /** Requested Active Time value(T3324). See @ref lte_psm_timeval_t */

  lte_psm_timeval_t req_active_time;

  /** Extended periodic TAU value(T3412). See @ref lte_psm_timeval_t */

  lte_psm_timeval_t ext_periodic_tau_time;
} lte_psm_setting_t;



/**
 * @struct lte_apn_setting
 * Definition of APN setting used in lte_activate_pdn().
 * @typedef lte_apn_setting_t
 * See @ref lte_apn_setting
 */

typedef struct lte_apn_setting
{
  /** Access point name. It is terminated with '\0'.
   *  Maximum length is @ref LTE_APN_LEN including '\0'. */

  int8_t   *apn;

  /** Type of IP for APN. Definition is as below.@n
   *  - @ref LTE_APN_IPTYPE_IP@n
   *  - @ref LTE_APN_IPTYPE_IPV6@n
   *  - @ref LTE_APN_IPTYPE_IPV4V6@n */

  uint8_t  ip_type;

  /** Type of Authentication. Definition is as below.@n
   *  - @ref LTE_APN_AUTHTYPE_NONE@n
   *  - @ref LTE_APN_AUTHTYPE_PAP@n
   *  - @ref LTE_APN_AUTHTYPE_CHAP@n */

  uint8_t  auth_type;

  /** Type of APN. Bit setting definition is as below.@n
   *  - @ref LTE_APN_TYPE_UNKNOWN@n
   *  - @ref LTE_APN_TYPE_DEFAULT@n
   *  - @ref LTE_APN_TYPE_MMS@n
   *  - @ref LTE_APN_TYPE_SUPL@n
   *  - @ref LTE_APN_TYPE_DUN@n
   *  - @ref LTE_APN_TYPE_HIPRI@n
   *  - @ref LTE_APN_TYPE_FOTA@n
   *  - @ref LTE_APN_TYPE_IMS@n
   *  - @ref LTE_APN_TYPE_CBS@n
   *  - @ref LTE_APN_TYPE_IA@n
   *  - @ref LTE_APN_TYPE_EMERGENCY@n */

  uint32_t apn_type;

  /** User name. It is terminated with '\0'.
   *  Maximum length is @ref LTE_APN_USER_NAME_LEN including '\0'. */

  int8_t   *user_name;

  /** Password. It is terminated with '\0'.
   *  Maximum length is @ref LTE_APN_PASSWD_LEN including '\0'. */

  int8_t   *password;
} lte_apn_setting_t;

/**
 * @struct lte_ipaddr
 * Definition of ip address used in lte_pdn_t.
 * @typedef lte_ipaddr_t
 * See @ref lte_ipaddr
 */

typedef struct lte_ipaddr
{
  /** Type of IP address. Definition is as below.@n
   *  - @ref LTE_IPTYPE_V4@n
   *  - @ref LTE_IPTYPE_V6@n */

  uint8_t ip_type;

  /** IP address. It is terminated with '\0'.@n
   *  eg. (IPv4) 192.0.2.1, (IPv6) 2001:db8:85a3:0:0:8a2e:370:7334 */

  int8_t  address[LTE_IPADDR_MAX_LEN];
} lte_ipaddr_t;

/**
 * @struct lte_pdn
 * Definition of pdn information used in activate_pdn_cb_t.
 * @typedef lte_pdn_t
 * See @ref lte_pdn
 */

typedef struct lte_pdn
{
  /** PDN session id. The range is from
   *  @ref LTE_PDN_SESSIONID_MIN to @ref LTE_PDN_SESSIONID_MAX. */

  uint8_t      session_id;

  /** PDN active status. Definition is as below.@n
   *  - @ref LTE_PDN_ACTIVE@n
   *  - @ref LTE_PDN_DEACTIVE@n */

  uint8_t      active;

  /** APN type of PDN. Bit setting definition is as below.@n
   *  - @ref LTE_APN_TYPE_UNKNOWN@n
   *  - @ref LTE_APN_TYPE_DEFAULT@n
   *  - @ref LTE_APN_TYPE_MMS@n
   *  - @ref LTE_APN_TYPE_SUPL@n
   *  - @ref LTE_APN_TYPE_DUN@n
   *  - @ref LTE_APN_TYPE_HIPRI@n
   *  - @ref LTE_APN_TYPE_FOTA@n
   *  - @ref LTE_APN_TYPE_IMS@n
   *  - @ref LTE_APN_TYPE_CBS@n
   *  - @ref LTE_APN_TYPE_IA@n
   *  - @ref LTE_APN_TYPE_EMERGENCY@n */

  uint32_t     apn_type;

  /** Number of valid ip addresses */

  uint8_t      ipaddr_num;

  /** IP address information. See @ref lte_ipaddr_t */

  lte_ipaddr_t address[LTE_PDN_IPADDR_MAX_COUNT];

  /** IMS registored status.
   *  This is valid when LTE_APN_TYPE_IMS is set in apn_type.@n
   *  Definition is as below.@n
   *  - @ref LTE_IMS_NOT_REGISTERED@n
   *  - @ref LTE_IMS_REGISTERED@n */

  uint8_t      ims_register;

  /** Status of data communication enability. Definition is as below.@n
   *  - @ref LTE_DATA_ALLOW@n
   *  - @ref LTE_DATA_DISALLOW@n */

  uint8_t      data_allow;

  /** Status of roaming data communication enability.
   *  Definition is as below.@n
   *  - @ref LTE_DATA_ALLOW@n
   *  - @ref LTE_DATA_DISALLOW@n */

  uint8_t      data_roaming_allow;
} lte_pdn_t;

/**
 * @struct lte_reject_cause
 * Definition of LTE network reject cause used in lte_nw_err_info_t.
 * @typedef lte_reject_cause_t
 * See @ref lte_reject_cause
 */

typedef struct lte_reject_cause
{

  /**
   * Category of reject cause. Definition is as below..
   *  - @ref LTE_REJECT_CATEGORY_EMM@n
   *  - @ref LTE_REJECT_CATEGORY_ESM@n
   */

  uint8_t category;

  /**
   * Value of LTE newtwork reject cause.
   * Definition is See 3GPP TS 24.008 13.7.0
   */

  uint8_t value;
} lte_reject_cause_t;

/**
 * @struct lte_nw_err_info
 * Definition of LTE network error infomation used in lte_netinfo_t.
 * @typedef lte_nw_err_info_t
 * See @ref lte_nw_err_info
 */

typedef struct lte_nw_err_info
{
  /**
   * Type of LTE network error. Definition is as below.@n
   *  - @ref LTE_NETERR_MAXRETRY@n
   *  - @ref LTE_NETERR_REJECT@n
   *  - @ref LTE_NETERR_NWDTCH@n
   */

  uint8_t            err_type;

  /**
   * LTE network attach request reject cause. It can be referneced when
   *  - @ref LTE_NETERR_REJECT is ser in err_type field@n
   *  See @ref lte_reject_cause_t
   */

  lte_reject_cause_t reject_cause;
} lte_nw_err_info_t;

/**
 * @struct lte_netinfo
 * Definition of lte network information used in get_netinfo_cb_t.
 * @typedef lte_netinfo_t
 * See @ref lte_netinfo
 */

typedef struct lte_netinfo
{
  /** LTE network status. Definition is as below.@n
   *  - @ref LTE_NETSTAT_NOT_REG_NOT_SEARCHING@n
   *  - @ref LTE_NETSTAT_REG_HOME@n
   *  - @ref LTE_NETSTAT_NOT_REG_SEARCHING@n
   *  - @ref LTE_NETSTAT_REG_DENIED@n
   *  - @ref LTE_NETSTAT_UNKNOWN@n
   *  - @ref LTE_NETSTAT_REG_ROAMING@n
   *  - @ref LTE_NETSTAT_REG_SMS_ONLY_HOME@n
   *  - @ref LTE_NETSTAT_REG_SMS_ONLY_ROAMING@n
   *  - @ref LTE_NETSTAT_NOT_REG_EMERGENCY@n
   *  - @ref LTE_NETSTAT_REG_CSFB_NOT_PREF_HOME@n
   *  - @ref LTE_NETSTAT_REG_CSFB_NOT_PREF_ROAMING@n */

  uint8_t           nw_stat;

  /**
   * LTE network error infomation. It can be referneced when
   *  - @ref LTE_NETSTAT_REG_DENIED is set in nw_stat field.@n
   *  See @ref lte_nw_err_info_t
   */

  lte_nw_err_info_t nw_err;

  /** Number of PDN status informations. */

  uint8_t           pdn_num;

  /** List of PDN status. See @ref lte_pdn_t*/

  lte_pdn_t         *pdn_stat;
} lte_netinfo_t;

/**
 * @struct lte_error_info
 * Definition of error information used in lte_get_errinfo().
 * @typedef lte_errinfo_t
 * See @ref lte_error_info
 */

typedef struct lte_error_info
{
  /** Enable error indicator. Bit setting definition is as below.@n
   *  - @ref LTE_ERR_INDICATOR_ERRCODE@n
   *  - @ref LTE_ERR_INDICATOR_ERRNO@n
   *  - @ref LTE_ERR_INDICATOR_ERRSTR@n */

  uint8_t err_indicator;

  /** Last error code. See 3GPP TS 27.007 9.2 */

  int32_t err_result_code;

  /** Last error no. See <errno.h> */

  int32_t err_no;

  /** Error string use debug only */

  uint8_t err_string[LTE_ERROR_STRING_MAX_LEN];
} lte_errinfo_t;


/**
 * @struct lte_ce_setting
 * Definition of CE settings used in lte_set_ce().
 * This is notified by get_ce_cb_t
 * @typedef lte_ce_setting_t
 * See @ref lte_ce_setting
 */

typedef struct lte_ce_setting
{
  /** Mode A enable. Definition is as below.@n
   *  - @ref LTE_ENABLE@n
   *  - @ref LTE_DISABLE@n */

  bool mode_a_enable;

  /** Mode B enable. Definition is as below.@n
   *  - @ref LTE_ENABLE@n
   *  - @ref LTE_DISABLE@n */

  bool mode_b_enable;
} lte_ce_setting_t;

/**
 * @struct lte_siminfo
 * Definition of parameters for SIM information.
 * This is notified by get_siminfo_cb_t
 * @typedef lte_siminfo_t
 * See @ref lte_siminfo
 */

typedef struct lte_siminfo
{
  /** Indicates which parameter to get.
   *  Bit setting definition is as below.@n
   *  - @ref LTE_SIMINFO_GETOPT_MCCMNC@n
   *  - @ref LTE_SIMINFO_GETOPT_SPN@n
   *  - @ref LTE_SIMINFO_GETOPT_ICCID@n
   *  - @ref LTE_SIMINFO_GETOPT_IMSI@n
   *  - @ref LTE_SIMINFO_GETOPT_GID1@n
   *  - @ref LTE_SIMINFO_GETOPT_GID2@n
   */

  uint32_t option;

  /** Mobile Country Code (000-999). It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_MCCMNC is set in option field. */

  uint8_t  mcc[LTE_SIMINFO_MCC_DIGIT];

  /** Digit number of mnc(2-3). It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_MCCMNC is set in option field. */

  uint8_t  mnc_digit;

  /** Mobile Network Code (00-999). It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_MCCMNC is set in option field. */

  uint8_t  mnc[LTE_SIMINFO_MNC_DIGIT_MAX];

  /** Length of Service provider name. It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_SPN is set in option field. */

  uint8_t  spn_len;

  /** Service provider name. It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_SPN is set in option field. */

  uint8_t  spn[LTE_SIMINFO_SPN_LEN];

  /** Length of ICCID. It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_ICCID is set in option field. */

  uint8_t  iccid_len;

  /** ICCID. It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_ICCID is set in option field.@n
   *  If the ICCID is 19 digits, "F" is set to the 20th digit. */

  uint8_t  iccid[LTE_SIMINFO_ICCID_LEN];

  /** Length of IMSI. It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_IMSI is set in option field. */

  uint8_t  imsi_len;

  /** International Mobile Subscriber Identity. It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_IMSI is set in option field. */

  uint8_t  imsi[LTE_SIMINFO_IMSI_LEN];

  /** Length of GID1. It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_GID1 is set in option field. */

  uint8_t  gid1_len;

  /** Group Identifier Level 1. It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_GID1 is set in option field. */

  uint8_t  gid1[LTE_SIMINFO_GID_LEN];

  /** Length of GID2. It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_GID2 is set in option field. */

  uint8_t  gid2_len;

  /** Group Identifier Level 1. It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_GID2 is set in option field. */

  uint8_t  gid2[LTE_SIMINFO_GID_LEN];
} lte_siminfo_t;

/** Definition of callback function.
 *  Since lte_get_version() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_version().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] version : The version information of the modem.
 *                      See @ref lte_version_t
 */

typedef void (*get_ver_cb_t)(uint32_t result, lte_version_t *version);

/** Definition of callback function.
 *  Since lte_get_phoneno() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_phoneno().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] errcause : Error cause. It is set only if the result is
 *                       not successful. As below value stored.@n
 * - @ref LTE_ERR_WAITENTERPIN@n
 * - @ref LTE_ERR_UNEXPECTED@n
 *
 * @param[in] phoneno : A character string indicating phone number.
 *                      It is terminated with '\0'
 */

typedef void (*get_phoneno_cb_t)(uint32_t result, uint8_t errcause,
                                 int8_t *phoneno);

/** Definition of callback function.
 *  Since lte_get_imsi() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_imsi().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] errcause : Error cause. It is set only if the result is
 *                       not successful. As below value stored.@n
 * - @ref LTE_ERR_WAITENTERPIN@n
 * - @ref LTE_ERR_UNEXPECTED@n
 * @param[in] imsi : A character string indicating IMSI.
 *                   It is terminated with '\0'
 */

typedef void (*get_imsi_cb_t)(uint32_t result, uint8_t errcause,
                              int8_t *imsi);

/** Definition of callback function.
 *  Since lte_get_imei() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_imei().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] imei : A character string indicating IMEI.
 *                   It is terminated with '\0'
 */

typedef void (*get_imei_cb_t)(uint32_t result, int8_t *imei);

/** Definition of callback function.
 *  Since lte_get_pinset() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_pinset().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] pinset : PIN settings information.
 *                     See @ref lte_getpin_t
 */

typedef void (*get_pinset_cb_t)(uint32_t result, lte_getpin_t *pinset);

/** Definition of callback function.
 *  Since lte_set_pinenable() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_set_pinenable().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] attemptsleft : Number of attempts left.
 *                           It is set only if the result is not successful.
 */

typedef void (*set_pinenable_cb_t)(uint32_t result, uint8_t attemptsleft);

/** Definition of callback function.
 *  Since lte_change_pin() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_change_pin().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] attemptsleft : Number of attempts left.
 *                           It is set only if the result is not successful.
 */

typedef void (*change_pin_cb_t)(uint32_t result, uint8_t attemptsleft);

/** Definition of callback function.
 *  Since lte_enter_pin() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_enter_pin().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] simstat : State after PIN enter.
 *                      As below value stored.@n
 * - @ref LTE_PINSTAT_READY@n
 * - @ref LTE_PINSTAT_SIM_PIN@n
 * - @ref LTE_PINSTAT_SIM_PUK@n
 * - @ref LTE_PINSTAT_PH_SIM_PIN@n
 * - @ref LTE_PINSTAT_PH_FSIM_PIN@n
 * - @ref LTE_PINSTAT_PH_FSIM_PUK@n
 * - @ref LTE_PINSTAT_SIM_PIN2@n
 * - @ref LTE_PINSTAT_SIM_PUK2@n
 * - @ref LTE_PINSTAT_PH_NET_PIN@n
 * - @ref LTE_PINSTAT_PH_NET_PUK@n
 * - @ref LTE_PINSTAT_PH_NETSUB_PIN@n
 * - @ref LTE_PINSTAT_PH_NETSUB_PUK@n
 * - @ref LTE_PINSTAT_PH_SP_PIN@n
 * - @ref LTE_PINSTAT_PH_SP_PUK@n
 * - @ref LTE_PINSTAT_PH_CORP_PIN@n
 * - @ref LTE_PINSTAT_PH_CORP_PUK@n
 * @param[in] attemptsleft : Number of attempts left.
 *                           It is set only if the result is not successful.
 *                           If simstat is other than PIN, PUK, PIN2, PUK2,
 *                           set the number of PIN.
 */

typedef void (*enter_pin_cb_t)(uint32_t result,
                               uint8_t simstat,
                               uint8_t attemptsleft);

/** Definition of callback function.
 *  Since lte_get_localtime() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_localtime().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] localtime : Local time. See @ref lte_localtime_t
 */

typedef void (*get_localtime_cb_t)(uint32_t result,
                                   lte_localtime_t *localtime);

/** Definition of callback function.
 *  Since lte_get_operator() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_operator().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] oper : A character string indicating network operator.
 *                   It is terminated with '\0' If it is not connected,
 *                   the first character is '\0'.
 */

typedef void (*get_operator_cb_t)(uint32_t result, int8_t *oper);

/** Definition of callback function.
 *  Since lte_get_edrx() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_edrx().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] settings : eDRX settings. See @ref lte_edrx_setting_t
 */

typedef void (*get_edrx_cb_t)(uint32_t result, lte_edrx_setting_t *settings);

/** Definition of callback function.
 *  Since lte_set_edrx() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_set_edrx().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 */

typedef void (*set_edrx_cb_t)(uint32_t result);

/** Definition of callback function.
 *  Since lte_get_psm() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_psm().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] settings : PSM settings. See @ref lte_psm_setting_t
 */

typedef void (*get_psm_cb_t)(uint32_t result, lte_psm_setting_t *settings);

/** Definition of callback function.
 *  Since lte_set_psm() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_set_psm().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 */

typedef void (*set_psm_cb_t)(uint32_t result);

/** Definition of callback function.
 *  Since lte_get_ce() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_ce().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] settings : CE settings. See @ref lte_ce_setting_t
 */

typedef void (*get_ce_cb_t)(uint32_t result, lte_ce_setting_t *settings);

/** Definition of callback function.
 *  Since lte_set_ce() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_set_ce().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 */

typedef void (*set_ce_cb_t)(uint32_t result);

/** Definition of callback function.
 *  When the SIM state changes, the SIM state is
 *  reported by this function.
 * @param[in] simstat : The SIM state.
 *                      As below value stored.@n
 * - @ref LTE_SIMSTAT_REMOVAL@n
 * - @ref LTE_SIMSTAT_INSERTION@n
 * - @ref LTE_SIMSTAT_WAIT_PIN_UNLOCK@n
 * - @ref LTE_SIMSTAT_PERSONAL_FAILED@n
 * - @ref LTE_SIMSTAT_ACTIVATE@n
 * - @ref LTE_SIMSTAT_DEACTIVATE@n
 */

typedef void (*simstat_report_cb_t)(uint32_t simstat);

/** Definition of callback function.
 *  When the local time changes, the local time is
 *  reported by this function.
 * @param[in] localtime : Local time. See @ref lte_localtime_t
 */

typedef void (*localtime_report_cb_t)(lte_localtime_t *localtime);

/** Definition of callback function.
 *  The quality information is reported by this function. It is reported
 *  at intervals of the set report period.
 * @param[in] quality : Quality information. See @ref lte_quality_t
 */

typedef void (*quality_report_cb_t)(lte_quality_t *quality);

/** Definition of callback function.
 *  The cell information is reported by this function. It is reported
 *  at intervals of the set report period.
 * @param[in] cellinfo : Cell information. See @ref lte_cellinfo_t
 */

typedef void (*cellinfo_report_cb_t)(lte_cellinfo_t *cellinfo);

/** Definition of callback function.
 *  Since lte_radio_on() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_radio_on().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 */

typedef void (*radio_on_cb_t)(uint32_t result);

/** Definition of callback function.
 *  Since lte_radio_off() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_radio_off().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 */

typedef void (*radio_off_cb_t)(uint32_t result);

/** Definition of callback function.
 *  Since lte_get_netinfo() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_netinfo().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] info : Pointer of LTE network infomation.
 *                   See @ref lte_netinfo_t
 */

typedef void (*get_netinfo_cb_t)(uint32_t result, lte_netinfo_t *info);

/** Definition of callback function.
 *  Since lte_get_imscap() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_imscap.
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] imscap : The IMS capability.
 *                     As below value stored.@n
 * - @ref LTE_ENABLE@n
 * - @ref LTE_DISABLE@n
 */

typedef void (*get_imscap_cb_t)(uint32_t result, bool imscap);

/** Definition of callback function.
 *  Since lte_activate_pdn() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_activate_pdn.
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * - @ref LTE_RESULT_CANCEL@n
 * @param[in] pdn : The connect pdn information. See @ref lte_pdn_t
 */

typedef void (*activate_pdn_cb_t)(uint32_t result, lte_pdn_t *pdn);

/** Definition of callback function.
 *  Since lte_deactivate_pdn() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_deactivate_pdn.
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 */

typedef void (*deactivate_pdn_cb_t)(uint32_t result);

/** Definition of callback function.
 *  Since lte_dataallow() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_dataallow.
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 */

typedef void (*data_allow_cb_t)(uint32_t result);

/** Definition of callback function.
 *  The modem restart is reported by this function. It is reported
 *  at modem reset.
 * @param[in] reason : Reason of modem restart.
 *                     As below value stored.@n
 * - @ref LTE_RESTART_USER_INITIATED@n
 * - @ref LTE_RESTART_MODEM_INITIATED@n
 */

typedef void (*restart_report_cb_t)(uint32_t reason);

/** Definition of callback function.
 *  The change LTE network infomation is reported by this function.
 *  It is reported at LTE network connection status.
 * @param[in] info : Pointer of LTE network infomation.
 *                   See @ref lte_netinfo_t
 */

typedef void (*netinfo_report_cb_t)(lte_netinfo_t *info);

/** Definition of callback function.
 *  Since lte_get_siminfo() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_siminfo().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] siminfo : SIM information. See @ref lte_siminfo_t
 */

typedef void (*get_siminfo_cb_t)(uint32_t result, lte_siminfo_t *siminfo);

/** Definition of callback function.
 *  Since lte_get_dynamic_edrx_param() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_dynamic_edrx_param().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] param : eDRX dynamic parameter. See @ref lte_edrx_setting_t
 */

typedef void (*get_dynamic_edrx_param_cb_t)(uint32_t result,
                                            lte_edrx_setting_t *param);

/** Definition of callback function.
 *  Since lte_get_dynamic_psm_param() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_dynamic_psm_param().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] param : PSM dynamic parameter. See @ref lte_psm_setting_t
 */

typedef void (*get_dynamic_psm_param_cb_t)(uint32_t result,
                                           lte_psm_setting_t *param);

/** Definition of callback function.
 *  Since lte_get_quality() is an asynchronous API,
 *  the quality information is notified by this function
 * @param[in] result : The result of lte_get_quality().
 *                     As below value stored.@n
 * - @ref LTE_RESULT_OK@n
 * - @ref LTE_RESULT_ERROR@n
 * @param[in] quality : Quality information. See @ref lte_quality_t
 */

typedef void (*get_quality_cb_t)(uint32_t result,
                                 lte_quality_t *quality);

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/** @name Functions */
/** @{ */

/**
 * Initialize the LTE library.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_initialize(void);

/**
 * Finalize the LTE library.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_finalize(void);

/**
 * Register a callback to notify that the modem has started or restarted.@n
 * You must call this function after lte_initialize.@n
 * The callback will be invoked if the modem starts successfully
 * after calling lte_power_on.@n Some APIs have to wait until
 * this callback is invoked. If no wait, those API return
 * with an error.(-ENETDOWN)@n The callback will also be invoked
 * if a restart occurs on the modem. You can retrieve restart cause
 * with argument of callback.@n
 * Attention to the following when @ref LTE_RESTART_MODEM_INITIATED is set.
 * - (Asynchronous API) Callback is canceled and API becomes available.
 * - (Synchronization API) API returns with an error.
 *   The return value is -ENETDOWN for the LTE API.
 *   The errno is ENETDOWN for the socket API.
 * - It shuold close the socket by user application.
 *
 * @param [in] restart_callback: Callback function to notify that
 *                               modem restarted.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_report_restart(restart_report_cb_t restart_callback);

/**
 * Power on the modem.@n
 * You must call this function after lte_set_report_restart.@n
 * The callback which registered by lte_set_report_restart
 * will be invoked if the modem starts successfully.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_power_on(void);

/**
 * Power off the modem@n
 * Attention to the following when this API calling.@n
 * - For asynchronous API
 *   - callback is canceled.
 * - For synchronous API
 *   - API returns with an error.
 *     - The return value is -ENETDOWN for the LTE API.
 *     - The errno is ENETDOWN for the socket API.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_power_off(void);

/**
 * Radio on and start to search for network.
 *
 * @param [in] callback: Callback function to notify that
 *                       radio on is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_radio_on(radio_on_cb_t callback);

/**
 * Radio off.@n
 * If this function is called with LTE already connected,
 * the network will be disconnected.
 *
 * @param [in] callback: Callback function to notify that
 *                       radio off is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_radio_off(radio_off_cb_t callback);

/**
 * Get current network infomation of the LTE.
 *
 * @param [in] callback: Callback function to notify that
 *                       get network infomation completed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_netinfo(get_netinfo_cb_t callback);

/**
 * PDN activation.@n
 * It connects to the PDN of the APN specified by the argument.
 * When connecting for the first time, you need to set LTE_APN_TYPE_IA
 * to APN type.@n When the connection is successful,
 * the IP address obtained from the network is set in the modem.
 *
 * @param [in] apn: The pointer of the apn setting.
 *                  See @ref lte_apn_setting_t for valid parameters.
 * @param [in] callback: Callback function to notify that
 *                       PDN activation completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_activate_pdn(lte_apn_setting_t *apn, activate_pdn_cb_t callback);

/**
 * PDN activation cancel.@n
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_activate_pdn_cancel(void);

/**
 * PDN deactivation.@n
 * Disconnect the PDN corresponding to the session ID
 * obtained by lte_activate_pdn.@n When the disconnection is successful,
 * the IP address set for the modem is released.
 *
 * @param [in] session_id: The numeric value of the session ID.
 *                         Use the value obtained by the lte_activate_pdn.
 * @param [in] callback: Callback function to notify that
 *                       LTE PDN deactivation completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_deactivate_pdn(uint8_t session_id, deactivate_pdn_cb_t callback);

/**
 * Allow or disallow to data communication for specified PDN.@n
 * If the application performs data communication in the disallow state,
 * the modem discards the data.
 *
 * @param [in] session_id: The numeric value of the session ID.
 *                         Use the value obtained by the lte_activate_pdn.
 * @param [in] allow: Allow or disallow to data communication for
 *                    all network. Definition is as below.@n
 *  - @ref LTE_DATA_ALLOW@n
 *  - @ref LTE_DATA_DISALLOW@n
 * @param [in] roaming_allow: Allow or disallow to data communication for
 *                            roaming network. Definition is as below.@n
 *  - @ref LTE_DATA_ALLOW@n
 *  - @ref LTE_DATA_DISALLOW@n
 * @param [in] callback: Callback function to notify that
 *                       configuration has changed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_data_allow(uint8_t session_id, uint8_t allow,
                       uint8_t roaming_allow, data_allow_cb_t callback);

/**
 * Get whether the modem supports IMS or not.
 *
 * @param [in] callback: Callback function to notify that
 *                       get of IMS capability.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_imscap(get_imscap_cb_t callback);

/**
 * Get the version of the modem.
 *
 * @param [in] callback: Callback function to notify that
 *                       get of version is completed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_version(get_ver_cb_t callback);

/**
 * Get phone number.
 *
 * @param [in] callback: Callback function to notify that
 *                       get of phoneno is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_phoneno(get_phoneno_cb_t callback);

/**
 * Get International Mobile Subscriber Identity.
 *
 * @param [in] callback: Callback function to notify that
 *                       get of IMSI is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_imsi(get_imsi_cb_t callback);

/**
 * Get International Mobile Equipment Identifier.
 *
 * @param [in] callback: Callback function to notify that
 *                       get of IMEI is completed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_imei(get_imei_cb_t callback);

/**
 * Get Personal Identification Number settings information.
 *
 * @param [in] callback: Callback function to notify that
 *                       get of PIN settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_pinset(get_pinset_cb_t callback);

/**
 * Set Personal Identification Number enable.
 *
 * @param [in] enable: "Enable" or "Disable".
 *                      Definition is as below.@n
 *  - @ref LTE_PIN_ENABLE@n
 *  - @ref LTE_PIN_DISABLE@n
 * @param [in] pincode: Current PIN code. Minimum number of digits is 4.
 *                      Maximum number of digits is 8, end with '\0'.
 *                      (i.e. Max 9 byte)
 * @param [in] callback: Callback function to notify that
 *                       set of PIN enable is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_pinenable(bool enable, int8_t *pincode,
                          set_pinenable_cb_t callback);

/**
 * Change Personal Identification Number.@n
 * It can be changed only when PIN is enable.
 *
 * @param [in] target_pin: Target of change PIN.
 *                      Definition is as below.@n
 *  - @ref LTE_TARGET_PIN@n
 *  - @ref LTE_TARGET_PIN2@n
 * @param [in] pincode: Current PIN code. Minimum number of digits is 4.
 *                      Maximum number of digits is 8, end with '\0'.
 *                      (i.e. Max 9 byte)
 * @param [in] new_pincode: New PIN code. Minimum number of digits is 4.
 *                          Maximum number of digits is 8, end with '\0'.
 *                          (i.e. Max 9 byte)
 * @param [in] callback: Callback function to notify that
 *                       change of PIN is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_change_pin(int8_t target_pin, int8_t *pincode,
                       int8_t *new_pincode, change_pin_cb_t callback);

/**
 * Enter Personal Identification Number.
 *
 * @param [in] pincode: Current PIN code. Minimum number of digits is 4.
 *                      Maximum number of digits is 8, end with '\0'.
 *                      (i.e. Max 9 byte)
 * @param [in] new_pincode: If not used, set NULL.
 *                          If the PIN is SIM PUK or SIM PUK2,
 *                          the new_pincode is required.@n
 *                          Minimum number of digits is 4.
 *                          Maximum number of digits is 8,
 *                          end with '\0'. (i.e. Max 9 byte)
 * @param [in] callback: Callback function to notify that
 *                       PIN enter is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_enter_pin(int8_t *pincode, int8_t *new_pincode,
                      enter_pin_cb_t callback);

/**
 * Get local time.
 *
 * @param [in] callback: Callback function to notify that
 *                       get local time is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_localtime(get_localtime_cb_t callback);

/**
 * Get connected network operator information.
 *
 * @param [in] callback: Callback function to notify that
 *                       get network operator information is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_operator(get_operator_cb_t callback);

/**
 * Get eDRX settings.
 *
 * @param [in] callback: Callback function to notify that
 *                       get eDRX settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_edrx(get_edrx_cb_t callback);

/**
 * Set eDRX settings.
 *
 * @param [in] settings: eDRX settings
 * @param [in] callback: Callback function to notify that
 *                       get eDRX settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_edrx(lte_edrx_setting_t *settings,
                     set_edrx_cb_t callback);

/**
 * Get PSM settings.
 *
 * @param [in] callback: Callback function to notify that
 *                       get PSM settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_psm(get_psm_cb_t callback);

/**
 * Set PSM settings.
 *
 * @param [in] settings: PSM settings
 * @param [in] callback: Callback function to notify that
 *                       get PSM settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_psm(lte_psm_setting_t *settings,
                    set_psm_cb_t callback);

/**
 * Get CE settings.
 *
 * @param [in] callback: Callback function to notify that
 *                       get CE settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_ce(get_ce_cb_t callback);

/**
 * Set CE settings.
 *
 * @param [in] settings: CE settings
 * @param [in] callback: Callback function to notify that
 *                       get CE settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_ce(lte_ce_setting_t *settings,
                   set_ce_cb_t callback);

/**
 * Change the report setting of the SIM state.
 * The default report setting is disable.
 *
 * @param [in] simstat_callback: Callback function to notify that SIM state.
 *                               If NULL is set,
 *                               the report setting is disabled.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_report_simstat(simstat_report_cb_t simstat_callback);

/**
 * Change the report setting of the local time.
 * The default report setting is disable.
 *
 * @param [in] localtime_callback: Callback function to notify that
 *                                 local time. If NULL is set,
 *                                 the report setting is disabled.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_report_localtime(localtime_report_cb_t localtime_callback);

/**
 * Change the report setting of the quality information.
 * The default report setting is disable.
 *
 * @param [in] quality_callback: Callback function to notify that
 *                               quality information. If NULL is set,
 *                               the report setting is disabled.
 * @param [in] period: Reporting cycle in sec (1-4233600)
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_report_quality(quality_report_cb_t quality_callback,
                               uint32_t period);

/**
 * Change the report setting of the cell information.
 * The default report setting is disable.
 *
 * @param [in] cellinfo_callback: Callback function to notify that
 *                                cell information. If NULL is set,
 *                                the report setting is disabled.
 * @param [in] period: Reporting cycle in sec (1-4233600)
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_report_cellinfo(cellinfo_report_cb_t cellinfo_callback,
                                uint32_t period);

/**
 * Change the report setting of the LTE network information.
 * The default report setting is disable.
 *
 * @param [in] netinfo_callback: Callback function to notify that
 *                               cell information. If NULL is set,
 *                               the report setting is disabled.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_report_netinfo(netinfo_report_cb_t netinfo_callback);

/**
 * Get LTE API last error information.@n
 * Call this function when LTE_RESULT_ERROR is returned by
 * callback function. Detailed error information can be obtained.
 *
 * @param [in] info: Pointer of error information.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_errinfo(lte_errinfo_t *info);

/**
 * Get SIM information such as MCC/MNC.
 *
 * @param [in] option:   Indicates which parameter to get.
 *                       Bit setting definition is as below.@n
 *                       - @ref LTE_SIMINFO_GETOPT_MCCMNC@n
 *                       - @ref LTE_SIMINFO_GETOPT_SPN@n
 *                       - @ref LTE_SIMINFO_GETOPT_ICCID@n
 *                       - @ref LTE_SIMINFO_GETOPT_IMSI@n
 *                       - @ref LTE_SIMINFO_GETOPT_GID1@n
 *                       - @ref LTE_SIMINFO_GETOPT_GID2@n
 * @param [in] callback: Callback function to notify that
 *                       get of SIM information is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_siminfo(uint32_t option, get_siminfo_cb_t callback);

/**
 * Get eDRX dynamic parameter.
 *
 * This API can be issued after connect to the LTE network
 * with lte_activate_pdn().
 *
 * @param [in] callback: Callback function to notify that
 *                       get eDRX dynamic parameter is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_dynamic_edrx_param(get_dynamic_edrx_param_cb_t callback);

/**
 * Get PSM dynamic parameter.
 *
 * This API can be issued after connect to the LTE network
 * with lte_activate_pdn().
 *
 * @param [in] callback: Callback function to notify that
 *                       get PSM dynamic parameter is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_dynamic_psm_param(get_dynamic_psm_param_cb_t callback);

/**
 * Get quality information.
 *
 * @param [in] callback: Callback function to notify that
 *                       get quality information is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_quality(get_quality_cb_t callback);

/** @} */

#undef EXTERN
#ifdef __cplusplus
}
#endif

/** @} */

/** @} */

#endif /* __MODULES_INCLUDE_LTE_LTE_API_H */
