/****************************************************************************
 * modules/include/lte/lte_api.h
 *
 *   Copyright 2018, 2019, 2020 Sony Semiconductor Solutions Corporation
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
 * @defgroup lte LTE Library API
 * @brief LTE library for using LTE network
 *
 * - Abbreviations and terms
 *  - PDN : Packet Data Network
 *
 *      Route for transferring packets between the terminal and LTE networks.
 *
 *  - APN : Access Point Name
 *
 *      Settings required when connecting to an LTE network.
 *
 *  - IMSI : International Mobile Subscriber Identity
 *
 *      International subscriber identification number recorded
 *      on the SIM card.
 *
 *  - IMEI : International Mobile Equipment Identifier
 *
 *      International identification number assigned to 
 *      data communication terminals
 *
 *  - PIN : Personal Identification Number
 *
 *  - MCC : Mobile Country Code
 *
 *      The mobile country code consists of three decimal digits.
 *
 *  - MNC : Mobile Network Code
 *
 *      The mobile network code consists of two or three decimal digits.
 *
 *  - eDRX : extended Discontinuous Reception
 *
 *      Communication technology that reduces power consumption
 *      by increasing the reception interval of various signals transmitted 
 *      from LTE networks.
 *
 *  - PSM : Power Saving Mode
 *
 *      Communication technology that reduces power consumption 
 *      by not communicating with the LTE network 
 *      for a certain period of time.
 *
 *  - CE : Coverage Enhancement
 *
 *      Communication technology that attempts to resend data and eventually 
 *      restores the original data even if the data is corrupted 
 *      due to weak electric field communication.
 *
 * - LTE API system
 *  - Network connection API
 *
 *      Radio ON / OFF, PDN connection establishment / destruction.
 *
 *  - Communication quality and communication state API
 *
 *      Acquisition of radio status, communication status, and local time.
 *
 *  - SIM card control API
 *
 *      Get phone number / IMSI, set the PIN, get SIM status.
 *
 *  - Modem setting API
 *
 *      Get modem firmware version and IMEI. Update communication settings.
 *
 * - API call type
 *
 *      There are two types of LTE API: synchronous and asynchronous.
 *
 *  - Synchronous API
 *    - Notifies the processing result as a return value.
 *
 *    - Blocks the task that called the API until 
 *      processing is completed on the modem.
 *
 *    - If the retrun value is -EPROTO, you can get the error code 
 *      with lte_get_errinfo.
 *
 *    - If the argument attribute is out, the argument must be allocated 
 *      by the caller.
 *
 *  - Asynchronous API
 *    - The processing result is notified by callback.
 *      The callback is invoked in the task context.
 *
 *    - Blocks the task that called the API until it requests 
 *      processing from the modem.
 *
 *    - Notifies the processing request result as a return value.
 *
 *    - The callback is registered with the argument of each API.
 *      Registration is canceled when the processing result is notified.
 *
 *    - The same API cannot be called until the processing result is notified 
 *      by the callback.(-EINPROGRESS is notified with a return value.)
 *
 *    - If the callback reports an error (LTE_RESULT_ERROR), 
 *      detailed error information can be acquired with lte_get_errinfo.
 *
 *  For some APIs, both synchronous and asynchronous APIs are available.
 *  The correspondence table of API is as follows.
 *  
 *
 * | Synchronous API                   | Asynchronous API                |
 * | :-------------------------------- | :------------------------------ |
 * | @ref lte_initialize               |                                 |
 * | @ref lte_finalize                 |                                 |
 * | @ref lte_set_report_restart       |                                 |
 * | @ref lte_power_on                 |                                 |
 * | @ref lte_power_off                |                                 |
 * | @ref lte_set_report_netinfo       |                                 |
 * | @ref lte_set_report_simstat       |                                 |
 * | @ref lte_set_report_localtime     |                                 |
 * | @ref lte_set_report_quality       |                                 |
 * | @ref lte_set_report_cellinfo      |                                 |
 * | @ref lte_get_errinfo              |                                 |
 * | @ref lte_activate_pdn_cancel      |                                 |
 * | @ref lte_radio_on_sync            | @ref lte_radio_on               |
 * | @ref lte_radio_off_sync           | @ref lte_radio_off              |
 * | @ref lte_activate_pdn_sync        | @ref lte_activate_pdn           |
 * | @ref lte_deactivate_pdn_sync      | @ref lte_deactivate_pdn         |
 * | @ref lte_data_allow_sync          | @ref lte_data_allow             |
 * | @ref lte_get_netinfo_sync         | @ref lte_get_netinfo            |
 * | @ref lte_get_imscap_sync          | @ref lte_get_imscap             |
 * | @ref lte_get_version_sync         | @ref lte_get_version            |
 * | @ref lte_get_phoneno_sync         | @ref lte_get_phoneno            |
 * | @ref lte_get_imsi_sync            | @ref lte_get_imsi               |
 * | @ref lte_get_imei_sync            | @ref lte_get_imei               |
 * | @ref lte_get_pinset_sync          | @ref lte_get_pinset             |
 * | @ref lte_set_pinenable_sync       | @ref lte_set_pinenable          |
 * | @ref lte_change_pin_sync          | @ref lte_change_pin             |
 * | @ref lte_enter_pin_sync           | @ref lte_enter_pin              |
 * | @ref lte_get_localtime_sync       | @ref lte_get_localtime          |
 * | @ref lte_get_operator_sync        | @ref lte_get_operator           |
 * | @ref lte_get_edrx_sync            | @ref lte_get_edrx               |
 * | @ref lte_set_edrx_sync            | @ref lte_set_edrx               |
 * | @ref lte_get_psm_sync             | @ref lte_get_psm                |
 * | @ref lte_set_psm_sync             | @ref lte_set_psm                |
 * | @ref lte_get_ce_sync              | @ref lte_get_ce                 |
 * | @ref lte_set_ce_sync              | @ref lte_set_ce                 |
 * | @ref lte_get_siminfo_sync         | @ref lte_get_siminfo            |
 * | @ref lte_get_current_edrx_sync    | @ref lte_get_current_edrx       |
 * | @ref lte_get_current_psm_sync     | @ref lte_get_current_psm        |
 * | @ref lte_get_quality_sync         | @ref lte_get_quality            |
 * | @ref lte_get_cellinfo_sync        |                                 |
 *
 * @{
 * @file  lte_api.h
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

#define LTE_IPTYPE_V4         (0) /**< Internet protocol type: IPv4 */
#define LTE_IPTYPE_V6         (1) /**< Internet protocol type: IPv6 */
#define LTE_IPTYPE_V4V6       (2) /**< Internet protocol type: IPv4/v6 */

/** Internet protocol type: IP 
 *  @deprecated Use @ref LTE_IPTYPE_V4 instead. */

#define LTE_APN_IPTYPE_IP     LTE_IPTYPE_V4 

/** Internet protocol type: IPv6 
 *  @deprecated Use @ref LTE_IPTYPE_V6 instead. */

#define LTE_APN_IPTYPE_IPV6   LTE_IPTYPE_V6

/** Internet protocol type: IPv4/v6 
 *  @deprecated Use @ref LTE_IPTYPE_V4V6 instead. */

#define LTE_APN_IPTYPE_IPV4V6 LTE_IPTYPE_V4V6

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

/** Enable setting of PIN lock
 *  @deprecated Use @ref LTE_ENABLE instead. */

#define LTE_PIN_ENABLE  LTE_ENABLE

/** Disable setting of PIN lock
 *  @deprecated Use @ref LTE_DISABLE instead. */

#define LTE_PIN_DISABLE LTE_DISABLE

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

#define LTE_MCC_DIGIT     (3)  /**< Digit number of Mobile Country Code */
#define LTE_MNC_DIGIT_MAX (3)  /**< Max digit number of Mobile Network Code */

/** Digit number of mcc
 *  @deprecated Use @ref LTE_MCC_DIGIT instead. */

#define LTE_CELLINFO_MCC_DIGIT     LTE_MCC_DIGIT

/** Max digit number of mnc
 *  @deprecated Use @ref LTE_MNC_DIGIT_MAX instead. */

#define LTE_CELLINFO_MNC_DIGIT_MAX LTE_MNC_DIGIT_MAX

#define LTE_EDRX_ACTTYPE_WBS1     (0) /**< E-UTRAN (WB-S1 mode)   */
#define LTE_EDRX_ACTTYPE_NBS1     (1) /**< E-UTRAN (NB-S1 mode)   */
#define LTE_EDRX_ACTTYPE_ECGSMIOT (2) /**< EC-GSM-IoT (A/Gb mode) */
#define LTE_EDRX_ACTTYPE_GSM      (3) /**< GSM (A/Gb mode)        */
#define LTE_EDRX_ACTTYPE_IU       (4) /**< UTRAN (Iu mode)        */
#define LTE_EDRX_ACTTYPE_NOTUSE   (5) /**< eDRX is not running    */

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

/** Indicates to get for Mobile Country Code/Mobile Network Code of SIM */

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

/** Digit number of mcc
 *  @deprecated Use @ref LTE_MCC_DIGIT instead. */

#define LTE_SIMINFO_MCC_DIGIT      LTE_MCC_DIGIT

/** Max digit number of mnc
 *  @deprecated Use @ref LTE_MNC_DIGIT_MAX instead. */

#define LTE_SIMINFO_MNC_DIGIT_MAX  LTE_MNC_DIGIT_MAX

#define LTE_SIMINFO_SPN_LEN   (16)  /**< Maximum length of SPN */
#define LTE_SIMINFO_ICCID_LEN (10)  /**< Maximum length of ICCCID */
#define LTE_SIMINFO_IMSI_LEN  (15)  /**< Maximum length of IMSI */
#define LTE_SIMINFO_GID_LEN   (128) /**< Maximum length of GID */

#define LTE_PHONENO_LEN  (41)  /**< Maximum length of phone number */
#define LTE_IMEI_LEN     (16)  /**< Maximum length of IMEI */
#define LTE_OPERATOR_LEN (17)  /**< Maximum length of network operator */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @struct lte_version
 *
 * Definition of version information of the modem.
 * This is notified by get_ver_cb_t
 *
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
 *
 * Definition of PIN setting information.
 * This is notified by get_pinset_cb_t
 *
 * @typedef lte_getpin_t
 * See @ref lte_getpin
 */

typedef struct lte_getpin
{
  /** PIN enable. Definition is as below.
   *  - @ref LTE_ENABLE
   *  - @ref LTE_DISABLE */

  uint8_t enable;

  /** PIN status. Refer to the this parameter only
      when enable is @ref LTE_ENABLE. */

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
 *
 * Definition of local time. This is notified by
 * get_localtime_cb_t and localtime_report_cb_t
 *
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
 *
 * Definition of parameters for quality information.
 * This is reported by quality_report_cb_t
 * and notified by get_quality_cb_t
 *
 * @typedef lte_quality_t
 * See @ref lte_quality
 */

typedef struct lte_quality
{
  /** Valid flag. Definition is as below.
   *  - @ref LTE_VALID
   *  - @ref LTE_INVALID
   *
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
 *
 * Definition of parameters for cell information.
 * This is reported by cellinfo_report_cb_t
 *
 * @typedef lte_cellinfo_t
 * See @ref lte_cellinfo
 */

typedef struct lte_cellinfo
{
  /** Valid flag. Definition is as below.
   *  - @ref LTE_VALID
   *  - @ref LTE_INVALID
   *
   *  Refer to the following parameters only when this flag
   *  is @ref LTE_VALID. This is because valid parameters can not be
   *  acquired when RF function is OFF and so on */

  bool     valid;

  /** Physical cell ID (0-503) */

  uint32_t phycell_id;

  /** EARFCN (0-262143) */

  uint32_t earfcn;

  /** Mobile Country Code (000-999) */

  uint8_t  mcc[LTE_MCC_DIGIT];

  /** Digit number of Mobile Network Code(2-3) */

  uint8_t  mnc_digit;

  /** Mobile Network Code (00-999) */

  uint8_t  mnc[LTE_MNC_DIGIT_MAX];
} lte_cellinfo_t;

/**
 * @struct lte_edrx_setting
 *
 * Definition of eDRX settings used in lte_set_edrx().
 * This is notified by get_edrx_cb_t
 *
 * @typedef lte_edrx_setting_t
 * See @ref lte_edrx_setting
 */

typedef struct lte_edrx_setting
{
  /** eDRX act type. Definition is as below.
   *  - @ref LTE_EDRX_ACTTYPE_WBS1
   *  - @ref LTE_EDRX_ACTTYPE_NBS1
   *  - @ref LTE_EDRX_ACTTYPE_ECGSMIOT
   *  - @ref LTE_EDRX_ACTTYPE_GSM
   *  - @ref LTE_EDRX_ACTTYPE_IU
   *  - @ref LTE_EDRX_ACTTYPE_NOTUSE */

  uint8_t  act_type;

  /** eDRX enable. Definition is as below.
   *  - @ref LTE_ENABLE
   *  - @ref LTE_DISABLE */

  bool     enable;

  /** eDRX cycle.
   *  This variable is not vaild when LTE_EDRX_ACTTYPE_NOTUSE
   *  is set to act_type.
   *  Definitions are below:
   *  - @ref LTE_EDRX_CYC_512
   *  - @ref LTE_EDRX_CYC_1024
   *  - @ref LTE_EDRX_CYC_2048
   *  - @ref LTE_EDRX_CYC_4096
   *  - @ref LTE_EDRX_CYC_6144
   *  - @ref LTE_EDRX_CYC_8192
   *  - @ref LTE_EDRX_CYC_10240
   *  - @ref LTE_EDRX_CYC_12288
   *  - @ref LTE_EDRX_CYC_14336
   *  - @ref LTE_EDRX_CYC_16384
   *  - @ref LTE_EDRX_CYC_32768
   *  - @ref LTE_EDRX_CYC_65536
   *  - @ref LTE_EDRX_CYC_131072
   *  - @ref LTE_EDRX_CYC_262144 */

  uint32_t edrx_cycle;

  /** Paging time window.
   *  This variable is not vaild when LTE_EDRX_ACTTYPE_NOTUSE
   *  is set to act_type.
   *  Definitions are below:
   *  - @ref LTE_EDRX_PTW_128
   *  - @ref LTE_EDRX_PTW_256
   *  - @ref LTE_EDRX_PTW_384
   *  - @ref LTE_EDRX_PTW_512
   *  - @ref LTE_EDRX_PTW_640
   *  - @ref LTE_EDRX_PTW_768
   *  - @ref LTE_EDRX_PTW_896
   *  - @ref LTE_EDRX_PTW_1024
   *  - @ref LTE_EDRX_PTW_1152
   *  - @ref LTE_EDRX_PTW_1280
   *  - @ref LTE_EDRX_PTW_1408
   *  - @ref LTE_EDRX_PTW_1536
   *  - @ref LTE_EDRX_PTW_1664
   *  - @ref LTE_EDRX_PTW_1792
   *  - @ref LTE_EDRX_PTW_1920
   *  - @ref LTE_EDRX_PTW_2048 */

  uint32_t ptw_val;
} lte_edrx_setting_t;

/**
 * @struct lte_psm_timeval
 *
 * Definition of timer information for PSM
 *
 * @typedef lte_psm_timeval_t
 * See @ref lte_psm_timeval
 */

typedef struct lte_psm_timeval
{
  /** Unit of timer value. Definition is as below.
   *  - When kind of timer is Requested Active Time
   *    - @ref LTE_PSM_T3324_UNIT_2SEC
   *    - @ref LTE_PSM_T3324_UNIT_1MIN
   *    - @ref LTE_PSM_T3324_UNIT_6MIN
   *    - @ref LTE_PSM_T3324_UNIT_DEACT
   *
   *  - When kind of timer is Extended periodic TAU Time
   *    - @ref LTE_PSM_T3412_UNIT_2SEC
   *    - @ref LTE_PSM_T3412_UNIT_30SEC
   *    - @ref LTE_PSM_T3412_UNIT_1MIN
   *    - @ref LTE_PSM_T3412_UNIT_10MIN
   *    - @ref LTE_PSM_T3412_UNIT_1HOUR
   *    - @ref LTE_PSM_T3412_UNIT_10HOUR
   *    - @ref LTE_PSM_T3412_UNIT_320HOUR
   *    - @ref LTE_PSM_T3412_UNIT_DEACT */

  uint8_t unit;

  /** Timer value (0-31) */

  uint8_t time_val;
} lte_psm_timeval_t;

/**
 * @struct lte_psm_setting
 *
 * Definition of PSM settings used in lte_set_psm().
 * This is notified by get_psm_cb_t
 *
 * @typedef lte_psm_setting_t
 * See @ref lte_psm_setting
 */

typedef struct lte_psm_setting
{
  /** PSM enable. Definition is as below.
   *  - @ref LTE_ENABLE
   *  - @ref LTE_DISABLE */

  bool              enable;

  /** Requested Active Time value(T3324). See @ref lte_psm_timeval_t */

  lte_psm_timeval_t req_active_time;

  /** Extended periodic TAU value(T3412). See @ref lte_psm_timeval_t */

  lte_psm_timeval_t ext_periodic_tau_time;
} lte_psm_setting_t;



/**
 * @struct lte_apn_setting
 *
 * Definition of APN setting used in lte_activate_pdn().
 *
 * @typedef lte_apn_setting_t
 * See @ref lte_apn_setting
 */

typedef struct lte_apn_setting
{
  /** Access point name. It is terminated with '\0'.
   *  Maximum length is @ref LTE_APN_LEN including '\0'. */

  int8_t   *apn;

  /** Type of IP for APN. Definition is as below.
   *  - @ref LTE_APN_IPTYPE_IP
   *  - @ref LTE_APN_IPTYPE_IPV6
   *  - @ref LTE_APN_IPTYPE_IPV4V6 */

  uint8_t  ip_type;

  /** Type of Authentication. Definition is as below.
   *  - @ref LTE_APN_AUTHTYPE_NONE
   *  - @ref LTE_APN_AUTHTYPE_PAP
   *  - @ref LTE_APN_AUTHTYPE_CHAP */

  uint8_t  auth_type;

  /** Type of APN. Bit setting definition is as below.
   *  - @ref LTE_APN_TYPE_UNKNOWN
   *  - @ref LTE_APN_TYPE_DEFAULT
   *  - @ref LTE_APN_TYPE_MMS
   *  - @ref LTE_APN_TYPE_SUPL
   *  - @ref LTE_APN_TYPE_DUN
   *  - @ref LTE_APN_TYPE_HIPRI
   *  - @ref LTE_APN_TYPE_FOTA
   *  - @ref LTE_APN_TYPE_IMS
   *  - @ref LTE_APN_TYPE_CBS
   *  - @ref LTE_APN_TYPE_IA
   *  - @ref LTE_APN_TYPE_EMERGENCY */

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
 *
 * Definition of ip address used in lte_pdn_t.
 *
 * @typedef lte_ipaddr_t
 * See @ref lte_ipaddr
 */

typedef struct lte_ipaddr
{
  /** Type of IP address. Definition is as below.
   *  - @ref LTE_IPTYPE_V4
   *  - @ref LTE_IPTYPE_V6 */

  uint8_t ip_type;

  /** IP address. It is terminated with '\0'.
   *  eg. (IPv4) 192.0.2.1, (IPv6) 2001:db8:85a3:0:0:8a2e:370:7334 */

  int8_t  address[LTE_IPADDR_MAX_LEN];
} lte_ipaddr_t;

/**
 * @struct lte_pdn
 *
 * Definition of pdn information used in activate_pdn_cb_t.
 *
 * @typedef lte_pdn_t
 * See @ref lte_pdn
 */

typedef struct lte_pdn
{
  /** PDN session id. The range is from
   *  @ref LTE_PDN_SESSIONID_MIN to @ref LTE_PDN_SESSIONID_MAX. */

  uint8_t      session_id;

  /** PDN active status. Definition is as below.
   *  - @ref LTE_PDN_ACTIVE
   *  - @ref LTE_PDN_DEACTIVE */

  uint8_t      active;

  /** APN type of PDN. Bit setting definition is as below.
   *  - @ref LTE_APN_TYPE_UNKNOWN
   *  - @ref LTE_APN_TYPE_DEFAULT
   *  - @ref LTE_APN_TYPE_MMS
   *  - @ref LTE_APN_TYPE_SUPL
   *  - @ref LTE_APN_TYPE_DUN
   *  - @ref LTE_APN_TYPE_HIPRI
   *  - @ref LTE_APN_TYPE_FOTA
   *  - @ref LTE_APN_TYPE_IMS
   *  - @ref LTE_APN_TYPE_CBS
   *  - @ref LTE_APN_TYPE_IA
   *  - @ref LTE_APN_TYPE_EMERGENCY */

  uint32_t     apn_type;

  /** Number of valid ip addresses */

  uint8_t      ipaddr_num;

  /** IP address information. See @ref lte_ipaddr_t */

  lte_ipaddr_t address[LTE_PDN_IPADDR_MAX_COUNT];

  /** IMS registored status.
   *  This is valid when LTE_APN_TYPE_IMS is set in apn_type.
   *  Definition is as below.
   *  - @ref LTE_IMS_NOT_REGISTERED
   *  - @ref LTE_IMS_REGISTERED */

  uint8_t      ims_register;

  /** Status of data communication enability. Definition is as below.
   *  - @ref LTE_DATA_ALLOW
   *  - @ref LTE_DATA_DISALLOW */

  uint8_t      data_allow;

  /** Status of roaming data communication enability.
   *  Definition is as below.
   *  - @ref LTE_DATA_ALLOW
   *  - @ref LTE_DATA_DISALLOW */

  uint8_t      data_roaming_allow;
} lte_pdn_t;

/**
 * @struct lte_reject_cause
 *
 * Definition of LTE network reject cause used in lte_nw_err_info_t.
 *
 * @typedef lte_reject_cause_t
 * See @ref lte_reject_cause
 */

typedef struct lte_reject_cause
{

  /**
   * Category of reject cause. Definition is as below..
   *  - @ref LTE_REJECT_CATEGORY_EMM
   *  - @ref LTE_REJECT_CATEGORY_ESM
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
 *
 * Definition of LTE network error information used in lte_netinfo_t.
 *
 * @typedef lte_nw_err_info_t
 * See @ref lte_nw_err_info
 */

typedef struct lte_nw_err_info
{
  /**
   * Type of LTE network error. Definition is as below.
   *  - @ref LTE_NETERR_MAXRETRY
   *  - @ref LTE_NETERR_REJECT
   *  - @ref LTE_NETERR_NWDTCH
   */

  uint8_t            err_type;

  /**
   * LTE network attach request reject cause. It can be referneced when
   *  - @ref LTE_NETERR_REJECT is ser in err_type field
   *  See @ref lte_reject_cause_t
   */

  lte_reject_cause_t reject_cause;
} lte_nw_err_info_t;

/**
 * @struct lte_netinfo
 *
 * Definition of lte network information used in get_netinfo_cb_t.
 *
 * @typedef lte_netinfo_t
 * See @ref lte_netinfo
 */

typedef struct lte_netinfo
{
  /** LTE network status. Definition is as below.
   *  - @ref LTE_NETSTAT_NOT_REG_NOT_SEARCHING
   *  - @ref LTE_NETSTAT_REG_HOME
   *  - @ref LTE_NETSTAT_NOT_REG_SEARCHING
   *  - @ref LTE_NETSTAT_REG_DENIED
   *  - @ref LTE_NETSTAT_UNKNOWN
   *  - @ref LTE_NETSTAT_REG_ROAMING
   *  - @ref LTE_NETSTAT_REG_SMS_ONLY_HOME
   *  - @ref LTE_NETSTAT_REG_SMS_ONLY_ROAMING
   *  - @ref LTE_NETSTAT_NOT_REG_EMERGENCY
   *  - @ref LTE_NETSTAT_REG_CSFB_NOT_PREF_HOME
   *  - @ref LTE_NETSTAT_REG_CSFB_NOT_PREF_ROAMING */

  uint8_t           nw_stat;

  /**
   * LTE network error information. It can be referneced when
   *  - @ref LTE_NETSTAT_REG_DENIED is set in nw_stat field.
   *  See @ref lte_nw_err_info_t
   */

  lte_nw_err_info_t nw_err;

  /** Number of PDN status informations.
   *  The maximum number of PDNs is @ref LTE_SESSION_ID_MAX. */

  uint8_t           pdn_num;

  /** List of PDN status. See @ref lte_pdn_t
   *
   *  @attention When using the lte_getnetinfo, 
   *             the maximum number of PDNs status areas must be allocated.
   */

  lte_pdn_t         *pdn_stat;
} lte_netinfo_t;

/**
 * @struct lte_error_info
 *
 * Definition of error information used in lte_get_errinfo().
 *
 * @typedef lte_errinfo_t
 * See @ref lte_error_info
 */

typedef struct lte_error_info
{
  /** Enable error indicator. Bit setting definition is as below.
   *  - @ref LTE_ERR_INDICATOR_ERRCODE
   *  - @ref LTE_ERR_INDICATOR_ERRNO
   *  - @ref LTE_ERR_INDICATOR_ERRSTR */

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
 *
 * Definition of CE settings used in lte_set_ce().
 * This is notified by get_ce_cb_t
 *
 * @typedef lte_ce_setting_t
 * See @ref lte_ce_setting
 */

typedef struct lte_ce_setting
{
  /** Mode A enable. Definition is as below.
   *  - @ref LTE_ENABLE
   *  - @ref LTE_DISABLE */

  bool mode_a_enable;

  /** Mode B enable. Definition is as below.
   *  - @ref LTE_ENABLE
   *  - @ref LTE_DISABLE */

  bool mode_b_enable;
} lte_ce_setting_t;

/**
 * @struct lte_siminfo
 *
 * Definition of parameters for SIM information.
 * This is notified by get_siminfo_cb_t
 *
 * @typedef lte_siminfo_t
 * See @ref lte_siminfo
 */

typedef struct lte_siminfo
{
  /** Indicates which parameter to get.
   *  Bit setting definition is as below.
   *  - @ref LTE_SIMINFO_GETOPT_MCCMNC
   *  - @ref LTE_SIMINFO_GETOPT_SPN
   *  - @ref LTE_SIMINFO_GETOPT_ICCID
   *  - @ref LTE_SIMINFO_GETOPT_IMSI
   *  - @ref LTE_SIMINFO_GETOPT_GID1
   *  - @ref LTE_SIMINFO_GETOPT_GID2
   */

  uint32_t option;

  /** Mobile Country Code (000-999). It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_MCCMNC is set in option field. */

  uint8_t  mcc[LTE_MCC_DIGIT];

  /** Digit number of Mobile Network Code(2-3). It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_MCCMNC is set in option field. */

  uint8_t  mnc_digit;

  /** Mobile Network Code (00-999). It can be referneced when
   *  - @ref LTE_SIMINFO_GETOPT_MCCMNC is set in option field. */

  uint8_t  mnc[LTE_MNC_DIGIT_MAX];

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
   *  - @ref LTE_SIMINFO_GETOPT_ICCID is set in option field.
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
 *
 *  Since lte_get_version() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_version().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] version : The version information of the modem.
 *                      See @ref lte_version_t
 */

typedef void (*get_ver_cb_t)(uint32_t result, lte_version_t *version);

/** Definition of callback function.
 *
 *  Since lte_get_phoneno() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_phoneno().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] errcause : Error cause. It is set only if the result is
 *                       not successful. As below value stored.
 * - @ref LTE_ERR_WAITENTERPIN
 * - @ref LTE_ERR_UNEXPECTED
 *
 * @param[in] phoneno : A character string indicating phone number.
 *                      It is terminated with '\0'
 */

typedef void (*get_phoneno_cb_t)(uint32_t result, uint8_t errcause,
                                 int8_t *phoneno);

/** Definition of callback function.
 *
 *  Since lte_get_imsi() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_imsi().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] errcause : Error cause. It is set only if the result is
 *                       not successful. As below value stored.
 * - @ref LTE_ERR_WAITENTERPIN
 * - @ref LTE_ERR_UNEXPECTED
 * @param[in] imsi : A character string indicating IMSI.
 *                   It is terminated with '\0'
 */

typedef void (*get_imsi_cb_t)(uint32_t result, uint8_t errcause,
                              int8_t *imsi);

/** Definition of callback function.
 *
 *  Since lte_get_imei() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_imei().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] imei : A character string indicating IMEI.
 *                   It is terminated with '\0'
 */

typedef void (*get_imei_cb_t)(uint32_t result, int8_t *imei);

/** Definition of callback function.
 *
 *  Since lte_get_pinset() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_pinset().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] pinset : PIN settings information.
 *                     See @ref lte_getpin_t
 */

typedef void (*get_pinset_cb_t)(uint32_t result, lte_getpin_t *pinset);

/** Definition of callback function.
 *
 *  Since lte_set_pinenable() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_set_pinenable().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] attemptsleft : Number of attempts left.
 *                           It is set only if the result is not successful.
 */

typedef void (*set_pinenable_cb_t)(uint32_t result, uint8_t attemptsleft);

/** Definition of callback function.
 *
 *  Since lte_change_pin() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_change_pin().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] attemptsleft : Number of attempts left.
 *                           It is set only if the result is not successful.
 */

typedef void (*change_pin_cb_t)(uint32_t result, uint8_t attemptsleft);

/** Definition of callback function.
 *
 *  Since lte_enter_pin() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_enter_pin().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] simstat : State after PIN enter.
 *                      As below value stored.
 * - @ref LTE_PINSTAT_READY
 * - @ref LTE_PINSTAT_SIM_PIN
 * - @ref LTE_PINSTAT_SIM_PUK
 * - @ref LTE_PINSTAT_PH_SIM_PIN
 * - @ref LTE_PINSTAT_PH_FSIM_PIN
 * - @ref LTE_PINSTAT_PH_FSIM_PUK
 * - @ref LTE_PINSTAT_SIM_PIN2
 * - @ref LTE_PINSTAT_SIM_PUK2
 * - @ref LTE_PINSTAT_PH_NET_PIN
 * - @ref LTE_PINSTAT_PH_NET_PUK
 * - @ref LTE_PINSTAT_PH_NETSUB_PIN
 * - @ref LTE_PINSTAT_PH_NETSUB_PUK
 * - @ref LTE_PINSTAT_PH_SP_PIN
 * - @ref LTE_PINSTAT_PH_SP_PUK
 * - @ref LTE_PINSTAT_PH_CORP_PIN
 * - @ref LTE_PINSTAT_PH_CORP_PUK
 *
 * @param[in] attemptsleft : Number of attempts left.
 *                           It is set only if the result is not successful.
 *                           If simstat is other than PIN, PUK, PIN2, PUK2,
 *                           set the number of PIN.
 */

typedef void (*enter_pin_cb_t)(uint32_t result,
                               uint8_t simstat,
                               uint8_t attemptsleft);

/** Definition of callback function.
 *
 *  Since lte_get_localtime() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_localtime().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] localtime : Local time. See @ref lte_localtime_t
 */

typedef void (*get_localtime_cb_t)(uint32_t result,
                                   lte_localtime_t *localtime);

/** Definition of callback function.
 *
 *  Since lte_get_operator() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_operator().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] oper : A character string indicating network operator.
 *                   It is terminated with '\0' If it is not connected,
 *                   the first character is '\0'.
 */

typedef void (*get_operator_cb_t)(uint32_t result, int8_t *oper);

/** Definition of callback function.
 *
 *  Since lte_get_edrx() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_edrx().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] settings : eDRX settings. See @ref lte_edrx_setting_t
 */

typedef void (*get_edrx_cb_t)(uint32_t result, lte_edrx_setting_t *settings);

/** Definition of callback function.
 *
 *  Since lte_set_edrx() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_set_edrx().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 */

typedef void (*set_edrx_cb_t)(uint32_t result);

/** Definition of callback function.
 *
 *  Since lte_get_psm() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_psm().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] settings : PSM settings. See @ref lte_psm_setting_t
 */

typedef void (*get_psm_cb_t)(uint32_t result, lte_psm_setting_t *settings);

/** Definition of callback function.
 *
 *  Since lte_set_psm() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_set_psm().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 */

typedef void (*set_psm_cb_t)(uint32_t result);

/** Definition of callback function.
 *
 *  Since lte_get_ce() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_ce().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] settings : CE settings. See @ref lte_ce_setting_t
 */

typedef void (*get_ce_cb_t)(uint32_t result, lte_ce_setting_t *settings);

/** Definition of callback function.
 *
 *  Since lte_set_ce() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_set_ce().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 */

typedef void (*set_ce_cb_t)(uint32_t result);

/** Definition of callback function.
 *
 *  When the SIM state changes, the SIM state is
 *  reported by this function.
 *
 * @param[in] simstat : The SIM state.
 *                      As below value stored.
 * - @ref LTE_SIMSTAT_REMOVAL
 * - @ref LTE_SIMSTAT_INSERTION
 * - @ref LTE_SIMSTAT_WAIT_PIN_UNLOCK
 * - @ref LTE_SIMSTAT_PERSONAL_FAILED
 * - @ref LTE_SIMSTAT_ACTIVATE
 * - @ref LTE_SIMSTAT_DEACTIVATE
 */

typedef void (*simstat_report_cb_t)(uint32_t simstat);

/** Definition of callback function.
 *
 *  When the local time changes, the local time is
 *  reported by this function.
 *
 * @param[in] localtime : Local time. See @ref lte_localtime_t
 */

typedef void (*localtime_report_cb_t)(lte_localtime_t *localtime);

/** Definition of callback function.
 *
 *  The quality information is reported by this function. It is reported
 *  at intervals of the set report period.
 *
 * @param[in] quality : Quality information. See @ref lte_quality_t
 */

typedef void (*quality_report_cb_t)(lte_quality_t *quality);

/** Definition of callback function.
 *
 *  The cell information is reported by this function. It is reported
 *  at intervals of the set report period.
 *
 * @param[in] cellinfo : Cell information. See @ref lte_cellinfo_t
 */

typedef void (*cellinfo_report_cb_t)(lte_cellinfo_t *cellinfo);

/** Definition of callback function.
 *
 *  Since lte_radio_on() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_radio_on().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 */

typedef void (*radio_on_cb_t)(uint32_t result);

/** Definition of callback function.
 *
 *  Since lte_radio_off() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_radio_off().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 */

typedef void (*radio_off_cb_t)(uint32_t result);

/** Definition of callback function.
 *
 *  Since lte_get_netinfo() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_netinfo().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] info : Pointer of LTE network information.
 *                   See @ref lte_netinfo_t
 */

typedef void (*get_netinfo_cb_t)(uint32_t result, lte_netinfo_t *info);

/** Definition of callback function.
 *
 *  Since lte_get_imscap() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_imscap.
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] imscap : The IMS capability.
 *                     As below value stored.
 * - @ref LTE_ENABLE
 * - @ref LTE_DISABLE
 */

typedef void (*get_imscap_cb_t)(uint32_t result, bool imscap);

/** Definition of callback function.
 *
 *  Since lte_activate_pdn() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_activate_pdn.
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 * - @ref LTE_RESULT_CANCEL
 *
 * @param[in] pdn : The connect pdn information. See @ref lte_pdn_t
 */

typedef void (*activate_pdn_cb_t)(uint32_t result, lte_pdn_t *pdn);

/** Definition of callback function.
 *
 *  Since lte_deactivate_pdn() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_deactivate_pdn.
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 */

typedef void (*deactivate_pdn_cb_t)(uint32_t result);

/** Definition of callback function.
 *
 *  Since lte_dataallow() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_dataallow.
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 */

typedef void (*data_allow_cb_t)(uint32_t result);

/** Definition of callback function.
 *
 *  The modem restart is reported by this function. It is reported
 *  at modem reset.
 *
 * @param[in] reason : Reason of modem restart.
 *                     As below value stored.
 * - @ref LTE_RESTART_USER_INITIATED
 * - @ref LTE_RESTART_MODEM_INITIATED
 */

typedef void (*restart_report_cb_t)(uint32_t reason);

/** Definition of callback function.
 *
 *  The change LTE network information is reported by this function.
 *  It is reported at LTE network connection status.
 *
 * @param[in] info : Pointer of LTE network information.
 *                   See @ref lte_netinfo_t
 */

typedef void (*netinfo_report_cb_t)(lte_netinfo_t *info);

/** Definition of callback function.
 *
 *  Since lte_get_siminfo() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_siminfo().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] siminfo : SIM information. See @ref lte_siminfo_t
 */

typedef void (*get_siminfo_cb_t)(uint32_t result, lte_siminfo_t *siminfo);

/** Definition of callback function.
 *
 *  Since lte_get_dynamic_edrx_param() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_dynamic_edrx_param().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] param : eDRX dynamic parameter. See @ref lte_edrx_setting_t.
 */

typedef void (*get_dynamic_edrx_param_cb_t)(uint32_t result,
                                            lte_edrx_setting_t *param);

/** Definition of callback function.
 *
 *  Since lte_get_dynamic_psm_param() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_dynamic_psm_param().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] param : PSM dynamic parameter. See @ref lte_psm_setting_t
 */

typedef void (*get_dynamic_psm_param_cb_t)(uint32_t result,
                                           lte_psm_setting_t *param);

/** Definition of callback function.
 *
 *  Since lte_get_current_edrx() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_current_edrx().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] settings : Current eDRX settings. See @ref lte_edrx_setting_t.
 */

typedef void (*get_current_edrx_cb_t)(uint32_t result,
                                      lte_edrx_setting_t *settings);

/** Definition of callback function.
 *
 *  Since lte_get_current_psm() is an asynchronous API,
 *  the result is notified by this function.
 *
 * @param[in] result : The result of lte_get_current_psm().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
 * @param[in] settings : Current PSM settings. See @ref lte_psm_setting_t
 */

typedef void (*get_current_psm_cb_t)(uint32_t result,
                                     lte_psm_setting_t *settings);

/** Definition of callback function.
 *
 *  Since lte_get_quality() is an asynchronous API,
 *  the quality information is notified by this function.
 *
 * @param[in] result : The result of lte_get_quality().
 *                     As below value stored.
 * - @ref LTE_RESULT_OK
 * - @ref LTE_RESULT_ERROR
 *
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
 * Initialize resources used in LTE API.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_initialize(void);

/**
 * Release resources used in LTE API.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_finalize(void);

/**
 * Register the callback to notify that the modem has started up.
 *
 * The callback will be invoked if the modem starts successfully
 * after calling lte_power_on. Some APIs have to wait until
 * this callback is invoked. If no wait, those API return
 * with an error. (-ENETDOWN)
 *
 * The callback is also invoked when the modem is restarted.
 * The cause of the restart can be obtained from the callback argument.
 *
 * This function must be called after lte_initialize.
 *
 * @attention Attention to the following 
 *   when @ref LTE_RESTART_MODEM_INITIATED is set.
 * - Asynchronous API callbacks for which results have not been 
 *   notified are canceled and becomes available.
 *
 * - The processing result of the synchronous API 
 *   being called results in an error. (Return value is -ENETDOWN)
 *   The errno is ENETDOWN for the socket API.
 *
 * - It should close the socket by user application.
 *
 * @param [in] restart_callback: Callback function to notify that
 *                               modem restarted.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_report_restart(restart_report_cb_t restart_callback);

/**
 * Power on the modem.
 *
 * The callback which registered by lte_set_report_restart
 * will be invoked if the modem starts successfully.
 *
 * This function must be called after lte_set_report_restart.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_power_on(void);

/**
 * Power off the modem
 *
 * @attention Attention to the following when this API calling.
 * - For asynchronous API
 *   - callback is canceled.
 *
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
 * With the radio on, to start the LTE network search.
 *
 * @attention Attention to the following when this API calling.
 * - If SIM is PIN locked, the result will be an error.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_radio_on_sync(void);

/**
 * With the radio on, to start the LTE network search.
 *
 * @attention Attention to the following when this API calling.
 * - If SIM is PIN locked, the result will be an error.
 *
 * @param [in] callback: Callback function to notify that
 *                       radio on is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_radio_on(radio_on_cb_t callback);

/**
 * Exit LTE network searches with the radio off.
 *
 * If this function is called when a PDN has already been constructed, 
 * the PDN is discarded.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_radio_off_sync(void);

/**
 * Exit LTE network searches with the radio off.
 *
 * If this function is called when a PDN has already been constructed, 
 * the PDN is discarded.
 *
 * @param [in] callback: Callback function to notify that
 *                       radio off is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_radio_off(radio_off_cb_t callback);

/**
 * Get LTE network information.
 *
 * @attention The maximum number of PDNs status areas must be allocated
 *            before calls this API.
 *
 * @param [in] pdn_num: Number of pdn_stat allocated by the user.
 *                      The range is from @ref LTE_PDN_SESSIONID_MIN to
 *                      @ref LTE_PDN_SESSIONID_MAX.
 *
 * @param [out] info: The LTE network information.
 *                    See @ref lte_netinfo_t
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_netinfo_sync(uint8_t pdn_num, lte_netinfo_t *info);

/**
 * Get LTE network information.
 *
 * @param [in] callback: Callback function to notify that
 *                       get network information completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_netinfo(get_netinfo_cb_t callback);

/**
 * Constructs a PDN with the specified APN settings.
 *
 * When constructs the initial PDN, 
 * LTE_APN_TYPE_IA must be set to the APN type. 
 *
 * When PDN construction is successful, 
 * an IP address is given from the LTE network.
 *
 * @attention Attention to the following when this API calling.
 * - The initial PDN construction may take a few minutes 
 *   depending on radio conditions.
 *
 * - If API is not returned, please check if the APN settings are correct.
 *
 * @param [in] apn: The pointer of the apn setting.
 *                  See @ref lte_apn_setting_t for valid parameters.
 *
 * @param [out] pdn: The construction PDN information. 
 *                   See @ref lte_pdn_t.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 * If canceling, -ECANCELED is returned.
 */

int32_t lte_activate_pdn_sync(lte_apn_setting_t *apn, lte_pdn_t *pdn);

/**
 * Constructs a PDN with the specified APN settings.
 *
 * When constructs the initial PDN, 
 * LTE_APN_TYPE_IA must be set to the APN type. 
 *
 * When PDN construction is successful, 
 * an IP address is given from the LTE network.
 *
 * @attention Attention to the following when this API calling.
 * - The initial PDN construction may take a few minutes 
 *   depending on radio conditions.
 *
 * - If the callback is not notified, please check 
 *   if the APN settings are correct.
 *
 * @param [in] apn: The pointer of the apn setting.
 *                  See @ref lte_apn_setting_t for valid parameters.
 *
 * @param [in] callback: Callback function to notify that
 *                       PDN activation completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_activate_pdn(lte_apn_setting_t *apn, activate_pdn_cb_t callback);

/**
 * Cancel PDN construction.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_activate_pdn_cancel(void);

/**
 * Discard the constructed PDN.
 *
 * Discards the PDN corresponding to the session ID 
 * obtained by lte_activate_pdn.
 *
 * When the discard process is successful, the IP address assigned to 
 * the modem is released to the LTE network.
 *
 * @param [in] session_id: The numeric value of the session ID.
 *                         Use the value obtained by the lte_activate_pdn.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_deactivate_pdn_sync(uint8_t session_id);

/**
 * Discard the constructed PDN.
 *
 * Discards the PDN corresponding to the session ID 
 * obtained by lte_activate_pdn.
 *
 * When the discard process is successful, the IP address assigned to 
 * the modem is released to the LTE network.
 *
 * @param [in] session_id: The numeric value of the session ID.
 *                         Use the value obtained by the lte_activate_pdn.
 *
 * @param [in] callback: Callback function to notify that
 *                       LTE PDN deactivation completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_deactivate_pdn(uint8_t session_id, deactivate_pdn_cb_t callback);

/**
 * Allow or disallow to data communication for specified PDN.
 *
 * If the application performs data communication in the disallow state,
 * the modem discards the data.
 *
 * @param [in] session_id: The numeric value of the session ID.
 *                         Use the value obtained by the lte_activate_pdn.
 *
 * @param [in] allow: Allow or disallow to data communication for
 *                    all network. Definition is as below.
 *  - @ref LTE_DATA_ALLOW
 *  - @ref LTE_DATA_DISALLOW
 *
 * @param [in] roaming_allow: Allow or disallow to data communication for
 *                            roaming network. Definition is as below.
 *  - @ref LTE_DATA_ALLOW
 *  - @ref LTE_DATA_DISALLOW
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_data_allow_sync(uint8_t session_id, uint8_t allow,
                            uint8_t roaming_allow);

/**
 * Allow or disallow to data communication for specified PDN.
 *
 * If the application performs data communication in the disallow state,
 * the modem discards the data.
 *
 * @param [in] session_id: The numeric value of the session ID.
 *                         Use the value obtained by the lte_activate_pdn.
 *
 * @param [in] allow: Allow or disallow to data communication for
 *                    all network. Definition is as below.
 *  - @ref LTE_DATA_ALLOW
 *  - @ref LTE_DATA_DISALLOW
 *
 * @param [in] roaming_allow: Allow or disallow to data communication for
 *                            roaming network. Definition is as below.
 *  - @ref LTE_DATA_ALLOW
 *  - @ref LTE_DATA_DISALLOW
 *
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
 * @param [out] imscap: The IMS capability.
 *                      As below value stored.
 *  - @ref LTE_ENABLE
 *  - @ref LTE_DISABLE
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_imscap_sync(bool *imscap);

/**
 * Get whether the modem supports IMS or not.
 *
 * @param [in] callback: Callback function to notify when
 *                       getting IMS capability is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_imscap(get_imscap_cb_t callback);

/**
 * Acquires the FW version information of the modem.
 *
 * @param [out] version: The version information of the modem.
 *                        See @ref lte_version_t
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_version_sync(lte_version_t *version);

/**
 * Acquires the FW version information of the modem.
 *
 * @param [in] callback: Callback function to notify when
 *                       getting the version is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_version(get_ver_cb_t callback);

/**
 * Get phone number from SIM.
 *
 * @param [out] phoneno: A character string indicating phone number.
 *                       It is terminated with '\0'.
 *                       The maximum number of phone number areas 
 *                       must be allocated. See @ref LTE_PHONENO_LEN.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_phoneno_sync(int8_t *phoneno);

/**
 * Get phone number from SIM.
 *
 * @param [in] callback: Callback function to notify when
 *                       getting the phone number is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_phoneno(get_phoneno_cb_t callback);

/**
 * Get International Mobile Subscriber Identity from SIM.
 *
 * @param [out] imsi: A character string indicating IMSI.
 *                    It is terminated with '\0'.
 *                    The maximum number of IMSI areas 
 *                    must be allocated. See @ref LTE_SIMINFO_IMSI_LEN.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_imsi_sync(int8_t *imsi);

/**
 * Get International Mobile Subscriber Identity from SIM.
 *
 * @param [in] callback: Callback function to notify when
 *                       getting IMSI is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_imsi(get_imsi_cb_t callback);

/**
 * Get International Mobile Equipment Identifier from the modem.
 *
 * @param [out] imei: A character string indicating IMEI.
 *                    It is terminated with '\0'.
 *                    The maximum number of IMEI areas 
 *                    must be allocated. See @ref LTE_IMEI_LEN.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_imei_sync(int8_t *imei);

/**
 * Get International Mobile Equipment Identifier from the modem.
 *
 * @param [in] callback: Callback function to notify when
 *                       getting IMEI is completed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_imei(get_imei_cb_t callback);

/**
 * Get Personal Identification Number settings.
 *
 * @param [out] pinset: PIN settings information.
 *                      See @ref lte_getpin_t.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_pinset_sync(lte_getpin_t *pinset);

/**
 * Get Personal Identification Number settings.
 *
 * @param [in] callback: Callback function to notify when
 *                       getting the PIN setting is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_pinset(get_pinset_cb_t callback);

/**
 * Set Personal Identification Number enable.
 *
 * @param [in] enable: "Enable" or "Disable".
 *                      Definition is as below.
 *  - @ref LTE_ENABLE
 *  - @ref LTE_DISABLE
 *
 * @param [in] pincode: Current PIN code. Minimum number of digits is 4.
 *                      Maximum number of digits is 8, end with '\0'.
 *                      (i.e. Max 9 byte)
 *
 * @param [out] attemptsleft: Number of attempts left.
 *                            Set only if failed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_pinenable_sync(bool enable, int8_t *pincode,
                               uint8_t *attemptsleft);

/**
 * Set Personal Identification Number enable.
 *
 * @param [in] enable: "Enable" or "Disable".
 *                      Definition is as below.
 *  - @ref LTE_ENABLE
 *  - @ref LTE_DISABLE
 *
 * @param [in] pincode: Current PIN code. Minimum number of digits is 4.
 *                      Maximum number of digits is 8, end with '\0'.
 *                      (i.e. Max 9 byte)
 *
 * @param [in] callback: Callback function to notify that
 *                       setting of PIN enables/disables is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_pinenable(bool enable, int8_t *pincode,
                          set_pinenable_cb_t callback);

/**
 * Change Personal Identification Number.
 *
 * It can be changed only when PIN is enable.
 *
 * @param [in] target_pin: Target of change PIN.
 *                      Definition is as below.
 *  - @ref LTE_TARGET_PIN
 *  - @ref LTE_TARGET_PIN2
 *
 * @param [in] pincode: Current PIN code. Minimum number of digits is 4.
 *                      Maximum number of digits is 8, end with '\0'.
 *                      (i.e. Max 9 byte)
 *
 * @param [in] new_pincode: New PIN code. Minimum number of digits is 4.
 *                          Maximum number of digits is 8, end with '\0'.
 *                          (i.e. Max 9 byte)
 *
 * @param [out] attemptsleft: Number of attempts left.
 *                            Set only if failed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_change_pin_sync(int8_t target_pin, int8_t *pincode,
                            int8_t *new_pincode, uint8_t *attemptsleft);

/**
 * Change Personal Identification Number.
 *
 * It can be changed only when PIN is enable.
 *
 * @param [in] target_pin: Target of change PIN.
 *                      Definition is as below.
 *  - @ref LTE_TARGET_PIN
 *  - @ref LTE_TARGET_PIN2
 *
 * @param [in] pincode: Current PIN code. Minimum number of digits is 4.
 *                      Maximum number of digits is 8, end with '\0'.
 *                      (i.e. Max 9 byte)
 *
 * @param [in] new_pincode: New PIN code. Minimum number of digits is 4.
 *                          Maximum number of digits is 8, end with '\0'.
 *                          (i.e. Max 9 byte)
 *
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
 *
 * @param [in] new_pincode: If not used, set NULL.
 *                          If the PIN is SIM PUK or SIM PUK2,
 *                          the new_pincode is required.
 *                          Minimum number of digits is 4.
 *                          Maximum number of digits is 8,
 *                          end with '\0'. (i.e. Max 9 byte)
 *
 * @param [out] simstat: State after PIN enter.
 *                       As below value stored.
 * - @ref LTE_PINSTAT_READY
 * - @ref LTE_PINSTAT_SIM_PIN
 * - @ref LTE_PINSTAT_SIM_PUK
 * - @ref LTE_PINSTAT_PH_SIM_PIN
 * - @ref LTE_PINSTAT_PH_FSIM_PIN
 * - @ref LTE_PINSTAT_PH_FSIM_PUK
 * - @ref LTE_PINSTAT_SIM_PIN2
 * - @ref LTE_PINSTAT_SIM_PUK2
 * - @ref LTE_PINSTAT_PH_NET_PIN
 * - @ref LTE_PINSTAT_PH_NET_PUK
 * - @ref LTE_PINSTAT_PH_NETSUB_PIN
 * - @ref LTE_PINSTAT_PH_NETSUB_PUK
 * - @ref LTE_PINSTAT_PH_SP_PIN
 * - @ref LTE_PINSTAT_PH_SP_PUK
 * - @ref LTE_PINSTAT_PH_CORP_PIN
 * - @ref LTE_PINSTAT_PH_CORP_PUK
 *
 * @param [out] attemptsleft: Number of attempts left.
 *                            Set only if failed.
 *                            If simstat is other than PIN, PUK, PIN2, PUK2,
 *                            set the number of PIN.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_enter_pin_sync(int8_t *pincode, int8_t *new_pincode,
                           uint8_t *simstat, uint8_t *attemptsleft);

/**
 * Enter Personal Identification Number.
 *
 * @param [in] pincode: Current PIN code. Minimum number of digits is 4.
 *                      Maximum number of digits is 8, end with '\0'.
 *                      (i.e. Max 9 byte)
 *
 * @param [in] new_pincode: If not used, set NULL.
 *                          If the PIN is SIM PUK or SIM PUK2,
 *                          the new_pincode is required.
 *                          Minimum number of digits is 4.
 *                          Maximum number of digits is 8,
 *                          end with '\0'. (i.e. Max 9 byte)
 *
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
 * @param [out] localtime: Local time. See @ref lte_localtime_t.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_localtime_sync(lte_localtime_t *localtime);

/**
 * Get local time.
 *
 * @param [in] callback: Callback function to notify when
 *                       getting local time is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_localtime(get_localtime_cb_t callback);

/**
 * Get connected network operator information.
 *
 * @param [out] oper: A character string indicating network operator.
 *                    It is terminated with '\0' If it is not connected,
 *                    the first character is '\0'.
 *                    The maximum number of network operator areas 
 *                    must be allocated. See @ref LTE_OPERATOR_LEN.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_operator_sync(int8_t *oper);

/**
 * Get connected network operator information.
 *
 * @param [in] callback: Callback function to notify when 
 *                       getting network operator information is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_operator(get_operator_cb_t callback);

/**
 * Get eDRX settings.
 *
 * @param [out] settings: eDRX settings. See @ref lte_edrx_setting_t.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_edrx_sync(lte_edrx_setting_t *settings);

/**
 * Get eDRX settings.
 *
 * @param [in] callback: Callback function to notify when 
 *                       getting eDRX settings are completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_edrx(get_edrx_cb_t callback);

/**
 * Set eDRX settings.
 *
 * @param [in] settings: eDRX settings.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_edrx_sync(lte_edrx_setting_t *settings);

/**
 * Set eDRX settings.
 *
 * @param [in] settings: eDRX settings.
 *
 * @param [in] callback: Callback function to notify that 
 *                       eDRX settings are completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_edrx(lte_edrx_setting_t *settings,
                     set_edrx_cb_t callback);

/**
 * Get PSM settings.
 *
 * @param [out] settings: PSM settings. See @ref lte_psm_setting_t.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_psm_sync(lte_psm_setting_t *settings);

/**
 * Get PSM settings.
 *
 * @param [in] callback: Callback function to notify when 
 *                       getting PSM settings are completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_psm(get_psm_cb_t callback);

/**
 * Set PSM settings.
 *
 * @param [in] settings: PSM settings.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_psm_sync(lte_psm_setting_t *settings);

/**
 * Set PSM settings.
 *
 * @param [in] settings: PSM settings.
 *
 * @param [in] callback: Callback function to notify that 
 *                       PSM settings are completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_psm(lte_psm_setting_t *settings,
                    set_psm_cb_t callback);

/**
 * Get CE settings.
 *
 * @param [out] settings: CE settings. See @ref lte_ce_setting_t.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_ce_sync(lte_ce_setting_t *settings);

/**
 * Get CE settings.
 *
 * @param [in] callback: Callback function to notify when 
 *                       getting CE settings are completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_ce(get_ce_cb_t callback);

/**
 * Set CE settings.
 *
 * @param [in] settings: CE settings
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_ce_sync(lte_ce_setting_t *settings);

/**
 * Set CE settings.
 *
 * @param [in] settings: CE settings
 *
 * @param [in] callback: Callback function to notify that
 *                       CE settings are completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_ce(lte_ce_setting_t *settings,
                   set_ce_cb_t callback);

/**
 * Notifies the SIM status to the application.
 *
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
 * Notifies the Local time to the application.
 *
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
 * Notifies the communication quality information to the application.
 *
 * Invoke the callback at the specified report interval.
 *
 * The default report setting is disable.
 *
 * @attention When changing the notification cycle, stop and start again.
 *
 * @param [in] quality_callback: Callback function to notify that
 *                               quality information. If NULL is set,
 *                               the report setting is disabled.
 *
 * @param [in] period: Reporting cycle in sec (1-4233600)
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_report_quality(quality_report_cb_t quality_callback,
                               uint32_t period);

/**
 * Notifies the LTE network cell information to the application.
 *
 * Invoke the callback at the specified report interval.
 *
 * The default report setting is disable.
 *
 * @attention When changing the notification cycle, stop and start again.
 *
 * @param [in] cellinfo_callback: Callback function to notify that
 *                                cell information. If NULL is set,
 *                                the report setting is disabled.
 *
 * @param [in] period: Reporting cycle in sec (1-4233600)
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_set_report_cellinfo(cellinfo_report_cb_t cellinfo_callback,
                                uint32_t period);

/**
 * Notifies the LTE network status to the application.
 *
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
 * Get LTE API last error information.
 *
 * Call this function when LTE_RESULT_ERROR is returned by
 * callback function. The detailed error information can be obtained.
 *
 * @param [in] info: Pointer of error information.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_errinfo(lte_errinfo_t *info);

/**
 * Get SIM information such as Mobile Country Code/Mobile Network Code.
 *
 * @param [in] option:   Indicates which parameter to get.
 *                       Bit setting definition is as below.
 *                       - @ref LTE_SIMINFO_GETOPT_MCCMNC
 *                       - @ref LTE_SIMINFO_GETOPT_SPN
 *                       - @ref LTE_SIMINFO_GETOPT_ICCID
 *                       - @ref LTE_SIMINFO_GETOPT_IMSI
 *                       - @ref LTE_SIMINFO_GETOPT_GID1
 *                       - @ref LTE_SIMINFO_GETOPT_GID2
 *
 * @param [out] siminfo: SIM information. See @ref lte_siminfo_t.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_siminfo_sync(uint32_t option, lte_siminfo_t *siminfo);

/**
 * Get SIM information such as Mobile Country Code/Mobile Network Code.
 *
 * @param [in] option:   Indicates which parameter to get.
 *                       Bit setting definition is as below.
 *                       - @ref LTE_SIMINFO_GETOPT_MCCMNC
 *                       - @ref LTE_SIMINFO_GETOPT_SPN
 *                       - @ref LTE_SIMINFO_GETOPT_ICCID
 *                       - @ref LTE_SIMINFO_GETOPT_IMSI
 *                       - @ref LTE_SIMINFO_GETOPT_GID1
 *                       - @ref LTE_SIMINFO_GETOPT_GID2
 *
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
 * @deprecated Use @ref lte_get_current_edrx instead.
 *
 * This API can be issued after connect to the LTE network
 * with lte_activate_pdn().
 *
 * @param [in] callback: Callback function to notify when
 *                       getting eDRX dynamic parameter is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_dynamic_edrx_param(get_dynamic_edrx_param_cb_t callback);

/**
 * Get current eDRX settings.
 *
 * This API can be issued after connect to the LTE network
 * with lte_activate_pdn().
 *
 * Get the settings negotiated between the modem and the network.
 *
 * @param [out] settings: Current eDRX settings.
 *                        See @ref lte_edrx_setting_t.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_current_edrx_sync(lte_edrx_setting_t *settings);

/**
 * Get current eDRX settings.
 *
 * This API can be issued after connect to the LTE network
 * with lte_activate_pdn().
 *
 * Get the settings negotiated between the modem and the network.
 *
 * @param [in] callback: Callback function to notify when
 *                       getting current eDRX settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_current_edrx(get_current_edrx_cb_t callback);

/**
 * Get PSM dynamic parameter.
 *
 * @deprecated Use @ref lte_get_current_psm instead.
 *
 * This API can be issued after connect to the LTE network
 * with lte_activate_pdn().
 *
 * @param [in] callback: Callback function to notify when
 *                       getting PSM dynamic parameter is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_dynamic_psm_param(get_dynamic_psm_param_cb_t callback);

/**
 * Get current PSM settings.
 *
 * This API can be issued after connect to the LTE network
 * with lte_activate_pdn().
 *
 * Get the settings negotiated between the modem and the network.
 *
 * @param [OUT] settings: Current PSM settings.
 *                        See @ref lte_psm_setting_t.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_current_psm_sync(lte_psm_setting_t *settings);

/**
 * Get current PSM settings.
 *
 * This API can be issued after connect to the LTE network
 * with lte_activate_pdn().
 *
 * Get the settings negotiated between the modem and the network.
 *
 * @param [in] callback: Callback function to notify when
 *                       getting current PSM settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_current_psm(get_current_psm_cb_t callback);

/**
 * Get communication quality information.
 *
 * @param [out] quality: Quality information. See @ref lte_quality_t
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_quality_sync(lte_quality_t *quality);

/**
 * Get communication quality information.
 *
 * @param [in] callback: Callback function to notify when 
 *                       getting quality information is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_quality(get_quality_cb_t callback);

/**
 * Get LTE network cell information.
 *
 * @param [out] cellinfo: LTE network cell information.
 *                        See @ref lte_cellinfo_t
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned according to <errno.h>.
 */

int32_t lte_get_cellinfo_sync(lte_cellinfo_t *cellinfo);

/** @} */

/** @} */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MODULES_INCLUDE_LTE_LTE_API_H */
