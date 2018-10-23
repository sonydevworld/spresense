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

/**
 * @file lte_api.h
 */

#ifndef __MODULES_INCLUDE_LTE_LTE_API_H
#define __MODULES_INCLUDE_LTE_LTE_API_H

/**
 * @defgroup lte LTE API
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * @defgroup lteresult Result code
 * Definitions of result code. These are notified by asynchronous
 * API callback such as attach_net_cb_t
 * @{
 */

#define LTE_RESULT_OK     (0) /**< Result code on success */
#define LTE_RESULT_ERROR  (1) /**< Result code on failure */
#define LTE_RESULT_CANCEL (2) /**< Result code on cancel */

/** @} lteresult */

/**
 * @defgroup ltevalidflg Validate flag
 * Definitions of validate flag used in lte_quality_t and lte_cellinfo_t
 * @{
 */

#define LTE_VALID   (true)  /**< Valid */
#define LTE_INVALID (false) /**< Invalid */

/** @} ltevalidflg */

/**
 * @defgroup lteenableflg Enable flag
 * Definitions of enable flag used in lte_edrx_setting_t and lte_ce_setting_t
 * 
 * @{
 */

#define LTE_ENABLE  (true)  /**< Enable */
#define LTE_DISABLE (false) /**< Disable */

/** @} lteenableflg */

/**
 * @defgroup lteerrcause Error cause
 * Definitions of error cause. These are notified by asynchronous
 * API callback such as attach_net_cb_t
 * @{
 */

#define LTE_ERR_WAITENTERPIN (1)   /**< Waiting for PIN enter */
#define LTE_ERR_REJECT       (2)   /**< Rejected from the network */
#define LTE_ERR_MAXRETRY     (3)   /**< No response from the network */
#define LTE_ERR_BARRING      (4)   /**< Network barring */
#define LTE_ERR_DETACHED     (5)   /**< Network detached */
#define LTE_ERR_UNEXPECTED   (255) /**< Unexpected cause */

/** @} lteerrcause */

/**
 * @defgroup ltesessionid Session ID
 * Definitions of session ID value range.
 * These are used in lte_data_on(), lte_data_off() lte_set_apn().
 * @{
 */

#define LTE_SESSION_ID_MIN (1) /**< Minimum value of session ID */
#define LTE_SESSION_ID_MAX (5) /**< Maximum value of session ID */

/** @} ltesessionid */

/**
 * @defgroup ltepowerctl Power control
 * Definitions of power control used in lte_power_control().
 * @{
 */

#define LTE_POWERON  (true)  /**< Power on the modem */
#define LTE_POWEROFF (false) /**< Power off the modem */

/** @} ltepowerctl */

/**
 * @defgroup lteslpmode Sleep mode
 * Definitions of sleep mode used in lte_set_sleepmode().
 * Otherwise These are notified by get_slpmode_cb_t
 * @{
 */

#define LTE_SLPMODE_LIGHTSLEEP (0) /**< Sleep mode: light sleep */
#define LTE_SLPMODE_DEEPSLEEP  (1) /**< Sleep mode: deep sleep */
#define LTE_SLPMODE_LIGHTHIBER (2) /**< Sleep mode: light hibernation */

/** @} lteslpmode */

/**
 * @defgroup ltenetstat Network state
 * Definitions of network state. These are notified by netstat_report_cb_t
 * @{
 */

#define LTE_NETSTAT_ATTACH     (0) /**< LTE attached */
#define LTE_NETSTAT_DETACH     (1) /**< LTE detached */
#define LTE_NETSTAT_CONNECT    (2) /**< Data connected */
#define LTE_NETSTAT_DISCONNECT (3) /**< Data disconnected */
#define LTE_NETSTAT_CHANGE_APN (4) /**< Connected APN has changed */

/** @} ltenetstat */

/**
 * @defgroup ltedatatype Data type
 * Definitions of Data type used in lte_set_dataconfig().
 * Otherwise These are notified by get_dataconfig_cb_t
 * @{
 */

#define LTE_DATA_TYPE_USER (0) /**< Data type: user data */
#define LTE_DATA_TYPE_IMS  (1) /**< Data type: IMS */

/** @} ltedatatype */

/**
 * @defgroup lteapnlen Length of APN
 * Definition of maximum string length of the APN name
 * @{
 */

#define LTE_APN_LEN 101 /**< The maximum string length of the APN name */

/** @} lteapnlen */

/**
 * @defgroup lteapnusrnamelen Length of user name
 * Definition of maximum string length of the APN user name
 * @{
 */

#define LTE_APN_USER_NAME_LEN 64 /**< The maximum string length of
                                      the APN user name */

/** @} lteapnusrnamelen */

/**
 * @defgroup lteapnpasswdlen Length of password
 * Definition of maximum string length of the APN password
 * @{
 */

#define LTE_APN_PASSWD_LEN 32 /**< The maximum string length of
                                   the APN password */

/** @} lteapnpasswdlen */

/**
 * @defgroup lteapniptype APN IP type
 * Definitions of internet protocol type
 * @{
 */

#define LTE_APN_IPTYPE_IP     (0) /**< Internet protocol type IP */
#define LTE_APN_IPTYPE_IPV6   (1) /**< Internet protocol type IPv6 */
#define LTE_APN_IPTYPE_IPV4V6 (2) /**< Internet protocol type IPv4/v6 */

/** @} lteapniptype */

/**
 * @defgroup lteapnauthtype APN PPP authentication type
 * Definitions of PPP authentication type
 * @{
 */

#define LTE_APN_AUTHTYPE_NONE (0) /**< PPP authentication type NONE */
#define LTE_APN_AUTHTYPE_PAP  (1) /**< PPP authentication type PAP */
#define LTE_APN_AUTHTYPE_CHAP (2) /**< PPP authentication type CHAP */

/** @} lteapnauthtype */

/**
 * @defgroup lteverlen Length of version information
 * Definitions of length of character string for version
 * @{
 */

#define LTE_VER_BB_PRODUCT_LEN (5)  /**< Length of character string for
                                         BB product */
#define LTE_VER_NP_PACKAGE_LEN (32) /**< Length of character string for
                                         NP package */

/** @} lteverlen */

/**
 * @defgroup ltepinenable PIN enable
 * Definitions of pin enable. Used in lte_set_pinenable()
 * @{
 */

#define LTE_PIN_ENABLE  (true)  /**< Enable setting of PIN lock */
#define LTE_PIN_DISABLE (false) /**< Disable setting of PIN lock */

/** @} ltepinenable */

/**
 * @defgroup ltepinstat PIN status
 * Definitions of PIN status. These are used in lte_getpin_t
 * @{
 */

#define LTE_PINSTAT_READY         (0)  /**< Not pending for any password */
#define LTE_PINSTAT_SIM_PIN       (1)  /**< Waiting SIM PIN to be given */
#define LTE_PINSTAT_SIM_PUK       (2)  /**< Waiting SIM PUK to be given */
#define LTE_PINSTAT_PH_SIM_PIN    (3)  /**< Waiting phone to SIM card
                                            password to be given */
#define LTE_PINSTAT_PH_FSIM_PIN   (4)  /**< Waiting phone-to-very first SIM
                                            card password to be given */
#define LTE_PINSTAT_PH_FSIM_PUK   (5)  /**< Waiting phone-to-very first SIM
                                            card unblocking password
                                            to be given */
#define LTE_PINSTAT_SIM_PIN2      (6)  /**< Waiting SIM PIN2 to be given */
#define LTE_PINSTAT_SIM_PUK2      (7)  /**< Waiting SIM PUK2 to be given */
#define LTE_PINSTAT_PH_NET_PIN    (8)  /**< Waiting network personalization
                                            password to be given */
#define LTE_PINSTAT_PH_NET_PUK    (9)  /**< Waiting network personalization
                                            unblocking password to be given */
#define LTE_PINSTAT_PH_NETSUB_PIN (10) /**< Waiting network subset
                                            personalization password to be
                                            given */
#define LTE_PINSTAT_PH_NETSUB_PUK (11) /**< Waiting network subset
                                            personalization unblocking
                                            password to be given */
#define LTE_PINSTAT_PH_SP_PIN     (12) /**< Waiting service provider
                                            personalization password to be
                                            given */
#define LTE_PINSTAT_PH_SP_PUK     (13) /**< Waiting service provider
                                            personalization unblocking
                                            password to be given */
#define LTE_PINSTAT_PH_CORP_PIN   (14) /**< Waiting corporate personalization
                                            password to be given */
#define LTE_PINSTAT_PH_CORP_PUK   (15) /**< Waiting corporate personalization
                                            unblocking password to be given */

/** @}  ltepinstat */

/**
 * @defgroup ltetargetpin Target PIN
 * Definitions of target PIN. These are used in lte_change_pin()
 * @{
 */

#define LTE_TARGET_PIN  (0)  /**< Select of PIN change */
#define LTE_TARGET_PIN2 (1)  /**< Select of PIN2 change */

/** @}  ltetargetpin */

/**
 * @defgroup ltesimstate SIM state
 * Definitions of SIM state. These are notified by simstat_report_cb_t
 * @{
 */

#define LTE_SIMSTAT_REMOVAL         (0) /**< SIM removal signal detected */
#define LTE_SIMSTAT_INSERTION       (1) /**< SIM insertion signal detected */
#define LTE_SIMSTAT_WAIT_PIN_UNLOCK (2) /**< SIM init passed, wait for
                                             PIN unlock */
#define LTE_SIMSTAT_PERSONAL_FAILED (3) /**< Personalization failed, wait for
                                             run-time depersonalization */
#define LTE_SIMSTAT_ACTIVATE        (4) /**< Activation completed.
                                             Event is sent always at any SIM
                                             activation completion */

/** @} ltesimstate */

/**
 * @defgroup ltecellinfo Cell info
 * Definitions of Cell info. Used in lte_cellinfo_t.
 * @{
 */

#define LTE_CELLINFO_MCC_DIGIT     (3)  /**< Digit number of mcc */
#define LTE_CELLINFO_MNC_DIGIT_MAX (3)  /**< Max digit number of mnc */

/** @} ltecellinfo */

/**
 * @defgroup lteedrxtype eDRX act type
 * Definitions of eDRX act type used in lte_edrx_setting_t
 * @{
 */

#define LTE_EDRX_ACTTYPE_WBS1 (0)  /**< E-UTRAN (WB-S1 mode) */

/** @} lteedrxtype */

/**
 * @defgroup lteedrxcyc eDRX cycle
 * Definitions of eDRX cycle used in lte_edrx_setting_t
 * @{
 */

#define LTE_EDRX_CYC_512     (0)  /**< eDRX cycle     5.12 sec */
#define LTE_EDRX_CYC_1024    (1)  /**< eDRX cycle    10.24 sec */
#define LTE_EDRX_CYC_2048    (2)  /**< eDRX cycle    20.48 sec */
#define LTE_EDRX_CYC_4096    (3)  /**< eDRX cycle    40.96 sec */
#define LTE_EDRX_CYC_6144    (4)  /**< eDRX cycle    61.44 sec */
#define LTE_EDRX_CYC_8192    (5)  /**< eDRX cycle    81.92 sec */
#define LTE_EDRX_CYC_10240   (6)  /**< eDRX cycle   102.40 sec */
#define LTE_EDRX_CYC_12288   (7)  /**< eDRX cycle   122.88 sec */
#define LTE_EDRX_CYC_14336   (8)  /**< eDRX cycle   143.36 sec */
#define LTE_EDRX_CYC_16384   (9)  /**< eDRX cycle   163.84 sec */
#define LTE_EDRX_CYC_32768   (10) /**< eDRX cycle   327.68 sec */
#define LTE_EDRX_CYC_65536   (11) /**< eDRX cycle   655.36 sec */
#define LTE_EDRX_CYC_131072  (12) /**< eDRX cycle  1310.72 sec */
#define LTE_EDRX_CYC_262144  (13) /**< eDRX cycle  2621.44 sec */

/** @} lteedrxcyc */

/**
 * @defgroup lteedrxptw eDRX paging time window
 * Definitions of eDRX paging time window used in lte_edrx_setting_t
 * @{
 */

#define LTE_EDRX_PTW_128      (0) /**< Paging time window  1.28 sec */
#define LTE_EDRX_PTW_256      (1) /**< Paging time window  2.56 sec */
#define LTE_EDRX_PTW_384      (2) /**< Paging time window  3.84 sec */
#define LTE_EDRX_PTW_512      (3) /**< Paging time window  5.12 sec */
#define LTE_EDRX_PTW_640      (4) /**< Paging time window  6.40 sec */
#define LTE_EDRX_PTW_768      (5) /**< Paging time window  7.68 sec */
#define LTE_EDRX_PTW_896      (6) /**< Paging time window  8.96 sec */
#define LTE_EDRX_PTW_1024     (7) /**< Paging time window 10.24 sec */
#define LTE_EDRX_PTW_1152     (8) /**< Paging time window 11.52 sec */
#define LTE_EDRX_PTW_1280     (9) /**< Paging time window 12.80 sec */
#define LTE_EDRX_PTW_1408    (10) /**< Paging time window 14.08 sec */
#define LTE_EDRX_PTW_1536    (11) /**< Paging time window 15.36 sec */
#define LTE_EDRX_PTW_1664    (12) /**< Paging time window 16.64 sec */
#define LTE_EDRX_PTW_1792    (13) /**< Paging time window 17.92 sec */
#define LTE_EDRX_PTW_1920    (14) /**< Paging time window 19.20 sec */
#define LTE_EDRX_PTW_2048    (15) /**< Paging time window 20.48 sec */

/** @} lteedrxptw */

/**
 * @defgroup ltepsmt3324unit Requested Active Time value (T3324)
 * Definitions of Requested Active Time value (T3324)
 * used in lte_psm_timeval_t
 * @{
 */

#define LTE_PSM_T3324_UNIT_2SEC (0) /**< Unit of request active time(T3324)
                                         2 sec */
#define LTE_PSM_T3324_UNIT_1MIN (1) /**< Unit of request active time(T3324)
                                         1 min */
#define LTE_PSM_T3324_UNIT_6MIN (2) /**< Unit of request active time(T3324)
                                         6 min */

/** @} ltepsmt3324unit */

/**
 * @defgroup ltepsmt3412unit Extended periodic TAU(Tracking Area Update)
 * value (T3412)
 * Definitions of Extended periodic TAU(Tracking Area Update) value (T3412)
 * used in lte_psm_timeval_t
 * @{
 */

#define LTE_PSM_T3412_UNIT_2SEC    (0)  /**< Unit of extended periodic TAU
                                             time(T3412) 2 sec */
#define LTE_PSM_T3412_UNIT_30SEC   (1)  /**< Unit of extended periodic TAU
                                             time(T3412) 30 sec */
#define LTE_PSM_T3412_UNIT_1MIN    (2)  /**< Unit of extended periodic TAU
                                             time(T3412) 1 min */
#define LTE_PSM_T3412_UNIT_10MIN   (3)  /**< Unit of extended periodic TAU
                                             time(T3412) 10 min */
#define LTE_PSM_T3412_UNIT_1HOUR   (4)  /**< Unit of extended periodic TAU
                                             time(T3412) 1 hour */
#define LTE_PSM_T3412_UNIT_10HOUR  (5)  /**< Unit of extended periodic TAU
                                             time(T3412) 10 hour */
#define LTE_PSM_T3412_UNIT_320HOUR (6)  /**< Unit of extended periodic TAU
                                             time(T3412) 320 hour */

/** @} ltepsmt3412unit */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @typedef lte_getapndata_t
 * Definition of APN settings.
 * This is notified by get_apnset_cb_t
 */

typedef struct lte_getapndata
{
  uint8_t session_id;       /**< Numeric value of the session ID */
  int8_t  apn[LTE_APN_LEN]; /**< Access Point Name. It is
                                 terminated with '\0' */
  uint8_t ip_type;          /**< Internet Protocol type. See @ref lteapn */
} lte_getapndata_t;

/**
 * @typedef lte_getapnset_t
 * Definition of APN settings.
 * This is notified by get_apnset_cb_t
 */

typedef struct lte_getapnset
{
  int8_t           listnum;  /**< Number of APN data list. */
  lte_getapndata_t *apnlist; /**< List of APN data.
                                  See @ref lte_getapndata_t */
} lte_getapnset_t;

  /**
 * @typedef lte_lte_datastatdata_t
 * Definition of Data connection state.
 * This is notified by get_datastat_cb_t
 */
typedef struct lte_lte_datastatdata
{
  uint8_t session_id;     /**< Numeric value of the session ID */
  uint8_t state;          /**< The state of data communication. */
} lte_lte_datastatdata_t;

typedef struct lte_getdatastat
{
  uint8_t                listnum;     /**< Number of APN data list. */
  lte_lte_datastatdata_t *statelist;  /**< List of Data communication state.
                                          See @ref lte_lte_datastatdata */
} lte_getdatastat_t;

/**
 * @typedef lte_version_t
 * Definition of version information of the modem.
 * This is notified by get_ver_cb_t
 */

typedef struct lte_version
{
  int8_t bb_product[LTE_VER_BB_PRODUCT_LEN]; /**< BB product version. It is
                                                  terminated with '\0' */
  int8_t np_package[LTE_VER_NP_PACKAGE_LEN]; /**< NP package version. It is
                                                  terminated with '\0' */
} lte_version_t;

/**
 * @typedef lte_getpin_t
 * Definition of PIN setting information.
 * This is notified by get_pinset_cb_t
 */

typedef struct lte_getpin
{
  uint8_t enable;            /**< PIN enable. See @ref ltepinenable */
  uint8_t status;            /**< PIN status. Refer to the this parameter only
                                  when @enable is LTE_PIN_DISABLE.
                                  See @ref ltepinstat */
  uint8_t pin_attemptsleft;  /**< PIN attempts left */
  uint8_t puk_attemptsleft;  /**< PUK attempts left */
  uint8_t pin2_attemptsleft; /**< PIN2 attempts left */
  uint8_t puk2_attemptsleft; /**< PUK2 attempts left */
} lte_getpin_t;

/**
 * @typedef lte_localtime_t
 * Definition of local time. This is notified by
 * get_localtime_cb_t and localtime_report_cb_t
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
 * @typedef lte_quality_t
 * Definition of parameters for quality information.
 * This is reported by quality_report_cb_t
 */

typedef struct lte_quality
{
  bool    valid; /**< Refer to the following parameters only when this flag
                      is valid. This is because valid parameters can not be
                      acquired when RF function is OFF and so on */
  int16_t rsrp;  /**< RSRP in dBm (-140-0) */
  int16_t rsrq;  /**< RSRQ in dBm (-60-0) */
  int16_t sinr;  /**< SINR in dBm (-128-40) */
  int16_t rssi;  /**< RSSI in dBm */
} lte_quality_t;

/**
 * @typedef lte_cellinfo_t
 * Definition of parameters for cell information.
 * This is reported by cellinfo_report_cb_t
 */

typedef struct lte_cellinfo
{
  bool     valid;                           /**< Refer to the following
                                                 parameters only when this
                                                 flag is valid. This is
                                                 because valid parameters
                                                 can not be acquired when RF
                                                 function is OFF and so on */
  uint32_t phycell_id;                      /**< Physical cell ID (0-503) */
  uint32_t earfcn;                          /**< EARFCN (0-262143) */
  uint8_t  mcc[LTE_CELLINFO_MCC_DIGIT];     /**< Mobile Country Code (000-999)
                                                 See @ref ltecellinfo */
  uint8_t  mnc_digit;                       /**< Digit number of mnc(2-3) */
  uint8_t  mnc[LTE_CELLINFO_MNC_DIGIT_MAX]; /**< Mobile Network Code (00-999)
                                                 See @ref ltecellinfo */
} lte_cellinfo_t;

/**
 * @typedef lte_edrx_setting_t
 * Definition of eDRX settings used in lte_set_edrx().
 * This is notified by get_edrx_cb_t 
 */

typedef struct lte_edrx_setting
{
  uint8_t  act_type;   /**< eDRX act type. @ref lteedrxtype */
  bool     enable;     /**< eDRX enable. See @ref lteenableflg */
  uint32_t edrx_cycle; /**< eDRX cycle. See @ref lteedrxcyc */
  uint32_t ptw_val;    /**< Paging time window. See @ref lteedrxptw */
} lte_edrx_setting_t;

/**
 * @typedef lte_psm_timeval_t
 * Definition of timer information for PSM
 */

typedef struct lte_psm_timeval
{
  uint8_t unit;     /**< Unit of timer value.
                         See @ref ltepsmt3324unit or @ref ltepsmt3412unit */
  uint8_t time_val; /**< Timer value (1-31) */
} lte_psm_timeval_t;

/**
 * @typedef lte_psm_setting_t
 * Definition of PSM settings used in lte_set_psm().
 * This is notified by get_psm_cb_t 
 */

typedef struct lte_psm_setting
{
  bool              enable;                /**< PSM enable.
                                                See @ref lteenableflg */
  lte_psm_timeval_t req_active_time;       /**< Requested Active Time value
                                                (T3324) */
  lte_psm_timeval_t ext_periodic_tau_time; /**< Extended periodic TAU value
	                                            (T3412) */
} lte_psm_setting_t;

/**
 * @typedef lte_ce_setting_t
 * Definition of CE settings used in lte_set_ce().
 * This is notified by get_ce_cb_t 
 */

typedef struct lte_ce_setting
{
  bool mode_a_enable; /**< Mode A enable. See @ref lteenableflg */
  bool mode_b_enable; /**< Mode B enable. See @ref lteenableflg */
} lte_ce_setting_t;

/** Definition of callback function.
 *  Since lte_power_control() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_power_control().
 * See @ref lteresult
 */

typedef void (*power_control_cb_t)(uint32_t result);

/** Definition of callback function.
 *  Since lte_attach_network() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_attach_network().
 * See @ref lteresult
 * @param[in] errcause : This parameter is valid when result
 * is an error. See @ref lteerrcause
 */

typedef void (*attach_net_cb_t)(uint32_t result, uint32_t errcause);

/** Definition of callback function.
 *  Since lte_detach_network() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_detach_network().
 * See @ref lteresult
 */

typedef void (*detach_net_cb_t)(uint32_t result);

/** Definition of callback function.
 *  Since lte_get_netstat() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_netstat().
 * See @ref lteresult
 * @param[in] state  : The state of network state.
 * See @ref ltenetstat
 */

typedef void (*get_netstat_cb_t)(uint32_t result, uint32_t state);

/** Definition of callback function.
 *  Since lte_data_on() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_data_on().
 * See @ref lteresult
 * @param[in] errcause : This parameter is valid when result
 * is an error. See @ref lteerrcause
 */

typedef void (*data_on_cb_t)(uint32_t result, uint32_t errcause);

/** Definition of callback function.
 *  Since lte_data_off() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_data_off().
 * See @ref lteresult
 * @param[in] errcause : This parameter is valid when result
 * is an error. See @ref lteerrcause
 */

typedef void (*data_off_cb_t)(uint32_t result, uint32_t errcause);

/** Definition of callback function.
 *  Since lte_get_datastat() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_datastat().
 * See @ref lteresult
 * @param[in] state  : The state of data communication.
 * See @ref lte_getdatastat_t
 */
typedef void (*get_datastat_cb_t)(uint32_t result,
  lte_getdatastat_t *datastate);

/** Definition of callback function.
 *  Since lte_get_dataconfig() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result: The result of lte_get_dataconfig().
 * See @ref lteresult
 * @param[in] data_type: Data type. See @ref ltedatatype
 * @param[in] general : Data transfer for general.
 * See @ref lteenableflg
 * @param[in] roaming : Data transfer for roaming.
 * See @ref lteenableflg
 */

typedef void (*get_dataconfig_cb_t)(uint32_t result, uint32_t data_type,
                                    bool general, bool roaming);

/** Definition of callback function.
 *  Since lte_set_dataconfig() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_set_dataconfig().
 * See @ref lteresult
 */

typedef void (*set_dataconfig_cb_t)(uint32_t result);

/** Definition of callback function.
 *  Since lte_get_apnset() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_apnset().
 * See @ref lteresult
 * @param[in] apn : APN settings.
 * See @ref lte_getapnset_t
 */

typedef void (*get_apnset_cb_t)(uint32_t result, lte_getapnset_t *apn);

/** Definition of callback function.
 *  Since lte_set_apn() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_set_apn().
 * See @ref lteresult
 */

typedef void (*set_apn_cb_t)(uint32_t result);

/** Definition of callback function.
 *  Since lte_get_version() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_version().
 * See @ref lteresult
 * @param[in] version : The version information of the modem.
 * See @ref lte_version_t
 */

typedef void (*get_ver_cb_t)(uint32_t result, lte_version_t *version);

/** Definition of callback function.
 *  Since lte_get_phoneno() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_phoneno().
 * See @ref lteresult
 * @param[in] errcause : Error cause.
 * See @ref lteerrcause
 * It is set only if the result is not successful.
 * @param[in] phoneno : A character string indicating phone number.
 * It is terminated with '\0'
 */

typedef void (*get_phoneno_cb_t)(uint32_t result, uint8_t errcause,
                                 int8_t *phoneno);

/** Definition of callback function.
 *  Since lte_get_imsi() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_imsi().
 * See @ref lteresult
 * @param[in] errcause : Error cause.
 * It is set only if the result is not successful.
 * See @ref lteerrcause
 * @param[in] imsi : A character string indicating IMSI.
 * It is terminated with '\0'
 */

typedef void (*get_imsi_cb_t)(uint32_t result, uint8_t errcause,
                              int8_t *imsi);

/** Definition of callback function.
 *  Since lte_get_imei() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_imei().
 * See @ref lteresult
 * @param[in] imei : A character string indicating IMEI.
 * It is terminated with '\0'
 */

typedef void (*get_imei_cb_t)(uint32_t result, int8_t *imei);

/** Definition of callback function.
 *  Since lte_get_pinset() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_pinset().
 * See @ref lteresult
 * @param[in] pinset : PIN settings information.
 * See @ref lte_getpin_t
 */

typedef void (*get_pinset_cb_t)(uint32_t result, lte_getpin_t *pinset);

/** Definition of callback function.
 *  Since lte_set_pinenable() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_set_pinenable().
 * See @ref lteresult
 * @param[in] attemptsleft : Number of attempts left.
 * It is set only if the result is not successful.
 */

typedef void (*set_pinenable_cb_t)(uint32_t result, uint8_t attemptsleft);

/** Definition of callback function.
 *  Since lte_change_pin() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_change_pin().
 * See @ref lteresult
 * @param[in] attemptsleft : Number of attempts left.
 * It is set only if the result is not successful.
 */

typedef void (*change_pin_cb_t)(uint32_t result, uint8_t attemptsleft);

/** Definition of callback function.
 *  Since lte_enter_pin() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_enter_pin().
 * See @ref lteresult
 * @param[in] simstat : State after PIN enter.
 * See @ref ltesimstat
 * @param[in] attemptsleft : Number of attempts left.
 * It is set only if the result is not successful.
 * If simstat is other than PIN, PUK, PIN2, PUK2, set the number of PIN.
 */

typedef void (*enter_pin_cb_t)(uint32_t result,
                               uint8_t simstat,
                               uint8_t attemptsleft);

/** Definition of callback function.
 *  Since lte_get_localtime() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_localtime().
 * See @ref lteresult
 * @param[in] localtime : Local time.
 * See @ref lte_localtime_t
 */

typedef void (*get_localtime_cb_t)(uint32_t result,
                                   lte_localtime_t *localtime);

/** Definition of callback function.
 *  Since lte_get_operator() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_operator().
 * See @ref lteresult
 * @param[in] oper : A character string indicating network operator.
 * It is terminated with '\0'
 * If it is not connected, the first character is '\0'.
 */

typedef void (*get_operator_cb_t)(uint32_t result, int8_t *oper);

/** Definition of callback function.
 *  Since lte_get_sleepmode() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_sleepmode().
 * See @ref lteresult
 * @param[in] sleepmode : Sleep mode of the modem. 
 * See @ref lteslpmode
 */

typedef void (*get_slpmode_cb_t)(uint32_t result, uint32_t sleepmode);

/** Definition of callback function.
 *  Since lte_set_sleepmode() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_set_sleepmode().
 * See @ref lteresult
 */

typedef void (*set_slpmode_cb_t)(uint32_t result);

/** Definition of callback function.
 *  Since lte_get_edrx() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_edrx().
 * See @ref lteresult
 * @param[in] settings : eDRX settings.
 * See @ref lte_edrx_setting_t
 */

typedef void (*get_edrx_cb_t)(uint32_t result, lte_edrx_setting_t *settings);

/** Definition of callback function.
 *  Since lte_set_edrx() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_set_edrx().
 * See @ref lteresult
 */

typedef void (*set_edrx_cb_t)(uint32_t result);

/** Definition of callback function.
 *  Since lte_get_psm() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_psm().
 * See @ref lteresult
 * @param[in] settings : PSM settings.
 * See @ref lte_psm_setting_t
 */

typedef void (*get_psm_cb_t)(uint32_t result, lte_psm_setting_t *settings);

/** Definition of callback function.
 *  Since lte_set_psm() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_set_psm().
 * See @ref lteresult
 */

typedef void (*set_psm_cb_t)(uint32_t result);

/** Definition of callback function.
 *  Since lte_get_ce() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_get_ce().
 * See @ref lteresult
 * @param[in] settings : CE settings.
 * See @ref lte_ce_setting_t
 */

typedef void (*get_ce_cb_t)(uint32_t result, lte_ce_setting_t *settings);

/** Definition of callback function.
 *  Since lte_set_ce() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] result : The result of lte_set_ce().
 * See @ref lteresult
 */

typedef void (*set_ce_cb_t)(uint32_t result);

/** Definition of callback function.
 *  When the network state changes, the network state is
 *  reported by this function.
 * @param[in] netstat : The network state.
 * See @ref ltenetstat
 */

typedef void (*netstat_report_cb_t)(uint32_t netstat);

/** Definition of callback function.
 *  When the SIM state changes, the SIM state is
 *  reported by this function.
 * @param[in] simstat : The SIM state.
 * See @ref ltesimstate
 */

typedef void (*simstat_report_cb_t)(uint32_t simstat);

/** Definition of callback function.
 *  When the local time changes, the local time is
 *  reported by this function.
 * @param[in] localtime : Local time.
 * See @ref lte_localtime_t
 */

typedef void (*localtime_report_cb_t)(lte_localtime_t *localtime);

/** Definition of callback function.
 *  The quality information is reported by this function. It is reported
 *  at intervals of the set report period.
 * @param[in] quality : Quality information.
 * See @ref lte_quality_t
 */

typedef void (*quality_report_cb_t)(lte_quality_t *quality);

/** Definition of callback function.
 *  The cell information is reported by this function. It is reported
 *  at intervals of the set report period.
 * @param[in] cellinfo : Cell information.
 * See @ref lte_cellinfo_t
 */

typedef void (*cellinfo_report_cb_t)(lte_cellinfo_t *cellinfo);

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

/**
 * @defgroup lte_funcs Functions
 * @{
 */

/**
 * Initialize the LTE library
 *
 * lte_initialize() initialize the LTE library resources.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_initialize(void);

/**
 * Finalize the LTE library
 *
 * lte_finalize() finalize the LTE library resources.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_finalize(void);

/**
 * Power on/off the modem
 *
 * lte_power_control() control power on/off the modem.
 *
 * @param [in] on: "Power on" or "Power off".
 * See @ref ltepowerctl
 * @param [in] callback: Callback function to notify that
 * power on/off has been completed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_power_control(bool on, power_control_cb_t callback);

/**
 * Attach to the LTE network.
 *
 * lte_attach_network() attach to the LTE network.
 *
 * @param [in] callback: Callback function to notify that
 * attach completed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_attach_network(attach_net_cb_t callback);

/**
 * Detach from the LTE network.
 *
 * lte_detach_network() detach from the LTE network.
 *
 * @param [in] callback: Callback function to notify that
 * detach completed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_detach_network(detach_net_cb_t callback);

/**
 * Get network state of the LTE.
 * The network state means whether it has been attached or not.
 *
 * lte_get_netstat() get network state of the LTE.
 *
 * @param [in] callback: Callback function to notify that
 * get network state completed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_netstat(get_netstat_cb_t callback);

/**
 * Enable the data communication feature.
 *
 * lte_data_on() enable the data communication feature.
 *
 * @param [in] session_id: The numeric value of the session ID defined
 * in the apn setting. See @ref ltesessionid for valid range
 * @param [in] callback: Callback function to notify that
 * enabling the data communication completed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_data_on(uint8_t session_id, data_on_cb_t callback);

/**
 * Disable the data communication feature.
 *
 * lte_data_off() disable the data communication feature.
 *
 * @param [in] session_id: The numeric value of the session ID defined
 * in the apn setting. See @ref ltesessionid for valid range
 * @param [in] callback: Callback function to notify that
 * disabling the data communication completed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_data_off(uint8_t session_id, data_off_cb_t callback);

/**
 * Get state of the data communication
 *
 * lte_get_datastat() get state of the data communication.
 *
 * @param [in] session_id: The numeric value of the session ID defined
 * in the apn setting. See @ref ltesessionid for valid range.
 *
 * @param [in] callback: Callback function to notify that
 * get data communication state completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_datastat(get_datastat_cb_t callback);

/**
 * Get Configuration of the data transfer.
 * There are two types of data that can be specified: user data or IMS.
 *
 * lte_get_dataconfig() get configration of the data transfer.
 *
 * @param [in] data_type: Data type. See @ref ltedatatype
 * @param [in] callback: Callback function to notify that
 * get configuration of the data transfer completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_dataconfig(uint32_t data_type, get_dataconfig_cb_t callback);

/**
 * Change configuration of the data transfer.
 * There are two types of data that can be specified: user data or IMS.
 *
 * lte_set_dataconfig() change configration of the data transfer.
 * 
 * Details are shown in the table below.
 *
 * general  | roaming  | data transfer(Home/Roaming)
 * -------- | -------- | -----------------------------
 * disable  | disable  | Not available / Not available
 * disable  | enable   | Not available / Not available
 * enable   | disable  | Available / Not available
 * enable   | enable   | Available / Available
 *
 * @param [in] data_type: Data type. See @ref ltedatatype
 * @param [in] general: Data transfer for general.
 * See @ref lteenableflg
 * @param [in] roaming: Data transfer for roaming.
 * See @ref lteenableflg
 * @param [in] callback: Callback function to notify that
 * change configuration of the data transfer completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_set_dataconfig(uint32_t data_type, bool general, bool roaming,
                           set_dataconfig_cb_t callback);

/**
 * Get access point name settings.
 *
 * lte_get_apnset() Get APN settings.
 *
 * @param [in] callback: Callback function to notify that
 * get of APN setting completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_apnset(get_apnset_cb_t callback);

/**
 * Set access point name settings.
 *
 * lte_set_apn() set APN settings.
 *
 * @param [in] session_id: The numeric value of the session ID.
 * See @ref ltesessionid for valid range
 * @param [in] apn: Character string of Access Point Name.
 * The maximum string length is LTE_APN_LEN, end with '\0'. See @ref lteapnlen
 * @param [in] ip_type: Internet protocol type. See @ref lteapniptype
 * @param [in] auth_type: Authentication type. See @ref lteapnauthtype
 * @param [in] user_name: Character string of user name.
 * The maximum string length is LTE_APN_USER_NAME_LEN, end with '\0'.
 * See @ref lteapnusrnamelen
 * @param [in] password: Character string of password.
 * The maximum string length is LTE_APN_PASSWD_LEN, end with '\0'.
 * See @ref lteapnusrnamelen
 * @param [in] callback: Callback function to notify that
 * set of APN setting completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_set_apn(uint8_t session_id, int8_t *apn, uint8_t ip_type,
                    uint8_t auth_type, int8_t *user_name, int8_t *password,
                    set_apn_cb_t callback);

/**
 * Get the version of the modem.
 *
 * lte_get_version() get the version of the modem.
 *
 * @param [in] callback: Callback function to notify that
 * get of version is completed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_version(get_ver_cb_t callback);

/**
 * Get phone number.
 *
 * lte_get_phoneno() Get phone number.
 *
 * @param [in] callback: Callback function to notify that
 * get of phoneno is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_phoneno(get_phoneno_cb_t callback);

/**
 * Get International Mobile Subscriber Identity.
 *
 * lte_get_imsi() Get IMSI.
 *
 * @param [in] callback: Callback function to notify that
 * get of IMSI is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_imsi(get_imsi_cb_t callback);

/**
 * Get International Mobile Equipment Identifier.
 *
 * lte_get_imei() get the IMEI.
 *
 * @param [in] callback: Callback function to notify that
 * get of IMEI is completed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_imei(get_imei_cb_t callback);

/**
 * Get Personal Identification Number settings information.
 *
 * lte_get_pinset() Get PIN settings information.
 *
 * @param [in] callback: Callback function to notify that
 * get of PIN settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_pinset(get_pinset_cb_t callback);

/**
 * Set Personal Identification Number enable.
 *
 * lte_set_pinenable() Set PIN enable settings.
 *
 * @param [in] enable: "Enable" or "Disable".
 * See @ref ltepinenable
 * @param [in] pincode: Current PIN code. Minimum number of digits is 4.
 * Maximum number of digits is 8, end with '\0'. (i.e. Max 9 byte)
 * @param [in] callback: Callback function to notify that
 * set of PIN enable is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_set_pinenable(bool enable, int8_t *pincode,
                          set_pinenable_cb_t callback);

/**
 * Change Personal Identification Number.
 *
 * lte_change_pin() Change PIN settings.
 *
 * @param [in] target_pin: Target of change PIN.
 * See @ref ltetargetpin
 * @param [in] pincode: Current PIN code. Minimum number of digits is 4.
 * Maximum number of digits is 8, end with '\0'. (i.e. Max 9 byte)
 * @param [in] new_pincode: New PIN code. Minimum number of digits is 4.
 * Maximum number of digits is 8, end with '\0'. (i.e. Max 9 byte)
 * @param [in] callback: Callback function to notify that
 * change of PIN is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_change_pin(int8_t target_pin, int8_t *pincode,
                       int8_t *new_pincode, change_pin_cb_t callback);

/**
 * Enter Personal Identification Number.
 *
 * lte_enter_pin() Enter PIN.
 *
 * @param [in] pincode: Current PIN code. Minimum number of digits is 4.
 * Maximum number of digits is 8, end with '\0'. (i.e. Max 9 byte)
 * @param [in] new_pincode: If not used, set NULL.
 * If the PIN is SIM PUK or SIM PUK2, the new_pincode is required.
 * Minimum number of digits is 4.
 * Maximum number of digits is 8, end with '\0'. (i.e. Max 9 byte)
 * @param [in] callback: Callback function to notify that
 * PIN enter is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_enter_pin(int8_t *pincode, int8_t *new_pincode,
                      enter_pin_cb_t callback);

/**
 * Get local time.
 *
 * lte_get_localtime() Get local time.
 *
 * @param [in] callback: Callback function to notify that
 * get local time is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_localtime(get_localtime_cb_t callback);

/**
 * Get network operator information.
 *
 * lte_get_operator() Get network operator information.
 *
 * @param [in] callback: Callback function to notify that
 * get network operator information is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_operator(get_operator_cb_t callback);

/**
 * Get sleep mode of the modem.
 *
 * lte_get_sleepmode() get sleep mode of the modem.
 *
 * @param [in] callback: Callback function to notify that
 * get sleep mode is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_sleepmode(get_slpmode_cb_t callback);

/**
 * Set sleep mode of the modem.
 *
 * lte_set_sleepmode() set sleep mode of the modem.
 *
 * @param [in] sleepmode: Sleep mode of the modem.
 * See @ref lteslpmode
 * @param [in] callback: Callback function to notify that
 * set sleep mode is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_set_sleepmode(uint32_t sleepmode, set_slpmode_cb_t callback);

/**
 * Get eDRX settings.
 *
 * lte_get_edrx() get eDRX settings.
 *
 * @param [in] callback: Callback function to notify that
 * get eDRX settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_edrx(get_edrx_cb_t callback);

/**
 * Set eDRX settings.
 *
 * lte_set_edrx() set eDRX settings.
 *
 * @param [in] settings: eDRX settings
 * @param [in] callback: Callback function to notify that
 * get eDRX settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_set_edrx(lte_edrx_setting_t *settings,
                     set_edrx_cb_t callback);

/**
 * Get PSM settings.
 *
 * lte_get_psm() get PSM settings.
 *
 * @param [in] callback: Callback function to notify that
 * get PSM settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_psm(get_psm_cb_t callback);

/**
 * Set PSM settings.
 *
 * lte_set_psm() set PSM settings.
 *
 * @param [in] settings: PSM settings
 * @param [in] callback: Callback function to notify that
 * get PSM settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_set_psm(lte_psm_setting_t *settings,
                    set_psm_cb_t callback);

/**
 * Get CE settings.
 *
 * lte_get_ce() get CE settings.
 *
 * @param [in] callback: Callback function to notify that
 * get CE settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_get_ce(get_ce_cb_t callback);

/**
 * Set CE settings.
 *
 * lte_set_ce() set CE settings.
 *
 * @param [in] settings: CE settings
 * @param [in] callback: Callback function to notify that
 * get CE settings is completed.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_set_ce(lte_ce_setting_t *settings,
                   set_ce_cb_t callback);

/**
 * Change the report setting of the LTE network state.
 *
 * lte_set_report_netstat() Change the report setting of the LTE
 * network state and data communication state.
 * The default report setting is disable.
 *
 * @param [in] netstat_callback: Callback function to notify that
 * LTE network state. If NULL is set, the report setting is disabled.
 * @param [in] result_callback: Callback function to notify that
 * report setting has changed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_set_report_netstat(netstat_report_cb_t netstat_callback);

/**
 * Change the report setting of the SIM state.
 *
 * lte_set_report_simstat() Change the report setting of the SIM
 * state. The default report setting is disable.
 *
 * @param [in] simstat_callback: Callback function to notify that
 * SIM state. If NULL is set, the report setting is disabled.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_set_report_simstat(simstat_report_cb_t simstat_callback);

/**
 * Change the report setting of the local time.
 *
 * lte_set_report_localtime() Change the report setting of the local
 * time. The default report setting is disable.
 *
 * @param [in] localtime_callback: Callback function to notify that
 * local time. If NULL is set, the report setting is disabled.
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_set_report_localtime(localtime_report_cb_t localtime_callback);

/**
 * Change the report setting of the quality information.
 *
 * lte_set_report_quality() Change the report setting of the quality
 * information. The default report setting is disable.
 *
 * @param [in] quality_callback: Callback function to notify that
 * quality information. If NULL is set, the report setting is disabled.
 * @param [in] period: Reporting cycle in sec (1-4233600)
 * @param [in] result_callback: Callback function to notify that
 * report setting has changed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_set_report_quality(quality_report_cb_t quality_callback,
                               uint32_t period);

/**
 * Change the report setting of the cell information.
 *
 * lte_set_report_cellinfo() Change the report setting of the cell
 * information. The default report setting is disable.
 *
 * @param [in] cellinfo_callback: Callback function to notify that
 * cell information. If NULL is set, the report setting is disabled.
 * @param [in] period: Reporting cycle in sec (1-4233600)
 * @param [in] result_callback: Callback function to notify that
 * report setting has changed
 *
 * @return On success, 0 is returned. On failure,
 * negative value is returned.
 */

int32_t lte_set_report_cellinfo(cellinfo_report_cb_t cellinfo_callback,
                                uint32_t period);

/** @} lte_funcs */

#undef EXTERN
#ifdef __cplusplus
}
#endif

/** @} lte */

#endif /* __MODULES_INCLUDE_LTE_LTE_API_H */
