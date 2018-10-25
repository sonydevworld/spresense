/****************************************************************************
 * modules/bluetooth/hal/bcm20706/include/bt/bt_hfp_hf.h
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
 * @file    bt_hfp_hf.h
 */

#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_HFP_HF_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_HFP_HF_H

/**
 * @defgroup BT
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <bt/bt_comm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/**
 * @defgroup bt_defs Defines
 * @{
 */

/**
 * @name hands-free AT command code
 * @{
 */
#define BT_HF_AT_COMMAND_SPK                     0x20    /* Update speaker volume */
#define BT_HF_AT_COMMAND_MIC                     0x21    /* Update microphone volume */
#define BT_HF_AT_COMMAND_A                       0x22    /* Answer incoming call */
#define BT_HF_AT_COMMAND_BINP                    0x23    /* Retrieve number from voice tag */
#define BT_HF_AT_COMMAND_BVRA                    0x24    /* Enable/Disable voice recognition */
#define BT_HF_AT_COMMAND_BLDN                    0x25    /* Last Number redial */
#define BT_HF_AT_COMMAND_CHLD                    0x26    /* Call hold command */
#define BT_HF_AT_COMMAND_CHUP                    0x27    /* Call hang up command */
#define BT_HF_AT_COMMAND_CIND                    0x28    /* Read Indicator Status */
#define BT_HF_AT_COMMAND_CNUM                    0x29    /* Retrieve Subscriber number */
#define BT_HF_AT_COMMAND_D                       0x2A    /* Place a call using a number or memory dial */
#define BT_HF_AT_COMMAND_NREC                    0x2B    /* Disable Noise reduction and echo canceling in AG */
#define BT_HF_AT_COMMAND_VTS                     0x2C    /* Transmit DTMF tone */
#define BT_HF_AT_COMMAND_BTRH                    0x2D    /* CCAP incoming call hold */
#define BT_HF_AT_COMMAND_COPS                    0x2E    /* Query operator selection */
#define BT_HF_AT_COMMAND_CMEE                    0x2F    /* Enable/disable extended AG result codes */
#define BT_HF_AT_COMMAND_CLCC                    0x30    /* Query list of current calls in AG */
#define BT_HF_AT_COMMAND_BIA                     0x31    /* Activate/Deactivate indicators */
#define BT_HF_AT_COMMAND_BIEV                    0x32    /* Send HF indicator value to peer */
#define BT_HF_AT_COMMAND_UNAT                    0x33    /* Transmit AT command not in the spec  */
#define BT_HF_AT_COMMAND_MAX                     0x33    /* For command validation */
/** @} */

/**
 * @name hands-free Group Event Opcode Type
 * @{
 */
#define BT_EVT_HF_COMMAND_STATUS                 0x00    /* hands-free HF command status event */
#define BT_EVT_HF_DISCONNECT                     0x02    /* hands-free HF disconnect event */
#define BT_EVT_HF_AUDIO_CONNECT                  0x04    /* hands-free HF audio connect event */
#define BT_EVT_HF_AUDIO_DISCONNECT               0x05    /* hands-free HF audio disconnect event */
#define BT_EVT_HF_RESPONSE                       0x06    /* hands-free HF response event */
#define BT_EVT_HF_CONNECT                        0x07    /* hands-free HF connect event */
#define BT_EVT_HF_AG_FEATURE                     0x08    /* hands-free HF peer device feature event */
/** @} */

/**
 * @name BT HF supported feature flags
 * @{
 */
#define BT_HFP_HF_FEATURE_ECNR                             0x00000001    /* EC and/or NR fucntion */
#define BT_HFP_HF_FEATURE_3WAY_CALLING                     0x00000002    /* Three-way calling */
#define BT_HFP_HF_FEATURE_CLIP_CAPABILITY                  0x00000004    /* CLI presentation capaability */
#define BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION     0x00000008    /* Voice recognition activation */
#define BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL            0x00000010    /* Remote volume control */
#define BT_HFP_HF_FEATURE_ENHANCED_CALL_STATUS             0x00000020    /* Enhanccd call status, not supported now */
#define BT_HFP_HF_FEATURE_ENHANCED_CALL_CONTROL            0x00000040    /* Enhance call control, not supported now */
#define BT_HFP_HF_FEATURE_CODEC_NEGOTIATION                0x00000080    /* Codec negotiation */
#define BT_HFP_HF_FEATURE_HF_INDICATORS                    0x00000100    /* HF Indicators */
#define BT_HFP_HF_FEATURE_ESCO_S4_T2_SETTINGS_SUPPORT      0x00000200    /* eSCO S4(and T2) Setting Supported, not supported now */
/** @} */

/** @} bt_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup bt_datatypes Data types
 * @{
 */

/**@brief BT HF codec type
 */
typedef enum
{
  BT_HFP_HF_CSVD_CODEC = 1,
  BT_HFP_HF_MSBC_CODEC = 2
} BT_HFP_HF_CODEC_TYPE;

/**@brief BT HF response opcode
 */
typedef enum
{
  HF_OK_RES = 0x20,
  HF_ERROR_RES,
  HF_EXTERROR_RES,                       /* +CMEE */
  HF_INCOMING_CALL,                      /* +RING */
  HF_SPEAKER_GAIN,                       /* +VGS */
  HF_MICROPHONE_GAIN,                    /* +VGM */
  HF_INCOMING_CALL_WAITING,              /* +CCWA */
  HF_CALL_HOLD,                          /* +CHLD */
  HF_AG_INDICATORS,                      /* +CIND */
  HF_CALLER_PHONE_NUM,                   /* +CLIP */
  HF_AG_INDICATOR_CHANGED,               /* +CIEV */
  HF_NUMBER_ATTACHED_TO_VOICE_TAG,       /* +BINP */
  HF_VOICE_RECOGNITION_STATUS,           /* +BVRA */
  HF_INBAND_RING_TONE,                   /* +BSIR */
  HF_SUBSCRIBER_NUM,                     /* +CNUM */
  HF_CALL_HOLD_STATUS,                   /* +BTRH */
  HF_OPERATOR_INFO,                      /* +COPS */
  HF_ACTIVE_CALL_LIST,                   /* +CLCC */
  HF_SUPPORTED_HF_INDICATORS,            /* +BIND */
  HF_CODEC_SELECTION,                    /* +BCS, ref@ BT_HFP_HF_CODEC_TYPE */
  HF_UNKNOWN_AT_RES,
} BT_HF_RES_OPCODE;

/**@brief BT HF connect event structure
*/
typedef struct hfConnect
{
  BT_ADDR addr;
  BT_PROFILE_TYPE btProfileType;
} BT_HF_CONNECT;

/**@brief BT HF peer device AG feature
*/
typedef struct
{
  BT_ADDR addr;
  uint32_t feature;
} BT_HF_AG_FEATURE;

/*
 *@name BT HF maximum optional string length
 *@{
 */
#define BT_HF_MAX_OPTIONAL_STRING_LEN        128
/** @} */

/**@brief BT HF reponse event structure
*/
typedef struct hfResponse
{
  BT_ADDR addr;
  BT_HF_RES_OPCODE opcode;
  uint16_t numeric;
  char optionalString[BT_HF_MAX_OPTIONAL_STRING_LEN];
} BT_HF_RESPONSE;

/**@brief HF callback function
 */
typedef int (*handsfreeEvtCallBack)(BT_SESSION_EVT *);

/** @} bt_datatypes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup bt_funcs Functions
 * @{
 */

/**@brief   Create hands-free connection to remote device
 *
 * @param[in]  addr: remote device address
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find nearby device
 * @retval     -ENOMEM: no more memory
 * @retval     -EINVAL: invalid argument
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_HfConnect(BT_ADDR *addr);

/**@brief   distroy hands-free connection to remote device
 *
 * @param[in]  addr: remote device address
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find connected device
 * @retval     -EINVAL: invalid argument
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_HfDisconnect(BT_ADDR *addr);

/**@brief   set hands-free event callback
 *
 * @param[in]  addr: remote device
 * @param[in]  cb: hands-free rx callback
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find connected device
 * @retval     -EINVAL: invalid argument
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_HfSetEvtCallBack(BT_ADDR *addr, handsfreeEvtCallBack cb);

/**@brief   hands-free audio connect
 *
 * @param[in]  addr: remote device address
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find nearby device
 * @retval     -EINVAL: invalid argument
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_HfConnectAudio(BT_ADDR *addr);

/**@brief   hands-free audio disconnect
 *
 * @param[in]  addr: remote device address
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find connected device
 * @retval     -EINVAL: invalid argument
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_HfDisconnectAudio(BT_ADDR *addr);

/**@brief   hands-set button press
 *
 * @param[in]  addr: remote device address
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find connected device
 * @retval     -EINVAL: invalid argument
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_HsButtonPress(BT_ADDR *addr);

/**@brief   hands-free send AT command
 *
 * @param[in]  addr: remote device address
 * @param[in]  code: command code
 * @param[in]  numeric: numeric number
 * @param[in]  optionalString: optional supporting character string, end with '\0'
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find connected device
 * @retval     -EINVAL: invalid argument
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_HfSendAT(BT_ADDR *addr, uint16_t code, uint16_t numeric, char *optionalString);

/**@brief   hands-free set HF supported feature
 *          This API shall be called before BT_CommonInitializeWithBtBinary()
 *
 * @param[in]  btHfFeature: HF supported feature ref@BT HF supported feature flags
 *
 * @return     0 on success, otherwise error
 * @retval     -EINVAL: invalid argument
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_HfSetSupportedFeature(uint32_t *btHfFeature);

/** @} bt_funcs */

/** @} BT */

#endif /* __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_HFP_HF_H */
