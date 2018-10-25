/****************************************************************************
 * modules/bluetooth/hal/bcm20706/include/bt/bt_hfp_ag.h
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
 * @file    bt_hfp_ag.h
 */

#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_HFP_AG_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_HFP_AG_H

/**
 * @defgroup BT
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <bt/bt_comm.h>
#include <bt/bt_hfp_hf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/**
 * @defgroup bt_defs Defines
 * @{
 */

/**
 * @name AG Group Event Opcode Type
 * @{
 */
#define BT_EVT_AG_DISCONNECT                     0x02    /* hands-free AG disconnect event */
#define BT_EVT_AG_CONNECT                        0x03    /* hands-free AG connect event */
#define BT_EVT_AG_AUDIO_CONNECT                  0x04    /* hands-free AG audio connect event */
#define BT_EVT_AG_AUDIO_DISCONNECT               0x05    /* hands-free AG audio disconnect event */
#define BT_EVT_AG_RECEIVE_AT_CMD                 0x06    /* hands-free AG receive AT command event */
/** @} */

/** @} bt_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup bt_datatypes Data types
 * @{
 */

/**@brief BT AG connect event structure
*/
typedef struct
{
  BT_ADDR addr;
  uint32_t feature;
} BT_AG_CONNECT;

/**
 *@name BT maximum AT command length
 *@{
 */
#define BT_AG_MAX_AT_CMD_LEN        128
/** @} */

/**@brief BT AG receive AT command event structure
*/
typedef struct agAtData
{
  BT_ADDR addr;
  uint16_t len;
  char atCmd[BT_AG_MAX_AT_CMD_LEN];
} BT_AG_AT_CMD;

/**@brief AG callback function
 */
typedef int (*audioGatewayEvtCallBack)(BT_SESSION_EVT *);

/** @} bt_datatypes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup bt_funcs Functions
 * @{
 */

/**@brief   Create AG connection to remote device
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
int BT_AgConnect(BT_ADDR *addr);

/**@brief   distroy AG connection to remote device
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
int BT_AgDisconnect(BT_ADDR *addr);

/**@brief   set AG event callback
 *
 * @param[in]  addr: remote device
 * @param[in]  cb: AG rx callback
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
int BT_AgSetEvtCallBack(BT_ADDR *addr, audioGatewayEvtCallBack cb);

/**@brief   AG audio connect
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
int BT_AgConnectAudio(BT_ADDR *addr);

/**@brief   AG audio disconnect
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
int BT_AgDisconnectAudio(BT_ADDR *addr);

/** @} bt_funcs */

/** @} BT */

#endif /* __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_HFP_AG_H */
