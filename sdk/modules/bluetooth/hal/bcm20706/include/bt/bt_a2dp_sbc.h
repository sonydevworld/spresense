/****************************************************************************
 * modules/bluetooth/hal/bcm20706/include/bt/bt_a2dp_sbc.h
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
 * @file       bt_a2dp_sbc.h
 */
#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_A2DP_SBC_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_A2DP_SBC_H

/**
 * @defgroup BT
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <bt/bt_comm.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup bt_datatypes Data types
 * @{
 */

/**@brief sbc codec sampling frequency
 */
typedef enum audioSamplingFrequency
{
   BT_AUDIO_FREQUENCY_16KHZ,
   BT_AUDIO_FREQUENCY_32KHZ,
   BT_AUDIO_FREQUENCY_44KHZ,
   BT_AUDIO_FREQUENCY_48KHZ,
} BT_AUDIO_FREQUENCY;

/**@brief sbc codec channel mode
 */
typedef enum audioChannelMode
{
   BT_AUDIO_CHANNEL_MODE_MONO,
   BT_AUDIO_CHANNEL_MODE_STEREO,
} BT_AUDIO_CHANNEL_MODE;

/** @} bt_datatypes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup bt_funcs Functions
 * @{
 */

/**@brief   send audio data to remote device
 *
 * @param[in]  addr: remote device
 * @param[in]  freq: sbc codec sampling frequency
 * @param[in]  mode: sbc codec channel mode
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find connected device
 * @retval     -EINVAL: a2dp started
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_A2dpSrcSbcStartPlay(BT_ADDR *addr, BT_AUDIO_FREQUENCY freq, BT_AUDIO_CHANNEL_MODE mode);

/** @} bt_funcs */

/** @} BT */
#endif /* __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_A2DP_SBC_H */
