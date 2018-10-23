/****************************************************************************
 * modules/include/bluetooth/bt_a2dp_codecs.h
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
 * @file bt_a2dp_codecs.h
 * @author Sony Semiconductor Solutions Corporation
 * @brief Bluetooth A2DP common header for SDK on Spresense.
 * @details This header file includes bluetooth A2DP common definition between
 *          API and HAL I/F.
 *           - Codec information
 */

#ifndef __MODULES_INCLUDE_BLUETOOTH_BT_A2DP_CODECS_H
#define __MODULES_INCLUDE_BLUETOOTH_BT_A2DP_CODECS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * @name Codec Information format data max length
 * @{
 */
#define CODEC_INFO_MAX_LENGTH  20
/** @} */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**@brief Masks for supported Codecs
 */
typedef enum
{
	BT_A2DP_SINK_CODEC_SBC             = 1 << 0, /**< SBC Codec */
	BT_A2DP_SINK_CODEC_AAC             = 1 << 2, /**< AAC */
	/**< New codec might be add */
} BT_A2DP_CODEC_TYPE;

/**@brief Masks for supported Codecs
 */
typedef enum
{
	BT_A2DP_SINK_DIRECT_TRANSFER       = 0x00, /**< Use direct transfer to BB */
	BT_A2DP_SINK_PACKET_TRANSFER       = 0x01, /**< Use packet transfer to decode */
} BT_A2DP_TRANSFER_TYPE;

/**@brief Masks for supported sample frequency TODO: Need to create mask
 */
typedef enum
{
	BT_A2DP_SAMPLE_FREQ_8000HZ,
	BT_A2DP_SAMPLE_FREQ_11000HZ,
	BT_A2DP_SAMPLE_FREQ_12000HZ,
	BT_A2DP_SAMPLE_FREQ_16000HZ,
	BT_A2DP_SAMPLE_FREQ_22050HZ,
	BT_A2DP_SAMPLE_FREQ_24000HZ,
	BT_A2DP_SAMPLE_FREQ_32000HZ,
	BT_A2DP_SAMPLE_FREQ_44100HZ,
	BT_A2DP_SAMPLE_FREQ_48000HZ,
	BT_A2DP_SAMPLE_FREQ_64000HZ,
	BT_A2DP_SAMPLE_FREQ_88000HZ,
	BT_A2DP_SAMPLE_FREQ_96000HZ
} BT_A2DP_SAMPLE_FREQ;

/**@brief Masks for supported channel TODO: Need to create mask
 */
typedef enum
{
	BT_A2DP_CHANNEL_MONO,
	BT_A2DP_CHANNEL_DUAL,
	BT_A2DP_CHANNEL_STEREO,
	BT_A2DP_CHANNEL_JOINTSTEREO
} BT_A2DP_CHANNEL_MODE;

/**@brief Masks for supported allocation method
 */
typedef enum
{
	BT_A2DP_CHANNEL_SNR,
	BT_A2DP_CHANNEL_LOUDNESS
} BT_A2DP_SBC_ALLOC_MODE;

/**@brief data type for the SBC Codec Information Element
 * <pre>
 * sampFreq:    bit7-bit4 are defined as sampling frequency
 *
 * chMode:      bit3-bit0 are defined as channel mode
 *
 * blockLen:    bit7-bit4 are defined as number of blocks
 *
 * numSubbands: bit3-bit2 are defined as number of subbands
 *
 * allocMthd:   bit1-bit0 are defined as allocation mode
 *
 * maxBitpool:  maximum bitpool value (<= 250)
 *
 * minBitpool:  minimum bitpool value (>= 2)
 * </pre>
 */
typedef struct
{
	BT_A2DP_SAMPLE_FREQ    sampFreq;
	BT_A2DP_CHANNEL_MODE   chMode;
	uint8_t                blockLen;
	uint8_t                numSubbands;
	BT_A2DP_SBC_ALLOC_MODE allocMthd;
	uint8_t                maxBitpool;
	uint8_t                minBitpool;
} BT_A2D_SBC_CODEC_INFO;


/**@brief Masks for supported Codecs
 */
typedef enum
{
	BT_A2DP_MPEG2_AAC_LC,
	BT_A2DP_MPEG4_AAC_LC,
	BT_A2DP_MPEG4_AAC_LTP,
	BT_A2DP_MPEG4_AAC_SSR
} BT_A2DP_AAC_PROFILE;

/**@brief data type for the MPEG-2, 4 AAC Codec Information Element
 * <pre>
 * profile:  bit7-bit4 are defined as object type, bit3-bit0 are not used
 *
 * sampFreq: bit15-bit4 are defined as sampling frequency
 *
 * chnl:     bit3-bit2 are defined as channels
 *
 * vbr:      bit0 is defined as Variable Bit Rate
 *           0x01: VBR is supported
 *
 * bitrate:  bit22-bit0 are defined as bit rate
 * </pre>
 */
typedef struct
{
	BT_A2DP_AAC_PROFILE  profile;
	BT_A2DP_SAMPLE_FREQ  sampFreq;
	BT_A2DP_CHANNEL_MODE chnl;
	uint8_t              isVbrSupported;
	uint32_t             bitrate;
} BT_A2D_AAC_CODEC_INFO;

/**@brief Vendor Specific Codec information element type
 */
typedef struct
{
	uint8_t  codecInfoLength;
	uint8_t  codecInfo[CODEC_INFO_MAX_LENGTH];
} BT_A2D_VENDOR_CODEC_INFO;

/**@brief Codec information element structure, used to provide info of a single type of codec
 */
typedef struct
{
	BT_A2DP_CODEC_TYPE codecId;
	union
	{
		BT_A2D_SBC_CODEC_INFO    sbc;
		BT_A2D_AAC_CODEC_INFO    aac;
		BT_A2D_VENDOR_CODEC_INFO vsp;
	} codec_info;
  BT_A2DP_TRANSFER_TYPE transfer_type;
} BT_AUDIO_CODEC_INFO;

#endif /* __MODULES_INCLUDE_BLUETOOTH_BT_A2DP_CODECS_H */
