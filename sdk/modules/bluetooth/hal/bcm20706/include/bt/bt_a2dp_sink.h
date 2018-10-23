/****************************************************************************
 * modules/bluetooth/hal/bcm20706/include/bt/bt_a2dp_sink.h
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
 * @file    bt_a2dp_sink.h
 */


#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_A2DP_SINK_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_A2DP_SINK_H

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
 * @name A2DP_SINK Group Event Opcode Type
 * @{
 */
#define BT_EVT_A2DP_SINK_CMD_STATUS        0x01    /**< Command status event for the requested operation*/
#define BT_EVT_A2DP_SINK_CONNECT           0x02    /**< Audio connection opened*/
#define BT_EVT_A2DP_SINK_CONNECT_FAIL      0x04    /**< Connection attempt failed event*/
#define BT_EVT_A2DP_SINK_DISCONNECT        0x05    /**< Audio connection closed*/
#define BT_EVT_A2DP_SINK_START             0x06    /**< Audio start command received event*/
#define BT_EVT_A2DP_SINK_STOP              0x07    /**< Audio stop command received event*/
#define BT_EVT_A2DP_SINK_RECEIVE_DATA      0x0A    /**< Receive audio data event*/
/** @} */

/**
 * @name cie format data max length
 * @{
 */
#define CIE_MAX_LENGTH  20
/** @} */

/**
 * @name for SBC Codec Specific Information Element
 */
#define A2D_SBC_IE_SAMP_FREQ_MSK    0xF0    /**< b7-b4 sampling frequency */
#define A2D_SBC_IE_SAMP_FREQ_16     0x80    /**< b7:16  kHz */
#define A2D_SBC_IE_SAMP_FREQ_32     0x40    /**< b6:32  kHz */
#define A2D_SBC_IE_SAMP_FREQ_44     0x20    /**< b5:44.1kHz */
#define A2D_SBC_IE_SAMP_FREQ_48     0x10    /**< b4:48  kHz */

#define A2D_SBC_IE_CH_MD_MSK        0x0F    /**< b3-b0 channel mode */
#define A2D_SBC_IE_CH_MD_MONO       0x08    /**< b3: mono */
#define A2D_SBC_IE_CH_MD_DUAL       0x04    /**< b2: dual */
#define A2D_SBC_IE_CH_MD_STEREO     0x02    /**< b1: stereo */
#define A2D_SBC_IE_CH_MD_JOINT      0x01    /**< b0: joint stereo */

#define A2D_SBC_IE_BLOCKS_MSK       0xF0    /**< b7-b4 number of blocks */
#define A2D_SBC_IE_BLOCKS_4         0x80    /**< 4 blocks */
#define A2D_SBC_IE_BLOCKS_8         0x40    /**< 8 blocks */
#define A2D_SBC_IE_BLOCKS_12        0x20    /**< 12blocks */
#define A2D_SBC_IE_BLOCKS_16        0x10    /**< 16blocks */

#define A2D_SBC_IE_SUBBAND_MSK      0x0C    /**< b3-b2 number of subbands */
#define A2D_SBC_IE_SUBBAND_4        0x08    /**< b3: 4 */
#define A2D_SBC_IE_SUBBAND_8        0x04    /**< b2: 8 */

#define A2D_SBC_IE_ALLOC_MD_MSK     0x03    /**< b1-b0 allocation mode */
#define A2D_SBC_IE_ALLOC_MD_S       0x02    /**< b1: SNR */
#define A2D_SBC_IE_ALLOC_MD_L       0x01    /**< b0: loundess */

#define A2D_SBC_IE_MIN_BITPOOL      2
#define A2D_SBC_IE_MAX_BITPOOL      250

#define BT_AUDIO_A2DP_SBC_MAX_BITPOOL  59   /**< max sbc bit pool */
/** @} */

/**
 * @name for SBC Codec Specific Information Element
 */
#define A2D_M24_IE_OBJ_MSK          0xF0    /**< b7-b4 object type. b3-b0 is RFA,not used */
#define A2D_M24_IE_OBJ_2LC          0x80    /**< b7: MPEG-2 AAC LC */
#define A2D_M24_IE_OBJ_4LC          0x40    /**< b6: MPEG-4 AAC LC */
#define A2D_M24_IE_OBJ_4LTP         0x20    /**< b5: MPEG-4 AAC LTP */
#define A2D_M24_IE_OBJ_4S           0x10    /**< b4: MPEG-4 AAC scalable */

#define A2D_M24_IE_SAMP_FREQ_MSK    0xFFF0    /**< sampling frequency */
#define A2D_M24_IE_SAMP_FREQ_8      0x8000    /**< b7:8  kHz */
#define A2D_M24_IE_SAMP_FREQ_11     0x4000    /**< b6:11  kHz */
#define A2D_M24_IE_SAMP_FREQ_12     0x2000    /**< b5:12  kHz */
#define A2D_M24_IE_SAMP_FREQ_16     0x1000    /**< b4:16  kHz */
#define A2D_M24_IE_SAMP_FREQ_22     0x0800    /**< b3:22.05kHz */
#define A2D_M24_IE_SAMP_FREQ_24     0x0400    /**< b2:24  kHz */
#define A2D_M24_IE_SAMP_FREQ_32     0x0200    /**< b1:32  kHz */
#define A2D_M24_IE_SAMP_FREQ_44     0x0100    /**< b0:44.1kHz */
#define A2D_M24_IE_SAMP_FREQ_48     0x0080    /**< b7:48  kHz */
#define A2D_M24_IE_SAMP_FREQ_64     0x0040    /**< b6:64  kHz */
#define A2D_M24_IE_SAMP_FREQ_88     0x0020    /**< b5:88  kHz */
#define A2D_M24_IE_SAMP_FREQ_96     0x0010    /**< b4:96  kHz */

#define A2D_M24_IE_CHNL_MSK         0x0C    /**< b3-b2 channels */
#define A2D_M24_IE_CHNL_1           0x08    /**< b3: 1 channel */
#define A2D_M24_IE_CHNL_2           0x04    /**< b2: 2 channels */

#define A2D_M24_IE_VBR_MSK          0x80    /**< b7: VBR */

#define A2D_M24_IE_BITRATE3_MSK     0x7F0000    /**< octect3*/
#define A2D_M24_IE_BITRATE45_MSK    0x00FFFF    /**< octect4, 5*/
#define A2D_M24_IE_BITRATE_MSK      0x7FFFFF    /**< b7-b0 of octect 3, all of octect4, 5*/
/** @} */

/** @} bt_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup bt_datatypes Data types
 * @{
 */

/**@brief BT Event a2dp sink started
 */
typedef struct
{
  BT_ADDR addr;
} BT_EVT_A2DP_SINK_STARTED;

/**@brief BT Event a2dp sink stop
 */
typedef struct
{
  BT_ADDR addr;
} BT_EVT_A2DP_SINK_STOPPED;

/**@brief BT audio sink receive data structure
 */
typedef struct
{
  uint16_t len;
  uint8_t pAudioData[BT_EVT_AUDIO_DATA_LEN];
} BT_AUDIO_DATA_PACKET;

/**@brief Masks for supported Codecs
 */
typedef enum
{
  BT_BCM_A2DP_SINK_CODEC_SBC             = 0x00, /**< SBC Codec */
  BT_BCM_A2DP_SINK_CODEC_M12             = 0x01, /**< MPEG-1, 2 Codecs */
  BT_BCM_A2DP_SINK_CODEC_M24             = 0x02, /**< MPEG-2, 4 Codecs */
  BT_BCM_A2DP_SINK_CODEC_VENDOR_SPECIFIC = 0xFF, /**< Vendor specific codec */
} BT_BCM_A2DP_SINK_CODE;

/**@brief data type for the SBC Codec Information Element
 * <pre>
 * sampFreq:    bit7-bit4 are defined as sampling frequency
 *              0x80: 16000
 *              0x40: 32000
 *              0x20: 44100
 *              0x10: 48000
 *
 * chMode:      bit3-bit0 are defined as channel mode
 *              0x08: mono
 *              0x04: dual
 *              0x02: stereo
 *              0x01: joint stereo
 *
 * blockLen:    bit7-bit4 are defined as number of blocks
 *              0x80: 4 blocks
 *              0x40: 8 blocks
 *              0x20: 12 blocks
 *              0x10: 16 blocks
 *
 * numSubbands: bit3-bit2 are defined as number of subbands
 *              0x08: 4
 *              0x04: 8
 *
 * allocMthd:   bit1-bit0 are defined as allocation mode
 *              0x02: SNR
 *              0x01: loundess
 *
 * maxBitpool:  maximum bitpool value (<= 250)
 *
 * minBitpool:  minimum bitpool value (>= 2)
 * </pre>
 */
typedef struct
{
  uint8_t   sampFreq;      /**< Sampling frequency */
  uint8_t   chMode;        /**< Channel mode */
  uint8_t   blockLen;      /**< Block length */
  uint8_t   numSubbands;   /**< Number of subbands */
  uint8_t   allocMthd;     /**< Allocation method */
  uint8_t   maxBitpool;    /**< Maximum bitpool */
  uint8_t   minBitpool;    /**< Minimum bitpool */
} BT_A2D_SBC_CIE;

/**@brief data type for the MPEG-1, 2 Audio Codec Information Element
 * <pre>
 * layer:       bit7-bit5 are defined as layer
 *              0x80: layer1 (mp1)
 *              0x40: layer2 (mp2)
 *              0x20: layer3 (mp3)
 *
 * crc:         bit4 is defined as CRC protection
 *              0x10: CRC protection is supported
 *
 * chMode:      bit3-bit0 are defined as channel mode
 *              0x08: mono
 *              0x04: dual
 *              0x02: stereo
 *              0x01: joint stereo
 *
 * mpf:         bit6 is defined as Media Payload Format
 *              0x40: MPF-2 is supported
 *
 * sampFreq:    bit5-bit0 are defined as sampling frequency
 *              0x20: 16000
 *              0x10: 22050
 *              0x08: 24000
 *              0x04: 32000
 *              0x02: 44100
 *              0x01: 48000
 *
 * vbr:         bit0 is defined as Variable Bit Rate
 *              0x01: VBR is supported
 *
 * bitrate:     bit14-bit0 are defined as bit rate index
 *              0x0001: '0000'
 *              0x0002: '0001'
 *              0x0004: '0010'
 *              0x0008: '0011'
 *              0x0010: '0100'
 *              0x0020: '0101'
 *              0x0040: '0110'
 *              0x0080: '0111'
 *              0x0100: '1000'
 *              0x0200: '1001'
 *              0x0400: '1010'
 *              0x0800: '1011'
 *              0x1000: '1100'
 *              0x2000: '1101'
 *              0x4000: '1110'
 * </pre>
 */
typedef struct
{
  uint8_t   layer;      /**< layers */
  uint8_t   crc;        /**< Support of CRC protection or not */
  uint8_t   chMode;     /**< Channel mode */
  uint8_t   mpf;        /**< 1, if MPF-2 is supported. 0, otherwise */
  uint8_t   sampFreq;   /**< Sampling frequency */
  uint8_t   vbr;        /**< Variable Bit Rate */
  uint16_t  bitrate;    /**< Bit rate index */
} BT_A2D_M12_CIE;

/**@brief data type for the MPEG-2, 4 AAC Codec Information Element
 * <pre>
 * objType:  bit7-bit4 are defined as object type, bit3-bit0 are not used
 *           0x80: MPEG-2 AAC LC
 *           0x40: MPEG-4 AAC LC
 *           0x20: MPEG-4 AAC LTP
 *           0x10: MPEG-4 AAC scalable
 *
 * sampFreq: bit15-bit4 are defined as sampling frequency
 *           0x8000: 8000
 *           0x4000: 11000
 *           0x2000: 12000
 *           0x1000: 16000
 *           0x0800: 22050
 *           0x0400: 24000
 *           0x0200: 32000
 *           0x0100: 44100
 *           0x0080: 48000
 *           0x0040: 64000
 *           0x0020: 88000
 *           0x0010: 96000
 *
 * chnl:     bit3-bit2 are defined as channels
 *           0x08: 1 channel
 *           0x04: 2 channels
 *
 * vbr:      bit0 is defined as Variable Bit Rate
 *           0x01: VBR is supported
 *
 * bitrate:  bit22-bit0 are defined as bit rate
 * </pre>
 */
typedef struct
{
  uint8_t   objType;    /**< Object type */
  uint16_t  sampFreq;   /**< Sampling frequency */
  uint8_t   chnl;       /**< Channel mode */
  uint8_t   vbr;        /**< Variable Bit Rate */
  uint32_t  bitrate;    /**< Bit rate index */
} BT_A2D_M24_CIE;

/**@brief Vendor Specific Codec information element type
 */
typedef struct
{
  uint8_t  cieLength;                   /**< Length of codec information element in octets */
  uint8_t  cie[CIE_MAX_LENGTH];         /**< Codec information element */
} BT_A2D_VENDOR_CIE;

/**@brief Codec information element structure, used to provide info of a single type of codec
 */
typedef struct
{
  BT_BCM_A2DP_SINK_CODE codecId; /**< One of WICED_BT_A2DP_CODEC_XXX, to indicate the valid element of the cie union */
  union
  {
    BT_A2D_SBC_CIE    sbc; /**< SBC information element */
    BT_A2D_M12_CIE    m12; /**< MPEG-1, 2 information element */
    BT_A2D_M24_CIE    m24; /**< MPEG-2, 4 information element */
    BT_A2D_VENDOR_CIE vsp; /**< Vendor Specific codec information element */
  }cie;
} BT_BCM_AUDIO_CODEC_INFO;

/**@brief BT Event a2dp sink connected
 */
typedef struct
{
  BT_ADDR addr;
  BT_BCM_AUDIO_CODEC_INFO btAudioCodecInfo;
} BT_EVT_A2DP_SINK_CONNECTED;

/**@brief BT Event a2dp sink disconnected
 */
typedef struct
{
  BT_ADDR addr;
} BT_EVT_A2DP_SINK_DISCONNECTED;

/**@brief a2dp sink callback function
 */
typedef void (*a2dpSinkEvtCallBack)(BT_SESSION_EVT *);

/** @} bt_datatypes */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup bt_funcs Functions
 * @{
 */

/**@brief   Create a2dp sink connection to remote device
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
int BT_A2dpSinkConnect(BT_ADDR *addr);

/**@brief   distroy a2dp sink connecion to remote device
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
int BT_A2dpSinkDisconnect(BT_ADDR *addr);

/**@brief   distroy avdtp connecion to remote device,
 *            this API is just used for PTS test
 *
 * @param[in]  addr: remote device address
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
int BT_AvdtpDisconnect(BT_ADDR *addr);

/**@brief   set a2dp sink event callback
 *
 * @param[in]  addr: remote device
 * @param[in]  cb: a2dp rx callback
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
int BT_A2dpSinkSetEvtCallBack(BT_ADDR *addr, a2dpSinkEvtCallBack cb);

/**@brief   Enable/disable AAC
 *
 * @param[in]  btAacIsEnable: bt a2dp sink aac enable/disable
 *
 * @return     0 on success, otherwise error
 * @retval     -ENOENT: can't find controller sessionn
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_A2dpSinkEnableAac(BT_BOOL btAacIsEnable);

/**@brief   Set a2dp sink codec
 *
 * @param[in]  btCodecInfo: bt a2dp sink codec information
 *
 * @return     0 on success, otherwise error
 * @retval     -EINVAL: invalid argument
 * @retval     -ENOENT: can't find controller sessionn
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BT_A2dpSinkSetCodec(BT_BCM_AUDIO_CODEC_INFO *btCodecInfo);

/** @} bt_funcs */

/** @} BT */

#endif /* __MODULES_BLUETOOTH_HAL_BCM20706_INCLUDE_BT_BT_A2DP_SINK_H */
