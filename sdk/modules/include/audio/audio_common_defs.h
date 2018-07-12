/****************************************************************************
 * modules/include/audio/audio_common_defs.h
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

#ifndef __MODULES_INCLUDE_AUDIO_AUDIO_COMMON_DEFS_H
#define __MODULES_INCLUDE_AUDIO_AUDIO_COMMON_DEFS_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/* API Documents creater with Doxgen */

/**
 * @defgroup audioutils_audio_high_level_api Audio High Level API
 * @{
 *
 * @file       audio_comomn_defs.h
 * @brief      CXD5602 Audio Common Definitions
 * @author     CXD5602 Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/memory_manager/MemHandle.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* const for baseband */
/** @name Constant parameters */
/** @{ */

#define AS_VOLUME_DAC            -20

/* TODO:Be removed in future */

#define AS_AC_CODEC_VOL_DAC      -20

/** @} */

/* ---------------------------------*/

/** @name Fix values of audio parameter */
/** @{ */

/**  Mic channel max */

#define AS_MIC_CHANNEL_MAX        8


/** Keep setting for #InitMicGainParam.mic_gain */

#define AS_MICGAIN_HOLD           215

/** Keep setting for #SetVolumeParam.input1_db,
 *                   #SetVolumeParam.input2_db,
 *                   #SetVolumeParam.master_db
 */

#define AS_VOLUME_HOLD            255

/** Mute setting for above parameters */

#define AS_VOLUME_MUTE            -1025

/** Keep setting for #SetBeepParam.beep_vol*/

#define AS_BEEP_VOL_HOLD          255

/** Keep setting for #SetBeepParam.beep_freq */

#define AS_BEEP_FREQ_HOLD         0

/** Mute setting for above parameters */

#define AS_MICGAIN_MUTE           -7855

/** @} */

/** @name Codec type */
/** @{ */

/** MP3 */

#define AS_CODECTYPE_MP3    0

/** WAV */

#define AS_CODECTYPE_WAV    1

/** AAC */

#define AS_CODECTYPE_AAC    2

/** OPUS */

#define AS_CODECTYPE_OPUS   3

/** MEDIA Packet */

#define AS_CODECTYPE_MEDIA  4

/** Linear PCM */

#define AS_CODECTYPE_LPCM  5

/** @} */

/** @name Bit length */
/** @{ */

/** 16bit */

#define AS_BITLENGTH_16  16

/** 24bit */

#define AS_BITLENGTH_24  24

/** @} */

/** @name Channel number */
/** @{ */

/** MONO (1ch) */

#define AS_CHANNEL_MONO    1

/** STEREO (2ch) */

#define AS_CHANNEL_STEREO  2

/** 4ch */

#define AS_CHANNEL_4CH     4

/** 6ch */

#define AS_CHANNEL_6CH     6

/** 8ch */

#define AS_CHANNEL_8CH     8

/** @} */

/** @name Sampling rate */
/** @{ */

/* Auto */

#define AS_SAMPLINGRATE_AUTO    0

/** 8kHz */

#define AS_SAMPLINGRATE_8000    8000

/** 11.025kHz */

#define AS_SAMPLINGRATE_11025   11025

/** 12kHz */

#define AS_SAMPLINGRATE_12000   12000

/** 16kHz */

#define AS_SAMPLINGRATE_16000   16000

/** 22.05kHz */

#define AS_SAMPLINGRATE_22050   22050

/** 24kHz */

#define AS_SAMPLINGRATE_24000   24000

/** 32kHz */

#define AS_SAMPLINGRATE_32000   32000

/** 44.1kHz */

#define AS_SAMPLINGRATE_44100   44100

/** 48kHz */

#define AS_SAMPLINGRATE_48000   48000

/** 64kHz */

#define AS_SAMPLINGRATE_64000   64000

/** 88.2kHz */

#define AS_SAMPLINGRATE_88200   88200

/** 96kHz */

#define AS_SAMPLINGRATE_96000   96000

/** 128kHz */

#define AS_SAMPLINGRATE_128000  128000

/** 176.4kHz */

#define AS_SAMPLINGRATE_176400  176400

/** 192kHz */

#define AS_SAMPLINGRATE_192000  192000

/** @} */

/** @name Bit rate */
/** @{ */

/** 8kbps */

#define AS_BITRATE_8000    8000

/** 16kbps */

#define AS_BITRATE_16000   16000

/** 24kbps */

#define AS_BITRATE_24000   24000

/** 32kbps */

#define AS_BITRATE_32000   32000

/** 40kbps */

#define AS_BITRATE_40000   40000

/** 48kbps */

#define AS_BITRATE_48000   48000

/** 56kbps */

#define AS_BITRATE_56000   56000

/** 64kbps */

#define AS_BITRATE_64000   64000

/** 80kbps */

#define AS_BITRATE_80000   80000

/** 96kbps */

#define AS_BITRATE_96000   96000

/** 112kbps */

#define AS_BITRATE_112000  112000

/** 128kbps */

#define AS_BITRATE_128000  128000

/** 144kbps */

#define AS_BITRATE_144000  144000

/** 160kbps */

#define AS_BITRATE_160000  160000

/** 192kbps */

#define AS_BITRATE_192000  192000

/** 224kbps */

#define AS_BITRATE_224000  224000

/** 256kbps */

#define AS_BITRATE_256000  256000

/** 320kbps */

#define AS_BITRATE_320000  320000

/** 384kbps */

#define AS_BITRATE_384000  384000

/** 448kbps */

#define AS_BITRATE_448000  448000

/** 510kbps */

#define AS_BITRATE_510000  510000

/** @} */

/**< DSP path length */

#define AS_AUDIO_DSP_PATH_LEN 24 

/****************************************************************************
 * Public Types
 ****************************************************************************/
typedef void (*PcmProcDoneCallback)(int32_t identifier, bool is_end);

/** PCM data parameter */

typedef struct
{
  /*! \brief [in] Data identifier, use anyway */

  int32_t identifier;

  /*! \brief [in] Callback for any pcm data processing */

  PcmProcDoneCallback callback;

  /*! \brief [in] Memory handle for output PCM data */

  MemMgrLite::MemHandle mh;

  /*! \brief [in] Sample number of PCM data */

  uint32_t sample;

  /*! \brief [in] Byte size of input data */

  uint32_t size;

  /*! \brief [in] True: Last data of current session. */

  bool is_end;

  /*! \brief [in] True: Valid PCM frame. */

  bool is_valid;

  /*! \brief [in] Data bit length */

  uint8_t bit_length;

} AsPcmDataParam;

#endif /* __MODULES_INCLUDE_AUDIO_AUDIO_COMMON_DEFS_H */

