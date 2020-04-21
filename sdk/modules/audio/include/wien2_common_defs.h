/****************************************************************************
 * modules/audio/include/wien2_common_defs.h
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

#ifndef __MODULES_AUDIO_INCLUDE_WIEN2_COMMON_DEFS_H
#define __MODULES_AUDIO_INCLUDE_WIEN2_COMMON_DEFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/chip/audio.h>
#include "memutils/common_utils/common_types.h"

#define __WIEN2_BEGIN_NAMESPACE  namespace Wien2 {
#define __WIEN2_END_NAMESPACE    }
#define __USING_WIEN2    using namespace Wien2;

#define __APU_BEGIN_NAMESPACE  namespace Apu {
#define __APU_END_NAMESPACE    }
#define __USING_APU    using namespace Apu;

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APU_COMMAND_QUEUE_SIZE  3  /* Apu command Queue size */

#define SRC_CUT_OFF        0.91f  /* SRC cut off frequency */
#define SRC_ATTENUATION    80     /* SRC filter attenuation [dB] */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/***   PCM format(Bit length)   ***/
enum audio_pcm_format_type_e
{
  AudPcmFormatInt24 = 0,  /**< 24bit PCM */
  AudPcmFormatInt20,      /**< 20bit PCM */
  AudPcmFormatInt18,      /**< 18bit PCM */	
  AudPcmFormatInt16,      /**< 16bit PCM */
  AudPcmFormatInt32,      /**< 32bit PCM */
  AudPcmFormatFloat32     /**< 32bit PCM (float type) */
};
typedef enum audio_pcm_format_type_e AudioPcmFormat;


enum AudioChannelFormat
{
  AudPackFormat = 0,
  Aud6ChannelFormat,
  Aud2ChannelFormat
};

enum AudioSamplingRate
{
  AUD_INVALID_FS = -1,
  AudFs_96000,
  AudFs_88200,
  AudFs_64000,
  AudFs_48000,
  AudFs_44100,
  AudFs_32000,
  AudFs_24000,
  AudFs_22050,
  AudFs_16000,
  AudFs_12000,
  AudFs_11025,
  AudFs_08000,
  AudFs_128000,
  AudFs_176400,
  AudFs_192000,
  AudFs_MAX
};

/* Definitions of channel construction of PCM */

enum AudioChannelConfig
{
  AUD_PCM_INVALID_CH_CONFIG = 0xffffffff,
  AUD_PCM_CH_CONFIG_1_1 = 0,  /* Dual mono */
  AUD_PCM_CH_CONFIG_1_0,      /* Mono */
  AUD_PCM_CH_CONFIG_2_0,      /* Stereo */
  AUD_PCM_CH_CONFIG_3_0,
  AUD_PCM_CH_CONFIG_2_1,
  AUD_PCM_CH_CONFIG_3_1,
  AUD_PCM_CH_CONFIG_2_2,
  AUD_PCM_CH_CONFIG_3_2,
  AUD_PCM_CH_CONFIG_3_2_LFE,  /* Depend on definition in Apu */
  AUD_PCM_CH_CONFIG_1_1_LFE,
  AUD_PCM_CH_CONFIG_1_0_LFE,
  AUD_PCM_CH_CONFIG_2_0_LFE,
  AUD_PCM_CH_CONFIG_3_0_LFE,
  AUD_PCM_CH_CONFIG_2_1_LFE,
  AUD_PCM_CH_CONFIG_3_1_LFE,
  AUD_PCM_CH_CONFIG_2_2_LFE,
  AUD_PCM_CH_CONFIG_NUM
};

/* Definitions of channel selection of mixer */

enum AudioMixerChSelect
{
  AudChSelThrough = 0,  /* Output ch equals to input ch */
  AudChSelLelf,         /* Select L ch */
  AudChSelRight,        /* Select R ch */
};

enum AudioCodec
{
  InvalidCodecType = 0xffffffff,
  AudCodecMP3 = 0,
  AudCodecLPCM,
  AudCodecAAC,
  AudCodecOPUS,
  AudCodecHEAAC,
  AudCodecMP2,
  AudCodecFLAC,
  AudCodecSBC,
  AudCodecLDAC,
  AudCodecWMA,
  AudCodecNoCodec,
  AudCodecNum
};

enum WaveMode
{
  InvalidWave = 0xffffffff,
  SinWave = 0,
  RectangularWave,
  WaveModeNum
};

struct BufferHeader
{
  uint32_t size;
  unsigned long *p_buffer;
};

enum AudioChannelNum
{
  NotChannels,
  MonoChannels,
  TwoChannels,
  ThreeChannels,
  FourChannels,
  FiveChannels,
  SixChannels
};

enum PcmByteEndian
{
  LittleEndian = 0,
  BigEndian = 1
};

/* SRC input-output bit length per a sample [Byte] */

enum AudioSrcPcmWordLen
{
  AudSrc16BitLen = 2,   /* 16bitPCM */
  AudSrc32BitLen = 4    /* 32bitPCM */
};

enum AudioBitRate
{
  InvalidBitRate = -1,
  AudBitRate8KBPS = 0,
  AudBitRate16KBPS,
  AudBitRate24KBPS,
  AudBitRate32KBPS,
  AudBitRate40KBPS,
  AudBitRate48KBPS,
  AudBitRate56KBPS,
  AudBitRate64KBPS,
  AudBitRate80KBPS,
  AudBitRate96KBPS,
  AudBitRate112KBPS,
  AudBitRate128KBPS,
  AudBitRate144KBPS,
  AudBitRate160KBPS,
  AudBitRate176KBPS,
  AudBitRate192KBPS,
  AudBitRate224KBPS,
  AudBitRate256KBPS,
  AudBitRate288KBPS,
  AudBitRate320KBPS,
  AudBitRate352KBPS,
  AudBitRate384KBPS,
  AudBitRate416KBPS,
  AudBitRate448KBPS,
  AudBitRate_MAX
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

const uint32_t AudioFs2ApuValue[] =
{
  96000,
  88200,
  64000,
  48000,
  44100,
  32000,
  24000,
  22050,
  16000,
  12000,
  11025,
  8000,
  128000,
  176400,
  192000
};

static const uint32_t SampleNumPerFrame[AudCodecNum] =
{
  1152,  /* AudCodecMP3 */
  768,   /* AudCodecLPCM */ /* Any integer in capable */
  1024,  /* AudCodecAAC */
  160,   /* AudCodecOPUS */
  2048,  /* AudCodecHEAAC (Not suppot) */
  0,     /* AudCodecMP2 (Not suppot) */
  576,   /* AudCodecFLAC (Not suppot)  */
  128*8, /* AudCodecSBC (Not suppot) */
  128,   /* AudCodecLDAC (Not suppo) */
  1024,  /* AudCodecWMA (Not suppot) */
  0      /* AudCodecNoCodec (Don't use) */
};


static const uint16_t Mp3SampleNumPerFrameWith16kHz = 576;

static const uint8_t AudioIOValidChannelNum[] =
{
  0,
  1,
  2,
  3,
  4,
  5,
  6
};

static const uint32_t AudioBitRate2ApuValue[] =
{
  8000,
  16000,
  24000,
  32000,
  40000,
  48000,
  56000,
  64000,
  80000,
  96000,
  112000,
  128000,
  144000,
  160000,
  176000,
  192000,
  224000,
  256000,
  288000,
  320000,
  352000,
  384000,
  416000,
  448000
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

inline AudioSamplingRate value2AudioFsDef(uint32_t value)
{
  switch(value)
    {
      case 96000:
        return AudFs_96000;
      case 88200:
        return AudFs_88200;
      case 64000:
        return AudFs_64000;
      case 48000:
        return AudFs_48000;
      case 44100:
        return AudFs_44100;
      case 32000:
        return AudFs_32000;
      case 24000:
        return AudFs_24000;
      case 22050:
        return AudFs_22050;
      case 16000:
        return AudFs_16000;
      case 12000:
        return AudFs_12000;
      case 11025:
        return AudFs_11025;
      case 8000:
        return AudFs_08000;
      case 128000:
        return AudFs_128000;
      case 176400:
        return AudFs_176400;
      case 192000:
        return AudFs_192000;
      default:
        break;
    }
  return AUD_INVALID_FS;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

__WIEN2_END_NAMESPACE

#endif /* #define __MODULES_AUDIO_INCLUDE_WIEN2_COMMON_DEFS_H */
