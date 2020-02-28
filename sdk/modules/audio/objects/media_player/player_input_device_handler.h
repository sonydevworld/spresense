/****************************************************************************
 * modules/audio/objects/media_player/player_input_device_handler.h
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

#ifndef __MODULES_AUDIO_OBJECTS_MEDIA_PLAYER_PLAYER_INPUT_DEVICE_HANDLER_H
#define __MODULES_AUDIO_OBJECTS_MEDIA_PLAYER_PLAYER_INPUT_DEVICE_HANDLER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "audio/audio_high_level_api.h"
#include "memutils/common_utils/common_assert.h"
#include "objects/stream_parser/input_data_mng_obj.h"
#include "wien2_common_defs.h"

#ifdef CONFIG_AUDIOUTILS_PLAYER_CODEC_MP3
#  include "objects/stream_parser/mp3_stream_mng.h"
#endif
#ifdef CONFIG_AUDIOUTILS_PLAYER_CODEC_PCM
#  include "objects/stream_parser/ram_lpcm_data_source.h"
#endif
#ifdef CONFIG_AUDIOUTILS_PLAYER_CODEC_AAC
#  include "objects/stream_parser/ram_aaclc_data_source.h"
#endif
#ifdef CONFIG_AUDIOUTILS_PLAYER_CODEC_OPUS
#  include "objects/stream_parser/ram_opus_data_source.h"
#endif

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

class PlayerInputDeviceHandler
{
public:
  PlayerInputDeviceHandler():
    m_es_sampling_rate(0),
    m_codec_type(InvalidCodecType)
    {}

  ~PlayerInputDeviceHandler() {}

  struct PlayerInHandle
    {
      union
      {
        AsPlayerInputDeviceHdlrForRAM* p_ram_device_handle;
      };
    };

  virtual bool initialize(PlayerInHandle* p_handle) = 0;
  virtual uint32_t setParam(const AsInitPlayerParam& param) = 0;
  virtual uint32_t start() = 0;
  virtual bool getEs(void* p_es, uint32_t* es_byte_size) = 0;
  virtual bool stop() = 0;

  uint32_t getSamplingRate()
    {
      return m_es_sampling_rate;
    }

  AudioCodec getCodecType()
    {
      return m_codec_type;
    }

  uint16_t getSampleNumPerFrame()
    {
      switch (m_codec_type)
        {
          case AudCodecMP3:
            if (m_es_sampling_rate != AudioFs2ApuValue[AudFs_16000])
              {
                return SampleNumPerFrame[m_codec_type];
              }
            return Mp3SampleNumPerFrameWith16kHz;
          case AudCodecLPCM:
            return m_cur_wav_au_sample_num;
          case AudCodecSBC:
          case AudCodecAAC:
          case AudCodecWMA:
          case AudCodecLDAC:
          case AudCodecFLAC:
            return SampleNumPerFrame[m_codec_type];
          case AudCodecOPUS:
            if (m_es_sampling_rate == AudioFs2ApuValue[AudFs_16000])
              {
                return SampleNumPerFrame[m_codec_type]*2;
              }
            return SampleNumPerFrame[m_codec_type];
          default:
            D_ASSERT(0);
            break;
        }
    return 0;
    }
  uint8_t getChannelNum()
    {
      return m_ch_num;
    }
  uint8_t getBitLen()
    {
      return m_bit_len;
    }

protected:
  uint32_t    m_es_sampling_rate;
  AudioCodec  m_codec_type;
  uint16_t    m_cur_wav_au_sample_num;
  uint8_t     m_ch_num;
  uint8_t     m_init_player_api_codec_type;
  uint8_t     m_bit_len;
};

/*--------------------------------------------------------------------*/
class InputHandlerOfRAM : public PlayerInputDeviceHandler
{
public:
  InputHandlerOfRAM():
    PlayerInputDeviceHandler(),
    m_wav_au_size(0),
    m_p_es_source_hdl(NULL),
    m_notification_read_es_size(0)
    {
      m_codec_type = AudCodecLPCM;
    }

  ~InputHandlerOfRAM() {}

  virtual bool initialize(PlayerInHandle* p_handle);
  virtual uint32_t setParam(const AsInitPlayerParam& param);
  virtual uint32_t start();
  virtual bool getEs(void* p_es, uint32_t* es_byte_size);
  virtual bool stop();

private:
  uint32_t                m_wav_au_size;
  InputDataManagerObject *m_p_es_source_hdl;
  uint32_t                m_notification_read_es_size;

  AsPlayerInputDeviceHdlrForRAM m_in_device_handler;
#ifdef CONFIG_AUDIOUTILS_PLAYER_CODEC_MP3
  Mp3StreamMng  m_ram_mp3_es_source;
#endif
#ifdef CONFIG_AUDIOUTILS_PLAYER_CODEC_PCM
  RawLpcmDataSource m_raw_wav_es_source;
#endif
#ifdef CONFIG_AUDIOUTILS_PLAYER_CODEC_AAC
  RamAACLCDataSource m_ram_aaclc_es_source;
#endif
#ifdef CONFIG_AUDIOUTILS_PLAYER_CODEC_OPUS
  RamOpusDataSource m_ram_opus_es_source;
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

__WIEN2_END_NAMESPACE

#endif /* __MODULES_AUDIO_OBJECTS_MEDIA_PLAYER_PLAYER_INPUT_DEVICE_HANDLER_H */

