/****************************************************************************
 * modules/audio/objects/media_player/player_input_device_handler.cpp
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "objects/media_player/player_input_device_handler.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "audio/audio_high_level_api.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LPCM_SAMPLE_HIGH   512
#define LPCM_SAMPLE_MID    640
#define LPCM_SAMPLE_LOW    320

#define BIT_TO_BYTE(bit_length) ( \
                                 ((bit_length) % 8 == 0) ? \
                                 (bit_length) / 8 : ((bit_length) / 8) + 1 \
                                )

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool InputHandlerOfRAM::initialize(PlayerInHandle* p_handle)
{
  /* Do error check first. */

  if (p_handle->p_ram_device_handle->simple_fifo_handler == NULL ||
      p_handle->p_ram_device_handle->callback_function == NULL)
    {
      return false;
    }

  m_in_device_handler.simple_fifo_handler         =
    p_handle->p_ram_device_handle->simple_fifo_handler;
  m_in_device_handler.callback_function           =
    p_handle->p_ram_device_handle->callback_function;
  m_in_device_handler.notification_threshold_size =
    p_handle->p_ram_device_handle->notification_threshold_size;

  return true;
}

/*--------------------------------------------------------------------*/
uint32_t InputHandlerOfRAM::setParam(const AsInitPlayerParam& param)
{
  switch (param.codec_type)
    {
#ifdef CONFIG_AUDIOUTILS_PLAYER_CODEC_MP3
      case AS_CODECTYPE_MP3:
        m_p_es_source_hdl = &m_ram_mp3_es_source;
        break;
#endif
#ifdef CONFIG_AUDIOUTILS_PLAYER_CODEC_PCM
      case AS_CODECTYPE_WAV:
        m_p_es_source_hdl = &m_raw_wav_es_source;
        break;
#endif
#ifdef CONFIG_AUDIOUTILS_PLAYER_CODEC_AAC
      case AS_CODECTYPE_AAC:
        m_p_es_source_hdl = &m_ram_aaclc_es_source;
        break;
      case AS_CODECTYPE_MEDIA:
        m_p_es_source_hdl = &m_ram_aaclc_es_source;
        break;
#endif
#ifdef CONFIG_AUDIOUTILS_PLAYER_CODEC_OPUS
      case AS_CODECTYPE_OPUS:
        m_p_es_source_hdl = &m_ram_opus_es_source;
        break;
#endif
      default:
        return AS_ECODE_COMMAND_PARAM_CODEC_TYPE;
    }

  m_init_player_api_codec_type = param.codec_type;
  if (param.codec_type != AS_CODECTYPE_MEDIA)
    {
      m_codec_type = static_cast<AudioCodec>(param.codec_type);
    }
  else
    {
      m_codec_type = AudCodecAAC;
      return AS_ECODE_OK;
    }

  switch (param.channel_number)
    {
      case AS_CHANNEL_MONO:
      case AS_CHANNEL_STEREO:
        break;
      default:
        return AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER;
    }
  m_ch_num = param.channel_number;

  switch (param.sampling_rate)
    {
      case AS_SAMPLINGRATE_AUTO:
        if (param.codec_type != AS_CODECTYPE_MP3 &&
            param.codec_type != AS_CODECTYPE_AAC)
          {
            return AS_ECODE_COMMAND_PARAM_SAMPLING_RATE;
          }
        break;
      case AS_SAMPLINGRATE_8000:
      case AS_SAMPLINGRATE_16000:
      case AS_SAMPLINGRATE_24000:
        m_cur_wav_au_sample_num = LPCM_SAMPLE_LOW;
        break;
      case AS_SAMPLINGRATE_32000:
      case AS_SAMPLINGRATE_44100:
      case AS_SAMPLINGRATE_48000:
        m_cur_wav_au_sample_num = LPCM_SAMPLE_MID;
        break;
      case AS_SAMPLINGRATE_64000:
      case AS_SAMPLINGRATE_88200:
      case AS_SAMPLINGRATE_96000:
      case AS_SAMPLINGRATE_176400:
      case AS_SAMPLINGRATE_192000:
        m_cur_wav_au_sample_num = LPCM_SAMPLE_HIGH;
        break;
      default:
        return AS_ECODE_COMMAND_PARAM_SAMPLING_RATE;
    }
  m_es_sampling_rate = param.sampling_rate;

  switch (param.bit_length)
    {
      case AS_BITLENGTH_16:
      case AS_BITLENGTH_24:
        if (m_codec_type == AudCodecLPCM)
          {
            m_wav_au_size = m_cur_wav_au_sample_num * m_ch_num *
                            BIT_TO_BYTE(param.bit_length);
          }
        break;
      default:
        return AS_ECODE_COMMAND_PARAM_BIT_LENGTH;
    }
  m_bit_len = param.bit_length;

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
uint32_t InputHandlerOfRAM::start()
{
  InitInputDataManagerParam init_param;
  size_t ret = CMN_SimpleFifoGetOccupiedSize((CMN_SimpleFifoHandle*)
    m_in_device_handler.simple_fifo_handler);
  if (!ret)
    {
      return AS_ECODE_SIMPLE_FIFO_UNDERFLOW;
    }

  m_notification_read_es_size = 0;

  init_param.p_simple_fifo_handler =
    m_in_device_handler.simple_fifo_handler;
  init_param.codec_type            = m_init_player_api_codec_type;
  init_param.in_sampling_rate      = m_es_sampling_rate;
  init_param.in_ch_num             = m_ch_num;
  if (!m_p_es_source_hdl->init(init_param))
    {
      return AS_ECODE_COMMAND_PARAM_INPUT_HANDLER;
    }

  uint32_t sampling_rate_value = 0;
  if (!m_p_es_source_hdl->getSamplingRate(&sampling_rate_value))
    {
      return AS_ECODE_COMMAND_PARAM_SAMPLING_RATE;
    }
  m_es_sampling_rate = sampling_rate_value;

  uint32_t ch_num = 0;
  if (!m_p_es_source_hdl->getChNum(&ch_num))
    {
      return AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER;
    }
  m_ch_num = ch_num;
  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool InputHandlerOfRAM::stop()
{
  m_p_es_source_hdl->finish();
  return true;
}

/*--------------------------------------------------------------------*/
bool InputHandlerOfRAM::getEs(void* p_es, uint32_t* es_byte_size)
{
  if (m_codec_type == AudCodecLPCM)
    {
      *es_byte_size = m_wav_au_size;
    }
  InputDataManagerObject::GetEsResult state =
    m_p_es_source_hdl->getEs(p_es, es_byte_size);

  switch (state)
    {
      case InputDataManagerObject::EsExist:
        {
          if (m_in_device_handler.notification_threshold_size == 0)
            {
              m_in_device_handler.callback_function(*es_byte_size);
            }
          else
            {
              m_notification_read_es_size += *es_byte_size;
              if (m_in_device_handler.notification_threshold_size <=
                m_notification_read_es_size)
                {
                  m_in_device_handler.callback_function(
                    m_notification_read_es_size);
                  m_notification_read_es_size = 0;
                }
            }
        }
    return true;
  case InputDataManagerObject::EsEnd:
  case InputDataManagerObject::EsNone:
    return false;
  default:
    break;
  }
  return false;
}

__WIEN2_END_NAMESPACE
