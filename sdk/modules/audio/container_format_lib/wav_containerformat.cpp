/****************************************************************************
 * modules/audio/container_format_lib/wav_containerformat.cpp
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

#include <string.h>
#include "audio/utilities/wav_containerformat.h"

/*--------------------------------------------------------------------------*/
bool WavContainerFormat::init(uint16_t format_id,
                              uint16_t channel_number,
                              uint32_t sampling_rate)
{
  return init(format_id, channel_number, sampling_rate, BIT_WIDTH_16);
}

/*--------------------------------------------------------------------------*/
bool WavContainerFormat::init(uint16_t format_id,
                              uint16_t channel_number,
                              uint32_t sampling_rate,
                              uint8_t bitwidth)
{
  switch (format_id)
    {
      case WAVE_FORMAT_PCM:
        break;
      default:
        return false;
    }
  m_format_id = format_id;

  switch (channel_number)
    {
      case CHANNEL_1CH:
      case CHANNEL_2CH:
      case CHANNEL_4CH:
      case CHANNEL_6CH:
      case CHANNEL_8CH:
        break;
      default:
        return false;
    }
  m_channel_number = channel_number;

  switch (sampling_rate)
    {
      case SAMPLINGRATE_8000:
      case SAMPLINGRATE_11025:
      case SAMPLINGRATE_12000:
      case SAMPLINGRATE_16000:
      case SAMPLINGRATE_22050:
      case SAMPLINGRATE_24000:
      case SAMPLINGRATE_32000:
      case SAMPLINGRATE_44100:
      case SAMPLINGRATE_48000:
      case SAMPLINGRATE_64000:
      case SAMPLINGRATE_88200:
      case SAMPLINGRATE_96000:
      case SAMPLINGRATE_128000:
      case SAMPLINGRATE_176400:
      case SAMPLINGRATE_192000:
        break;

      default:
        return false;
    }
  m_sampling_rate = sampling_rate;

  switch (bitwidth)
    {
      case BIT_WIDTH_16:
      case BIT_WIDTH_24:
      case BIT_WIDTH_32:
        break;

      default:
        return false;
    }

  m_bitwidth = bitwidth;

  return true;
}

/*--------------------------------------------------------------------------*/
bool WavContainerFormat::getHeader(WAVHEADER *wav_header, uint32_t data_size)
{
  if (wav_header == NULL)
    {
      return false;
    }

  wav_header->riff       = CHUNKID_RIFF;
  wav_header->wave       = FORMAT_WAVE;
  wav_header->fmt        = SUBCHUNKID_FMT;
  wav_header->data       = SUBCHUNKID_DATA;
  wav_header->fmt_size   = FMT_CHUNK_SIZE;
  wav_header->format     = WAVE_FORMAT_PCM;
  wav_header->channel    = m_channel_number;
  wav_header->rate       = m_sampling_rate;
  wav_header->avgbyte    = m_sampling_rate * m_channel_number * 2;
  wav_header->block      = m_channel_number * 2;
  wav_header->bit        = m_bitwidth;
  if (data_size == 0)
    {
      wav_header->total_size = 0;
      wav_header->data_size  = 0;
    }
  else
    {
      wav_header->total_size = data_size + sizeof(WAVHEADER) - 8;
      wav_header->data_size = data_size;
    }

  return true;
}
