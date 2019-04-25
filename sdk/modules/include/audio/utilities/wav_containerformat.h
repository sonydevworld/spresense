/****************************************************************************
 * modules/include/audio/utilities/wav_containerformat.h
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

#ifndef MODULES_INCLUDE_AUDIO_UTILITIES_WAV_CONTAINERFORMAT_H
#define MODULES_INCLUDE_AUDIO_UTILITIES_WAV_CONTAINERFORMAT_H

#include "audio/utilities/wav_containerformat_common.h"

/** Format ID */

#define FORMAT_ID_PCM WAVE_FORMAT_PCM

/** For wave header */

#define FMT_CHUNK_SIZE    16

/** WAV container format (will generate) */

struct wav_header_s
{
  /*! \brief character "RIFF" */

  uint32_t riff;

  /*! \brief Whole size exclude "RIFF" */

  uint32_t total_size;

  /*! \brief character "WAVE" */

  uint32_t wave;

  /*! \brief character "fmt " */

  uint32_t fmt;

  /*! \brief fmt chunk size */

  uint32_t fmt_size;

  /*! \brief format type */

  uint16_t format;

  /*! \brief channel number */

  uint16_t channel;

  /*! \brief sampling rate */

  uint32_t rate;

  /*! \brief rate * block */

  uint32_t avgbyte;

  /*! \brief channels * bit / 8 */

  uint16_t block;

  /*! \brief bit length */

  uint16_t bit;

  /*! \brief character "data"             */

  uint32_t data;

  uint32_t data_size;
};
typedef struct wav_header_s WAVHEADER;

class WavContainerFormat
{
public:
  WavContainerFormat() :
    m_format_id(0),
    m_channel_number(0),
    m_sampling_rate(0),
    m_bitwidth(0)
    {}
  ~WavContainerFormat() {}

  /**
   * @brief Init function
   *
   * @details Set WAV container information
   *          A bitlength is fixed to 16bit
   *
   * @param[in] format_id:      WAV format type
   * @param[in] channel_number: Number of channels 
   * @param[in] sampling_rate:  Sampling rate
   */

  bool init(uint16_t  format_id,
            uint16_t  channel_number,
            uint32_t  sampling_rate);

  /**
   * @brief Init function (Bitwidth can be specified)
   *
   * @details Set WAV container information
   *
   * @param[in] format_id:      WAV format type
   * @param[in] channel_number: Number of channels 
   * @param[in] sampling_rate:  Sampling rate
   * @param[in] bitwidth:       Bit per sample 
   */

  bool init(uint16_t  format_id,
            uint16_t  channel_number,
            uint32_t  sampling_rate,
            uint8_t   bitwidth);

  /**
   * @brief Get WAV header
   *
   * @details Get WAV container header information.
   *          The Informations depends on parameters of "init()".
   *
   * @param[out] wav_header: WAV continer header
   * @param[in]  data_size:  Wav data size to set header parameter 
   */

  bool getHeader(WAVHEADER *wav_header,
                 uint32_t data_size);

private:
  uint16_t  m_format_id;
  uint16_t  m_channel_number;
  uint32_t  m_sampling_rate;
  uint8_t   m_bitwidth;
};

#endif /* MODULES_INCLUDE_AUDIO_UTILITIES_WAV_CONTAINERFORMAT_H */
