/****************************************************************************
 * modules/include/audio/utilities/audio_wav_containerformat.h
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

#ifndef MODULES_INCLUDE_AUDIO_UTILITIES_AUDIO_WAV_CONTAINERFORMAT_H
#define MODULES_INCLUDE_AUDIO_UTILITIES_AUDIO_WAV_CONTAINERFORMAT_H

#include "audio_wav_containerformat_common.h"

/* Format ID */

#define FORMAT_ID_PCM WAVE_FORMAT_PCM

/* For wave header. */

#define FMT_CHUNK_SIZE    16

struct wav_header_s
{
  uint32_t riff;       /* "RIFF"             */
  uint32_t total_size; /* Whole size exclude "RIFF" */
  uint32_t wave;       /* "WAVE"             */
  uint32_t fmt;        /* "fmt "             */
  uint32_t fmt_size;   /* fmt chunk size     */
  uint16_t format;     /* format type        */
  uint16_t channel;    /* channel number     */
  uint32_t rate;       /* sampling rate      */
  uint32_t avgbyte;    /* rate * block       */
  uint16_t block;      /* channels * bit / 8 */
  uint16_t bit;        /* bit length         */
  uint32_t data;       /* "data"             */
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

  /* Init function
   *
   *   A bitlength is fixed to 16bit
   *   Set WAV container information
   */

  bool init(uint16_t  format_id,
            uint16_t  channel_number,
            uint32_t  sampling_rate);

  /* Init function
   *
   *   Set WAV container information
   */

  bool init(uint16_t  format_id,
            uint16_t  channel_number,
            uint32_t  sampling_rate,
            uint8_t   bitwidth);

  /* Get WAV header
   *
   *   Get WAV container header information.
   *   Informations depends on parameters of "init()".
   */

  bool getHeader(WAVHEADER *wav_header,
                 uint32_t data_size);
private:
  uint16_t  m_format_id;
  uint16_t  m_channel_number;
  uint32_t  m_sampling_rate;
  uint8_t   m_bitwidth;
};

#endif /* MODULES_INCLUDE_AUDIO_UTILITIES_AUDIO_WAV_CONTAINERFORMAT_H */
