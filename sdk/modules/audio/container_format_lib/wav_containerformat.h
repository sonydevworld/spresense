/****************************************************************************
 * modules/audio/container_format_lib/wav_containerformat.h
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

#ifndef MODULES_AUDIO_CONTAINER_FORMAT_LIB_WAV_CONTAINERFORMAT_H
#define MODULES_AUDIO_CONTAINER_FORMAT_LIB_WAV_CONTAINERFORMAT_H

/* Channel number */

#define CHANNEL_1CH  1  /* MONO   */
#define CHANNEL_2CH  2  /* STEREO */
#define CHANNEL_4CH  4
#define CHANNEL_6CH  6
#define CHANNEL_8CH  8

/* Sampling rate */

#define SAMPLINGRATE_8000    8000   /* 8kHz      */
#define SAMPLINGRATE_11025   11025  /* 11.025kHz */
#define SAMPLINGRATE_12000   12000  /* 12kHz     */
#define SAMPLINGRATE_16000   16000  /* 16kHz     */
#define SAMPLINGRATE_22050   22050  /* 2205kHz   */
#define SAMPLINGRATE_24000   24000  /* 24kHz     */
#define SAMPLINGRATE_32000   32000  /* 32kHz     */
#define SAMPLINGRATE_44100   44100  /* 44.1kHz   */
#define SAMPLINGRATE_48000   48000  /* 48kHz     */
#define SAMPLINGRATE_64000   64000  /* 64kHz     */
#define SAMPLINGRATE_88200   88200  /* 88.2kHz   */
#define SAMPLINGRATE_96000   96000  /* 96kHz     */
#define SAMPLINGRATE_128000  128000 /* 128kHz    */
#define SAMPLINGRATE_176400  176400 /* 176.4kHz  */
#define SAMPLINGRATE_192000  192000 /* 192kHz    */

/* Bit width */

#define BIT_WIDTH_16 16
#define BIT_WIDTH_24 24
#define BIT_WIDTH_32 32 

/* Format ID */

#define FORMAT_ID_PCM   0x0001  /* Linear PCM */

/* For wave header. */

#define CHUNKID_RIFF      "RIFF"
#define FORMAT_WAVE       "WAVE"
#define SUBCHUNKID_FMT    "fmt "
#define SUBCHUNKID_DATA   "data"
#define FMT_SIZE          0x10

struct wav_header_s
{
  uint8_t  riff[4];    /* "RIFF"             */
  uint32_t total_size;
  uint8_t  wave[4];    /* "WAVE"             */
  uint8_t  fmt[4];     /* "fmt "             */
  uint32_t fmt_size;   /* fmt chunk size     */
  uint16_t format;     /* format type        */
  uint16_t channel;    /* channel number     */
  uint32_t rate;       /* sampling rate      */
  uint32_t avgbyte;    /* rate * block       */
  uint16_t block;      /* channels * bit / 8 */
  uint16_t bit;        /* bit length         */
  uint8_t  data[4];    /* "data"             */
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
   * A bitlength is fixed to 16bit 
   */

  bool init(uint16_t  format_id,
            uint16_t  channel_number,
            uint32_t  sampling_rate);

  /* Init function */

  bool init(uint16_t  format_id,
            uint16_t  channel_number,
            uint32_t  sampling_rate,
            uint8_t   bitwidth);

  bool getHeader(WAVHEADER *wav_header,
           uint32_t data_size);
private:
  uint16_t  m_format_id;
  uint16_t  m_channel_number;
  uint32_t  m_sampling_rate;
  uint8_t   m_bitwidth;
};

#endif /* MODULES_AUDIO_CONTAINER_FORMAT_LIB_WAV_CONTAINERFORMAT_H */
