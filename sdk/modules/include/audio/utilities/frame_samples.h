/****************************************************************************
 * modules/include/audio/utilities/frame_samples.h
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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

#ifndef MODULES_INCLUDE_AUDIO_UTILITIES_FRAME_SAMPLES_H
#define MODULES_INCLUDE_AUDIO_UTILITIES_FRAME_SAMPLES_H

#include "audio/audio_common_defs.h"

/* When MP3 encode and fs is 16kHz, 22.05kHz, 24kHz, sample num of
 * 1au(access unit) is 1152/2 = 576 (It depend on MPEG2 compliant).
 * Therefore, at first, value is (#1)"CapSampleNumPerFrame[m_codec_type] / 2".
 * And sample num of captured and SRC filterd data is to be 576,
 * return ((#1) * 48000 / m_sampling_rate(Hz)).
 *
 * The process below is only for fs is 48kHz, 16kHz.
 * To correspontd to 32000Hz, 44100Hz..., need conversion process to 
 * sample num per 1au to be 1152.
 */


inline uint32_t getCapSampleNumPerFrame(uint8_t codec_type, uint32_t fs)
{
  const uint32_t CapSampleNumPerFrame[] =
  {
    1152,  /* MP3 */
    768,   /* WAV */ /* Any integer in capable */
    1024,  /* AAC */
    160,   /* OPUS */
    1024,  /* AAC */
    768,   /* LPCM */
  };

  if (codec_type > AS_CODECTYPE_LPCM)
    {
      return 0;
    }

  if (codec_type == AS_CODECTYPE_MP3 && fs < 32000)
    {
      return (CapSampleNumPerFrame[codec_type] / 2 * 48000 /
        fs);
    }
  else if (codec_type == AS_CODECTYPE_OPUS)
    {
      /* 20ms. */

      return ((fs / 50) * (48000 / fs));
    }
  else
    {
    }

  return CapSampleNumPerFrame[codec_type];
}

#endif /* MODULES_INCLUDE_AUDIO_UTILITIES_FRAME_SAMPLES_H */

