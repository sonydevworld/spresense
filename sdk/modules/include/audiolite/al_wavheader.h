/****************************************************************************
 * modules/include/audiolite/al_wavheader.h
 *
 *   Copyright 2023 Sony Semiconductor Solutions Corporation
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

#ifndef __INCLUDE_AUDIOLITE_WAVHEADER_H
#define __INCLUDE_AUDIOLITE_WAVHEADER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RIFF_ID "RIFF"
#define RIFF_FMT "WAVE"
#define FMTCNK_ID "fmt "
#define FMTCNK_SZ (16)
#define FMT_WAV   (1)
#define DATACNK_ID "data"

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

struct al_wavhdr
{
  uint8_t riff_chunk_id[4];
  uint32_t riff_chunk_size;
  uint8_t riff_form_type[4];
  uint8_t fmt_chunk_id[4];
  uint32_t fmt_chunk_size;
  uint16_t fmt_wave_format_type;
  uint16_t fmt_channel;
  uint32_t fmt_samples_per_sec;
  uint32_t fmt_bytes_per_sec;
  uint16_t fmt_block_size;
  uint16_t fmt_bits_per_sample;
  uint8_t data_chunk_id[4];
  uint32_t data_chunk_size;
};

#endif /* __INCLUDE_AUDIOLITE_WAVHEADER_H */
