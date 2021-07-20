/****************************************************************************
 * examples/digital_filter/wav.c
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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

#include <stdio.h>
#include <stdint.h>
#include "wav.h"

/* #define DEBUG_PRINT printf */
#define DEBUG_PRINT(...)

/****************************************************************************
 * Private Data type
 ****************************************************************************/

union wav_header_content
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/** open_input_wavfile() */

int open_input_wavfile(wav_instance_t *wav, const char *file_name)
{
  union wav_header_content hdr;
  
  wav->fp = fopen(file_name, "rb");
  if (wav->fp == NULL)
    {
      return -1;
    }
  
  fread(hdr.riff_chunk_id, 1, 4, wav->fp);
  DEBUG_PRINT("chunk_id = %c%c%c%c\n",
      hdr.riff_chunk_id[0],
      hdr.riff_chunk_id[1],
      hdr.riff_chunk_id[2],
      hdr.riff_chunk_id[3]);

  fread(&hdr.riff_chunk_size, 4, 1, wav->fp);
  DEBUG_PRINT("chunk_size = %d\n", hdr.riff_chunk_size);

  fread(hdr.riff_form_type, 1, 4, wav->fp);
  DEBUG_PRINT("form_type = %c%c%c%c\n",
      hdr.riff_form_type[0],
      hdr.riff_form_type[1],
      hdr.riff_form_type[2],
      hdr.riff_form_type[3]);

  fread(hdr.fmt_chunk_id, 1, 4, wav->fp);
  DEBUG_PRINT("fmt_chunk_id = %c%c%c%c\n",
      hdr.fmt_chunk_id[0],
      hdr.fmt_chunk_id[1],
      hdr.fmt_chunk_id[2],
      hdr.fmt_chunk_id[3]);

  fread(&hdr.fmt_chunk_size, 4, 1, wav->fp);
  DEBUG_PRINT("fmt_chunk_size = %d\n", hdr.fmt_chunk_size);

  fread(&hdr.fmt_wave_format_type, 2, 1, wav->fp);
  DEBUG_PRINT("fmt_wave_format_type = %d\n", hdr.fmt_wave_format_type);

  fread(&hdr.fmt_channel, 2, 1, wav->fp);
  DEBUG_PRINT("channels = %d\n", hdr.fmt_channel);

  fread(&hdr.fmt_samples_per_sec, 4, 1, wav->fp);
  DEBUG_PRINT("samples_per_sec = %d\n", hdr.fmt_samples_per_sec);
  wav->fs = hdr.fmt_samples_per_sec;

  fread(&hdr.fmt_bytes_per_sec, 4, 1, wav->fp);
  DEBUG_PRINT("bytes_per_sec = %d\n", hdr.fmt_bytes_per_sec);

  fread(&hdr.fmt_block_size, 2, 1, wav->fp);
  DEBUG_PRINT("block_size = %d\n", hdr.fmt_block_size);

  fread(&hdr.fmt_bits_per_sample, 2, 1, wav->fp);
  DEBUG_PRINT("bit per sample = %d\n", hdr.fmt_bits_per_sample);
  wav->bits = hdr.fmt_bits_per_sample;

  fread(hdr.data_chunk_id, 1, 4, wav->fp);
  DEBUG_PRINT("data_chunk_id = %c%c%c%c\n",
      hdr.data_chunk_id[0],
      hdr.data_chunk_id[1],
      hdr.data_chunk_id[2],
      hdr.data_chunk_id[3]);

  fread(&hdr.data_chunk_size, 4, 1, wav->fp);
  DEBUG_PRINT("chunk_size=%d\n", hdr.data_chunk_size);
  wav->length = hdr.data_chunk_size / 2;

  wav->read_len = 0;

  return 0;
}

/** open_output_wavfile() */

int open_output_wavfile(wav_instance_t *wav, const char *file_name)
{
  union wav_header_content hdr;
  
  wav->fp = fopen(file_name, "wb");
  if (wav->fp == NULL)
    {
      return -1;
    }

  hdr.riff_chunk_id[0] = 'R';
  hdr.riff_chunk_id[1] = 'I';
  hdr.riff_chunk_id[2] = 'F';
  hdr.riff_chunk_id[3] = 'F';
  fwrite(hdr.riff_chunk_id, 1, 4, wav->fp);

  hdr.riff_chunk_size = 36 + wav->length * 2;
  fwrite(&hdr.riff_chunk_size, 4, 1, wav->fp);

  hdr.riff_form_type[0] = 'W';
  hdr.riff_form_type[1] = 'A';
  hdr.riff_form_type[2] = 'V';
  hdr.riff_form_type[3] = 'E';
  fwrite(hdr.riff_form_type, 1, 4, wav->fp);
  
  hdr.fmt_chunk_id[0] = 'f';
  hdr.fmt_chunk_id[1] = 'm';
  hdr.fmt_chunk_id[2] = 't';
  hdr.fmt_chunk_id[3] = ' ';
  fwrite(hdr.fmt_chunk_id, 1, 4, wav->fp);

  hdr.fmt_chunk_size = 16;
  fwrite(&hdr.fmt_chunk_size, 4, 1, wav->fp);

  hdr.fmt_wave_format_type = 1;
  fwrite(&hdr.fmt_wave_format_type, 2, 1, wav->fp);

  hdr.fmt_channel = 1;
  fwrite(&hdr.fmt_channel, 2, 1, wav->fp);

  hdr.fmt_samples_per_sec = wav->fs;
  fwrite(&hdr.fmt_samples_per_sec, 4, 1, wav->fp);

  hdr.fmt_bytes_per_sec = wav->fs * wav->bits / 8;
  fwrite(&hdr.fmt_bytes_per_sec, 4, 1, wav->fp);

  hdr.fmt_block_size = wav->bits / 8;
  fwrite(&hdr.fmt_block_size, 2, 1, wav->fp);

  hdr.fmt_bits_per_sample = wav->bits;
  fwrite(&hdr.fmt_bits_per_sample, 2, 1, wav->fp);
  
  hdr.data_chunk_id[0] = 'd';
  hdr.data_chunk_id[1] = 'a';
  hdr.data_chunk_id[2] = 't';
  hdr.data_chunk_id[3] = 'a';
  fwrite(hdr.data_chunk_id, 1, 4, wav->fp);

  hdr.data_chunk_size = wav->length * 2;
  fwrite(&hdr.data_chunk_size, 4, 1, wav->fp);

  return 0;
}

/** close_wavfile() */

void close_wavfile(wav_instance_t *wav)
{
  fclose(wav->fp);
}

/** read_wavdata() */

int read_wavdata(wav_instance_t *wav, float *data, int len)
{
  int i;
  int16_t tmp_data;

  for (i = 0; wav->read_len < wav->length && i < len; i++, wav->read_len++)
    {

      /* PCM data is stored as little endian in WAV format */

      fread(&tmp_data, 2, 1, wav->fp);
      data[i] = (float)tmp_data;
    }
  
  return i;
}

/** read_wavdata16() */

int read_wavdata16(wav_instance_t *wav, int16_t *data, int len)
{
  int i;
  int16_t tmp_data;

  for (i = 0; wav->read_len < wav->length && i < len; i++, wav->read_len++)
    {

      /* PCM data is stored as little endian in WAV format */

      fread(&tmp_data, 2, 1, wav->fp);
      data[i] = tmp_data;
    }
  
  return i;
}

/** write_wavdata() */

void write_wavdata(wav_instance_t *wav, float *data, int len)
{
  int i;
  int16_t tmp_data;

  for (i = 0; i < len; i++)
    {
      /* Clip data */

      if (data[i] > 32767.0)
        {
          data[i] = 32767.0;
        }
      else if (data[i] < -32768.0)
        {
          data[i] = -32768.0;
        }

      tmp_data = (short)data[i];
      fwrite(&tmp_data, 2, 1, wav->fp);
    }
}

/** write_wavdata16() */

void write_wavdata16(wav_instance_t *wav, int16_t *data, int len)
{
  int i;

  for (i = 0; i < len; i++)
    {
      fwrite(&data[i], 2, 1, wav->fp);
    }
}
