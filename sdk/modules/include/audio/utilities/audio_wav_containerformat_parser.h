/****************************************************************************
 * modules/include/audio/utilities/audio_wav_containerformat_parser.h
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

#ifndef MODULES_INCLUDE_AUDIO_UTILITIES_AUDIO_WAV_CONTAINERFORMAT_PARSER_H
#define MODULES_INCLUDE_AUDIO_UTILITIES_AUDIO_WAV_CONTAINERFORMAT_PARSER_H

#include "audio_wav_containerformat_common.h"

typedef void* handel_wav_parser;

/* For line buffer mode. */

#define STDIO_BUFFER_SIZE 4096

#define MAX_CHUNK_LIST 128

struct chunk_s
{
  uint32_t chunk_id;
  int32_t  size;
};
typedef struct chunk_s chunk_t;

struct chunk_list_s
{
  uint8_t  cnt;
  chunk_t  chunk[MAX_CHUNK_LIST];
};
typedef struct chunk_list_s chunk_list_t;


struct riff_chunk_s
{
  struct chunk_s  chunk;
  uint32_t        type;
};
typedef struct riff_chunk_s riff_chunk_t;

struct fmt_chunk_s
{
  uint16_t  format;
  uint16_t  channel;
  uint32_t  rate;
  uint32_t  avgbyte;
  uint16_t  block;
  uint16_t  bit;
  uint16_t  extended_size;
};
typedef struct fmt_chunk_s fmt_chunk_t;

struct handel_wav_parser_s
{
  chunk_list_t  chunk_list;
  uint32_t      chunk_offset[MAX_CHUNK_LIST];
  uint32_t      data_offset;
  uint32_t      cur_offset;
  uint32_t      file_size;
  uint32_t      data_size;
  uint32_t      read_size;
  FILE          *fd;
};
typedef struct handel_wav_parser_s handel_wav_parser_t;


class WavContainerFormatParser
{
public:
  WavContainerFormatParser() {}
  ~WavContainerFormatParser() {}

  /*
   * Parse WAV container
   *
   *   Parse WAV container and return handle, and "fmt" chunk.
   *   The "fmt" chunk include sampling rate, bit length, ch num... and more.
   *
   */

  handel_wav_parser parseChunk(const char *file_path, fmt_chunk_t *fmt);

  /*
   * Get Chunk List
   *
   *   Get list of chunks which is included in designated WAV file.
   *
   */

  bool getChunkList(handel_wav_parser handle, chunk_list_t *list);

  /*
   * Get Chunk
   *
   *   Get chunk by chunk id.
   *
   */

  bool getChunk(handel_wav_parser handle, uint32_t chunk_id, int8_t *buffer);

  /*
   * Get Data Chunk
   *
   *   Get Data chunk.
   *
   */

  int32_t getDataChunk(handel_wav_parser handle, uint16_t format, int8_t *buffer, uint32_t size);

  /*
   * Reset Parser
   *
   *   Close WAV file and free internal memory area.
   *
   */

  void resetParser(handel_wav_parser handle);

private:
};

#endif /* MODULES_INCLUDE_AUDIO_UTILITIES_AUDIO_WAV_CONTAINERFORMAT_PARSER_H */
