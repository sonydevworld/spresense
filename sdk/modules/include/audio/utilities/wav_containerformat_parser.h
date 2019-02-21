/****************************************************************************
 * modules/include/audio/utilities/wav_containerformat_parser.h
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

#ifndef MODULES_INCLUDE_AUDIO_UTILITIES_WAV_CONTAINERFORMAT_PARSER_H
#define MODULES_INCLUDE_AUDIO_UTILITIES_WAV_CONTAINERFORMAT_PARSER_H

#include "audio/utilities/wav_containerformat_common.h"

typedef void* handel_wav_parser;

/* For line buffer mode. */

#define STDIO_BUFFER_SIZE 4096

#define MAX_CHUNK_LIST 128

/** Chunk information */

struct chunk_s
{
  /*! \brief Chunk ID */

  uint32_t chunk_id;

  /*! \brief Chunk size */

  int32_t  size;
};
typedef struct chunk_s chunk_t;

/** Chunk list */

struct chunk_list_s
{
  /*! \brief Number of chunks in a list */

  uint8_t  cnt;

  /*! \brief Chunk list (MAX num is MAX_CHUNK_LIST) */

  chunk_t  chunk[MAX_CHUNK_LIST];
};
typedef struct chunk_list_s chunk_list_t;

/** RIFF chunk structure */

struct riff_chunk_s
{
  struct chunk_s  chunk;
  uint32_t        type;
};
typedef struct riff_chunk_s riff_chunk_t;

/** FMT chunk structure */

struct fmt_chunk_s
{
  /*! \brief Number of chunks in a list */

  uint16_t  format;

  /*! \brief Number of channel */

  uint16_t  channel;

  /*! \brief Sampling rate (fs) */

  uint32_t  rate;

  /*! \brief Averate byte per second */

  uint32_t  avgbyte;

  /*! \brief block size */

  uint16_t  block;

  /*! \brief bit per sample */

  uint16_t  bit;

  /*! \brief extend area size */

  uint16_t  extended_size;
};
typedef struct fmt_chunk_s fmt_chunk_t;

/** Handle structure of the parser */

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

/** Class definition of the parser */

class WavContainerFormatParser
{
public:
  WavContainerFormatParser() {}
  ~WavContainerFormatParser() {}

  /**
   * @brief Parse WAV container
   *
   * @details Parse WAV container and return handle, and "fmt" chunk.\n
   *          The "fmt" chunk include sampling rate, bit length, ch num... and more.
   *
   * @param[in]  file_path: Path of Target WAV format file
   * @param[out] fmt:       Information of FMT chunk
   *
   * @retval handle of the parser
   */
  
  handel_wav_parser parseChunk(const char *file_path, fmt_chunk_t *fmt);
  
  /**
   * @brief Get Chunk List
   *
   * @details Get list of chunks which is included in designated WAV file.
   *
   * @param[in]  handle: Handle of the parser 
   * @param[out] list  : list of chunks in WAV format file 
   *
   * @retval result
   */
  
  bool getChunkList(handel_wav_parser handle, chunk_list_t *list);
  
  /**
   * @brief Get Chunk
   *
   * @details  Get chunk by chunk id.
   *
   * @param[in] handle:   Handle of the parser
   * @param[in] chunk_id: ID of require chunk 
   * @param[in] buffer:   Memory address which will store chunk data 
   *
   * @retval result
   */
  
  bool getChunk(handel_wav_parser handle, uint32_t chunk_id, int8_t *buffer);
  
  /**
   * @brief Get Data Chunk
   *
   * @details Get Data chunk.
   *
   * @param[in] handle: Handle of the parser
   * @param[in] format: WAV format(currently, support only WAVE_FORMAT_PCM)
   * @param[in] buffer: Memory address which will store data chunk data
   * @param[in] size:   Size of buffer
   *
   * @retval got size
   */
  
  int32_t getDataChunk(handel_wav_parser handle, uint16_t format, int8_t *buffer, uint32_t size);
  
  /**
   * @brief Reset Parser
   *
   * @details Close WAV file and free internal memory area.
   *
   * @param[in] handle: Handle of parser
   */
  
  void resetParser(handel_wav_parser handle);

private:
};

#endif /* MODULES_INCLUDE_AUDIO_UTILITIES_WAV_CONTAINERFORMAT_PARSER_H */
