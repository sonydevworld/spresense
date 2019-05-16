/****************************************************************************
 * modules/audio/container_format_lib/wav_containerformat_parser.cpp
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
#include <stdio.h>
#include <stdlib.h>
#include "audio/utilities/wav_containerformat_parser.h"

/*--------------------------------------------------------------------------*/
handel_wav_parser WavContainerFormatParser::parseChunk(const char* file_path,
                                                       fmt_chunk_t* fmt)
{
  handel_wav_parser_t* wav_parser =
    (handel_wav_parser_t*)malloc(sizeof(handel_wav_parser_t));
  if (wav_parser == NULL)
    {
      return NULL;
    }
  memset((void *)wav_parser, 0, sizeof(handel_wav_parser_t));

  FILE *fd = fopen(file_path, "r");
  if (fd == 0)
    {
      free((void *)wav_parser);
      return NULL;
    }
  setvbuf(fd, NULL, _IOLBF, STDIO_BUFFER_SIZE);

  int ret;
  riff_chunk_t riff_chunk;
  ret = fread((void*)&riff_chunk, 1, sizeof(riff_chunk_t), fd);
  if (ret < 0 || sizeof(riff_chunk_t) != ret)
    {
      fclose(fd);
      free((void *)wav_parser);
      return NULL;
    }
  if (riff_chunk.chunk.chunk_id == CHUNKID_RIFF)
    {
      uint32_t offset = sizeof(riff_chunk_t);
      wav_parser->file_size = riff_chunk.chunk.size;
      chunk_t chunk;
      while (1)
        {
          ret = fread((void*)&chunk, 1, sizeof(chunk_t), fd);
          if (ret < 0 || sizeof(chunk_t) != ret)
             {
               fclose(fd);
               free((void *)wav_parser);
               return NULL;
             }

          wav_parser->chunk_list.chunk[wav_parser->chunk_list.cnt].chunk_id =
            chunk.chunk_id;
          wav_parser->chunk_list.chunk[wav_parser->chunk_list.cnt].size =
            chunk.size;
          wav_parser->chunk_offset[wav_parser->chunk_list.cnt] =
            offset + sizeof(chunk_t);
          wav_parser->chunk_list.cnt++;
          offset += sizeof(chunk_t);
          switch (chunk.chunk_id)
            {
              case SUBCHUNKID_FMT:
                {
                  fmt_chunk_t  fmt_chunk;
                  ret = fread((void*)&fmt_chunk, 1, chunk.size, fd);
                  if (ret < 0 || ret != chunk.size)
                    {
                      fclose(fd);
                      free((void *)wav_parser);
                      return NULL;
                    }
                  offset += chunk.size;
                  memcpy(fmt, &fmt_chunk, sizeof(fmt_chunk_t));
                  if ((int32_t)sizeof(fmt_chunk_t) > chunk.size)
                    {
                      fmt->extended_size = 0;
                    }
                }
                break;

              case SUBCHUNKID_DATA:
                wav_parser->data_offset = offset;
                wav_parser->data_size = chunk.size;
                wav_parser->cur_offset = offset;
                wav_parser->read_size = chunk.size;
                wav_parser->fd = fd;
                return (handel_wav_parser)wav_parser;

              default:
                int8_t data;
                for (int32_t i = 0; i < chunk.size; i++)
                 {
                   ret = fread((void*)&data, 1, 1, fd);
                 }
                offset += chunk.size;
                break;
            }
        }
    }

  fclose(fd);
  free((void *)wav_parser);

  return NULL;
}

/*--------------------------------------------------------------------------*/
bool WavContainerFormatParser::getChunkList(handel_wav_parser handel,
                                            chunk_list_t* list)
{
  if (handel == NULL)
    {
      return false;
    }
  handel_wav_parser_t* wav_parser = (handel_wav_parser_t *)handel;

  list->cnt = wav_parser->chunk_list.cnt;
  for (uint8_t i = 0; i < wav_parser->chunk_list.cnt; i++)
    {
      list->chunk[i].chunk_id = wav_parser->chunk_list.chunk[i].chunk_id;
      list->chunk[i].size = wav_parser->chunk_list.chunk[i].size;
    }
  return true;
}

/*--------------------------------------------------------------------------*/
bool WavContainerFormatParser::getChunk(handel_wav_parser handel,
                                        uint32_t          chunk_id,
                                        int8_t*           buffer)
{
  if (handel == NULL)
    {
      return false;
    }
  handel_wav_parser_t* wav_parser = (handel_wav_parser_t *)handel;

  for (uint8_t i = 0; i < wav_parser->chunk_list.cnt; i++)
    {
      if (wav_parser->chunk_list.chunk[i].chunk_id == chunk_id)
        {
          int ret;
          fseek(wav_parser->fd, wav_parser->chunk_offset[i], 0);
          ret = fread(buffer, 1, wav_parser->chunk_list.chunk[i].size,
                      wav_parser-> fd);
          fseek(wav_parser->fd, wav_parser->cur_offset, 0);
          if (ret < 0 || ret != wav_parser->chunk_list.chunk[i].size)
            {
              return false;
            }
          return true;
        }
    }

  return false;
}

/*--------------------------------------------------------------------------*/
int32_t WavContainerFormatParser::getDataChunk(handel_wav_parser handel,
                                               uint16_t          format,
                                               int8_t*           buffer,
                                               uint32_t          size)
{
  if (handel == NULL)
    {
      return false;
    }
  handel_wav_parser_t* wav_parser = (handel_wav_parser_t *)handel;

  int ret = 0;
  switch (format)
    {
      case WAVE_FORMAT_PCM:
        {
          uint32_t read_size = 0;
          if (wav_parser->read_size == 0)
            {
              break;
            }
          if (wav_parser->read_size >= size)
            {
              read_size = size;
            }
          else
            {
              read_size = wav_parser->read_size;
            }

          ret = fread(buffer, 1, read_size, wav_parser->fd);
          if (ret < 0)
            {
              return -1;
            }
          wav_parser->cur_offset += ret;
          wav_parser->read_size -= ret;
        }
        break;
      default:
        return -1;
    }
  return ret;
}

/*--------------------------------------------------------------------------*/
void WavContainerFormatParser::resetParser(handel_wav_parser handel)
{
  fclose(((handel_wav_parser_t *)handel)->fd);
  free(handel);
}
