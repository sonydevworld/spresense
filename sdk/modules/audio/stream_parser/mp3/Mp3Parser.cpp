/****************************************************************************
 * modules/audio/stream_parser/mp3/Mp3Parser.cpp
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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common/Mp3Parser.h"

/*--------------------------------------------------------------------------*/
static inline
  Mp3ParserReturnValueOfFile peekbuffer_mp3parser(MP3PARSER_Handle *ptr_hndl,
                                                  uint8_t *ptr_read_buff,
                                                  uint32_t read_size)
{
  CMN_SimpleFifoPeekHandle pPeekHandle;
  size_t size =
    CMN_SimpleFifoPeekWithOffset(ptr_hndl->src.simple_fifo_handler,
                                 &pPeekHandle,
                                 read_size,
                                 ptr_hndl->current_offset);
  if (!size)
    {
      return Mp3ParserReturnFileAccesError;
    }
  size = CMN_SimpleFifoCopyFromPeekHandle(&pPeekHandle,
                                          (void *)ptr_read_buff,
                                          read_size);
  if (!size)
    {
      return Mp3ParserReturnFileAccesError;
    }
  return Mp3ParserReturnFileFavorable;
}

/*--------------------------------------------------------------------------*/
static inline
  Mp3ParserReturnValueOfFile pollbuffer_mp3parser(MP3PARSER_Handle *ptr_hndl,
                                                  uint8_t *ptr_read_buff)
{
  size_t size = 0;
  if (ptr_hndl->current_offset <= MP3PARSER_LOCAL_POLL_BUFFERSIZE)
    {
      size = CMN_SimpleFifoPoll(ptr_hndl->src.simple_fifo_handler,
                                ptr_read_buff,
                                ptr_hndl->current_offset);
      if (!size)
        {
          return Mp3ParserReturnFileAccesError;
        }
    }
  else
    {
      uint32_t i = 0;
      for (i = 0;
            i < (ptr_hndl->current_offset - MP3PARSER_LOCAL_POLL_BUFFERSIZE);
              i += MP3PARSER_LOCAL_POLL_BUFFERSIZE)
        {
          size = CMN_SimpleFifoPoll(ptr_hndl->src.simple_fifo_handler,
                                    ptr_read_buff,
                                    MP3PARSER_LOCAL_POLL_BUFFERSIZE);
          if (!size)
            {
              return Mp3ParserReturnFileAccesError;
            }
          ptr_read_buff += MP3PARSER_LOCAL_POLL_BUFFERSIZE;
        }
      uint32_t remainder = ptr_hndl->current_offset - i;
      if (remainder)
        {
          size = CMN_SimpleFifoPoll(ptr_hndl->src.simple_fifo_handler,
                                    ptr_read_buff,
                                    remainder);
          if (!size)
            {
              return Mp3ParserReturnFileAccesError;
            }
        }
    }
  return Mp3ParserReturnFileFavorable;
}

/*--------------------------------------------------------------------------*/
static inline
  Mp3ParserReturnValueOfFile skipbuffer_mp3parser(MP3PARSER_Handle *ptr_hndl)
{
  size_t size = 0;
  size_t occupied_size =
    CMN_SimpleFifoGetOccupiedSize(ptr_hndl->src.simple_fifo_handler);
  if (ptr_hndl->current_offset > occupied_size)
    {
      ptr_hndl->current_offset = occupied_size;
    }
      size = CMN_SimpleFifoPoll(ptr_hndl->src.simple_fifo_handler,
                                NULL,
                                ptr_hndl->current_offset);
  if (!size)
    {
      return Mp3ParserReturnFileAccesError;
    }

  return Mp3ParserReturnFileFavorable;
}

/*--------------------------------------------------------------------------*/
static Mp3ParserReturnValueOfSyncSearch
  get_offset_mp3parser_search_sync(Mp3ParserLocalInfo *ptr_info)
{
  bool    exist_syncword = false;
  uint8_t *ptr_check = NULL;

  ptr_info->found_offset = 0;

  if (ptr_info->max_search_byte <= MP3PARSER_SYNCWORD_LENGTH)
    {
      return Mp3ParserReturnNoSyncword;
    }

  uint32_t i = 0;
  for (i = 0, ptr_check = ptr_info->ptr_start;
        i < (ptr_info->max_search_byte - MP3PARSER_SYNCWORD_LENGTH);
          i++, ptr_check++)
    {
      /* Check 1st byte. */

      if ((*ptr_check & MP3PARSER_SYNCWORD_1) == MP3PARSER_SYNCWORD_1)
        {
          /* When 1st byte matches, 2nd byte is checked. */

          if ((*(ptr_check + 1) & MP3PARSER_SYNCWORD_2) ==
               MP3PARSER_SYNCWORD_2)
            {
              /* When the 2nd byte matches, check the information
               * in the header.
               */

              ptr_info->uhd.copy_byte[0] = *ptr_check;
              ptr_info->uhd.copy_byte[1] = *(ptr_check + 1);
              ptr_info->uhd.copy_byte[2] = *(ptr_check + 2);
              ptr_info->uhd.copy_byte[3] = *(ptr_check + 3);

              /*  Check data integrity. */

              if (MP3PARSER_GET_LAYER(ptr_info->uhd.copy_byte[1]) ==
                   Mp3ParserLayerReserved)
                {
                  /* As layer is reserved, continue syncword search. */

                  continue;
                }
              if (MP3PARSER_GET_FS(ptr_info->uhd.copy_byte[2]) ==
                   MP3PARSER_FS_RESERVED)
                {
                  /* As sampling_frequency is reserved,
                   * continue syncword search.
                   */

                  continue;
                }
              if ((MP3PARSER_GET_BR(ptr_info->uhd.copy_byte[2]) ==
                   MP3PARSER_BITRATE_FREE) ||
                   (MP3PARSER_GET_BR(ptr_info->uhd.copy_byte[2]) ==
                   MP3PARSER_BITRATE_UNUSED))
                {
                  /* As bitrate_index is free or unused,
                   * continue syncword search.
                   */

                  continue;
                }
              if (MP3PARSER_GET_PRIVATE(ptr_info->uhd.copy_byte[2]) ==
                   MP3PARSER_PRIVATEBIT_ISOUSED)
                {
                  /* Since unusable private_bit is used,
                   * continue syncword search.
                   */

                  continue;
                }
              if (MP3PARSER_GET_EMPHAS(ptr_info->uhd.copy_byte[3]) ==
                   MP3PARSER_EMPHASIS_RESERVED)
                {
                  /* As emphasis is reserved, continue syncword search. */

                  continue;
                }

              /* If you come this far, you are certified as a syncword for
               * mp3 for the time being.
               * Set the offset from the beginning.
               */

             ptr_info->found_offset =
                (uint32_t)(ptr_check - ptr_info->ptr_start);
              exist_syncword = true;
              break;
            }
        }
    }

  if (!exist_syncword)
    {
      /* Check the last 3 bytes, but you can not check the header information
       * even if you hit it.
       * Just to be sure, reset ptr_check.
       */

      ptr_check = (ptr_info-> ptr_start + (ptr_info->max_search_byte - MP3PARSER_SYNCWORD_LENGTH));
      if (((*ptr_check & MP3PARSER_SYNCWORD_1) == MP3PARSER_SYNCWORD_1) &&
           ((*(ptr_check + 1) & MP3PARSER_SYNCWORD_2) == MP3PARSER_SYNCWORD_2))
        {
          /* Specified byte There may be a sync word near the end.
           * Set the offset from the beginning.
           */

          ptr_info->found_offset =
            (uint32_t)(ptr_check - ptr_info->ptr_start);
          return Mp3ParserReturnPendding;
        }
      if (((*(ptr_check + 1) & MP3PARSER_SYNCWORD_1) ==
           MP3PARSER_SYNCWORD_1) &&
             ((*(ptr_check + 2) & MP3PARSER_SYNCWORD_2) ==
               MP3PARSER_SYNCWORD_2))
        {
          /* Specified byte There may be a sync word near the end.
           * Set the offset from the beginning.
           */

          ptr_info->found_offset =
            (uint32_t)((ptr_check + 1) - ptr_info->ptr_start);
          return Mp3ParserReturnPendding;
        }
      if (((*(ptr_check + 2) & MP3PARSER_SYNCWORD_1) == MP3PARSER_SYNCWORD_1))
        {
          /* Specified byte There may be a sync word near the end.
           * Set the offset from the beginning.
           */

          ptr_info->found_offset =
            (uint32_t)((ptr_check + 2) - ptr_info->ptr_start);
          return Mp3ParserReturnPendding;
        }
      return Mp3ParserReturnNoSyncword;
    }

  return Mp3ParserReturnFoundSyncword;
}

/*--------------------------------------------------------------------------*/
static Mp3ParserReturnValueOfSyncSearch
  loopbuffer_mp3parser_search_sync(MP3PARSER_Handle *ptr_hndl,
                                   Mp3ParserLocalInfo *ptr_info,
                                   uint8_t local_buff[])
{
  Mp3ParserReturnValueOfSyncSearch status = Mp3ParserReturnNoSyncword;

  ptr_info->max_search_byte = MP3PARSER_LOCAL_READFILE_BUFFERSIZE;
  ptr_info->search_offset   = ptr_hndl->current_offset;

  do
    {
      if (peekbuffer_mp3parser(ptr_hndl,
                               &local_buff[0],
                               MP3PARSER_LOCAL_READFILE_BUFFERSIZE) !=
                               Mp3ParserReturnFileFavorable)
        {
          return Mp3ParserReturnNoSyncword;
        }

      /* Syncword search processing. */
 
      status = get_offset_mp3parser_search_sync(ptr_info);
      if (status == Mp3ParserReturnFoundSyncword)
        {
          ptr_info->found_offset +=
            (ptr_info->search_offset - ptr_hndl->current_offset);
          return Mp3ParserReturnFoundSyncword;
        }
      if (status == Mp3ParserReturnPendding)
        {
          /* Set current position to syncword position. */

          ptr_info->search_offset += ptr_info->found_offset;
          ptr_hndl->current_offset = ptr_info->search_offset;
        }
      else
        {
          /* If a syncword can not be found, move by the buffer size. */

          ptr_info->search_offset += MP3PARSER_LOCAL_READFILE_BUFFERSIZE;
          ptr_hndl->current_offset = ptr_info->search_offset;
        }
    }
  while(status != Mp3ParserReturnFoundSyncword);

  return status;
}

/*--------------------------------------------------------------------------*/
static Mp3ParserReturnValueOfSyncSearch
  readdata_mp3parser_search_two_of_sync(MP3PARSER_Handle *ptr_hndl,
                                        Mp3ParserLocalInfo *ptr_info,
                                        uint8_t local_buff[])
{
  Mp3ParserReturnValueOfSyncSearch offset_syncword =
    Mp3ParserReturnNoSyncword;

  /* Search for the first syncword. */

  offset_syncword =
    loopbuffer_mp3parser_search_sync(ptr_hndl, ptr_info, &local_buff[0]);

  if (offset_syncword == Mp3ParserReturnFoundSyncword)
    {
      /* Set offset position of syncword. */

      ptr_info->sync_offset_1 =
        (ptr_hndl->current_offset + ptr_info->found_offset);

      /* Calculate frame length from header. */

      uint8_t copy_byte1 = ptr_info->uhd.copy_byte[1];
      uint8_t copy_byte2 = ptr_info->uhd.copy_byte[2];
      ptr_info->frame_length_1 =
        MP3PARSER_CALC_FRAME_SIZE(MP3PARSER_GET_ID(copy_byte1),
                                  MP3PARSER_GET_LAYER(copy_byte1),
                                  MP3PARSER_GET_BR(copy_byte2),
                                  MP3PARSER_GET_FS(copy_byte2),
                                  MP3PARSER_GET_PADDING(copy_byte2));
      if (ptr_info->sync_offset_1 != 0)
        {
          ptr_hndl->current_offset = ptr_info->sync_offset_1;
          skipbuffer_mp3parser(ptr_hndl);
        }
      offset_syncword = Mp3ParserReturnFound1stOnly;
      return offset_syncword;
    }
  else
    {
      /* Initialization when the first one is not detected. */

      ptr_info->sync_offset_1  = 0;
      ptr_info->frame_length_1 = 0;
      ptr_info->sync_offset_2  = 0;
      ptr_info->frame_length_2 = 0;
      return offset_syncword;
    }
}

/*--------------------------------------------------------------------------*/
static Mp3ParserReturnValueOfSyncSearch
  mp3parser_parse(MP3PARSER_Handle *ptr_hndl,
                  Mp3ParserLocalInfo *ptr_info)
{
  uint8_t local_buff[MP3PARSER_LOCAL_READFILE_BUFFERSIZE];
  Mp3ParserReturnValueOfSyncSearch status = Mp3ParserReturnNoSyncword;

  /* Search for the first syncword from your current location. */

  ptr_info->ptr_start = &local_buff[0];
  ptr_info->search_offset = 0;
  /*  */
  do
    {
      /* File reading and sync word search processing. */

      status = readdata_mp3parser_search_two_of_sync(ptr_hndl,
                                                     ptr_info,
                                                     &local_buff[0]);
    }
  while(status == Mp3ParserReturnSearchContinue);

  return status;
}

/*--------------------------------------------------------------------------*/
static void mp3parser_check_id3v2tag(MP3PARSER_Handle *ptr_hndl,
                                     Mp3ParserLocalInfo *ptr_info,
                                     uint8_t local_buff[])
{
  /* Check ID3v2 tag. */

  if ((local_buff[Mp3ParserID3v2HeadIndexID1] == MP3PARSER_ID3V2_ID1) &&
       (local_buff[Mp3ParserID3v2HeadIndexID2] == MP3PARSER_ID3V2_ID2) &&
         (local_buff[Mp3ParserID3v2HeadIndexID3] == MP3PARSER_ID3V2_ID3))
    {
      /* Skip detection of ID3v2 tag. */

      ptr_hndl->current_offset =
        MP3PARSER_ID3v2_GET_LENGTH(local_buff[Mp3ParserID3v2HeadIndexLen1],
                                   local_buff[Mp3ParserID3v2HeadIndexLen2],
                                   local_buff[Mp3ParserID3v2HeadIndexLen3],
                                   local_buff[Mp3ParserID3v2HeadIndexLen4]);
      ptr_hndl->current_offset += Mp3ParserID3v2HeaderLength;
    }
}

/*--------------------------------------------------------------------------*/
static void mp3parser_check_id3v1tag(MP3PARSER_Handle *ptr_hndl,
                                     Mp3ParserLocalInfo *ptr_info,
                                     uint8_t local_buff[])
{
  /* Check ID3v1 tag. */

  if ((local_buff[Mp3ParserID3v1HeadIndexID1] == MP3PARSER_ID3V1_ID1) &&
       (local_buff[Mp3ParserID3v1HeadIndexID2] == MP3PARSER_ID3V1_ID2) &&
         (local_buff[Mp3ParserID3v1HeadIndexID3] == MP3PARSER_ID3V1_ID3))
    {
      if (local_buff[Mp3ParserID3v1HeadIndexID4] != MP3PARSER_ID3V1_ID4)
        {
          /* Skip detection of ID3v1.0,1.1 tag. */

          ptr_hndl->current_offset = MP3PARSER_ID3v1_FIXED_LENGTH;
        }
      else
        {
          /* Skip detection of ID3v1.2 tag. */

          ptr_hndl->current_offset = MP3PARSER_ID3v1_2_FIXED_LENGTH;
        }
    }
}

/*--------------------------------------------------------------------------*/
Mp3ParserReturnValueOfFile
  mp3parser_buffer_check_tag(MP3PARSER_Handle *ptr_hndl,
                             Mp3ParserLocalInfo *ptr_info)
{
  Mp3ParserReturnValueOfFile status = Mp3ParserReturnFileFavorable;
  ptr_hndl->current_offset             = 0;
  ptr_hndl->counter_of_extracted_frame = 0;

  /* Read the file. */

  uint8_t local_buff[MP3PARSER_LOCAL_READFILE_BUFFERSIZE];
  status = peekbuffer_mp3parser(ptr_hndl,
                                &local_buff[0],
                                MP3PARSER_LOCAL_READFILE_BUFFERSIZE);
  if (status != Mp3ParserReturnFileFavorable)
    {
      return status;
    }

  /* Check ID3v2 tag. */

  mp3parser_check_id3v2tag(ptr_hndl, ptr_info, &local_buff[0]);
  if (ptr_hndl->current_offset)
    {
      status = skipbuffer_mp3parser(ptr_hndl);
      if (status != Mp3ParserReturnFileFavorable)
        {
          return status;
        }
      ptr_hndl->current_offset = 0;
    }

  return Mp3ParserReturnFileFavorable;
}

/*--------------------------------------------------------------------------*/
Mp3ParserReturnValueOfFile
  mp3parser_buffer_check_tag_v1(MP3PARSER_Handle *ptr_hndl,
                                Mp3ParserLocalInfo *ptr_info)
{
  Mp3ParserReturnValueOfFile status = Mp3ParserReturnFileFavorable;
  ptr_hndl->current_offset             = 0;
  ptr_hndl->counter_of_extracted_frame = 0;

  /* Read the file. */

  uint8_t local_buff[MP3PARSER_LOCAL_READFILE_BUFFERSIZE];
  status = peekbuffer_mp3parser(ptr_hndl,
                                &local_buff[0],
                                MP3PARSER_LOCAL_READFILE_BUFFERSIZE);
  if (status != Mp3ParserReturnFileFavorable)
    {
      return status;
    }

  /* Check ID3v1 tag. */

  mp3parser_check_id3v1tag(ptr_hndl, ptr_info, &local_buff[0]);
  if (ptr_hndl->current_offset)
    {
      status = skipbuffer_mp3parser(ptr_hndl);
      if (status != Mp3ParserReturnFileFavorable)
        {
          return status;
        }
      ptr_hndl->current_offset = 0;
    }

  return Mp3ParserReturnFileFavorable;
}

/*--------------------------------------------------------------------------*/
uint32_t mp3parser_extract_frame(MP3PARSER_Handle *ptr_hndl,
                                 Mp3ParserLocalInfo *ptr_info,
                                 uint8_t* out_buffer)
{
  uint32_t extract_frame_length = ptr_info->frame_length_1;

  /* (Clipping mode setting = up to the next syncword), the frame length
   * is calculated.
   */

  if (ptr_hndl->extraction_mode == Mp3ParserExtractFrameAndTrailingPadding)
    {
      if (ptr_info->sync_offset_2)
        {
          /* When detecting the next sync word,
           * set the frame length between two sync words.
           */

          extract_frame_length =
            (ptr_info->sync_offset_2 - ptr_info->sync_offset_1);
        }
      else
        {
          /* If there is no next sync word, recalculate frame length. */

          if ((ptr_info->frame_length_1 + ptr_hndl->search_max_2nd_sync) <
               (ptr_hndl->size_of_src - ptr_info->sync_offset_1))
            {
              /* (Calculated frame length + next sync word search upper limit)
               * to the frame length.
               */

              extract_frame_length =
                (ptr_info->frame_length_1 + ptr_hndl->search_max_2nd_sync);
            }
          else
            {
              /* If "next syncword search limit" is added,
               * if the total buffer length is exceeded,
               * the "length until the last buffer" is set to the
               * frame length.
               */

              extract_frame_length =
                (ptr_hndl->size_of_src - ptr_info->sync_offset_1);
            }
        }
    }

  if (out_buffer)
    {
      /* Cut out one frame. */

      Mp3ParserReturnValueOfFile status;
      ptr_hndl->current_offset = ptr_info->frame_length_1;
      status = pollbuffer_mp3parser(ptr_hndl, out_buffer);
      ptr_hndl->current_offset = 0;
      if (status != Mp3ParserReturnFileFavorable)
        {
          extract_frame_length = 0;
        }
    }

  return extract_frame_length;
}

/*--------------------------------------------------------------------------*/
int32_t mp3parser_get_frameheader(MP3PARSER_Handle *ptr_hndl,
                                  Mp3ParserLocalInfo *ptr_info,
                                  uint8_t *ptr_local_buff)
{
  Mp3ParserReturnValueOfSyncSearch status = Mp3ParserReturnNoSyncword;

  uint32_t temp_current_offset;
  temp_current_offset = ptr_hndl->current_offset;
  ptr_hndl->current_offset = 0;

  ptr_info->ptr_start = ptr_local_buff;

  /* Syncword search processing. */

  if (peekbuffer_mp3parser(ptr_hndl,
                           ptr_local_buff,
                           MP3PARSER_LOCAL_READFILE_BUFFERSIZE) !=
       Mp3ParserReturnFileFavorable)
    {
      return MP3PARSER_NO_FRAME_HEADER;
    }

  /* Check ID3v2 tag. */

  mp3parser_check_id3v2tag(ptr_hndl, ptr_info, ptr_local_buff);

  status =
    loopbuffer_mp3parser_search_sync(ptr_hndl, ptr_info, ptr_local_buff);

  ptr_hndl->current_offset = temp_current_offset;

  if (status != Mp3ParserReturnFoundSyncword)
    {
      return MP3PARSER_NO_FRAME_HEADER;
    }

  return MP3PARSER_SUCCESS;
}

/*--------------------------------------------------------------------------*/
Mp3ParserReturnValueOfSyncSearch
  mp3parser_distribute_processing(MP3PARSER_Handle *ptr_hndl,
                                  Mp3ParserLocalInfo *ptr_info)
{
  Mp3ParserReturnValueOfSyncSearch status = Mp3ParserReturnNoSyncword;

  /* Check IDv1 tag. */

  if (mp3parser_buffer_check_tag_v1(ptr_hndl, ptr_info) !=
       Mp3ParserReturnFileFavorable)
    {
      return Mp3ParserReturnNoSyncword;
    }

  /* Check ID3v2 tag. */

  if (mp3parser_buffer_check_tag(ptr_hndl, ptr_info) !=
       Mp3ParserReturnFileFavorable)
    {
      return Mp3ParserReturnNoSyncword;
    }

  status = mp3parser_parse(ptr_hndl, ptr_info);

  return status;
}

/*--------------------------------------------------------------------------*/
int32_t  Mp3Parser_initialize(MP3PARSER_Handle *ptr_hndl,
                              CMN_SimpleFifoHandle *simple_fifo_handler,
                              MP3PARSER_Config *config)
{
  if ((!ptr_hndl) || (!simple_fifo_handler) || (!config))
    {
      return MP3PARSER_PARAMETER_ERROR;
    }

  ptr_hndl->pConfig = config;  /* Set other parameter information. */

  /* Set handle information. */

  ptr_hndl->src_type                = Mp3ParserSrcBuffer;
  ptr_hndl->src.simple_fifo_handler = simple_fifo_handler;
  ptr_hndl->current_offset          = MP3PARSER_DEFAULT_RAM_OFFSET;
  ptr_hndl->extraction_mode         = MP3PARSER_DEFAULT_EXTRACTION_MODE;

  return MP3PARSER_SUCCESS;
}

/*--------------------------------------------------------------------------*/
int32_t  Mp3Parser_pollSingleFrame(MP3PARSER_Handle *ptr_hndl,
                                   uint8_t *out_buffer,
                                   uint32_t out_buffer_size,
                                   uint32_t *out_frame_size,
                                   int32_t *ready_to_extract_frames)
{
  if ((!ptr_hndl) ||
       (!out_buffer) ||
         (!out_frame_size) ||
           (!ready_to_extract_frames))
    {
      return MP3PARSER_PARAMETER_ERROR;
    }

  Mp3ParserLocalInfo local_info; /* Temporary information for
                                  * library internal use. */

  /* Call distribution processing. */

  Mp3ParserReturnValueOfSyncSearch status =
    mp3parser_distribute_processing(ptr_hndl,
                                    (Mp3ParserLocalInfo *)&local_info);

  if ((status != Mp3ParserReturnFoundSyncword) &&
       (status != Mp3ParserReturnFound1stOnly))
    {
      return MP3PARSER_NO_FRAME_HEADER;
    }

  /* Compare the cutout frame length and acquisition size. */

  if (out_buffer_size < local_info.frame_length_1)
    {
      return MP3PARSER_NO_OUTPUT_REGION;
    }

  /* Frame cutting out process. */

  *out_frame_size =
    mp3parser_extract_frame(ptr_hndl,
                            (Mp3ParserLocalInfo *)&local_info,
                            out_buffer);
  if (*out_frame_size != 0)
    {
      *ready_to_extract_frames = MP3PARSER_NEXT_SYNC_FOUND;
      return MP3PARSER_SUCCESS;
    }
  else
    {
      return MP3PARSER_NO_FRAME_HEADER;
    }
}

/*--------------------------------------------------------------------------*/
int32_t  Mp3Parser_finalize(MP3PARSER_Handle *ptr_hndl)
{
  if (!ptr_hndl)
    {
      return MP3PARSER_PARAMETER_ERROR;
    }

  /* Clear handle information. */

  ptr_hndl->counter_of_extracted_frame = 0;
  ptr_hndl->src.simple_fifo_handler    = 0;
  ptr_hndl->src_type                   = Mp3ParserSrcBuffer;
  ptr_hndl->extraction_mode            = 0;
  ptr_hndl->search_max_1st_sync        = 0;
  ptr_hndl->search_max_2nd_sync        = 0;
  ptr_hndl->size_of_src                = 0;
  ptr_hndl->current_offset             = 0;

  return MP3PARSER_SUCCESS;
}

/*--------------------------------------------------------------------------*/
int32_t  Mp3Parser_getSamplingRate(MP3PARSER_Handle *ptr_hndl,
                                   uint32_t *ptr_sampling_rate)
{
  if (!ptr_hndl)
    {
      return MP3PARSER_PARAMETER_ERROR;
    }

  Mp3ParserLocalInfo local_info;
  uint8_t LocalBuff[MP3PARSER_LOCAL_READFILE_BUFFERSIZE];

  int32_t rst = mp3parser_get_frameheader(ptr_hndl,
                                          (Mp3ParserLocalInfo *)&local_info,
                                          (uint8_t *)&LocalBuff[0]);
  if (rst != MP3PARSER_SUCCESS)
    {
      return MP3PARSER_NO_FRAME_HEADER;
    }

  /* Get sampling rate.(convert index to value)*/

  uint32_t idx = 0;
  if (MP3PARSER_GET_ID(local_info.uhd.copy_byte[1]) == Mp3ParserMpeg1)
    {
      /* Versoin-1(MPEG-1). */

      idx = MP3PARSER_GET_FS(local_info.uhd.copy_byte[2]);
      *ptr_sampling_rate = mp3_parser_v1_sampling_frequency[idx];
    }
  else
    {
      /* Versoin-2(MPEG-2). */

      idx = MP3PARSER_GET_FS(local_info.uhd.copy_byte[2]);
      *ptr_sampling_rate = mp3_parser_v2_sampling_frequency[idx];
    }

  return MP3PARSER_SUCCESS;
}
