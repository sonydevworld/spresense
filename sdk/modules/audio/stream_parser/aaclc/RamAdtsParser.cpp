/****************************************************************************
 * modules/audio/stream_parser/aaclc/RamAdtsParser.cpp
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

#include "common/RamAdtsParser.h"
#include "common/RamAdtsParser_Common.h"

static uint8_t poll_buff[PARSER_LOCAL_POLL_BUFFERSIZE];

/*--------------------------------------------------------------------------*/
static int32_t adtsparser_skip_data(AdtsHandle *pHandle, uint8_t *pReadData)
{
  size_t size = 0;

  size_t occupied_size =
    CMN_SimpleFifoGetOccupiedSize(pHandle->pSimpleFifoHandler);
  if (pHandle->parse_size > occupied_size)
    {
      pHandle->parse_size = occupied_size;
    }
  if (pHandle->parse_size <= PARSER_LOCAL_POLL_BUFFERSIZE)
    {
      size = CMN_SimpleFifoPoll(pHandle->pSimpleFifoHandler,
                                pReadData,
                                pHandle->parse_size);
      if (!size)
        {
          return AdtsParserConnotDataAccess;
        }
    }
  else
    {
      uint32_t i = 0;
      for (i = 0;
           i < (pHandle->parse_size - PARSER_LOCAL_POLL_BUFFERSIZE);
           i += PARSER_LOCAL_POLL_BUFFERSIZE)
        {
          size = CMN_SimpleFifoPoll(pHandle->pSimpleFifoHandler,
                                    pReadData,
                                    PARSER_LOCAL_POLL_BUFFERSIZE);
          if (!size)
            {
              return AdtsParserConnotDataAccess;
            }
        }
      uint32_t remainder = pHandle->parse_size - i;
      if (remainder)
        {
          size = CMN_SimpleFifoPoll(pHandle->pSimpleFifoHandler,
                                    pReadData,
                                    remainder);
          if (!size)
            {
              return AdtsParserConnotDataAccess;
            }
        }
    }
  pHandle->parse_size = 0;

  return AdtsParserNormal;
}

/*--------------------------------------------------------------------------*/
static int32_t adtsparser_pool_data(AdtsHandle *pHandle, int8_t *pReadData)
{
  size_t size = 0;
  size_t occupied_size =
    CMN_SimpleFifoGetOccupiedSize(pHandle->pSimpleFifoHandler);
  if (pHandle->parse_size > occupied_size)
    {
      return AdtsParserConnotDataAccess;
    }

  size = CMN_SimpleFifoPoll(pHandle->pSimpleFifoHandler,
                            pReadData,
                            pHandle->parse_size);
  if (!size)
    {
      return AdtsParserConnotDataAccess;
    }
  pHandle->parse_size = 0;

  return AdtsParserNormal;
}

/*--------------------------------------------------------------------------*/
static int32_t adtsparser_peek_data(AdtsHandle *pHandle,
                                    uint8_t *pReadData,
                                    uint32_t peekSize)
{
  CMN_SimpleFifoPeekHandle pPeekHandle;

  size_t size = CMN_SimpleFifoPeekWithOffset(pHandle->pSimpleFifoHandler,
                                             &pPeekHandle,
                                             peekSize,
                                             pHandle->current_pos);
  if (!size)
    {
      return AdtsParserConnotDataAccess;
    }
  size =
    CMN_SimpleFifoCopyFromPeekHandle(&pPeekHandle, (void *)pReadData, size);
  if (!size)
    {
      return AdtsParserConnotDataAccess;
    }
  pHandle->parse_size = size;

  return AdtsParserNormal;
}

/*--------------------------------------------------------------------------*/
static int32_t adtsparser_syncword_search(uint8_t *pReadData,
                                          uint8_t *syncword_idx)
{
  for (int i = 0; i < 2; i++)
    {
      if (ADTS_CHECK_SYNCWORD(*(pReadData+(i)), *(pReadData+(i + 1))) ==
           ADTS_OK)
        {
          /* Because it is conceivable that a coincident sync word matches,
           * check the following data.
           */

          uint8_t consistency_check;
          consistency_check = *(pReadData+(i + 1));
          if (((consistency_check & 0x0F) == 1) ||
               ((consistency_check & 0x0F) == 0))
            {
              *syncword_idx = i;
              return AdtsParserNormal;
            }
        }
    }
  return AdtsParserCannotGetHeader;
}

/*--------------------------------------------------------------------------*/
int32_t AdtsParser_Initialize(AdtsHandle *pHandle,
                              CMN_SimpleFifoHandle *pSimpleFifoHandler,
                              AdtsParserErrorDetail *uipErrDetail)
{
  int32_t rc = ADTS_ERR;

  if (uipErrDetail)
    {
      *uipErrDetail = AdtsParserAbnormalArg;
    }
  if ((pHandle) && (pSimpleFifoHandler) && (uipErrDetail))
    {
      pHandle->pSimpleFifoHandler = pSimpleFifoHandler;
      pHandle->current_pos = 0;
      pHandle->search_pos  = 0;
      pHandle->parse_size  = 0;

      *uipErrDetail = AdtsParserNormal;
      rc = ADTS_OK;
    }
  return rc;
}


/*--------------------------------------------------------------------------*/
int32_t AdtsParser_ReadFrame(AdtsHandle *pHandle,
                             int8_t *pBuff,
                             uint32_t *pSize,
                             uint16_t *usResult,
                             AdtsParserErrorDetail *uipErrDetail)
{
  int32_t rc = ADTS_ERR;

  if (usResult)
    {
      *usResult = HDR_OK;
    }

  if (uipErrDetail)
    {
      *uipErrDetail = AdtsParserAbnormalArg;
    }

  if ((pHandle) && (pBuff) && (pSize) && (usResult) && (uipErrDetail))
    {
      size_t occupied_size = 0;
      pHandle->current_pos = 0;
      pHandle->search_pos  = 0;
      while (1)
        {
          int32_t rst = adtsparser_peek_data(pHandle,
                                             poll_buff,
                                             ADTSPARSER_SYNCWORD_SEARCH_SIZE);
          if (rst != AdtsParserNormal)
            {
              occupied_size =
                CMN_SimpleFifoGetOccupiedSize(pHandle->pSimpleFifoHandler);
              pHandle->parse_size = occupied_size;
              adtsparser_skip_data(pHandle, poll_buff);
              *uipErrDetail = AdtsParserConnotDataAccess;
              return rc;
            }
          pHandle->search_pos++;
          pHandle->current_pos = pHandle->search_pos;

          uint8_t syncword_idx;
          if (adtsparser_syncword_search(poll_buff, &syncword_idx) ==
               AdtsParserNormal)
            {
              if (syncword_idx == 0)
                {
                  pHandle->search_pos = (pHandle->search_pos - 1);
                }
              break;
            }
        }
      if (pHandle->search_pos != 0)
        {
          pHandle->parse_size = pHandle->search_pos;
          if (adtsparser_skip_data(pHandle, poll_buff) != AdtsParserNormal)
            {
              *uipErrDetail = AdtsParserConnotDataAccess;
              return rc;
            }
        }

      /* Read header information. */

      pHandle->current_pos = 0;
      if (adtsparser_peek_data(pHandle, poll_buff, ADTS_HEADER_SIZE) !=
           AdtsParserNormal)
        {
          occupied_size =
            CMN_SimpleFifoGetOccupiedSize(pHandle->pSimpleFifoHandler);
          pHandle->parse_size = occupied_size;
          adtsparser_skip_data(pHandle, poll_buff);
          *uipErrDetail = AdtsParserConnotDataAccess;
          return rc;
        }


      /* Check syncword. */

      if (ADTS_CHECK_SYNCWORD(*poll_buff, *(poll_buff + 1)) == ADTS_OK)
        {
          *uipErrDetail = AdtsParserAbnormalHeader;

          /* Checking the profile. */

          if ((*(poll_buff + 2) & ADTS_MASK_PROFILE) != ADTS_PROFILE_AACLC)
            {
              *usResult |= HDR_PROFILE_NG;
            }

          /* Check sampling rate. */

          if (ADTS_GET_SAMPLING_RATE(*(poll_buff + 2)) == 0)
            {
              *usResult |= HDR_SAMLERATE_NG;
            }

          /* Extract frame size.(including header) */

          uint32_t frame_size =
            ADTS_GET_FRAMELENGTH(*(poll_buff + 3),
                                 *(poll_buff + 4),
                                 *(poll_buff + 5));

          if (frame_size <= *pSize)
            {
              *uipErrDetail = AdtsParserNormal;
              pHandle->parse_size = frame_size;

              /* Reading a frame.(including header) */

              if (adtsparser_pool_data(pHandle, pBuff) == AdtsParserNormal)
                {
                  pHandle->parse_size = 0;
                  *pSize = frame_size;
                  rc = ADTS_OK;
                }
              else
                {
                  occupied_size =
                    CMN_SimpleFifoGetOccupiedSize(pHandle->pSimpleFifoHandler);
                  pHandle->parse_size = occupied_size;
                  adtsparser_skip_data(pHandle, poll_buff);
                  *uipErrDetail = AdtsParserConnotDataAccess;
                  *usResult |= HDR_FRAMESIZE_NG;
                  *pSize = 0;
                }
            }
          else
            {
              *usResult |= HDR_FRAMESIZE_NG;
              *uipErrDetail = AdtsParserShortageBuffer;
            }
        }
      else
        {
          *usResult |= HDR_SYNCWORD_NG;
          *uipErrDetail = AdtsParserCannotGetHeader;
        }
    }
  return rc;
}

/*--------------------------------------------------------------------------*/
int32_t AdtsParser_Finalize(AdtsHandle *pHandle,
                            AdtsParserErrorDetail *uipErrDetail)
{
  int32_t rc = ADTS_ERR;

  if (uipErrDetail)
    {
      *uipErrDetail = AdtsParserAbnormalArg;
    }
  if ((pHandle) && (uipErrDetail))
    {
      pHandle->pSimpleFifoHandler = 0;
      pHandle->current_pos = 0;
      pHandle->search_pos = 0;
      pHandle->parse_size = 0;

      *uipErrDetail = AdtsParserNormal;
      rc = ADTS_OK;
    }
  return rc;
}

/*--------------------------------------------------------------------------*/
int32_t AdtsParser_GetSamplingRate(AdtsHandle *pHandle,
                                   uint32_t *pSmplingRate,
                                   AdtsParserErrorDetail *uipErrDetail)
{
  int32_t rc = ADTS_ERR;

  if (pSmplingRate)
    {
      *pSmplingRate = 0;
    }
  if (uipErrDetail)
    {
      *uipErrDetail = AdtsParserAbnormalArg;
    }

  if ((pHandle) && (pSmplingRate) && (uipErrDetail))
    {
      pHandle->current_pos = 0;
      pHandle->search_pos  = 0;
      while (1)
        {
          if (adtsparser_peek_data(pHandle, poll_buff, 3) != AdtsParserNormal)
            {
              *uipErrDetail = AdtsParserConnotDataAccess;
              return rc;
            }
          pHandle->search_pos++;
          pHandle->current_pos = pHandle->search_pos;

          uint8_t syncword_idx;
          if (adtsparser_syncword_search(poll_buff, &syncword_idx) ==
               AdtsParserNormal)
            {
              if (syncword_idx == 0)
                {
                  pHandle->search_pos = (pHandle->search_pos - 1);
                }
              break;
            }
        }

      /* Read header information. */

      pHandle->current_pos = pHandle->search_pos;
      if (adtsparser_peek_data(pHandle, poll_buff, ADTS_HEADER_SIZE) !=
           AdtsParserNormal)
        {
          *uipErrDetail = AdtsParserConnotDataAccess;
          return rc;
        }

      /* Check syncword. */

      if (ADTS_CHECK_SYNCWORD(*poll_buff, *(poll_buff + 1)) == ADTS_OK)
        {
          *pSmplingRate = ADTS_GET_SAMPLING_RATE(*(poll_buff + 2));
          *uipErrDetail = AdtsParserNormal;
          rc = ADTS_OK;
        }
      else
        {
          *uipErrDetail = AdtsParserAbnormalHeader;
        }
    }

  return rc;
}
