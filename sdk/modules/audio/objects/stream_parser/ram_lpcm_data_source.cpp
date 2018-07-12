/****************************************************************************
 * modules/audio/objects/stream_parser/ram_lpcm_data_source.cpp
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "ram_lpcm_data_source.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

bool RawLpcmDataSource::init(const InitInputDataManagerParam &param)
{
  if (!checkSimpleFifoHandler(param))
    {
      return false;
    }
  setInitParam(param);
  return true;
}

InputDataManagerObject::GetEsResult
  RawLpcmDataSource::getEs(FAR void *es_buf, FAR uint32_t *es_size)
{
  size_t size = 0;
  if (!getOccupiedSize(&size))
    {
      return EsEnd;
    }
  uint32_t read_size = 0;
  if (size >= *es_size)
    {
      read_size = *es_size;
    }
  else
    {
      read_size = size;
    }
  size_t poll_size = 0;
  if (!simpleFifoPoll(es_buf, read_size, &poll_size))
    {
      return EsEnd;
    }
  *es_size = poll_size;
  return EsExist;
}

bool RawLpcmDataSource::finish()
{
  return true;
}

bool RawLpcmDataSource::getSamplingRate(FAR uint32_t* p_sampling_rate)
{
  if (!ParseChunk())
    {
      return false;
    }

  *p_sampling_rate = m_in_sampling_rate;
  return true;
}

bool RawLpcmDataSource::getChNum(FAR uint32_t* p_ch_num)
{
  *p_ch_num = m_ch_num;
  return true;
}

bool RawLpcmDataSource::getBitPerSample(FAR uint32_t *p_bit_per_sample)
{
  return false;
}

bool RawLpcmDataSource::ParseChunk(void)
{
  riff_chunk_t riff_chunk;
  if (!simpleFifoPeek(&riff_chunk, sizeof(riff_chunk_t)))
    {
      return false;
    }

  if (riff_chunk.chunk.chunk_id == CHUNKID_RIFF)
    {
      CMN_SimpleFifoPoll(p_simple_fifo_handler, NULL, sizeof(riff_chunk_t));

      size_t poll_size = 0;
      chunk_t chunk;
      uint8_t chunk_data[CHUNK_BUF_SIZE];
      bool rst = false;
      while (1)
        {
          rst = simpleFifoPoll(&chunk, sizeof(chunk_t), &poll_size);
          if (!rst || sizeof(chunk_t) != poll_size)
             {
               return false;
             }
          switch (chunk.chunk_id)
            {
              case SUBCHUNKID_FMT:
                {
                  fmt_chunk_t  fmt_chunk;
                  rst = simpleFifoPoll(&fmt_chunk, chunk.size, &poll_size);
                  if (!rst || chunk.size != (int32_t)poll_size)
                    {
                      return false;
                    }
                  m_in_sampling_rate = fmt_chunk.rate;
                  m_ch_num = fmt_chunk.channel;
                }
                break;

              case SUBCHUNKID_DATA:
               return true;

              case SUBCHUNKID_JUNK:
              case SUBCHUNKID_LIST:
              case SUBCHUNKID_ID3:
              case SUBCHUNKID_FACT:
              case SUBCHUNKID_PLST:
              case SUBCHUNKID_CUE:
              case SUBCHUNKID_LABL:
              case SUBCHUNKID_NOTE:
              case SUBCHUNKID_LTXT:
              case SUBCHUNKID_SMPL:
              case SUBCHUNKID_INST:
              case SUBCHUNKID_BEXT:
              case SUBCHUNKID_IXML:
              case SUBCHUNKID_QLTY:
              case SUBCHUNKID_MEXT:
              case SUBCHUNKID_LEVL:
              case SUBCHUNKID_LINK:
              case SUBCHUNKID_AXML:
              case SUBCHUNKID_CONT:
              default:
                if (chunk.size > CHUNK_BUF_SIZE)
                  {
                    rst = simpleFifoPoll(chunk_data, CHUNK_BUF_SIZE, &poll_size);
                    if (!rst || CHUNK_BUF_SIZE != poll_size)
                      {
                        return false;
                      }
                    rst = simpleFifoPoll(NULL, chunk.size - CHUNK_BUF_SIZE, &poll_size);
                    if (!rst || chunk.size - CHUNK_BUF_SIZE != (int32_t)poll_size)
                      {
                        return false;
                      }
                  }
                else
                  {
                    rst = simpleFifoPoll(chunk_data, chunk.size, &poll_size);
                    if (!rst || chunk.size != (int32_t)poll_size)
                      {
                        return false;
                      }
                  }
                break;
            }
        }
    }
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

__WIEN2_END_NAMESPACE
