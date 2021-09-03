/****************************************************************************
 * modules/audio/objects/stream_parser/mp3_stream_mng.cpp
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

#include "mp3_stream_mng.h"

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

bool Mp3StreamMng::init(const InitInputDataManagerParam &param)
{
  if (!m_done_open)
    {
      if (!checkSimpleFifoHandler(param))
        {
          return false;
        }
      setInitParam(param);
      int result = 0;
      result = Mp3Parser_initialize((FAR MP3PARSER_Handle *)&m_handle,
                                    p_simple_fifo_handler,
                                    (FAR MP3PARSER_Config *)&m_config);

      if (result == MP3PARSER_SUCCESS)
        {
          m_done_open = true;
          return true;
        }
    }
  return false;
}

InputDataManagerObject::GetEsResult
  Mp3StreamMng::getEs(FAR void *es_buf, FAR uint32_t *es_size)
{
  InputDataManagerObject::GetEsResult ret = EsEnd;
  int32_t ready_to_extract_frames = 0;
  if (m_done_open && (0 < *es_size))
    {
      size_t size = 0;
      if (!getOccupiedSize(&size))
        {
          return ret;
        }

      uint32_t max_buf_size = *es_size;
      if (MP3PARSER_SUCCESS ==
            Mp3Parser_pollSingleFrame((FAR MP3PARSER_Handle *)&m_handle,
                                      (FAR uint8_t *)es_buf,
                                      max_buf_size, es_size,
                                      &ready_to_extract_frames))
        {
          ret = EsExist;
        }
    }
  return ret;
}

bool Mp3StreamMng::finish()
{
  if (m_done_open)
    {
      Mp3Parser_finalize((FAR MP3PARSER_Handle *)&m_handle);
      m_done_open = false;
      return true;
    }
  return false;
}

bool Mp3StreamMng::getSamplingRate(FAR uint32_t *p_sampling_rate)
{
  if (m_done_open)
    {
      if (MP3PARSER_SUCCESS ==
            Mp3Parser_getSamplingRate((FAR MP3PARSER_Handle *)&m_handle,
                                      p_sampling_rate))
        {
          if (m_in_sampling_rate == AS_SAMPLINGRATE_AUTO)
            {
              m_in_sampling_rate = *p_sampling_rate;
              return true;
            }
          if (*p_sampling_rate != m_in_sampling_rate)
            {
              return false;
            }
          return true;
        }
    }
  return false;
}

bool Mp3StreamMng::getChNum(FAR uint32_t *p_ch_num)
{
  *p_ch_num = m_ch_num;
  return true;
}

bool Mp3StreamMng::getBitPerSample(FAR uint32_t *p_bit_per_sample)
{
  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

__WIEN2_END_NAMESPACE
