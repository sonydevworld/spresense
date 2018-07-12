/****************************************************************************
 * modules/audio/objects/stream_parser/ram_aaclc_data_source.cpp
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

#include "ram_aaclc_data_source.h"
#include "string.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DATA_BUFF_LEN_SIZE 2
#define A2DP_AAC_BUFF_SIZE 1024

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t peek_data[A2DP_AAC_BUFF_SIZE];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

bool RamAACLCDataSource::init(const InitInputDataManagerParam &param)
{
  if (!checkSimpleFifoHandler(param))
    {
      return false;
    }
  setInitParam(param);

  if (m_codec_type == AS_CODECTYPE_AAC)
    {
      int32_t err_detail = 0;

      /* Clear handle information. */

      memset(&m_handle, 0, sizeof(AdtsHandle));

      /* File open of ADTS, and check format.
       * (Error detail is currently unused.)
       */

      if (AdtsParser_Initialize(&m_handle,
                                p_simple_fifo_handler,
                                reinterpret_cast<FAR AdtsParserErrorDetail *>
                                  (&err_detail)) == ADTS_OK)
        {
          return true;
        }
      return false;
    }
  return true;
}

InputDataManagerObject::GetEsResult
  RamAACLCDataSource::getEs(FAR void *es_buf, FAR uint32_t *es_size)
{
  size_t size = 0;
  if (!getOccupiedSize(&size))
    {
      return EsEnd;
    }

  if (m_codec_type == AS_CODECTYPE_AAC)
    {
      uint32_t max_es_buf_size = *es_size;
      InputDataManagerObject::GetEsResult ret = EsExist;
      uint32_t read_size = 0;
      uint16_t check_result = 0;
      AdtsParserErrorDetail err_detail;
      if (0 < max_es_buf_size)
        {
          read_size = max_es_buf_size;

          /* Read ADTS frames.
           * (Results of the validity check and details of the error
           *  are currently unused.)
           */

          if (AdtsParser_ReadFrame(&m_handle,
                                   reinterpret_cast<FAR int8_t *>(es_buf),
                                   &read_size,
                                   &check_result,
                                   &err_detail)
                != ADTS_OK)
            {
              read_size = 0;
            }
        }

      *es_size = read_size;
      if (read_size == 0)
        {
          ret = EsEnd;
        }
      return ret;
    }
  else
    {
      uint16_t payload_size = 0;
      if (!simpleFifoPeek(&payload_size, DATA_BUFF_LEN_SIZE))
        {
          return EsEnd;
        }
      if (A2DP_AAC_BUFF_SIZE < payload_size)
        {
          return EsEnd;
        }
      if (size < (uint32_t)payload_size + DATA_BUFF_LEN_SIZE)
        {
          return EsEnd;
        }
      if (!simpleFifoOffset(DATA_BUFF_LEN_SIZE))
        {
          return EsEnd;
        }
      size_t poll_size = 0;
      memset(peek_data, 0, A2DP_AAC_BUFF_SIZE);
      if (!simpleFifoPoll(peek_data, payload_size, &poll_size))
        {
          return EsEnd;
        }

      InfoStreamMuxConfig stream_mux_config;
      memset(&stream_mux_config, 0, sizeof(InfoStreamMuxConfig));
      uint8_t *rest = AACLC_getNextLatm(peek_data, &stream_mux_config);
      if (rest != 0)
        {
          *es_size = stream_mux_config.info_stream_frame[0].frame_length;
          memcpy((FAR char *)es_buf,
                 &peek_data[stream_mux_config.info_stream_frame[0].
                 payload_offset / 8],
                 *es_size);
        }
      return EsExist;
    }
}

bool RamAACLCDataSource::finish()
{
  if (m_codec_type == AS_CODECTYPE_AAC)
    {
      AdtsParserErrorDetail err_detail;

      /* Close file of ADTS.
       * (Error detail is currently unused.)
       */

      AdtsParser_Finalize(&m_handle, &err_detail);

      /* Clear handle information. */

      memset(&m_handle, 0, sizeof(AdtsHandle));
    }
  return true;
}

bool RamAACLCDataSource::getSamplingRate(FAR uint32_t *p_sampling_rate)
{
  if (m_codec_type == AS_CODECTYPE_AAC)
    {
      AdtsParserErrorDetail err_detail;

      /* Get Sampling rate value.
       * (Error detail is currently unused.)
       */

      if (ADTS_OK == AdtsParser_GetSamplingRate(&m_handle,
                                                p_sampling_rate,
                                                &err_detail))
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
      return false;
    }
  else
    {
      size_t size = CMN_SimpleFifoGetOccupiedSize(p_simple_fifo_handler);
      if (size <= 0)
        {
          return false;
        }

      CMN_SimpleFifoPeekHandle pPeekHandle;
      size = CMN_SimpleFifoPeekWithOffset(p_simple_fifo_handler,
                                          &pPeekHandle,
                                          DATA_BUFF_LEN_SIZE,
                                          0);
      if (!size)
        {
          return false;
        }
      uint16_t payload_size = 0;
      CMN_SimpleFifoCopyFromPeekHandle(&pPeekHandle,
                                       &payload_size, DATA_BUFF_LEN_SIZE);
      if (A2DP_AAC_BUFF_SIZE < payload_size)
        {
          return false;
        }

      size = CMN_SimpleFifoPeekWithOffset(p_simple_fifo_handler,
                                          &pPeekHandle,
                                          payload_size,
                                          DATA_BUFF_LEN_SIZE);
      if (!size)
        {
          return false;
        }
      CMN_SimpleFifoCopyFromPeekHandle(&pPeekHandle, peek_data, payload_size);

      InfoStreamMuxConfig stream_mux_config;
      memset(&stream_mux_config, 0, sizeof(InfoStreamMuxConfig));

      uint8_t *rest = AACLC_getNextLatm(peek_data, &stream_mux_config);
      if (rest == 0)
        {
          return false;
        }

      if (stream_mux_config.info_stream_id[0].asc.
          channel_configuration != MonoChannels &&
          stream_mux_config.info_stream_id[0].asc.
          channel_configuration != TwoChannels)
        {
          return false;
        }
      m_ch_num = stream_mux_config.info_stream_id[0].asc.
        channel_configuration;

      if (stream_mux_config.info_stream_id[0].asc.
          sampling_frequency_index < AudFs_48000 ||
          stream_mux_config.info_stream_id[0].asc.
          sampling_frequency_index > AudFs_24000)
        {
          return false;
        }
      *p_sampling_rate = AudioFs2ApuValue[stream_mux_config.
        info_stream_id[0].asc.sampling_frequency_index];

      return true;
    }
}

bool RamAACLCDataSource::getChNum(FAR uint32_t *p_ch_num)
{
  *p_ch_num = m_ch_num;
  return true;
}

bool RamAACLCDataSource::getBitPerSample(FAR uint32_t *p_bit_per_sample)
{
  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

__WIEN2_END_NAMESPACE

