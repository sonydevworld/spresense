/****************************************************************************
 * modules/audio/objects/stream_parser/ram_opus_data_source.cpp
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

#include "ram_opus_data_source.h"
#include "string.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_AUDIOUTILS_UNUSE_ORIGINAL_OPUS_FORMAT
#  define OPUS_DATA_LEN         4
#  define OPUS_FINAL_RANGE_LEN  4
#else
#  define OPUS_DATA_LEN         1
#  define OPUS_FINAL_RANGE_LEN  0
#endif /* CONFIG_AUDIOUTILS_UNUSE_ORIGINAL_OPUS_FORMAT */

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

bool RamOpusDataSource::init(const InitInputDataManagerParam &param)
{
  if (!checkSimpleFifoHandler(param))
    {
      return false;
    }
  setInitParam(param);
  return true;
}

InputDataManagerObject::GetEsResult
  RamOpusDataSource::getEs(FAR void *es_buf, FAR uint32_t *es_size)
{
  size_t size = 0;
  if (!getOccupiedSize(&size))
    {
      return EsEnd;
    }

  uint8_t date_size[OPUS_DATA_LEN] = { 0 };
  if (!simpleFifoPeek(date_size, OPUS_DATA_LEN))
    {
      return EsEnd;
    }

#ifdef CONFIG_AUDIOUTILS_UNUSE_ORIGINAL_OPUS_FORMAT
  uint32_t len = char_to_int(date_size);
#else
  uint32_t len = date_size[0];
#endif /* CONFIG_AUDIOUTILS_UNUSE_ORIGINAL_OPUS_FORMAT */
  if (size < (len + OPUS_DATA_LEN + OPUS_FINAL_RANGE_LEN))
    {
      return EsEnd;
    }

  bool rst = false;
#ifdef CONFIG_AUDIOUTILS_UNUSE_ORIGINAL_OPUS_FORMAT
  rst = simpleFifoOffset(OPUS_DATA_LEN + OPUS_FINAL_RANGE_LEN);
#else
  rst = simpleFifoOffset(OPUS_DATA_LEN);
#endif /* CONFIG_AUDIOUTILS_UNUSE_ORIGINAL_OPUS_FORMAT */
  if (!rst)
    {
      return EsEnd;
    }

  size_t poll_size = 0;
  if (!simpleFifoPoll(es_buf, len, &poll_size))
    {
      return EsEnd;
    }
  *es_size = poll_size;
  return EsExist;
}

bool RamOpusDataSource::finish()
{
  return true;
}

bool RamOpusDataSource::getSamplingRate(FAR uint32_t *p_sampling_rate)
{
  *p_sampling_rate = m_in_sampling_rate;
  return true;
}

bool RamOpusDataSource::getChNum(FAR uint32_t *p_ch_num)
{
  *p_ch_num = m_ch_num;
  return true;
}

bool RamOpusDataSource::getBitPerSample(FAR uint32_t *p_bit_per_sample)
{
  return false;
}

#ifdef CONFIG_AUDIOUTILS_UNUSE_ORIGINAL_OPUS_FORMAT
uint32_t RamOpusDataSource::char_to_int(uint8_t ch[4])
{
  uint32_t result = 0;

  result = (((uint32_t)ch[0] << 24) |
            ((uint32_t)ch[1] << 16) |
            ((uint32_t)ch[2] <<  8) |
             (uint32_t)ch[3]);
  return result;
}
#endif /* CONFIG_AUDIOUTILS_UNUSE_ORIGINAL_OPUS_FORMAT */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

__WIEN2_END_NAMESPACE

