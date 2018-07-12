/****************************************************************************
 * modules/audio/objects/stream_parser/input_data_mng_obj.h
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

#ifndef __MODULES_AUDIO_OBJECTS_STREAM_PARSER_INPUT_DATA_MNG_OBJ_H
#define __MODULES_AUDIO_OBJECTS_STREAM_PARSER_INPUT_DATA_MNG_OBJ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "audio/audio_high_level_api.h"
#include "wien2_common_defs.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct init_input_data_mng_param_s
{
public:
  void    *p_simple_fifo_handler;
  uint8_t  codec_type;
  uint32_t in_sampling_rate;
  uint8_t  in_ch_num;
};
typedef struct init_input_data_mng_param_s InitInputDataManagerParam;

class InputDataManagerObject
{
public:
  InputDataManagerObject() {}
  ~InputDataManagerObject() {}

  /* Result Type of getEs(). */

  enum get_es_result_e
  {
    EsExist = 0,
    EsEnd,
    EsNone,
  };
  typedef enum get_es_result_e GetEsResult;

  virtual bool init(const InitInputDataManagerParam &param) = 0;
  virtual bool finish() = 0;
  virtual GetEsResult getEs(FAR void *es_buff, FAR uint32_t *es_size) = 0;
  virtual bool getSamplingRate(FAR uint32_t *sampling_rate) = 0;
  virtual bool getChNum(FAR uint32_t *p_ch_num) = 0;

  bool checkSimpleFifoHandler(const InitInputDataManagerParam &param)
    {
      if (param.p_simple_fifo_handler == NULL)
        {
          return false;
        }
      return true;
    }
  void setInitParam(const InitInputDataManagerParam &param)
    {
      p_simple_fifo_handler = static_cast<CMN_SimpleFifoHandle *>
        (param.p_simple_fifo_handler);
      m_codec_type = param.codec_type;
      m_in_sampling_rate = param.in_sampling_rate;
      m_ch_num = param.in_ch_num;
    }
  bool getOccupiedSize(FAR size_t *size)
    {
      *size = CMN_SimpleFifoGetOccupiedSize(p_simple_fifo_handler);
      if (*size <= 0)
        {
          return false;
        }
      return true;
    }
  bool simpleFifoPeek(FAR void* buferr, size_t size)
    {
      CMN_SimpleFifoPeekHandle pPeekHandle;
      size_t peek_size = CMN_SimpleFifoPeekWithOffset(p_simple_fifo_handler,
                                                      &pPeekHandle, size,
                                                      0);
      if (peek_size != size)
        {
          return false;
        }
      peek_size = CMN_SimpleFifoCopyFromPeekHandle(&pPeekHandle,
                                                   buferr,
                                                   size);
      if (peek_size != size)
        {
          return false;
        }
      return true;
    }
  bool simpleFifoPoll(FAR void *buferr,
                      size_t req_size,
                      FAR size_t *poll_size)
    {
      size_t size = CMN_SimpleFifoPoll(p_simple_fifo_handler,
                                       buferr,
                                       req_size);
      if (size <= 0)
        {
          return false;
        }
      *poll_size = size;
      return true;
    }
  bool simpleFifoOffset(size_t req_size)
    {
      int8_t buferr[128];
      size_t size = CMN_SimpleFifoPoll(p_simple_fifo_handler,
                                       buferr,
                                       req_size);
      if (req_size != size)
        {
          return false;
        }
      return true;
    }

protected:
  CMN_SimpleFifoHandle *p_simple_fifo_handler;
  uint32_t m_in_sampling_rate;
  uint32_t m_ch_num;
  uint8_t  m_codec_type;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

__WIEN2_END_NAMESPACE

#endif /* __MODULES_AUDIO_OBJECTS_STREAM_PARSER_INPUT_DATA_MNG_OBJ_H */
