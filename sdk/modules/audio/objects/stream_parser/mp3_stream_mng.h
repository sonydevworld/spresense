/****************************************************************************
 * modules/audio/objects/stream_parser/mp3_stream_mng.h
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

#ifndef __MODULES_AUDIO_OBJECTS_STREAM_PARSER_MP3_STREAM_MNG_H
#define __MODULES_AUDIO_OBJECTS_STREAM_PARSER_MP3_STREAM_MNG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "input_data_mng_obj.h"
#include "common/Mp3Parser.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

class Mp3StreamMng : public InputDataManagerObject
{
public:
  Mp3StreamMng() :
    m_done_open(false)
  {}
  ~Mp3StreamMng() {}

  virtual bool init(const InitInputDataManagerParam &param);
  virtual InputDataManagerObject::GetEsResult getEs(FAR void *es_buf,
                                                    FAR uint32_t *es_size);
  virtual bool finish();
  virtual bool getSamplingRate(FAR uint32_t *p_sampling_rate);
  virtual bool getChNum(FAR uint32_t *p_ch_num);
  virtual bool getBitPerSample(FAR uint32_t *p_bit_per_sample);

private:
  MP3PARSER_Handle m_handle;
  MP3PARSER_Config m_config;

  bool    m_done_open;
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

#endif /* __MODULES_AUDIO_OBJECTS_STREAM_PARSER_MP3_STREAM_MNG_H */
