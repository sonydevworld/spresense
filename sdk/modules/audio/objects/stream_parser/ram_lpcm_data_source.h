/****************************************************************************
 * modules/audio/objects/stream_parser/ram_lpcm_data_source.h
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

#ifndef __MODULES_AUDIO_OBJECTS_STREAM_PARSER_RAM_LPCM_DATA_SOURCE_H
#define __MODULES_AUDIO_OBJECTS_STREAM_PARSER_RAM_LPCM_DATA_SOURCE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "input_data_mng_obj.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Required chunk. */

#define CHUNKID_RIFF     0x46464952  /* RIFF */
#define SUBCHUNKID_FMT   0x20746D66  /* fmt  */
#define SUBCHUNKID_DATA  0x61746164  /* data */

/* Option chunk. */

#define SUBCHUNKID_JUNK  0x4B4E554A  /* JUNK */
#define SUBCHUNKID_LIST  0x5453494C  /* LIST */
#define SUBCHUNKID_ID3   0x20336469  /* id3  */
#define SUBCHUNKID_FACT  0x74636166  /* fact */
#define SUBCHUNKID_PLST  0x74736C70  /* plst */
#define SUBCHUNKID_CUE   0x20657563  /* cue  */
#define SUBCHUNKID_LABL  0x6C62616C  /* labl */
#define SUBCHUNKID_NOTE  0x65746F6E  /* note */
#define SUBCHUNKID_LTXT  0x7478746C  /* ltxt */
#define SUBCHUNKID_SMPL  0x6C706D73  /* smpl */
#define SUBCHUNKID_INST  0x74736E69  /* inst */
#define SUBCHUNKID_BEXT  0x74786562  /* bext */
#define SUBCHUNKID_IXML  0x4C4D5869  /* iXML */
#define SUBCHUNKID_QLTY  0x79746C71  /* qlty */
#define SUBCHUNKID_MEXT  0x7478656D  /* mext */
#define SUBCHUNKID_LEVL  0x6C76656C  /* levl */
#define SUBCHUNKID_LINK  0x6B6E696C  /* link */
#define SUBCHUNKID_AXML  0x6C6D7861  /* axml */
#define SUBCHUNKID_CONT  0x746E6F63  /* cont */

#define CHUNK_BUF_SIZE  1024

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct chunk_s
{
  uint32_t chunk_id;
  int32_t  size;
};
typedef struct chunk_s chunk_t;

struct riff_chunk_s
{
  struct chunk_s  chunk;
  uint32_t        type;
};
typedef struct riff_chunk_s riff_chunk_t;

struct fmt_chunk_s
{
  uint16_t  format;
  uint16_t  channel;
  uint32_t  rate;
  uint32_t  avgbyte;
  uint16_t  block;
  uint16_t  bit;
  uint16_t  extended_size;
};
typedef struct fmt_chunk_s fmt_chunk_t;

class RawLpcmDataSource : public InputDataManagerObject
{
public:
  RawLpcmDataSource() {}
  ~RawLpcmDataSource() {}

  virtual bool init(const InitInputDataManagerParam& param);
  virtual InputDataManagerObject::GetEsResult getEs(FAR void *es_buf,
                                                    FAR uint32_t *es_size);
  virtual bool finish();
  virtual bool getSamplingRate(FAR uint32_t *p_sampling_rate);
  virtual bool getChNum(FAR uint32_t *p_ch_num);
  virtual bool getBitPerSample(FAR uint32_t *p_bit_per_sample);

private:
  bool ParseChunk(void);
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

#endif /* __MODULES_AUDIO_OBJECTS_STREAM_PARSER_RAM_LPCM_DATA_SOURCE_H */
