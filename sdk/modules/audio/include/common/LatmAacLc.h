/****************************************************************************
 * modules/audio/include/common/LatmAacLc.h
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

#ifndef __MODULES_AUDIO_INCLUDE_COMMON_LATMAACLC_H_
#define __MODULES_AUDIO_INCLUDE_COMMON_LATMAACLC_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
 
/* If you set config_length, set value blow to config_length_flag
 */

#define LATM_ENABLE_CONFIG_LENGTH  1 /* config_length_flag enable */

/* Definition for Structure menber */

#define LATM_VAL_OF_4BIT    0xF
#define LATM_MAX_STREAM_ID  15       /* Stream ID Max */
#define LATM_MIN_STREAM_ID  0        /* Stream ID Min */

/* Index of information table is "Stream ID"
 */

#define LATM_MAX_STREAM_ARRAY    (LATM_MAX_STREAM_ID + 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/*
 * AudioSpecificConfig
 *
 * Keep every streamd ID in StreamMuxConfig
 * * When "use_same_config flag = 1", copy previous AudioSpcificConfig 
 */

struct info_audio_specific_config_s /* Index is streamID */
{
  /* audioObjectType(Max=95) */

  uint8_t audio_object_type;

  /* channelConfiguration(4-bit(valid range 0 - 0xf)) */

  uint8_t channel_configuration;

  /* samplingFrequencyIndex(4-bit(valid range 0 - 0xf)) */

  uint8_t sampling_frequency_index;

  /* When use program_config_element, keep information below at least
   * (it too big to keep whole PCE information)
   */

  /* A object type which set in program_config_element
   * when "chanelConfiguration = 0"
   */

  uint8_t pce_object_type;

  /* samplingFrequencyIndex which set in program_config_element
   * when "chanelConfiguration = 0"
   */

  uint8_t pce_sampling_frequency_index;

  /* For extention */

  int8_t   sbr_present_flag;     /* sbrPresentFlag */
  int8_t   ps_present_flag;      /* psPresentFlag */
  uint8_t  extension_audio_object_type; /* extensionAudioObjectType */

  /* extensionChannelConfiguration(ER-BSAC) */

  uint8_t  extension_channel_configuration;

  /* extensionSamplingFrequencyIndex */

  uint8_t  extension_sampling_frequency_index;

  /* extensionSamplingFrequency
   * (Use when 24-bit extensionSamplingFrequencyIndex=0xF(escape value))
   */

  uint32_t extension_sampling_frequency;

  /* FS value is not for extention, however, put on last due to alignment of structure */

  /* samplingFrequency
   * (Use when 24-bit samplingFrequencyIndex=0xF(escape value))
   */

  uint32_t sampling_frequency;

  /* config_length is set by user (if not set, take as 0) */

  /* config_length enable/disable flag(=1 enable !=1 disable) */

  uint8_t config_length_flag;
  uint8_t reserved;

  /* bit size of AudioSpecificConfig (Effective only when "config_length_flag=1" */

  int config_length;

  /*----- SpecificConfig (Comment out because size will be too big) -----*/

#ifdef LATMTEST_DBG_COMMENT
  union
  {
    /* When "channelConfiguration=0", depend on program_config_element()  
     * in GASpecificConfig. But no need for parser
     */

    struct GASpecificConfig  ga;  /* AAC */
    struct SSCSpecificConfig ssc; /* AudioObjectType=28(SSC) */
  } spConfig;
#endif /* LATMTEST_DBG_COMMENT */
};
typedef struct info_audio_specific_config_s InfoAudioSpecificConfig;

struct info_stream_id_s /* index is streamID */
{
  /* streamID is 0-15 (index of infoStreamID[]) */

  int8_t stream_id;     /* streamID */

  /* (Revese resolution)streamID */

  uint8_t prog;         /* Program number for the streamID */
  uint8_t lay;          /* Layer number for the streamID */

  /* Following is correspond to streamID */

  /* frameLengthType(payload type) */

  uint8_t frame_length_type;

  /* frameLength(User when "9-bit frameLengthType=1") */

  uint32_t frame_length;

  /* latmBufferFullness(When "frameLengthType=0") */

  uint8_t latm_buffer_fullness;

  /* useSameConfig(if euqals to 1, AudioSpecificConfig is ignored) */

  uint8_t use_same_config;

  /* Regardless of useSameConfig, prepare AudioSpecificConfig
   * (logically, there is a case that if StreamMuxConfig is exist
   * but AudioSpecificConfig is not exist
   */

  InfoAudioSpecificConfig asc;

  /* Offset from top of LATM
   * Correspond payload offset (Only if StreamMuxConfig exists)
   */

  uint32_t payload_offset;
};
typedef struct info_stream_id_s InfomationStreamID;

struct info_stream_frame_s
{
  /* Following is correspond to streamID */

  /* frameLengthType (payload type) */

  uint8_t frame_length_type;

  /* frameLength(Use when 9-bit frameLengthType=1) */

  uint32_t frame_length;

  /* Offset from top of LATM
   * Correspond payload offset (Only if StreamMuxConfig exist)
   */

  uint32_t payload_offset;
};
typedef struct info_stream_frame_s InfomationStreamFrame;

/* A structure information which should be kept by user for internal process.
 * (Of StreamMuxConfig)
 *
 * [Usage]
 * 1. User havet to allocate buffer for this structrue.
 * 2. When call APIs, user gives pointer to top of the buffer as 2nd argument.
 * 3. After this, if call APIs consecutive, don't free the buffer.
 *    (Don't free the buffer while read LATM frame consecutively.)
 *
 * * "info_stream_id[].asc.config_length", menber of structure, is set by user.
 *   ...If you know size of AudioConfigSpecific(), set bit length by the size,
 *      if you not, set to 0.
 */

struct info_stream_mux_config_s
{
  /* streamID MAX(0-15) */

  uint8_t max_stream_id;

  /* 1-bit audioMuxVersion */

  uint8_t audio_muxversion;

  /* 1-bit audioMuxVersionA */

  uint8_t audio_muxversion_a;

  /* 1-bit allStreamsSameTimeFraming */

  uint8_t all_streams_sametime_framing;

  /* 1-bit otherDataPresent */

  uint8_t other_data_present;
  uint8_t reserved;

  /* 6-bit(0-63) numSubFrames */

  uint8_t num_sub_frames;

  /* 4-bit(0-15) numProgram */

  uint8_t num_program;

  /* 3-bit(0-7)(index is program) numLayer */

  uint8_t num_layer[(LATM_VAL_OF_4BIT + 1)];

  /* otherDataLenBits(Use when otherDataPresent=1) */

  uint32_t other_data_len_bits;

  /* progSIndx(Use when allStreamsSameTimeFraming=0) */

  uint8_t   prog_stream_indx[LATM_MAX_STREAM_ID];

  /* laySIndx(Use when allStreamsSameTimeFraming=0) */

  uint8_t   lay_stream_indx[LATM_MAX_STREAM_ID];

  /* Syntactically, maximum is "Program x Layey", but streamIdx=streamCnt=streamID,
   * therefore, actually maximun is 16.
   */

  /* Index is streamID */

  InfomationStreamID info_stream_id[LATM_MAX_STREAM_ARRAY];
  InfomationStreamFrame info_stream_frame[64];
};
typedef struct info_stream_mux_config_s InfoStreamMuxConfig;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/*
 * AACLC_getNextLatm()
 *
 * Get top of next LATM
 *
 * arg1 : Top of LOAS/LATM(ex, top of payload)
 * arg2 : Top of information structure (see above)
 *
 * return : Top of next LATM frame begin with current LATM frame which is appointed by arg1.
 *          0=NG(AudioObjectType which is written in LATM header is out of support)
 */
FAR uint8_t *AACLC_getNextLatm(FAR uint8_t *ptr_readbuff,
                               FAR InfoStreamMuxConfig *ptr_stream_mux_config);

#endif /* __MODULES_AUDIO_INCLUDE_COMMON_LATMAACLC_H_ */
