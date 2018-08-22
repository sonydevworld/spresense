/****************************************************************************
 * modules/audio/include/common/RamAdtsParser_Common.h
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

#ifndef __MODULES_AUDIO_INCLUDE_COMMON_RAMADTSPARSER_COMMON_H
#define __MODULES_AUDIO_INCLUDE_COMMON_RAMADTSPARSER_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "common/RamAdtsParser.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/** ADTS header
 * [11:00] syncword
 * [12:12] id
 * [14:13] layer
 * [15:15] protection bit
 * [17:16] profile
 * [21:18] sampling frequency
 * [22:22] private bit
 * [25:23] channel configuration
 * [26:26] original copy
 * [27:27] home
 * [28:28] copyright id bit
 * [29:29] copyright id start
 * [42:30] frame length
 * [53:43] adts buffer full
 * [55:54] number of raw data block
 */

#define ADTS_HEADER_SIZE    7    /* byte */

#define ADTSPARSER_SYNCWORD_1    0xFF
#define ADTSPARSER_SYNCWORD_2    0xF0

/* check SYNCWORD */

#define ADTS_CHECK_SYNCWORD(hdr0,hdr1)  \
          ((((hdr0 & ADTSPARSER_SYNCWORD_1) == ADTSPARSER_SYNCWORD_1) && \
            ((hdr1 & ADTSPARSER_SYNCWORD_2) == ADTSPARSER_SYNCWORD_2)) ? \
            ADTS_OK : ADTS_ERR)

/*----- Profile -----*/

#define ADTS_MASK_PROFILE     0xC0    /* Mask [2] */
#define ADTS_PROFILE_AACLC    0x40    /* profile=AAC-LC */

/*----- SamplingFrequency -----*/

#define ADTS_MASK_SAMPLING_RATE    0x3C /* Mask [2] */
#define ADTS_RSHIFT_SAMPLING_RATE  2    /* Right shift width after mask */

/* Macro for get sampling frequency */

#define ADTS_GET_SAMPLING_RATE(hdr2)  \
          (AdtsSamplingFrequency[((hdr2 & ADTS_MASK_SAMPLING_RATE) >> \
            ADTS_RSHIFT_SAMPLING_RATE)])

/*----- FrameLength -----*/

#define ADTS_MASK_FRAMELENGTH_3    0x03    /* Mask [3] */
#define ADTS_MASK_FRAMELENGTH_4    0xFF    /* Mask [4] */
#define ADTS_MASK_FRAMELENGTH_5    0xE0    /* Mask [5] */
#define ADTS_LSHIFT_FRAMELENGTH_3  11      /* Left shift width after mask */
#define ADTS_LSHIFT_FRAMELENGTH_4  3       /* Left shift width after mask */
#define ADTS_RSHIFT_FRAMELENGTH_5  5       /* Right shift width after mask */

/* Macro for get frame size */

#define ADTS_GET_FRAMELENGTH(hdr3,hdr4,hdr5)  \
          (((hdr3 & ADTS_MASK_FRAMELENGTH_3) << ADTS_LSHIFT_FRAMELENGTH_3) | \
          ((hdr4 & ADTS_MASK_FRAMELENGTH_4) << ADTS_LSHIFT_FRAMELENGTH_4) | \
          ((hdr5 & ADTS_MASK_FRAMELENGTH_5) >> ADTS_RSHIFT_FRAMELENGTH_5))

#define ADTSPARSER_SYNCWORD_SEARCH_SIZE 3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ADTS header information */

struct adts_parser_union_head_s
{
  int8_t hd_byte[ADTS_HEADER_SIZE];
};
typedef struct adts_parser_union_head_s AdtsParserUnionHead;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Sampling frequency table 
* (indx=sampling_frequency)
 */

/* SamplingFrequency (ISO compliant) */

const uint32_t AdtsSamplingFrequency[16] =
{
  96000, /* index: 0x0 */
  88200, /*        0x1 */
  64000, /*        0x2 */
  48000, /*        0x3 */
  44100, /*        0x4 */
  32000, /*        0x5 */
  24000, /*        0x6 */
  22050, /*        0x7 */
  16000, /*        0x8 */
  12000, /*        0x9 */
  11025, /*        0xA */
  8000,  /*        0xB */
  0,     /*        0xC */
  0,     /*        0xD */
  0,     /*        0xE */
  0      /*        0xF */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __MODULES_AUDIO_INCLUDE_COMMON_RAMADTSPARSER_COMMON_H */
