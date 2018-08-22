/****************************************************************************
 * modules/audio/include/common/Mp3Parser.h
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

#ifndef __MODULES_AUDIO_INCLUDE_COMMON_MP3PARSER_H
#define __MODULES_AUDIO_INCLUDE_COMMON_MP3PARSER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include "memutils/simple_fifo/CMN_SimpleFifo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*----- Return Value -----*/

#define MP3PARSER_SUCCESS           0  /* Success */
#define MP3PARSER_NO_FRAME_HEADER   -1 /* Frame header is not exist */
#define MP3PARSER_NO_OUTPUT_REGION  -2 /* Buffer size for frame extraction is not enough */
#define MP3PARSER_NO_CAPABILITY     -3 /* Cannot use */
#define MP3PARSER_FILE_ERROR        -4 /* Specified file is not exist */
#define MP3PARSER_PARAMETER_ERROR   -5 /* Parameter is not correct */

/* (ready_to_extract_frames) */

#define MP3PARSER_NEXT_SYNC_UNFOUND  0 /* Next SYNCWORD was not found */
#define MP3PARSER_NEXT_SYNC_FOUND    1 /* Next SYNCWORD was found */

#define MP3PARSER_SYNCWORD_LENGTH    3 /* Syncword length */

/*----- Misc default value (If not specified, use this) -----*/

/* Upper limit of 1st SYNCWORD search range (byte) */

#define MP3PARSER_DEFAULT_1ST_SYNC_SEARCH_MAX 0xFFFFFFFF

/* Upper limit of 2nd SYNCWORDK search range (byte) */

#define MP3PARSER_DEFAULT_2ND_SYNC_SEARCH_MAX 256

/* Default value of extraction mode */

#define MP3PARSER_DEFAULT_EXTRACTION_MODE  (Mp3ParserExtractFrameOnly)

#define MP3PARSER_DEFAULT_RAM_OFFSET  0xFFFFFFFF

#define MP3PARSER_SYNCWORD_1    0xFF
#define MP3PARSER_SYNCWORD_2    0xF0

/* For checking any items (reserved will not use) */

#define MP3PARSER_FS_RESERVED        3    /* '11' */
#define MP3PARSER_BITRATE_FREE       0    /* '0000' */
#define MP3PARSER_BITRATE_UNUSED     15   /* '1111' */
#define MP3PARSER_PRIVATEBIT_ISOUSED 1    /* '1' */
#define MP3PARSER_EMPHASIS_RESERVED  2    /* '10' */

/* Members of 2nd byte */

#define MP3PARSER_GET_ID(byte)          ((byte & 0x08) >> 3)
#define MP3PARSER_GET_LAYER(byte)       ((byte & 0x06) >> 1)
#define MP3PARSER_GET_PROTECTION(byte)   (byte & 0x01)
/* Members of 3rd byte */
#define MP3PARSER_GET_BR(byte)          ((byte & 0xF0) >> 4)
#define MP3PARSER_GET_FS(byte)          ((byte & 0x0C) >> 2)
#define MP3PARSER_GET_PADDING(byte)     ((byte & 0x02) >> 1)
#define MP3PARSER_GET_PRIVATE(byte)      (byte & 0x01)
/* Members of 4th byte */
#define MP3PARSER_GET_MODE(byte)        ((byte & 0xC0) >> 6)
#define MP3PARSER_GET_MODEEXT(byte)     ((byte & 0x30) >> 4)
#define MP3PARSER_GET_CPRIGHT(byte)     ((byte & 0x08) >> 3)
#define MP3PARSER_GET_ORGHOME(byte)     ((byte & 0x04) >> 2)
#define MP3PARSER_GET_EMPHAS(byte)       (byte & 0x02)

#define MP3PARSER_BITLENGTH_BYTE     8      /* Bit length per a byte */
#define MP3PARSER_SLOTLENGTH_LAYER1  4      /* 1-slot length of LAYER1 */
#define MP3PARSER_SLOTLENGTH_LAYER23 1      /* 2/3-slot length of LAYER1 */

/* Macros to get sample num ber a byte
 * * You can get magic number(ex, 144, 12) which appears in
 * "calculation expression for slot length" from ISO document.
 */

#define MP3PARSER_CONST_BYTE_SAMPLES_V1(layer) \
          (mp3_parser_v1_num_samples_frame[layer] / MP3PARSER_BITLENGTH_BYTE)
#define MP3PARSER_CONST_BYTE_SAMPLES_V1_LAYER1 \
          (mp3_parser_v1_num_samples_frame[Mp3ParserLayer1] / \
            MP3PARSER_BITLENGTH_BYTE / MP3PARSER_SLOTLENGTH_LAYER1)
#define MP3PARSER_CONST_BYTE_SAMPLES_V2(layer) \
          (mp3_parser_v2_num_samples_frame[layer] / \
            MP3PARSER_BITLENGTH_BYTE)
#define MP3PARSER_CONST_BYTE_SAMPLES_V2_LAYER1 \
          (mp3_parser_v2_num_samples_frame[Mp3ParserLayer1] / \
            MP3PARSER_BITLENGTH_BYTE / MP3PARSER_SLOTLENGTH_LAYER1)

/* Macros to get frame byte length
 * ...if padding is there, add to this value
 * (1st dimention index = sampling_frequency)
 * (2nd dimention index = bitrate_index)
 *
 * (sample num per a byte * bitrate / sampling_frequency) + padding length
 */

/* For padding addition(Unit of return value is "slot") */

#define MP3PARSER_CALC_PADDING(padding)    ((padding == 1) ? 1 : 0)

/* Layer1 ... slot = 4byte */

#define MP3PARSER_CALC_LAYER1_V1_FRAME_SIZE(br_idx,fs_idx,padding) \
          ((((MP3PARSER_CONST_BYTE_SAMPLES_V1_LAYER1 * \
            mp3_parser_v1_bitrate[Mp3ParserLayer1][br_idx] / \
            mp3_parser_v1_sampling_frequency[fs_idx])) + \
            MP3PARSER_CALC_PADDING(padding)) * MP3PARSER_SLOTLENGTH_LAYER1)
#define MP3PARSER_CALC_LAYER1_V2_FRAME_SIZE(br_idx,fs_idx,padding) \
          ((((MP3PARSER_CONST_BYTE_SAMPLES_V2_LAYER1 * \
            mp3_parser_v2_bitrate[Mp3ParserLayer1][br_idx] / \
            mp3_parser_v2_sampling_frequency[fs_idx])) + \
            MP3PARSER_CALC_PADDING(padding)) * MP3PARSER_SLOTLENGTH_LAYER1)

/* Layer3/Layer2 */

#define MP3PARSER_CALC_LAYER3_V1_FRAME_SIZE(layer,br_idx,fs_idx,padding) \
          ((MP3PARSER_CONST_BYTE_SAMPLES_V1(layer) * \
            mp3_parser_v1_bitrate[layer][br_idx] / \
            mp3_parser_v1_sampling_frequency[fs_idx]) + \
            MP3PARSER_CALC_PADDING(padding))
#define MP3PARSER_CALC_LAYER3_V2_FRAME_SIZE(layer,br_idx,fs_idx,padding) \
          ((MP3PARSER_CONST_BYTE_SAMPLES_V2(layer) * \
            mp3_parser_v2_bitrate[layer][br_idx] / \
            mp3_parser_v2_sampling_frequency[fs_idx]) + \
            MP3PARSER_CALC_PADDING(padding))

/* Common for any Layer (mpeg1/2 separated) */

#define MP3PARSER_CALC_V1_FRAME_SIZE(layer,br_idx,fs_idx,padding)  \
          ((layer == Mp3ParserLayer1) ? \
            MP3PARSER_CALC_LAYER1_V1_FRAME_SIZE(br_idx,fs_idx,padding) : \
            MP3PARSER_CALC_LAYER3_V1_FRAME_SIZE(layer,br_idx,fs_idx,padding))
#define MP3PARSER_CALC_V2_FRAME_SIZE(layer,br_idx,fs_idx,padding)  \
          ((layer == Mp3ParserLayer1) ? \
            MP3PARSER_CALC_LAYER1_V2_FRAME_SIZE(br_idx,fs_idx,padding) : \
            MP3PARSER_CALC_LAYER3_V2_FRAME_SIZE(layer,br_idx,fs_idx,padding))

/* Common for any Layer */

#define MP3PARSER_CALC_FRAME_SIZE(id,layer,br_idx,fs_idx,padding)  \
          ((id == Mp3ParserMpeg1) ? \
            MP3PARSER_CALC_V1_FRAME_SIZE(layer,br_idx,fs_idx,padding) : \
            MP3PARSER_CALC_V2_FRAME_SIZE(layer,br_idx,fs_idx,padding))

/* Mininum frame of MP3 frmaes(MPEG2 Layer3 fs=24kHz br=8k) */

#define MP3PARSER_MIN_MP3FRAME_LENGTH  (24 + 4)

#define MP3PARSER_NULL          0
#define MP3PARSER_HEADSIZE      4

/* For if extraction target is file */

/* Size of local buffer which used for file read */

#define MP3PARSER_LOCAL_READFILE_BUFFERSIZE  32

/* Size of local buffer which used for file read */

#define MP3PARSER_LOCAL_POLL_BUFFERSIZE      1024

#ifndef O_BINARY
#define O_BINARY  0
#endif

#ifdef WINDOWS
/* File seek origin "Top of the file" */

#define PA3PARSER_FILESEEK_ORIGIN_TOP  0

/* File seek origin "Bottom of the file" */

#define PA3PARSER_FILESEEK_ORIGIN_END  2
#else
/*--------------------- for CXD5602 --------------------*/

/* File seek origin "Top of the file" */

#define PA3PARSER_FILESEEK_ORIGIN_TOP   1  /* FS_FSEEK_SET */

/* File seek origin "Bottom of the file" */

#define PA3PARSER_FILESEEK_ORIGIN_END   2  /* FS_FSEEK_END */
#endif

/* ID3v2 tag */

#define MP3PARSER_ID3V2_ID1      0x49  /* 'I' */
#define MP3PARSER_ID3V2_ID2      0x44  /* 'D' */
#define MP3PARSER_ID3V2_ID3      0x33  /* '3' */

/* Get ID3v2 tag length (High oreder bit of every bytes are ignored) */

#define MP3PARSER_ID3v2_GET_LENGTH(len1,len2,len3,len4) \
          (uint32_t)(((len1 & 0x7F) << 21) | ((len2 & 0x7F) << 14) | \
            ((len3 & 0x7F) << 7) | (len4 & 0x7F))

/* ID3v1 tag length(fixed value) */

#define MP3PARSER_ID3v1_FIXED_LENGTH    128
#define MP3PARSER_ID3v1_2_FIXED_LENGTH  227

/* ID3v1 tag */

#define MP3PARSER_ID3V1_ID1     0x54  /* 'T' */
#define MP3PARSER_ID3V1_ID2     0x41  /* 'A' */
#define MP3PARSER_ID3V1_ID3     0x47  /* 'G' */
#define MP3PARSER_ID3V1_ID4     0x2B  /* '+' */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* MP3 extraction mode (Designate at start API) */

enum mp3parser_extraction_mode_e
{
  Mp3ParserExtractFrameOnly = 0,          /* Extract by calcurated frame length */
  Mp3ParserExtractFrameAndTrailingPadding /* Extract here to next SyncWord as a frame */
};
typedef enum mp3parser_extraction_mode_e MP3PARSER_ExtractionMode;

/* MP3 extraction target type */

enum mp3parser_src_type_e
{
  Mp3ParserSrcBuffer = 0,   /* Extraction target is buffer */
  Mp3ParserSrcFile          /* Extraction target is file */
};
typedef enum mp3parser_src_type_e MP3PARSER_SrcType;

/* For library function check API */

enum mp3parser_capability_e
{
  Mp3ParserCapabilityPeekable = 0,  /* Peekabale? */
};
typedef enum mp3parser_capability_e MP3PARSER_Capability;

/** "Other parameters" for Library start API
 *  (Buffer for parameter information should be allocated by calling source)
 *
 *  * This parameters are common for extraction target "buffer" and "file"
 */

struct mp3parser_config_s
{
  uint32_t search_max_1st_sync; /* 1st syncword search range (byte) */
  uint32_t search_max_2nd_sync; /* 2nd syncword search range (byte) */
  uint8_t  extraction_mode;     /* Extranction mode */
  uint8_t  reserved[3];
};
typedef struct mp3parser_config_s MP3PARSER_Config;

/** Handle information for API call
 *  (Buffer for handle information should be allocated by calling source)
 *
 *  * Initialize when library starts, and part of them will be modified every extraction
 *
 */

struct mp3parser_handle_s
{
  int32_t  counter_of_extracted_frame; /* Extracted frame num */
  MP3PARSER_SrcType src_type;          /* Extraction target type */
  uint8_t  extraction_mode;            /* Extraction mode */
  uint32_t search_max_1st_sync;        /* 1st syncword search range (byte) */
  uint32_t search_max_2nd_sync;        /* 2nd syncword search range (byte) */
  union
  {
    /* Simple Fifo handle of extraction target (When extraction target type = buffer) */

    FAR CMN_SimpleFifoHandle *simple_fifo_handler;

    /* Top of buffer for extraction target (When extraction target type = buffer) */

    FAR uint8_t *top_buff;
  } src;

  /* Size of Extraction target (Common for type "buffer" and "file") */

  uint32_t size_of_src;

  /* Current offset of Extraction target (Common for type "buffer" and "file") */

  uint32_t current_offset;
  FAR MP3PARSER_Config  *pConfig;      /* Othe parameters information */
};
typedef struct mp3parser_handle_s MP3PARSER_Handle;

/*--------------------------------------------------------------------------*/
#if 0  /* BitField Structure -(Now not used, however, it remains as "header information" comment */
#ifdef WINDOWS
/** MP3 header - bit field
 *  (for little endian)
 */

struct mp3parser_header_s
{
  uint8_t  syncword1 : 8;             /* [0] bit0-7 */
  uint8_t  protection_bit : 1;        /* [1] bit7 */
  uint8_t  layer : 2;                 /* [1] bit5-6 */
  uint8_t  id : 1;                    /* [1] bit4 */
  uint8_t  syncword2 : 4;             /* [1] bit0-3 */
  uint8_t  private_bit : 1;           /* [2] bit7 */
  uint8_t  padding_bit : 1;           /* [2] bit6 */
  uint8_t  sampling_frequency : 2;    /* [2] bit4-5 */
  uint8_t  bitrate_index : 4;         /* [2] bit0-3 */
  uint8_t  emphasis : 2;              /* [3] bit6-7 */
  uint8_t  original_home : 1;         /* [3] bit5 */
  uint8_t  copyright : 1;             /* [3] bit4 */
  uint8_t  mode_extension : 2;        /* [3] bit2-3 */
  uint8_t  mode : 2;                  /* [3] bit0-1 */
};
typedef struct mp3parser_header_s Mp3ParserHeader;
#else
/** MP3 header - bit field
 *  (for big endian)
 */

struct mp3parser_header_s
{
  unsigned int  syncword : 12;           /* [0] bit0-7 [1] bit0-3 */
  unsigned int  id : 1;                  /* [1] bit4 */
  unsigned int  layer : 2;               /* [1] bit5-6 */
  unsigned int  protection_bit : 1;      /* [1] bit7 */
  unsigned int  bitrate_index : 4;       /* [2] bit0-3 */
  unsigned int  sampling_frequency : 2;  /* [2] bit4-5 */
  unsigned int  padding_bit : 1;         /* [2] bit6 */
  unsigned int  private_bit : 1;         /* [2] bit7 */
  unsigned int  mode : 2;                /* [3] bit0-1 */
  unsigned int  mode_extension : 2;      /* [3] bit2-3 */
  unsigned int  copyright : 1;           /* [3] bit4 */
  unsigned int  original_home : 1;       /* [3] bit5 */
  unsigned int  emphasis : 2;            /* [3] bit6-7 */
};
typedef struct mp3parser_header_s Mp3ParserHeader;
#endif
#endif

/* MP3 header info */

struct mp3parser_union_head_s
{
  uint8_t copy_byte[4];
};
typedef struct mp3parser_union_head_s Mp3ParserUnionHead;

/* Layer type */

enum mp3parser_header_layer_e
{
  Mp3ParserLayerReserved = 0,
  Mp3ParserLayer3,
  Mp3ParserLayer2,
  Mp3ParserLayer1,
};
typedef enum mp3parser_header_layer_e Mp3ParserHeaderLayer;

/* ID(version) */

enum mp3parser_header_id_e
{
  Mp3ParserMpeg2 = 0,      /* MPEG2 (version2) */
  Mp3ParserMpeg1,          /* MPEG1 (version1) */
};
typedef enum mp3parser_header_id_e Mp3ParserHeaderId;

/* I/F for internal functions */

struct mp3parser_local_info_s
{
  Mp3ParserUnionHead  uhd;   /* MP3 header(4byte fixed) */
  FAR uint8_t *ptr_start;    /* temporary */
  uint32_t max_search_byte;  /* temporary */
  uint32_t search_offset;    /* temporary */
  uint32_t found_offset;     /* temporary */
  uint32_t sync_offset_1;    /* 1th syncword position (Offset from top) */
  uint32_t sync_offset_2;    /* 2nd syncword position (Offset from top) */
  uint32_t frame_length_1;   /* 1st frame length (calculated) */
  uint32_t frame_length_2;   /* 2nd frame length (calculated) */
  uint32_t remain_length;    /* Remain size */
};
typedef struct mp3parser_local_info_s Mp3ParserLocalInfo;

/* Return value of file access function */

enum mp3parser_return_value_of_file_e
{
  Mp3ParserReturnNoFilename = (-3),     /* Filename Error */
  Mp3ParserReturnFileOpenError = (-2),  /* File open error */
  Mp3ParserReturnFileAccesError = (-1), /* File access error */
  Mp3ParserReturnFileFavorable = 0,     /* No error */
};
typedef enum mp3parser_return_value_of_file_e Mp3ParserReturnValueOfFile;

/* index of "byte table" of ID3v2 tag header */

enum mp3parser_id3v2_header_index_e
{
  Mp3ParserID3v2HeadIndexID1 = 0,
  Mp3ParserID3v2HeadIndexID2,
  Mp3ParserID3v2HeadIndexID3,
  Mp3ParserID3v2HeadIndexVer1,
  Mp3ParserID3v2HeadIndexVer2,
  Mp3ParserID3v2HeadIndexFlag,
  Mp3ParserID3v2HeadIndexLen1,
  Mp3ParserID3v2HeadIndexLen2,
  Mp3ParserID3v2HeadIndexLen3,
  Mp3ParserID3v2HeadIndexLen4,
  Mp3ParserID3v2HeaderLength     /* byte 10 (Use for ID3v2 tag header length) */
};
typedef enum mp3parser_id3v2_header_index_e Mp3ParserID3v2HeaderIndex;

/* index of "byte table" of ID3v1 tag header */

enum mp3parser_id3v1_header_index_e
{
  Mp3ParserID3v1HeadIndexID1 = 0,
  Mp3ParserID3v1HeadIndexID2,
  Mp3ParserID3v1HeadIndexID3,
  Mp3ParserID3v1HeadIndexID4,
};
typedef enum mp3parser_id3v1_header_index_e Mp3ParserID3v1HeaderIndex;

/* Return value from syncword search function */

enum mp3parser_return_value_of_sync_search_e
{
  Mp3ParserReturnNoSyncword = (-1), /* syncword not detected */
  Mp3ParserReturnPendding = 0,      /* Pending(There is a data which seems to syncword 
                                     * in range, however, header information is out of
                                     * range, and cannot confirm.)
                                     */
  Mp3ParserReturnFoundSyncword = 1, /* syncword detected */
  Mp3ParserReturnFound1stOnly,      /* Only 1st syncword is detected */
  Mp3ParserReturnSearchContinue,    /* Skip fake syncword and continue processing */
};
typedef enum mp3parser_return_value_of_sync_search_e \
               Mp3ParserReturnValueOfSyncSearch;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Sample num per a frame. (Order is as same as "Mp3ParserHeaderLayer") */

static const uint32_t mp3_parser_v1_num_samples_frame[] =
{
  0,
  1152,
  1152,
  384
};

static const uint32_t mp3_parser_v2_num_samples_frame[] =
{
  0,
  576,
  1152,
  384
};

/* Layer bitrate table 
 * (1st dimention index = Mp3ParserHeaderLayer)
 * (2nd dimention index = bitrate_index)
 */

static const int32_t mp3_parser_v1_bitrate[4][16] =
{
  /* mp3_parser_layer_reserved */

  {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
  },

  /* mp3_parser_layer_3 */

  {
    0,
    32000,
    40000,
    48000,
    56000,
    64000,
    80000,
    96000,
    112000,
    128000,
    160000,
    192000,
    224000,
    256000,
    320000,
    -1
  },

  /* mp3_parser_layer_2 */

  {
    0,
    32000,
    48000,
    56000,
    64000,
    80000,
    96000,
    112000,
    128000,
    160000,
    192000,
    224000,
    256000,
    320000,
    384000,
    -1
  },

  /* mp3_parser_layer_1 */

  {
    0,
    32000,
    64000,
    96000,
    128000,
    160000,
    192000,
    224000,
    256000,
    288000,
    320000,
    352000,
    384000,
    416000,
    448000,
    -1
  }
};

static const int32_t mp3_parser_v2_bitrate[4][16] =
{
  /* mp3_parser_layer_reserved */

  {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
  },

  /* mp3_parser_layer_3 */

  {
    0,
    8000,
    16000,
    24000,
    32000,
    40000,
    48000,
    56000,
    64000,
    80000,
    96000,
    112000,
    128000,
    144000,
    160000,
    -1
  },

  /* mp3_parser_layer_2 */

  {
    0,
    8000,
    16000,
    24000,
    32000,
    40000,
    48000,
    56000,
    64000,
    80000,
    96000,
    112000,
    128000,
    144000,
    160000,
    -1
  },

  /* mp3_parser_layer_1 */

  {
    0,
    32000,
    48000,
    56000,
    64000,
    80000,
    96000,
    112000,
    128000,
    144000,
    160000,
    176000,
    192000,
    224000,
    256000,
    -1
  }
};

/* Sampling frequency table (index = sampling_frequency) */

static const uint32_t mp3_parser_v1_sampling_frequency[4] =
{
  44100,
  48000,
  32000,
  0
};

static const uint32_t mp3_parser_v2_sampling_frequency[4] =
{
  22500,
  24000,
  16000,
  0
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* External APIs */

int32_t Mp3Parser_initialize(FAR MP3PARSER_Handle *ptr_hndl,
                             FAR CMN_SimpleFifoHandle *simple_fifo_handler,
                             FAR MP3PARSER_Config *config);
int32_t Mp3Parser_pollSingleFrame(FAR MP3PARSER_Handle *ptr_hndl,
                                  FAR uint8_t *out_buffer,
                                  uint32_t out_buffer_size,
                                  FAR uint32_t *out_frame_size,
                                  FAR int32_t *ready_to_extract_frames);
int32_t Mp3Parser_finalize(FAR MP3PARSER_Handle *ptr_hndl);
int32_t Mp3Parser_getSamplingRate(FAR MP3PARSER_Handle *ptr_hndl,
                                  FAR uint32_t *ptr_sampling_rate);

/* Internal functions */

uint32_t mp3parser_extract_frame(FAR MP3PARSER_Handle *ptr_hndl,
                                 FAR Mp3ParserLocalInfo *ptr_info,
                                 FAR uint8_t *out_buffer);
Mp3ParserReturnValueOfSyncSearch mp3parser_distribute_processing( \
                                   FAR MP3PARSER_Handle *ptr_hndl,
                                   FAR Mp3ParserLocalInfo *ptr_info);
int32_t mp3parser_get_frameheader(FAR MP3PARSER_Handle *ptr_hndl,
                                  FAR Mp3ParserLocalInfo *ptr_info,
                                  FAR uint8_t *ptr_local_buff);
Mp3ParserReturnValueOfFile mp3parser_buffer_check_tag( \
                             FAR MP3PARSER_Handle *ptr_hndl,
                             FAR Mp3ParserLocalInfo *ptr_info);
Mp3ParserReturnValueOfFile mp3parser_buffer_check_tag_v1( \
                             FAR MP3PARSER_Handle *ptr_hndl,
                             FAR Mp3ParserLocalInfo *ptr_info);

#endif /* __MODULES_AUDIO_INCLUDE_COMMON_MP3PARSER_H */
