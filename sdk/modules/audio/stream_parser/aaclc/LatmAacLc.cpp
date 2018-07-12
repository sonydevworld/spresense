/****************************************************************************
 * modules/audio/stream_parser/aaclc/LatmAacLc.cpp
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

#include <stdint.h>
#ifdef WINDOWS
#  include <sys/stat.h>
#  include <io.h>
#endif
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "common/LatmAacLc.h"

/* Syncword to use with LATM / LOAS.
 * (Compare after obtaining with 11bit value -> long value)
 */

#define LATM_LENGTH_OF_SYNCWORD  11
#define LATM_LENGTH_OF_FRAME     13
#define LATM_SYNCWORD_LOAS       0x2B7        /* -010 1011 0111 */
#define LATM_SYNCWORD_EXT_LOAS   LATM_SYNCWORD_LOAS
#define LATM_SYNCWORD_EXT_PS     0x548        /* -101 0100 1000 */


/* Channel_Configuration[ISO standard] */

#define LATM_CC_IDX_PCE    0        /* call program_config_element() */

/* SamplingFrequencyIndex[ISO standard] */

#define LATM_FS_IDX_ESC    0xf      /* ESC value */

#define LOCAL_CHECK_NG    -1

/* bit length */

#define LATM_BIT_OF_BYTE  8
#define LATM_BIT_OF_LONG  32
#define LATM_VAL_OF_5BIT  0x1F

/* AudioObjectType[ISO standard] */

enum latm_aot_e
{
  AotNull = 0,
  AotAacMain,                     /*  1:AAC-MAIN */
  AotAacLc,                       /*  2:AAC-LC */
  AotAacSsr,                      /*  3:AAC-SSR */
  AotAacLtp,                      /*  4:AAC-LTP */
  AotSbr,                         /*  5:AAC-SBR */
  AotAacScalable,                 /*  6:AAC scalable */
  AotTwinVq,                      /*  7:TwinVQ */
  AotCelp,                        /*  8:CELP */
  AotHvxc,                        /*  9:HVXC */
  AotReserved10,
  AotReserved11,
  AotTtsi,                        /* 12:TTSI */
  AotMainSynthesis,               /* 13:Main synthesis */
  AotWavetableSynthesis,          /* 14:Wavetable synthesis */
  AotGeneralMidi,                 /* 15:General MIDI */
  AotAlgorithmicSynthesisAudioFx, /* 16:Algorithmic synthesis and Audio FX */
  AotErAacLc,                     /* 17:ER AAC-LC */
  AotReserved18,
  AotErAacLtp,                    /* 19:ER AAC-LTP */
  AotErAacScalable,               /* 20:ER AAC scalable */
  AotErTwinVq,                    /* 21:ER TwinVQ */
  AotErBsac,                      /* 22:ER BSAC */
  AotErAacLd,                     /* 23:ER AAC-LD */
  AotErCelp,                      /* 24:ER CELP */
  AotErHvxc,                      /* 25:ER HVXC */
  AotErHiln,                      /* 26:ER HILN */
  AotErParametric,                /* 27:ER Parametric */
  AotSsc,                         /* 28:SSC */
  AotPs,                          /* 29:PS */
  AotMpegSurround,                /* 30:MPEG Surround */
  AotEscape,                      /* 31:(escape) */
  AotLayer1,                      /* 32:Layer-1 */
  AotLayer2,                      /* 33:Layer-2 */
  AotLayer3,                      /* 34:Layer-3 */
  AotDst,                         /* 35:DST */
  AotAls,                         /* 36:ALS */
  AotSls,                         /* 37:SLS */
  AotSlsNonCore,                  /* 38:SLS non-core */
  AotErAacEld,                    /* 39:ER AAC-ELD */
  AotSmrSimple,                   /* 40:SMR Simple */
  AotSmrMain,                     /* 41:SMR Main */
  AotMax
};
typedef enum latm_aot_e LatmAot;

/* framLengthType[ISO standard] */

enum latm_flt_e
{
  FltVariablePayload = 0, /* 0:Payload with variable frame length */
  FltFixedPayload,  /* 1:Payload with fixed frame length */
  FltReserved2,     /* 2:(reserved) */
  FltCelp1of2,      /* 3:Payload CELP with one of 2 kinds of frame length */
  FltCelpFixed,     /* 4:Payload CELP or ER CELP with fixed length */
  FltErCelp1of4,    /* 5:Payload ER CELP with one of 4 kinds frame length */
  FltHvxcFixed,     /* 6:Payload HVXC or ER HVXC with fixed frame length */
  FltHvxc1of4,      /* 7:Payload HVXC or ER HVXC with one of
                     *    4 kinds frame length
                     */
  FltMax
};
typedef enum latm_flt_e LatmFlt;

/* Shared information to be used in each function in this tool. */

struct latm_local_info_s
{
  uint8_t  *ptr_check_latm;    /* Current pointer */
  uint32_t  total_bit_length;  /* Cumulative bit length read in */
  uint32_t  stream_cnt;        /* = StreamID */

  /* Temporarily use for data passing purpose. */

  uint32_t  temp_long;        /* long value */
};
typedef struct latm_local_info_s LatmLocalInfo;

/* Information necessary only when using chunk with Payload. */

struct use_chunk_info_s
{
  uint8_t num_chunk;
  uint8_t reserve;
  int8_t stream_cnt_chunk[LATM_MAX_STREAM_ID]; /* Subscript = chunk number
                                                * (streamID - 1)
                                                */
};
typedef struct use_chunk_info_s UseChunkInfo;

/* Mask value at bit processing. */

static const uint8_t BitMaskLessByteTable[9] =
{
  0x00,  /* 0-bit. */
  0x80,  /* 1-bit. */
  0xC0,  /* 2-bit. */
  0xE0,  /* 3-bit. */
  0xF0,  /* 4-bit. */
  0xF8,  /* 5-bit. */
  0xFC,  /* 6-bit. */
  0xFE,  /* 7-bit. */
  0xFF   /* 8-bit. */
};

#ifdef WINDOWS
/*--------------------------------------------------------------------------*/
static inline uint32_t convByteToLong(uint8_t byte_value[], uint8_t max_byte)
{
  uint32_t rtn_value = 0;

  for (int32_t i = 0, j = max_byte - 1; i < max_byte; i++, j--)
    {
      rtn_value |= (uint32_t)(byte_value[i] << (j * LATM_BIT_OF_BYTE));
    }

  return rtn_value;
}
#else
/*--------------------------------------------------------------------------*/
static inline uint32_t convByteToLong(uint8_t byte_value[], uint8_t max_byte)
{
  uint32_t rtn_value = 0;

  for (int32_t i = 0; i < max_byte; i++)
    {
      rtn_value |= (uint32_t)(byte_value[i] << (i * LATM_BIT_OF_BYTE));
    }

  return rtn_value;
}
#endif

/*--------------------------------------------------------------------------*/
static uint8_t bitReadLessByte(LatmLocalInfo *ptr_info,
                               uint32_t length_for_read)
{
  uint8_t rtn_value = 0;

  uint8_t modulo_bit =
    (uint8_t)(ptr_info->total_bit_length % LATM_BIT_OF_BYTE);

  /* Check if it spans 2 bytes.(modulo_bit is the start bit) */

  if ((LATM_BIT_OF_BYTE - modulo_bit) >= (uint8_t)length_for_read)
    {
      /* When it does not span 2 byte. */

      rtn_value =
        (*(ptr_info->ptr_check_latm) &
          (BitMaskLessByteTable[length_for_read] >> modulo_bit));
      ptr_info->total_bit_length += length_for_read;

      if (ptr_info->total_bit_length % LATM_BIT_OF_BYTE)
        {
          rtn_value >>=
            (LATM_BIT_OF_BYTE - (ptr_info->total_bit_length %
              LATM_BIT_OF_BYTE));
        }
    }
  else
    {
      /* Read 1st byte. */

      rtn_value =
        (*(ptr_info->ptr_check_latm) & ~BitMaskLessByteTable[modulo_bit]);

      /* Since the return value is 1 byte, it shifts to the upper bit side. */

      rtn_value <<= (length_for_read - (LATM_BIT_OF_BYTE - modulo_bit));

      /* If total_bit_length is last added with length_for_read,
       * this line is not needed
       */

      ptr_info->total_bit_length += (LATM_BIT_OF_BYTE - modulo_bit);

      /* Advance pointer for byte again. */

      (ptr_info->ptr_check_latm)++;

      /* Calculate the number of bits in 2nd byte. */

      modulo_bit = (length_for_read - (LATM_BIT_OF_BYTE - modulo_bit));

      /* Reads the 2nd byte and sets the value combined with 1st byte
       * as the return value.
       */

      rtn_value |=
        ((*(ptr_info->ptr_check_latm) & BitMaskLessByteTable[modulo_bit]) >>
          (LATM_BIT_OF_BYTE - modulo_bit));
      ptr_info->total_bit_length += modulo_bit;
    }

  /* Advance the pointer when there are no remaining bits after processing. */

  if (!(ptr_info->total_bit_length % LATM_BIT_OF_BYTE))
    {
      (ptr_info->ptr_check_latm)++;
    }

  return rtn_value;
}

/*--------------------------------------------------------------------------*/
static uint32_t bitReadLessLong(LatmLocalInfo *ptr_info,
                                uint32_t length_for_read)
{
  uint32_t i = 0;
  uint8_t  byte_value[4] =
  {
    0
  };

  uint8_t max_byte = (length_for_read / LATM_BIT_OF_BYTE);
  uint8_t modulo_bit = (length_for_read % LATM_BIT_OF_BYTE);

  /* Read the bit remainder first. */

  if (modulo_bit)
    {
      byte_value[0] = bitReadLessByte(ptr_info, modulo_bit);
      i = 1;
      max_byte++;
    }

  /* Process remaining by byte unit.
   * (ptr_check_latm and total_bit_length are updated in bitReadLessByte ())
   */

  while ((i < (int32_t)max_byte) && (i < sizeof(byte_value)))
    {
      byte_value[i] = bitReadLessByte(ptr_info, LATM_BIT_OF_BYTE);
      i++;
    }

  /* Convert byte array to long. */

  uint32_t rtn_long_value = convByteToLong(&byte_value[0], max_byte);

  return rtn_long_value;
}

/*--------------------------------------------------------------------------*/
static uint8_t searchStreamID(InfoStreamMuxConfig *ptr_stream_mux_config,
                              uint8_t prog_chunk_indx,
                              uint8_t lay_chunk_indx)
{
  uint8_t rtn_value = LOCAL_CHECK_NG;

  /* Search for streamID corresponding to progCIndx and layCIndx,
   * and return streamID when found.
   */

  for (int32_t i = LATM_MIN_STREAM_ID;
        i < (int32_t)ptr_stream_mux_config->max_stream_id; i++)
    {
      if (ptr_stream_mux_config->info_stream_id[i].stream_id >= 0)
        {
          /* If stream_id value is used, check prog and lay. */

          if ((ptr_stream_mux_config->
               info_stream_id[i].prog == prog_chunk_indx) &&
                 (ptr_stream_mux_config->
                   info_stream_id[i].lay == lay_chunk_indx))
            {
              /* If program and layer match, the corresponding streamID
               * is set to return value.
               */

              rtn_value = ptr_stream_mux_config->info_stream_id[i].stream_id;
              break;
            }
        }
    }

  return rtn_value;
}

/*--------------------------------------------------------------------------*/
static void
  clearInfoStreamMuxConfigTable(InfoStreamMuxConfig *ptr_stream_mux_config)
{
  int32_t i = 0;

  /* Clear part of information.
   * (Only necessary parts.Do not clear bits_to_decode etc)
   */

  for (i = 0; i < LATM_VAL_OF_4BIT + 1; i++)
    {
      ptr_stream_mux_config->num_layer[i] = 0;
    }
  for (i = 0; i < LATM_MAX_STREAM_ID; i++)
    {
      ptr_stream_mux_config->prog_stream_indx[i] = 0xFF;
      ptr_stream_mux_config->lay_stream_indx[i]  = 0xFF;
    }
}

/*--------------------------------------------------------------------------*/
static int copyAudioSpecificConfig(InfoStreamMuxConfig *ptr_stream_mux_config,
                                   uint8_t stream_cnt)
{
  int prev_cnt = (stream_cnt - 1);

  if (prev_cnt > LATM_MIN_STREAM_ID)
    {
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.audio_object_type =
          ptr_stream_mux_config->
            info_stream_id[prev_cnt].asc.audio_object_type;
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.channel_configuration =
          ptr_stream_mux_config->
            info_stream_id[prev_cnt].asc.channel_configuration;
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.sampling_frequency_index =
          ptr_stream_mux_config->
            info_stream_id[prev_cnt].asc.sampling_frequency_index;
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.sbr_present_flag =
          ptr_stream_mux_config->
            info_stream_id[prev_cnt].asc.sbr_present_flag;
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.ps_present_flag =
          ptr_stream_mux_config->
            info_stream_id[prev_cnt].asc.ps_present_flag;
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.extension_audio_object_type =
          ptr_stream_mux_config->
            info_stream_id[prev_cnt].asc.extension_audio_object_type;
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.extension_channel_configuration =
          ptr_stream_mux_config->
            info_stream_id[prev_cnt].asc.extension_channel_configuration;
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.extension_sampling_frequency_index =
          ptr_stream_mux_config->
            info_stream_id[prev_cnt].asc.extension_sampling_frequency_index;
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.extension_sampling_frequency =
          ptr_stream_mux_config->
            info_stream_id[prev_cnt].asc.extension_sampling_frequency;
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.sampling_frequency =
          ptr_stream_mux_config->
            info_stream_id[prev_cnt].asc.sampling_frequency;
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.pce_object_type =
          ptr_stream_mux_config->
            info_stream_id[prev_cnt].asc.pce_object_type;
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.pce_sampling_frequency_index =
          ptr_stream_mux_config->
            info_stream_id[prev_cnt].asc.pce_sampling_frequency_index;
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.config_length_flag =
          ptr_stream_mux_config->
            info_stream_id[prev_cnt].asc.config_length_flag;
      ptr_stream_mux_config->
        info_stream_id[stream_cnt].asc.config_length =
          ptr_stream_mux_config->info_stream_id[prev_cnt].asc.config_length;
    }

  return prev_cnt;
}

/*--------------------------------------------------------------------------*/
static bool serachNextBitsForSyncWord(LatmLocalInfo *ptr_info,
                                      uint32_t sync_length,
                                      uint32_t search_word)
{
  LatmLocalInfo backup;

  /* Search for syncword while bit shifting.
   * (Avoid searching indefinitely, with 8 bits as the upper limit)
   */

  for (int32_t i = 0; i < 8; i++)
    {
      /* Pointers and cumulative bit are updated,
       * so copy it temporarily and use it.
       */

      backup.total_bit_length = (ptr_info->total_bit_length + i);
      uint32_t byte_of_total = (backup.total_bit_length / 8);

      /* Fit the read pointer to the cumulative bit. */

      backup.ptr_check_latm = (ptr_info->ptr_check_latm + byte_of_total);

      /* Read specified bit length. */

      uint32_t dummy_read = bitReadLessLong(&backup, sync_length);
      if (dummy_read == search_word)
        {
          /* Set the next bit position of syncword. */

          ptr_info->temp_long = backup.total_bit_length;
          return true;
        }
    }

  return false;
}

/*--------------------------------------------------------------------------*/
static int32_t AACLC_checkLOAS(LatmLocalInfo *ptr_info)
{
  LatmLocalInfo temp;
  int32_t length_latm_frame = 0;

  /* Pointers and cumulative bit are updated,
   * so copy it temporarily and use it.
   */

  temp.ptr_check_latm = ptr_info->ptr_check_latm;
  temp.total_bit_length = ptr_info->total_bit_length;

  /* Search syncword. */

  if (serachNextBitsForSyncWord(&temp,
                                LATM_LENGTH_OF_SYNCWORD,
                                LATM_SYNCWORD_LOAS))
    {
      /* In the case of syncword, get the LATM frame length. */

      temp.total_bit_length = temp.temp_long;
      temp.ptr_check_latm =
        (ptr_info->ptr_check_latm +
          (temp.total_bit_length / LATM_BIT_OF_BYTE));

      /* Get the frame length after updating pointer and cumulative bit. */

      length_latm_frame = bitReadLessLong(&temp, LATM_LENGTH_OF_FRAME);
    }

  return length_latm_frame;
}

/*--------------------------------------------------------------------------*/
static int32_t iso_byteAlignment(LatmLocalInfo *ptr_info)
{
  /* When this function is terminated, idle-read so that the cumulative
   * bit length becomes "positive bit (no bit remainder)"
   */

  uint32_t modulo_bit = (8 - (ptr_info->total_bit_length % LATM_BIT_OF_BYTE));
  if (modulo_bit)
    {
      /* Idle read . */

      bitReadLessByte(ptr_info, modulo_bit);
    }

  return modulo_bit;
}

/*--------------------------------------------------------------------------*/
static uint32_t isoLatmGetValue(LatmLocalInfo *ptr_info)
{
  uint32_t helper_value = 0;

  uint8_t bytes_for_value = bitReadLessByte(ptr_info, 2);

  /* Below is the ISO standard. */

  for (int32_t i = 0; i <= (int32_t)bytes_for_value; i++)
    {
      uint8_t value_tmp = bitReadLessByte(ptr_info, 8);
      helper_value *= (2 ^ 8);
      helper_value += value_tmp;
    }

  return helper_value;
}

/*--------------------------------------------------------------------------*/
static int32_t
  isoProgramConfigElement(LatmLocalInfo *ptr_info,
                          InfoStreamMuxConfig *ptr_stream_mux_config)
{
  int32_t bit_length = 0;

  /* At the present moment, only the bit length is obtained by reading
   * each bit.(Hold object_type and sampling_frequency_index)
   */

  /* Element_instance_tag processing. */

  uint32_t dummy_read = bitReadLessByte(ptr_info, 4);
  bit_length += 4;

  /* Object_type processing. */

  ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
    asc.pce_object_type = bitReadLessByte(ptr_info, 2);
  bit_length += 2;

  /* Sampling_frequency_index processing. */

  ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
    asc.pce_sampling_frequency_index = bitReadLessByte(ptr_info, 4);
  bit_length += 4;

  /* Num_front_channel_elements processing. */

  uint32_t num_front_channel_elements = bitReadLessByte(ptr_info, 4);
  bit_length += 4;

  /* Num_side_channel_elements processing. */

  uint32_t num_side_channel_elements = bitReadLessByte(ptr_info, 4);
  bit_length += 4;

  /* Num_back_channel_elements processing. */

  uint32_t num_back_channel_elements = bitReadLessByte(ptr_info, 4);
  bit_length += 4;

  /* Bum_lfe_channel_elements processing. */

  uint32_t num_lfe_channel_elements = bitReadLessByte(ptr_info, 2);
  bit_length += 2;

  /* Num_assoc_data_elements processing. */

  uint32_t num_assoc_data_elements = bitReadLessByte(ptr_info, 3);
  bit_length += 3;

  /* Num_valid_cc_elements processing. */

  uint32_t num_valid_cc_elements = bitReadLessByte(ptr_info, 4);
  bit_length += 4;

  /* Mono_mixdown_present processing. */

  dummy_read = bitReadLessByte(ptr_info, 1);
  bit_length++;
  if (dummy_read)
    {
      /* Mono_mixdown_element_number processing. */

      dummy_read = bitReadLessByte(ptr_info, 4);
      bit_length += 4;
    }

  /* Stereo_mixdown_present processing. */

  dummy_read = bitReadLessByte(ptr_info, 1);
  bit_length++;
  if (dummy_read)
    {
      /* Stereo_mixdown_element_number processing. */

      dummy_read = bitReadLessByte(ptr_info, 4);
      bit_length += 4;
    }

  /* Matrix_mixdown_idx_present processing.. */

  dummy_read = bitReadLessByte(ptr_info, 1);
  bit_length++;
  if (dummy_read)
    {
      /* Matrix_mixdown_idx processing. */

      dummy_read = bitReadLessByte(ptr_info, 2);
      bit_length += 2;

      /* Pseudo_surround_enable processing. */
      dummy_read = bitReadLessByte(ptr_info, 1);
      bit_length += 1;
    }

  int32_t  i = 0;
  for (i = 0; i < (int32_t)num_front_channel_elements; i++)
    {
      /* Front_element_is_cpe[i] processing. */

      dummy_read = bitReadLessByte(ptr_info, 1);
      bit_length += 1;

      /* Front_element_tag_select[i] processing.. */

      dummy_read = bitReadLessByte(ptr_info, 4);
      bit_length += 4;
    }
  for (i = 0; i < (int32_t)num_side_channel_elements; i++)
    {
      /* Side_element_is_cpe[i] processing. */

      dummy_read = bitReadLessByte(ptr_info, 1);
      bit_length += 1;

      /* Side_element_tag_select[i] processing. */

      dummy_read = bitReadLessByte(ptr_info, 4);
      bit_length += 4;
    }
  for (i = 0; i < (int32_t)num_back_channel_elements; i++)
    {
      /* Back_element_is_cpe[i] processing. */

      dummy_read = bitReadLessByte(ptr_info, 1);
      bit_length += 1;

      /* Back_element_tag_select[i] processing. */

      dummy_read = bitReadLessByte(ptr_info, 4);
      bit_length += 4;
    }
  for (i = 0; i < (int32_t)num_lfe_channel_elements; i++)
    {
      /* Lfe_element_tag_select[i] processing. */

      dummy_read = bitReadLessByte(ptr_info, 4);
      bit_length += 4;
    }
  for (i = 0; i < (int32_t)num_assoc_data_elements; i++)
    {
      /* Assoc_data_element_tag_select[i] processing. */

      dummy_read = bitReadLessByte(ptr_info, 4);
      bit_length += 4;
    }
  for (i = 0; i < (int32_t)num_valid_cc_elements; i++)
    {
      /* Cc_element_is_ind_sw[i] processing. */

      dummy_read = bitReadLessByte(ptr_info, 1);
      bit_length += 1;

      /* Valid_cc_element_tag_select[i] processing. */
      dummy_read = bitReadLessByte(ptr_info, 4);
      bit_length += 4;
    }

  /* Byte_alignment processing. */

  bit_length += iso_byteAlignment(ptr_info);

  /* Comment_field_bytes processing. */

  uint32_t comment_field_bytes = bitReadLessByte(ptr_info, 8);
  bit_length += 8;
  for (i = 0; i < (int32_t)comment_field_bytes; i++)
    {
      /* Comment_field_data[i] processing. */

      dummy_read = bitReadLessByte(ptr_info, 8);
      bit_length += 8;
    }

  return bit_length;
}

/*--------------------------------------------------------------------------*/
static int32_t isoGASpecificConfig(LatmLocalInfo *ptr_info,
                                   InfoStreamMuxConfig *ptr_stream_mux_config)
{
  int32_t bit_length = 0;

  /* At the present moment, only the bit length is obtained by reading
   * each bit.(Do not keep information)
   */

  /* [ISO standard] frameLengthFlag processing. */

  uint32_t dummy_read = bitReadLessByte(ptr_info, 1);
  bit_length++;

  /* [ISO standard] dependsOnCoreCoder processing. */

  dummy_read = bitReadLessByte(ptr_info, 1);
  bit_length++;
  if (dummy_read)
    {
      /* [ISO standard] coreCoderDelay processing. */

      dummy_read = bitReadLessLong(ptr_info, 14);
      bit_length += 14;
    }

  /* [ISO standard] extensionFlag processing. */

  uint32_t extensionFlag = bitReadLessByte(ptr_info, 1);
  bit_length++;

  /* [ISO standard] channel_configuration processing. */

  if (ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
       asc.channel_configuration == LATM_CC_IDX_PCE)
    {
      /* [ISO standard] program_config_element() processing. */

      bit_length += isoProgramConfigElement(ptr_info,ptr_stream_mux_config);
    }
  else
    {
      /* Clear the program_config_element when it is not in use. */

      ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
        asc.pce_object_type = AotNull;
      ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
        asc.pce_sampling_frequency_index = 0;
    }

  /* [ISO standard] 6:AAC scalable 20:ER AAC scalable. */

  if ((ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
       asc.audio_object_type == AotAacScalable) ||
         (ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
           asc.audio_object_type == AotErAacScalable))
    {
      /* Since it is not supported this time, it is regarded as an error. */

      return LOCAL_CHECK_NG;
    }
  if (extensionFlag)
    {
      /* [ISO standard] 22:ER BSAC. */

      if (ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
           asc.audio_object_type == AotErBsac)
        {
          /* Since it is not supported this time,
           * it is regarded as an error.
           */

          return LOCAL_CHECK_NG;
        }

      /* [ISO standard] 17:ER AAC-LC 19:ER AAC-LTP 20:ER AAC scalable
       * 23:ER AAC-LD.
       */

      if ((ptr_stream_mux_config->
           info_stream_id[ptr_info->stream_cnt].asc.
             audio_object_type == AotErAacLc) ||
               (ptr_stream_mux_config->
                 info_stream_id[ptr_info->stream_cnt].
                   asc.audio_object_type == AotErAacLtp) ||
                    (ptr_stream_mux_config->
                      info_stream_id[ptr_info->stream_cnt].
                        asc.audio_object_type == AotErAacScalable) ||
                          (ptr_stream_mux_config->
                            info_stream_id[ptr_info->stream_cnt].
                              asc.audio_object_type == AotErAacLd))
        {
          /* Since it is not supported this time,
           * it is regarded as an error.
           */

          return LOCAL_CHECK_NG;
        }

      /* [ISO standard] extensionFlag3 processing. */

      dummy_read = bitReadLessByte(ptr_info, 1);
      bit_length++;
    }

  return bit_length;
}

/*--------------------------------------------------------------------------*/
static uint32_t isoGetAudioObjectType(LatmLocalInfo *ptr_info)
{
  /* According to ISO standard. */

  uint32_t audioObjectType = bitReadLessByte(ptr_info, 5);
  if (audioObjectType == LATM_VAL_OF_5BIT)
    {
      audioObjectType += bitReadLessByte(ptr_info, 6);
    }

  return audioObjectType;
}

/*--------------------------------------------------------------------------*/
static int32_t
  isoAudioSpecificConfig(LatmLocalInfo *ptr_info,
                         InfoStreamMuxConfig *ptr_stream_mux_config)
{
  int32_t bit_length = 0;

  /* Keep information in AudioSpecificConfig while conforming to ISO. */

  /* [ISO standard] GetAudioObjectType() processing. */

  ptr_stream_mux_config->
    info_stream_id[ptr_info->stream_cnt].asc.audio_object_type =
      isoGetAudioObjectType(ptr_info);
  if (ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
       asc.audio_object_type > LATM_VAL_OF_5BIT)
    {
      bit_length += 11;
    }
  else
    {
      bit_length += 5;
    }

  /* [ISO standard] samplingFrequencyIndex processing. */

  ptr_stream_mux_config->
   info_stream_id[ptr_info->stream_cnt].asc.sampling_frequency_index =
     bitReadLessByte(ptr_info, 4);
  bit_length += 4;

  /* [ISO standard] Check esc_value. */

  if (ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
       asc.sampling_frequency_index == LATM_FS_IDX_ESC)
    {
      /* [ISO standard] samplingFrequency processing. */

      ptr_stream_mux_config->
        info_stream_id[ptr_info->stream_cnt].asc.sampling_frequency =
          bitReadLessLong(ptr_info, 24);
      bit_length += 24;
    }
  else
    {
      /* Clear it when not in use. */

      ptr_stream_mux_config->
        info_stream_id[ptr_info->stream_cnt].asc.sampling_frequency = 0;
    }

  /* [ISO standard] channelConfiguration processing. */

  ptr_stream_mux_config->
    info_stream_id[ptr_info->stream_cnt].asc.channel_configuration =
      bitReadLessByte(ptr_info, 4);
  bit_length += 4;
  ptr_stream_mux_config->
    info_stream_id[ptr_info->stream_cnt].asc.ps_present_flag = (-1);
  ptr_stream_mux_config->
    info_stream_id[ptr_info->stream_cnt].asc.sbr_present_flag = (-1);

  /* [ISO standard] 5:SBR 29:PS. */

  if ((ptr_stream_mux_config->
       info_stream_id[ptr_info->stream_cnt].
         asc.audio_object_type == AotSbr) ||
           (ptr_stream_mux_config->
             info_stream_id[ptr_info->stream_cnt].
               asc.audio_object_type == AotPs))
    {
      ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
        asc.extension_audio_object_type = AotSbr;
      ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
        asc.sbr_present_flag = 1;
      if (ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
           asc.audio_object_type == AotPs)
        {
          ptr_stream_mux_config->
            info_stream_id[ptr_info->stream_cnt].asc.ps_present_flag = 1;
        }
      /* [ISO standard] extensionSamplingFrequencyIndex processing. */

      ptr_stream_mux_config->
        info_stream_id[ptr_info->stream_cnt].
          asc.extension_sampling_frequency_index =
            bitReadLessByte(ptr_info, 4);
      bit_length += 4;

      /* [ISO standard] Check esc_value. */

      if (ptr_stream_mux_config->
           info_stream_id[ptr_info->stream_cnt].
             asc.extension_sampling_frequency_index == LATM_FS_IDX_ESC)
        {
          /* [ISO standard] extensionSamplingFrequency processing. */

          ptr_stream_mux_config->
            info_stream_id[ptr_info->stream_cnt].
              asc.extension_sampling_frequency = bitReadLessLong(ptr_info, 24);
          bit_length += 24;
        }
      else
        {
          /* Clear it when not in use. */

          ptr_stream_mux_config->
            info_stream_id[ptr_info->stream_cnt].
              asc.extension_sampling_frequency = 0;
        }

      /* [ISO standard] GetAudioObjectType() processing.
       * Read AudioObjectType (second time) again.
       */

      ptr_stream_mux_config->
        info_stream_id[ptr_info->stream_cnt].asc.audio_object_type =
          isoGetAudioObjectType(ptr_info);

      /* This time, I expect that AAC-MAIN/AAC-LC will be set. */

      if (ptr_stream_mux_config->
           info_stream_id[ptr_info->stream_cnt].asc.audio_object_type >
             LATM_VAL_OF_5BIT)
        {
          bit_length += 11;
        }
      else
        {
          bit_length += 5;
        }

      /* [ISO standard] 22:ER BSAC. */

      if (ptr_stream_mux_config->
           info_stream_id[ptr_info->stream_cnt].asc.audio_object_type ==
             AotErBsac)
        {
          /* Since it is not supported this time,
           * it is regarded as an error.
           */

          return LOCAL_CHECK_NG;
        }
    }
  else
    {
      ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
        asc.extension_audio_object_type = AotNull;

      /* Also clear below. */

      ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
        asc.extension_sampling_frequency_index = 0;
      ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
        asc.extension_sampling_frequency = 0;
      ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
        asc.extension_channel_configuration = 0;
    }

  /* [ISO standard] Processing of audio_object_type. */

  switch (ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
           asc.audio_object_type)
    {
      case AotAacMain:
      case AotAacLc:
        {
          /* [ISO standard] GASpecificConfig() processing. */

          int32_t dummy_length =
            isoGASpecificConfig(ptr_info, ptr_stream_mux_config);
          if (dummy_length == LOCAL_CHECK_NG)
            {
              return LOCAL_CHECK_NG;
            }
          bit_length += dummy_length;
        }
        break;

      default:
        /* Since it is not supported this time, it is regarded as an error. */

        return LOCAL_CHECK_NG;
    }

  /* [ISO standard] Check extended data. */

  if ((ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
       asc.extension_audio_object_type != AotSbr) &&
         ((int32_t)(ptr_stream_mux_config->
           info_stream_id[ptr_info->stream_cnt].
             asc.config_length - bit_length) >= 16))
    {
      /* [ISO standard] syncExtensionType processing. */

      uint32_t sync_ext_type = bitReadLessLong(ptr_info, 11);
      bit_length += 11;

      /* [ISO standard] Check extended syncword. */

      if (sync_ext_type == LATM_SYNCWORD_EXT_LOAS)
        {
          /* [ISO standard] GetAudioObjectType() */

          ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
            asc.extension_audio_object_type = isoGetAudioObjectType(ptr_info);
          if (ptr_stream_mux_config->
               info_stream_id[ptr_info->stream_cnt].
                 asc.extension_audio_object_type > LATM_VAL_OF_5BIT)
            {
              bit_length += 11;
            }
          else
            {
              bit_length += 5;
            }

          /* [ISO standard] Check extended SBR. */

          switch (ptr_stream_mux_config->
                   info_stream_id[ptr_info->stream_cnt].
                     asc.extension_audio_object_type)
            {
              case AotSbr:
                /* [ISO standard] sbrPresentFlag processing. */

                ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
                  asc.sbr_present_flag = bitReadLessByte(ptr_info, 1);
                bit_length++;

                /* [ISO standard] Check acquisition SBR flag. */

                if (ptr_stream_mux_config->
                     info_stream_id[ptr_info->stream_cnt].
                       asc.sbr_present_flag)
                  {
                    /* [ISO standard]
                     * extensionSamplingFrequencyIndex processing.
                     */

                    ptr_stream_mux_config->
                      info_stream_id[ptr_info->stream_cnt].
                        asc.extension_sampling_frequency_index =
                          bitReadLessByte(ptr_info, 4);
                    bit_length += 4;
                  }
                else
                  {
                    /* Extended AOT is SBR but clears it because there
                     * is no config.
                     */

                    ptr_stream_mux_config->
                      info_stream_id[ptr_info->stream_cnt].
                        asc.extension_sampling_frequency_index = 0;
                    ptr_stream_mux_config->
                      info_stream_id[ptr_info->stream_cnt].
                        asc.extension_sampling_frequency = 0;
                  }

                /* [ISO standard] Check extended sampling rate index. */

                if (ptr_stream_mux_config->
                     info_stream_id[ptr_info->stream_cnt].
                       asc.extension_sampling_frequency_index ==
                         LATM_FS_IDX_ESC)
                  {
                    /* [ISO standard]
                     * extensionSamplingFrequency processing.
                     */

                    ptr_stream_mux_config->
                      info_stream_id[ptr_info->stream_cnt].
                        asc.extension_sampling_frequency =
                          bitReadLessLong(ptr_info, 24);
                    bit_length += 24;
                  }

                /* [ISO standard] bits_to_decode() >= 12. */

                if ((int32_t)(ptr_stream_mux_config->
                     info_stream_id[ptr_info->stream_cnt].
                       asc.config_length - bit_length) >= 12)
                  {
                    /* [ISO standard] syncExtensionType.
                     * According to the ISO standard, nextbits () is not used.
                     */

                    sync_ext_type = bitReadLessLong(ptr_info, 11);
                    bit_length += 11;
                    if (sync_ext_type == LATM_SYNCWORD_EXT_PS)
                      {
                        /* [ISO standard] psPresentFlag processing. */

                        ptr_stream_mux_config->
                          info_stream_id[ptr_info->stream_cnt].
                            asc.ps_present_flag =
                              bitReadLessByte(ptr_info, 1);
                        bit_length++;
                      }
                  }
                break;

              case AotErBsac:
                /* Since it is not supported this time,
                 * it is regarded as an error.
                 */

                return LOCAL_CHECK_NG;

              default:
                /* If it is not SBR, make it an error. */

                return LOCAL_CHECK_NG;
            }
        }
      else
        {
          /* Since there is a problem with the reliability of data,
           * it makes an error.
           */

          return LOCAL_CHECK_NG;
        }
    }

  return bit_length;
}

/*--------------------------------------------------------------------------*/
static int32_t isoPayloadLengthInfo(LatmLocalInfo *ptr_info,
                                   InfoStreamMuxConfig *ptr_stream_mux_config,
                                   UseChunkInfo *ptr_chunk_info)
{
  int32_t i = 0;
  uint32_t dummy_length = 0;

  if (ptr_stream_mux_config->all_streams_sametime_framing)
    {
      /* In the ISO standard, a loop of a two-dimensional
       * array with prog and lay.
       */

      for (i = LATM_MIN_STREAM_ID;
           i < (int32_t)ptr_stream_mux_config->max_stream_id; i++)
        {
          switch (ptr_stream_mux_config->info_stream_id[i].frame_length_type)
            {
              case FltVariablePayload:
                {
                  /* Since frameLength is set only when fixed length,
                   * it is not used in this case.
                   */

                  ptr_stream_mux_config->info_stream_id[i].frame_length = 0;
                  uint8_t tmp = 0;
                  do
                    {
                      tmp = bitReadLessByte(ptr_info, 8);
                      dummy_length += 8;

                      /* [ISO standard] MuxSlotLengthBytes processing*/

                      ptr_stream_mux_config->
                        info_stream_id[i].frame_length += tmp;
                    }
                  while (tmp==UINT8_MAX);
                }
                break;

              case FltCelp1of2:
              case FltErCelp1of4:
              case FltHvxc1of4:
                /* Since it is not supported this time,
                 * it is regarded as an error.
                 */

                return LOCAL_CHECK_NG;

              case FltFixedPayload:
                /* Fixed length does nothing. */

                break;

              default:
                /* Since it is not supported this time,
                 * it is regarded as an error.
                 */

                return LOCAL_CHECK_NG;
            }
        }
    }
  else
    {
      /* [ISO standard] numChunk processing. */

      ptr_chunk_info->num_chunk = bitReadLessByte(ptr_info, 4);
      dummy_length += 4;

      for (i = 0; i <= (int32_t)ptr_chunk_info->num_chunk; i++)
        {
          /* [ISO standard] streamIndx processing. */

          uint8_t tmp = bitReadLessByte(ptr_info, 4);
          dummy_length += 4;
          uint8_t prog = ptr_stream_mux_config->prog_stream_indx[tmp];
          uint8_t lay = ptr_stream_mux_config->lay_stream_indx[tmp];

          /* Search streamID corresponding to prog and lay. */

          ptr_chunk_info->stream_cnt_chunk[i] =
            searchStreamID(ptr_stream_mux_config, prog, lay);
          if (ptr_chunk_info->stream_cnt_chunk[i] == LOCAL_CHECK_NG)
            {
              /* Since streamID can not be found, it is regarded as
               * an error.
               */

              return LOCAL_CHECK_NG;
            }

          switch (ptr_stream_mux_config->
                   info_stream_id[(ptr_chunk_info->stream_cnt_chunk[i])].
                     frame_length_type)
            {
              case FltVariablePayload:
                {
                  /* Since frameLength is set only when fixed length,
                   * it is not used in this case.
                   */

                  ptr_stream_mux_config->
                    info_stream_id[(ptr_chunk_info->stream_cnt_chunk[i])].
                      frame_length = 0;
                  do
                    {
                      /* [ISO standard] tmp processing. */

                      tmp = bitReadLessByte(ptr_info, 8);
                      dummy_length += 8;
                      ptr_stream_mux_config->
                        info_stream_id[(ptr_chunk_info->stream_cnt_chunk[i])].
                          frame_length += tmp;
                    }
                  while (tmp==UINT8_MAX);

                  /* [ISO standard] AuEndFlag processing. */

                  tmp = bitReadLessByte(ptr_info, 1);
                  dummy_length += 1;
                }
                break;

              case FltCelp1of2:
              case FltErCelp1of4:
              case FltHvxc1of4:
                /* Since it is not supported this time,
                 * it is regarded as an error.
                 */

                return LOCAL_CHECK_NG;

              case FltFixedPayload:
                /* Fixed length does nothing. */

                break;

              default:
                /* Since it is not supported this time,
                 * it is regarded as an error.
                 */

                return LOCAL_CHECK_NG;
            }
        }
    }

  return dummy_length;
}

/*--------------------------------------------------------------------------*/
static int32_t isoPayloadMux(LatmLocalInfo *ptr_info,
                             InfoStreamMuxConfig *ptr_stream_mux_config,
                             UseChunkInfo *ptr_chunk_info)
{
  int32_t i          = 0;
  int32_t rtn_length = 0;
  uint32_t payload_length = 0;

  /* We do not call the payload (raw_data), so we only update
   * total_bit_length and ptr_check_latm.
   */

  if (ptr_stream_mux_config->all_streams_sametime_framing)
    {
      for (i = 0; i < (int32_t)ptr_stream_mux_config->max_stream_id; i++)
        {
          /* At fixed length, frameLength is byte long, so it is set to bit.
           * (According to the ISO standard, it is described that
           * frame length is added to frameLength by 20 and then x 8)
           */

          switch (ptr_stream_mux_config->info_stream_id[i].frame_length_type)
            {
              case FltFixedPayload:
                /* [ISO standard] Payload processing. */

                payload_length =
                 ((ptr_stream_mux_config->
                   info_stream_id[i].frame_length + 20) * LATM_BIT_OF_BYTE);
                break;

              case FltVariablePayload:
                /* For variable length, treat it as bit length as it is.
                 * (This local use, not the ISO standard)
                 */

                payload_length =
                  ptr_stream_mux_config->info_stream_id[i].frame_length;
                break;

              default:
                /* Since it is not supported this time,
                 * it is regarded as an error.
                 */

                return LOCAL_CHECK_NG;
            }

          /* Keep offset from LATM start of each payload. */

          ptr_stream_mux_config->info_stream_id[i].payload_offset =
            ptr_info->total_bit_length;

          /* Add bit length for payload. */

          ptr_info->total_bit_length += payload_length;
          rtn_length += payload_length;
        }
    }
  else
    {
      for (i = 0; i <= (int32_t)ptr_chunk_info->num_chunk; i++)
        {
          /* [ISO standard] Payload processing. */

          payload_length =
            ptr_stream_mux_config->
              info_stream_id[(ptr_chunk_info->stream_cnt_chunk[i])].
                frame_length;

          /* At fixed length, frameLength is byte long, so it is set to bit.
           * (According to the ISO standard, it is described that
           * frame length is added to frameLength by 20 and then x 8)
           */

          switch (ptr_stream_mux_config->
                   info_stream_id[(ptr_chunk_info->stream_cnt_chunk[i])].
                     frame_length_type)
            {
              case FltFixedPayload:
                payload_length = ((payload_length + 20) * LATM_BIT_OF_BYTE);
                break;

              case FltVariablePayload:
                /* For variable length, treat it as bit length as it is.
                 * (This local use, not the ISO standard)
                 */
                break;

              default:
                /* Since it is not supported this time,
                 * it is regarded as an error.
                 */

                return LOCAL_CHECK_NG;
            }

          /* Keep offset from LATM start of each payload. */

          ptr_stream_mux_config->
            info_stream_id[(ptr_chunk_info->stream_cnt_chunk[i])].
              payload_offset = ptr_info->total_bit_length;

          /* Add bit length for payload. */

          ptr_info->total_bit_length += payload_length;
          rtn_length += payload_length;
        }
    }

  /* Update ptr_check_latm. */

  ptr_info->ptr_check_latm += (ptr_info->total_bit_length / LATM_BIT_OF_BYTE);

  return rtn_length;
}

/*--------------------------------------------------------------------------*/
static int32_t isoStreamMuxConfig(LatmLocalInfo *ptr_info,
                                  InfoStreamMuxConfig *ptr_stream_mux_config)
{
  uint32_t old_length = ptr_info->total_bit_length;

  /* Since the new StreamMuxConfig information is set,
   * the old information is cleared.
   */

  clearInfoStreamMuxConfigTable(ptr_stream_mux_config);

  /* [ISO standard] audioMuxVersion processing. */

  ptr_stream_mux_config->audio_muxversion = bitReadLessByte(ptr_info, 1);

  if (!ptr_stream_mux_config->audio_muxversion)
    {
      ptr_stream_mux_config->audio_muxversion_a = 0;
    }
  else
    {
      /* [ISO standard] audioMuxVersionA processing. */

      ptr_stream_mux_config->audio_muxversion_a =
        bitReadLessByte(ptr_info, 1);
    }

  int32_t dummy_length = 0;
  uint32_t dummy_read = 0;

  /* [ISO standard] audioMuxVersionA processing. */

  if (!(ptr_stream_mux_config->audio_muxversion_a))
    {
      if (ptr_stream_mux_config->audio_muxversion)
        {
          /* [ISO standard] LatmGetValue processing. */

          dummy_read = isoLatmGetValue(ptr_info);
        }

      /* Clear streamCnt at the same position as the ISO standard. */

      ptr_info->stream_cnt = LATM_MIN_STREAM_ID;

      /* [ISO standard] allStreamsSameTimeFraming processing. */

      ptr_stream_mux_config->all_streams_sametime_framing =
        bitReadLessByte(ptr_info, 1);

      /* [ISO standard] numSubFrames processing. */

      ptr_stream_mux_config->num_sub_frames = bitReadLessByte(ptr_info, 6);

      /* [ISO standard] numProgram processing. */

      ptr_stream_mux_config->num_program = bitReadLessByte(ptr_info, 4);

      ptr_stream_mux_config->info_stream_id[ptr_info->stream_cnt].
        stream_id = (-1);

      for (int32_t prog = 0;
            prog <= ((int32_t)ptr_stream_mux_config->num_program);
              prog++)
        {
          /* [ISO standard] numLayer processing. */

          ptr_stream_mux_config->num_layer[ptr_info->stream_cnt] =
            bitReadLessByte(ptr_info, 3);

          /* Although it is the upper limit value of the loop,
           * since stream_cnt changes within the loop, use another variable.
           */

          int32_t tmp_value =
            ptr_stream_mux_config->num_layer[ptr_info->stream_cnt];
          for (int32_t lay = 0; lay <= tmp_value; lay++)
            {
              /* [ISO standard] Only the following two items have
               * streamCnt not equal streamID.
               */

              ptr_stream_mux_config->
                prog_stream_indx[ptr_info->stream_cnt] = prog;
              ptr_stream_mux_config->
                lay_stream_indx[ptr_info->stream_cnt] = lay;

              /* Subsequent items are synchronized with streamID:
               * In the ISO standard, StreamID [prog] [lay] is used,
               * but 16 x 8 two-dimensional array (=128) of 16 x 8 is useless,
               * so it is not used.
               */

              ptr_stream_mux_config->
                info_stream_id[ptr_info->stream_cnt].stream_id =
                  ptr_info->stream_cnt;
              ptr_stream_mux_config->
                info_stream_id[ptr_info->stream_cnt].prog = prog;
              ptr_stream_mux_config->
                 info_stream_id[ptr_info->stream_cnt].lay = lay;
              if (!prog && !lay)
                {
                  /* [ISO standard] First time do not get from bit. */

                  ptr_stream_mux_config->
                    info_stream_id[ptr_info->stream_cnt].use_same_config = 0;
                }
              else
                {
                  /* [ISO standard] useSameConfig processing. */

                  ptr_stream_mux_config->
                    info_stream_id[ptr_info->stream_cnt].use_same_config =
                      bitReadLessByte(ptr_info, 1);
                }

              /* [ISO standard] When AudioSpecificConfig exists. */

              if (!(ptr_stream_mux_config->
                   info_stream_id[ptr_info->stream_cnt].use_same_config))
                {
                  /* Config_length is set from the user side. */

                  if (ptr_stream_mux_config->
                       info_stream_id[ptr_info->stream_cnt].asc.
                         config_length_flag != LATM_ENABLE_CONFIG_LENGTH)
                    {
                      /* When config_length is invalid,
                       * clear config_length internally.
                       */

                      ptr_stream_mux_config->
                        info_stream_id[ptr_info->stream_cnt].asc.
                          config_length = 0;
                    }
                  if (!ptr_stream_mux_config->audio_muxversion)
                    {
                      /* [ISO standard] AudioSpecificConfig processing. */

                      dummy_length =
                        isoAudioSpecificConfig(ptr_info,
                                               ptr_stream_mux_config);
                      if (dummy_length == LOCAL_CHECK_NG)
                        {
                          /* Since it is not supported this time,
                           * it is regarded as an error.
                           */

                          return LOCAL_CHECK_NG;
                        }
                    }
                  else
                    {
                      /* [ISO standard] LatmGetValue processing. */

                      int32_t asc_length = isoLatmGetValue(ptr_info);

                      /* [ISO standard] AudioSpecificConfig processing. */

                      dummy_length =
                        isoAudioSpecificConfig(ptr_info,
                                               ptr_stream_mux_config);
                      if (dummy_length == LOCAL_CHECK_NG)
                        {
                          /* Since it is not supported this time,
                           * it is regarded as an error.
                           */

                          return LOCAL_CHECK_NG;
                        }
                      asc_length -= dummy_length;
                      if (asc_length > 0)
                        {
                          /* [ISO standard] fillBits processing. */

                          if (asc_length >= LATM_BIT_OF_LONG)
                            {
                              /* Read in 4 bytes at a time. */

                              for (int32_t i = 0;
                                    i < (int32_t)(asc_length /
                                      LATM_BIT_OF_LONG); i++)
                                {
                                  dummy_read =
                                    bitReadLessLong(ptr_info,
                                                    LATM_BIT_OF_LONG);
                                }
                            }

                          /* Read the remainder bit of asc_length. */

                          if (asc_length % LATM_BIT_OF_LONG)
                            {
                              dummy_read =
                                bitReadLessLong(ptr_info,
                                                (asc_length %
                                                LATM_BIT_OF_LONG));
                            }
                        }
                    }
                }
              else
                {
                  /* When useSameConfig = 1:
                   * Copy last minute's AudioSpecificConfig data to this time.
                   */

                  if (copyAudioSpecificConfig(ptr_stream_mux_config,
                                              ptr_info->stream_cnt) <
                                                LATM_MIN_STREAM_ID)
                    {
                      /* Since it is not supported this time,
                       * it is regarded as an error.
                       */

                      return LOCAL_CHECK_NG;
                    }
                }

              /* [ISO standard] FrameLengthType processing. */

              ptr_stream_mux_config->
                info_stream_id[ptr_info->stream_cnt].frame_length_type =
                  bitReadLessByte(ptr_info, 3);

              /* [ISO standard] Sort by FrameLengthType. */

              switch (ptr_stream_mux_config->
                       info_stream_id[ptr_info->stream_cnt].frame_length_type)
                {
                  case FltVariablePayload:
                    /* [ISO standard] latmBufferFullness processing. */

                    ptr_stream_mux_config->
                      info_stream_id[ptr_info->stream_cnt].
                        latm_buffer_fullness = bitReadLessByte(ptr_info, 8);
                    if (!ptr_stream_mux_config->all_streams_sametime_framing)
                      {
                        if ((ptr_stream_mux_config->
                             info_stream_id[ptr_info->stream_cnt].asc.
                               audio_object_type == AotAacScalable ||
                                 ptr_stream_mux_config->
                                   info_stream_id[ptr_info->stream_cnt].asc.
                                     audio_object_type == AotErAacScalable) &&
                                      (ptr_stream_mux_config->
                                        info_stream_id[(ptr_info->
                                          stream_cnt - 1)].asc.
                                            audio_object_type == AotCelp ||
                                              ptr_stream_mux_config->
                                                info_stream_id[(ptr_info->
                                                  stream_cnt - 1)].asc.
                                                    audio_object_type ==
                                                      AotErCelp))
                          {
                            /* Since it is not supported this time,
                             * it is regarded as an error.
                             */

                            return LOCAL_CHECK_NG;
                          }
                      }
                     break;

                  case FltFixedPayload:
                    /* [ISO standard] FrameLength processing. */

                    ptr_stream_mux_config->
                      info_stream_id[ptr_info->stream_cnt].frame_length =
                        bitReadLessLong(ptr_info, 9);

                    /* Clear unused items. */

                    ptr_stream_mux_config->
                      info_stream_id[ptr_info->stream_cnt].
                        latm_buffer_fullness = 0;
                    break;

                  case FltCelp1of2:
                  case FltCelpFixed:
                  case FltErCelp1of4:
                    /* Since it is not supported this time,
                     * it is regarded as an error.
                     */

                    return LOCAL_CHECK_NG;

                  case FltHvxcFixed:
                  case FltHvxc1of4:
                    /* Since it is not supported this time,
                     * it is regarded as an error.
                     */

                    return LOCAL_CHECK_NG;

                  default:
                    /* Since it is not supported this time,
                     * it is regarded as an error.
                     */

                    return LOCAL_CHECK_NG;
                }

              /* [ISO standard] streamCnt processing. */

              ptr_info->stream_cnt++;

              /* It is not in the ISO standard, but to make sure it
               * makes a maximum check.
               */

              if (ptr_info->stream_cnt >= LATM_MAX_STREAM_ID)
                {
                  return LOCAL_CHECK_NG;
                }
            }
        }

      ptr_stream_mux_config->max_stream_id = ptr_info->stream_cnt;

      /* [ISO standard] otherDataPresent processing. */

      ptr_stream_mux_config->other_data_present =
        bitReadLessByte(ptr_info, 1);
      if (ptr_stream_mux_config->other_data_present)
        {
          if (ptr_stream_mux_config->audio_muxversion)
            {
              /* [ISO standard] LatmGetValue processing. */

              ptr_stream_mux_config->other_data_len_bits =
                isoLatmGetValue(ptr_info);
            }
          else
            {
              /* Implemented according to ISO standard. */

              ptr_stream_mux_config->other_data_len_bits = 0;
              do
                {
                  ptr_stream_mux_config->other_data_len_bits *= (2 ^ 8);

                  /* [ISO standard] otherDataLenEsc processing. */

                  dummy_read = bitReadLessByte(ptr_info, 1);

                  /* [ISO standard] otherDataLenTmp processing. */

                  ptr_stream_mux_config->other_data_len_bits +=
                    bitReadLessByte(ptr_info, 8);
                }
              while (dummy_read);
            }
        }
      else
        {
          /* Clear when not in use. */

          ptr_stream_mux_config->other_data_len_bits = 0;
        }

      /* [ISO standard] crcCheckPresent processing. */

      dummy_read = bitReadLessByte(ptr_info, 1);
      if (dummy_read)
        {
          /* [ISO standard] crcCheckSum processing. */

          dummy_read = bitReadLessByte(ptr_info, 8);
        }
    }
  else
    {
      return LOCAL_CHECK_NG;
    }

  /* Calculate isoStreamMuxConfig() length from cumulative bit. */

  dummy_length = (ptr_info->total_bit_length - old_length);

  return dummy_length;
}

/*--------------------------------------------------------------------------*/
static int32_t isoAudioMuxElement(LatmLocalInfo *ptr_info,
                                  InfoStreamMuxConfig *ptr_stream_mux_config)
{
  int32_t dummy_length = 0;
  uint32_t rtn_length = 0;
  UseChunkInfo uci;

  uci.num_chunk = 0;
  /* Fit to ISO standard AudioMuxElement(). */

  /* [ISO standard] Check the first bit. */

  if (!(*(ptr_info->ptr_check_latm) & 0x80))
    {
      /* UseSameStreamMux processing. */

      ptr_info->total_bit_length++;
      rtn_length++;

      /* Set information in StreamMuxConfig to local table. */

      dummy_length = isoStreamMuxConfig(ptr_info, ptr_stream_mux_config);
      if (dummy_length == LOCAL_CHECK_NG)
        {
          /* Since it is not supported this time,
           * it is regarded as an error.
           */

          ptr_stream_mux_config->max_stream_id = 0;
          return LOCAL_CHECK_NG;
        }
      rtn_length += dummy_length;
    }
  else
    {
      ptr_info->total_bit_length++;
      rtn_length++;

      /* Check if there is StreamMuxConfig information for the last time. */

      if ((ptr_stream_mux_config->max_stream_id < LATM_MIN_STREAM_ID) ||
           (ptr_stream_mux_config->max_stream_id > LATM_MAX_STREAM_ID))
        {
          /* There is no StreamMuxConfig information. */

          return LOCAL_CHECK_NG;
        }
    }

  /* Use information set in isoStreamMuxConfig(). */

  if (!(ptr_stream_mux_config->audio_muxversion_a))
    {
      for (int i = 0; i <= (int)ptr_stream_mux_config->num_sub_frames; i++)
        {
          /* [ISO standard] PayloadLengthInfo processing. */

          dummy_length =
            isoPayloadLengthInfo(ptr_info,
                                 ptr_stream_mux_config,
                                 &uci);
          if (dummy_length == LOCAL_CHECK_NG)
            {
              return LOCAL_CHECK_NG;
            }
          rtn_length += dummy_length;

          /* [ISO standard] PayloadMux processing. */

          dummy_length = isoPayloadMux(ptr_info, ptr_stream_mux_config, &uci);
          if (dummy_length == LOCAL_CHECK_NG)
            {
              return LOCAL_CHECK_NG;
            }
          rtn_length += dummy_length;
          ptr_stream_mux_config->info_stream_frame[i].frame_length_type =
            ptr_stream_mux_config->info_stream_id[0].frame_length_type;
          ptr_stream_mux_config->info_stream_frame[i].frame_length =
            ptr_stream_mux_config->info_stream_id[0].frame_length;
          ptr_stream_mux_config->info_stream_frame[i].payload_offset =
            ptr_stream_mux_config->info_stream_id[0].payload_offset;
        }

      /* [ISO standard] otherDataPresent processing. */

      if (ptr_stream_mux_config->other_data_present)
        {
          dummy_length = ptr_stream_mux_config->other_data_len_bits;
          rtn_length += dummy_length;

          /* Add the bit length of otherData. */

          ptr_info->total_bit_length +=
            ptr_stream_mux_config->other_data_len_bits;

          /* The current pointer ptr_check_latm is updated. */

          ptr_info->ptr_check_latm +=
            (ptr_info->total_bit_length / LATM_BIT_OF_BYTE);
        }
    }
  else
    {
      return LOCAL_CHECK_NG;
    }

  rtn_length += iso_byteAlignment(ptr_info);

  return rtn_length;
}

/*--------------------------------------------------------------------------*/
uint8_t *AACLC_getNextLatm(uint8_t *ptr_readbuff,
                           InfoStreamMuxConfig *ptr_stream_mux_config)
{
  LatmLocalInfo info;

  info.total_bit_length = 0;
  info.ptr_check_latm = ptr_readbuff;

  /* Check LOAS.(If LOAS 2 bytes later LATM header) */

  if (AACLC_checkLOAS(&info))
    {
      info.ptr_check_latm = (ptr_readbuff + 2);
      return info.ptr_check_latm;
    }

  /* When it is not LOAS.(syncword not found) */

  info.total_bit_length = 0;
  info.ptr_check_latm = ptr_readbuff;

  /* [ISO standard] AudioMuxElement () processing. */

  int32_t dummy_length = isoAudioMuxElement(&info, ptr_stream_mux_config);
  if (dummy_length == LOCAL_CHECK_NG)
    {
      /* When it is not the target ObjectType. */

      return 0;
    }

  /* Update ptr_check_latm. */

  info.ptr_check_latm = ((ptr_readbuff) + (dummy_length / LATM_BIT_OF_BYTE));

  return info.ptr_check_latm;
}

#ifdef LATMTEST_BY_CUNIT
#  include "LatmTest_Wrapper"
#endif
