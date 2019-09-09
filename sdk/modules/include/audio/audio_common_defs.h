/****************************************************************************
 * modules/include/audio/audio_common_defs.h
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

#ifndef __MODULES_INCLUDE_AUDIO_AUDIO_COMMON_DEFS_H
#define __MODULES_INCLUDE_AUDIO_AUDIO_COMMON_DEFS_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/* API Documents creater with Doxgen */

/**
 * @defgroup audioutils_audio_high_level_api Audio High Level API
 * @{
 *
 * @file       audio_common_defs.h
 * @brief      CXD5602 Audio Common Definitions
 * @author     CXD5602 Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/memory_manager/MemHandle.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* const for baseband */
/** @name Constant parameters */
/** @{ */

#define AS_VOLUME_DAC            -20

/* TODO:Be removed in future */

#define AS_AC_CODEC_VOL_DAC      -20

/** @} */

/* ---------------------------------*/

/** @name Fix values of audio parameter */
/** @{ */

/**  Mic channel max */

#define AS_MIC_CHANNEL_MAX        8


/** Keep setting for #InitMicGainParam.mic_gain */

#define AS_MICGAIN_HOLD           215

/** Keep setting for #SetVolumeParam.input1_db,
 *                   #SetVolumeParam.input2_db,
 *                   #SetVolumeParam.master_db
 */

#define AS_VOLUME_HOLD            255

/** Mute setting for above parameters */

#define AS_VOLUME_MUTE            -1025

/** Keep setting for #SetBeepParam.beep_vol*/

#define AS_BEEP_VOL_HOLD          255

/** Keep setting for #SetBeepParam.beep_freq */

#define AS_BEEP_FREQ_HOLD         0

/** Mute setting for above parameters */

#define AS_MICGAIN_MUTE           -7855

/** @} */

/** @name Codec type */
/** @{ */

/** MP3 */

#define AS_CODECTYPE_MP3    0

/** WAV */

#define AS_CODECTYPE_WAV    1

/** AAC */

#define AS_CODECTYPE_AAC    2

/** OPUS */

#define AS_CODECTYPE_OPUS   3

/** MEDIA Packet */

#define AS_CODECTYPE_MEDIA  4

/** Linear PCM */

#define AS_CODECTYPE_LPCM  5

/** @} */

/** @name Bit length */
/** @{ */

/** 16bit */

#define AS_BITLENGTH_16  16

/** 24bit */

#define AS_BITLENGTH_24  24

/** 24bit Unpacked (32bit) */

#define AS_BITLENGTH_32  32

/** @} */

/** @name Channel number */
/** @{ */

/** MONO (1ch) */

#define AS_CHANNEL_MONO    1

/** STEREO (2ch) */

#define AS_CHANNEL_STEREO  2

/** 4ch */

#define AS_CHANNEL_4CH     4

/** 6ch */

#define AS_CHANNEL_6CH     6

/** 8ch */

#define AS_CHANNEL_8CH     8

/** @} */

/** @name Sampling rate */
/** @{ */

/* Auto */

#define AS_SAMPLINGRATE_AUTO    0

/** 8kHz */

#define AS_SAMPLINGRATE_8000    8000

/** 11.025kHz */

#define AS_SAMPLINGRATE_11025   11025

/** 12kHz */

#define AS_SAMPLINGRATE_12000   12000

/** 16kHz */

#define AS_SAMPLINGRATE_16000   16000

/** 22.05kHz */

#define AS_SAMPLINGRATE_22050   22050

/** 24kHz */

#define AS_SAMPLINGRATE_24000   24000

/** 32kHz */

#define AS_SAMPLINGRATE_32000   32000

/** 44.1kHz */

#define AS_SAMPLINGRATE_44100   44100

/** 48kHz */

#define AS_SAMPLINGRATE_48000   48000

/** 64kHz */

#define AS_SAMPLINGRATE_64000   64000

/** 88.2kHz */

#define AS_SAMPLINGRATE_88200   88200

/** 96kHz */

#define AS_SAMPLINGRATE_96000   96000

/** 128kHz */

#define AS_SAMPLINGRATE_128000  128000

/** 176.4kHz */

#define AS_SAMPLINGRATE_176400  176400

/** 192kHz */

#define AS_SAMPLINGRATE_192000  192000

/** @} */

/** @name Bit rate */
/** @{ */

/** 8kbps */

#define AS_BITRATE_8000    8000

/** 16kbps */

#define AS_BITRATE_16000   16000

/** 24kbps */

#define AS_BITRATE_24000   24000

/** 32kbps */

#define AS_BITRATE_32000   32000

/** 40kbps */

#define AS_BITRATE_40000   40000

/** 48kbps */

#define AS_BITRATE_48000   48000

/** 56kbps */

#define AS_BITRATE_56000   56000

/** 64kbps */

#define AS_BITRATE_64000   64000

/** 80kbps */

#define AS_BITRATE_80000   80000

/** 96kbps */

#define AS_BITRATE_96000   96000

/** 112kbps */

#define AS_BITRATE_112000  112000

/** 128kbps */

#define AS_BITRATE_128000  128000

/** 144kbps */

#define AS_BITRATE_144000  144000

/** 160kbps */

#define AS_BITRATE_160000  160000

/** 192kbps */

#define AS_BITRATE_192000  192000

/** 224kbps */

#define AS_BITRATE_224000  224000

/** 256kbps */

#define AS_BITRATE_256000  256000

/** 320kbps */

#define AS_BITRATE_320000  320000

/** 384kbps */

#define AS_BITRATE_384000  384000

/** 448kbps */

#define AS_BITRATE_448000  448000

/** 510kbps */

#define AS_BITRATE_510000  510000

/** @} */

/** @defgroup attention_code Attention Code */
/** @{ */

/*! \brief Just Information
 *  \details Just notify information of intermal processing.
 */

#define AS_ATTENTION_CODE_INFORMATION 0x00

/*! \brief Warning Attention
 *  \details Internal process error that can continue processing.
 *           The system can work after this attention, but correctness is not guaranteed.
 *           For example, encode error, decode error.
 */

#define AS_ATTENTION_CODE_WARNING     0x01

/*! \brief Error Attention
 *  \details Internal process error tha cannot continue processing.
 *           The system cannot work correctly after this attention.
 *           For example, internal state error, DSP load error.
 */

#define AS_ATTENTION_CODE_ERROR       0x02

/*! \brief Fatal Attention
 *  \details Internal process error that cannot continue processing.
 *           Ths system should be rebooted.
 */

#define AS_ATTENTION_CODE_FATAL       0x03

/** @} */

/** @defgroup attention_sub_code Attention Sub Code */
/** @{ */

/*! \brief DMA Underflow
 *  \details DMA tranfering queue became empty because of DMA read/write request\n
 *           may be slower than DMA transfer speed. To avoid this, request DMA transfer\n
 *           faster than transfer speed. (tranfer speed : 48000 samples/sec)
 */

#define AS_ATTENTION_SUB_CODE_DMA_UNDERFLOW         0x01

/*! \brief DMA Overflow
 *  \details DMA captureing queue became full because of DMA read/write request\n
 *           may be faster than DMA transfer speed. To avoid this, request DMA capture\n
 *           faster than capture speed. (capture speed : 48000 samples/sec)
 */

#define AS_ATTENTION_SUB_CODE_DMA_OVERFLOW          0x02

/*! \brief DMA Error
 *  \details DMA error returned from DMW H/W. It may be wrong register setting.
 */

#define AS_ATTENTION_SUB_CODE_DMA_ERROR             0x03

/*! \brief APU Queue Full Error
 *  \details DSP response may be delayed and command buffer became full.
 */

#define AS_ATTENTION_SUB_CODE_APU_QUEUE_FULL        0x04

/*! \brief SimpleFIFO Underflow
 *  \details Simple fifo between application and sdk became empty. Insertion speed\n
 *           may be slower than extraction. Coordinate task priority of application\n
 *           to avoid queue remain decreaseing.
 */

#define AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_UNDERFLOW 0x05

/*! \brief SimpleFIFO Overflow
 *  \details Simple fifo between application and sdk became full. Extraction speed\n
 *           may be slower than Insertion. Coordinate task priority of application\n
 *           to avoid queue remain increaseing.
 */

#define AS_ATTENTION_SUB_CODE_SIMPLE_FIFO_OVERFLOW  0x06

/*! \brief Illegal Request
 *  \details Cannot accept this event on current state.
 */

#define AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST       0x07

/*! \brief Internal State Error
 *  \details Internal State Error.
 */

#define AS_ATTENTION_SUB_CODE_INTERNAL_STATE_ERROR  0x08

/*! \brief Unexpected Parameter
 *  \details The command parameter may be wrong. Revise command parameters.
 */

#define AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM      0x09

/*! \brief Internal Queue Pop Error
 *  \details The internal process queue is already empty but tried to extraction.
 */

#define AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR       0x0A

/*! \brief Internal Queue Push Error
 *  \details The internal process queue is already full but tried to insertion.
 */

#define AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR      0x0B

/*! \brief Internal Queue Missing Error
 *  \details The queue became empty unexpectedly.
 */

#define AS_ATTENTION_SUB_CODE_QUEUE_MISSING_ERROR   0x0C

/*! \brief Memory Handle Alloc Error
 *  \details All Memory handles may be used.
 *           Response from DSP may be delayed or get stacked up.
 */

#define AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR 0x0D

/*! \brief Memory Handle Free Error
 *  \details The handle may be already freed.
 */

#define AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR  0x0E

/*! \brief Task Create Error
 *  \details Task context cannot create.\n
 *           Revise max task creation number on menu config.
 */

#define AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR     0x0F

/*! \brief Instance Resource Error
 *  \details A class instanse could not create or delete.\n
 *           Check if there duplicate creation of audio objects/components.
 *           And, check heap area size too.
 */

#define AS_ATTENTION_SUB_CODE_RESOURCE_ERROR        0x10

/*! \brief Decoded size equal zero
 *  \details Decoded PCM size euqals to 0, Provided ES data may be broken.
 */

#define AS_ATTENTION_SUB_CODE_DECODED_SIZE_ZERO     0x11

/*! \brief DSP Load Error
 *  \details Tried to load DSP binary to sub core but it was failed.\n
 *           DSP binary may not be stored on dsp load path.
 */

#define AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR        0x12

/*! \brief DSP Unload Error
 *  \details Tried to unload DSP binary from sub core but it was failed.\n
 *           DSP binary may not loaded.
 */

#define AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR      0x13

/*! \brief DSP Exec Error
 *  \details The data or command which sent to DSP may not be correct format.
 */

#define AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR        0x14

/*! \brief DSP Result Error
 *  \details DSP result error.
 */

#define AS_ATTENTION_SUB_CODE_DSP_RESULT_ERROR      0x15

/*! \brief DSP Illegal Reply
 *  \details Command packet from DSP may be broken.
 *          If uses multi DSP, Check duplication of command buffer.
 */

#define AS_ATTENTION_SUB_CODE_DSP_ILLEGAL_REPLY     0x16

/*! \brief DSP Unload Done
 *  \details DSP unload done notification.
 */

#define AS_ATTENTION_SUB_CODE_DSP_UNLOAD_DONE       0x17

/*! \brief DSP Version Error
 *  \details Loaded DSP binary version is differ from expected.
 */

#define AS_ATTENTION_SUB_CODE_DSP_VERSION_ERROR     0x18

/*! \brief BaseBand Error
 *  \details Baseband power may be off. Power on baseband first.
 */

#define AS_ATTENTION_SUB_CODE_BASEBAND_ERROR        0x19

/*! \brief Stream Parser Error
 *  \details ES parsed result may be differ from Player initialize parameters.
 *           Match parameters and ES data.
 */

#define AS_ATTENTION_SUB_CODE_STREAM_PARSER_ERROR   0x1A

/*! \brief DSP Load Done
 *  \details DSP binary load done.
 */

#define AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE         0x1B

/*! \brief Rec Start Action Done
 *  \details Recording start.
 */

#define AS_ATTENTION_SUB_CODE_RECSTART              0x1C

/*! \brief Rec Stop Action Done
 *  \details Recording stop.
 */

#define AS_ATTENTION_SUB_CODE_RECSTOP               0x1D

/*! \brief DSP Debug Dump Log Alloc Error
 *  \details Log area remain size is less than required size.
 *           The log area may be used by Other DSPs.
 */

#define AS_ATTENTION_SUB_CODE_DSP_LOG_ALLOC_ERROR   0x1E

/*! \brief DSP Assertion Fail
 *  \details DSP internal error occured and DSP cannot keep processing.
 */

#define AS_ATTENTION_SUB_CODE_DSP_ASSETION_FAIL     0x1F

/*! \brief DSP Send Fail
 *  \details Inter CPU commucation with DSP is failed.
 */

#define AS_ATTENTION_SUB_CODE_DSP_SEND_ERROR        0x20

/*! \brief Allocate memory of heap area
 *  \details Notify that allocated from the heap area
 *           without using the memory pool that uses shared memory.
 */

#define AS_ATTENTION_SUB_CODE_ALLOC_HEAP_MEMORY     0x21

#define AS_ATTENTION_SUB_CODE_NUM   AS_ATTENTION_SUB_CODE_ALLOC_HEAP_MEMORY

/** @} */

/**< DSP path length */

#define AS_AUDIO_DSP_PATH_LEN 24 

/****************************************************************************
 * Public Types
 ****************************************************************************/
typedef void (*PcmProcDoneCallback)(int32_t identifier, bool is_end);

/** PCM data parameter */

typedef struct
{
  /*! \brief [in] Data identifier, use anyway */

  int32_t identifier;

  /*! \brief [in] Callback for any pcm data processing */

  PcmProcDoneCallback callback;

  /*! \brief [in] Memory handle for output PCM data */

  MemMgrLite::MemHandle mh;

  /*! \brief [in] Sample number of PCM data */

  uint32_t sample;

  /*! \brief [in] Byte size of input data */

  uint32_t size;

  /*! \brief [in] True: Last data of current session. */

  bool is_end;

  /*! \brief [in] True: Valid PCM frame. */

  bool is_valid;

  /*! \brief [in] Data bit length */

  uint8_t bit_length;

} AsPcmDataParam;

typedef struct
{
  /*! \brief [in] Memory handle for recognition result information. */

  MemMgrLite::MemHandle mh;

  /*! \brief [in] Size recognition result information. */

  uint32_t size;

} AsRecognitionInfo;

/** ErrorAttention Result (#AUDRLT_ERRORATTENTION) parameter */

#define ATTENTION_FILE_NAME_LEN 32

typedef struct
{
  /*! \brief [out] reserved */

  uint32_t reserved1;

  /*! \brief [out] Error Infomation, T.B.D. */

  uint8_t  error_code;

  /*! \brief [out] CPU ID (internal use only) */

  uint8_t  cpu_id;

  /*! \brief [out] for debug purpose */

  uint8_t  sub_module_id;

  /*! \brief [out] Error module infomation, T.B.D. */

  uint8_t  module_id;

  /*! \brief [out] Detailed Error Infomation, T.B.D. */

  uint32_t error_att_sub_code;

  /*! \brief [out] reserved */

  uint32_t reserved2;

  /*! \brief [out] Line No (internal use only) */

  uint16_t line_number;

  /*! \brief [out] Task ID (internal use only) */

  uint8_t  task_id;

  /*! \brief [out] reserved */

  uint8_t  reserved3;

  /*! \brief [out] File name (internal use only) */

  union
  {
    uint32_t align_dummy;
    char     error_filename[ATTENTION_FILE_NAME_LEN];
  };

} ErrorAttentionParam;

/** Audio Attention Callback function
 * @param[in] attparam: Attention detail parameter
 */

typedef void (*AudioAttentionCb)(const ErrorAttentionParam *attparam);

#ifndef ATTENTION_USE_FILENAME_LINE
typedef void (*obs_AudioAttentionCb)(uint8_t module_id,
                                     uint8_t error_code,
                                     uint8_t sub_code);
#else
typedef void (*obs_AudioAttentionCb)(uint8_t module_id,
                                     uint8_t error_code,
                                     uint8_t sub_code,
                                     FAR const char *file_name,
                                     uint32_t line);
#endif

#endif /* __MODULES_INCLUDE_AUDIO_AUDIO_COMMON_DEFS_H */
/**
 * @}
 */
/**
 * @}
 */
