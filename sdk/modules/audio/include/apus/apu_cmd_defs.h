/****************************************************************************
 * modules/audio/include/apus/apu_cmd_defs.h
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

#ifndef __MODULES_AUDIO_INCLUDE_APUS_APU_CMD_DEFS_H
#define __MODULES_AUDIO_INCLUDE_APUS_APU_CMD_DEFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "wien2_common_defs.h"

__WIEN2_BEGIN_NAMESPACE
__APU_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Invalid cpu id. */

#define APU_INVALID_CPU_ID 0xFF


/****************************************************************************
 * Public Types
 ****************************************************************************/

/*--------------------------------------------------------------------------
 * Event Types (for all)
 *--------------------------------------------------------------------------*/
enum ApuEventType
{
  InvalidApuEvent = 0xFF,
  BootEvent = 0,
  InitEvent,
  ExecEvent,
  FlushEvent,
  SetParamEvent,
  TuningEvent,
  ErrorEvent,
  ApuEventTypeNum
};

/***   For Flush   ***/
enum ApuFlushType
{
  InvalidApuFlushType = (-1),
  Clear = 0,
  Purge,
  ApuFlushTypeNum
};

/***   Now Do not Support   ***/
enum SetParamType
{
  InvalidSetParamType = 0xFF,
  SetParamBF = 0,
  SetParamNS,
  SetParamAES,
  SetParamREF,
  SetParamMIC,
  SetParamAGC,
  SetParamEQ,
  SetParamMFE,
  SetDebugMFE,
  SetParamXLOUD,
  SetDebugXLOUD,
  SetParamTypeNum
};
/*--------------------------------------------------------------------------
 * Result Types (for all)
 *--------------------------------------------------------------------------*/
enum exec_result_e
{
  ApuExecOK = 0,
  ApuExecError,
  ApuWarning
};
typedef enum exec_result_e ExecResult;


/***   Error source   ***/
enum internal_err_src_e
{
  FromLib = 0,  /**< Library error */
  FromDrv,      /**< Driver error */
  FromCtrl,     /**< Controler error */
};
typedef enum internal_err_src_e ApuInternalErrorSource;

/***   Internal Error Code   ***/
#define APU_SUCCESS                 0x00  /**< OK */
#define APU_STATE_ERROR             0x01  /**< State violation */
#define APU_DUMPINIT_ERROR          0x10  /**< Initialization error of dump function */
#define APU_CODECTYPE_ERROR         0x11  /**< Codec type value error */
#define APU_STATICAREAINSUFFICIENT  0x12  /**< Library static area insufficient */
#define APU_CHANNELFORMAT_ERROR     0x13  /**< Channel format value  error */
#define APU_SAMPLINGRATE_ERROR      0x14  /**< Sampling rate value error */
#define APU_BITRATE_ERROR           0x15  /**< Bit rate value error */
#define APU_BYTELEN_ERROR           0x16  /**< Byte length value error */
#define APU_COMPLEXITY_ERROR        0x17  /**< Complexity value error */
#define APU_CONTEXTID_ERROR         0x18  /**< Parameter Context Id Error */
#define APU_PROCESSMODE_ERROR       0x19  /**< Parameter Process Mode Error */
#define APU_EVENTTYPE_ERROR         0x1A  /**< Parameter Event Type Error */
#define APU_BUFFERADDR_ERROR        0x1B  /**< Parameter Buffer Address Error */
#define APU_BUFFERSIZE_ERROR        0x1C  /**< Parameter Buffer Size Error */
#define APU_LIBVERSION_ERROR        0x1D  /**< Library Version Error */
#define APU_INIT_ERROR              0x20  /**< Library initialization error */
#define APU_DECODE_ERROR            0x30  /**< Decoder library error */
#define APU_ENCODE_ERROR            0x40  /**< Encoder library error */
#define APU_SRC_ERROR               0x50  /**< Sampling rate converter library error */
#define APU_MFE_ERROR               0x60  /**< Mic front end library error */
#define APU_VAD_ERROR               0x70  /**< VAD library error */
#define APU_WUWSR_ERROR             0x80  /**< WUWSR library error */
#define APU_QUEUEFULL_ERROR         0x90  /**< Queue is full */
#define APU_QUEUEEMPTY_ERROR        0x91  /**< Queue is empty */
#define APU_QUEUEPOP_ERROR          0x92  /**< Queue pop error */
#define APU_QUEUEPUSH_ERROR         0x93  /**< Queue push error */
#define APU_RESORCEBUSY_ERROR       0x94  /**< Resorce busy error */
#define APU_CPUFIFOSEND_ERROR       0x95  /**< CPU fifo send error */

typedef unsigned char ApuInternalErrorCode;

/*--------------------------------------------------------------------------
 * Function Types
 *--------------------------------------------------------------------------*/
enum ApuProcessMode
{
  InvalidApuProcessMode = 0xFF,
  CommonMode = 0,
  DecMode,
  FilterMode,
  EncMode,
  RecognitionMode,
  OscMode,
  ApuProcessModeNum
};

/***   For Filer   ***/
enum ApuFilterType
{
  InvalidApuFilterType = 0xFF,
  SRC = 0,
  Downmix,
  MFE,
  XLOUD,
  BitWidthConv,
  ApuFilterTypeNum
};

/***   For Recognizer   ***/
enum ApuRecognitionType
{
  InvalidApuRecognitionType = (-1),
  Vad = 0,
  Wuwsr,
  VadWuwsr,
  PitchDet,
  FreqDet,
  ApuRecognitionTypeNum
};

/*--------------------------------------------------------------------------
 * Each Parameter Defintions
 *--------------------------------------------------------------------------*/

/***   SRC channel mode(for Multi-Core SRC)   ***/
enum AudioSRCChMode {
	AudSRCChModeAll = 0,  /**< All channel mode */
	AudSRCChModeEven,     /**< Even channel only mode */
	AudSRCChModeOdd       /**< Odd channel only mode */
};

/***   xLOUD mode   ***/
enum audio_xloud_mode_e
{
  AudXloudModeNormal = 0,  /**< Normal mode */
  AudXloudModeCinema,      /**< Cinema mode */
  AudXloudModeDisable      /**< Disable, Path through */
};
typedef enum audio_xloud_mode_e AudioXloudMode;

/***   xLOUD type   ***/
enum audio_xloud_set_type_e
{
  AudXloudSetCommon = 0,  /**< Common */
  AudXloudSetIndividual,  /**< Individual */
  AudXloudSetTypeNum
};
typedef enum audio_xloud_set_type_e AudioXloudSetType;

/***   EAX mode   ***/
enum audio_eax_mode_e
{
  AudEaxModeNormal = 0,  /**< Nomarl mode */
  AudEaxModeCall,        /**< Call mode */
  AudEaxModeDisable      /**< Disable, xLOUD not effective */
};
typedef enum audio_eax_mode_e AudioEaxMode;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

__APU_END_NAMESPACE
__WIEN2_END_NAMESPACE

#endif /* __MODULES_AUDIO_INCLUDE_APUS_APU_CMD_DEFS_H */

