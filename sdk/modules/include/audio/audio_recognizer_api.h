/****************************************************************************
 * modules/include/audio/audio_recognizer_api.h
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

#ifndef __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_RECOGNIZER_API_H
#define __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_RECOGNIZER_API_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/**
 * @defgroup audioutils_audio_recognizer_api Audio Recognizer API
 * @{
 *
 * @file       audio_recognizer_api.h
 * @brief      CXD5602 Audio Recognizer API
 * @author     CXD5602 Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AS_FEATURE_RECOGNIZER_ENABLE

/** @name Packet length of command */
/** @{ */

/*! \brief StartVoiceComamnd command (#AUDCMD_STARTVOICECOMMAND) packet length */

#define LENGTH_START_VOICE_COMMAND 3

/*! \brief StopVoiceCommand command (#AUDCMD_STOPVOICECOMMAND) packet length */

#define LENGTH_STOP_VOICE_COMMAND  2

/** @} */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* StartVoiceCommand */

/** Select Voice Command mode */

typedef enum
{
  /*! \brief Normal Voice Command feature(VAD + WUWSR) */

  AS_VOICE_COMMAND_USUAL = 0,

  /*! \brief VAD only */

  AS_VOICE_COMMAND_VAD_ONLY,
  AS_VOICE_COMMAND_NUM
} AsVoiceCommandVadOnly;

/** Voice Command Status */

typedef enum
{
  /*! \brief Voice Unrecognitzed */

  AS_RECOGNITION_STATUS_VOICE_UNRECOGNIZED = 0,

  /*! \brief Voice Recognized */

  AS_RECOGNITION_STATUS_VOICE_RECOGNIZED,

  /*! \brief Keyword Recognized */

  AS_RECOGNITION_STATUS_KEYWORD_RECOGNIZED,
  AS_RECOGNITION_STATUS_NUM
} AsVoiceRecognitionStatus;

/** Voice Command Callback function
 * @param[in] keyword : currently 0 only
 * @param[in] status  : Use #AsVoiceRecognitionStatus enum type
 */

typedef void (*AudioFindCommandCallbackFunction)(uint16_t keyword,
                                                 uint8_t status);

/** StartVoiceCommand Command (#AUDCMD_STARTVOICECOMMAND)
 * callback argument parameter
 */

typedef struct
{
  /*! \brief [in] keyword, Copied from #StartVoiceCommandParam.keyword */

  uint16_t keyword;

  /*! \brief [in] Voice Command Status, #AsVoiceRecognitionStatus enum type */

  uint8_t status;
} AudioFindCommandInfo;

/** StartVoiceCommand Command (#AUDCMD_STARTVOICECOMMAND) parameter */

typedef struct
{
  /*! \brief [in] Select Voice Command mode
   *
   * Use #AsVoiceCommandVadOnly enum type
   */

  uint8_t  vad_only;

  /*! \brief [in] reserved */

  uint8_t  reserved1;

  /*! \brief [in] Select keyword, 0:"Hello Sony" */

  uint16_t keyword;

  /*! \brief [in] callback function */

  AudioFindCommandCallbackFunction  callback_function;
} StartVoiceCommandParam;

/** Message queue ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Message queue id of voice recognizer */

  uint8_t recognizer;

  /*! \brief [in] Message queue id of audio_manager */

  uint8_t mng;

  /*! \brief [in] Message queue id of DSP */

  uint8_t dsp;
} AsRecognizerMsgQueId_t;

/** Pool ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Memory pool id of wuwsr input data */

  uint8_t wuwsr_in;
} AsRecognizerPoolId_t;

/** Activate function parameter */

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsRecognizerMsgQueId_t msgq_id;

  /*! \brief [in] ID of memory pool for processing data */

  AsRecognizerPoolId_t   pool_id;
} AsCreateRecognizerParam_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif
/**
 * @brief Activate voice recognizer
 *
 * @param[in] param: Parameters of resources used by voice recognizer
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_CreateRecognizer(FAR AsCreateRecognizerParam_t *param);

/**
 * @brief Deactivate voice recognizer
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeleteRecognizer(void);

#ifdef __cplusplus
}
#endif

#endif  /* AUDIO_RECOGNIZER_API_H */
/**
 * @}
 */

/**
 * @}
 */
