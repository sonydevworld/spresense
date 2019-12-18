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

/*! \brief InitRecognizer command (#AUDCMD_INIT_RECOGNIZER) packet length */

#define LENGTH_INIT_RECOGNIZER 3

/*! \brief StartRecognizer command (#AUDCMD_START_RECOGNIZER) packet length */

#define LENGTH_START_RECOGNIZER 2 

/*! \brief StopRecognizer command (#AUDCMD_STOP_RECOGNIZER) packet length */

#define LENGTH_STOP_RECOGNIZER  2

/*! \brief InitRecognizeProcDSP command (#AUDCMD_INIT_RECOGNIZER_DSP) packet length */

#define LENGTH_INIT_RECOGNIZER_DSP 4 

/*! \brief SetRecognizeProcDSP command (#AUDCMD_SET_RECOGNIZER_DSP) packet length */

#define LENGTH_SET_RECOGNIZER_DSP 4 

/** @} */

/*! \brief Length of Recognizer dsp file name and path */

#define AS_RECOGNIZER_FILE_PATH_LEN (AS_AUDIO_DSP_PATH_LEN)

/****************************************************************************
 * Public Types
 ****************************************************************************/
/** Event type of Recognizer */

typedef enum
{
  /*! \brief Activate */

  AsRecognizerEventAct = 0,

  /*! \brief Deactivate */

  AsRecognizerEventDeact,

  /*! \brief Init */

  AsRecognizerEventInit,

  /*! \brief Start */

  AsRecognizerEventStart,

  /*! \brief Exec */

  AsRecognizerEventExec,

  /*! \brief Stop */

  AsRecognizerEventStop,

  /*! \brief Init RecognizerProc */

  AsRecognizerEventInitRecognizerProc,

  /*! \brief Set RecognizerProc */

  AsRecognizerEventSetRecognizerProc,

} AsRecognizerEvent;

typedef enum
{
  /*! \brief Recognizer type is UserCustom */

  AsRecognizerTypeUserCustom = 0,

  /*! \brief Invalid type */

  AsRecognizerTypeInvalid = 0xff

} AsRecognizerType;

typedef enum
{
  /*! \brief Recognition result is notify by callback */

  AsNotifyPathCallback = 0,

  /*! \brief Recognition result is notify by message */

  AsNotifyPathMessage,

} AsRecognizerNotifyPath;

/** Recongizer Object Result header paramter */

typedef struct
{
  uint32_t result_code;

  uint32_t command_id;

  AsRecognizerEvent event;

} RecognizerResultHeader;

/** Recongizer Object Result paramter */

typedef struct
{
  RecognizerResultHeader header;

} RecognizerResult;

/** Activate parameter */

typedef void (*RecognizerCallback)(RecognizerResult *result);

typedef struct
{
  /*! \brief [in] Event callback */  

  RecognizerCallback cb;

} AsActivateRecognizerParam;

/** Init parameter */

typedef void (*RecognizerNotifyCallback)(AsRecognitionInfo info);

union AsNotifyDest
{
  /*! \brief [in] Recognition result notify callback */

  RecognizerNotifyCallback cb;

  struct __st_tunnel
  {
    uint8_t msgqid;
    uint32_t msgtype;
  } msg;
};

typedef struct
{
  /*! \brief [in] Set recognizer type 
   *
   * Use #AsRecognizerType enum type
   */

  uint8_t type;

  /*! \brief [in] Set dsp file name and path 
   */

  char dsp_path[AS_RECOGNIZER_FILE_PATH_LEN];

  /*! \brief [in] Select Data path from MicFrontend 
   *
   * Use #AsRecognizerNotifyPath enum type
   */

  uint8_t  notify_path;

  AsNotifyDest dest;

} AsInitRecognizerParam;

/** Start parameter */

typedef struct
{
  uint32_t reserve;

} AsStartRecognizerParam;

/** Stop parameter */

typedef struct
{
  uint32_t reserve; 

} AsStopRecognizerParam;

/** Deactivate parameter */

typedef struct
{
  uint32_t reserve; 

} AsDeactivateRecognizerParam;

/** InitRcgProc, SetRcgProc parameter */

typedef struct
{
  /*! \brief [in] Command packet address */

  uint8_t  *packet_addr;

  /*! \brief [in] Command packet size */

  uint32_t packet_size;

} AsInitRecognizerProcParam, AsSetRecognizerProcParam;

/** Recognizer command */

typedef struct
{
  /*! \brief [in] Command ID */

  uint32_t command_id;

} RecognizerCommandHeader;

typedef struct
{
  /* Command Header */

  RecognizerCommandHeader header;

  /* Command Parameters */

  union
  {
    /* Activate Paramters */

    AsActivateRecognizerParam act_param;

    /* Init Paramters */

    AsInitRecognizerParam init_param;

    /* Start Paramters */

    AsStartRecognizerParam start_param;

    /* Stop Paramters */

    AsStopRecognizerParam stop_param;

    /* Init Rcgproc Paramters */

    AsInitRecognizerProcParam initrcgproc_param;

    /* Set Rcgproc Paramters */

    AsSetRecognizerProcParam setrcgproc_param;

    /* Deactivate Paramters */

    AsDeactivateRecognizerParam deact_param;
  };

} RecognizerCommand;

/** Message queue ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Message queue id of sound recognizer */

  uint8_t recognizer;

  /*! \brief [in] Message queue id of audio_manager */

  uint8_t mng;

  /*! \brief [in] Message queue id of DSP */

  uint8_t dsp;

} AsRecognizerMsgQueId_t;

/** Pool ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Memory pool id of recognition result data */

  MemMgrLite::PoolId out;

  /*! \brief [in] Memory pool id of DSP communication Message */

  MemMgrLite::PoolId dsp;

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

/**
 * @brief Activate voice recognizer
 *
 * @param[in] param: Parameters of resources used by voice recognizer
 * @param[in] attcb: Attention callback of Recognizer. NULL means no callback.
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_CreateRecognizer(FAR AsCreateRecognizerParam_t *param,
                         AudioAttentionCb attcb);

__attribute__((deprecated(
                 "\n \
                  \n Deprecated create API is used. \
                  \n Use \"AS_CreateRecognizer(AsCreateRecognizerParam_t * \
                  \n                          AudioAttentionCb)\". \
                  \n \
                  \n")))
bool AS_CreateRecognizer(FAR AsCreateRecognizerParam_t *param);

/**
 * @brief Deactivate voice recognizer
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeleteRecognizer(void);

/**
 * @brief Check availability of Recognizer 
 *
 * @retval     true  : avaliable 
 * @retval     false : Not available 
 */

bool AS_checkAvailabilityRecognizer(void);

#endif  /* AUDIO_RECOGNIZER_API_H */
/**
 * @}
 */

/**
 * @}
 */
