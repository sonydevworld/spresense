/****************************************************************************
 * modules/include/audio/audio_frontend_api.h
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_INCLUDE_AUDIO_AUDIO_FRONTEND_API_H
#define __MODULES_INCLUDE_AUDIO_AUDIO_FRONTEND_API_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/**
 * @defgroup audioutils_audio_frontend_api Mic Frontend API
 * @{
 *
 * @file       audio_frontend_api.h
 * @brief      CXD5602 Audio Mic Frontend API
 * @author     CXD5602 Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "audio/audio_common_defs.h"
#include "audio/audio_object_common_api.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AS_FEATURE_FRONTEND_ENABLE

/** @name Packet length of command*/
/** @{ */

/*! \brief Init Mic Frontend command (#AUDCMD_INIT_MICFRONTEND) packet length */

#define  LENGTH_INIT_MICFRONTEND    4

/*! \brief InitPreProcessDSP command (#AUDCMD_INIT_PREPROCESS) packet length */

#define  LENGTH_INIT_PREPROCESS_DSP  4

/*! \brief SetPreProcessDSP command (#AUDCMD_SET_PREPROCESS_DSP) packet length */

#define  LENGTH_SET_PREPROCESS_DSP   4

/** @} */

/*! \brief Length of Recognizer dsp file name and path */

#define AS_PREPROCESS_FILE_PATH_LEN 22

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Event type of Mic FrontEnd */

typedef enum
{
  /*! \brief Activate */

  AsMicFrontendEventAct = 0,

  /*! \brief Deactivate */

  AsMicFrontendEventDeact,

  /*! \brief Init */

  AsMicFrontendEventInit,

  /*! \brief Start */

  AsMicFrontendEventStart,

  /*! \brief Stop */

  AsMicFrontendEventStop,

  /*! \brief Init PreProc */

  AsMicFrontendEventInitPreProc,

  /*! \brief Set PreProc */

  AsMicFrontendEventSetPreProc,

  /*! \brief Set MicGain */

  AsMicFrontendEventSetMicGain,

} AsMicFrontendEvent;

typedef enum
{
  AsMicFrontendDeviceMic = 0,

} AsMicFrontendInputDevice;

/** Select Mic Frontend PreProcess Enable */

typedef enum
{
  /*! \brief Pre Process through */

  AsMicFrontendPreProcThrough = 0,

  /*! \brief Pre Process Sampling Rate Conveter */

  AsMicFrontendPreProcSrc,

  /*! \brief Pre Process user customed */

  AsMicFrontendPreProcUserCustom,

  AsMicFrontendPreProcInvalid = 0xff,

} AsMicFrontendPreProcType;

typedef enum
{
  /*! \brief PCM data is send by callback */

  AsDataPathCallback = 0,

  /*! \brief PCM data is send by message */

  AsDataPathMessage,

  /*! \brief PCM data is send by simple FIFO */

  AsDataPathSimpleFIFO,

} AsMicFrontendDataPath;

/** Activate Mic FrontEnd Command */

typedef struct
{
  /*! \brief [in] Select Mic Frontend input device
   *
   * Use #AsSetRecorderStsInputDevice enum type
   */

  uint8_t  input_device;

} AsActivateFrontendParam;

typedef bool (*MicFrontendCallback)(AsMicFrontendEvent evtype, uint32_t result, uint32_t sub_result);

typedef struct
{
  AsActivateFrontendParam param;
   
  MicFrontendCallback cb;

} AsActivateMicFrontend;

/** Deactivate Mic FrontEnd Command */

typedef struct
{
  uint32_t reserve;

} AsDeactivateMicFrontendParam;

/** InitMicFrontend Command parameter */

typedef void (*FrontendDoneCallback)(AsPcmDataParam param);

union AsDataDest
{
  FrontendDoneCallback cb;

  CMN_SimpleFifoHandle *simple_fifo_handler;

  struct __st_tunnel
  {
    uint8_t msgqid;
    uint16_t msgtype;
  } msg;
};

typedef struct
{
  /*! \brief [in] Select InitMicFrontend input channels
   */

  uint8_t  channel_number;

  /*! \brief [in] Select InitMicFrontend input bit length
   */

  uint8_t  bit_length;

  /*! \brief [in] Samples per a frame
   */

  uint16_t samples_per_frame;

  /*! \brief [in] Output Fs
   *
   * !! effective only when preproc_type is "AsMicFrontendPreProcSrc" !!
   */

  uint32_t out_fs;

  /*! \brief [in] Select pre process enable 
   *
   * Use #AsMicFrontendPreProcType enum type
   */

  uint8_t  preproc_type;

  /*! \brief [in] Set dsp file name and path 
   */

  char dsp_path[AS_PREPROCESS_FILE_PATH_LEN];

  /*! \brief [in] Select Data path from MicFrontend 
   *
   * Use #AsMicFrontendDataPath enum type
   */

  uint8_t  data_path;

  /*! \brief [in] Data destination, callback or message 
   *
   * Follow "data_path" parameter to select callback or message
   */

  AsDataDest dest;

} AsInitMicFrontendParam;

/** StartMicFrontend Command parameter */

typedef struct
{
  uint32_t reserve;

} AsStartMicFrontendParam;

/** StopMicFrontend Command parameter */

typedef struct
{
  uint32_t stop_mode;

} AsStopMicFrontendParam;

/** InitPreProc, SetPreProc Command parameter */

typedef struct
{
  /*! \brief [in] Command packet address */

  uint8_t  *packet_addr;

  /*! \brief [in] Command packet size */

  uint32_t packet_size;

} AsInitPreProcParam, AsSetPreProcParam;

/** Set Mic Gain Command parameter */

typedef struct
{
  /*! \brief [in] Mic gain
   * 
   *  Analog microphone can set every 0.5 dB between 0 dB and 21 dB.
   *  In this parameter, a value from 0 to 210 is set for every 5.
   *
   *  Digital microphone can set every 0.01 dB between 78.50 dB and 0.00 dB
   *  In this parameter, a value from -7850 to 0 is set for every 1.
   */

  int16_t mic_gain[AS_MIC_CHANNEL_MAX];

} AsMicFrontendMicGainParam;

/** Frontendcommand header */

typedef struct
{
  /*! \brief [in] Command code */

  uint8_t command_code;

  /*! \brief [in] reserved */

  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;

} MicFrontendCommandHeader;

/** MicFrontendCommand definition */

typedef struct
{
  /*! \brief [in] Command Header */

  MicFrontendCommandHeader header;

  /*! \brief [in] Command Parameters */

  union
  {
    /*! \brief [in] for ActivateFrontend
     * (Object Interface==AS_ActivateFrontend)
     */
 
    AsActivateMicFrontend act_param;

    /*! \brief [in] for DeactivateFrontend
     * (Object Interface==AS_DeactivateFrontend)
     */
 
    AsDeactivateMicFrontendParam deact_param;

    /*! \brief [in] for InitFrontend
     * (Object Interface==AS_InitMicFrontend)
     */

    AsInitMicFrontendParam init_param;

    /*! \brief [in] for StartFrontend
     * (Object Interface==AS_StartMicFrontend)
     */

    AsStartMicFrontendParam start_param;

    /*! \brief [in] for StopFrontend
     * (Object Interface==AS_StopMicFrontend)
     */

    AsStopMicFrontendParam stop_param;

    /*! \brief [in] for InitPreProc
     * (Object Interface==AS_InitPreProcFrontend)
     */

    AsInitPreProcParam initpreproc_param;

    /*! \brief [in] for SetPreProc
     * (Object Interface==AS_SetPreProcFrontend)
     */

    AsSetPreProcParam setpreproc_param;

    /*! \brief [in] for SetMicGain
     * (Object Interface==AS_SetMicGainFrontend)
     */

    AsMicFrontendMicGainParam mic_gain_param;
  };

} MicFrontendCommand;

/** Message queue ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Message queue id of micfrontend */

  uint8_t micfrontend;

  /*! \brief [in] Message queue id of audio_manager */

  uint8_t mng;

  /*! \brief [in] Message queue id of DSP */

  uint8_t dsp;

} AsMicFrontendMsgQueId_t;

/** Pool ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Memory pool id of input data */

  MemMgrLite::PoolId input;

  /*! \brief [in] Memory pool id of PreProc */

  MemMgrLite::PoolId output;

  /*! \brief [in] Memory pool id of dsp command data */

  MemMgrLite::PoolId dsp;

} AsMicFrontendPoolId_t;

typedef struct
{
  /*! \brief [in] Memory pool id of input data */

  uint8_t input;

  /*! \brief [in] Memory pool id of PreProc */

  uint8_t output;

  /*! \brief [in] Memory pool id of dsp command data */

  uint8_t dsp;

} AsMicFrontendPoolId_old_t;

/** Activate function parameter */

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsMicFrontendMsgQueId_t   msgq_id;

  /*! \brief [in] ID of memory pool for processing data */

  AsMicFrontendPoolId_old_t pool_id;

} AsCreateMicFrontendParam_t;

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsMicFrontendMsgQueId_t msgq_id;

  /*! \brief [in] ID of memory pool for processing data */

  AsMicFrontendPoolId_t   pool_id;

} AsCreateMicFrontendParams_t;

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
 * @brief Create mic frontend
 *
 * @param[in] param: Parameters of resources used by mic frontend
 * @param[in] attcb: Attention callback of Frontend. NULL means no callback.
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_CreateMicFrontend(FAR AsCreateMicFrontendParam_t *param,
                          AudioAttentionCb attcb);

bool AS_CreateMicFrontend(FAR AsCreateMicFrontendParams_t *param,
                          AudioAttentionCb attcb);

/**
 * @brief Activate mic frontend
 *
 * @param[in] actparam: Activation parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_ActivateMicFrontend(FAR AsActivateMicFrontend *actparam);

/**
 * @brief Init mic frontend
 *
 * @param[in] initparam: Initialization parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_InitMicFrontend(FAR AsInitMicFrontendParam *initparam);

/**
 * @brief Start mic frontend
 *
 * @param[in] startparam: Start parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_StartMicFrontend(FAR AsStartMicFrontendParam *startparam);

/**
 * @brief Stop mic frontend
 *
 * @param[in] stopparam: Stop parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_StopMicFrontend(FAR AsStopMicFrontendParam *stopparam);

/**
 * @brief Init pre process 
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_InitPreprocFrontend(FAR AsInitPreProcParam *param);

/**
 * @brief Set pre process 
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_SetPreprocMicFrontend(FAR AsSetPreProcParam *param);

/**
 * @brief Set Mic gain 
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_SetMicGainMicFrontend(FAR AsMicFrontendMicGainParam *micgain_param);

/**
 * @brief Deactivate mic frontend
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeactivateMicFrontend(FAR AsDeactivateMicFrontendParam *deactparam);

/**
 * @brief Delete mic frontend
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeleteMicFrontend(void);

/**
 * @brief Check availability of MicFrontend 
 *
 * @retval     true  : avaliable 
 * @retval     false : Not available 
 */

bool AS_checkAvailabilityMicFrontend(void);

#endif  /* __MODULES_INCLUDE_AUDIO_AUDIO_FRONTEND_API_H */
/**
 * @}
 */

/**
 * @}
 */
