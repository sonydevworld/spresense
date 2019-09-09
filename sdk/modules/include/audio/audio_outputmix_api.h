/****************************************************************************
 * modules/include/audio/audio_outputmix_api.h
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

#ifndef __MODULES_INCLUDE_AUDIO_AUDIO_OUTPUTMIX_API_H
#define __MODULES_INCLUDE_AUDIO_AUDIO_OUTPUTMIX_API_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/**
 * @defgroup audioutils_audio_outputmix_api Audio OutputMix API
 * @{
 *
 * @file       audio_outputmix_api.h
 * @brief      CXD5602 Audio OutputMix API
 * @author     CXD5602 Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "audio/audio_common_defs.h"

#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/MsgPacket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AS_FEATURE_OUTPUTMIX_ENABLE

/*! \brief InitMPP command (#AUDCMD_INITMPP) packet length */

#define  LENGTH_INITMPP             5

/*! \brief SetMPP command (#AUDCMD_SETMPPPARAM) packet length */

#define  LENGTH_SUB_SETMPP_COMMON   4

/*! \brief SetMPP command (#AUDCMD_SETMPPPARAM) packet length */

#define  LENGTH_SUB_SETMPP_XLOUD    4

#define PF_COMMAND_PACKET_SIZE_MAX (32)

/****************************************************************************
 * Public Types
 ****************************************************************************/
enum AsOutputMixerHandle
{
  /*! \brief OutputMixer No.0 */

  OutputMixer0 = 0,

  /*! \brief OutputMixer No.1 */

  OutputMixer1,
};

enum AsOutputMixDevice
{
  /*! \brief Speaker out */

  HPOutputDevice = 0,

  /*! \brief I2S out */

  I2SOutputDevice,

  /*! \brief A2DP out */

  A2dpSrcOutputDevice,

  OutputMixDeviceNum
};

/**< Mixer type of output-mix object. */
enum AsOutputMixerType
{
  /*! \brief Main */

  MainOnly = 0,

  /*! \brief SoundEffet */

  SoundEffectOnly,

  /*! \brief Main & SoundEffet */

  MainSoundEffectMix,

  OutputMixerTypeNum
};

/**< PostFilter Enable */

enum AsOutputMixerPostFilter
{
  /*! Disable */

  PostFilterDisable = 0,

  /*! Enable */

  PostFilterEnable,
};

/**< Completion of output-mix object task. */
enum AsOutputMixDoneCmdType
{
  /*! \brief Activation done */

  OutputMixActDone = 0,

  /*! \brief Deactivation done */

  OutputMixDeactDone,

  /*! \brief Set Clock recovery done */

  OutputMixSetClkRcvDone,

  /*! \brief Init Postproc command done */

  OutputMixInitPostDone,

  /*! \brief Set Postproc command done */

  OutputMixSetPostDone,

  OutputMixDoneCmdTypeNum
};

/** Message queue ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Message queue id of output mixer */

  uint8_t mixer;

  /*! \brief [in] Message queue id of audio_manager */

  uint8_t mng;

  /*! \brief [in] Message queue id of dsp
   *              Effective only when use postfilter
   */

  uint8_t render_path0_filter_dsp;
  uint8_t render_path1_filter_dsp;

} AsOutputMixMsgQueId_t;

/** Pool ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Memory pool id of pcm data
   *              Effective only when use postfilter
   */

  MemMgrLite::PoolId render_path0_filter_pcm;
  MemMgrLite::PoolId render_path1_filter_pcm;

  /*! \brief [in] Memory pool id of dsp command data
   *              Effective only when use postfilter
   */

  MemMgrLite::PoolId render_path0_filter_dsp;
  MemMgrLite::PoolId render_path1_filter_dsp;

} AsOutputMixPoolId_t;

typedef struct
{
  /*! \brief [in] Memory pool id of pcm data
   *              Effective only when use postfilter
   */

  uint8_t render_path0_filter_pcm;
  uint8_t render_path1_filter_pcm;

  /*! \brief [in] Memory pool id of dsp command data
   *              Effective only when use postfilter
   */

  uint8_t render_path0_filter_dsp;
  uint8_t render_path1_filter_dsp;

} AsOutputMixPoolId_old_t;

/** Activate function parameter */

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsOutputMixMsgQueId_t msgq_id;

  /*! \brief [in] ID of memory pool for processing data */

  AsOutputMixPoolId_old_t   pool_id;

} AsCreateOutputMixParam_t;

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsOutputMixMsgQueId_t msgq_id;

  /*! \brief [in] ID of memory pool for processing data */

  AsOutputMixPoolId_t   pool_id;

} AsCreateOutputMixParams_t;

/** Activate function parameter */

struct AsOutputMixDoneParam
{
  /*! Handle */

  int handle;

  /*! Kind of done cmd */

  AsOutputMixDoneCmdType done_type;

  /*! result, true means OK */

  bool result;
};

typedef void (*OutputMixerCallback)(MsgQueId requester_dtq, MsgType msgtype, AsOutputMixDoneParam *param);
typedef void (*OutputMixerErrorCallback)(uint8_t handle);

typedef struct
{
  /*! \brief [in] Output device type
   *
   * Use #AsOutputMixDevice enum type
   */

  uint8_t output_device;

  /*! \brief [in] Mixer typ
   *
   * Use #AsOutputMixerType enum type
   */

  uint8_t mixer_type;

   /*! \brief [in] Enable postfilter
   *
   * Use #AsOutputMixerPostFilter enum type
   */

  uint8_t post_enable;

  /*! \brief [in] Done callback */

  OutputMixerCallback cb;

  /*! \brief [in] error callback */

  OutputMixerErrorCallback error_cb;

} AsActivateOutputMixer;

/** Deactivate function parameter */

typedef struct
{
  uint8_t reserve;

} AsDeactivateOutputMixer;

/** Data send function parameter */

typedef struct
{
  /*! \brief [in] Handle of OutputMixer */

  uint8_t handle;

  /*! \brief [in] Send done callback */

  PcmProcDoneCallback callback;

  /*! \brief [in] PCM data parameter */

  AsPcmDataParam pcm;

} AsSendDataOutputMixer;

/** Clock recovery function parameter */

typedef struct
{
  /*! \brief [in] Recovery direction (advance or delay) */

  int8_t   direction;

  /*! \brief [in] Recovery term */

  uint32_t times;

} AsFrameTermFineControl;

/** Init postproc parameter */

typedef struct
{
  /*! Command type (UserDefined, xLoud, etc... */

  uint8_t  cmd_type;

  /*! Init Command packet addr */

  uint8_t  *addr;

  /*! Init Command packet size */

  uint32_t size;

} AsInitPostProc;

typedef AsInitPostProc AsSetPostProc;

/** Clock recovery function parameter */

typedef struct
{
  uint8_t handle;

  union
  {
    AsActivateOutputMixer   act_param;
    AsDeactivateOutputMixer deact_param;
    AsFrameTermFineControl  fterm_param;
    AsInitPostProc          initpp_param;
    AsSetPostProc           setpp_param;
  };
} OutputMixerCommand;

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
 * @brief Create audio output mixer
 *
 * @param[in] param: Parameters of resources used by output mixer
 * @param[in] attcb: Attention callback of OutputMixer. NULL means no callback.
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_CreateOutputMixer(FAR AsCreateOutputMixParam_t *param,
                          AudioAttentionCb attcb);

bool AS_CreateOutputMixer(FAR AsCreateOutputMixParams_t *param,
                          AudioAttentionCb attcb);

__attribute__((deprecated(
                 "\n \
                  \n Deprecated create API is used. \
                  \n Use \"AS_CreateOutputMixer(AsCreateOutputMixParam_t * \
                  \n                           AudioAttentionCb)\". \
                  \n \
                  \n")))
bool AS_CreateOutputMixer(FAR AsCreateOutputMixParam_t *param);

bool AS_CreateOutputMixer(FAR AsCreateOutputMixParams_t *param);

/**
 * @brief Activate audio output mixer
 *
 * @param[in] actparam: Activation parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_ActivateOutputMixer(uint8_t handle, FAR AsActivateOutputMixer *actparam);

/**
 * @brief Send audio data via outputmixer
 *
 * @param[in] sendparam: Send data parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_SendDataOutputMixer(FAR AsSendDataOutputMixer *sendparam);

/**
 * @brief Set clock recovery parameters
 *
 * @param[in] ftermparam: clock recovery parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_FrameTermFineControlOutputMixer(uint8_t handle, FAR AsFrameTermFineControl *ftermparam);

/**
 * @brief Init Postproces DSP
 *
 * @param[in] initppparam: command parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_InitPostprocOutputMixer(uint8_t handle, FAR AsInitPostProc *initppparam);

/**
 * @brief Set parameters Postproces DSP
 *
 * @param[in] setppparam: command parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_SetPostprocOutputMixer(uint8_t handle, FAR AsSetPostProc *setppparam);

/**
 * @brief Deactivate audio output mixer
 *
 * @param[in] deactparam: Deactivation parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeactivateOutputMixer(uint8_t handle, FAR AsDeactivateOutputMixer *deactparam);

/**
 * @brief Delete output mixer
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeleteOutputMix(void);

/**
 * @brief Check availability of OutputMixer 
 *
 * @retval     true  : avaliable 
 * @retval     false : Not available 
 */

bool AS_checkAvailabilityOutputMixer(void);

#endif  /* __MODULES_INCLUDE_AUDIO_AUDIO_OUTPUTMIX_API_H */
/**
 * @}
 */

/**
 * @}
 */
