/****************************************************************************
 * modules/include/audio/audio_player_api.h
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

#ifndef __MODULES_INCLUDE_AUDIO_AUDIO_PLAYER_API_H
#define __MODULES_INCLUDE_AUDIO_AUDIO_PLAYER_API_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/**
 * @defgroup audioutils_audio_player_api Audio Player API
 * @{
 *
 * @file       audio_player_api.h
 * @brief      CXD5602 Audio Player API
 * @author     CXD5602 Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "audio/audio_common_defs.h"
#include "audio/audio_object_common_api.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/MsgPacket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Enable player feature. */

#define AS_FEATURE_PLAYER_ENABLE

/* Need to enable mixer feature when player feature is enabled.
 * Because player feature always use mixer feature.
 */

#define AS_FEATURE_OUTPUTMIX_ENABLE
  
/** @name Packet length of player command*/
/** @{ */

/*! \brief InitPlayer command (#AUDCMD_INITPLAYER) packet length */

#define LENGTH_INIT_PLAYER     (9)

/*! \brief InitSubPlayer command (#AUDCMD_INITSUBPLAYER) packet length */

#define LENGTH_INIT_SUBPLAYER  (LENGTH_INIT_PLAYER)

/*! \brief PlayPlayer command (#AUDCMD_PLAYPLAYER) packet length */

#define LENGTH_PLAY_PLAYER     (2)

/*! \brief PlaySubPlayer command (#AUDCMD_PLAYSUBPLAYER) packet length */

#define LENGTH_PLAY_SUBPLAYER  (LENGTH_PLAY_PLAYER)

/*! \brief StopPlayer command (#AUDCMD_STOPPLAYER) packet length */

#define LENGTH_STOP_PLAYER     (2)

/*! \brief StopSubPlayer command (#AUDCMD_STOPSUBPLAYER) packet length */

#define LENGTH_STOP_SUBPLAYER  (LENGTH_STOP_PLAYER)

/*! \brief ClkRecovery command ("AUDCMD_CLKRECOVERY)packet length */

#define LENGTH_CLK_RECOVERY  (2)

/*! \brief Set audio gain leve command ("AUDCMD_SETGAIN)packet length */

#define LENGTH_SET_GAIN (2)

/*! \brief Send Pfcommand command ("AUDCMD_SENDPOSTCMD") packet length */

#define LENGTH_SENDPOSTCMD (10)

/** @} */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Event type of player */

typedef enum
{
  /*! \brief Activate */

  AsPlayerEventAct = 0,

  /*! \brief Init */

  AsPlayerEventInit,

  /*! \brief Play */

  AsPlayerEventPlay,

  /*! \brief Stop */

  AsPlayerEventStop,

  /*! \brief Deactivate */

  AsPlayerEventDeact,

  /*! \brief Set gain */

  AsPlayerEventSetGain,

} AsPlayerEvent;

/** player id */

typedef enum
{
  AS_PLAYER_ID_0 = 0,

  AS_PLAYER_ID_1,

} AsPlayerId;

/** Select activate player */

typedef enum
{
  /*! \brief Activate main player */

  AS_ACTPLAYER_MAIN = 1,

  /*! \brief Activate sub player */

  AS_ACTPLAYER_SUB,

  /*! \brief Activate main & sub player */

  AS_ACTPLAYER_BOTH,
  AS_ACTPLAYER_NUM
} AsSetActivatePlayer;

/** Select Player Input device */

typedef enum
{
  /*! \brief eMMC FileSystem
   *  \deprecated It will be removed in the future
   */

  AS_SETPLAYER_INPUTDEVICE_EMMC = 0,

  /*! \brief A2DP Media Packet FIFO
   *  \deprecated It will be removed in the future
   */

  AS_SETPLAYER_INPUTDEVICE_A2DPFIFO,

  /*! \brief I2S input
   *  \deprecated It will be removed in the future
   */

  AS_SETPLAYER_INPUTDEVICE_I2SINPUT,

  /*! \brief RAM */

  AS_SETPLAYER_INPUTDEVICE_RAM,
  AS_SETPLAYER_INPUTDEVICE_NUM

} AsSetPlayerInputDevice;

/** Select Player Output device */

typedef enum
{
  /*! \brief CXD5247 SP/HP */

  AS_SETPLAYER_OUTPUTDEVICE_SPHP = 0,

  /*! \brief I2S Output */

  AS_SETPLAYER_OUTPUTDEVICE_I2SOUTPUT,

  /*! \brief A2DP Media Packet FIFO
   *  \deprecated It will be removed in the future
   */

  AS_SETPLAYER_OUTPUTDEVICE_A2DPFIFO,
  AS_SETPLAYER_OUTPUTDEVICE_NUM
} AsSetPlayerOutputDevice;

/** Select stop mode */

typedef enum
{
  /*! \brief Normal stop (immediately stop) */

  AS_STOPPLAYER_NORMAL = 0,

  /*! \brief Wait end of es */

  AS_STOPPLAYER_ESEND,

  /*! \brief Forcibly stop at system failure */
  AS_STOPPLAYER_FORCIBLY = 0xFF
} AsStopPlayerStopMode;

/**< Output sound period adjustment direction */

typedef enum
{
  /*! \brief Adjust to the + direction */

  OutputMixAdvance = -1,

  /*! \brief No adjust */

  OutputMixNoAdjust = 0,

  /*! \brief Adjust to the - direction */

  OutputMixDelay = 1,
} AsClkRecoveryDirection;

/**< Decodec PCM data send path  */

typedef enum
{
  /*! \brief Decodec PCM data will be replied by callback */

  AsPcmDataReply = 0,

  /*! \brief Decodec PCM data will be sent automatically */

  AsPcmDataTunnel,

} AsPcmDataPath;

/**< Request next decoding type  */

typedef enum
{
  AsNextNormalRequest = 0,
  AsNextStopResRequest,

} AsRequestNextType;

/* for AsPlayerInputDeviceHdlrForRAM */

/** SimpliFifo Callback function
 * @param[in] size : Set read size after reading the SimpleFifo
 */

typedef void (*AudioSimpleFifoReadDoneCallbackFunction)(uint32_t size);

/** internal of ram_handler (used in AsPlayerInputDeviceHdlr) parameter */

typedef struct
{
  /*! \brief [in] Set SimpleFifo handler
   *
   * Use CMN_SimpleFifoHandle (refer to include file)
   */

  void *simple_fifo_handler;

  /*! \brief [in] Set callback function,
   * Call this function when SimpleFifo was read
   */

  AudioSimpleFifoReadDoneCallbackFunction callback_function;

  /*! \brief [in] Read size notification threshold */

  uint32_t  notification_threshold_size;
} AsPlayerInputDeviceHdlrForRAM;

/** SetPlayerStatus Command (#AUDCMD_SETPLAYERSTATUS) parameter */

#if defined(__CC_ARM)
#pragma anon_unions
#endif

typedef struct
{
  /*! \brief [in] Select Player Input device
   *
   *  Use #AsSetPlayerInputDevice enum type
   */

  uint8_t  input_device;

  /*! \brief [in] Select Player Output device
   *
   * Use #AsSetPlayerOutputDevice enum type
   */

  uint8_t  output_device;

  /*! \brief [in] reserved */

  uint8_t  reserved0;

  /*! \brief [in] reserved */

  uint8_t  reserved1;

  /*! \brief [in] Set Player Input device handler, refer following. */

  AsPlayerInputDeviceHdlrForRAM* ram_handler;

} AsActivatePlayerParam;

typedef bool (*MediaPlayerCallback)(AsPlayerEvent evtype, uint32_t result, uint32_t sub_result);

typedef struct
{
  /*! \brief [in] MediaPlayer activation parameters */

  AsActivatePlayerParam param;

  /*! \brief [in] MediaPlayer done callback */

  MediaPlayerCallback cb;

} AsActivatePlayer;

typedef struct
{
  /*! \brief [in] Select activate player
   *
   * Use #AsSetActivatePlayer enum type
   */

  uint8_t  active_player;

  /*! \brief [in] post DSP 0 enable */

  uint8_t  post0_enable;

  /*! \brief [in] post DSP 1 enable */

  uint8_t  post1_enable;

  /*! \brief [in] reserved */

  uint8_t  reserve0;

  /*! \brief [in] Activation parameters for player0 */

  AsActivatePlayerParam player0;

  /*! \brief [in] Activation parameters for player1 */

  AsActivatePlayerParam player1;

#if !defined(__CC_ARM)
} SetPlayerStsParam ;
#else
} SetPlayerStsParam __attribute__((transparent_union));
#endif

/** DeactivatePlayer Command (#) parameter */
typedef struct
{
  uint32_t reserve0;
} AsDeactivatePlayer;

/** InitPlayer Command (#AUDCMD_INITPLAYER, AUDCMD_INITSUBPLAYER) parameter */

typedef void (*DecodeDoneCallback)(AsPcmDataParam param);

typedef struct
{
  /*! \brief [in] Select InitPlayer input channels
   *
   * Use #AsInitPlayerChannelNumberIndex enum type
   */

  uint8_t  channel_number;

  /*! \brief [in] Select InitPlayer input bit length
   *
   * Use #AsInitPlayerBitLength enum type
   */

  uint8_t  bit_length;

  /*! \brief [in] Select InitPlayer codec type
   *
   * Use #AsInitPlayerCodecType enum type
   */

  uint8_t  codec_type;

  /*! \brief [in] reserved */

  uint8_t  reserved1;

  /*! \brief [in] Select sampling rate of es data
   *
   * Use #AsInitPlayerSamplingRateIndex enum type
   */

  uint32_t sampling_rate;

  /*! \brief [in] Audio DSP path */

  char dsp_path[AS_AUDIO_DSP_PATH_LEN];

} AsInitPlayerParam;

/** PlayPlayer Command (#AUDCMD_PLAYPLAYER, #AUDCMD_PLAYSUBPLAYER) parameter */

typedef union
{
  /*! \brief [in] Decoded PCM notify callback
   *
   */

  DecodeDoneCallback callback;

  /*! \brief [in] Decoded PCM notify message 
   *
   */

  struct __st_pcm_msg 
  {
    MsgQueId id;
    uint32_t identifier;
  } msg;

} AsPcmDataDest;

typedef struct
{
  /*! \brief [in] Decoded PCM data path 
   *
   * Let PCM data callback to owner or send to indicated MsgQue
   * Use #AsPcmDataPath enum type
   */

  uint8_t pcm_path;

  /*! \brief [in] Decode done callback
   *
   * Set docode done callback function from MediaPlayer
   * When HighLevelAPI, this parameters is not need to set.
   */

  AsPcmDataDest pcm_dest;

} AsPlayPlayerParam;

/** StopPlayer Command (#AUDCMD_STOPPLAYER, #AUDCMD_STOPSUBPLAYER) parameter */

typedef struct
{
  /*! \brief [in] Stop mode which indicates immediate or wait end of es
   *
   * Use #AsStopPlayerStopMode enum type
   */

  uint8_t stop_mode;
} AsStopPlayerParam;

/** Set Audio gain level Command (#AUDCMD_SETGAIN) parameter */

typedef struct
{
  /*! \brief [in] Gain level Lch
   * Percentage 0 - 200 %
   */

  uint8_t l_gain;

  /*! \brief [in] Gain level Rch
   * Percentage 0 - 200 %
   */

  uint8_t r_gain;
} AsSetGainParam;

/** Request next decode Command (#AUDCMD_REQNEXT) parameter */

typedef struct
{
  /*! \brief [in] Request type
   *
   * Use #AsRequestNextType enum type
   */

  uint8_t type;

} AsRequestNextParam;

/** PlayerCommand definition */

typedef struct
{
  /*! \brief [in] target player id 
   * Use #AsPlayerId enum type
   */

  uint8_t player_id;

  union
  {
    AsActivatePlayer act_param;

    /*! \brief [in] for InitPlayer
     * (header.command_code==#AUDCMD_INITPLAYER)
     * (Object Interface==AS_InitPlayer)
     */
  
    AsInitPlayerParam init_param;
  
    /*! \brief [in] for PlayPlayer
     * (header.command_code==#AUDCMD_PLAYPLAYER)
     */
  
    AsPlayPlayerParam play_param;
  
    /*! \brief [in] for StopPlayer
     * (header.command_code==#AUDCMD_STOPPLAYER)
     */
  
    AsStopPlayerParam stop_param;
  
    AsRequestNextParam req_next_param;

    /*! \brief [in] for Adjust sound period
     * (header.command_code==#AUDCMD_CLKRECOVERY)
     */
  
    AsSetGainParam set_gain_param;
  
    /*! \brief [in] for deactivate player
     * (header.command_code==#AUDCMD_SETREADYSTATUS)
     */
  
    AsDeactivatePlayer deact_param;
  };
} PlayerCommand;

/** Message queue ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Message queue id of player */

  uint8_t player;

  /*! \brief [in] Message queue id of audio_manager */

  uint8_t mng;

  /*! \brief [in] Message queue id of output mixer */

  uint8_t mixer;

  /*! \brief [in] Message queue id of DSP */

  uint8_t dsp;
} AsPlayerMsgQueId_t;

/** Pool ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Memory pool id of es data */

  MemMgrLite::PoolId es;

  /*! \brief [in] Memory pool id of pcm data */

  MemMgrLite::PoolId pcm;

  /*! \brief [in] Memory pool id of dsp command data */

  MemMgrLite::PoolId dsp;

  /*! \brief [in] Memory pool id of src work area */

  MemMgrLite::PoolId src_work;
} AsPlayerPoolId_t;


typedef struct{
  /*! \brief [in] Memory pool id of es data */

  uint8_t es;

  /*! \brief [in] Memory pool id of pcm data */

  uint8_t pcm;

  /*! \brief [in] Memory pool id of dsp command data */

  uint8_t dsp;

  /*! \brief [in] Memory pool id of src work area */

  uint8_t src_work;
} AsPlayerPoolId_old_t;

/** Activate function parameter */

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsPlayerMsgQueId_t msgq_id;

  /*! \brief [in] ID of memory pool for processing data */

  AsPlayerPoolId_old_t   pool_id;

} AsCreatePlayerParam_t;

/** Activate function parameter */

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsPlayerMsgQueId_t msgq_id;

  /*! \brief [in] ID of memory pool for processing data */

  AsPlayerPoolId_t   pool_id;

} AsCreatePlayerParams_t;

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
 * @brief Create audio main player
 *
 * @param[in] param: Parameters of resources used by audio main player
 *
 * @retval     true  : success
 * @retval     false : failure
 * @deprecated Use AS_CreatePlayerMulti() instead.
 */

bool AS_CreatePlayer(AsPlayerId id, FAR AsCreatePlayerParam_t *param);

/**
 * @brief Create audio main player using memory pool in work area of src
 *
 * @param[in] param: Parameters of resources used by audio main player
 * @param[in] attcb: Attention callback of Player. NULL means no callback.
 *
 * @retval     true  : success
 * @retval     false : failure
 * @note New create interface. The use size of the heap area is small.
 */

bool AS_CreatePlayerMulti(AsPlayerId id,
                          FAR AsCreatePlayerParam_t *param,
                          AudioAttentionCb attcb);

bool AS_CreatePlayerMulti(AsPlayerId id,
                          FAR AsCreatePlayerParams_t *param,
                          AudioAttentionCb attcb);

__attribute__((deprecated(
                 "\n \
                  \n Deprecated create API is used. \
                  \n Use \"AS_CreatePlayerMulti(AsPlayerId, \
                  \n                           AsCreatePlayerParam_t *, \
                  \n                           AudioAttentionCb)\". \
                  \n \
                  \n")))
bool AS_CreatePlayerMulti(AsPlayerId id, FAR AsCreatePlayerParam_t *param);

bool AS_CreatePlayerMulti(AsPlayerId id, FAR AsCreatePlayerParams_t *param);

/**
 * @brief Activate audio (sub)player
 *
 * @param[in] actparam: Parameters for activation
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_ActivatePlayer(AsPlayerId id, FAR AsActivatePlayer *actparam);

/**
 * @brief Initialize audio (sub)player
 *
 * @param[in] initparam: Parameters for init player setting
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_InitPlayer(AsPlayerId id, FAR AsInitPlayerParam *initparam);

/**
 * @brief Play audio (sub)player
 *
 * @param[in] playparam: Parameters for play player
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_PlayPlayer(AsPlayerId id, FAR AsPlayPlayerParam *playparam);

/**
 * @brief Stop audio (sub)player
 *
 * @param[in] stopparam: Parameters for stop player
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_StopPlayer(AsPlayerId id, FAR AsStopPlayerParam *stopparam);

/**
 * @brief Set audio gain level of (sub)player
 *
 * @param[in] gainparam: Gain setting parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_SetPlayerGain(AsPlayerId id, FAR AsSetGainParam *gainparam);

/**
 * @brief Request next process(decode) to (sub)player
 *
 * @param[in] nextparam: parameters for next processing
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_RequestNextPlayerProcess(AsPlayerId id, FAR AsRequestNextParam *nextparam);

/**
 * @brief Deactivate (sub)player
 *
 * @param[in] deactparam: Deactivation parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeactivatePlayer(AsPlayerId id, FAR AsDeactivatePlayer *deactparam);

/**
 * @brief Deactivate audio main player
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeletePlayer(AsPlayerId id);

/**
 * @brief Check availability of MediaPlayer 
 *
 * @retval     true  : avaliable 
 * @retval     false : Not available 
 */

bool AS_checkAvailabilityMediaPlayer(AsPlayerId id);

#endif  /* __MODULES_INCLUDE_AUDIO_AUDIO_PLAYER_API_H */
/**
 * @}
 */

/**
 * @}
 */
