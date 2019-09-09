/****************************************************************************
 * modules/include/audio/audio_high_level_api.h
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

#ifndef __MODULES_INCLUDE_AUDIO_AUDIO_HIGH_LEVEL_API_H
#define __MODULES_INCLUDE_AUDIO_AUDIO_HIGH_LEVEL_API_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/* API Documents creater with Doxgen */

/**
 * @defgroup audioutils_audio_high_level_api Audio High Level API
 * @{
 *
 * @file       audio_high_level_api.h
 * @brief      CXD5602 Audio High Level API
 * @author     CXD5602 Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include "audio/audio_command_id.h"
#include "audio/audio_common_defs.h"
#ifdef CONFIG_AUDIOUTILS_PLAYER
#  include "audio/audio_player_api.h"
#  include "audio/audio_outputmix_api.h"
#  include "audio/audio_renderer_api.h"
#endif
#ifdef CONFIG_AUDIOUTILS_RECORDER
#  include "audio/audio_recorder_api.h"
#  include "audio/audio_frontend_api.h"
#  include "audio/audio_capture_api.h"
#endif
#ifdef CONFIG_AUDIOUTILS_SOUND_RECOGNIZER
#  include "audio/audio_frontend_api.h"
#  include "audio/audio_recognizer_api.h"
#  include "audio/audio_renderer_api.h"
#  include "audio/audio_capture_api.h"
#endif

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/** @name Reslt Code Packet length */
/** @{ */

/** NomalResponse Packet length */

#define LENGTH_AUDRLT  2

/** Maximum ErrorResponse Packet length */

#define LENGTH_AUDRLT_ERRORRESPONSE_MAX  8

/** Minimum ErrorResponse Packet length */

#define LENGTH_AUDRLT_ERRORRESPONSE_MIN  5

/** @} */

/** @name Packet length of command*/
/** @{ */

/*! \brief GetStatus command (#AUDCMD_GETSTATUS) packet length */

#define LENGTH_GETSTATUS            2

/*! \brief SetPlayerStaus command (#AUDCMD_SETPLAYERSTATUS) packet length */

#define LENGTH_SET_PLAYER_STATUS    6 

/*! \brief SetRecorderStatus command (#AUDCMD_SETRECORDERSTATUS) packet length */

#define LENGTH_SET_RECORDER_STATUS  4

/*! \brief SetRecorderStatus command (#AUDCMD_SETRECOGNIZERSTATUS) packet length */

#define LENGTH_SET_RECOGNIZER_STATUS  4

/*! \brief SetBaseBandStatus command (#AUDCMD_SETBASEBANDSTATUS) packet length */

#define LENGTH_SET_BASEBAND_STATUS  9

/*! \brief SetThroughStatus command (#AUDCMD_SETTHROUGHSTATUS) packet length */

#define LENGTH_SET_THROUGH_STATUS  3

/*! \brief SetReadyStatus command (#AUDCMD_SETREADYSTATUS) packet length */

#define LENGTH_SET_READY_STATUS     2

/*! \brief PowerOn command (#AUDCMD_POWERON) packet length */

#define LENGTH_POWERON              2

/*! \brief PowerOn command (#AUDCMD_SETPOWEROFFSTATUS) packet length */

#define LENGTH_SET_POWEROFF_STATUS  2

/*! \brief SetMicMap command (#AUDCMD_SETMICMAP) packet length */

#define LENGTH_SETMICMAP            4

/*! \brief InitMicGain command (#AUDCMD_INITMICGAIN) packet length */

#define LENGTH_INITMICGAIN          5

/*! \brief InitI2SParam command (#AUDCMD_INITI2SPARAM) packet length */

#define LENGTH_INITI2SPARAM         3

/*! \brief InitDEQParam command (#AUDCMD_INITDEQPARAM) packet length */

#define LENGTH_INITDEQPARAM         8

/*! \brief InitOutputSelect command (#AUDCMD_INITOUTPUTSELECT) packet length */

#define LENGTH_INITOUTPUTSELECT     2

/*! \brief InitDNCParam command (#AUDCMD_INITDNCPARAM) packet length */

#define LENGTH_INITDNCPARAM         2

/*! \brief InitClearStereo command (#AUDCMD_INITCLEARSTEREO) packet length */

#define LENGTH_INITCLEARSTEREO      2

/*! \brief SetVolume command (#AUDCMD_SETVOLUME) packet length */

#define LENGTH_SETVOLUME            3

/*! \brief SetVolumeMute command (#AUDCMD_SETVOLUMEMUTE) packet length */

#define LENGTH_SETVOLUMEMUTE        2

/*! \brief SetBeep command (#AUDCMD_SETBEEPPARAM) packet length */

#define LENGTH_SETBEEPPARAM         3

/*! \brief SetRenderingClk command ("AUDCMD_SETRENDERINGCLK)packet length */

#define LENGTH_SETRENDERINGCLK      2

/*! \brief InitSoundEffect command ("AUDCMD_INITSOUNDEFFECT)packet length */

#define LENGTH_INIT_SOUNDEFFECT     3

/*! \brief StartSoundEffect command ("AUDCMD_STARTSOUNDEFFECT)packet length */

#define LENGTH_START_SOUNDEFFECT    2

/*! \brief StopSoundEffect command ("AUDCMD_STOPSOUNDEFFECT)packet length */

#define LENGTH_STOP_SOUNDEFFECT     2

/*! \brief SetThroughPath command ("AUDCMD_SETTHROUGHPATH)packet length */

#define LENGTH_SET_THROUGH_PATH     4

/*! \brief SetSpDrvMode command ("AUDCMD_SETSPDRVMODE)packet length */

#define LENGTH_SETSPDRVMODE         2

/** @} */

/** @defgroup error_response_code Error Code */
/** @{ */

/*! \brief Response OK */

#define AS_ECODE_OK                              0x00

/*! \brief State Violation */

#define AS_ECODE_STATE_VIOLATION                 0x01

/*! \brief Packet Length Error */

#define AS_ECODE_PACKET_LENGTH_ERROR             0x02

/*! \brief Command Code Error */

#define AS_ECODE_COMMAND_CODE_ERROR              0x03

/*! \brief Not Suppot Command */

#define AS_ECODE_COMMAND_NOT_SUPPOT              0x04

/*! \brief Audio PowerOn Error */

#define AS_ECODE_AUDIO_POWER_ON_ERROR            0x05

/*! \brief Audio PowerOff Error */

#define AS_ECODE_AUDIO_POWER_OFF_ERROR           0x06

/*! \brief DSP Load Error */

#define AS_ECODE_DSP_LOAD_ERROR                  0x07

/*! \brief DSP Unload Error */

#define AS_ECODE_DSP_UNLOAD_ERROR                0x08

/*! \brief DSP Version Error */

#define AS_ECODE_DSP_VERSION_ERROR               0x09

/*! \brief DSP Exec Error */

#define AS_ECODE_DSP_EXEC_ERROR                  0x3b

/*! \brief DSP Stop Error */

#define AS_ECODE_DSP_STOP_ERROR                  0x3c

/*! \brief DSP set Error */

#define AS_ECODE_DSP_SET_ERROR                   0x3d

/*! \brief Set Audio Data Path Error */

#define AS_ECODE_SET_AUDIO_DATA_PATH_ERROR       0x0A

/*! \brief Clear Audio Data Path Error */

#define AS_ECODE_CLEAR_AUDIO_DATA_PATH_ERROR     0x0B

/*! \brief Not Audio Data Path */

#define AS_ECODE_NOT_AUDIO_DATA_PATH             0x0C

/*! \brief Decoder Library Initialize Error */

#define AS_ECODE_DECODER_LIB_INITIALIZE_ERROR    0x0D

/*! \brief Encoder Library Initialize Error */

#define AS_ECODE_ENCODER_LIB_INITIALIZE_ERROR    0x0E

/*! \brief Filter Library Initialize Error */

#define AS_ECODE_FILTER_LIB_INITIALIZE_ERROR     0x0F

/*! \brief Dump Initialize Error */

#define AS_ECODE_DUMP_INITIALIZE_ERROR           0x10

/*! \brief Parameter Codec Type Error */

#define AS_ECODE_COMMAND_PARAM_CODEC_TYPE        0x11

/*! \brief Parameter Area Insufficient Error */

#define AS_ECODE_COMMAND_PARAM_AREA_INSUFFICIENT 0x12

/*! \brief Parameter Channel Number Error */

#define AS_ECODE_COMMAND_PARAM_CHANNEL_NUMBER    0x13

/*! \brief Parameter Sampling Rate Error */

#define AS_ECODE_COMMAND_PARAM_SAMPLING_RATE     0x14

/*! \brief Parameter Bit Rate Error */

#define AS_ECODE_COMMAND_PARAM_BIT_RATE          0x15

/*! \brief Parameter Bit Length Error */

#define AS_ECODE_COMMAND_PARAM_BIT_LENGTH        0x16

/*! \brief Parameter Complexity Error */

#define AS_ECODE_COMMAND_PARAM_COMPLEXITY        0x17

/*! \brief Parameter Active Player Error */

#define AS_ECODE_COMMAND_PARAM_ACTIVE_PLAYER     0x18

/*! \brief Parameter Input Device Error */

#define AS_ECODE_COMMAND_PARAM_INPUT_DEVICE      0x19

/*! \brief ParameterOutput Device Error */

#define AS_ECODE_COMMAND_PARAM_OUTPUT_DEVICE     0x1A

/*! \brief Parameter Input Handler Error */

#define AS_ECODE_COMMAND_PARAM_INPUT_HANDLER     0x1B

/*! \brief Parameter Output Handler Error */

#define AS_ECODE_COMMAND_PARAM_OUTPUT_HANDLER    0x1C

/*! \brief Parameter Callback Error */

#define AS_ECODE_COMMAND_PARAM_CALLBACK          0x1D

/*! \brief Parameter Function Enable Error */

#define AS_ECODE_COMMAND_PARAM_FUNCTION_ENABLE   0x1E

/*! \brief Parameter Config Table Error */

#define AS_ECODE_COMMAND_PARAM_CONFIG_TABLE      0x1F

/*! \brief Parameter With MFE Error */

#define AS_ECODE_COMMAND_PARAM_WITH_MFE          0x20

/*! \brief Parameter With MPP Error */

#define AS_ECODE_COMMAND_PARAM_WITH_MPP          0x21

/*! \brief Parameter Output Data Error */

#define AS_ECODE_COMMAND_PARAM_OUTPUT_DATE       0x22

/*! \brief Parameter Select MIC Error */

#define AS_ECODE_COMMAND_PARAM_SELECT_MIC        0x23

/*! \brief Parameter MIC Gain Error */

#define AS_ECODE_COMMAND_PARAM_MIC_GAIN          0x24

/*! \brief Parameter I2S Id Error */

#define AS_ECODE_COMMAND_PARAM_I2S_ID            0x25

/*! \brief Parameter Bypass Mode Error */

#define AS_ECODE_COMMAND_PARAM_BYPASS_MODE       0x26

/*! \brief Parameter Vollume Error */

#define AS_ECODE_COMMAND_PARAM_VOLLUME           0x27

/*! \brief Parameter Input DB Error */

#define AS_ECODE_COMMAND_PARAM_INPUT_DB          0x28

/*! \brief Parameter Master DB Error */

#define AS_ECODE_COMMAND_PARAM_MASTER_DB         0x29

/*! \brief Parameter Beep Freq Error  */

#define AS_ECODE_COMMAND_PARAM_BEEP_FREQ         0x2A

/*! \brief DMAC Initialize Error */

#define AS_ECODE_DMAC_INITIALIZE_ERROR           0x2B

/*! \brief DMAC Read Error */

#define AS_ECODE_DMAC_READ_ERROR                 0x2C

/*! \brief DMAC Write Error */

#define AS_ECODE_DMAC_WRITE_ERROR                0x2D

/*! \brief Check Memory Pool Error */

#define AS_ECODE_CHECK_MEMORY_POOL_ERROR         0x2E

/*! \brief SimpleFIFO Underflow */

#define AS_ECODE_SIMPLE_FIFO_UNDERFLOW           0x2F

/*! \brief Set Mic Gain Error */

#define AS_ECODE_SET_MIC_GAIN_ERROR              0x30

/*! \brief Set I2S Param Error */

#define AS_ECODE_SET_I2S_PARAM_ERROR             0x31

/*! \brief Set Output Select Error */

#define AS_ECODE_SET_OUTPUT_SELECT_ERROR         0x32

/*! \brief Init Clear Stereo Error */

#define AS_ECODE_INIT_CLEAR_STEREO_ERROR         0x33

/*! \brief Set Volume Error */

#define AS_ECODE_SET_VOLUME_ERROR                0x34

/*! \brief Set Volume Mute Error */

#define AS_ECODE_SET_VOLUME_MUTE_ERROR           0x35

/*! \brief Set Beep Error */

#define AS_ECODE_SET_BEEP_ERROR                  0x36

/*! \brief Internal Queue Operation Error */

#define AS_ECODE_QUEUE_OPERATION_ERROR           0x37

/*! \brief Internal Command Code Error */

#define AS_ECODE_INTERNAL_COMMAND_CODE_ERROR     0x38

/*! \brief Parameter RenderingClk Error */

#define AS_ECODE_COMMAND_PARAM_RENDERINGCLK      0x39

/*! \brief Set Rendering Clock Error */

#define AS_ECODE_SET_RENDERINGCLK_ERROR          0x3A

/*! \brief Parameter SetSpDrvMode Error */

#define AS_ECODE_COMMAND_PARAM_SETSPDRVMODE      0x3B

/*! \brief Set Speaker Driver Mode Error */

#define AS_ECODE_SET_SPDRVMODE_ERROR             0x3C

/*! \brief Set Mic Map Error */

#define AS_ECODE_SET_MICMAP_ERROR                0x3D

/*! \brief Audio SW object cannot available Error */

#define AS_ECODE_OBJECT_NOT_AVAILABLE_ERROR      0x3E

/** @} */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Audio command header */

typedef struct
{
  /*! \brief [in] reserved */

  uint8_t reserved;

  /*! \brief [in] sub */

  uint8_t sub_code;

  /*! \brief [in] Command code */

  uint8_t command_code;

  /*! \brief [in] packet length */

  uint8_t packet_length;
} AudioCommandHeader;

/** Audio result header */

typedef struct
{
  /*! \brief [out] instance id */

  uint8_t instance_id;

  /*! \brief [out] sub code*/

  uint8_t sub_code;

  /*! \brief [out] result code*/

  uint8_t result_code;

  /*! \brief [out] packet length*/

  uint8_t packet_length;
} AudioResultHeader;

/** SetMicMap Command (#AUDCMD_SETMICMAP) parameter */

typedef struct
{
  /*! \brief [in] Set Mic mapping
   *
   *   mic_map[ch(0 <= ch < AS_MIC_CHANNEL_MAX] correspond to
   *   each channels, and you can map analog and digital Mics.
   *
   *   0x1 : Analog Mic 1
   *   0x2 : Analog Mic 2 
   *   0x3 : Analog Mic 3
   *   0x4 : Analog Mic 4
   *   0x5 : Digital Mic 1
   *   0x6 : Digital Mic 2
   *   0x7 : Digital Mic 3
   *   0x8 : Digital Mic 4
   *   0x9 : Digital Mic 5
   *   0xA : Digital Mic 6
   *   0xB : Digital Mic 7
   *   0xC : Digital Mic 8
   *   other : No assing
   *
   */

  uint8_t mic_map[AS_MIC_CHANNEL_MAX]; 

} SetMicMapParam;

/** InitMicGain Command (#AUDCMD_INITMICGAIN) parameter */

typedef struct
{
  /*! \brief [in] Set Mic gain
   *
   *  -7850:-78.50dB, ... , -5:-0.05dB, 0:0dB(default),
   *  5:+0.5dB, ... , 210:+21.0dB, #AS_MICGAIN_HOLD:keep setting
   */

  int16_t mic_gain[AS_MIC_CHANNEL_MAX];
} InitMicGainParam;

/** InitI2SParam Command (#AUDCMD_INITI2SPARAM) parameter */

typedef enum
{
  AS_I2S1,
  AS_I2S2
} AsI2sId;

typedef enum
{
  AS_I2S_BYPASS_MODE_DISABLE,
  AS_I2S_BYPASS_MODE_ENABLE
} AsI2SBypassMode;

typedef struct
{
  /*! \brief [in] Select I2S ID
   * Use #AsI2sId enum type
   */

  uint8_t  i2s_id;

  /*! \brief [in] Select I2S Bypass mode disable/enable
   *
   *  Use #AsBypassModeId enum type
   */

  uint8_t  bypass_mode_en;
  uint16_t reserved2;

  /*! \brief [in] Set I2S input/output frequency */

  uint32_t rate;
} InitI2SParam;

/** InitOutputSelect Command (#AUDCMD_INITOUTPUTSELECT) parameter */

typedef enum
{
  /*! Output device none */

  AS_OUT_OFF,

  /*! Output device speaker */

  AS_OUT_SP,

  /*! Output device i2s */

  AS_OUT_I2S
} AsOutDevice;

typedef struct
{
  /*! \brief [in] Select CXD5247 output devices
   *
   * Use #AsOutDeviceId enum type
   */

  uint8_t  output_device_sel;
  uint8_t  reserved1;
  uint16_t reserved2;
} InitOutputSelectParam;

/** \brief InitDNCParam Command (#AUDCMD_INITDNCPARAM) parameter
 *  \deprecated It will be removed in the future
 */

typedef enum
{
  AS_DNC_MODE_A,
  AS_DNC_MODE_B,
  AS_DNC_MODE_C
} AsDncMode;

typedef struct
{
  /*! \brief [in] Select DNC mode
   *
   * Use #AsDncModeId enum type
   */

  uint8_t  mode;
  uint8_t  reserved1;
  uint16_t reserved2;
} InitDNCParam;

/** InitClearStereo Command (#AUDCMD_INITCLEARSTEREO) parameter */

typedef struct {
  /*! \brief [in] Select ClearStereo Enable/Disable, 1:Enable, 0:Disable. */

  uint8_t cs_en;
  uint8_t reserved1;

  /*! \brief [in] Set ClearStero Volume
   *
   * -825:-82.5dB(default), ... -195:-19.5dB,
   * #AS_CS_VOL_HOLD:keep setting
   */

  int16_t cs_vol;
} InitClearStereoParam;

/** SetVolume Command (#AUDCMD_SETVOLUME) parameter */

typedef struct
{
  /*! \brief [in] Set Master Volume
   *
   * -1020:-102.0dB, ... 120:12.0dB,
   * #AS_VOLUME_HOLD:keep setting, #AS_VOLUME_MUTE(default):Mute
   */

  int16_t  master_db;
  uint16_t reserved1;

  /*! \brief [in] Set Input1 Volume
   *
   * -1020:-102.0dB, ... 120:12.0dB,
   * #AS_VOLUME_HOLD:keep setting, #AS_VOLUME_MUTE(default):Mute
   */

  int16_t  input1_db;

  /*! \brief [in] Set Input2 Volume
   *
   * -1020:-102.0dB, ... 120:12.0dB,
   * #AS_VOLUME_HOLD:keep setting, #AS_VOLUME_MUTE(default):Mute
   */

  int16_t  input2_db;
} SetVolumeParam;

/** SetVolumeMute Command (#AUDCMD_SETVOLUMEMUTE) parameter */

typedef struct
{
  /*! \brief [in] Select Master Volume Mute/UnMute
   *
   * Use #AsVolumeMute enum type.
   */

  uint8_t  master_mute;

  /*! \brief [in] Set Input1 Volume Mute/UnMute */

  uint8_t  input1_mute;

  /*! \brief [in] Set Input2 Volume Mute/UnMute */

  uint8_t  input2_mute;
  uint8_t  reserved1;
} SetVolumeMuteParam;

/** SetBeep Command (#AUDCMD_SETBEEPPARAM) parameter */

typedef struct
{
  /*! \brief [in] Select BEEP Enable/Disable
   *
   *  1:Enable, 0:Disable.
   */

  uint8_t  beep_en;
  uint8_t  reserved1;
  uint16_t reserved2;

  /*! \brief [in] Set BEEP Volume
   *
   * -90:-90dB, ... -12:-12dB(default), ... , 0:0dB,
   * #AS_BEEP_VOL_HOLD:keep setting
   */

  int16_t beep_vol;

  /*! \brief [in] Set BEEP Frequency
   *
   * 94:94Hz, ... 4085:4085Hz,
   * #AS_BEEP_FREQ_HOLD:keep setting
   */

  uint16_t beep_freq;
} SetBeepParam;

/** PowerOn Command (#AUDCMD_POWERON) parameter */

typedef struct
{
  /*! \brief [in] Enable or disable sound-effect playback function.
   *
   * Use #AsEnableSoundEffectFunc enum type.
   */

  uint8_t  enable_sound_effect;
  uint8_t  reserved1;
  uint8_t  reserved2;
  uint8_t  reserved3;
} PowerOnParam;

/** SetRenderingClk Command (#AUDCMD_SETRENDERINGCLK) parameter */

typedef struct
{
  /*! \brief [in] set rendering clock
   *
   * Use #AsClkMode enum type.
   */

  uint8_t  clk_mode;
  uint8_t  reserved1;
  uint8_t  reserved2;
  uint8_t  reserved3;
} SetRenderingClkParam;

/** SetSpDrvMode Command (#AUDCMD_SETSPDRVMODE) parameter */

typedef struct
{
  /*! \brief [in] set speaker driver mode
   *
   * Use #AsSpDrvMode enum type.
   */

  uint8_t  mode;
  uint8_t  reserved1;
  uint8_t  reserved2;
  uint8_t  reserved3;
} SetSpDrvModeParam;

/** SetRenderingClk Command (#AUDCMD_SETRENDERINGCLK) parameter */

typedef enum
{
  AS_CLKMODE_NORMAL,
  AS_CLKMODE_HIRES
} AsClkMode;

/** Enable or disable effect sound playback function. */

typedef enum
{
  /*! \brief Disable effect sound */

  AS_DISABLE_SOUNDEFFECT = 0,

  /*! \brief Enable effect sound
   *  \deprecated It will be removed in the future
   */

  AS_ENABLE_SOUNDEFFECT,
  AS_SOUNDEFFECT_NUM
} AsEnableSoundEffectFunc;

/** playback mode of effect sound */

typedef enum
{
  /*! \brief Play effect sound with limited data. */

  AS_STARTSOUNDEFFECT_NORMAL = 0,

  /*! \brief Play effect sound infinitely. */

  AS_STARTSOUNDEFFECT_INFINITE,
  AS_STARTSOUNDEFFECT_MODE_NUM
} AsStartSoundEffectMode;

/** Select direct input path */

typedef enum
{
  AS_THROUGH_PATH_IN_MIC = 0,
  AS_THROUGH_PATH_IN_I2S1,
  AS_THROUGH_PATH_IN_I2S2,
  AS_THROUGH_PATH_IN_MIXER,
  AS_THROUGH_PATH_IN_NUM
} AsThroughPathIn;

/** Select direct output path */

typedef enum
{
  AS_THROUGH_PATH_OUT_MIXER1 = 0,
  AS_THROUGH_PATH_OUT_MIXER2,
  AS_THROUGH_PATH_OUT_I2S1,
  AS_THROUGH_PATH_OUT_I2S2,
  AS_THROUGH_PATH_OUT_NUM
} AsThroughPathOut;

/** Speaker driver mode */

typedef enum
{
  AS_SP_DRV_MODE_LINEOUT = 0,
  AS_SP_DRV_MODE_1DRIVER,
  AS_SP_DRV_MODE_2DRIVER,
  AS_SP_DRV_MODE_4DRIVER,
  AS_SP_DRV_MODE_NUM
} AsSpDrvMode;

/** Through path of audio data (used in AsSetThroughPathParam) parameter */

typedef struct
{
  /*! \brief  Select direct path Enable/Disable
   *
   *  1:Enable, 0:Disable.
   */

  bool en;

  /*! \brief  input path
   *
   * Use AsThroughPathIn
   */

  uint8_t in;

  /*! \brief  output path
   *
   * Use AsThroughPathOut
   */

  uint8_t out;

} AsThroughPath;

/** SetThroughPath Command (#AUDCMD_SETDIRECTPATH) parameter */

typedef struct
{
  /*! \brief [in] path1 */

  AsThroughPath  path1;

  /*! \brief [in] path2 */

  AsThroughPath  path2;

} AsSetThroughPathParam;

#ifdef AS_FEATURE_OUTPUTMIX_ENABLE

/** Request Clock Recovery Command (#AUDCMD_CLKRECOVERY) parameter */

typedef struct
{
  /*! \brief [in] Handle of OutputMixer */

  uint8_t  player_id;

  int8_t   direction;

  uint32_t times;

} AsPlayerClockRecovery;

/** InitMpp Command (#AUDCMD_INITMPP) parameter */

typedef struct
{
  uint8_t  player_id;

  AsInitPostProc initpp_param;

} AsInitMediaPlayerPost;

/** SetMpp Command (#AUDCMD_SETMPPPARAM) parameter */

typedef struct
{
  uint8_t  player_id;

  AsSetPostProc setpp_param;

} AsSetMediaPlayerPost;

#endif

#ifdef AS_FEATURE_FRONTEND_ENABLE

typedef enum
{
  /*! \brief [in] Send audio data to Recorder
   */

  AsMicFrontendDataToRecorder = 0,

  /*! \brief [in] Send audio data to Recognizer
   */

  AsMicFrontendDataToRecognizer,

} AsFrontendDataDest;

/** Initialize Command (#AUDCMD_INIT_MICFRONTEND) parameter */

typedef struct
{
  /*! \brief [in] Select InitMicFrontend input channels
   */

  uint8_t  ch_num;

  /*! \brief [in] Select InitMicFrontend input bit length
   */

  uint8_t  bit_length;

  /*! \brief [in] Samples per a frame
   */

  uint16_t samples;

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

  char preprocess_dsp_path[AS_PREPROCESS_FILE_PATH_LEN];

  /*! \brief [in] Select Data path from MicFrontend 
   *
   * Use #AsMicFrontendDataDest enum type
   */

  uint8_t data_dest;
  
} AsInitMicFrontEnd;

#endif

#ifdef AS_FEATURE_RECOGNIZER_ENABLE

/** SetRecognizerStatus Command (#AUDCMD_SETRECOGNIZERSTATUS) parameter */

typedef struct
{
  /*! \brief [in] Select Recorder input device
   *
   * Use #AsMicFrontendInputDevice enum type
   */

  uint8_t  input_device;

} AsSetRecognizerStatus;

/** InitVoiceCommand Command (#AUDCMD_INIT_RECOGNIZER) parameter */

typedef void (*RecognizerFindCallback)(AsRecognitionInfo info);

typedef struct
{
  /*! \brief [in] Recognizer type 
   */

  uint8_t recognizer_type;

  /*! \brief [in] Recognizer file name and path 
   */

  char recognizer_dsp_path[AS_RECOGNIZER_FILE_PATH_LEN];

  /*! \brief [in] Recognizer find callback */

  RecognizerFindCallback fcb;

} AsInitRecognizer;

/** StartVoiceCommand Command (#AUDCMD_START_RECOGNIZER) parameter */

typedef struct
{
  uint32_t reserve;

} AsStartRecognizer;

/** StopVoiceCommand Command (#AUDCMD_STOP_RECOGNIZER) parameter */

typedef struct
{
  uint32_t reserve;

} AsStopRecognizer;
#endif /* AS_FEATURE_RECOGNIZER_ENABLE */

/** Audio command packet */

#if defined(__CC_ARM)
#pragma anon_unions
#endif

typedef struct
{
  /*! \brief [in] Command Header */

  AudioCommandHeader header;
  union
  {
#ifdef AS_FEATURE_EFFECTOR_ENABLE
    /*! \brief [in] for StartBB
     * (header.command_code==#AUDCMD_STARTBB)
     *  \deprecated It will be removed in the future
     */

    StartBBParam start_bb_param;

    /*! \brief [in] for StopBB
     * (header.command_code==#AUDCMD_STOPBB)
     *  \deprecated It will be removed in the future
     */

    StopBBParam stop_bb_param;

    /*! \brief [in] for SetBaseBandStatus
     * (header.command_code==#AUDCMD_SETBASEBANDSTATUS)
     *  \deprecated It will be removed in the future
     */

    SetBaseBandStatusParam set_baseband_status_param;
#endif
#ifdef AS_FEATURE_PLAYER_ENABLE
    /*! \brief [in] for SetPlayerStatus
     * (header.command_code==#AUDCMD_SETPLAYERSTATUS)
     */

    SetPlayerStsParam set_player_sts_param;

    /*! \brief [in] player command
     */

    PlayerCommand player;

#endif
#ifdef AS_FEATURE_OUTPUTMIX_ENABLE
    /*! \brief [in] for Adjust sound period
     * (header.command_code==#AUDCMD_CLKRECOVERY)
     */

    AsPlayerClockRecovery clk_recovery_param;

    /*! \brief [in] for InitMPP
     * (header.command_code==#AUDCMD_INITMPP)
     */

    AsInitMediaPlayerPost init_mpp_param;

    /*! \brief [in] for SetMPPParam
     * (header.command_code==#AUDCMD_SETMPPPARAM)
     */

    AsSetMediaPlayerPost set_mpp_param;

#endif
#ifdef AS_FEATURE_FRONTEND_ENABLE

    /*! \brief [in] for InitMicFrontend 
     * (header.command_code==#AUDCMD_INIT_MICFRONTEND)
     */

    AsInitMicFrontEnd init_micfrontend_param;

    /*! \brief [in] for InitPreProcessDSP
     * (header.command_code==#AUDCMD_INIT_PREPROCESS_DSP)
     */

    AsInitPreProcParam init_preproc_param;

    /*! \brief [in] for SetPreProcessDSP
     * (header.command_code==#AUDCMD_SET_PREPROCESS_DSP)
     */

    AsSetPreProcParam set_preproc_param;

#endif
#ifdef AS_FEATURE_RECORDER_ENABLE

    /*! \brief [in] for SetRecorderStatus
     * (header.command_code==#AUDCMD_SETRECORDERSTATUS)
     */

    AsActivateRecorderParam set_recorder_status_param;

    /*! \brief [in] Recorder command
     */

    RecorderCommand recorder;

#endif
#ifdef AS_FEATURE_RECOGNIZER_ENABLE

    /*! \brief [in] for SetRecognizerStatus 
     * (header.command_code==#AUDCMD_SETRECOGNIZERSTATUS)
     */

    AsSetRecognizerStatus set_recognizer_status_param;

    /*! \brief [in] for InitVoiceCommand
     * (header.command_code==#AUDCMD_INIT_RECOGNIZER)
     */

    AsInitRecognizer init_recognizer;

    /*! \brief [in] for StratVoiceCommand
     * (header.command_code==#AUDCMD_START_RECOGNIZER)
     */

    AsStartRecognizer start_recognizer;

    /*! \brief [in] for StopVoiceCommand
     * (header.command_code==#AUDCMD_STOP_RECOGNIZER)
     */

    AsStopRecognizer stop_recognizer;

    /*! \brief [in] for InitRecognizerDSPCommand 
     * (header.command_code==#AUDCMD_INIT_RECOGNIZER_DSP)
     */

    AsInitRecognizerProcParam init_rcg_param;

    /*! \brief [in] for SetRecognizerDSPCommand 
     * (header.command_code==#AUDCMD_SET_RECOGNIZER_DSP)
     */

    AsSetRecognizerProcParam set_rcg_param;

#endif

    /*! \brief [in] for SetMicMap
     * (header.command_code==#AUDCMD_SETMICMAP)
     */

    SetMicMapParam set_mic_map_param;

    /*! \brief [in] for InitMicGain
     * (header.command_code==#AUDCMD_INITMICGAIN)
     */

    InitMicGainParam init_mic_gain_param;

    /*! \brief [in] for InitI2SParam
     * (header.command_code==#AUDCMD_INITI2SPARAM)
     *  \deprecated It will be removed in the future
     */

    InitI2SParam init_i2s_param;

    /*! \brief [in] for InitOutputSelect
     * (header.command_code==#AUDCMD_INITOUTPUTSELECT)
     */

    InitOutputSelectParam init_output_select_param;

    /*! \brief [in] for InitDNCParam
     * (header.command_code==#AUDCMD_INITDNCPARAM)
     *  \deprecated It will be removed in the future
     */

    InitDNCParam init_dnc_param;

    /*! \brief [in] for InitClearStereo
     * (header.command_code==#AUDCMD_INITCLEARSTEREO)
     *  \deprecated It will be removed in the future
     */

    InitClearStereoParam init_clear_stereo_param;

    /*! \brief [in] for SetVolume
     * (header.command_code==#AUDCMD_SETVOLUME)
     */

    SetVolumeParam set_volume_param;

    /*! \brief [in] for SetVolumeMute
     * (header.command_code==#AUDCMD_SETVOLUMEMUTE)
     */

    SetVolumeMuteParam set_volume_mute_param;

    /*! \brief [in] for SetBeep
     * (header.command_code==#AUDCMD_SETBEEPPARAM)
     */

    SetBeepParam set_beep_param;

    /*! \brief [in] for PowerOn
     * (header.command_code==#AUDCMD_POWERON)
     */

    PowerOnParam power_on_param;

    /*! \brief [in] for SetThrouhgPath
     * (header.command_code==#AUDCMD_SETTHROUGHPATH)
     */

    AsSetThroughPathParam set_through_path;

    /*! \brief [in] for SetRenderingClk
     * (header.command_code==#AUDCMD_SETRENDERINGCLK)
     */

    SetRenderingClkParam set_renderingclk_param;

    /*! \brief [in] for SetSpDrvMode
     * (header.command_code==#AUDCMD_SETSPDRVMODE)
     */

    SetSpDrvModeParam set_sp_drv_mode;
  };

#ifdef __cplusplus
  uint8_t getCode(void)
    {
      return header.command_code;
    }
  bool isStateChange(void)
    {
      return ((getCode() & 0x40) != 0);
    }
#endif

#if !defined(__CC_ARM)
} AudioCommand ;
#else
} AudioCommand __attribute__((transparent_union));
#endif


/* Result Structures */

/* NotifyStatus */

/** Current Status */

typedef enum
{
  /*! \brief Ready */

  AS_MNG_STATUS_READY = 0,

  /*! \brief Baseband */

  AS_MNG_STATUS_BASEBAND,

  /*! \brief Player */

  AS_MNG_STATUS_PLAYER,

  /*! \brief Recorder */

  AS_MNG_STATUS_RECORDER,

  /*! \brief Recorder */

  AS_MNG_STATUS_RECOGNIZER,

  /*! \brief PowerOff */

  AS_MNG_STATUS_POWEROFF,

  /*! \brief Through */

  AS_MNG_STATUS_THROUGH,

  AS_MNG_STATUS_NUM
} AsMngStatus;

/** Current Sub Status */

typedef enum
{
  /*! \brief None */

  AS_MNG_SUB_STATUS_NONE = 0,

  /*! \brief PlayReady */

  AS_MNG_SUB_STATUS_PLAYREADY,

  /*! \brief PlayActive */

  AS_MNG_SUB_STATUS_PLAYACTIVE,

  /*! \brief PlayPause */

  AS_MNG_SUB_STATUS_PLAYPAUSE,

  /*! \brief RecorderReady */

  AS_MNG_SUB_STATUS_RECORDERREADY,

  /*! \brief RecorderActive */

  AS_MNG_SUB_STATUS_RECORDERACTIVE,

  /*! \brief RecognizerReady */

  AS_MNG_SUB_STATUS_RECOGNIZERREADY,

  /*! \brief RecognizerActive */

  AS_MNG_SUB_STATUS_RECOGNIZERACTIVE,

  /*! \brief BaseBandReady */

  AS_MNG_SUB_STATUS_BASEBANDREADY,

  /*! \brief BaseBandActive */

  AS_MNG_SUB_STATUS_BASEBANDACTIVE,

  /*! \brief WaitCommandword */

  AS_MNG_SUB_STATUS_WAITCMDWORD,
  AS_MNG_SUB_STATUS_NUM
} AsMngSubStatus;

/** Current VAD Status
 *(valid only NotifyStatus.sub_status_info==#AS_SUB_STATUS_WAITCOMMANDWORD)
 */

typedef enum
{
  /*! \brief Out of voice section */

  AS_VAD_STATUS_OUT_OF_VOICE_SECTION = 0,

  /*! \brief Inside voice section */

  AS_VAD_STATUS_INSIDE_VOICE_SECTION,
  AS_VAD_STATUS_NUM
} AsVadStatus;

/** Changed Status */

typedef enum
{
  /*! \brief to Ready */

  AS_STATUS_CHANGED_STS_READY = 0,

  /*! \brief to BaseBand */

  AS_STATUS_CHANGED_STS_BASEBAND,

  /*! \brief to  Player */

  AS_STATUS_CHANGED_STS_PLAYER,

  /*! \brief to  Recorder */

  AS_STATUS_CHANGED_STS_RECODER,

  /*! \brief to PowerOff */

  AS_STATUS_CHANGED_STS_POWEROFF,

  /*! \brief to Through */

  AS_STATUS_CHANGED_STS_THROUGH,

  AS_STATUS_CHANGED_STS_NUM
} AsStatusChangedSts;

/** Audio Module ID */

/** @defgroup module_id_code Module ID */
/** @{ */

typedef enum
{
  /*! \brief Audio Manager Module ID */

  AS_MODULE_ID_AUDIO_MANAGER = 0,

  /*! \brief Audio Baseband Driver Module ID */
  AS_MODULE_ID_AUDIO_DRIVER,

  /*! \brief FrontEnd Object ID */

  AS_MODULE_ID_MIC_FRONTEND_OBJ,

  /*! \brief Input Data Manager Object ID */

  AS_MODULE_ID_INPUT_DATA_MNG_OBJ,

  /*! \brief Media Recorder Object ID */

  AS_MODULE_ID_MEDIA_RECORDER_OBJ,

  /*! \brief Output Mix Object ID */

  AS_MODULE_ID_OUTPUT_MIX_OBJ,

  /*! \brief Player Object ID */

  AS_MODULE_ID_PLAYER_OBJ,

  /*! \brief Recognition Object ID */

  AS_MODULE_ID_RECOGNITION_OBJ,

  /*! \brief Sound Effect Object ID */

  AS_MODULE_ID_SOUND_EFFECT_OBJ,

  /*! \brief Capture Component ID */

  AS_MODULE_ID_CAPTURE_CMP,

  /*! \brief Decoder Component ID */

  AS_MODULE_ID_DECODER_CMP,

  /*! \brief Encoder Component ID */

  AS_MODULE_ID_ENCODER_CMP,

  /*! \brief Filter Component ID */

  AS_MODULE_ID_FILTER_CMP,

  /*! \brief Recognition Component ID */

  AS_MODULE_ID_RECOGNITION_CMP,

  /*! \brief Renderer Component ID */

  AS_MODULE_ID_RENDERER_CMP,

  /*! \brief Postfilter Component ID */

  AS_MODULE_ID_POSTPROC_CMP,
  AS_MODULE_ID_NUM,
} AsModuleId;

/** @} */

/** Audio Manager error code */

typedef enum
{
  /*! \brief success */
  
  AS_ERR_CODE_OK = 0,
  
  /*! \brief fail to create task */
  
  AS_ERR_CODE_TASK_CREATE = -1,
  
  /*! \brief illegal state */
  
  AS_ERR_CODE_ILLEGAL_STATE = -2
} AsErrorCode_e;

/** Select volume mute/unmute */

typedef enum
{
  /*! \brief keep setting */

  AS_VOLUMEMUTE_HOLD,

  /*! \brief unmute */

  AS_VOLUMEMUTE_UNMUTE,

  /*! \brief mute */

  AS_VOLUMEMUTE_MUTE,
  AS_VOLUMEMUTE_NUM
} AsVolumeMute;

/** Select beep mode */

typedef enum
{
  /*! \brief beep disable */

  AS_BEEPEN_DISABLE,

  /*! \brief beep enable */

  AS_BEEPEN_ENABLE,
  AS_BEEPEN_NUM
} asBeepEn;

/** NotifyStatus Result (#AUDRLT_NOTIFYSTATUS) parameter */

typedef struct
{
  /*! \brief VAD status
   *
   * #AsNotifyVadStatus enum type
   */

  uint8_t  vad_status;

  /*! \brief [out] reserved */

  uint8_t  reserved;

  /*! \brief [out] Current Sub Status
   *
   * #AsNotifySubStatusInfo enum type
   */

  uint8_t  sub_status_info;

  /*! \brief [out] Current Status
   *
   * #AsNotifyStatusInfo enum type
   */

  uint8_t  status_info;
} NotifyStatus;

/** StatusChanged Result (#AUDRLT_STATUSCHANGED) parameter */

typedef struct
{
  /*! \brief [out] Changed status
   *
   *#AsStatusChangedSts enum type
   */

  uint8_t  changed_status;

  /*! \brief reserved */

  uint8_t  reserved1;

  /*! \brief reserved */

  uint8_t  reserved2;

  /*! \brief reserved */

  uint8_t  reserved3;
} StatusChangedParam;

/** ErrorResponse Result (#AUDRLT_ERRORRESPONSE) parameter */

typedef struct
{
  /*! \brief Error Infomation, T.B.D. */

  uint32_t error_code;

  /*! \brief [out] reserved */

  uint16_t reserved1;

  /*! \brief [out] for debug purpose */

  uint8_t  sub_module_id;

  /*! \brief [out] Error module infomation, T.B.D. */

  uint8_t  module_id;

  /*! \brief [out] Detailed Error Infomation, T.B.D. */

  uint32_t error_sub_code;

  /*! \brief [out] reserved */

  uint32_t reserved2;

  /*! \brief [out] Head part of the command packet that caused the Error */

  uint32_t ErrorCommand[LENGTH_AUDRLT_ERRORRESPONSE_MAX-LENGTH_AUDRLT_ERRORRESPONSE_MIN];
} ErrorResponseParam;

/** Audio result packet
 */
#if defined(__CC_ARM)
#pragma anon_unions
#endif

typedef struct {
  /*! \brief [out] Result Header */

  AudioResultHeader header;

  union
  {
    /*! \brief [out] for NotifyStatus
     *  (header.result_code==#AUDRLT_NOTIFYSTATUS)
     */

    NotifyStatus   notify_status;

    /*! \brief [out] for StatusChanged
     * (header.result_code==#AUDRLT_STATUSCHANGED)
     */

    StatusChangedParam status_changed_param;

    /*! \brief [out] for ErrorResponse
     * (header.result_code==#AUDRLT_ERRORRESPONSE)
     */

    ErrorResponseParam error_response_param;
  };

#if !defined(__CC_ARM)
} AudioResult ;
#else
} AudioResult __attribute__((transparent_union));
#endif

/* Error Code */
/* [T.B.D]
 *
 * no error = 0
 * state error
 * paramater error
 * timeout
 *
 */

/** Activate function parameter */

typedef struct
{
  /*! \brief [in] MsgQueID of Application */

  uint8_t app;

  /*! \brief [in] MsgQueID of audio_manager */

  uint8_t mng;

  /*! \brief [in] MsgQueID of playerObject */

  uint8_t player_main;

  /*! \brief [in] MsgQueID of playerObject for Sound Effect */

  uint8_t player_sub;

  /*! \brief [in] MsgQueID of FrontendObject */

  uint8_t micfrontend;

  /*! \brief [in] MsgQueID of recorderObject */

  uint8_t recorder;

  /*! \brief [in] MsgQueID of mixerObject */

  uint8_t mixer;

  /*! \brief [in] MsgQueID of effectorObject */

  uint8_t effector;

  /*! \brief [in] MsgQueID of recognizerObject */

  uint8_t recognizer;
} AudioSubSystemIDs;

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
 * @brief Send Audio Command
 *
 * @param[in] packet: AudioCommand* Command packet
 *
 * @retval error code
 */

int AS_SendAudioCommand(AudioCommand* packet);

/**
 * @brief Receive Audio Result
 *
 * @param[out] packet: AudioResult*: Result packet
 *
 * @retval error code
 */

int AS_ReceiveAudioResult(AudioResult* packet);

/**
 * @brief Activate AudioSubSystem
 *
 * @param[in] ids: AudioSubSystemIDs* Message Queue ID of Audio Module
 *
 * @retval error code
 */

int AS_CreateAudioManager(AudioSubSystemIDs ids, AudioAttentionCb att_cb);

/**
 * @brief Activate AudioSubSystem[Deprecated]
 *        This API is to make it compatible with old application.
 *        Will delete most application seems to migrate to new API.
 *
 * @param[in] ids: AudioSubSystemIDs* Message Queue ID of Audio Module
 *
 * @retval error code
 */

__attribute__((deprecated("\nDeprecated attention callback type is used. \
                           \nPlease use \"AudioAttentionCb\" as callback type. \
                           \n")))
int AS_CreateAudioManager(AudioSubSystemIDs ids, obs_AudioAttentionCb obs_att_cb);

/**
 * @brief Deactivate AudioSubSystem
 *
 * @retval error code
 */

int AS_DeleteAudioManager(void);

/** @} */

#endif /* __MODULES_INCLUDE_AUDIO_AUDIO_HIGH_LEVEL_API_H */

/**
 * @}
 */
/**
 * @}
 */
