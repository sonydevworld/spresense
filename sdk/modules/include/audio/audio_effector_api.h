/****************************************************************************
 * modules/include/audio/audio_effector_api.h
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

#ifndef __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_EFFECTOR_API_H
#define __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_EFFECTOR_API_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/* API Documents creater with Doxgen */

/**
 * @defgroup audioutils_audio_effector_api Audio Effector API
 * @{
 *
 * @file       audio_effector_api.h
 * @brief      CXD5602 Audio Effector API
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

#define AS_FEATURE_EFFECTOR_ENABLE

/** @name sub code for SETMPPPARAM command(0x2C) */
/** @{ */

/*! \brief Sub Code: MPP common setting */

#define  SUB_SETMPP_COMMON    0x00

/*! \brief Sub Code: MPP xLOUD setting */

#define  SUB_SETMPP_XLOUD     0x01

/** @} */

/** @name Packet length of command*/
/** @{ */

/*! \brief StartBB command (#AUDCMD_STARTBB) packet length */

#define  LENGTH_STARTBB             3

/*! \brief StopBB command (#AUDCMD_STOPBB) packet length */

#define  LENGTH_STOPBB              3

/** @} */

/** Check xLOUD volume range */

#define CHECK_XLOUD_VOLUME_RANGE(vol)  \
            (bool)(((0 <= (vol)) && ((vol) <= 59)) ? true : false)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* enum for baseband */

/* StartBB */

/** Select Output Device */

typedef enum
{
  /*! \brief I2S 2ch
   *  \deprecated It will be removed in the future
   */

  AS_OUTPUT_DEVICE_I2S2CH = 0x03,

  /*! \brief SP/HP 2ch
   *  \deprecated It will be removed in the future
   */

  AS_OUTPUT_DEVICE_SP2CH = 0x30,

  /*! \brief I2S 2ch, SP/HP 2ch */

  AS_OUTPUT_DEVICE_SP2CH_I2S2CH = (AS_OUTPUT_DEVICE_I2S2CH | AS_OUTPUT_DEVICE_SP2CH),
} AsOutputDevice;

/** Select Input Device */

typedef enum
{
  /*! \brief I2S 2ch
   *  \deprecated It will be removed in the future
   */

  AS_INPUT_DEVICE_I2S2CH = 0x0003,

  /*! \brief Analog Mic 1ch
   *  \deprecated It will be removed in the future
   */

  AS_INPUT_DEVICE_AMIC1CH = 0x0100,

  /*! \brief Analog Mic 1ch, I2S 2ch */

  AS_INPUT_DEVICE_AMIC1CH_I2S2CH = (AS_INPUT_DEVICE_I2S2CH | AS_INPUT_DEVICE_AMIC1CH),

  /*! \brief Analog Mic 4ch, I2S 2ch
   *  \deprecated It will be removed in the future
   */

  AS_INPUT_DEVICE_AMIC4CH_I2S2CH = (AS_INPUT_DEVICE_I2S2CH | 0x0F00)
} AsInputDevice;

/** Select SP/HP output data */

typedef enum
{
  /*! \brief MPP output (I2S in) */

  AS_MPP_OUTPUT_I2SIN = 0,
  AS_SP_OUTPUT_DATA_NUM
} AsSpOutputData;

/** Select I2S output data */

typedef enum
{
  /*! \brief MFE output (MIC in) */

  AS_MFE_OUTPUT_MICSIN = 0,

  /*! \brief MIC through */

  AS_MIC_THROUGH,
  AS_I2S_OUTPUT_DATA_NUM
} AsI2sOutputData;

/** Select output MIC channels
 *  (valid only #StartBBParam.I2S_output_data == #AS_MIC_THROUGH)
 */

typedef enum
{
  /*! \brief MIC1 and MIC2 */

  AS_SELECT_MIC1_OR_MIC2 = 0x06,

  /*! \brief MIC0 and MIC3 */

  AS_SELECT_MIC0_OR_MIC3 = 0x09
} AsSelectOutputMic;

/* InitMFE */

/** Select MFE mode */

typedef enum
{
  /*! \brief Voice Recognition mode (default) */

  AS_MFE_MODE_RECOGNITION = 0,

  /*! \brief Voice Speaking mode */

  AS_MFE_MODE_SPEAKING,
  AS_MFE_MODE_NUM
} AsMfeModeId;

/** Select EchoCancel configuration */

typedef enum
{
  /*! \brief Not include echochancel function */

  AS_NOINCLUDE_ECHOCANCEL = 0,

  /*! \brief Incluce echochancel function */

  AS_INCLUDE_ECHOCANCEL,
  AS_MFE_INC_ECHOCANCEL_NUM
} AsMfeIncludeEchoCancel;

/** Select EchoCancel function */

typedef enum
{
  /*! \brief EchoCancel OFF */

  AS_DISABLE_ECHOCANCEL = 0,

  /*! \brief EchoCancel ON
   *  \deprecated It will be removed in the future
   */

  AS_ENABLE_ECHOCANCEL,
  AS_MFE_ENBL_ECHOCANCEL_NUM
} AsMfeEnableEchoCancel;

/* InitMPP */

/** Select MPP mode */

typedef enum
{
  /*! \brief xLOUD only */

  AS_MPP_MODE_XLOUD_ONLY = 0,
  AS_MPP_MODE_NUM
} AsMppModeId;

/** Select MPP coefficient table */

typedef enum
{
  /*! \brief for Speaker */

  AS_MPP_COEF_SPEAKER = 0,

  /*! \brief for Headphone */

  AS_MPP_COEF_HEADPHONE,
  AS_MPP_COEF_NUM
} AsMppCoefModeId;

/** Select xLOUD mode */

typedef enum
{
  /*! \brief Normal mode */

  AS_MPP_XLOUD_MODE_NORMAL = 0,

  /*! \brief Speaking mode */

  AS_MPP_XLOUD_MODE_SPEAKING,

  /*! \brief Disable */

  AS_MPP_XLOUD_MODE_DISABLE,
  AS_MPP_XLOUD_MODE_NUM
} AsMppXloudModeId;

/** Select EAX mode */

  typedef enum
{
  /*! \brief Off */

  AS_MPP_EAX_DISABLE = 0,

  /*! \brief On */

  AS_MPP_EAX_ENABLE,
  AS_MPP_EAX_NUM
} AsMppEaxModeId;

/* SetBaseBandStatus */

/** Select MFE function */

typedef enum
{
  /*! \brief no MFE function */

  AS_SET_BBSTS_WITH_MFE_NONE = 0,

  /*! \brief MFE active */

  AS_SET_BBSTS_WITH_MFE_ACTIVE,
  AS_SET_BBSTS_WITH_MFE_NUM
} AsSetBBStsWithMfe;

/** Select Voice Command function */

typedef enum
{
  /*! \brief no Voice Command function */

  AS_SET_BBSTS_WITH_VCMD_NONE = 0,

  /*! \brief Voice Command active */

  AS_SET_BBSTS_WITH_VCMD_ACTIVE,
  AS_SET_BBSTS_WITH_VCMD_NUM
} AsSetBBStsWithVoiceCommand;

/** Select MPP function */

typedef enum
{
  /*! \brief no MPP function */

  AS_SET_BBSTS_WITH_MPP_NONE = 0,

  /*! \brief MPP active
   *  \deprecated It will be removed in the future
   */

  AS_SET_BBSTS_WITH_MPP_ACTIVE,
  AS_SET_BBSTS_WITH_MPP_NUM
} AsSetBBStsWithMpp;

/** InitMFE Command (#AUDCMD_INITMFE) parameter */

typedef struct
{
  /*! \brief [in] Select MFE sampling frequency
   *
   * Use #AsMfeInputFsId enum type
   */

  uint16_t input_fs;

  /*! \brief [in] Select MFE SP numbers
   *
   * Use #AsMfeRefChNum enum type
   */

  uint8_t  ref_channel_num;

  /*! \brief [in] Select MFE MIC numbers
   *
   * Use #AsMfeMicChNum enum type
   */

  uint8_t  mic_channel_num;

  /*! \brief [in] Select EchoCancel function,
   *
   * Use #AsMfeEnableEchoCancel enum type
   */

  uint8_t  enable_echocancel;

  /*! \brief [in] Select EchoCancel configuration,
   *
   * Use #AsMfeIncludeEchoCancel enum type
   */

  uint8_t  include_echocancel;

  /*! \brief [in] reserved */

  uint8_t  reserved2;

  /*! \brief [in] Select MFE mode
   *
   * Use #AsMfeModeId enum type
   */

  uint8_t  mfe_mode;

  /*! \brief [in] Configuration table address */

  uint32_t config_table;
} InitMFEParam;

/** StartBB Command (#AUDCMD_STARTBB) parameter */

typedef struct
{
  /*! \brief [in] Select Output Device
   *
   * Use #AsOutputDevice enum type
   */

  uint8_t  output_device;

  /*! \brief [in] reserved */

  uint8_t  reserved1;

  /*! \brief [in] Select Input Device
   *
   * Use #AsInputDevice enum type
   */

  uint16_t input_device;

  /*! \brief [in] Select output MIC channels
   *
   *  valid only #I2S_output_data == #AS_MIC_THROUGH,
   *  Use #AsSelectOutputMic enum type
   */

  uint8_t  select_output_mic;

  /*! \brief [in] reserved */

  uint8_t  reserved2;

  /*! \brief [in] Select I2S output data
   *
   * Use #AsI2sOutputData enum type
   */

  uint8_t  I2S_output_data;

  /*! \brief [in] Select SP/HP output data
   *
   * Use #AsSpOutputData enum type
   */

  uint8_t  SP_output_data;
} StartBBParam;

/** StopBB Command (#AUDCMD_STOPBB) parameter */

typedef struct
{
  /*! \brief [in] T.B.D. */

  uint8_t  stop_device;

  /*! \brief [in] reserved */

  uint8_t  reserved1;

  /*! \brief [in] reserved */

  uint16_t reserved2;

  /*! \brief [in] reserved */

  uint8_t  reserved3;

  /*! \brief [in] reserved */

  uint8_t  reserved4;

  /*! \brief [in] reserved */

  uint8_t  reserved5;

  /*! \brief [in] reserved */

  uint8_t  reserved6;
} StopBBParam;

/** SetBaseBandStatus Command (#AUDCMD_SETBASEBANDSTATUS) parameter */

typedef struct
{
  /*! \brief [in] reserved */

  uint8_t  reserved1;

  /*! \brief [in] Select MPP function
   *
   * Use #AsSetBBStsWithMpp enum type
   */

  uint8_t  with_MPP;

  /*! \brief [in] Select Voice Command function
   *
   * Use #AsSetBBStsWithVoiceCommand enum type
   */

  uint8_t  with_Voice_Command;

  /*! \brief [in] Select MFE function
   *
   * Use #AsSetBBStsWithMfe enum type
   */

  uint8_t  with_MFE;

  /*! \brief [in] Select Input Device
   *
   * Use #AsInputDevice enum type
   */

  uint16_t input_device;

  /*! \brief [in] reserved */

  uint8_t  reserved2;

  /*! \brief [in] Select Output Device
   *
   * Use #AsOutputDevice enum type
   */

  uint8_t  output_device;

  /*! \brief [in] Audio DSP path */

  char dsp_path[AS_AUDIO_DSP_PATH_LEN];

} SetBaseBandStatusParam;

/** Message queue ID parameter of Activate API */

typedef struct
{
  /*! \brief [in] Message queue id of sound effector */

  uint8_t effector;

  /*! \brief [in] Message queue id of audio_manager */

  uint8_t mng;

  /*! \brief [in] Message queue id of voice recognizer */

  uint8_t recognizer;

  /*! \brief [in] Message queue id of DSP */

  uint8_t dsp;
} AsEffectorMsgQueId_t;

/** Pool ID parameter of Activate API */

typedef struct
{
  /*! \brief [in] Memory pool id of mic input data */

  uint8_t mic_in;

  /*! \brief [in] Memory pool id of i2s input data */

  uint8_t i2s_in;

  /*! \brief [in] Memory pool id of speaker/headphone output data */

  uint8_t sphp_out;

  /*! \brief [in] Memory pool id of i2s output data */

  uint8_t i2s_out;

  /*! \brief [in] Memory pool id of mfe output data */

  uint8_t mfe_out;
} AsEffectorPoolId_t;

/** Activate API parameter */

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsEffectorMsgQueId_t msgq_id;

  /*! \brief [in] ID of memory pool for processing data */

  AsEffectorPoolId_t   pool_id;
} AsCreateEffectorParam_t;

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
 * @brief Activate sound effector feature
 *
 * @param[in] param: Parameters of resources used by sound effector
 * @param[in] attcb: Attention callback of Player. NULL means no callback.
 *
 * @retval     true  : success
 * @retval     false : failure
 */
bool AS_CreateEffector(FAR AsCreateEffectorParam_t *param,
                       AudioAttentionCb attcb);

__attribute__((deprecated(
                 "\n \
                  \n Deprecated create API is used. \
                  \n Use \"AS_CreateEffector(AsCreateEffectorParam_t * \
                  \n                        AudioAttentionCb)\". \
                  \n \
                  \n")))
bool AS_CreateEffector(FAR AsCreateEffectorParam_t *param);

/**
 * @brief Deactivate sound effector feature
 *
 * @retval     true  : success
 * @retval     false : failure
 */
bool AS_DeleteEffector(void);

#endif  /* __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_EFFECTOR_API_H */
/**
 * @}
 */

/**
 * @}
 */
