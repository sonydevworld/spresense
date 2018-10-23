/****************************************************************************
 * modules/include/bluetooth/bt_hfp_features.h
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

/**
 * @file bt_hfp_features.h
 * @author Sony Semiconductor Solutions Corporation
 * @brief Bluetooth HFP common header for SDK on Spresense.
 * @details This header file includes bluetooth HFP common definition between
 *          API and HAL I/F.
 *           - Profile type
 *           - Support feature flag
 */

#ifndef __MODULES_INCLUDE_BLUETOOTH_BT_HFP_FEATURES_H
#define __MODULES_INCLUDE_BLUETOOTH_BT_HFP_FEATURES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @enum BT_PROFILE_TYPE
 * @brief BT HFP profile type
 */
typedef enum {
	BT_HFP_PROFILE = 0, /**< Hands free profile */
	BT_HSP_PROFILE      /**< Head set profile */
} BT_PROFILE_TYPE;

/**
 * @enum BT_HFP_HF_FEATURE_FLAG
 * @brief HFP HF device supported feature flags.
 */
typedef enum
{
  BT_HFP_HF_FEATURE_ECNR                         = 0x00000001 << 0, /**< EC and/or NR function */
  BT_HFP_HF_FEATURE_3WAY_CALLING                 = 0x00000001 << 1, /**< Three-way calling */
  BT_HFP_HF_FEATURE_CLIP_CAPABILITY              = 0x00000001 << 2, /**< CLI presentation capability */
  BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION = 0x00000001 << 3, /**< Voice recognition activation */
  BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL        = 0x00000001 << 4, /**< Remote volume control */
  BT_HFP_HF_FEATURE_ENHANCED_CALL_STATUS         = 0x00000001 << 5, /**< Enhanced call status */
  BT_HFP_HF_FEATURE_ENHANCED_CALL_CONTROL        = 0x00000001 << 6, /**< Enhanced call control */
  BT_HFP_HF_FEATURE_CODEC_NEGOTIATION            = 0x00000001 << 7, /**< Codec negotiation */
  BT_HFP_HF_FEATURE_HF_INDICATORS                = 0x00000001 << 8, /**< HF indicators */
  BT_HFP_HF_FEATURE_ESCO_S4_T2_SETTINGS_SUPPORT  = 0x00000001 << 9, /**< eSCO S4 (and T2) Settings Supported */
  BT_HFP_HF_FEATURE_ENHANCED_VOICE_RECOGNITION   = 0x00000001 << 10 /**< Enhanced voice recognition */
} BT_HFP_HF_FEATURE_FLAG;

/**
 * @enum BT_HFP_AG_FEATURE_FLAG
 * @brief HFP AG device supported feature flags.
 */
typedef enum
{
  BT_HFP_AG_FEATURE_3WAY_CALLING                 = 0x00000001 << 0,  /**< Three-way calling */
  BT_HFP_AG_FEATURE_ECNR                         = 0x00000001 << 1,  /**< EC and/or NR function */
  BT_HFP_AG_FEATURE_VOICE_RECOGNITION_ACTIVATION = 0x00000001 << 2,  /**< Voice recognition function */
  BT_HFP_AG_FEATURE_INBAND_RING_TONE_CAPABILITY  = 0x00000001 << 3,  /**< In-band ring tone capability */
  BT_HFP_AG_FEATURE_ATTACH_NUMBER_TO_VOICE_TAG   = 0x00000001 << 4,  /**< Attach a number to a voice tag */
  BT_HFP_AG_FEATURE_ABILITY_TO_REJECT_CALL       = 0x00000001 << 5,  /**< Ability to reject a call */
  BT_HFP_AG_FEATURE_ENHANCED_CALL_STATUS         = 0x00000001 << 6,  /**< Enhanced call status */
  BT_HFP_AG_FEATURE_ENHANCED_CALL_CONTROL        = 0x00000001 << 7,  /**< Enhanced call control */
  BT_HFP_AG_FEATURE_EXTENDED_ERROR_RESULT_CODES  = 0x00000001 << 8,  /**< Extended Error Result Codes */
  BT_HFP_AG_FEATURE_CODEC_NEGOTIATION            = 0x00000001 << 9,  /**< Codec negotiation */
  BT_HFP_AG_FEATURE_HF_INDICATORS                = 0x00000001 << 10, /**< HF indicators */
  BT_HFP_AG_FEATURE_ESCO_S4_T2_SETTINGS_SUPPORT  = 0x00000001 << 11, /**< eSCO S4 (and T2) Settings Supported */
  BT_HFP_AG_FEATURE_ENHANCED_VOICE_RECOGNITION   = 0x00000001 << 12  /**< Enhanced voice recognition */
} BT_HFP_AG_FEATURE_FLAG;

#endif /* __MODULES_INCLUDE_BLUETOOTH_BT_HFP_FEATURES_H */
