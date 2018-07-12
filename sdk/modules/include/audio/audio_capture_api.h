/****************************************************************************
 * modules/include/audio/audio_capture_api.h
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

#ifndef __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_CAPTURE_API_H
#define __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_CAPTURE_API_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/**
 * @defgroup audioutils_audio_capture_api Audio Capture API
 * @{
 *
 * @file       audio_capture_api.h
 * @brief      CXD5602 Audio Capture API
 * @author     CXD5602 Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Message queue ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Message queue id of capture device0 request */

  uint8_t dev0_req;

  /*! \brief [in] Message queue id of capture device0 for syncronizing */

  uint8_t dev0_sync;

  /*! \brief [in] Message queue id of capture device1 request */

  uint8_t dev1_req;

  /*! \brief [in] Message queue id of capture device1 for syncronizing */

  uint8_t dev1_sync;
} AsCaptureMsgQueId_t;

/** Activate function parameter */

typedef struct
{
  /*! \brief [in] ID for sending messages to each channels */

  AsCaptureMsgQueId_t msgq_id;
} AsCreateCaptureParam_t;

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
 * @brief Activate audio capture feature
 *
 * @param[in] param: Parameters of resources used by capture
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_CreateCapture(FAR AsCreateCaptureParam_t *param);

/**
 * @brief Deactivate audio capture feature
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeleteCapture(void);

#ifdef __cplusplus
}
#endif

#endif  /* __SONY_APPS_INCLUDE_AUDIOUTIL_AUDIO_CAPTURE_API_H */
/**
 * @}
 */

/**
 * @}
 */
