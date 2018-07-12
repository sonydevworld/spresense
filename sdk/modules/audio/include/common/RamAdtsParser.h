/****************************************************************************
 * modules/audio/include/common/RamAdtsParser.h
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

#ifndef __MODULES_AUDIO_INCLUDE_COMMON_RAMADTSPARSER_H
#define __MODULES_AUDIO_INCLUDE_COMMON_RAMADTSPARSER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/common_utils/common_types.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Result of header validity check */

#define  HDR_OK            0x0000  /* No Error */
#define  HDR_SYNCWORD_NG   0x0001  /* Error: syncword not found */
#define  HDR_PROFILE_NG    0x0002  /* Error: profile NG (not AAC-LC) */
#define  HDR_SAMLERATE_NG  0x0004  /* Error: Invalid Sample Rate */
#define  HDR_FRAMESIZE_NG  0x0008  /* Error: Frame Size */
#define  HDR_ERROR         0x8000  /* Undefined Error */

#define PARSER_LOCAL_POLL_BUFFERSIZE 1024

/* ADTS-API return value */

#define  ADTS_OK    0    /* Success */
#define  ADTS_ERR   1    /* Fail */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ADTS-DATA access handle (User in Parser) */

struct adts_handle_s
{
  FAR CMN_SimpleFifoHandle  *pSimpleFifoHandler;
  uint32_t        current_pos;      /* Current position(offset from top) */
  uint32_t        search_pos;       /* Search Position(offset from top) */
  uint32_t        parse_size;       /* Parse size */
};
typedef struct adts_handle_s AdtsHandle;

enum adts_parser_error_detail_e
{
  AdtsParserNormal = 0,       /* Normal (No error) */
  AdtsParserAbnormalArg,      /* Argument error (when return valuse = ADTS_ERR) */
  AdtsParserConnotDataAccess, /* File access error (when return valuse = ADTS_ERR) */
  AdtsParserCannotGetHeader,  /* Cannnot Get Header (when return valuse = ADTS_ERR) */
  AdtsParserAbnormalHeader,   /* Abnormal header (when return valuse = ADTS_ERR) */
  AdtsParserShortageBuffer,   /* Shortage buffer (when return valuse = ADTS_ERR) */
};
typedef enum adts_parser_error_detail_e AdtsParserErrorDetail;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/*!
 * @brief Initialize ADTS Parser
 *
 * @param[in] pHandle Pointer to ADTS-DATA access handle
 *
 * @param[in] simple_fifo_handler Pointer SimpleFIFO handle
 *
 * @param[out] uipErrDetail Error Detail
 *
 * @return Function return code
 */

int32_t AdtsParser_Initialize(FAR AdtsHandle *pHandle,
                              FAR CMN_SimpleFifoHandle *pSimpleFifoHandler,
                              FAR AdtsParserErrorDetail *uipErrDetail);

/*!
 * @brief Get a frame from ADTS Data
 *
 * @param[in] pHandle Pointer to ADTS-DATA access handle
 *
 * @param[in] buff Pointer to result(frame) buffer
 *
 * @param[in/out] uiSize - [in] result buffer size  [out] Frame size 
 *
 * @param[out] usResult Result of header validity check
 *
 * @param[out] uipErrDetail Error Detail
 *
 * @return Function return code
 */

int32_t AdtsParser_ReadFrame(FAR AdtsHandle *pHandle,
                             FAR int8_t *buff,
                             FAR uint32_t *uiSize,
                             FAR uint16_t *usResult,
                             FAR AdtsParserErrorDetail *uipErrDetail);

/*!
 * @brief Finalize ADTS Parser
 *
 * @param[in] pHandle Pointer to ADTS-DATA access handle
 *
 * @param[out] uipErrDetail Error Detail
 *
 * @return Function return code
 */

int32_t AdtsParser_Finalize(FAR AdtsHandle *pHandle,
                            FAR AdtsParserErrorDetail *uipErrDetail);

/*!
 * @brief Get Sampling frequency (Simply read from ADTS header)
 *
 * @param[in] pHandle Pointer to ADTS-DATA access handle
 *
 * @param[in] uipSmplingRate pointer Sampling frequency
 *
 * @param[out] uipErrDetail Error Detail
 *
 * @return Function return code
 */

int32_t AdtsParser_GetSamplingRate(FAR AdtsHandle *pHandle,
                                   FAR uint32_t *uipSmplingRate,
                                   FAR AdtsParserErrorDetail *uipErrDetail);

#endif /* __MODULES_AUDIO_INCLUDE_COMMON_RAMADTSPARSER_H */
