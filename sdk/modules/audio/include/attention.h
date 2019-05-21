/****************************************************************************
 * modules/audio/include/attention.h
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

#ifndef __MODULES_AUDIO_INCLUDE_ATTENTION_H
#define __MODULES_AUDIO_INCLUDE_ATTENTION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "audio/audio_high_level_api.h"
#include "memutils/message/Message.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct attention_info_s
{
  uint8_t module_id;
  uint8_t attention_id;
  uint8_t sub_code;
#ifdef ATTENTION_USE_FILENAME_LINE
  FAR const char *file_name;
  uint32_t line;
#endif /* ATTENTION_USE_FILENAME_LINE */
};
typedef struct attention_info_s AttentionInfo;

#ifdef __cplusplus
extern "C" {
#endif

/* Implement with the following IF for each project. */

extern void _RegisterAttentionCb(uint32_t module_id, AudioAttentionCb att_cb);
extern void _UnregisterAttentionCb(uint32_t module_id);

#ifdef ATTENTION_USE_FILENAME_LINE
extern void _Attention(uint8_t module_id, uint8_t sub_module_id, uint8_t attention_code, uint8_t sub_code,
		       const char* filename, uint16_t line);
#else
extern void _Attention(uint8_t module_id, uint8_t sub_module_id, uint8_t attention_code, uint8_t sub_code);
#endif

#define ATTENTION_CB_REGISTER(module_id, att_cb) \
    _RegisterAttentionCb(module_id, att_cb)
#define ATTENTION_CB_UNREGISTER(module_id) \
    _UnregisterAttentionCb(module_id)

#ifdef ATTENTION_USE_FILENAME_LINE
#define INFORMATION_ATTENTION(module_id, sub_module_id, sub_code) \
	_Attention((module_id), (sub_module_id), AS_ATTENTION_CODE_INFORMATION, (sub_code), __FILE__, __LINE__)
#define WARNING_ATTENTION(module_id, sub_module_id, sub_code) \
	_Attention((module_id), (sub_module_id), AS_ATTENTION_CODE_WARNING, (sub_code), __FILE__, __LINE__)
#define ERROR_ATTENTION(module_id, sub_module_id, sub_code) \
	_Attention((module_id), (sub_module_id), AS_ATTENTION_CODE_ERROR, (sub_code), __FILE__, __LINE__)
#define FATAL_ATTENTION(module_id, sub_module_id, sub_code) \
	_Attention((module_id), (sub_module_id), AS_ATTENTION_CODE_FATAL, (sub_code), __FILE__, __LINE__)
#else /* ATTENTION_USE_FILENAME_LINE */
#define INFORMATION_ATTENTION(module_id, sub_module_id, sub_code) \
	_Attention((module_id), (sub_module_id), AS_ATTENTION_CODE_INFORMATION, (sub_code))
#define WARNING_ATTENTION(module_id, sub_module_id, sub_code) \
	_Attention((module_id), (sub_module_id), AS_ATTENTION_CODE_WARNING, (sub_code))
#define ERROR_ATTENTION(module_id, sub_module_id, sub_code) \
	_Attention((module_id), (sub_module_id), AS_ATTENTION_CODE_ERROR, (sub_code))
#define FATAL_ATTENTION(module_id, sub_module_id, sub_code) \
	_Attention((module_id), (sub_module_id), AS_ATTENTION_CODE_FATAL, (sub_code))
#endif /* ATTENTION_USE_FILENAME_LINE */

#ifdef __cplusplus
}
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __MODULES_AUDIO_INCLUDE_ATTENTION_H */
