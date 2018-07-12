/****************************************************************************
 * modules/include/memutils/common_utils/common_attention.h
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

#ifndef __COMMON_ATTENTION_H_INCLUDED__
#define __COMMON_ATTENTION_H_INCLUDED__

typedef enum {
	INFORMATION_ATTENTION_CODE = 0, /* Just Information */
	WARNING_ATTENTION_CODE,         /* Warning. 自律復帰可能 */
	ERROR_ATTENTION_CODE,           /* 上位からの制御次第で復帰可能 */
	FATAL_ATTENTION_CODE,           /* 致命的な状態. リセットが必要 */
} ErrorAttensionCode;

#ifdef __cplusplus
extern "C" {
#endif

/** 各プロジェクト毎に下記 IF で実装する
 **/
#ifdef ATTENTION_USE_FILENAME_LINE
extern void _Attention(uint8_t module_id, uint8_t attention_code, uint8_t sub_code,
		       const char* filename, uint32_t line);
#else
extern void _Attention(uint8_t module_id, uint8_t attention_code, uint8_t sub_code);
#endif


#ifdef ATTENTION_USE_FILENAME_LINE
#define INFORMATION_ATTENTION(module_id, sub_code) \
	_Attention((module_id), INFORMATION_ATTENTION_CODE, (sub_code), __FILE__, __LINE__)
#define WARNING_ATTENTION(module_id, sub_code) \
	_Attention((module_id), WARNING_ATTENTION_CODE, (sub_code), __FILE__, __LINE__)
#define ERROR_ATTENTION(module_id, sub_code) \
	_Attention((module_id), ERROR_ATTENTION_CODE, (sub_code), __FILE__, __LINE__)
#define FATAL_ATTENTION(module_id, sub_code) \
	_Attention((module_id), FATAL_ATTENTION_CODE, (sub_code), __FILE__, __LINE__)
#else /* ATTENTION_USE_FILENAME_LINE */
#define INFORMATION_ATTENTION(module_id, sub_code) \
	_Attention((module_id), INFORMATION_ATTENTION_CODE, (sub_code))
#define WARNING_ATTENTION(module_id, sub_code) \
	_Attention((module_id), WARNING_ATTENTION_CODE, (sub_code))
#define ERROR_ATTENTION(module_id, sub_code) \
	_Attention((module_id), ERROR_ATTENTION_CODE, (sub_code))
#define FATAL_ATTENTION(module_id, sub_code) \
	_Attention((module_id), FATAL_ATTENTION_CODE, (sub_code))
#endif /* ATTENTION_USE_FILENAME_LINE */

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_ATTENTION_H_INCLUDED__ */
