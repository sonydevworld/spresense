/****************************************************************************
 * modules/include/utils/uconv.h
 *
 *   Copyright 2020 Sony Semiconductor Solutions Corporation
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

#ifndef __MODULES_INCLUDE_UTILS_UCONV_H
#define __MODULES_INCLUDE_UTILS_UCONV_H

/**
 * @defgroup uconv unicode converter API
 * @brief Unicode converter library
 *
 * @{
 * @file  uconv.h
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/** @name Functions */
/** @{ */

/**
 * Convert a string from UTF-8 format to UCS2.
 *
 * @param [in] src_size: Byte length of string encoded in UTF-8.
 *
 * @param [in] src: String pointer encoded in UTF-8.
 *
 * @param [in] dst_size: Maximum Byte length of UCS2 string buffer.
 *
 * @param [out] dst: Pointer stores the character string
 *                   converted to UCS2 format.
 *
 * @return On success, it returns the number of bytes
 *         in the converted string buffer.
 *         Otherwise negative value is returned according to <errno.h>.
 */

int uconv_utf8_to_ucs2(int src_size, uint8_t *src,
                       int dst_size, uint16_t *dst);

/**
 * Convert a string from UCS2 format to UTF-8.
 *
 * @param [in] src_size: Byte length of string encoded in UCS2.
 *
 * @param [in]  src: String pointer encoded in UCS2.
 *
 * @param [in] dst_size: Maximum Byte length of UTF-8 string buffer.
 *
 * @param [out] dst: Pointer stores the character string
 *                   converted to UTF-8 format.
 *
 * @return On success, it returns the number of bytes
 *         in the converted string buffer.
 *         Otherwise negative value is returned according to <errno.h>.
 *
 */

int uconv_ucs2_to_utf8(int src_size, uint16_t *src,
                       int dst_size, uint8_t *dst);

/** @} */

/** @} */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MODULES_INCLUDE_UTILS_UCONV_H */
