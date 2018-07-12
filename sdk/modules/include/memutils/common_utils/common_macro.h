/****************************************************************************
 * modules/include/memutils/common_utils/common_macro.h
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

#ifndef COMMON_MACRO_H_INCLUDED
#define COMMON_MACRO_H_INCLUDED

#include <stddef.h>	/* size_t, offsetof */

#ifndef __cplusplus
#ifndef __bool_true_false_are_defined
#define __bool_true_false_are_defined
typedef enum { false, true } bool;
#endif /* __bool_true_false_are_defined */
#endif /* __cplusplus */

#define ALIGN_OF(type) offsetof(struct { char x; type y; }, y)
#define COUNT_OF(array) (sizeof(array) / sizeof(array[0]))

#define ROUND_DOWN(n, power2) ((n) & ~((power2) - 1))
#define ROUND_UP(n, power2) ROUND_DOWN(((n) + ((power2) - 1)), power2)

#define JOIN_MACRO(x, y) JOIN_TOKEN(x, y)
#define JOIN_TOKEN(x, y) x ## y

#if defined(__CC_ARM)
#define INLINE __inline
#else
#define INLINE inline
#endif

#endif /* COMMON_MACRO_H_INCLUDED */
/*
 * $Log: $
 */
