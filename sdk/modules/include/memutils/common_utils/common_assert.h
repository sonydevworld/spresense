/****************************************************************************
 * modules/include/memutils/common_utils/common_assert.h
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

#ifndef COMMON_ASSERT_H_INCLUDED
#define COMMON_ASSERT_H_INCLUDED

#include "memutils/common_utils/common_macro.h"
#include "assert.h"

/* static assert.
 * When exp evaluates to false, a compile error occurs.
 * Exp can describe only constant expressions
 * that can be evaluated at compile time.
 */

#define S_ASSERT(exp) \
  enum { JOIN_MACRO(AssertionFail_, __LINE__) = 1/(!!(exp)) }

/* debug assert.
 * NDEBUG Empty sentence when defining a macro,
 * so expressions with side effects on exp or func can not be described.
 */

#ifdef NDEBUG
#define D_ASSERT(exp)			((void)0)
#define D_ASSERT2(exp, func)		((void)0)
#else /* NDEBUG */
#define D_ASSERT(exp)			F_ASSERT(exp)
#define D_ASSERT2(exp, func)		F_ASSERT2((exp), (func))
#endif /* NDEBUG */

/* fatal assert.
 * Unlike D_ASSERT, it does not become invalid when NDEBUG.
 * Expressions with side effects can be described in exp or func.
 */

#ifdef ASSERT_USE_RETURN_ADDR

/* (void)((exp) || (_AssertionFail(#exp, __FILE__, __LINE__,
 *                  GET_RETURN_ADDR()), 0))
 */

#define F_ASSERT(exp) ASSERT(exp)

/* (void)((exp) || ((func), _AssertionFail(#exp, __FILE__,
 *                  __LINE__, GET_RETURN_ADDR()), 0))
 */

#define F_ASSERT2(exp, func) ASSERT(exp)
#else  /* ASSERT_USE_RETURN_ADDR */

/* (void)((exp) || (_AssertionFail(#exp, __FILE__, __LINE__), 0)) */

#define F_ASSERT(exp) ASSERT(exp)

/* (void)((exp) || ((func), _AssertionFail(#exp, __FILE__, __LINE__), 0)) */

#define F_ASSERT2(exp, func) ASSERT(exp)
#endif /* ASSERT_USE_RETURN_ADDR */

#ifdef  __cplusplus
extern "C" {
#endif

#if defined(_MSC_VER)
extern void * _ReturnAddress(void);
#pragma intrinsic(_ReturnAddress)
#define GET_RETURN_ADDR()	_ReturnAddress()
#elif defined(__CC_ARM)
#define GET_RETURN_ADDR()	(void*)__return_address()
#else

/* For most gcc other than for x86, you can only specify 0 as an argument.
 * (0 for gcc 3.3.6 + allegro-2.2.3)
 */

#define GET_RETURN_ADDR() __builtin_return_address(0)
#endif

#ifdef  __cplusplus
} /* end of extern "C" */
#endif

#endif	/* COMMON_ASSERT_H_INCLUDED */
