/****************************************************************************
 * modules/include/memutils/os_utils/cpp_util.h
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

#ifndef CPP_UTIL_H_INCLUDED
#define CPP_UTIL_H_INCLUDED
#ifdef __cplusplus

//#include "chateau_assert.h"

////////////////////////////////////////////////////////////
// Copy guard class
//
class CopyGuard {
protected:
	CopyGuard() {}
	~CopyGuard() {}
private:
	CopyGuard(const CopyGuard&);
	const CopyGuard& operator=(const CopyGuard&);
}; // class CopyGuard


////////////////////////////////////////////////////////////
// Range checkable array class
// 範囲チェックを行うには、-DARRAY_ASSERT=CHATEAU_FATAL_ASSERTオプションでビルドする
//
#ifndef ARRAY_ASSERT
#define ARRAY_ASSERT(exp)
#endif

template<typename T, size_t N>
class Array {
public:
	// 配列同様の初期化を可能にするため、publicとする
	T m_elems[N];
public:
	typedef T elem_type_t;

	static size_t capacity() { return N; }

	T& operator[](size_t index) {
		ARRAY_ASSERT(index < N);
		return m_elems[index];
	}

	const T& operator[](size_t index) const {
		ARRAY_ASSERT(index < N);
		return m_elems[index];
	}
}; // class Array

////////////////////////////////////////////////////////////
// Template functions
//
template<typename T>
inline const T& Min(const T& a, const T& b) { return (a < b) ? a : b; }

template<typename T>
inline const T& Max(const T& a, const T& b) { return (a > b) ? a : b; }

#endif /* __cplusplus */
#endif /* CPP_UTIL_H_INCLUDED */
/*
 * $Log: $
 */
