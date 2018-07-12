/****************************************************************************
 * modules/include/memutils/memory_manager/RuntimeQue.h
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

#ifndef RUNTIME_QUE_H_INCLUDED
#define RUNTIME_QUE_H_INCLUDED

#include <new>			/* operator new */
#include <stdio.h>		/* printf */
#include "memutils/common_utils/common_assert.h"	/* S_ASSERT, D_ASSERT */
#include "memutils/os_utils/cpp_util.h"		/* CopyGuard */

#ifdef RUNTIME_QUE_DEBUG
	/* 64bit環境では、size_t型の表示に%zuを使用する必要がある */
#define DBG_P(...)	printf(__VA_ARGS__)
#else
#define DBG_P(...)
#endif /* RUNTIME_QUE_DEBUG */

/*****************************************************************
 * 実行時にキューの深さを決定する行列クラス
 *****************************************************************/
template<typename T,			/* element type */
	 typename NumT  = size_t>	/* element num type */
class RuntimeQue : CopyGuard {
	void	post_push() { ++m_count; if (++m_put == capacity()) m_put = 0; }
	void	post_pop()  { --m_count; if (++m_get == capacity()) m_get = 0; }

	NumT get_index(NumT n) const {
		return static_cast<NumT>((m_get + n < capacity()) ? m_get + n : m_get + n - capacity());
	}

public:
	RuntimeQue(void* area, NumT depth) :
		m_data(static_cast<T*>(area)),
		m_capacity(depth), 
		m_put(0), m_get(0), m_count(0) {
//		D_ASSERT(que_area && depth);
	}
	~RuntimeQue() { clear(); }

	const void* que_area() const { return m_data; }
	size_t	elem_size() const { return sizeof(T); }
	NumT	capacity() const { return m_capacity; }
	NumT	size() const { return m_count; }
	NumT	rest() const { return static_cast<NumT>(capacity() - size()); }
	bool	empty() const { return size() == 0; }
	bool	full() const { return size() == capacity(); }

	void	dump() const {
		printf("dump: put=%u, get=%u, capa=%u, size=%u, rest=%u, empty=%d, full=%d\n",
			m_put, m_get, capacity(), size(), rest(), empty(), full());
		for (NumT i = 0; i < size(); ++i) {
			printf("%u: data=%08x\n",
				i, *reinterpret_cast<const int*>(&m_data[get_index(i)]));
		}
	}

	void	clear() {
		while (pop()) ;
		m_put = m_get = 0;
	}

	/* キュー末尾にデータを入れる(placement newによるコピーコンストラクタ使用) */
	bool push(const T& data) {
		if (full()) return false;
		::new(&m_data[m_put]) T(data);
		DBG_P("push: pos=%u, cnt=%u\n", m_put, m_count);
		post_push();
		return true;
	}

	/* キュー先頭のデータをデストラクタ呼出し後に取り除く */
	bool pop() {
		if (empty()) return false;
		DBG_P("pop : pos=%u, cnt=%u\n", m_get, m_count);
		m_data[m_get].~T();
		post_pop();
		return true;
	}

	/* キュー先頭のデータを参照する */
	const T& top() const { return at(0); }

	/* キューのN番目(先頭が0)のデータを参照する */
	const T& at(NumT n) const {
		D_ASSERT(n < m_count);	/* 例外は使えないので、assertとする */
		DBG_P("at(%u): pos=%u\n", n, m_get);
		return m_data[get_index(n)];
	}

	T& writable_at(NumT n) { return const_cast<T&>(at(n)); }

	const T& front() const { return at(0); }
	T& front() { return writable_at(0); }

	const T& back() const { return at(static_cast<NumT>(m_count - 1)); }
	T& back() { return writable_at(static_cast<NumT>(m_count - 1)); }

private:
	T* 	m_data;		/* data area */
	NumT	m_capacity;	/* queue capacity (depth) */
	NumT	m_put;		/* data put position. 0 origin */
	NumT	m_get;		/* data get position. 0 origin */
	NumT	m_count;	/* data count in queue */
}; /* class RuntimeQue */

#undef DBG_P

#endif /* RUNTIME_QUE_H_INCLUDED */
