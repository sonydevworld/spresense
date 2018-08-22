/****************************************************************************
 * modules/include/memutils/message/BasicQueue.h
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

#ifndef BASIC_QUEUE_H_INCLUDED
#define BASIC_QUEUE_H_INCLUDED

#include <new>			/* placement new */
#include <stdio.h>		/* printf */
#include <string.h>		/* memcpy, memset */
#include "memutils/common_utils/common_types.h"	/* uint8_t, drm_t, INVALID_DRM */
#include "memutils/common_utils/common_assert.h"	/* D_ASSERT, F_ASSERT */
#include "memutils/os_utils/cpp_util.h"		/* CopyGuard */
#include "memutils/message/AssertInfo.h"		/* SizeErrorLog */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DRM_TO_CACHED_VA(drm) (void*)(drm)

/*****************************************************************
 * Type characteristic
 *****************************************************************/
template<typename> struct BasicQueueAddrTraits;

template<> struct BasicQueueAddrTraits<void*> {
  /* In static const, initialization of non-integer (pointer)
   * can not be performed, so it is set as enum.
   */

	enum { init_value = 0 };
};

template<> struct BasicQueueAddrTraits<drm_t> {
	enum { init_value = INVALID_DRM };
};

/*****************************************************************
 * Basic matrix class
 *****************************************************************/
template<typename AddrT,		/* queue address type. drm_t or void* */
	 typename SizeT,		/* element size type. uintN_t */
	 typename NumT,			/* element num type. uintN_t */
   uint8_t FillAfterPop = 0x00> /* The value to fill after pop.
                                 * When it is 0, no filling is done.
                                 */
class BasicQueue : CopyGuard {
public:
	BasicQueue() :
		m_data_area(AddrT(BasicQueueAddrTraits<AddrT>::init_value)),
		m_elem_size(0), m_capacity(0),
		m_put(0), m_get(0), m_count(0) {}

	BasicQueue(AddrT data_area, SizeT elem, NumT num) :
		m_data_area(data_area), m_elem_size(elem), m_capacity(num),
		m_put(0), m_get(0), m_count(0) {
		D_ASSERT(data_area != AddrT(BasicQueueAddrTraits<AddrT>::init_value) && elem && num);
	}

	~BasicQueue() { D_ASSERT(empty()); }

  /* Initialization by this function is required for the instance created
   * by default constructor.
   */

	void init(AddrT data_area, SizeT elem, NumT num) {
		D_ASSERT(data_area != AddrT(BasicQueueAddrTraits<AddrT>::init_value) && elem && num);
		D_ASSERT(is_init() == false); /* Reinitialization prohibited. */
		m_data_area = data_area;
		m_elem_size = elem;
		m_capacity = num;
	}

	bool	is_init() const { return m_data_area != AddrT(BasicQueueAddrTraits<AddrT>::init_value); }
	SizeT	elem_size() const { return m_elem_size; }
	NumT	capacity() const { return m_capacity; }
	NumT	size() const { return m_count; }
	NumT	rest() const { return static_cast<NumT>(capacity() - size()); }
	bool	empty() const { return size() == 0; }
	bool	full() const { return size() == capacity(); }

	void dump() const {
		printf("dump: put=%u, get=%u, capa=%u, elem=%u, size=%u, rest=%u, empty=%d, full=%d\n",
			m_put, m_get, capacity(), elem_size(), size(), rest(), empty(), full());
		printf("      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
		for (NumT i = 0; i < capacity(); ++i) {
			printf("% 3u: ", i);
			for (NumT j = 0; j < MIN(elem_size(), 16); ++j) {
				printf("%02x ", static_cast<uint8_t*>(getAddr(i))[j]);
			}
			printf("\n");
		}
	}

	void dump_active() const {
		printf("dump: size=%u, rest=%u\n", size(), rest());
		printf("      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
		for (NumT i = 0; i < size(); ++i) {
			printf("% 3u: ", i);
			for (NumT j = 0; j < MIN(elem_size(), 16); ++j) {
				printf("%02x ", static_cast<uint8_t*>(getAddr(getIndex(i)))[j]);
			}
			printf("\n");
		}
	}

  /* Clear the queue without destructor invocation. */

	void clear() {
		while (pop()) ;
		m_put = m_get = 0;
	}

  /* Inserting data at the end of the queue (using memcpy). */

	bool push(const void* data, SizeT len) {
		D_ASSERT2(len <= elem_size(), AssertParamLog(AssertIdSizeError, len, elem_size()));
		if (full()) return false;
		memcpy(getAddr(m_put), data, len);
		postPush();
		return true;
	}

  /* Inserting data at the end of the queue (using copy constructor). */

	template<typename T>
	bool push(const T& data) {
		D_ASSERT2(sizeof(T) <= elem_size(), AssertParamLog(AssertIdSizeError, sizeof(T), elem_size()));
		if (full()) return false;
		::new(getAddr(m_put)) T(data);	/* call copy constructor */
		postPush();
		return true;
	}

  /* Remove queue head data without destructor call. */

	bool pop() {
		if (empty()) return false;
		postPop();
		return true;
	}

  /* Remove the data at the head of the queue after calling the destructor. */

	template<typename T>
	bool pop() {
		D_ASSERT2(sizeof(T) <= elem_size(), AssertParamLog(AssertIdSizeError, sizeof(T), elem_size()));
		if (empty()) return false;
		static_cast<T*>(getAddr(m_get))->~T();	/* call destructor */
		postPop();
		return true;
	}

  /* Refer to the data at the head of the queue. */

	template<typename T>
	const T& top() const { return at<T>(0); }

  /* Refer to the data of the Nth (head data is 0) of the queue */

	template<typename T>
	const T& at(NumT n) const {
		D_ASSERT2(sizeof(T) <= elem_size(), AssertParamLog(AssertIdSizeError, sizeof(T), elem_size()));
		D_ASSERT2(n < m_count, AssertParamLog(AssertIdBadParam, n, m_count));
		return *static_cast<T*>(getAddr(getIndex(n)));
	}

	template<typename T>
	T& writable_at(NumT n) { return const_cast<T&>(at<T>(n)); }

  /* Refer to the data at the head of the queue. */

	template<typename T>
	const T& front() const { return at<T>(0); }

	template<typename T>
	T& front() { return writable_at<T>(0); }

  /* Reference data at the end of the queue. */

	template<typename T>
	const T& back() const { return at<T>(static_cast<NumT>(m_count - 1)); }

	template<typename T>
	T& back() { return writable_at<T>(static_cast<NumT>(m_count - 1)); }

protected:
	void postPush() {
		++m_count;
		if (++m_put == capacity()) {
			m_put = 0;
		}
	}

	void postPop()  {
		if (FillAfterPop) {
			memset(getAddr(m_get), FillAfterPop, elem_size());
		}
		--m_count;
		if (++m_get == capacity()) {
			m_get = 0;
		}
	}

	NumT getIndex(NumT n) const {
		return static_cast<NumT>((m_get + n < capacity()) ? m_get + n : m_get + n - capacity());
	}

  /* Call separation by tag and dispatch. */

	void* getAddr(NumT n) const { return getAddr(n, AddrT()); }

	void* getAddr(NumT n, drm_t /* dummy */) const {
		return static_cast<uint8_t*>(DRM_TO_CACHED_VA(m_data_area)) + n * elem_size();
	}

	void* getAddr(NumT n, void* /* dummy */) const {
		return static_cast<uint8_t*>(m_data_area) + n * elem_size();
	}

private:
	AddrT	m_data_area;	/* point to data area */
	SizeT	m_elem_size;	/* element size */
	NumT	m_capacity;	/* number of elements */
	NumT	m_put;		/* data put position. 0 origin */
	NumT	m_get;		/* data get position. 0 origin */
	NumT	m_count;	/* data count in queue */
}; /* class BasicQueue */


#ifdef TEST_BASIC_QUEUE
//typedef BasicQueue<void*, uint8_t, uint8_t, 0x55> TestQue;
//typedef BasicQueue<void*, uint16_t, uint8_t, 0x55> TestQue;
//typedef BasicQueue<void*, uint8_t, uint16_t, 0x55> TestQue;
typedef BasicQueue<void*, uint16_t, uint16_t, 0x55> TestQue;
//typedef BasicQueue<void*, uint32_t, uint32_t, 0x55> TestQue;

static void testQueOperation(TestQue& que)
{
	for (uint32_t i = 0; i <= que.capacity(); ++i) {
		que.dump_active();

		F_ASSERT(que.size() == i);
		F_ASSERT(que.rest() == que.capacity() - i);
		F_ASSERT(que.empty() == (i == 0));
		F_ASSERT(que.full() == (i == que.capacity()));

		uint32_t n = 'a' + i;
		if (i % 2 == 0) {
			F_ASSERT(que.push(n) == (i < que.capacity()));
		} else {
			F_ASSERT(que.push(&n, sizeof(n)) == (i < que.capacity()));
		}
		F_ASSERT(que.top<uint32_t>() == 'a');
		F_ASSERT(que.front<uint32_t>() == 'a');
		F_ASSERT(que.back<uint32_t>() == static_cast<uint32_t>('a' + MIN(i, static_cast<uint32_t>(que.capacity() - 1))));
	}
	que.dump();

	for (uint32_t i = 0; i <= que.capacity(); ++i) {
		F_ASSERT(que.size() == que.capacity() - i);
		F_ASSERT(que.rest() == i);
		F_ASSERT(que.empty() == (i == que.capacity()));
		F_ASSERT(que.full() == (i == 0));

		if (i % 2 == 0) {
			F_ASSERT(que.pop() == (i < que.capacity()));
		} else {
			F_ASSERT(que.pop<uint32_t>() == (i < que.capacity()));
		}
		if (i < static_cast<uint32_t>(que.capacity() - 1)) {
			F_ASSERT(que.top<uint32_t>() == 'b' + i);
			F_ASSERT(que.front<uint32_t>() == 'b' + i);
			F_ASSERT(que.back<uint32_t>() == static_cast<uint32_t>('a' + que.capacity() - 1));
		}
		que.dump_active();
	}
	que.dump();
}

void testBasicQueue()
{
	printf("Start TestBasicQueue()\n");

	const uint32_t QueElemSize = 64;
	const uint32_t QueElemNum = 16;
	static uint8_t s_queue_data[QueElemSize * QueElemNum];

	{
		TestQue que;
		F_ASSERT(que.is_init() == false);
		que.init(s_queue_data, QueElemSize, QueElemNum);
		F_ASSERT(que.is_init());
		F_ASSERT(que.elem_size() == QueElemSize);
		F_ASSERT(que.capacity() == QueElemNum);
		testQueOperation(que);
		que.clear();
	}
	{
		TestQue que(s_queue_data, QueElemSize, QueElemNum);
		F_ASSERT(que.is_init());
		F_ASSERT(que.elem_size() == QueElemSize);
		F_ASSERT(que.capacity() == QueElemNum);
		testQueOperation(que);
		que.clear();
	}

	printf("End TestBasicQueue()\n");
}
#endif /* TEST_BASIC_QUEUE */

#endif /* BASIC_QUEUE_H_INCLUDED */
