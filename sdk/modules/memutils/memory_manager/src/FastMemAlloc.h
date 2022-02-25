/****************************************************************************
 * modules/memutils/memory_manager/src/FastMemAlloc.h
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

#ifndef FAST_MEM_ALLOC_H_INCLUDED
#define FAST_MEM_ALLOC_H_INCLUDED

#include <new>			/* operator new */
#include <stdio.h>		/* printf */
#include "memutils/common_utils/common_macro.h"	/* MEMUTILS_ROUND_UP */
#include "memutils/common_utils/common_assert.h"	/* D_ASSERT */
#include "memutils/os_utils/cpp_util.h"		/* CopyGuard */

////////////////////////////////////////////////////////////

class FastMemAlloc : CopyGuard {
	size_t	m_init_addr;
	size_t	m_init_size;
	size_t	m_alloc_size;
public:
	FastMemAlloc() : m_init_addr(0), m_init_size(0), m_alloc_size(0) {}
	FastMemAlloc(void* adr, size_t sz) :
		m_init_addr(reinterpret_cast<size_t>(adr)), m_init_size(sz), m_alloc_size(0) {}
	~FastMemAlloc() {}

	void set(void* adr, size_t sz)
	{ m_init_addr = reinterpret_cast<size_t>(adr); m_init_size = sz; m_alloc_size = 0; }

	void reset() { m_alloc_size = 0; }

	void*	addr() const { return reinterpret_cast<void*>(m_init_addr); }
	size_t	size() const { return m_init_size; }
	size_t	rest() const { return m_init_size - m_alloc_size; }

	/* fastest and unaligned version */
	void* alloc(size_t sz) {
		if (sz > rest()) return 0;
		void* adr = reinterpret_cast<void*>(m_init_addr + m_alloc_size);
		m_alloc_size += sz;
		return adr;
	}

	/* aligned version */
	void* alloc(size_t sz, size_t align) {
		D_ASSERT(align != 0);
		size_t adr  = m_init_addr + m_alloc_size;
		size_t skip = MEMUTILS_ROUND_UP(adr, align) - adr;
		if (sz + skip > rest()) return 0;
		m_alloc_size += sz + skip;
		return reinterpret_cast<void*>(adr + skip);
	}

	/* release from tail */
	void release_size(size_t sz) { D_ASSERT(sz <= m_alloc_size); m_alloc_size -= sz; }
	void release_addr(void* adr) {
		release_size(m_alloc_size - (reinterpret_cast<size_t>(adr) - m_init_addr));
	}

	void dump() const {
		printf("init_addr=%08x, init_size=%08x, alloc_size=%08x\n",
			m_init_addr, m_init_size, m_alloc_size);
	}
}; /* class FastMemAlloc */

/* fastest and unaligned version */
inline void* operator new(size_t sz, FastMemAlloc& obj) throw()
{
	if (sz == 0) sz = 1;	/* convention of operator new */
	return obj.alloc(sz);
}

inline void* operator new[](size_t sz, FastMemAlloc& obj) throw()
{
	return operator new(sz, obj);
}

/* aligned version */
inline void* operator new(size_t sz, FastMemAlloc& obj, size_t align) throw()
{
	if (sz == 0) sz = 1;	/* convention of operator new */
	return obj.alloc(sz, align);
}

inline void* operator new[](size_t sz, FastMemAlloc& obj, size_t align) throw()
{
	return operator new(sz, obj, align);
}

/* placement delete. NOP */
inline void operator delete(void*, FastMemAlloc&) throw() {}
inline void operator delete[](void*, FastMemAlloc&) throw() {}
inline void operator delete(void*, FastMemAlloc&, size_t) throw() {}
inline void operator delete[](void*, FastMemAlloc&, size_t) throw() {}

#endif /* FAST_MEM_ALLOC_H_INCLUDED */
