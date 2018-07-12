/****************************************************************************
 * modules/include/memutils/s_stl/queue.h
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

#ifndef QUEUE_H_INCLUDED
#define QUEUE_H_INCLUDED

#include "memutils/s_stl/s_stl_config.h"
#include "memutils/s_stl/buffer.h"

__STL_BEGIN_NAMESPACE

/*-------------------------------------------------------------------*/
template<class T, int N> class Queue{

private:

	RingBuffer<T, N> data_buf;
	int cnt;
public:
	Queue() : cnt(0){}

	bool push(const T& new_data){
		if(cnt>=N){ return FALSE; } else { cnt++; }
		return data_buf.alloc_front(new_data);
	}

	bool pop(void){
		if(cnt<=0){ return FALSE; } else { cnt--; }
		return data_buf.free_back();
	}

	const T& top(void) const {
		return data_buf.get_back();
	}

	T& writable_at(const int n){
		return const_cast<T&>(data_buf.at_back(n));
	}

	const T& at(const int n) const {
		return data_buf.at_back(n);
	}

	int size(void) const { return cnt; }
	int rest(void) const { return (N-cnt); }

	bool empty(void) const { return (cnt==0); }
	bool full(void) const { return (cnt==N); }

	void clear(void){
		cnt = 0;
		data_buf.clear();
	}
};

__STL_END_NAMESPACE

#endif /* QUEUE_H_INCLUDED */

