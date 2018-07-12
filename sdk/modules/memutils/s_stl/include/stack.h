/****************************************************************************
 * modules/memutils/s_stl/include/stack.h
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

#ifndef STACK_H_INCLUDED
#define STACK_H_INCLUDED

#include "s_stl_config.h"
#include "buffer.h"

__STL_BEGIN_NAMESPACE

template<class T, int N> class Stack{

private:

	RingBuffer<T, N> data_buf;
	int cnt;
public:
	Stack() : cnt(0){}

	bool push(const T& new_data){
		if(cnt>=N){ return FALSE; } else {cnt++;}
		data_buf.alloc_front(new_data);
		return TRUE;
	}

	bool pop(void){
		if(cnt<=0){ return FALSE; } else {cnt--;}
		return data_buf.free_front();
	}
	const T& top(void) const {
		return data_buf.get_front();
	}

	int size(void) const { return cnt; }
	int rest(void) const { return (N-cnt); }

	bool empty(void) const { return (cnt==0); }
	bool full(void) const { return (cnt==N); }

	void clear(void){
		cnt=0;
		data_buf.clear();
	}
};
		
__STL_END_NAMESPACE

#endif // STACK_H_INCLUDED

