/****************************************************************************
 * modules/memutils/s_stl/include/m_stack.h
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

#ifndef M_STACK_H_INCLUDED
#define M_STACK_H_INCLUDED

#include "s_stl_config.h"
#include "buffer.h"

__USING_S_STL;

__M_STL_BEGIN_NAMESPACE

template<class T, int N> class Stack{

private:

	RingBuffer<T, N> data_buf;
	int cnt;
	ID  mtx,sem;

public:
	Stack() : cnt(0){
		CREATE_MTX(mtx,0);
		if(mtx < E_OK) _fatal;
		CREATE_SEM(sem,N);
		if(sem < E_OK){ _fatal; }
	}

	bool push(const T& new_data,TMO tim_out){
		ER err = TAKE_SEM(sem,tim_out);
		if(err == E_TMOUT){	return FALSE; }
		if(err != E_OK){ _fatal; }
		if(LOCK_MTX(mtx)){ _fatal; }
		if(cnt>=N){
			if(GIVE_MTX(mtx)){ _fatal; }
			return FALSE;
		}
		cnt++;
		bool rst=data_buf.alloc_front(new_data);
		if(GIVE_MTX(mtx)){ _fatal; }
		return rst;
	}

	bool pop(void){
		if(LOCK_MTX(mtx)){ _fatal; }
		if(cnt<=0){
			if(GIVE_MTX(mtx)){ _fatal; }
			return FALSE;
		} else {cnt--;}
		bool rst=data_buf.free_front();
		if(GIVE_MTX(mtx)){ _fatal; }
		if(GIVE_SEM(sem)){ _fatal; }
		return rst;
	}

	bool tpop(T* data){
		if(LOCK_MTX(mtx)){ _fatal; }
		if(cnt<=0){
			if(GIVE_MTX(mtx)){ _fatal; }
			return FALSE;
		} else { cnt--; }
		*data = top();
		bool rst = data_buf.free_front();
		if(GIVE_MTX(mtx)){ _fatal; }
		if(GIVE_SEM(sem)){ _fatal; }
		return rst;
	}

	const T& top(void) const {
		return data_buf.get_front();
	}

	bool empty(void) const { return (cnt==0); }
	bool full(void) const { return (cnt==N); }

	void clear(void){
		cnt=0;
		data_buf.clear();
	}
};
		
__M_STL_END_NAMESPACE

#endif // M_STACK_H_INCLUDED

