/****************************************************************************
 * modules/memutils/s_stl/c/src/deque.c
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

#include "s_stl_config.h"
#include "buffer.h"
#include "deque.h"

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 *	Dequeのオブジェクト生成
 *******************************************************************************
 */
Deque *Deque_new(unsigned int size, unsigned int depth)
{
	return (Deque *)RingBuffer_new(size, depth);
}


/*******************************************************************************
 *	Dequeのフロントへのpush処理
 *******************************************************************************
 */
int Deque_push_front(unsigned char *new_data, Deque *this)
{
	return RingBuffer_alloc_front(new_data, &this->data_buf);
}


/*******************************************************************************
 *	Dequeのバックへのpush処理
 *******************************************************************************
 */
int Deque_push_back(unsigned char *new_data, Deque *this)
{
	return RingBuffer_alloc_back(new_data, &this->data_buf);
}


/*******************************************************************************
 *	Dequeのフロントデータのpop処理
 *******************************************************************************
 */
int Deque_pop_front(Deque *this)
{
	return RingBuffer_free_front(&this->data_buf);
}


/*******************************************************************************
 *	Dequeのバックデータのpop処理
 *******************************************************************************
 */
int Deque_pop_back(Deque *this)
{
	return RingBuffer_free_back(&this->data_buf);
}


/*******************************************************************************
 *	Dequeのフロントデータの参照
 *******************************************************************************
 */
unsigned char *Deque_front(Deque *this)
{
	return RingBuffer_get_front(&this->data_buf);
}


/*******************************************************************************
 *	Dequeのバックデータの参照
 *******************************************************************************
 */
unsigned char *Deque_back(Deque *this)
{
	return RingBuffer_get_back(&this->data_buf);
}


/*******************************************************************************
 *	Dequeのデータ有無確認
 *******************************************************************************
 */
int Deque_empty(Deque *this)
{
	return RingBuffer_empty(&this->data_buf);
}


/*******************************************************************************
 *	Dequeのデータfull確認
 *******************************************************************************
 */
int Deque_full(Deque *this)
{
	return RingBuffer_full(&this->data_buf);
}


/*******************************************************************************
 *	Dequeのデータクリア
 *******************************************************************************
 */
void Deque_clear(Deque *this)
{
	RingBuffer_clear(&this->data_buf);
}


#ifdef __cplusplus
}
#endif

