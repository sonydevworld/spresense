/****************************************************************************
 * modules/memutils/s_stl/c/src/stack.c
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
#include "stack.h"

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 *	Queueのオブジェクト生成
 *******************************************************************************
 */
Stack *Stack_new(unsigned int size, unsigned int depth)
{
	return (Stack *)RingBuffer_new(size, depth);
}


/*******************************************************************************
 *	Stackのpush処理
 *******************************************************************************
 */
int Stack_push(unsigned char *new_data, Stack *this)
{
	return RingBuffer_alloc_front(new_data, &this->data_buf);
}


/*******************************************************************************
 *	Stackのpop処理
 *******************************************************************************
 */
int Stack_pop(Stack *this)
{
	return RingBuffer_free_front(&this->data_buf);
}


/*******************************************************************************
 *	Stackの先頭データ参照
 *******************************************************************************
 */
unsigned char *Stack_top(Stack *this)
{
	return RingBuffer_get_front(&this->data_buf);
}


/*******************************************************************************
 *	Stackのデータ有無確認
 *******************************************************************************
 */
int Stack_empty(Stack *this)
{
	return RingBuffer_empty(&this->data_buf);
}


/*******************************************************************************
 *	Stackのデータfull確認
 *******************************************************************************
 */
int Stack_full(Stack *this)
{
	return RingBuffer_full(&this->data_buf);
}


/*******************************************************************************
 *	Stackのデータクリア
 *******************************************************************************
 */
void Stack_clear(Stack *this)
{
	RingBuffer_clear(&this->data_buf);
}


#ifdef __cplusplus
}
#endif

