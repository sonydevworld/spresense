/****************************************************************************
 * modules/memutils/s_stl/c/src/buffer.c
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

#include <stdlib.h>	/* malloc, free */
#include "s_stl_config.h"
#include "buffer.h"

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 *	RingBufferのオブジェクト生成
 *******************************************************************************
 */
RingBuffer *RingBuffer_new(unsigned int size, unsigned int depth)
{
	RingBuffer	*new_data;

	/* オブジェクト領域(データ管理領域)を確保		*/
	new_data  = (RingBuffer *)malloc(sizeof(RingBuffer));
	D_ASSERT(new_data);
	if (new_data) {
		/* 初期値設定		*/
		new_data->size   = size;
		new_data->depth  = depth;
		new_data->cnt    = 0;

		/* データ領域を確保 */
		new_data->buffer = (unsigned char *)malloc(size * depth);
		D_ASSERT(new_data->buffer);
		if (new_data->buffer==NULL) {
			free(new_data);
			return NULL;
		}
		new_data->head_p = new_data->buffer;
		new_data->tail_p = new_data->buffer;
	}
	return new_data;
}


/*******************************************************************************
 *	RingBufferのフロントへのデータ格納
 *******************************************************************************
 */
int RingBuffer_alloc_front(unsigned char *new_data, RingBuffer *this)
{
	unsigned int	i;
	unsigned char	*buffer_start, *buffer_end;

	if(this->cnt >= this->depth){ return FALSE; } else { this->cnt++; }
	if(this->cnt != 1){
		buffer_start = this->buffer;
		buffer_end = &this->buffer[this->size * (this->depth - 1)];
		if((this->head_p + this->size) > buffer_end){
			/* データ設定位置を領域先頭に移動	*/
			this->head_p = buffer_start;
		}else{
			/* データ設定位置を先に移動		*/
			this->head_p += this->size;
		}
	}
	for(i=0;i<this->size;i++){
		this->head_p[i] = new_data[i];
	}
	return TRUE;
}


/*******************************************************************************
 *	RingBufferのバックへのデータ格納
 *******************************************************************************
 */
int RingBuffer_alloc_back(unsigned char *new_data, RingBuffer *this)
{
	unsigned int	i;
	unsigned char	*buffer_start, *buffer_end;

	if(this->cnt >= this->depth){ return FALSE; } else { this->cnt++; }
	if(this->cnt != 1){
		buffer_start = this->buffer;
		buffer_end = &this->buffer[this->size * (this->depth - 1)];
		if((this->tail_p - this->size) < buffer_start){
			/* データ設定位置を領域最後尾に移動	*/
			this->tail_p=buffer_end;
		}else{
			/* データ設定位置を後ろに移動		*/
			this->tail_p-=this->size;
		}
	}
	for(i=0;i<this->size;i++){
		this->tail_p[i] = new_data[i];
	}
	return TRUE;
}


/*******************************************************************************
 *	RingBufferのフロントデータのクリア
 *******************************************************************************
 */
int RingBuffer_free_front(RingBuffer *this)
{
	unsigned char	*buffer_start, *buffer_end;

	if(this->cnt <= 0){ return FALSE; } else { this->cnt--; }
	if(this->cnt != 0){
		buffer_start = this->buffer;
		buffer_end = &this->buffer[this->size * (this->depth - 1)];
		if((this->head_p - this->size) < buffer_start){
			/* データクリア位置を領域最後尾に移動	*/
			this->head_p=buffer_end;
		}else{
			/* データ設定位置を後ろに移動		*/
			this->head_p-=this->size;
		}
	}
	return TRUE;
}


/*******************************************************************************
 *	RingBufferのバックデータのクリア
 *******************************************************************************
 */
int RingBuffer_free_back(RingBuffer *this)
{
	unsigned char	*buffer_start, *buffer_end;

	if(this->cnt <= 0){ return FALSE; } else { this->cnt--; }
	if(this->cnt != 0){
		buffer_start = this->buffer;
		buffer_end = &this->buffer[this->size * (this->depth - 1)];
		if((this->tail_p + this->size) > buffer_end){
			/* データクリア位置を領域先頭に移動	*/
			this->tail_p = buffer_start;
		}else{
			/* データクリア位置を後ろに移動		*/
			this->tail_p += this->size;
		}
	}
	return TRUE;
}


/*******************************************************************************
 *	RingBufferのフロントデータの参照
 *******************************************************************************
 */
unsigned char *RingBuffer_get_front(RingBuffer *this){ return this->head_p; }


/*******************************************************************************
 *	RingBufferのバックデータの参照
 *******************************************************************************
 */
unsigned char *RingBuffer_get_back(RingBuffer *this){ return this->tail_p; }


/*******************************************************************************
 *	RingBufferのデータ有無確認
 *******************************************************************************
 */
int RingBuffer_empty(RingBuffer *this){ return (this->cnt==0); }


/*******************************************************************************
 *	RingBufferのデータデータfull確認
 *******************************************************************************
 */
int RingBuffer_full(RingBuffer *this){ return (this->cnt==this->depth); }


/*******************************************************************************
 *	RingBufferのデータクリア
 *******************************************************************************
 */
void RingBuffer_clear(RingBuffer *this){
	while(this->cnt > 0){
		RingBuffer_free_back(this);
	}
}

#ifdef __cplusplus
}
#endif


