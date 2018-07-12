/****************************************************************************
 * modules/memutils/s_stl/c/include/buffer.h
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

#ifndef BUFFER_H
#define BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 *	RingBufferのデータ構造定義
 *******************************************************************************
 */
typedef struct {
	unsigned char	*buffer;	/* データ格納領域先頭		      */
	unsigned char	*head_p;	/* データ格納領域書込み可能先頭位置   */
	unsigned char	*tail_p;	/* データ格納領域書込み可能最後尾位置 */
	unsigned int	size;		/* データ型サイズ		      */
	unsigned int	depth;		/* データ格納数			      */
	unsigned int	cnt;		/* 現在登録データ数		      */
} RingBuffer;


/*******************************************************************************
 *	ライブラリプロトタイプ宣言
 *******************************************************************************
 */
extern RingBuffer *RingBuffer_new(unsigned int, unsigned int);
extern int RingBuffer_alloc_front(unsigned char *, RingBuffer *);
extern int RingBuffer_alloc_back(unsigned char *, RingBuffer *);
extern int RingBuffer_free_front(RingBuffer *);
extern int RingBuffer_free_back(RingBuffer *);
extern unsigned char *RingBuffer_get_front(RingBuffer *);
extern unsigned char *RingBuffer_get_back(RingBuffer *);
extern int RingBuffer_empty(RingBuffer *);
extern int RingBuffer_full(RingBuffer *);
extern void RingBuffer_clear(RingBuffer *);

#ifdef __cplusplus
}
#endif


#endif // BUFFER_H

