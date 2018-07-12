/****************************************************************************
 * modules/memutils/s_stl/c/include/deque.h
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

#ifndef DEQUE_H
#define DEQUE_H

#include "buffer.h"

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 *	Dequeのデータ構造定義
 *******************************************************************************
 */
typedef struct {
	RingBuffer	data_buf;	/* RingBufferでDeque機能を実現	      */
//	unsigned int	*cnt;		/* 現在登録データ数		      */
} Deque;


/*******************************************************************************
 *	ライブラリプロトタイプ宣言
 *******************************************************************************
 */
extern Deque *Deque_new(unsigned int, unsigned int);
extern int Deque_push_front(unsigned char *, Deque *);
extern int Deque_push_back(unsigned char *, Deque *);
extern int Deque_pop_front(Deque *);
extern int Deque_pop_back(Deque *);
extern unsigned char *Deque_front(Deque *);
extern unsigned char *Deque_back(Deque *);
extern int Deque_empty(Deque *);
extern int Deque_full(Deque *);
extern void Deque_clear(Deque *);

#ifdef __cplusplus
}
#endif


#endif // DEQUE_H

