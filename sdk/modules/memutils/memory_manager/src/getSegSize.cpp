/****************************************************************************
 * modules/memutils/memory_manager/src/getSegSize.cpp
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

#include "memutils/memory_manager/MemHandleBase.h"
#include "BasicPool.h"

namespace MemMgrLite {

/*****************************************************************
 * メモリセグメントのアドレスを取得する
 *****************************************************************/
PoolSize Manager::getSegSize(const MemHandleBase& mh)
{
	MemPool* pool = findPool(mh.getPoolId());

#ifdef USE_MEMMGR_RINGBUF_POOL
	/* 仮想関数を使用しない方針なので、該当プール型にダウンキャストする */
	PoolSize size = 0;
	switch (pool->getPoolType()) {
	case BasicType:
		size = static_cast<BasicPool*>(pool)->getSegSize();
		break;
	case RingBufType:
		size = static_cast<RingBufPool*>(pool)->getSegSize();
	default:
		D_ASSERT(false);
		break;
	}
	return size;
#else
	/* BasicPoolのみ使用時は、各種チェックを省略する */
	return static_cast<BasicPool*>(pool)->getSegSize();
#endif
}

} /* end of namespace MemMgrLite */

/* getSegSize.cxx */
