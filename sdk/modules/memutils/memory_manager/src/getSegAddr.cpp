/****************************************************************************
 * modules/memutils/memory_manager/src/getSegAddr.cpp
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
PoolAddr Manager::getSegAddr(const MemHandleBase& mh)
{
	MemPool* pool = findPool(mh.getPoolId());

#ifdef USE_MEMMGR_RINGBUF_POOL
	/* 仮想関数を使用しない方針なので、該当プール型にダウンキャストする */
	switch (pool->getPoolType()) {
	case BasicType:
		return static_cast<BasicPool*>(pool)->getSegAddr(mh);
	case RingBufType:
		return static_cast<RingBufPool*>(pool)->getSegAddr(mh);
	default:
		D_ASSERT(false);
		return BadPoolAddr;
	}
#else
	/* BasicPoolのみ使用時は、各種チェックを省略する */
	return static_cast<BasicPool*>(pool)->getSegAddr(mh);
#endif
}

/*****************************************************************
 * Basicプールのセグメントのアドレスを取得する
 *****************************************************************/
PoolAddr BasicPool::getSegAddr(const MemHandleBase& mh) const
{
	NumSeg seg_no = mh.getSegNo();
	D_ASSERT(seg_no != NullSegNo && seg_no <= getPoolNumSegs());

	return getPoolAddr() + ((seg_no - 1) * getSegSize());
}

} /* end of namespace MemMgrLite */

/* getSegAddr.cxx */
