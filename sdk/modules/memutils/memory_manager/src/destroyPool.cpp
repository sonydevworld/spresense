/****************************************************************************
 * modules/memutils/memory_manager/src/destroyPool.cpp
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

#include "memutils/memory_manager/Manager.h"
#include "BasicPool.h"

namespace MemMgrLite {

/*****************************************************************
 * Destroy a memory pool object
 * 排他制御は呼び出し側で行うこと
 *****************************************************************/
void Manager::destroyPool(MemPool* pool)
{
#ifdef USE_MEMMGR_RINGBUF_POOL
	/* 仮想関数を使用しない方針なので、該当プール型にダウンキャストする */
	switch (pool->getPoolType()) {
	case BasicType:
		static_cast<BasicPool*>(pool)->~BasicPool();
		break;
	case RingBufType:
		static_cast<RingBufPool*>(pool)->~RingBufPool();
		break;
	default:
		D_ASSERT(false);	/* Unsupport pool type */
		break;
	}
#else
	/* BasicPoolのみ使用時は、各種チェックを省略する */
	static_cast<BasicPool*>(pool)->~BasicPool();
#endif
}

/*****************************************************************
 * Basicプールのデストラクタ
 *****************************************************************/
BasicPool::~BasicPool()
{
#ifdef USE_MEMMGR_DEBUG_OUTPUT
	printf("~BasicPool: PoolId=%d\n", getPoolId());
#endif
}

/*****************************************************************
 * メモリプールのデストラクタ
 *****************************************************************/
MemPool::~MemPool()
{
#ifdef CONFIG_MEMUTILS_MEMORY_MANAGER_USE_FENCE
	if (verifyPoolFence() != 0) {
		F_ASSERT(0);	/* pool fence verification error */
	}
#endif

	if (!m_seg_no_que.full()) {
#ifdef USE_MEMMGR_DEBUG_OUTPUT
		printf("~MemPool: Segment leak found. PoolId=%d\n", getPoolId());
		for (int i = 0; i < getPoolNumSegs(); ++i) {
			if (m_ref_cnt_array[i] != 0) {
				printf(" SegNo=%d: RefCnt=%d\n", i + 1, m_ref_cnt_array[i]);
			}
		}
#endif
		F_ASSERT(0);	/* memory segment leaked */
	}
}

} /* end of namespace MemMgrLite */

/* destroyPool.cxx */
