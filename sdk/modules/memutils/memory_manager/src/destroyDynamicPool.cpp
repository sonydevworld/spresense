/****************************************************************************
 * modules/memutils/memory_manager/src/destroyDynamicPool.cpp
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

#include "ScopedLock.h"
#include "memutils/memory_manager/Manager.h"

#ifdef USE_MEMMGR_DYNAMIC_POOL

namespace MemMgrLite {

/*****************************************************************
 * 動的メモリプールの破棄(削除)
 *****************************************************************/
void Manager::destroyDynamicPool(PoolId id)
{
#ifdef USE_MEMMGR_DEBUG_OUTPUT
	printf("Manager::destroyDynamicPool(PoolId=%d)\n", id);
#endif
	D_ASSERT(id >= theManager->m_pool_num && id < theManager->m_pool_num + NUM_DYN_POOLS);

	int no = id - theManager->m_pool_num;	/* プールIDから配列の番号に変換する */
	D_ASSERT(theManager->m_dynamic_pools[no]);	/* 使用中のはず */

	ScopedLock lock;

	destroyPool(theManager->m_dynamic_pools[no]);
	theManager->m_dynamic_pools[no] = NULL;

	/* プール番号を返却 */
	D_ASSERT(theManager->m_pool_no_que.full() == false);
	(void)theManager->m_pool_no_que.push(no);
}

} /* end of namespace MemMgrLite */

#endif /* USE_MEMMGR_DYNAMIC_POOL */

/* destroyDynamicPool.cxx */
