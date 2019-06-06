/****************************************************************************
 * modules/memutils/memory_manager/src/getUsedSegs.cpp
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

namespace MemMgrLite {

/********************************************************************************
 * 静的プール群の使用中のセグメントをメモリハンドルに格納し、格納した数を返す
 ********************************************************************************/
uint32_t Manager::getStaticPoolsUsedSegs(uint8_t sec, MemHandleBase* mhs, uint32_t num_mhs)
{
	D_ASSERT(mhs && num_mhs);
	D_ASSERT(isStaticPoolAvailable(sec));

	uint32_t n = 0;

	/* プールID=0は予約 */
	for (uint8_t id = 1; id < theManager->m_pool_num[sec]; ++id) {
		MemPool* pool = theManager->m_static_pools[sec][id];
		if (pool) {
			n += pool->getUsedSegs(mhs + n, num_mhs - n);
			if (n == num_mhs) break;
		}
	}
	return n;
}

/********************************************************************************
 * メモリプールの使用中のセグメントをメモリハンドルに格納し、格納した数を返す
 ********************************************************************************/
uint32_t MemPool::getUsedSegs(MemHandleBase* mhs, uint32_t num_mhs)
{
	D_ASSERT(mhs && num_mhs);

#if 0 /* 効率化のため、生成直後の空のハンドル群を渡すよう仕様書に記載済み */
	for (uint32_t i = 0; i < num_mhs; ++i) {
		mhs[i].freeSeg();
	}
#endif

	uint32_t n = 0;
	for (uint32_t i = 0; i < getPoolNumSegs(); ++i) {
		/* 使用中のセグメントならば、メモリハンドルに結びつける */
		if (m_ref_cnt_array[i] != 0) {
			incSegRefCnt(i + 1);	/* セグメント番号は、1 origin */
			mhs[n].m_proxy = MemHandleBase::makeMemHandleProxy(getPoolId(), i + 1, 0);
			if (++n == num_mhs) break;
		}
	}
	return n;
}

} /* end of namespace MemMgrLite */

/* getUsedSegs.cxx */
