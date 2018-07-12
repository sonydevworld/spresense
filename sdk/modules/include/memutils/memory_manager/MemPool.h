/****************************************************************************
 * modules/include/memutils/memory_manager/MemPool.h
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
#ifndef MEMPOOL_H_INCLUDED
#define MEMPOOL_H_INCLUDED

#include "memutils/memory_manager/RuntimeQue.h"
#include "memutils/memory_manager/MemMgrTypes.h"

/*
 * 以下の理由により仮想関数は使用禁止
 * ・テキスト非共有のマルチコアでは、仮想関数が使えない
 * ・仮想関数を使用するとインスタンス毎に4byteサイズが増える
 */

namespace MemMgrLite {

/*****************************************************************
 * Memory pool base class (16 or 20bytes)
 *****************************************************************/
class MemPool : CopyGuard {
	friend class Manager;
protected:
	MemPool(const PoolAttr& attr, FastMemAlloc& fma);
	~MemPool();

	/* 例外を使用できないので、コンストラクタのエラーはこの関数で調べる */
	bool isFailed() {
		if (m_seg_no_que.que_area() == NULL || m_ref_cnt_array == NULL) {
			return true;
		}
		return false;
	}

	PoolId		getPoolId()   const { return m_attr.id; }
	PoolType	getPoolType() const { return m_attr.type; }
	PoolAddr	getPoolAddr() const { return m_attr.addr; }
	PoolSize	getPoolSize() const { return m_attr.size; }
	NumSeg		getPoolNumSegs() const { return m_attr.num_segs; }
	NumSeg		getPoolNumAvailSegs() const { return m_seg_no_que.size(); }
#ifdef CONFIG_MEMUTILS_MEMORY_MANAGER_USE_FENCE
	bool		isPoolFenceEnable() const { return m_attr.fence; }
	void		initPoolFence();
	uint32_t	verifyPoolFence();
#endif
#ifdef USE_MEMMGR_MULTI_CORE
	LockId		getPoolLockId() const { return m_attr.spl_id; }
#endif

	/*
	 * セグメント参照カウンタ値の取得・更新
	 */
	SegRefCnt	getSegRefCnt(NumSeg seg_no) const {
		D_ASSERT(seg_no != NullSegNo && seg_no <= getPoolNumSegs());
		D_ASSERT(m_ref_cnt_array[seg_no - 1] != 0);	/* 使用中のはず */
		return m_ref_cnt_array[seg_no - 1];
	}
	void		incSegRefCnt(NumSeg seg_no);

	/*
	 * 使用中のメモリセグメントをメモリハンドルに格納し、格納した数を返す
	 */
	uint32_t	getUsedSegs(MemHandleBase* mhs, uint32_t num_mhs);

	/*
	 * メモリプールからセグメントを取得する
	 * 排他制御は呼出し側で行うこと
	 */
	MemHandleProxy	allocSeg();

	/*
	 * 参照カウンタを減算し、参照がなくなった場合はセグメントを返却する
	 * 排他制御は呼出し側で行うこと
	 */
	void	freeSeg(MemHandleBase& mh);

protected:
	/*
	 * 静的プールの場合は、MemoryPoolLayoutsの該当箇所を指し示す
	 * 動的プールやアドレス書き換え等が必要な場合は、別途PoolAttr格納場所を用意すること
	 */
	const PoolAttr&	m_attr;		/* pool attributes */

	/*
	 * 使用可能なセグメント番号(1 origin)を保持するキュー (8 or 12bytes)
	 * キューデータ用の領域は別途用意する必要がある
	 */
	RuntimeQue<NumSeg, NumSeg>	m_seg_no_que;

	/*
	 * セグメント参照カウンタ配列へのポインタ
	 * ポインタを格納する代わりに、SegRefCnt m_ref_cnt_array[0]; とすれば
	 * 4bytes節約できるが、本クラスの継承クラスでデータメンバを持てなくなる
	 */
	SegRefCnt* const	m_ref_cnt_array;
}; /* class MemPool */

} /* namespace MemMgrLite */

#endif /* MEMPOOL_H_INCLUDED */
