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

/* Virtual function is prohibited for the following reason.
 * - Text Non-shared multicore can not use virtual function.
 * - By using virtual function 4 bytes size increases for each instance.
 */

namespace MemMgrLite {

/*****************************************************************
 * Memory pool base class (16 or 20bytes)
 *****************************************************************/
class MemPool : CopyGuard {
	friend class Manager;
protected:
	MemPool(const PoolSectionAttr& attr, FastMemAlloc& fma);
	~MemPool();

  /* Since exceptions can not be used, constructor errors are
   * checked with this function.
   */

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

  /* Get and update of segment reference counter value. */

	SegRefCnt	getSegRefCnt(NumSeg seg_no) const {
		D_ASSERT(seg_no != NullSegNo && seg_no <= getPoolNumSegs());
		D_ASSERT(m_ref_cnt_array[seg_no - 1] != 0); /* It should be in use. */
		return m_ref_cnt_array[seg_no - 1];
	}
	void		incSegRefCnt(NumSeg seg_no);

  /* Store the memory segment in use on the memory handle and
   * return the stored number.
   */

	uint32_t	getUsedSegs(MemHandleBase* mhs, uint32_t num_mhs);

  /* Get a segment from the memory pool.
   * Exclusive control should be done on the caller side.
   */

	MemHandleProxy	allocSeg();

  /* Subtract the reference counter and return the segment
   * if there is no reference.
   * Exclusive control should be done on the caller side.
   */

	void	freeSeg(MemHandleBase& mh);

protected:
  /* In the case of a static pool, it points to the corresponding part
   * of MemoryPoolLayouts.
   * When dynamic pooling, address rewriting, etc. are necessary,
   * separate PoolAttr storage places should be prepared
   */

	const PoolSectionAttr&	m_attr;		/* pool attributes */

  /* A queue (8 or 12 bytes) holding an usable segment number (1 origin).
   * It is necessary to separately prepare the area for queue data.
   */

	RuntimeQue<NumSeg, NumSeg>	m_seg_no_que;

  /* Pointer to segment reference counter array.
   * If you do not store the pointer and set it to
   * SegRefCnt m_ref_cnt_array[0], you can save 4 bytes.
   * However, you can not have data members in this class inheritance class.
   */

	SegRefCnt* const	m_ref_cnt_array;
}; /* class MemPool */

} /* namespace MemMgrLite */

#endif /* MEMPOOL_H_INCLUDED */
