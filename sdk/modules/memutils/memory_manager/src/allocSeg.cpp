/****************************************************************************
 * modules/memutils/memory_manager/src/allocSeg.cpp
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
#include "memutils/memory_manager/MemHandleBase.h"
#include "BasicPool.h"

namespace MemMgrLite {

/*****************************************************************
 * APIでメモリセグメントを取得する。成功時はtrueを返す
 *****************************************************************/
err_t MemHandleBase::allocSeg(PoolId id, size_t size_for_check)
{
  freeSeg();

  return Manager::allocSeg(id, size_for_check, this->m_proxy);
}

/*****************************************************************
 * メモリセグメントを取得して、操作用のハンドルオブジェクトを返す
 *****************************************************************/
err_t Manager::allocSeg(PoolId id, size_t size_for_check, MemHandleProxy &proxy)
{
  MemPool* pool = findPool(id);

#ifdef USE_MEMMGR_RINGBUF_POOL
  /* 仮想関数を使用しない方針なので、該当プール型にダウンキャストする */
  switch (pool->getPoolType()) {
  case BasicType:
    return static_cast<BasicPool*>(pool)->allocSeg(size_for_check);
  case RingBufType:
    return static_cast<RingBufPool*>(pool)->allocSeg(size_for_check);
  default:
    D_ASSERT(false);
    return MemHandleProxy();  /* default constructor */
  }
#else
  /* BasicPoolのみ使用時は、各種チェックを省略する */
  return static_cast<BasicPool*>(pool)->allocSeg(size_for_check, proxy);
#endif
}

/*****************************************************************
 * Basicプールのセグメントハンドルを取得する
 *****************************************************************/
err_t BasicPool::allocSeg(size_t size_for_check, MemHandleProxy &proxy)
{
  if (size_for_check > getSegSize())
    {
      return ERR_DATA_SIZE;
    }

  ScopedLock lock;
  proxy = MemPool::allocSeg();

  if (proxy == 0)
    {
      return ERR_MEM_EMPTY;
    }

  return ERR_OK;
}

/*****************************************************************
 * メモリプールからセグメントハンドルを取得する
 * 排他制御は呼出し側で行うこと
 *****************************************************************/
MemHandleProxy MemPool::allocSeg()
{
  MemHandleProxy  mhp = 0;

  if (m_seg_no_que.size()) {
    /* セグメント番号を割り当てる */
    NumSeg seg_no = m_seg_no_que.top();
    m_seg_no_que.pop();

    /* セグメント参照カウンタを設定する */
    D_ASSERT(m_ref_cnt_array[seg_no - 1] == 0);  /* 未使用のはず */
    m_ref_cnt_array[seg_no - 1] = 1;  /* インクリメントより代入の方が効率が良い */

    mhp = MemHandleBase::makeMemHandleProxy(getPoolId(), seg_no, 0);
  }
  return mhp;
}

} /* end of namespace MemMgrLite */

/* allocSeg.cxx */
