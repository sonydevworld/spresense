/****************************************************************************
 * modules/memutils/memory_manager/src/createPool.cpp
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

#include <string.h>    /* memset */
#include "FastMemAlloc.h"  /* FastMemAlloc class */
#include "memutils/memory_manager/Manager.h"
#include "BasicPool.h"

namespace MemMgrLite {

/*****************************************************************
 * Create a memory pool object
 *****************************************************************/
MemPool* Manager::createPool(const PoolSectionAttr& attr, FastMemAlloc& fma)
{
  MemPool* pool = NULL;
#ifdef USE_MEMMGR_RINGBUF_POOL
  switch (attr.type) {
  case BasicType:
    pool = new(fma, sizeof(uint32_t)) BasicPool(attr, fma);
    break;
  case RingBufType:
    pool = new(fma, sizeof(uint32_t)) RingBufPool(attr, fma);
    break;
  default:
    D_ASSERT(false);  /* Unsupport pool type */
    break;
  }
#else
  /* BasicPoolのみ使用時は、各種チェックを省略する */
  pool = new(fma, sizeof(uint32_t)) BasicPool(attr, fma);
#endif
  /* コンストラクタ内でエラー? */
  if (pool && pool->isFailed()) {
    pool = NULL;
  }

  if (pool) {
    /* 成功時の処理 */
  }
  return pool;
}

/*****************************************************************
 * Basicプールのコンストラクタ
 *****************************************************************/
BasicPool::BasicPool(const PoolSectionAttr& attr, FastMemAlloc& fma) :
  MemPool(attr, fma)
{
#ifdef USE_MEMMGR_DEBUG_OUTPUT
  printf("BasicPool: created. [fma.rest=%08x] ", fma.rest());
  attr.printInfo();
#endif
}

/*****************************************************************
 * メモリプールのコンストラクタ
 *****************************************************************/
MemPool::MemPool(const PoolSectionAttr& attr, FastMemAlloc& fma) :
  m_attr(attr),
  m_seg_no_que(fma.alloc(sizeof(NumSeg) * attr.num_segs, sizeof(NumSeg)), attr.num_segs),
  m_ref_cnt_array(static_cast<SegRefCnt*>(fma.alloc(sizeof(SegRefCnt) * attr.num_segs, sizeof(SegRefCnt))))
{
  if (m_seg_no_que.que_area() && m_ref_cnt_array) { /* alloc成功 ? */
    /* 使用可能なセグメント番号(1 origin)を設定 */
    for (uint32_t i = 1; i <= static_cast<uint32_t>(attr.num_segs); ++i) {
      (void)m_seg_no_que.push(static_cast<NumSeg>(i));
    }

    /* 参照カウンタ配列を初期化 */
    memset(m_ref_cnt_array, 0x00, sizeof(SegRefCnt) * attr.num_segs);

#ifdef CONFIG_MEMUTILS_MEMORY_MANAGER_USE_FENCE
    if (isPoolFenceEnable()) {
      initPoolFence();  /* プールフェンスを初期化 */
    }
#endif
  }
}

} /* end of namespace MemMgrLite */

/* createPool.cxx */
