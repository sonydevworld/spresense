/****************************************************************************
 * modules/memutils/memory_manager/src/createDynamicPool.cpp
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

#include "FastMemAlloc.h"
#include "ScopedLock.h"
#include "memutils/memory_manager/Manager.h"

#ifdef USE_MEMMGR_DYNAMIC_POOL

namespace MemMgrLite {

/*****************************************************************
 * 動的メモリプールを生成して、プールIDを返す
 *****************************************************************/
PoolId Manager::createDynamicPool(const PoolAttr& attr, void* work_area, uint32_t area_size)
{
#ifdef USE_MEMMGR_DEBUG_OUTPUT
  printf("Manager::createDynamicPool(");
  attr.printInfo(false);
  printf(", work_area=%08x, area_size=%08x)\n", work_area, area_size);
#endif
  D_ASSERT(attr.num_segs && attr.addr % sizeof(uint32_t) == 0 && attr.size >= attr.num_segs);
  D_ASSERT(reinterpret_cast<uint32_t>(work_area) % sizeof(uint32_t) == 0);
  D_ASSERT(area_size >= getDynamicPoolWorkSize(attr));
  D_ASSERT(theManager);

  /* 動的プールの空き番号を取得 */
  uint32_t no;
  {
    ScopedLock lock;
    if (theManager->m_pool_no_que.size()) {
      no = theManager->m_pool_no_que.top();
      theManager->m_pool_no_que.pop();
    } else {
      return NullPoolId;
    }
  }

  /* 作業領域用のアロケータを初期化 */
  FastMemAlloc  fma(work_area, area_size);

  /* 引数attrの内容を作業領域にコピーして、プールIDを設定する */
  PoolAttr* pool_attr = new(fma, sizeof(uint32_t)) PoolAttr(attr);
  D_ASSERT(pool_attr);  /* 下のF_ASSERTだけでも問題ないが念のため */
  pool_attr->id = no + theManager->m_pool_num;  /* 番号に下駄を履かせて、IDに変換する */
#ifdef CONFIG_MEMUTILS_MEMORY_MANAGER_USE_FENCE
  /* フェンス有効時は、フェンスを除いたプール領域定義値に変更する */
  if (attr.fence) {
    pool_attr->addr += sizeof(uint32_t);
    pool_attr->size -= sizeof(uint32_t) * 2;
  }
#endif
  /* メモリプールを生成する */
  D_ASSERT(theManager->m_dynamic_pools[no] == NULL);  /* 空きのはず */
  theManager->m_dynamic_pools[no] = createPool(*pool_attr, fma);
  if (theManager->m_dynamic_pools[no] == NULL) {
    F_ASSERT(0); /* このアサートに引っかかるのは、通常work_areaのサイズ不足 */
  }
  return pool_attr->id;
}

} /* end of namespace MemMgrLite */

#endif /* USE_MEMMGR_DYNAMIC_POOL */

/* createDynamicPool.cxx */
