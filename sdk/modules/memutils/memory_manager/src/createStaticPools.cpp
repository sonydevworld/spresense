/****************************************************************************
 * modules/memutils/memory_manager/src/createStaticPools.cpp
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

namespace MemMgrLite {

/*****************************************************************
 * 静的メモリプール群の生成
 *****************************************************************/
err_t Manager::createStaticPools(NumLayout layout_no, void* work_area, uint32_t area_size, const PoolAttr *pool_attr)
{
#ifdef USE_MEMMGR_DEBUG_OUTPUT
  printf("Manager::createStaticPools(layout_no=%d, work_area=%08x, area_size=%08x)\n",
    layout_no, work_area, area_size);
#endif
  if (reinterpret_cast<uint32_t>(work_area) % sizeof(uint32_t) != 0)
    {
      return ERR_ADR_ALIGN;
    }
  
  if (theManager == NULL)
    {
      return ERR_STS;
    }
 
  if (isStaticPoolAvailable())
    {
      return ERR_STS;
    }

  FastMemAlloc  fma(work_area, area_size);

  ScopedLock lock;

  for (uint32_t i = 0; i < theManager->m_pool_num; ++i) {
    if (pool_attr[i].id == NullPoolId) {
      break;  /* ID:0はプール定義の終了を示す */
    }

    /* メモリプールを生成する */
#ifdef USE_MEMMGR_COPIED_POOL_ATTR
    PoolAttr* copied_attr = new(fma, sizeof(uint32_t)) PoolAttr(pool_attr[i]);
    D_ASSERT(copied_attr);  /* 下のF_ASSERTだけでも問題ないが念のため */
    theManager->m_static_pools[pool_attr[i].id] = createPool(*copied_attr, fma);
#else
    theManager->m_static_pools[pool_attr[i].id] = createPool(pool_attr[i], fma);
#endif
    if (theManager->m_static_pools[pool_attr[i].id] == NULL) {
      return ERR_DATA_SIZE; /* work_areaのサイズ不足 */
    }
  }
  theManager->m_layout_no = layout_no;  /* 生成に成功したのでレイアウト番号を設定する */

  return ERR_OK;
}

} /* end of namespace MemMgrLite */

/* createStaticPools.cxx */
