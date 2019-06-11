/****************************************************************************
 * modules/memutils/memory_manager/src/fence.cpp
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

#include <string.h>
#include "ScopedLock.h"
#include "memutils/memory_manager/MemManager.h"

#ifdef CONFIG_MEMUTILS_MEMORY_MANAGER_USE_FENCE

namespace MemMgrLite {

/*
 * ポインタとして参照された時に例外が発生するよう
 * RAM/ROM範囲外で、LSBが立っている値が望ましい
 */
const uint32_t FenceValue = 0xfece2bad;    /* fe(n)ce to bad */
extern PoolAddr const FixedAreaFences[];

/*****************************************************************
 * FixedAreaのフェンスを初期化
 *****************************************************************/
void Manager::initFixedAreaFences()
{
  if(theManager->m_fix_fene_num != 0){
    const PoolAddr* fexed_fences = FixedAreaFences;
    for (uint32_t i = 0; i < theManager->m_fix_fene_num; ++i) {
      *static_cast<uint32_t*>(translatePoolAddrToVa(fexed_fences[i])) = FenceValue;
    }
  }
}

/*****************************************************************
 * FixedAreaのフェンスを検証
 *****************************************************************/
uint32_t Manager::verifyFixedAreaFences()
{
  uint32_t ng_cnt = 0;

  if(theManager->m_fix_fene_num != 0){
    const PoolAddr* fexed_fences = FixedAreaFences;
    for (uint32_t i = 0; i < theManager->m_fix_fene_num; ++i) {
      uint32_t* va = static_cast<uint32_t*>(translatePoolAddrToVa(fexed_fences[i]));
      if (*va != FenceValue) {
#ifdef USE_MEMMGR_DEBUG_OUTPUT
        printf("FixedArea fence verification error: addr=%08x(%08x), value=%08x\n",
          va, FixedAreaFences[i], *va);
#endif
        ++ng_cnt;
      }
    }
  }
  return ng_cnt;
}

/*****************************************************************
 * プールフェンスを初期化
 *****************************************************************/
void MemPool::initPoolFence()
{
  *static_cast<uint32_t*>(translatePoolAddrToVa(m_attr.addr - sizeof(uint32_t))) = FenceValue;
  *static_cast<uint32_t*>(translatePoolAddrToVa(m_attr.addr + m_attr.size))      = FenceValue;
}

/*****************************************************************
 * プールフェンスを検証
 *****************************************************************/
uint32_t MemPool::verifyPoolFence()
{
  uint32_t ng_cnt = 0;
  if (isPoolFenceEnable()) {
    uint32_t* lower = static_cast<uint32_t*>(translatePoolAddrToVa(m_attr.addr - sizeof(uint32_t)));
    if (*lower != FenceValue) {
#ifdef USE_MEMMGR_DEBUG_OUTPUT
      printf("Pool#%d lower fence verification error: addr=%08x(%08x), value=%08x\n",
        m_attr.id, lower, m_attr.addr - sizeof(uint32_t), *lower);
#endif
      ++ng_cnt;
    }

    uint32_t* upper = static_cast<uint32_t*>(translatePoolAddrToVa(m_attr.addr + m_attr.size));
    if (*upper != FenceValue) {
#ifdef USE_MEMMGR_DEBUG_OUTPUT
      printf("Pool#%d upper fence verification error: addr=%08x(%08x), value=%08x\n",
        m_attr.id, upper, m_attr.addr + m_attr.size, *upper);
#endif
      ++ng_cnt;
    }
  }
  return ng_cnt;
}

/*****************************************************************
 * 静的メモリプールのフェンスを検証
 *****************************************************************/
uint32_t Manager::verifyStaticPoolsFence(uint8_t sec_no)
{
  uint32_t ng_cnt = 0;

  ScopedLock lock;

  /* プールID=0は予約 */
  for (uint32_t i = 1; i < theManager->m_pool_num[sec_no]; ++i) {
    if (theManager->m_static_pools[sec_no][i] != NULL) {
      ng_cnt += theManager->m_static_pools[sec_no][i]->verifyPoolFence();
    }
  }
  return ng_cnt;
}

} /* end of namespace MemMgrLite */

#endif /* CONFIG_MEMUTILS_MEMORY_MANAGER_USE_FENCE */

/* fence.cxx */
