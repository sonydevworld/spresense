/****************************************************************************
 * modules/memutils/memory_manager/src/initPerCpu.cpp
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
#include "memutils/memory_manager/Manager.h"

namespace MemMgrLite {

Manager* Manager::theManager;

extern MemPool* static_pools[];

/*****************************************************************
 * MemoryManagerのCPU毎の初期化。各CPUで1回だけ呼出すこと
 *****************************************************************/
err_t Manager::initPerCpu(void* manager_area, MemPool** pools[], uint8_t* pool_num, uint8_t* layout_no)
{
#ifdef USE_MEMMGR_DEBUG_OUTPUT
  printf("Manager::initPerCpu(addr=%08x)\n", manager_area);
#endif
  /* 2重初期化のチェック */

  if (theManager != NULL)
    {
      return ERR_STS;
    }

  /* 領域の初期化は、initFirstで実行済みなので、代入のみ */
  theManager = static_cast<Manager*>(manager_area);

  theManager->m_fix_fene_num = CONFIG_MEMUTILS_MEMORY_MANAGER_NUM_FIXED_AREA_FENCES;
  theManager->m_pool_num     = pool_num;
  theManager->m_layout_no    = layout_no;
  theManager->m_static_pools = pools;

  /* initFirst()が実行済みか、署名を確認する */

  if (memcmp(theManager->m_signature, MEMMGR_SIGNATURE, sizeof(theManager->m_signature)) != 0)
    {
      return ERR_STS;
    }

#ifdef CONFIG_MEMUTILS_MEMORY_MANAGER_USE_FENCE
  initFixedAreaFences();  /* FixedAreaフェンスを初期化 */
#endif

  return ERR_OK;
}

err_t Manager::initPerCpu(void* manager_area, uint32_t pool_num)
{
  static uint8_t   tmp_sec[1];
  static uint8_t   tmp_layout[1];
  static MemPool **tmp_pools[1];
  tmp_sec[0]    = pool_num;
  tmp_layout[0] = BadLayoutNo;
  tmp_pools[0]  = static_pools;

  return initPerCpu(manager_area, tmp_pools, tmp_sec, tmp_layout);
}

} /* end of namespace MemMgrLite */

/* initPerCpu.cxx */
