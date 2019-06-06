/****************************************************************************
 * modules/memutils/memory_manager/src/initFirst.cpp
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

#include <new>
#include <string.h>
#include "memutils/memory_manager/Manager.h"

namespace MemMgrLite {

/*****************************************************************
 * MemoryManager全体の初期化。単一のCPUで1回だけ実行すること
 *****************************************************************/
err_t Manager::initFirst(void* manager_area, uint32_t area_size)
{
#ifdef USE_MEMMGR_DEBUG_OUTPUT
  printf("Manager::initFirst(addr=%08x, size=%08x)\n", manager_area, area_size);
#endif

  if (reinterpret_cast<uint32_t>(manager_area) % sizeof(uint32_t) != 0)
    {
      return ERR_ADR_ALIGN;
    }

  if (area_size < sizeof(Manager))
    {
      return ERR_DATA_SIZE;
    }

  if (theManager != NULL)
    {
      return ERR_STS;
    }

  /* MemoryManager管理領域の初期化。theManagerへの代入は、initPerCpuで実行する */

  new(manager_area) Manager;

  return ERR_OK;
}

/*****************************************************************
 * MemoryManager Finalize Process
 *****************************************************************/
err_t Manager::finalize()
{
  if (theManager == NULL)
    {
      return ERR_STS;
    }

  memset(theManager, 0x00, sizeof(Manager));
  theManager = NULL;

  return ERR_OK;
}

/*****************************************************************
 * Constructor
 *****************************************************************/
Manager::Manager() :
	m_layout_no(0),
	m_static_pools(0)
#ifdef USE_MEMMGR_DYNAMIC_POOL
	, m_dynamic_pools()
	, m_pool_no_que(m_pool_no_array, NUM_DYN_POOLS)
{
	for (int i = 0; i < NUM_DYN_POOLS; ++i) {
		m_pool_no_que.push(i);	/* 動的プールの空き番号をキューイング */
	}
#else
{
#endif
	/* 処理の最後に署名(初期化済み判定とダンプ時の目印用)を設定する */
	memcpy(m_signature, MEMMGR_SIGNATURE, sizeof(m_signature));
}

} /* end of namespace MemMgrLite */

/* initFirst.cxx */
