/****************************************************************************
 * modules/memutils/memory_manager/src/destroyStaticPools.cpp
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
#include "memutils/memory_manager/Manager.h"

namespace MemMgrLite {

/*****************************************************************
 * Destroy (delete) static memory pools.
 * When static memory pool is not generated, nothing is done.
 *****************************************************************/
void Manager::destroyStaticPools(uint8_t sec_no)
{
#ifdef USE_MEMMGR_DEBUG_OUTPUT
  printf("Manager::destroyStaticPools: layout_no=%d\n", getCurrentLayoutNo());
#endif

  if ( sec_no > 3)
    {
      /* sec_no is out of range! */

      printf("Out of range sec_no=%d!\n", sec_no);
      return;
    }

  if (isStaticPoolAvailable(sec_no))
    {
      ScopedLock lock;

      /* Pool ID = 0 is reserved. */

      for (uint32_t i = 1; i < theManager->m_pool_num[sec_no]; ++i)
        {
          if (theManager->m_static_pools[sec_no][i] != NULL)
            {
              destroyPool(theManager->m_static_pools[sec_no][i]);
              theManager->m_static_pools[sec_no][i] = NULL;
            }
        }

      /* Disable layout number. */

      theManager->m_layout_no[sec_no] = BadLayoutNo;
    }
}

} /* End of namespace MemMgrLite. */

/* destroyStaticPools.cxx */
