/****************************************************************************
 * modules/memutils/memory_manager/src/ScopedLock.cpp
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

#ifdef USE_MEMMGR_MULTI_CORE

namespace MemMgrLite {

/*****************************************************************
 * スピンロックIDが指定されていれば、スピンロックを獲得する
 * ベースクラスで、割込み禁止にしている
 *****************************************************************/
ScopedLock::ScopedLock(LockId id) : m_spl_id(id)
{
#ifdef TIME_MEASUREMENT
#endif
	if (m_spl_id != NullLockId) {
		/* スピンロック獲得 */
	}
}

/*****************************************************************
 * スピンロック獲得済みであれば、解放する
 * ベースクラスで、割込み禁止を解除する
 *****************************************************************/
ScopedLock::~ScopedLock()
{
	if (m_spl_id != NullLockId) {
		/* スピンロック解放 */
	}
#ifdef TIME_MEASUREMENT
#endif
}

} /* end of namespace MemMgrLite */

#endif /* USE_MEMMGR_MULTI_CORE */

/* ScopedLock.cxx */
