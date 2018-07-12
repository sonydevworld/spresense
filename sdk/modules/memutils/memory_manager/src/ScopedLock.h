/****************************************************************************
 * modules/memutils/memory_manager/src/ScopedLock.h
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

#ifndef SCOPEDLOCK_H_INCLUDED
#define SCOPEDLOCK_H_INCLUDED

#include "memutils/memory_manager/MemMgrTypes.h"

#include "memutils/os_utils/chateau_osal.h"
namespace MemMgrLite {
static uint32_t context;
static inline bool isDisableInt() { return Chateau_GetInterruptMask(); }
static inline void disableInt() { if (Chateau_IsTaskContext()) Chateau_LockInterrupt(&context); else Chateau_LockInterruptIsr(&context); }
static inline void enableInt()  { if (Chateau_IsTaskContext()) Chateau_UnlockInterrupt(&context); else Chateau_UnlockInterruptIsr(&context); }
} /* namespace MemMgrLite */

namespace MemMgrLite {

/*****************************************************************
 * Interrupt lock class
 *****************************************************************/
class InterruptLock : CopyGuard {
public:
	InterruptLock() : m_locked(isDisableInt()) { if (!m_locked) disableInt(); }
	~InterruptLock() { if (!m_locked) enableInt(); }
private:
	bool m_locked;
}; /* class InterruptLock */

/*****************************************************************
 * Scoped lock class
 *****************************************************************/
class ScopedLock : InterruptLock {
public:
#ifdef USE_MEMMGR_MULTI_CORE
	ScopedLock(LockId id = NullLockId);
	~ScopedLock()
#else
	/* InterruptLockクラスで必要なロックは全て行われている */
	ScopedLock() {}
	~ScopedLock() {}
#endif
private:
#ifdef USE_MEMMGR_MULTI_CORE
	LockId	m_spl_id;	/* SpinLock ID */
#endif
}; /* class ScopedLock */

} /* namespace MemMgrLite */

#endif /* SCOPEDLOCK_H_INCLUDED */
