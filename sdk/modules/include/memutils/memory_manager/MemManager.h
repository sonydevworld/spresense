/****************************************************************************
 * modules/include/memutils/memory_manager/MemManager.h
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

/**
 * @defgroup memutils Memory Utility Libraries
 * @{
 */

#ifndef MEMMANAGER_H_INCLUDED
#define MEMMANAGER_H_INCLUDED

/**
 * @defgroup memutils_memory_manager Memory Manager Lite
 *
 * "Memory Manager Lite" provides fixed memory pools with implicitly manage acquisition and release.
 *
 * @{
 * @file MemHandle.h
 * @brief  Memory Handler API
 * @author CXD5602 Media SW Team
 */

#include <string.h>		/* memset */
#include "memutils/memory_manager/MemHandleBase.h"
#include "memutils/memory_manager/MemMgrTypes.h"

/**
 * @namespace MemMgrLite
 * @brief namespace for "Memory Manager".
 */
namespace MemMgrLite {

/*
 * Address Converters for MemHandle.
 */
static inline void* translatePoolAddrToVa(PoolAddr addr)
{
	return reinterpret_cast<void*>(addr);
}

static inline void* translateVaToPa(void* addr)
{
  /* If the address is in the virtual address converter support area
   * (0x00000000 to 0x000FFFFF), convert it to a physical address
   */

  if (addr < reinterpret_cast<void*>(0x00100000))
    {
      uint32_t tileId;
      uint32_t tileVal;
      uint32_t cpuId;
      uint32_t reg;
      uint32_t pa;
      uint32_t va;

      va = (uint32_t)addr;
      tileId = (va >> 16) & 0xf;
      cpuId  = *(volatile uint32_t *)((0x4c000000 | 0x02002000) + 0x40);
      reg = (0x02012000 + 0x04) + (0x04 * (tileId / 2)) + ((cpuId - 2) * 0x20);
      pa = *(volatile uint32_t *)(reg);
      tileVal = ((pa >> ((tileId & 0x1) * 16)) & 0x01ff) << 16;

      return (void *)(0x0c000000 | tileVal | (va & 0xffff));
    }

  return addr;
}

/**
 *  @class MemHandle
 *  @brief Memory Handler Class for "Memory Manager Lite".
 *         This is only wrapper class for convert project-specific address.
 */

class MemHandle : public MemHandleBase {
public:
	MemHandle() : MemHandleBase() {}
#ifdef USE_MEMMGR_SEG_DELETER
	MemHandle(PoolId id, size_t size, bool use_deleter = false) :
		MemHandleBase(id, size, use_deleter) {}
#else
	MemHandle(PoolId id, size_t size) : MemHandleBase(id, size) {}
	MemHandle(uint8_t id, size_t size) : MemHandleBase(id, size) {}
#endif
	MemHandle(const MemHandle& mh) : MemHandleBase(mh) {}

       /** The getter for the virtual address on MemHandle
         * @return void* the virtual address of this handle pointing area.
         */
	void*	getVa()         const { return translatePoolAddrToVa(getAddr()); }

       /** The getter for the physical address on MemHandle.
         * @return void* the physical address of this handle pointing area.
         */
	void*	getPa()         const { return translateVaToPa(getVa()); }

       /** the filler for all area on MemHandle with "c". This is for debug.
         * @param[in] c  A charactera to be filled.
         * @return void
         */
	void	fill(unsigned char c) { memset(getVa(), c, getSize()); }

#ifdef USE_MEMMGR_DEBUG_OUTPUT
	void printInfo(bool newline = true) const {
		printf("PoolId=%3d, SegNo=%3d, Flags=%02x", getPoolId(), getSegNo(), getFlags());
		if (isAvail()) {
			printf(", VA=%08x(%08x), Size=%08x, SegRefCnt=%3d",
				getVa(), getAddr(), getSize(), getRefCnt());
		}
		if (newline) printf("\n");
	}
#endif /* USE_MEMMGR_DEBUG_OUTPUT */

}; /* class MemHandle */

S_ASSERT(sizeof(MemHandle) == 4);

} /* namespace MemMgrLite */

/**
 * @}
 */

/**
 * @}
 */

#endif /* MEMMANAGER_H_INCLUDED */
