/****************************************************************************
 * modules/include/memutils/memory_manager/MemMgrTypes.h
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

#ifndef MEMMGRTYPES_H_INCLUDED
#define MEMMGRTYPES_H_INCLUDED

/**
 * @defgroup memutils_memory_manager Memory Manager Lite
 *
 * @{
 * @file   MemMgrTypes.h
 * @brief  Types definitions for "Memory Manager".
 * @author CXD5602 Media SW Team
 */

#include <stdio.h>
#include <sdk/config.h>
#include "memutils/common_utils/common_types.h"  /* uintN_t */
#include "memutils/os_utils/cpp_util.h"          /* CopyGuard class */

/* The header depending on the configuration of mem_layout.h
 * should be described after this.
 */

#ifdef USE_MEMMGR_MULTI_CORE
#include "SpinLockManager.h"
#endif

#define MEMMGR_SIGNATURE  "MML"

class FastMemAlloc;  /* This class is outside the namespace. */

/**
 * @namespace MemMgrLite
 * @brief namespace for "Memory Manager".
 */
namespace MemMgrLite {

class MemPool;
class MemHandleBase;
typedef uint32_t  MemHandleProxy; /* For avoiding cross reference with
                                   * MemHandleBase class.
                                   */

/** Number of Memory Layouts */
typedef uint8_t    NumLayout;    /* Number of memory layout(Max 255)/ */
typedef uint8_t    NumSection;   /* Number of memory section(Max 255)/ */
const NumLayout    BadLayoutNo = 0xff;   /* The layout number is 0 origin. */
const NumSection   BadSectionNo = 0xff;  /* The section number is 0 origin. */

/** ID of a segment pool */
typedef struct {
  uint8_t pool:6;	/* Memory pool ID(1 origin. Max 63). */
  uint8_t sec:2;  /* Memory section ID(0 origin. Max 3). */
} PoolId;
const PoolId    NullPoolId = {0,0}; /* Pool ID 0 is unused. */

/** Type of segment pools */
typedef uint8_t    PoolType;    /* Type of memory pool. */

/**
 * @enum 
 * @brief Pool types.
 */
enum
{
  /* enum has a compiler that will be 4 bytes. */

  /** the type number of fixed pools. (Now only support this type.) */
  BasicType,
  RingBufType,
  /** Number of types. */
  NumPoolTypes  /* number of pool types */
};

/* The address scheme use for the memory pool address depends on the project.
 * For PoolAddr type, address 0 is valid,
 * so 0xffffffff is used as an invalid address.
 */

typedef uint32_t  PoolAddr;
const PoolAddr    BadPoolAddr = 0xffffffff;

typedef uint32_t  PoolSize;    /* Size of memory pool (byte order). */

#ifdef USE_MEMMGR_OVER255_SEGMENTS
typedef uint16_t  NumSeg;      /* Number of memory segment(Max 65535). */
#else
typedef uint8_t    NumSeg;     /* Number of memory segment(Max 255). */
#endif
const NumSeg    NullSegNo = 0; /* Segment number 0 is unused. */

typedef uint8_t    SegRefCnt;  /* Segment reference count(Max 255). */

#ifdef USE_MEMMGR_MULTI_CORE
/* InterCpuLock::SpinLockId is uint16_t,
 * but it is defined as a different type for memory saving
 */

typedef uint8_t    LockId;      /* Spin lock id(1 origin. Max 255). */
const LockId    NullLockId = 0; /* Spin lock id 0 is unused. */

/* To fit in the flags field of MemHandle, make it less than 6 bits. */

typedef uint8_t    CpuId;         /* CPU-ID (0 origin. Max 15). */
const CpuId    MaskCpuId = 0x0f;  /* Lower 4 bits are valid. */
const CpuId    MaxCpuId = MaskCpuId;
#endif

/*****************************************************************
 * Memory Pool Attributes (12 or 16bytes)
 *****************************************************************/
struct PoolAttr {
  uint8_t   id;
  PoolType  type;    /* pool type */
  NumSeg    num_segs;  /* number of memory segments */
#ifdef CONFIG_MEMUTILS_MEMORY_MANAGER_USE_FENCE
  bool    fence;
#endif
#ifdef USE_MEMMGR_MULTI_CORE
  LockId    spl_id;
#endif
  PoolAddr  addr;    /* pool address */
  PoolSize  size;    /* pool size (bytes) */

#ifdef USE_MEMMGR_DEBUG_OUTPUT
  void printInfo(bool newline = true) const {
    printf("PoolId=%d Type=%d NumSegs=%3d Addr=%08x Size=%08x",
      id, type, num_segs, addr, size);
#ifdef CONFIG_MEMUTILS_MEMORY_MANAGER_USE_FENCE
    printf(" fence=%d", fence);
#endif
#ifdef USE_MEMMGR_MULTI_CORE
    printf(" Spinlock=%d", spl_id);
#endif
    if (newline) printf("\n");
  }
#endif /* USE_MEMMGR_DEBUG_OUTPUT */
}; /* struct PoolAttr */

struct PoolSectionAttr {
  PoolId    id;      /* pool ID */
  PoolType  type;    /* pool type */
  NumSeg    num_segs;  /* number of memory segments */
#ifdef CONFIG_MEMUTILS_MEMORY_MANAGER_USE_FENCE
  bool    fence;
#endif
#ifdef USE_MEMMGR_MULTI_CORE
  LockId    spl_id;
#endif
  PoolAddr  addr;    /* pool address */
  PoolSize  size;    /* pool size (bytes) */

#ifdef USE_MEMMGR_DEBUG_OUTPUT
  void printInfo(bool newline = true) const {
    printf("PoolId=%d Type=%d NumSegs=%3d Addr=%08x Size=%08x",
      id, type, num_segs, addr, size);
#ifdef CONFIG_MEMUTILS_MEMORY_MANAGER_USE_FENCE
    printf(" fence=%d", fence);
#endif
#ifdef USE_MEMMGR_MULTI_CORE
    printf(" Spinlock=%d", spl_id);
#endif
    if (newline) printf("\n");
  }
#endif /* USE_MEMMGR_DEBUG_OUTPUT */
}; /* struct PoolAttr */

inline bool operator == (PoolId id1, PoolId id2)
{
  return (id1.sec == id2.sec && id1.pool == id2.pool);
}

inline bool operator != (PoolId id1, PoolId id2)
{
  return !(id1.sec == id2.sec && id1.pool == id2.pool);
}

} /* namespace MemMgrLite */

/**
 * @}
 */

/**
 * @}
 */

#endif /* MEMMGRTYPES_H_INCLUDED */
