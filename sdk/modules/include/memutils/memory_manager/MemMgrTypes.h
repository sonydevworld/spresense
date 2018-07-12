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
 * @file   MemMgeTypes.h
 * @brief  Types definitions for "Memory Manager".
 * @author CXD5602 Media SW Team
 */

#include <stdio.h>
#include <sdk/config.h>
#include "memutils/common_utils/common_types.h"  /* uintN_t */
#include "memutils/os_utils/cpp_util.h"          /* CopyGuard class */

/* mem_layout.hのコンフィグレーションに依存するヘッダは、これ以降に記述すること */
#ifdef USE_MEMMGR_MULTI_CORE
#include "SpinLockManager.h"
#endif

#define MEMMGR_SIGNATURE  "MML"

class FastMemAlloc;  /* このクラスは名前空間外 */

/**
 * @namespace MemMgrLite
 * @brief namespace for "Memory Manager".
 */
namespace MemMgrLite {

class MemPool;
class MemHandleBase;
typedef uint32_t  MemHandleProxy;    /* MemHandleBaseクラスとの相互参照回避用 */

/** Number of Memory Layouts */
typedef uint8_t    NumLayout;    /* メモリレイアウト数(最大255) */
const NumLayout    BadLayoutNo = 0xff;  /* レイアウト番号は0 origin */

/** ID of a segment pool */
typedef uint8_t    PoolId;      /* メモリプールID(1 origin. 最大255) */
const PoolId    NullPoolId = 0;    /* プールIDの0は未使用 */

/** Type of segment pools */
typedef uint8_t    PoolType;    /* メモリプールタイプ */

/**
 * @enum 
 * @brief Pool types.
 */
enum {            /* enumは、4bytesになるコンパイラがある */
  /** the type number of fixed pools. (Now only support this type.) */
  BasicType,
  RingBufType,
  /** Number of types. */
  NumPoolTypes  /* number of pool types */
};

/*
 * メモリプールアドレスに使用するアドレス体系は、プロジェクトに依存する
 * Allegro CPU環境では、通常0 originの物理アドレスを使用する
 * PoolAddr型では、0番地が有効なため、不正アドレスとして0xffffffffを使用する
 */
typedef uint32_t  PoolAddr;
const PoolAddr    BadPoolAddr = 0xffffffff;

typedef uint32_t  PoolSize;    /* メモリプールサイズ(byte単位) */

#ifdef USE_MEMMGR_OVER255_SEGMENTS
typedef uint16_t  NumSeg;      /* メモリセグメント数(最大65535) */
#else
typedef uint8_t    NumSeg;      /* メモリセグメント数(最大255) */
#endif
const NumSeg    NullSegNo = 0;    /* セグメント番号の0は未使用 */

typedef uint8_t    SegRefCnt;    /* セグメント参照カウント(最大255) */

#ifdef USE_MEMMGR_MULTI_CORE
/* InterCpuLock::SpinLockIdはuint16_tだが、メモリ節約のため別型で定義 */
typedef uint8_t    LockId;      /* スピンロックID(1 origin. 最大255) */
const LockId    NullLockId = 0;    /* スピンロックIDの0は未使用 */

/* MemHandleのflagsフィールドに収めるため、6bit以下とすること */
typedef uint8_t    CpuId;      /* CPU-ID (0 origin 最大15) */
const CpuId    MaskCpuId = 0x0f;  /* とりあえず下位4bitを有効とする */
const CpuId    MaxCpuId = MaskCpuId;
#endif

/*****************************************************************
 * Memory Pool Attributes (12 or 16bytes)
 *****************************************************************/
struct PoolAttr {
  PoolId    id;    /* pool ID */
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

} /* namespace MemMgrLite */

/**
 * @}
 */

/**
 * @}
 */


#endif /* MEMMGRTYPES_H_INCLUDED */
