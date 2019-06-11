/****************************************************************************
 * modules/include/memutils/memory_manager/MemHandleBase.h
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

#ifndef MEMHANDLEBASE_H_INCLUDED
#define MEMHANDLEBASE_H_INCLUDED

/**
 * @defgroup memutils_memory_manager Memory Manager Lite
 *
 * @{
 * @file MemHandleBase.h
 * @brief  Memory Manager Lite Memory Handle Base class API
 * @author CXD5602 Media SW Team
 */

#include "memutils/common_utils/common_errcode.h"
#include "memutils/memory_manager/Manager.h"

namespace MemMgrLite {

/*****************************************************************
 * Memory Handle Base class (4bytes fixed)
 *****************************************************************/
/**
 *  @class MemHandleBase
 *  @brief Memory Handler Base Class for Memory Handler Base Class.
 *         This class`s methods can called only from MemHandle.(Cannot call this class directory.)
 */
class MemHandleBase {
public:
  MemHandleBase() { clear(); }

#ifdef USE_MEMMGR_SEG_DELETER
  MemHandleBase(PoolId id, size_t size_for_check, bool use_deleter = false) {
    Manager::allocSeg(id, size_for_check, m_proxy, use_deleter);
  }
#else
  MemHandleBase(PoolId id, size_t size_for_check) {
    Manager::allocSeg(id, size_for_check, m_proxy);
  }

  MemHandleBase(uint8_t id, size_t size_for_check) {
    PoolId pool_id;
    pool_id.sec = 0;
    pool_id.pool = id;
    Manager::allocSeg(pool_id, size_for_check, m_proxy);
  }

#endif

  MemHandleBase(const MemHandleBase& mh) {
    m_proxy = mh.m_proxy;
    if (isAvail()) Manager::incSegRefCnt(getPoolId(), getSegNo());
  }

  ~MemHandleBase() { freeSeg(); }

  MemHandleBase&  operator=(const MemHandleBase& mh);

#ifdef USE_MEMMGR_SEG_DELETER
  err_t allocSeg(PoolId id, size_t size_for_check, bool use_deleter = false);
#else
  /** The allocator from a pool area for MemHandle.
    * @param[in] id             The pool id that MemHandle be allocated.
    * @param[in] size_for_check The size the user wants.(for only size check.)
    *  @return ERR_OK        : success
    *  @return ERR_DATA_SIZE : error, size is over segment size
    *  @return ERR_MEM_EMPTY : error, there are no segments available
    */
  err_t allocSeg(PoolId id, size_t size_for_check);

  err_t allocSeg(uint8_t id, size_t size_for_check){
    PoolId pool_id;
    pool_id.sec = 0;
    pool_id.pool = id;
    return allocSeg(pool_id, size_for_check);
  }
#endif
  /** The free from a pool area for MemHandle.
    * @return void (If this handler did not allocate, this method do nothing.)
    */
  void    freeSeg() { if (isAvail()) Manager::freeSeg(*this); }

  bool    isAvail() const { return m_proxy; }
  bool    isNull() const { return !isAvail(); }
  bool    isSame(const MemHandleBase& mh) { return m_proxy == mh.m_proxy; }
  PoolId    getPoolId() const { return m_seg_info.pool_id; }
  NumSeg    getSegNo()  const { return m_seg_info.seg_no; }
  uint8_t    getFlags() const { return m_seg_info.flags; }  /* for debug */
#ifdef USE_MEMMGR_MULTI_CORE
  CpuId    getCpuId() const { return getFlags() & MaskCpuId; }
#endif
  PoolAddr  getAddr() const { return Manager::getSegAddr(*this); }
  PoolSize  getSize() const { return Manager::getSegSize(*this); }
  SegRefCnt  getRefCnt() const { return Manager::getSegRefCnt(getPoolId(), getSegNo()); }

private:
  friend class MemPool;

  struct SegInfo {
    PoolId    pool_id;
    uint8_t   flags;
    NumSeg    seg_no;    /* segment number (1 origin) */
#ifndef USE_MEMMGR_OVER255_SEGMENTS
    uint8_t    resv;
#endif
  }; /* struct SegInfo */

  /* Internally treat it as simple binary so that
   * unnecessary constructor/destructor will not be executed.
   */

  static MemHandleProxy makeMemHandleProxy(PoolId id, NumSeg seg_no, uint8_t flags) {
    SegInfo  seg = { id, flags, seg_no
#ifndef USE_MEMMGR_OVER255_SEGMENTS
        , 0
#endif
    };
    return *reinterpret_cast<MemHandleProxy*>(&seg);
  }

  void clear() { m_proxy = 0; }

private:
  union {
    SegInfo    m_seg_info;
    MemHandleProxy  m_proxy;
  };
}; /* class MemHandleBase */

S_ASSERT(sizeof(MemHandleBase) == 4);

} /* namespace MemMgrLite */

/**
 * @}
 */

/**
 * @}
 */

#endif /* MEMHANDLEBASE_H_INCLUDED */
