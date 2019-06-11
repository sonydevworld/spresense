/****************************************************************************
 * modules/include/memutils/memory_manager/Manager.h
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

#ifndef MANAGER_H_INCLUDED
#define MANAGER_H_INCLUDED

#include <sdk/config.h>

/**
 * @defgroup memutils_memory_manager Memory Manager Lite
 * @{
 *
 * @file   Manager.h
 * @brief  Memory Manager API
 * @author CXD5602 Media SW Team
 */

#include "memutils/common_utils/common_errcode.h"
#include "memutils/memory_manager/MemPool.h"

/**
 * @namespace MemMgrLite
 * @brief namespace for "Memory Manager".
 */
namespace MemMgrLite {

/*****************************************************************
 * Manager class
 *****************************************************************/

/**
 * @class Manager
 * @brief Memory Management Class for "Memory Manager Lite".
 *        User can create only one instance.
 */
class Manager : CopyGuard {

public:

  /** The 1st initialize method on boot. 
    * This method only called once on the system.(from system processor)
    * @param[in] manager_area the area pointer of the data structure for management "memory manager". 
    * @param[in] area_size    the size of the data structure for management "memory manager". 
    * @return ERR_OK        : success
    * @return ERR_STS       : error, initFirst() is already executing
    * @return ERR_DATA_SIZE : error, area_size less than sizeof(MemMgrLite::Manager)
    * @return ERR_ADR_ALIGN : error, address is not 4bytes alignment
    */
  static err_t  initFirst(void* manager_area, uint32_t area_size);

  /** The 2nd initialize method on boot. 
    * This method only called once on each processor.
    * @param[in] manager_area The area pointer of the data structure for management "memory manager". 
    * @return ERR_OK        : success
    * @return ERR_STS       : error, initPerCpu() is already executing
    */
  static err_t  initPerCpu(void* manager_area, uint32_t pool_num);

  static err_t  initPerCpu(void* manager_area, MemPool ** static_pools[], uint8_t* pool_num, uint8_t* layout_no);

  /* MemoryManager Finalize Process */
  /** The finalize method on power-off. 
    * This method only called once on the system.(from system processor)
    * @return void
    */
  static err_t  finalize();

  /** The static memory layout creation method. 
    * This mathod provide changing memory layouts. 
    * @param[in] layout_no A layout number. 
    * @param[in] work_area The area pointer of memory pools used by "memory manager" with layout number.
    * @param[in] area_size The size of memory pools used by "memory manager" with layout number.
    * @return ERR_OK        : success
    * @return ERR_STS       : error, initPerCpu() is already executing
    * @return ERR_ARG       : error, layout_no is over max(NUM_MEM_LAYOUTS-1)
    * @return ERR_DATA_SIZE : error, area_size less than MEMMGR_MAX_WORK_SIZE
    * @return ERR_ADR_ALIGN : error, work area is not 4bytes alignment
    */
  static err_t  createStaticPools(uint8_t sec_no, NumLayout layout_no, void* work_area, uint32_t area_size, const PoolSectionAttr *pool_attr);
  static err_t  createStaticPools(NumLayout layout_no, void* work_area, uint32_t area_size, const PoolAttr *pool_attr){
    return createStaticPools(0, layout_no, work_area, area_size, reinterpret_cast <const PoolSectionAttr*>(pool_attr));
  }

  /** The static memory layout destroy method.
    * This mathod provide clear the memory layout. 
    * @return void
    */
  static void  destroyStaticPools(uint8_t sec_no);
  static void  destroyStaticPools(){ destroyStaticPools(0); }

  /** The getter method for layout number.
    * @return NumLayout The current layout number.
    */
  static NumLayout  getCurrentLayoutNo(uint8_t sec_no) { return theManager->m_layout_no[sec_no]; }
  static bool isStaticPoolAvailable(uint8_t sec_no) { return getCurrentLayoutNo(sec_no) != BadLayoutNo; }

#ifdef USE_MEMMGR_DYNAMIC_POOL
  /* Create/destroy dynamic memory pool. */

  static PoolId  createDynamicPool(const PoolAttr& attr, void* work_area, uint32_t area_size);
  static void  destroyDynamicPool(PoolId id);
  static uint32_t  getDynamicPoolWorkSize(const PoolAttr& attr) { return DYN_POOL_WORK_SIZE(attr); }
#endif

  /* Get used memory segment information. */

  static uint32_t  getStaticPoolsUsedSegs(uint8_t sec,MemHandleBase* mhs, uint32_t num_mhs);
  static uint32_t  getUsedSegs(uint8_t sec,PoolId id, MemHandleBase* mhs, uint32_t num_mhs) {
    return findPool(id)->getUsedSegs(mhs, num_mhs);
  }

  /* Get memory pool information. */

  static bool  isPoolAvailable(PoolId id) { return getPoolObject(id) != NULL; }
  static PoolType  getPoolType(PoolId id) { return findPool(id)->getPoolType(); }
  static PoolAddr  getPoolAddr(PoolId id) { return findPool(id)->getPoolAddr(); }
  static PoolSize  getPoolSize(PoolId id) { return findPool(id)->getPoolSize(); }
  static NumSeg  getPoolNumSegs(PoolId id) { return findPool(id)->getPoolNumSegs(); }
  static NumSeg  getPoolNumAvailSegs(PoolId id) { return findPool(id)->getPoolNumAvailSegs(); }
#ifdef CONFIG_MEMUTILS_MEMORY_MANAGER_USE_FENCE
  static bool  isPoolFenceEnable(PoolId id) { return findPool(id)->isPoolFenceEnable(); }
#endif
#ifdef USE_MEMMGR_MULTI_CORE
  static LockId  getPoolLockId(PoolId id) { return findPool(id)->getPoolLockId(); }
#endif

#ifdef USE_MEMMGR_DEBUG_OUTPUT
//  void    printInfo(PoolId id);
#endif

#ifdef CONFIG_MEMUTILS_MEMORY_MANAGER_USE_FENCE
  /* Verify the fence and return the error detection count. */

  static uint32_t  verifyFixedAreaFences();
  static uint32_t  verifyStaticPoolsFence(uint8_t sec);
  static uint32_t  verifyPoolFence(PoolId id) { return findPool(id)->verifyPoolFence(); }
#endif

private:
  Manager();  /* called from initFirst */

  static MemPool* createPool(const PoolSectionAttr& attr, FastMemAlloc& fma);
  static void  destroyPool(MemPool* pool);
  static void  initFixedAreaFences();

  /* Returns a pointer to the memory pool object corresponding to the pool ID.
   * If the object has not been created, it returns NULL.
   */

  static MemPool* getPoolObject(PoolId id) {
    if (id.pool < theManager->m_pool_num[id.sec]) {
      return theManager->m_static_pools[id.sec][id.pool];
#ifdef USE_MEMMGR_DYNAMIC_POOL
    } else if (id < theManager->m_pool_num + NUM_DYN_POOLS) {
      return theManager->m_dynamic_pools[id.pool - theManager->m_pool_num];
#endif
    } else {
      D_ASSERT(0);  /* Illegal pool ID. */
      return NULL;
    }
  }

  static MemPool* findPool(PoolId id) {
    MemPool* p = getPoolObject(id);
    D_ASSERT(p);
    return p;
  }

  /* Memory segment allocate/free/get information. */

  friend class MemHandleBase;
#ifdef USE_MEMMGR_SEG_DELETER
  static err_t allocSeg(PoolId id, size_t size_for_check, MemHandleProxy &proxy, bool use_deleter);
#else
  static err_t allocSeg(PoolId id, size_t size_for_check, MemHandleProxy &proxy);
#endif
  static void      freeSeg(MemHandleBase& mh);
  static PoolAddr  getSegAddr(const MemHandleBase& mh);
  static PoolSize  getSegSize(const MemHandleBase& mh);
  static SegRefCnt getSegRefCnt(PoolId id, NumSeg seg_no) { return findPool(id)->getSegRefCnt(seg_no); }
  static void      incSegRefCnt(PoolId id, NumSeg seg_no) { findPool(id)->incSegRefCnt(seg_no); }

private:
  static Manager*  theManager;    /* for singleton */

  uint8_t    m_signature[3]; /* Initialize determination and
                              * mark for dumping.
                              */
  uint32_t   m_fix_fene_num; /* Number of fence. */

  uint8_t*   m_layout_no;    /* Current memory layout number. */
  uint8_t*   m_pool_num;     /* Number of pool. */
  MemPool*** m_static_pools;

#ifdef USE_MEMMGR_DYNAMIC_POOL
  MemPool*  m_dynamic_pools[NUM_DYN_POOLS];
  RuntimeQue<PoolId, PoolId>  m_pool_no_que; /* 8bytes */
  PoolId    m_pool_no_array[NUM_DYN_POOLS];  /* Data area of the queue */
#endif
}; /* class Manager */

} /* namespace MemMgrLite */

/**
 * @}
 */

/**
 * @}
 */

#endif /* MANAGER_H_INCLUDED */
