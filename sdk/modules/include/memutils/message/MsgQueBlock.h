/****************************************************************************
 * modules/include/memutils/message/MsgQueBlock.h
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

#ifndef MSG_QUE_BLOCK_H_INCLUDED
#define MSG_QUE_BLOCK_H_INCLUDED

/**
 * @defgroup memutils_message Message Library (for class object send)
 * @{
 *
 * @file   MsgQueBlock.h
 * @brief  Message Library API
 * @author CXD5602 Media SW Team
 */

#include "memutils/common_utils/common_types.h"
#include "memutils/common_utils/common_assert.h"
#include "memutils/os_utils/chateau_osal.h"
#include "memutils/message/cache.h"
#include "memutils/message/MsgQue.h"
#include "memutils/message/MsgLog.h"
#ifdef USE_MULTI_CORE
#include "SpinLockManager.h"	/* InterCpuLock::SpinLockId */
#endif

#include <semaphore.h>

/*****************************************************************
 * Message queue block class
 *****************************************************************/
/**
 * @class MsgQueBlock
 * @brief Message Queue Class.
 */
class MsgQueBlock : CopyGuard {
public:
#ifdef USE_MULTI_CORE
	typedef InterCpuLock::SpinLockId	SpinLockId;
#else
	typedef uint16_t	SpinLockId;
#endif

  /** Receive a Object from another task. 
   * this method can receive a object from refer(MsgQueId) with type(MsgType).
   * @param[in] ms timeout time(millisecond)
   * @param[out] **packet the pointer of Massage packet.
   * @return err_t error code
   */
  err_t recv(uint32_t ms, FAR MsgPacket **packet);

  /* Discard message packet. */

  err_t pop();

  /* Get CPU-ID of queue owner (recipient). */

	MsgCpuId getOwner() const { return m_owner; }

  /* Get whether it is a shared queue or not. */

	bool isShare() const { return m_spinlock != 0; }

  /* Get message packet count. */

	uint16_t getNumMsg(MsgPri pri) const;

  /* Get the number of message packets that can be stored.
   * (Unused queue returns 0)
   */

	uint16_t getRest(MsgPri pri) const;

  /* Reset the information of message. */

  void reset()
    {
      /* Reset semaphore with initialize value 0. */

      Chateau_DeleteSemaphore(m_count_sem);
      Chateau_CreateSemaphore(&m_count_sem, 0, 0);
    }

  /* Debug only. */

	void dump() const;
	void dumpQue(MsgPri pri) const { m_que[pri].dump(); }

protected:
	friend class MsgLib;

  /* Static initialization only. */

	MsgQueBlock(MsgQueId id, MsgCpuId owner, SpinLockId spinlock);

  /* Dynamic initialization. */

  err_t setup(drm_t n_drm, uint16_t n_size, uint16_t n_num,
    drm_t h_drm, uint16_t h_size, uint16_t h_num);

  /* Get queue element size. */

	uint16_t getElemSize(MsgPri pri) const { return m_que[pri].elem_size(); }

  /* Check if own CPU owned queue. */

#ifdef USE_MULTI_CORE
	bool isOwn() const { return GET_CPU_ID() == getOwner(); }
#else
  /* If it is not multi, always return true. */

	bool isOwn() const { return true; }
#endif
  /* Find the size of the transmitted message. */

	template<typename T>
	static size_t getSendSize(const T& param, bool type_check);

  /* Message sending process from task context */

	template<typename T>
	err_t send(MsgPri pri, MsgType type, MsgQueId reply, MsgFlags flags, const T& param);

  /* Message transmission processing from ISR.
   * (Only to non-shared queue owned by own CPU)
   */

	template<typename T>
	err_t sendIsr(MsgPri pri, MsgType type, MsgQueId reply, const T& param);

  /* Notify other CPU that sending message.
   * (H/W dependent part. User implements for each CPU)
   */

	void notifySend(MsgCpuId cpu, MsgQueId dest);

  /* Notify receipt of message.(from other CPU) */

	void notifyRecv();

  /* Insert a message packet header at the end of the queue
   * and return that address.
   */

	MsgPacket* pushHeader(MsgPri pri, const MsgPacketHeader& header);

  /* Lock/Unlock Queue. */

	void lock();
	void unlock();

	struct Tally {
		uint32_t	total_pending;
		uint16_t	max_pending;
		uint16_t	max_queuing[NumMsgPri];
	public:
		Tally() { clear(); }
		void clear() { memset(this, 0x00, sizeof(*this)); }
		void dump() const {
			printf("tally: total_pending=%d, max_pending=%d, max_queuing=%d, %d\n",
				total_pending, max_pending, max_queuing[MsgPriNormal], max_queuing[MsgPriHigh]);
		}
	};

private:
	const MsgQueId		m_id;     /* ID of message queue. */
	bool			m_initDone;       /* Flag of Initialized. */
	const MsgCpuId		m_owner;  /* CPU-ID of queue owner (recipient). */
	Chateau_sem_handle_t	m_count_sem;  /* Count semaphore indicating
                                       * the total number of messages.
                                       */
	const SpinLockId	m_spinlock;   /* ID of Spin lock.
                                   * 0, no sharing between CPUs.
                                   */
	uint16_t		m_pendingMsgCount;  /* Number of messages waiting
                                   * for parameter write.
                                   */
	MsgQue			m_que[NumMsgPri]; /* Queue by priority. */
	MsgQue*			m_cur_que;        /* Queue during message processing. */
	Tally			m_tally;            /* Various measurement values. */
	uint32_t		m_context;        /* Variable for locking. */
}; /* class MsgQueBlock */

/*****************************************************************
 * Constructor (Static initialization only)
 *****************************************************************/
inline MsgQueBlock::MsgQueBlock(MsgQueId id, MsgCpuId owner, SpinLockId spinlock) :
	m_id(id),
	m_initDone(false),
	m_owner(owner),
//	m_count_sem(0),
	m_spinlock(spinlock),
	m_pendingMsgCount(0),
	m_cur_que(NULL),
	m_tally()
{
	m_count_sem.semcount = 0;
}

/*****************************************************************
 * Dynamic initialization
 *****************************************************************/
inline err_t MsgQueBlock::setup(drm_t n_drm, uint16_t n_size, uint16_t n_num,
  drm_t h_drm, uint16_t h_size, uint16_t h_num)
{
  /* What has not been initialized yet. */

  if (m_initDone != false)
    {
      return ERR_STS;
    }

  /* Set queue address, element length, number of elements. */

  m_que[MsgPriNormal].init(n_drm, n_size, n_num);
  if (h_drm != INVALID_DRM)
    {
      m_que[MsgPriHigh].init(h_drm, h_size, h_num);
    }

  /* Cache clear message area. */

  if (isShare())
    {
      Dcache_clear(DRM_TO_CACHED_VA(n_drm), n_size * n_num);
      if (h_drm != INVALID_DRM)
        {
          Dcache_clear(DRM_TO_CACHED_VA(h_drm), h_size * h_num);
        }
    }

  /* Create semaphore with initial value 0. */

  Chateau_CreateSemaphore(&m_count_sem, 0, 0);

  m_initDone = true;  /* Initialization end. */
  Dcache_flush(this, sizeof(*this));  /* Do sync at the caller
                                       * collectively.
                                       */

  return ERR_OK;
}

/*****************************************************************
 * Get message packet count
 *****************************************************************/
inline uint16_t MsgQueBlock::getNumMsg(MsgPri pri) const
{
	D_ASSERT2(pri == MsgPriNormal || pri == MsgPriHigh, AssertParamLog(AssertIdBadParam, pri));

  /* Dynamic information of other CPU owned queue must be read
   * after cache clearing.
   */

	if (!isOwn()) {
		Dcache_clear_sync(this, sizeof(*this));
	}
	return m_que[pri].size();
}

/*****************************************************************
 * Get the number of message packets that can be stored
 * (Unused queue returns 0)
 *****************************************************************/
inline uint16_t MsgQueBlock::getRest(MsgPri pri) const
{
	D_ASSERT2(pri == MsgPriNormal || pri == MsgPriHigh, AssertParamLog(AssertIdBadParam, pri));

  /* Dynamic information of other CPU owned queue must be read
   * after cache clearing.
   */

	if (!isOwn()) {
		Dcache_clear_sync(this, sizeof(*this));
	}
	return m_que[pri].rest();
}

/*****************************************************************
 * Get the size of the transmitted message
 *****************************************************************/
/* Send message size(With parameter). */

template<typename T>
size_t MsgQueBlock::getSendSize(const T& /* param */, bool type_check)
{
	return type_check ?
			sizeof(MsgPacketHeader) + sizeof(TypeHolder<T>) :
			sizeof(MsgPacketHeader) + ROUND_UP(sizeof(T), sizeof(int));
}
/* Send message size(No parameter). */

template<>
inline size_t MsgQueBlock::getSendSize<MsgNullParam>(const MsgNullParam& /* param */, bool /* type_check */)
{
	return sizeof(MsgPacketHeader);
}

/* Send message size(Address Range Parameter). */

template<>
inline size_t MsgQueBlock::getSendSize<MsgRangedParam>(const MsgRangedParam& param, bool /* type_check */)
{
	return sizeof(MsgPacketHeader) + param.getParamSize();
}


/*****************************************************************
 * Class for acquiring message packet information
 *****************************************************************/
template<typename T>
struct MsgPacketInfo {
	static const bool typed_param = true;
	static const bool null_param = false;
};

template<>
struct MsgPacketInfo<MsgNullParam> {
	static const bool typed_param = false;
	static const bool null_param = true;
};

template<>
struct MsgPacketInfo<MsgRangedParam> {
	static const bool typed_param = false;
	static const bool null_param = false;
};

/*****************************************************************
 * Message sending process from task context
 *****************************************************************/
template<typename T>
err_t MsgQueBlock::send(MsgPri pri, MsgType type, MsgQueId reply, MsgFlags flags, const T& param)
{
  /* Check that the message fits in the element size of the queue */

  bool type_check = MSG_PARAM_TYPE_MATCH_CHECK && MsgPacketInfo<T>::typed_param && isOwn();
  size_t send_size = getSendSize(param, type_check);
  if (send_size > getElemSize(pri))
    {
      return ERR_DATA_SIZE;
    }

  /* Put the message packet header in the queue
   * and add the parameter after the interrupt is enabled.
   */

  lock(); /* In the shared queue,
           * the cache of the queue management area is also cleared.
           */

  MsgPacket* msg = pushHeader(pri, MsgPacketHeader(type, reply, flags));
  if (msg)
    {
      /* If it is a shared queue, cache flush of the packet header part.
       * (The synchronization process is performed by the unlock process)
       */

      if (isShare())
        {
          Dcache_flush_clear(msg, ROUND_UP(sizeof(MsgPacketHeader), CACHE_BLOCK_SIZE));
        }

      /* Since most ITRON APIs can not be executed in the interrupt
       *  disabled state, interrupts are permitted here.
       */

      unlock(); /* In the shared queue, the cache flush
                 * of the queue management area is also performed.
                 */

      /* Add parameter. (When there is no parameter, empty function) */

      msg->setParam(param, type_check); /* ITRON's API executable
                                         * with copy constructor.
                                         */

      DUMP_MSG_SEQ_LOCK(MsgSeqLog('s', m_id, pri, m_que[pri].size(), msg));

     /* When the parameter part is added to the shared queue,
      * the message packet region is cached flash.
      */

      
      if (!MsgPacketInfo<T>::null_param && isShare())
        {
          Dcache_flush_clear_sync(msg, ROUND_UP(send_size, CACHE_BLOCK_SIZE));
        }

      if (isShare() == false || isOwn())
        {
          /* Update total message count */

          Chateau_SignalSemaphoreTask(m_count_sem);
        } else {
          /* Request to update the total number of messages
           * by inter-CPU communication.
           */

          notifySend(m_owner, m_id);
        }
    }
  else
    {
      /* In the shared queue, the cache flush of the queue management area
       * is also performed.
       */

      unlock();
    }
  return (msg) ? ERR_OK : ERR_QUE_FULL;
}

/*****************************************************************
 * Message transmission processing from ISR
 * (Only to non-shared queue owned by own CPU)
 *****************************************************************/
template<typename T>
err_t MsgQueBlock::sendIsr(MsgPri pri, MsgType type, MsgQueId reply, const T& param)
{
  /* Transmission from the ISR to the shared queue is prohibited. */

  D_ASSERT2(isShare() == false, AssertParamLog(AssertIdBadMsgQueState, m_id));

  /* Check that the message fits in the element size of the queue. */

  bool type_check = MSG_PARAM_TYPE_MATCH_CHECK && MsgPacketInfo<T>::typed_param;
  if (getSendSize(param, type_check) > getElemSize(pri))
    {
      return ERR_DATA_SIZE;
    }

  /* Queue the message packet header. */

  MsgPacket* msg = pushHeader(pri, MsgPacketHeader(type, reply, MsgPacket::MsgFlagNull));
  if (msg)
    {
      /* Add parameter. (When there is no parameter, empty function) */

       /* ITRON API can not be executed with copy constructor. */

      msg->setParam(param, type_check);

      /* Update total message count. */

      Chateau_SignalSemaphoreIsr(m_count_sem);
      DUMP_MSG_SEQ(MsgSeqLog('i', m_id, pri, m_que[pri].size(), msg));
    }
  return (msg) ? ERR_OK : ERR_QUE_FULL;
}

/*****************************************************************
 * Insert a message packet header at the end of the queue
 * and return that address
 *****************************************************************/
inline MsgPacket* MsgQueBlock::pushHeader(MsgPri pri, const MsgPacketHeader& header)
{
	D_ASSERT2(pri == MsgPriNormal || pri == MsgPriHigh, AssertParamLog(AssertIdBadParam, pri));
  /* Check unused high priority queue specification. */

	D_ASSERT2(m_que[pri].is_init(), AssertParamLog(AssertIdBadMsgQueState, m_id, pri));
#ifdef USE_MULTI_CORE
	D_ASSERT2(isOwn() || (isShare() && InterCpuLock::SpinLockManager::isMember(m_spinlock)),
		AssertParamLog(AssertIdBadParam, m_id));
#else
	D_ASSERT2(isOwn(), AssertParamLog(AssertIdBadParam, m_id));
#endif

	MsgPacket* msg = m_que[pri].pushHeader(header);
	if (msg) {
		if (m_que[pri].size() > m_tally.max_queuing[pri]) {
			m_tally.max_queuing[pri] = m_que[pri].size();
      /* Leave peak value, message type etc in the log. */

			DUMP_MSG_PEAK(m_id, pri, MsgPeakLog(m_tally.max_queuing[pri], msg, m_que[pri].frontMsg()));
		}
	}
	return msg;
}

/*****************************************************************
  * Notify receipt of message(from other CPU)
 *****************************************************************/
inline void MsgQueBlock::notifyRecv() {
	Chateau_SignalSemaphoreIsr(m_count_sem);
}

/*****************************************************************
 * Receive message packet
 *****************************************************************/
inline err_t MsgQueBlock::recv(uint32_t ms, FAR MsgPacket **packet)
{
  bool result;

  /* Check if own CPU is owned, and
   * check that the previously received packet is discarded.
   */

  if (!(isOwn() && m_cur_que == NULL))
    {
      return ERR_QUE_FULL;
    }

retry:  /* Wait to receive. */

  if (ms != TIME_FOREVER)
    {
      timespec tm;
      tm.tv_sec = ms / 1000;
      tm.tv_nsec = ms % 1000;
      result = Chateau_TimedWaitSemaphore(m_count_sem, tm);
    }
  else
    {
      result = Chateau_WaitSemaphore(m_count_sem);
    }

  if (result == false)
    {
      return ERR_SEM_TAKE;
    }

  /* If it is a shared queue, clear the lock & queue control area cache. */

  if (isShare())
    {
      lock();
    }

  /* Get a pointer to the message packet of the queue
   * with the highest priority.
   */

  MsgPri pri = (m_que[MsgPriHigh].size()) ? MsgPriHigh : MsgPriNormal;
  MsgQue* que = &m_que[pri];
  MsgPacket* msg = que->frontMsg();
  
  if (msg == NULL)
    {
      return ERR_QUE_EMP;
    }

  /* If it waits for parameter writing,
   * it stores it and returns to waiting for message again.
   */

  if (msg->getFlags() & MsgPacket::MsgFlagWaitParam)
    {
      /* When recv is done with nonshared queue and multiple tasks,
       * inconsistency may occur with the following increment.
       * However, simultaneous recv to the same queue due to multiple tasks
       * is not supported because it is outside the support range
       * by specification.
       */

      /* Record the number of times the semaphore count was consumed. */

      ++m_pendingMsgCount;
      m_tally.max_pending = MAX(m_pendingMsgCount, m_tally.max_pending);
      ++m_tally.total_pending;

      if (isShare())
        {
#ifdef USE_MULTI_CORE
          /* In the case of a message from another CPU,
           * it is necessary to clear the area cache.
           */

          if (msg->getSrcCpu() != GET_CPU_ID())
            {
              Dcache_clear(msg, que->elem_size());
            }
#endif
          /* Cache flushing and unlocking queue management area. */

          unlock();
        }
      goto retry;
    }

  /* Update queue management area before cache flush. */

  uint16_t pending = m_pendingMsgCount;
  m_pendingMsgCount = 0;
  m_cur_que = que;

  /* If shared queue, cache queue management area cache flash & unlock. */
  
  if (isShare())
    {
      unlock();
    }

  /* Recover the semaphore count that was consumed while waiting
   * for parameter write.
   */

  while (pending--)
    {
      Chateau_SignalSemaphoreTask(m_count_sem);
    }

  DUMP_MSG_SEQ_LOCK(MsgSeqLog('r', m_id, pri, m_que[pri].size(), msg));

  *packet = msg;

  return ERR_OK;
}

/*****************************************************************
 * Discard message packet
 *****************************************************************/
inline err_t MsgQueBlock::pop()
{
  /* Check if own CPU is owned, and check Packet Received */

  if (!(isOwn() && m_cur_que != NULL))
    {
      return ERR_STS;
    }

  /* Check that the parameter length of the message packet
   * to be discarded is 0.
   */

  MsgPacket* msg = m_cur_que->frontMsg();

  if (msg->getParamSize() != 0)
    {
      return ERR_MEM_BUSY;
    }

  lock();

  /* Discard the packet from the queue. */

  if (m_cur_que->pop() == false)
    {
      return ERR_QUE_FREE;
    }

  /* In case of shared queue, clear cache of discarded packet area.
   * (The synchronization process is performed by the unlock process)
   * It is indispensable to prevent the value from remaining
   * at the next recv and to prevent write back of the dirty cache.
   */

#if MSG_FILL_VALUE_AFTER_POP == 0x00
  if (isShare())
    {
      Dcache_clear(msg, m_cur_que->elem_size());
    }
#else
  if (isShare())
    {
      Dcache_flush_clear(msg, m_cur_que->elem_size());
    } /* flush is the fill value write after pop. */
#endif
  m_cur_que = NULL; /* Make the packet unreceived state. */
  unlock();

  return ERR_OK;
}

/*****************************************************************
 * Lock queue
 *****************************************************************/
inline void MsgQueBlock::lock() {
	if (isShare() == false) { /* Check local (nonshared) queue. */
		Chateau_LockInterrupt(&m_context);
	} else {
#ifdef USE_MULTI_CORE
		InterCpuLock::SpinLockManager::acquire(m_spinlock);
    /* Clear cache of queue management area (self instance). */

		Dcache_clear_sync(this, sizeof(*this));
#else
		F_ASSERT(0);
#endif
	}
}

/*****************************************************************
 * Unlock queue
 *****************************************************************/
inline void MsgQueBlock::unlock() {
	if (isShare() == false) { /* Check local (nonshared) queue. */
		Chateau_UnlockInterrupt(&m_context);
	} else {
#ifdef USE_MULTI_CORE
    /* Cache flash & clear of queue management area (self instance). */

		Dcache_flush_clear_sync(this, sizeof(*this));
		InterCpuLock::SpinLockManager::release(m_spinlock);
#else
		F_ASSERT(0);
#endif
	}
}

/*****************************************************************
 * Dump display of message cube lock
 * Because execution of this function causes the message packet
 * to appear in the cache, caution is required because there
 * is a possibility that message reception from other CPU
 * may be affected.
 *****************************************************************/
inline void MsgQueBlock::dump() const
{
	printf("ID:%d, init=%d, owner=%d, spinlock=%d, count_sem=%d, cur_pending=%d, cur_que=%08x\n",
		m_id, m_initDone, m_owner, m_spinlock, m_count_sem, m_pendingMsgCount, m_cur_que);
	m_tally.dump();

	printf("Normal priority queue=%08x\n", &m_que[MsgPriNormal]);
	m_que[MsgPriNormal].dump();

	if (m_que[MsgPriHigh].capacity()) {
		printf("High priority queue=%08x\n", &m_que[MsgPriHigh]);
		m_que[MsgPriHigh].dump();
	}
}

/**
 * @}
 */

/**
 * @}
 */

#endif /* MSG_QUE_BLOCK_H_INCLUDED */
