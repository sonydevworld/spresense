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
 * メッセージキューブロッククラス
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

  /* メッセージパケットの破棄 */
  err_t pop();

	/* キュー所有者(受信者)のCPU-IDの取得 */
	MsgCpuId getOwner() const { return m_owner; }

	/* 共有キューか否かを取得 */
	bool isShare() const { return m_spinlock != 0; }

	/* メッセージパケット数の取得 */
	uint16_t getNumMsg(MsgPri pri) const;

	/* 格納可能なメッセージパケット数の取得(未使用のキューは、0を返す) */
	uint16_t getRest(MsgPri pri) const;

	/* デバッグ専用 */
	void dump() const;
	void dumpQue(MsgPri pri) const { m_que[pri].dump(); }

protected:
	friend class MsgLib;

	/* 静的な初期化のみ */
	MsgQueBlock(MsgQueId id, MsgCpuId owner, SpinLockId spinlock);

  /* 動的な初期化 */
  err_t setup(drm_t n_drm, uint16_t n_size, uint16_t n_num,
    drm_t h_drm, uint16_t h_size, uint16_t h_num);

	/* キューの要素サイズの取得 */
	uint16_t getElemSize(MsgPri pri) const { return m_que[pri].elem_size(); }

	/* 自CPUの所有キューか調べる */
#ifdef USE_MULTI_CORE
	bool isOwn() const { return GET_CPU_ID() == getOwner(); }
#else
	/* multiでなければ、常にtrueを返す */
	bool isOwn() const { return true; }
#endif
	/* 送信メッセージサイズを求める */
	template<typename T>
	static size_t getSendSize(const T& param, bool type_check);

	/* タスクコンテキストからのメッセージ送信処理 */
	template<typename T>
	err_t send(MsgPri pri, MsgType type, MsgQueId reply, MsgFlags flags, const T& param);

	/* ISRからのメッセージ送信処理(自CPU所有の非共有キュー宛てのみ) */
	template<typename T>
	err_t sendIsr(MsgPri pri, MsgType type, MsgQueId reply, const T& param);

	/* メッセージを送信したことを他CPUへ通知 (H/W依存部。CPU毎にユーザーが実装する) */
	void notifySend(MsgCpuId cpu, MsgQueId dest);

	/* (他CPUからの)メッセージ受信を通知 */
	void notifyRecv();

	/* キュー末尾にメッセージパケットヘッダを挿入して、そのアドレスを返す */
	MsgPacket* pushHeader(MsgPri pri, const MsgPacketHeader& header);

	/* キューをロック/アンロックする */
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
	const MsgQueId		m_id;			/* メッセージキューID */
	bool			m_initDone;		/* 初期化済みフラグ */
	const MsgCpuId		m_owner;		/* キュー所有者(受信者)のCPU-ID */
	Chateau_sem_handle_t	m_count_sem;		/* 総メッセージ数を示す計数セマフォ */
	const SpinLockId	m_spinlock;		/* スピンロックID。0ならばCPU間の共有なし */
	uint16_t		m_pendingMsgCount;	/* パラメタ書込み待ちメッセージ数 */
	MsgQue			m_que[NumMsgPri];	/* 優先度毎のキュー */
	MsgQue*			m_cur_que;		/* メッセージ処理中のキュー */
	Tally			m_tally;		/* 各種計測値 */
	uint32_t		m_context;		/* ロック用の変数 */
}; /* class MsgQueBlock */

/*****************************************************************
 * Constructor (静的な初期化のみ)
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
 * 動的な初期化
 *****************************************************************/
inline err_t MsgQueBlock::setup(drm_t n_drm, uint16_t n_size, uint16_t n_num,
  drm_t h_drm, uint16_t h_size, uint16_t h_num)
{
  /* まだ初期化されていないこと */
  if (m_initDone != false)
    {
      return ERR_STS;
    }

  /* キューのアドレス、要素長、要素数を設定 */
  m_que[MsgPriNormal].init(n_drm, n_size, n_num);
  if (h_drm != INVALID_DRM)
    {
      m_que[MsgPriHigh].init(h_drm, h_size, h_num);
    }

  /* メッセージ領域をキャッシュクリア */
  if (isShare())
    {
      Dcache_clear(DRM_TO_CACHED_VA(n_drm), n_size * n_num);
      if (h_drm != INVALID_DRM)
        {
          Dcache_clear(DRM_TO_CACHED_VA(h_drm), h_size * h_num);
        }
    }

  /* セマフォを初期値0で生成 */
  Chateau_CreateSemaphore(&m_count_sem, 0, 0);

  m_initDone = true;  /* 初期化終了 */
  Dcache_flush(this, sizeof(*this));  /* syncは呼び出し元でまとめて行う */

  return ERR_OK;
}

/*****************************************************************
 * メッセージパケット数の取得
 *****************************************************************/
inline uint16_t MsgQueBlock::getNumMsg(MsgPri pri) const
{
	D_ASSERT2(pri == MsgPriNormal || pri == MsgPriHigh, AssertParamLog(AssertIdBadParam, pri));

	/* 他CPU所有キューの動的情報は、キャッシュクリアしてから読込むこと */
	if (!isOwn()) {
		Dcache_clear_sync(this, sizeof(*this));
	}
	return m_que[pri].size();
}

/*****************************************************************
 * 格納可能なメッセージパケット数の取得(未使用のキューは、0を返す)
 *****************************************************************/
inline uint16_t MsgQueBlock::getRest(MsgPri pri) const
{
	D_ASSERT2(pri == MsgPriNormal || pri == MsgPriHigh, AssertParamLog(AssertIdBadParam, pri));

	/* 他CPU所有キューの動的情報は、キャッシュクリアしてから読込むこと */
	if (!isOwn()) {
		Dcache_clear_sync(this, sizeof(*this));
	}
	return m_que[pri].rest();
}

/*****************************************************************
 * 送信メッセージサイズを求める
 *****************************************************************/
/* 送信メッセージサイズ(パラメータあり) */
template<typename T>
size_t MsgQueBlock::getSendSize(const T& /* param */, bool type_check)
{
	return type_check ?
			sizeof(MsgPacketHeader) + sizeof(TypeHolder<T>) :
			sizeof(MsgPacketHeader) + ROUND_UP(sizeof(T), sizeof(int));
}
/* 送信メッセージサイズ(パラメータなし) */
template<>
inline size_t MsgQueBlock::getSendSize<MsgNullParam>(const MsgNullParam& /* param */, bool /* type_check */)
{
	return sizeof(MsgPacketHeader);
}

/* 送信メッセージサイズ(アドレス範囲パラメタ) */
template<>
inline size_t MsgQueBlock::getSendSize<MsgRangedParam>(const MsgRangedParam& param, bool /* type_check */)
{
	return sizeof(MsgPacketHeader) + param.getParamSize();
}


/*****************************************************************
 * メッセージパケット情報取得用クラス
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
 * タスクコンテキストからのメッセージ送信処理
 *****************************************************************/
template<typename T>
err_t MsgQueBlock::send(MsgPri pri, MsgType type, MsgQueId reply, MsgFlags flags, const T& param)
{
  /* メッセージがキューの要素サイズに収まることを確認する */

  bool type_check = MSG_PARAM_TYPE_MATCH_CHECK && MsgPacketInfo<T>::typed_param && isOwn();
  size_t send_size = getSendSize(param, type_check);
  if (send_size > getElemSize(pri))
    {
      return ERR_DATA_SIZE;
    }

  /* メッセージパケットヘッダをキューに入れ、割込み許可後にパラメタを追記する */

  lock(); /* 共有キューでは、キュー管理領域のキャッシュクリアも行う */
  MsgPacket* msg = pushHeader(pri, MsgPacketHeader(type, reply, flags));
  if (msg)
    {
      /* 共有キューならば、パケットヘッダ部のキャッシュフラッシュ(syncは、下のunlockに任せる) */

      if (isShare())
        {
          Dcache_flush_clear(msg, ROUND_UP(sizeof(MsgPacketHeader), CACHE_BLOCK_SIZE));
        }

      /* ほとんどのITRON APIは割込み禁止状態では実行できないため、ここで割込みを許可 */

      unlock(); /* 共有キューでは、キュー管理領域のキャッシュフラッシュも行う */

      /* パラメタを追記 (パラメタなし時は、空関数) */

      msg->setParam(param, type_check); /* copy constructorで、ITRONのAPI実行可能 */
      DUMP_MSG_SEQ_LOCK(MsgSeqLog('s', m_id, pri, m_que[pri].size(), msg));

     /* パラメタ部を共有キューに追記した場合は、メッセージパケット領域をキャッシュフラッシュ */
      
      if (!MsgPacketInfo<T>::null_param && isShare())
        {
          Dcache_flush_clear_sync(msg, ROUND_UP(send_size, CACHE_BLOCK_SIZE));
        }

      if (isShare() == false || isOwn())
        {
          Chateau_SignalSemaphoreTask(m_count_sem); /* 総メッセージ数を更新 */
        } else {
          notifySend(m_owner, m_id);  /* CPU間通信で、総メッセージ数の更新を依頼 */
        }
    }
  else
    {
      unlock(); /* 共有キューでは、キュー管理領域のキャッシュフラッシュも行う */
    }
  return (msg) ? ERR_OK : ERR_QUE_FULL;
}

/*****************************************************************
 * ISRからのメッセージ送信処理(自CPU所有の非共有キュー宛てのみ)
 *****************************************************************/
template<typename T>
err_t MsgQueBlock::sendIsr(MsgPri pri, MsgType type, MsgQueId reply, const T& param)
{
  /* ISRから共有キューへ送信は禁止する */

  D_ASSERT2(isShare() == false, AssertParamLog(AssertIdBadMsgQueState, m_id));

  /* メッセージがキューの要素サイズに収まることを確認する */

  bool type_check = MSG_PARAM_TYPE_MATCH_CHECK && MsgPacketInfo<T>::typed_param;
  if (getSendSize(param, type_check) > getElemSize(pri))
    {
      return ERR_DATA_SIZE;
    }

  /* メッセージパケットヘッダをキューに入れる */

  MsgPacket* msg = pushHeader(pri, MsgPacketHeader(type, reply, MsgPacket::MsgFlagNull));
  if (msg)
    {
      /* パラメタを追記 (パラメタなし時は、空関数) */

      msg->setParam(param, type_check); /* copy constructorで、ITRONのAPIは実行不可 */
      Chateau_SignalSemaphoreIsr(m_count_sem);  /* 総メッセージ数を更新 */
      DUMP_MSG_SEQ(MsgSeqLog('i', m_id, pri, m_que[pri].size(), msg));
    }
  return (msg) ? ERR_OK : ERR_QUE_FULL;
}

/*****************************************************************
 * キュー末尾にメッセージパケットヘッダを挿入して、そのアドレスを返す
 *****************************************************************/
inline MsgPacket* MsgQueBlock::pushHeader(MsgPri pri, const MsgPacketHeader& header)
{
	D_ASSERT2(pri == MsgPriNormal || pri == MsgPriHigh, AssertParamLog(AssertIdBadParam, pri));
	/* (高優先度)未使用キュー指定のチェック */
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
			/* ピーク値、メッセージタイプ等をログに残す */
			DUMP_MSG_PEAK(m_id, pri, MsgPeakLog(m_tally.max_queuing[pri], msg, m_que[pri].frontMsg()));
		}
	}
	return msg;
}

/*****************************************************************
 * (他CPUからの)メッセージ受信を通知
 *****************************************************************/
inline void MsgQueBlock::notifyRecv() {
	Chateau_SignalSemaphoreIsr(m_count_sem);
}

/*****************************************************************
 * メッセージパケットの受信
 *****************************************************************/
inline err_t MsgQueBlock::recv(uint32_t ms, FAR MsgPacket **packet)
{
  bool result;

  /* 自CPU所有, 以前受信したパケットが破棄されていることのチェック */

  if (!(isOwn() && m_cur_que == NULL))
    {
      return ERR_QUE_FULL;
    }

retry:  /* 受信待ち */
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

  /* 共有キューならば、ロック & キュー管理領域をキャッシュクリア */

  if (isShare())
    {
      lock();
    }

  /* 優先度の最も高いキューのメッセージパケットへのポインタを取得 */

  MsgPri pri = (m_que[MsgPriHigh].size()) ? MsgPriHigh : MsgPriNormal;
  MsgQue* que = &m_que[pri];
  MsgPacket* msg = que->frontMsg();
  
  if (msg == NULL)
    {
      return ERR_QUE_EMP;
    }

  /*
   * send()のsetParam()とDcache_flush_sync()の間でキャッシュクリアが発生すると
   * パラメタ部を消失してしまうため、以下のクリアはpop()へ移動した
   *  if (isShare()) { Dcache_clear_sync(msg, que->elem_size()); }
   */

  /* パラメタ書込み待ちならば、それを記憶して、再度メッセージ待ちに戻る */

  if (msg->getFlags() & MsgPacket::MsgFlagWaitParam)
    {
      /*
      * 非共有キューかつ複数のタスクでrecvを行うと、以下のインクリメントで不整合が
      * 発生する可能性がある。しかし複数タスクによる同一キューへの同時recvは仕様上
      * サポート範囲外なのでケアしない
      */

      ++m_pendingMsgCount;  /* セマフォカウントを消費してしまった回数を記録する */
      m_tally.max_pending = MAX(m_pendingMsgCount, m_tally.max_pending);
      ++m_tally.total_pending;

      if (isShare())
        {
#ifdef USE_MULTI_CORE
          /* 他CPUからのメッセージの場合は、領域のキャッシュクリアが必要 */

          if (msg->getSrcCpu() != GET_CPU_ID())
            {
              Dcache_clear(msg, que->elem_size());
            }
#endif
          unlock();	/* キュー管理領域をキャッシュフラッシュ & アンロック */
        }
      goto retry;
    }

  /* キュー管理領域をキャッシュフラッシュ前に更新 */

  uint16_t pending = m_pendingMsgCount;
  m_pendingMsgCount = 0;
  m_cur_que = que;

  /* 共有キューならば、キュー管理領域をキャッシュフラッシュ & アンロック */
  
  if (isShare())
    {
      unlock();
    }

  /* パラメタ書込み待ちの間に消費してしまったセマフォカウントを復旧する */

  while (pending--)
    {
      Chateau_SignalSemaphoreTask(m_count_sem);
    }

  DUMP_MSG_SEQ_LOCK(MsgSeqLog('r', m_id, pri, m_que[pri].size(), msg));

  *packet = msg;

  return ERR_OK;
}

/*****************************************************************
 * メッセージパケットの破棄 
 *****************************************************************/
inline err_t MsgQueBlock::pop()
{
  /* 自CPU所有, パケット受信済みのチェック */

  if (!(isOwn() && m_cur_que != NULL))
    {
      return ERR_STS;
    }

  /* 破棄するメッセージパケットのパラメタ長は、0であること */

  MsgPacket* msg = m_cur_que->frontMsg();

  if (msg->getParamSize() != 0)
    {
      return ERR_MEM_BUSY;
    }

  lock();

  /* キューからパケットを破棄する */

  if (m_cur_que->pop() == false)
    {
      return ERR_QUE_FREE;
    }

  /*
   * 共有キューの場合は、破棄したパケット領域のキャッシュをクリア (syncは、下のunlockに任せる)
   * 次回recv時に値が残っていることを防ぐためとdirtyキャッシュの書き戻しを防ぐために必須である
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
    } /* flushは、pop後のfill値書込み */
#endif
  m_cur_que = NULL; /* パケット未受信状態にする */
  unlock();

  return ERR_OK;
}

/*****************************************************************
 * キューをロックする
 *****************************************************************/
inline void MsgQueBlock::lock() {
	if (isShare() == false) {	/* ローカル(非共有)キュー? */
		Chateau_LockInterrupt(&m_context);
	} else {
#ifdef USE_MULTI_CORE
		InterCpuLock::SpinLockManager::acquire(m_spinlock);
		/* キュー管理領域(自インスタンス)のキャッシュクリア */
		Dcache_clear_sync(this, sizeof(*this));
#else
		F_ASSERT(0);
#endif
	}
}

/*****************************************************************
 * キューをアンロックする
 *****************************************************************/
inline void MsgQueBlock::unlock() {
	if (isShare() == false) {	/* ローカル(非共有)キュー? */
		Chateau_UnlockInterrupt(&m_context);
	} else {
#ifdef USE_MULTI_CORE
		/* キュー管理領域(自インスタンス)のキャッシュフラッシュ&クリア */
		Dcache_flush_clear_sync(this, sizeof(*this));
		InterCpuLock::SpinLockManager::release(m_spinlock);
#else
		F_ASSERT(0);
#endif
	}
}

/*****************************************************************
 * メッセージキューブロックのダンプ表示
 * 本関数の実行により、メッセージパケットがキャッシュに載るため
 * 他CPUからのメッセージ受信に影響が出る可能性があるため注意が必要
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
