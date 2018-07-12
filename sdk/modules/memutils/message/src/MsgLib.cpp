/****************************************************************************
 * modules/memutils/message/src/MsgLib.cpp
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

#include "memutils/message/Message.h"

uint32_t	MsgLib::num_msg_pools;
uint32_t	MsgLib::msgq_top_drm = 0;

/*****************************************************************
 * メッセージキューブロックの0番は未使用なので、ヘッダとして使用する
 *****************************************************************/
struct MsgQueBlock_0 {
	char		lib_name[6];
	char		version[4];
	uint16_t	num_msgq_pools;
	uint32_t	start_drm;
	char		resv[46];
public:
	MsgQueBlock_0(const char* name, const char* ver, uint16_t num_msgq, uint32_t start) :
		num_msgq_pools(num_msgq),
		start_drm(start) {
		memcpy(lib_name, name, sizeof(lib_name));
		memcpy(version, ver, sizeof(version));
		memset(resv, 0x00, sizeof(resv));
	}
}; /* MsgQueBlock_0 */

//S_ASSERT(sizeof(MsgQueBlock_0) == CACHE_BLOCK_SIZE);

/*
 * 本来は型を定義した箇所で、S_ASSERTによるサイズチェックを行いたいが
 * 現在のS_ASSERTの実装では、ヘッダファイルに記述すると2重定義が発生して
 * エラーになる事があるためソースファイルでチェックを行う
 */

/* dmp_layout.confのDMP_MSG_SEQ2エントリがtrueの時に有効 */
#if defined(DMP_MSG_SEQ2_NUM) && DMP_MSG_SEQ2_NUM != 0
	S_ASSERT(sizeof(MsgLog) <= DMP_MSG_SEQ2_BYTES - DMP_MSG_SEQ2_TIME);
#endif

/* dmp_layout.confのDMP_MSG_PEAKエントリがtrueの時に有効 */
#if defined(DMP_MSG_PEAK_NUM) && DMP_MSG_PEAK_NUM != 0
	S_ASSERT(sizeof(MsgPeakLog) <= DMP_MSG_PEAK_BYTES - DMP_MSG_PEAK_TIME);
	/* ログ領域のエントリ数をチェック */
	S_ASSERT((num_msg_pools - 1) * NumMsgPri <= DMP_MSG_PEAK_NUM);
#endif

#ifdef USE_MULTI_CORE
/* multiにする場合は、SYS_GetCpuIdの処理の検討が必要 */
extern "C" {
int SYS_GetCpuId(void)
{
	return 2;
}
} /* extern "C" */
#endif

extern const MsgQueDef MsgqPoolDefs[];

/*****************************************************************
 * 全体の初期化(単一のCPUで1回だけ実行すること)
 *****************************************************************/
err_t MsgLib::initFirst(uint32_t num_pools, uint32_t top_drm)
{
  err_t    err_code = ERR_OK;
  uint32_t context  = 0;

  num_msg_pools = num_pools;
  msgq_top_drm  = top_drm;

  Chateau_LockInterrupt(&context);
  if(isInitFirstComplete() == false)
    {
      const FAR MsgQueDef* src = MsgqPoolDefs;

      /* コンストラクタ実行前なので、uint8_t*型とする */

      uint8_t* dst = static_cast<uint8_t*>(DRM_TO_CACHED_VA(msgq_top_drm));

      /* メッセージキューブロックの0番は未使用なので、ヘッダとして使用 */

      new(dst) MsgQueBlock_0(MSG_LIB_NAME, MSG_LIB_VER, num_msg_pools, msgq_top_drm);

      /* キュー管理領域の静的な初期化 */

      for (uint32_t i = 1; i < num_msg_pools; ++i)
        {
          new(dst + i * sizeof(MsgQueBlock)) MsgQueBlock(i, src[i].owner, src[i].spinlock);
        }

      /* initPerCpu()でキャッシュクリアされるので、ここではフラッシュのみ */

      Dcache_flush_sync(dst, sizeof(MsgQueBlock) * num_msg_pools);
    }
  else
    {
      /* execute more than once. return status error */

      err_code = ERR_STS;
    }

  Chateau_UnlockInterrupt(&context);

  return err_code;
}

/*****************************************************************
 * CPU毎の初期化
 *****************************************************************/
err_t MsgLib::initPerCpu()
{
  err_t    err_code  = ERR_OK;
  uint32_t context   = 0;
  bool     init_done = false;

  Chateau_LockInterrupt(&context);
  const FAR MsgQueDef* src = MsgqPoolDefs;
  MsgQueBlock* mqb = static_cast<FAR MsgQueBlock*>(DRM_TO_CACHED_VA(msgq_top_drm));
  Dcache_clear_sync(mqb, sizeof(MsgQueBlock) * num_msg_pools);

  /* キュー管理領域の動的な初期化 */

  for (uint32_t id = 1; id < num_msg_pools; ++id)
    {
      /* initFirstの実行チェック */

      if (mqb[id].m_id != id)
        {
          err_code = ERR_STS;
          break;
        }

      err_code = isInitComplete(id, init_done);
      if (err_code == ERR_OK)
        {
          if (init_done == false && mqb[id].isOwn())
            {
              /* 自CPU所有キューの初期化(アドレス設定、セマフォ生成、キャッシュ操作など) */

              err_code = mqb[id].setup(src[id].n_drm, src[id].n_size, src[id].n_num, src[id].h_drm, src[id].h_size, src[id].h_num);
              if (err_code != ERR_OK)
                {
                  break;
                }
            }
        }
      else
        {
          break;
        }
    }
  cache_sync();
  Chateau_UnlockInterrupt(&context);

  return err_code;
}

/*****************************************************************
 * Finalize Process
 *****************************************************************/
err_t MsgLib::finalize()
{
  uint32_t context = 0;

  if (msgq_top_drm == 0)
    {
      return ERR_STS;
    }

  Chateau_LockInterrupt(&context);

  FAR MsgQueBlock_0* dst = static_cast<FAR MsgQueBlock_0*>(DRM_TO_CACHED_VA(msgq_top_drm));
  memset(dst->lib_name, 0x00, sizeof(dst->lib_name));
  msgq_top_drm = 0;

  Chateau_UnlockInterrupt(&context);

  return ERR_OK;
}

/*****************************************************************
 * InitFirst 済みかどうかのチェック
 *****************************************************************/
bool MsgLib::isInitFirstComplete()
{
  FAR uint8_t* dst  = static_cast<FAR uint8_t*>(DRM_TO_CACHED_VA(msgq_top_drm));
  uint8_t libname[] = MSG_LIB_NAME;
  FAR uint8_t* src  = &libname[0];

  for (uint8_t i = 0; i < sizeof(libname)/sizeof(libname[0]) - 1; ++i)
    {
      if(*dst != *src)
        {
          return false;
        }
      ++dst;
      ++src;
    }
  return true;
}

/*****************************************************************
 * メッセージキューの初期化状態の取得
 *****************************************************************/
err_t MsgLib::isInitComplete(MsgQueId id, bool &done)
{
  if (id == MSG_QUE_NULL || id >= num_msg_pools)
    {
      return ERR_ARG;
    }

  FAR MsgQueBlock* mqb = static_cast<FAR MsgQueBlock*>(DRM_TO_CACHED_VA(msgq_top_drm));

  /* initFirstの実行チェック */

  if (mqb[id].m_id != id)
    {
      return ERR_STS;
    }

  /* 他CPU所有キューの動的情報は、キャッシュクリアしてから読込むこと */

  if (mqb[id].m_initDone == false && !mqb[id].isOwn())
    {
      Dcache_clear_sync(&mqb[id], sizeof(MsgQueBlock));
    }

  done = mqb[id].m_initDone;

  return ERR_OK;
}

/*****************************************************************
 * 指定されたメッセージキューブロックへの参照を取得
 *****************************************************************/
err_t MsgLib::referMsgQueBlock(MsgQueId id, FAR MsgQueBlock **que)
{
  err_t err_code  = ERR_OK;
  bool  init_done = false;

  if (que == NULL)
  {
    return ERR_ARG;
  }

  err_code = isInitComplete(id, init_done);
  if (err_code == ERR_OK)
    {
      if (!init_done)
        {
          return ERR_STS;
        }
    }
  else
    {
      return err_code;
    }

  FAR MsgQueBlock* mqb = static_cast<FAR MsgQueBlock*>(DRM_TO_CACHED_VA(msgq_top_drm));
  *que = &mqb[id];

  return ERR_OK;
}

/*****************************************************************
 * Transmission of message packet.
 *****************************************************************
 */
err_t MsgLib::send(MsgQueId dest, MsgPri pri, MsgType type, MsgQueId reply)
{
  FAR MsgQueBlock* que;
  err_t            err_code = ERR_OK;

  err_code = referMsgQueBlock(dest, &que);
  if (err_code == ERR_OK)
    {
      return que->send(pri, type, reply, MsgPacket::MsgFlagNull, MsgNullParam());
    }

  return err_code;
}

err_t MsgLib::send(MsgQueId dest, MsgPri pri, MsgType type, MsgQueId reply, const void* param, size_t param_size)
{
  FAR MsgQueBlock* que;
  err_t            err_code = ERR_OK;

  err_code = referMsgQueBlock(dest, &que);
  if (err_code == ERR_OK)
    {
      return que->send(pri, type, reply, MsgPacket::MsgFlagWaitParam, MsgRangedParam(param, param_size));
    }

  return err_code;
}

err_t MsgLib::sendIsr(MsgQueId dest, MsgPri pri, MsgType type, MsgQueId reply)
{
  FAR MsgQueBlock* que;
  err_t            err_code = ERR_OK;

  err_code = referMsgQueBlock(dest, &que);
  if (err_code == ERR_OK)
    {
      return que->sendIsr(pri, type, reply, MsgNullParam());
    }
  return err_code;
}

err_t MsgLib::sendIsr(MsgQueId dest, MsgPri pri, MsgType type, MsgQueId reply, const void* param, size_t param_size)
{
  FAR MsgQueBlock* que;
  err_t            err_code = ERR_OK;

  err_code = referMsgQueBlock(dest, &que);
  if (err_code == ERR_OK)
    {
      return que->sendIsr(pri, type, reply, MsgRangedParam(param, param_size));
    }

  return err_code;
}

/*****************************************************************
 * Notify message reception.
 *****************************************************************
 */
err_t MsgLib::notifyRecv(MsgQueId dest)
{
  FAR MsgQueBlock* que;
  err_t            err_code = ERR_OK;

  err_code = referMsgQueBlock(dest, &que);
  if (err_code == ERR_OK)
    {
      que->notifyRecv();
    }

  return err_code;
}

/*****************************************************************
 * 全てのメッセージキューブロックのダンプ表示
 * 本関数の実行により、メッセージパケットがキャッシュに載るため
 * 他CPUからのメッセージ受信に影響が出る可能性があるため注意が必要
 *****************************************************************/
void MsgLib::dump()
{
	MsgQueBlock* mqb = static_cast<MsgQueBlock*>(DRM_TO_CACHED_VA(msgq_top_drm));
	for (uint32_t id = 1; id < num_msg_pools; ++id) {
		mqb[id].dump();
	}
}

/* end of MsgLib.cxx */
